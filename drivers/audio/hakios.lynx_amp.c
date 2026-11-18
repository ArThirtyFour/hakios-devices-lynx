/*
 * The Pixel 7a's speaker amplifiers: two Cirrus Logic CS35L41s on the SPI controller at 0x10960000 (USI7 of PERIC0),
 * the left one on the controller's own chip select (gpp14-3), the right one ("R", cirrus,right-channel-amp) on a GPIO
 * one (gpp22-1). The AoC drives their audio over its TDM_0 port (DSP-A, 4 slots of 32 bits, 16-bit stereo at 48 kHz: a
 * 6.144 MHz bit clock); this driver is their control side. Only with boot-arg amp=N.
 *
 *   amp=1  a look: the SPI clock and controller, both chips reset (gpp17-1, gpp25-2), their OTP boot, device and revision ids
 *   amp=2  and set up for the AoC's TDM as Linux's cs35l41 does without its DSP (no firmware: the protection algorithm is
 *          not run, so the level is kept low): test key, rev B2 errata, the OTP's trims, GPIO2 the interrupt line, the boost
 *          (1 uH, 4 uF, 3.5 A peak: the device tree), DSP-A as a clock consumer, 48 kHz, 32-bit slots of 16-bit data, the
 *          left on slot 0 and the right on slot 1, the PLL from the 6.144 MHz bit clock, ASP RX1 straight to the DAC, class
 *          H, the analog gain at its least; lynx_amp_speaker(1) (the AoC's driver, once its TDM runs) powers both up --
 *          Linux's "Main AMP" widget: the power-up patch, GLOBAL_EN, PUP_DONE -- and lynx_amp_speaker(0) down
 *   speaker=1  needed for any of it to be heard: without it lynx_amp_speaker(1) refuses and the amplifiers stay powered down
 *
 * The SPI controller is the touch screen's (drivers/input/hakios.lynx_touch.c: Samsung's s3c64xx, polled, 8-bit), on
 * PERIC0 USI7 instead of PERIC1 USI0. The chips' SPI format (Linux's cs35l41_regmap_spi): a 32-bit big-endian register
 * address (bit 31 set for a read), 16 bits of padding, the 32-bit big-endian value.
 *
 * The clock and pins (like the touch screen's) are the loader's to set (lynxld/touch_up.c: ACPM for the rails, CMU
 * gates, the USI in SPI mode) -- this driver brings the clock up itself as the touch screen's loader does, since the
 * amps are not on the touch's bus and need their own PERIC0 USI7 gate (CMU_PERIC0 0x10800000: mux 0x6b0, div 0x1828,
 * gates 0x2098/0x209c/0x20e0). A probe (probes/tspi.c, pointed at this bus) is the safer first step on the phone.
 *
 * Sources: Linux sound/soc/codecs/cs35l41*.c and include/sound/cs35l41.h, Google's aoc/alsa/aoc_alsa_card.c (the TDM
 * format), the phone's device tree (spi@10960000, cs35l41@0/@1, their pins), and the Pixel 8a's hakios.akita_amp.c,
 * whose structure this follows (amp=3/4 -- the speaker protection firmware -- is not ported here yet: it needs the
 * cs35l41-dsp1-spk-prot.wmfw/.bin files from the phone, as the akita driver loads them).
 */
#include <stdint.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

/* ---- the addresses ------------------------------------------------------------------------------------------------ */

#define SPI_PHYS                0x10960000ULL   /* spi_7: spi@10960000, USI7 of PERIC0 */
#define CMU_PHYS                0x10800000ULL   /* CMU_PERIC0 */
#define CMU_MUX_USI7            0x6b0           /* PLL_CON0_MUX_CLKCMU_PERIC0_USI7_USI_USER: bit 4, 0 = the crystal */
#define CMU_DIV_USI7            0x1828          /* CLK_CON_DIV_DIV_CLK_PERIC0_USI7_USI: bits 3:0 */
#define CMU_GATE_IPCLK_USI7     0x2098          /* CLK_BLK_PERIC0_UID_USI7_USI_IPCLKPORT_IPCLK */
#define CMU_GATE_PCLK_USI7      0x209c          /* ..._PCLK */
#define CMU_GATE_RSTNSYNC_USI7  0x20e0          /* ..._RSTNSYNC ..._IPCLKPORT_CLK */
#define CMU_GATE_ON             0x00300000      /* MANUAL (bit 20) and CG_VAL (bit 21): held on */
#define SYSREG_PHYS             0x10821000ULL   /* sysreg_peric0 */
#define SYSREG_USI7_MODE        0x018           /* samsung,usi-offset = <0x18> */
#define USI_MODE_SPI            0x2

/* the pins: an Exynos bank is CON (4 bits a pin) at +0, DAT at +4; the offsets are pinctrl-gs201.c's bank table */
#define RST_L_PHYS              0x10840000ULL   /* gpp17 of pinctrl@10840000 (PERIC0) at +0x220: pin 1 */
#define RST_L_BANK              0x220
#define RST_L_PIN               1
#define RST_R_PHYS              0x10C40000ULL   /* gpp25 of pinctrl@10C40000 (PERIC1) at +0xa0: pin 2 */
#define RST_R_BANK              0xa0
#define RST_R_PIN               2
#define CS_R_PHYS               0x10C40000ULL   /* gpp22 of pinctrl@10C40000 (PERIC1) at +0x40: pin 1 (active low) */
#define CS_R_BANK               0x40
#define CS_R_PIN                1

/* s3c64xx SPI */
#define CH_CFG                  0x00
#define MODE_CFG                0x08
#define CS_REG                  0x0c
#define INT_EN                  0x10
#define STATUS                  0x14
#define TX_DATA                 0x18
#define RX_DATA                 0x1c
#define PACKET_CNT              0x20
#define PENDING_CLR             0x24
#define SWAP_CFG                0x28
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define CH_TXCH_ON              (1u << 0)
#define CH_RXCH_ON              (1u << 1)
#define CH_SW_RST               (1u << 5)
#define CS_INACTIVE             (1u << 0)
#define PACKET_CNT_EN           (1u << 16)
#define USI_CON_RESET           (1u << 0)
#define USI_OPTION_CLKREQ_ON    (1u << 1)
#define USI_OPTION_CLKSTOP_ON   (1u << 2)
#define RX_LVL(s)               (((s) >> 15) & 0x7f)
#define FIFO_INFLIGHT           32

/* CS35L41 (include/sound/cs35l41.h) */
#define CS35L41_DEVID           0x00000000
#define CS35L41_REVID           0x00000004
#define CS35L41_OTPID           0x00000010
#define CS35L41_IRQ1_STATUS3    0x00010018
#define CS35L41_IRQ1_STATUS4    0x0001001c
#define CS35L41_OTP_BOOT_DONE   0x02
#define CS35L41_OTP_BOOT_ERR    0x80000000
#define CS35L41_CHIP_ID         0x35a40
#define CS35L41R_CHIP_ID        0x35b40

/* include/sound/cs35l41.h */
#define L41_TEST_KEY_CTL             0x00000040
#define L41_PWR_CTRL1                0x00002014
#define L41_PWR_CTRL2                0x00002018
#define L41_PWR_CTRL3                0x0000201c
#define L41_GLOBAL_CLK_CTRL          0x00002c0c
#define L41_PLL_CLK_CTRL             0x00002c04
#define L41_SP_ENABLES               0x00004800
#define L41_SP_RATE_CTRL             0x00004804
#define L41_SP_FORMAT                0x00004808
#define L41_SP_HIZ_CTRL              0x0000480c
#define L41_SP_FRAME_TX_SLOT         0x00004810
#define L41_SP_FRAME_RX_SLOT         0x00004820
#define L41_SP_TX_WL                 0x00004830
#define L41_SP_RX_WL                 0x00004840
#define L41_DAC_PCM1_SRC             0x00004c00
#define L41_AMP_DIG_VOL_CTRL         0x00006000
#define L41_AMP_GAIN_CTRL            0x00006c04
#define L41_IRQ1_STATUS1             0x00010010
#define L41_IRQ1_STATUS2             0x00010014
#define L41_IRQ1_MASK1               0x00010110
#define L41_GPIO_PAD_CONTROL         0x0000242c
#define L41_GPIO1_CTRL1              0x00011008
#define L41_GPIO2_CTRL1              0x0001100c
#define L41_BSTCVRT_COEFF            0x00003810
#define L41_BSTCVRT_SLOPE_LBST       0x00003814
#define L41_BSTCVRT_PEAK_CUR         0x00003808
#define L41_TST_FS_MON0              0x00002d10
#define L41_VIMON_SPKMON_RESYNC      0x00004100
#define L41_VPVBST_FS_SEL            0x00004400
#define L41_BSTCVRT_DCM_CTRL         0x0000381c
#define L41_DSP1_YM_ACCEL_PL0_PRI    0x02bc20e0
#define L41_DSP1_XM_ACCEL_PL0_PRI    0x02bc2020
#define L41_ASP_TX3_SRC              0x00004c28
#define L41_ASP_TX4_SRC              0x00004c2c
#define L41_DSP1_CCM_CORE_CTRL       0x02bc1000
#define L41_OTP_MEM0                 0x00000400
#define L41_OTP_SIZE_WORDS           0x00000020
#define L41_INT1_MASK_DEFAULT        0x7ffcfe3f
#define L41_GLOBAL_EN_MASK           0x00000001
#define L41_PUP_DONE_MASK            0x01000000
#define L41_PDN_DONE_MASK            0x00800000
#define L41_BST_EN_MASK              0x00000030
#define L41_BST_EN_SHIFT             0x00000004
#define L41_BST_EN_DEFAULT           0x00000002
#define L41_BST_K1_MASK              0x000000ff
#define L41_BST_K1_SHIFT             0x00000000
#define L41_BST_K2_MASK              0x0000ff00
#define L41_BST_K2_SHIFT             0x00000008
#define L41_BST_SLOPE_MASK           0x0000ff00
#define L41_BST_SLOPE_SHIFT          0x00000008
#define L41_BST_LBST_VAL_MASK        0x00000003
#define L41_BST_LBST_VAL_SHIFT       0x00000000
#define L41_BST_IPK_MASK             0x0000007f
#define L41_BST_IPK_SHIFT            0x00000000
#define L41_ASP_DOUT_HIZ_MASK        0x00000003
#define L41_GPIO2_CTRL_MASK          0x07000000
#define L41_GPIO2_CTRL_SHIFT         0x00000018
#define L41_GPIO_POL_MASK            0x00001000
#define L41_GPIO_DIR_MASK            0x80000000
#define L41_GLOBAL_FS_MASK           0x0000001f
#define L41_GLOBAL_FS_SHIFT          0x00000000
#define L41_ASP_WIDTH_RX_MASK        0xff000000
#define L41_ASP_WIDTH_RX_SHIFT       0x00000018
#define L41_ASP_RX_WL_MASK           0x0000003f
#define L41_ASP_RX_WL_SHIFT          0x00000000
#define L41_ASP_FMT_MASK             0x00000700
#define L41_ASP_FMT_SHIFT            0x00000008
#define L41_SCLK_MSTR_MASK           0x00000010
#define L41_LRCLK_MSTR_MASK          0x00000001
#define L41_SCLK_INV_MASK            0x00000040
#define L41_LRCLK_INV_MASK           0x00000004
#define L41_PLL_OPENLOOP_MASK        0x00000800
#define L41_REFCLK_FREQ_MASK         0x000007e0
#define L41_REFCLK_FREQ_SHIFT        0x00000005
#define L41_PLL_CLK_EN_MASK          0x00000010
#define L41_PLL_CLK_SEL_MASK         0x00000007
#define L41_PLLSRC_SCLK              0x00000000
#define L41_ASP_SOURCE_MASK          0x0000007f
#define L41_AMP_GAIN_PCM_MASK        0x000003e0
#define L41_AMP_GAIN_PCM_SHIFT       0x00000005
#define L41_AMP_SHORT_ERR            0x80000000
#define L41_TEMP_WARN                0x00008000
#define L41_TEMP_ERR                 0x00020000
#define L41_BST_OVP_ERR              0x00000040
#define L41_BST_DCM_UVP_ERR          0x00000080
#define L41_BST_SHORT_ERR            0x00000100
#define L41_PLL_LOCK                 0x00000002
#define L41_AMP_VOL_PCM_MASK         0x00003ff8      /* AMP_DIG_VOL_CTRL bits 13:3, 1/8 dB steps, signed; 0x400 mute */
#define L41_AMP_EN_MASK              0x00000001      /* PWR_CTRL2: "Main AMP" */
#define L41_CLASSH_EN_MASK           0x00000010      /* PWR_CTRL3 bit 4: "CLASS H" */
#define L41_ASP_RX1_EN_MASK          0x00010000      /* SP_ENABLES bit 16: "ASPRX1" */
#define L41_DACPCM_ASPRX1            0x08            /* DAC_PCM1_SRC: "PCM Source" = ASP */
#define L41_GPIO2_INT_PUSH_PULL_LOW  4               /* cirrus,gpio-src-select = <5> -> GPIO2 the interrupt */
#define L41_FS_48K                   0x03
#define L41_REFCLK_6144000           0x28            /* cs35l41_pll_sysclk */
#define L41_FS_MON_6144000           (16u | (24u << 12))     /* cs35l41_fs_mon: fs1 16, fs2 24 */
#define L41_REVID_B2                 0xb2
#define L41_REVID_REG                0x00000004
#define L41_OTPID_REG                0x00000010

/* Linux cs35l41-lib.c otp_map_1 (OTP ids 0x01, 0x08; bit offset 16, word offset 2): register, shift, size */
static const struct { uint32_t reg; uint8_t shift, size; } otp_map_1[] = {
	{ 0x00002030, 0, 4 }, { 0x00002030, 7, 1 }, { 0x0000208c, 24, 6 }, { 0x00002090, 14, 4 }, { 0x00002090, 10, 4 },
	{ 0x0000300c, 11, 4 }, { 0x0000394c, 23, 2 }, { 0x00003950, 0, 7 }, { 0x00003950, 8, 7 }, { 0x00003950, 16, 8 },
	{ 0x00003950, 24, 8 }, { 0x00003954, 0, 7 }, { 0x00003954, 8, 7 }, { 0x00003954, 16, 8 }, { 0x00003954, 24, 8 },
	{ 0x00003958, 0, 7 }, { 0x00003958, 8, 7 }, { 0x00003958, 16, 8 }, { 0x00003958, 24, 8 }, { 0x0000395c, 0, 7 },
	{ 0x0000395c, 8, 7 }, { 0x0000395c, 16, 8 }, { 0x0000395c, 24, 8 }, { 0x0000416c, 0, 8 }, { 0x00004160, 0, 7 },
	{ 0x0000416c, 8, 8 }, { 0x00004160, 16, 10 }, { 0x0000416c, 16, 12 }, { 0x0000416c, 28, 1 }, { 0x00004170, 0, 6 },
	{ 0x00004170, 6, 1 }, { 0x00004170, 8, 6 }, { 0x00004170, 14, 1 }, { 0x00004170, 16, 9 }, { 0x00004360, 0, 5 },
	{ 0x00004360, 6, 9 }, { 0x00004448, 0, 8 }, { 0x00004448, 8, 8 }, { 0x00004448, 16, 8 }, { 0x00004448, 24, 8 },
	{ 0x0000444c, 0, 3 }, { 0x00006e30, 0, 5 }, { 0x00006e30, 8, 5 }, { 0x00006e30, 16, 5 }, { 0x00006e30, 24, 5 },
	{ 0x00006e34, 0, 5 }, { 0x00006e34, 8, 5 }, { 0x00006e34, 16, 5 }, { 0x00006e34, 24, 5 }, { 0x00006e38, 0, 5 },
	{ 0x00006e38, 8, 5 }, { 0x00006e38, 16, 5 }, { 0x00006e38, 24, 5 }, { 0x00006e3c, 0, 5 }, { 0x00006e3c, 8, 5 },
	{ 0x00006e3c, 16, 5 }, { 0x00006e3c, 24, 5 }, { 0x00006e40, 0, 5 }, { 0x00006e40, 8, 5 }, { 0x00006e40, 16, 5 },
	{ 0x00006e40, 24, 5 }, { 0x00006e44, 0, 5 }, { 0x00006e48, 0, 10 }, { 0x00006e48, 10, 10 }, { 0x00006e48, 20, 10 },
	{ 0x00006e4c, 0, 10 }, { 0x00006e4c, 10, 10 }, { 0x00006e4c, 20, 10 }, { 0x00006e50, 0, 10 }, { 0x00006e50, 10, 10 },
	{ 0x00006e50, 20, 10 }, { 0x00006e54, 0, 10 }, { 0x00006e54, 10, 10 }, { 0x00006e54, 20, 10 }, { 0x00006e58, 0, 10 },
	{ 0x00006e58, 10, 10 }, { 0x00006e58, 20, 10 }, { 0x00006e5c, 0, 10 }, { 0x00006e5c, 10, 10 }, { 0x00006e5c, 20, 10 },
	{ 0x00006e60, 0, 10 }, { 0x00006e60, 10, 10 }, { 0x00006e60, 20, 10 }, { 0x00006e64, 0, 10 }, { 0x00007418, 7, 5 },
	{ 0x0000741c, 0, 5 }, { 0x0000741c, 11, 4 }, { 0x0000741c, 19, 4 }, { 0x00007434, 17, 1 }, { 0x00007434, 18, 7 },
	{ 0x00007068, 0, 9 }, { 0x0000410c, 7, 1 }, { 0x0000400c, 0, 7 }, { 0x00000000, 0, 1 }, { 0x00017040, 0, 8 },
	{ 0x00017040, 8, 8 }, { 0x00017040, 16, 8 }, { 0x00017040, 24, 8 }, { 0x00017044, 0, 24 },
};
/* 99 entries */

static vm_offset_t spi, cmu, sysreg, rst_l, rst_r, cs_r;
static uint32_t mode;
/* the controller's chip-select register while the right amplifier (its own chip select a GPIO) is spoken to: Samsung's
 * spi-s3c64xx.c with samsung,spi-chip-select-mode set (AUTO_CS_MODE) writes SIG_INACT | SLAVE_AUTO | NSC_CNT_2; in manual
 * mode with its own select inactive the controller does not clock at all */
#define CS_AUTO                 (1u << 1)
#define CS_NSC_CNT_2            (2u << 4)
static uint32_t cs_reg_right = CS_INACTIVE | CS_AUTO | CS_NSC_CNT_2;

static inline uint32_t
rd(vm_offset_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void
wr(vm_offset_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* pin PIN of the bank at BANK an output at LEVEL */
static void
pin_out(vm_offset_t bank, int pin, int level)
{
	uint32_t dat = rd(bank + 4);

	wr(bank + 4, level ? (dat | (1u << pin)) : (dat & ~(1u << pin)));
	wr(bank, (rd(bank) & ~(0xfu << (pin * 4))) | (1u << (pin * 4)));
}

/* ---- the clock, the USI block, the controller (as the touch screen's loader does for USI0) ------------------------ */

static void
spi_clock_on(void)
{
	wr(cmu + CMU_MUX_USI7, rd(cmu + CMU_MUX_USI7) & ~(1u << 4));        /* the crystal */
	wr(cmu + CMU_DIV_USI7, rd(cmu + CMU_DIV_USI7) & ~0xfu);
	wr(cmu + CMU_GATE_IPCLK_USI7, CMU_GATE_ON);
	wr(cmu + CMU_GATE_PCLK_USI7, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI7, CMU_GATE_ON);
}

static bool
spi_init(void)
{
	uint32_t v;

	wr(sysreg + SYSREG_USI7_MODE, USI_MODE_SPI);
	wr(spi + USI_CON, rd(spi + USI_CON) & ~USI_CON_RESET);
	delay(10);
	v = rd(spi + USI_OPTION);
	wr(spi + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);

	wr(spi + CH_CFG, CH_SW_RST);
	delay(10);
	wr(spi + CH_CFG, 0);
	wr(spi + MODE_CFG, 0);
	wr(spi + CS_REG, CS_INACTIVE);
	wr(spi + INT_EN, 0);
	wr(spi + SWAP_CFG, 0);
	wr(spi + PENDING_CLR, 0x1f);
	wr(spi + PENDING_CLR, 0);

	/* the controller answers: a field of MODE_CFG written and read back */
	wr(spi + MODE_CFG, 0x55u << 5);
	v = rd(spi + MODE_CFG);
	wr(spi + MODE_CFG, 0);
	lynx_tlog("lynx amp: SPI controller at 0x%llx: MODE_CFG read back 0x%x (0x%x written), STATUS 0x%x\n", SPI_PHYS, v,
	    0x55u << 5, rd(spi + STATUS));
	return v == (0x55u << 5);
}

/* one transfer to chip CHIP (0 left, 1 right): LENGTH bytes out of TX, as many in to RX */
static bool
spi_transfer(int chip, const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	uint32_t sent = 0, got = 0;
	uint64_t deadline;

	wr(spi + CH_CFG, CH_SW_RST);
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, (length & 0xffff) | PACKET_CNT_EN);
	wr(spi + CH_CFG, CH_TXCH_ON | CH_RXCH_ON);
	if (chip == 0) {
		wr(spi + CS_REG, 0);
	} else {
		pin_out(cs_r, CS_R_PIN, 0);
		wr(spi + CS_REG, cs_reg_right);
	}
	clock_interval_to_deadline(50, NSEC_PER_MSEC, &deadline);
	while (got < length && mach_absolute_time() < deadline) {
		while (sent < length && sent - got < FIFO_INFLIGHT) {
			wr(spi + TX_DATA, tx ? tx[sent] : 0);
			sent++;
		}
		for (uint32_t n = RX_LVL(rd(spi + STATUS)); n > 0 && got < length; n--) {
			uint8_t b = (uint8_t)rd(spi + RX_DATA);

			if (rx) {
				rx[got] = b;
			}
			got++;
		}
	}
	if (chip == 0) {
		wr(spi + CS_REG, CS_INACTIVE);
	} else {
		pin_out(cs_r, CS_R_PIN, 1);
		wr(spi + CS_REG, CS_INACTIVE);
	}
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, 0);
	return got == length;
}

/* ---- the CS35L41's registers -------------------------------------------------------------------------------------- */

static bool
amp_read(int chip, uint32_t reg, uint32_t *value)
{
	uint8_t tx[10] = { (uint8_t)((reg >> 24) | 0x80), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
	uint8_t rx[10];

	if (!spi_transfer(chip, tx, rx, sizeof(tx))) {
		return false;
	}
	*value = ((uint32_t)rx[6] << 24) | ((uint32_t)rx[7] << 16) | ((uint32_t)rx[8] << 8) | rx[9];
	return true;
}

static bool
amp_write(int chip, uint32_t reg, uint32_t value)
{
	uint8_t tx[10] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg, 0, 0,
		           (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value };

	return spi_transfer(chip, tx, NULL, sizeof(tx));
}

static bool
amp_update(int chip, uint32_t reg, uint32_t mask, uint32_t value)
{
	uint32_t v;

	if (!amp_read(chip, reg, &v)) {
		return false;
	}
	return amp_write(chip, reg, (v & ~mask) | (value & mask));
}

static void
amp_test_key(int chip, bool unlock)
{
	amp_write(chip, L41_TEST_KEY_CTL, unlock ? 0x55 : 0xcc);
	amp_write(chip, L41_TEST_KEY_CTL, unlock ? 0xaa : 0x33);
}

/* cs35l41_otp_unpack (the test key unlocked): the chip's factory trims from its OTP into their registers */
static int
amp_otp_unpack(int chip)
{
	uint32_t mem[L41_OTP_SIZE_WORDS], val;
	int bit = 16, word = 2, written = 0;

	for (uint32_t i = 0; i < L41_OTP_SIZE_WORDS; i++) {
		if (!amp_read(chip, L41_OTP_MEM0 + 4 * i, &mem[i])) {
			return -1;
		}
	}
	for (unsigned i = 0; i < sizeof(otp_map_1) / sizeof(otp_map_1[0]); i++) {
		int size = otp_map_1[i].size;

		if (bit + size - 1 >= 32) {             /* across two words */
			val = (mem[word] >> bit) & (uint32_t)((1ull << (32 - bit)) - 1);
			word++;
			val |= (mem[word] & (uint32_t)((1ull << (bit + size - 32)) - 1)) << (32 - bit);
			bit += size - 32;
		} else {
			val = (mem[word] >> bit) & (uint32_t)((1ull << size) - 1);
			bit += size;
		}
		if (bit == 32) {
			bit = 0;
			word++;
		}
		if (otp_map_1[i].reg != 0) {
			uint32_t mask = (uint32_t)(((1ull << size) - 1) << otp_map_1[i].shift);
			amp_update(chip, otp_map_1[i].reg, mask, val << otp_map_1[i].shift);
			written++;
		}
	}
	return written;
}

/* chip CHIP set up for the AoC's TDM_0, powered down still (cs35l41_probe's second half, set_pdata, set_dai_fmt,
 * hw_params, set_sysclk, the DAPM path ASPRX1 -> PCM Source -> CLASS H) */
static bool
amp_setup(int chip)
{
	const char *name = chip ? "right" : "left";
	uint32_t revid = 0, otpid = 0;

	amp_read(chip, L41_REVID_REG, &revid);
	amp_read(chip, L41_OTPID_REG, &otpid);
	if ((revid & 0xff) != L41_REVID_B2 || (otpid & 0xf) != 0x8) {
		lynx_tlog("lynx amp: %s: revision 0x%x, OTP id 0x%x -- only B2 with OTP map 8 is known here; not set up\n", name,
		    revid, otpid);
		return false;
	}
	amp_test_key(chip, true);
	/* cs35l41_revb2_errata_patch, then DSP1_CCM_CORE_CTRL 0 */
	amp_write(chip, L41_VIMON_SPKMON_RESYNC, 0);
	amp_write(chip, 0x4310, 0);
	amp_write(chip, L41_VPVBST_FS_SEL, 0);
	amp_write(chip, L41_BSTCVRT_DCM_CTRL, 0x51);
	amp_write(chip, L41_DSP1_YM_ACCEL_PL0_PRI, 0);
	amp_write(chip, L41_DSP1_XM_ACCEL_PL0_PRI, 0);
	amp_write(chip, L41_PWR_CTRL2, 0);
	amp_write(chip, L41_AMP_GAIN_CTRL, 0);
	amp_write(chip, L41_ASP_TX3_SRC, 0);
	amp_write(chip, L41_ASP_TX4_SRC, 0);
	amp_write(chip, L41_DSP1_CCM_CORE_CTRL, 0);
	int trims = amp_otp_unpack(chip);
	amp_test_key(chip, false);

	/* GPIO1 an input (unused), GPIO2 the interrupt line, push-pull active low; the critical interrupts unmasked */
	amp_update(chip, L41_GPIO1_CTRL1, L41_GPIO_POL_MASK | L41_GPIO_DIR_MASK, L41_GPIO_DIR_MASK);
	amp_update(chip, L41_GPIO2_CTRL1, L41_GPIO_POL_MASK | L41_GPIO_DIR_MASK, 0);
	amp_update(chip, L41_GPIO_PAD_CONTROL, L41_GPIO2_CTRL_MASK, L41_GPIO2_INT_PUSH_PULL_LOW << L41_GPIO2_CTRL_SHIFT);
	amp_write(chip, L41_IRQ1_MASK1, L41_INT1_MASK_DEFAULT);

	/* cs35l41_boost_config: 1000 nH -> LBST 0; 4 uF -> range 0; 3500 mA -> ((3500 - 1600) / 50) + 0x10 */
	amp_update(chip, L41_BSTCVRT_COEFF, L41_BST_K1_MASK | L41_BST_K2_MASK, (0x24u << L41_BST_K1_SHIFT) | (0x24u << L41_BST_K2_SHIFT));
	amp_update(chip, L41_BSTCVRT_SLOPE_LBST, L41_BST_SLOPE_MASK | L41_BST_LBST_VAL_MASK, 0x75u << L41_BST_SLOPE_SHIFT);
	amp_update(chip, L41_BSTCVRT_PEAK_CUR, L41_BST_IPK_MASK, ((3500u - 1600u) / 50u + 0x10u) << L41_BST_IPK_SHIFT);
	amp_update(chip, L41_PWR_CTRL2, L41_BST_EN_MASK, L41_BST_EN_DEFAULT << L41_BST_EN_SHIFT);
	amp_update(chip, L41_SP_HIZ_CTRL, L41_ASP_DOUT_HIZ_MASK, 3);   /* cirrus,asp-sdout-hiz = <3> */

	/* DSP-A, the AoC the clock provider, neither clock inverted; 48 kHz; 32-bit slots (the word length below) */
	amp_update(chip, L41_SP_FORMAT, L41_SCLK_MSTR_MASK | L41_LRCLK_MSTR_MASK | L41_ASP_FMT_MASK | L41_LRCLK_INV_MASK |
	    L41_SCLK_INV_MASK, 0);
	amp_update(chip, L41_GLOBAL_CLK_CTRL, L41_GLOBAL_FS_MASK, L41_FS_48K << L41_GLOBAL_FS_SHIFT);
	amp_update(chip, L41_SP_FORMAT, L41_ASP_WIDTH_RX_MASK, 32u << L41_ASP_WIDTH_RX_SHIFT);
	{
		/* the word length the amps take from each 32-bit slot: 32, as Android's mixer gives them (amp_wl=N: 16, 24, 32) */
		uint32_t wl = 32;

		PE_parse_boot_argn("amp_wl", &wl, sizeof(wl));
		if (wl != 16 && wl != 24 && wl != 32) {
			wl = 32;
		}
		amp_update(chip, L41_SP_RX_WL, L41_ASP_RX_WL_MASK, wl << L41_ASP_RX_WL_SHIFT);
	}
	/* the left amplifier plays slot 0, the right slot 1 (RX1 bits 5:0, RX2 bits 13:8) */
	amp_update(chip, L41_SP_FRAME_RX_SLOT, 0x3f3f, chip ? 0x0001 : 0x0100);

	/* the PLL from SCLK (6.144 MHz = 48 kHz x 4 slots x 32 bits); the frame-sync monitor for that clock */
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_OPENLOOP_MASK, L41_PLL_OPENLOOP_MASK);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_REFCLK_FREQ_MASK, L41_REFCLK_6144000 << L41_REFCLK_FREQ_SHIFT);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_EN_MASK, 0);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_SEL_MASK, L41_PLLSRC_SCLK);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_OPENLOOP_MASK, 0);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_EN_MASK, L41_PLL_CLK_EN_MASK);
	amp_write(chip, L41_TST_FS_MON0, L41_FS_MON_6144000);

	/* the path without the DSP: ASP RX1 on, the DAC fed from it, class H; 0 dB digital, the least analog gain */
	amp_update(chip, L41_SP_ENABLES, L41_ASP_RX1_EN_MASK, L41_ASP_RX1_EN_MASK);
	amp_update(chip, L41_DAC_PCM1_SRC, L41_ASP_SOURCE_MASK, L41_DACPCM_ASPRX1);
	amp_update(chip, L41_PWR_CTRL3, L41_CLASSH_EN_MASK, L41_CLASSH_EN_MASK);
	amp_update(chip, L41_AMP_DIG_VOL_CTRL, L41_AMP_VOL_PCM_MASK, 0);
	{
		/* the analog gain: the least (0: 0.5 dB) unless amp_gain=N (0-20, 0.5 dB + N dB; Android's mixer: 17) */
		uint32_t gain = 0;

		PE_parse_boot_argn("amp_gain", &gain, sizeof(gain));
		if (gain > 20) {
			gain = 0;
		}
		amp_update(chip, L41_AMP_GAIN_CTRL, L41_AMP_GAIN_PCM_MASK, gain << L41_AMP_GAIN_PCM_SHIFT);
	}

	uint32_t fmt = 0, slots = 0, pll = 0, pwr2 = 0, src = 0;
	amp_read(chip, L41_SP_FORMAT, &fmt);
	amp_read(chip, L41_SP_FRAME_RX_SLOT, &slots);
	amp_read(chip, L41_PLL_CLK_CTRL, &pll);
	amp_read(chip, L41_PWR_CTRL2, &pwr2);
	amp_read(chip, L41_DAC_PCM1_SRC, &src);
	lynx_tlog("lynx amp: %s: set up: %d OTP trims, SP_FORMAT 0x%x, RX slots 0x%x, PLL_CLK_CTRL 0x%x, PWR_CTRL2 0x%x, DAC source 0x%x\n",
	    name, trims, fmt, slots, pll, pwr2, src);
	return trims > 0;
}

static bool amps_ready;
static uint32_t speaker_allowed;                /* boot-arg speaker=1: lynx_amp_speaker(1) may power them up (audible) */

static void
amp_say_status(int chip, const char *when)
{
	uint32_t st[4] = { 0 }, pwr1 = 0;

	for (int i = 0; i < 4; i++) {
		amp_read(chip, L41_IRQ1_STATUS1 + 4 * i, &st[i]);
	}
	amp_read(chip, L41_PWR_CTRL1, &pwr1);
	lynx_tlog("lynx amp: %s %s: PWR_CTRL1 0x%x, IRQ1_STATUS 0x%x 0x%x 0x%x 0x%x%s%s%s%s%s%s%s\n", chip ? "right" : "left", when,
	    pwr1, st[0], st[1], st[2], st[3], (st[0] & L41_AMP_SHORT_ERR) ? " AMP SHORT" : "", (st[0] & L41_TEMP_ERR) ? " TEMP ERROR" : "",
	    (st[0] & L41_TEMP_WARN) ? " temp warning" : "", (st[0] & L41_BST_OVP_ERR) ? " BOOST OVP" : "",
	    (st[0] & L41_BST_SHORT_ERR) ? " BOOST SHORT" : "", (st[0] & L41_BST_DCM_UVP_ERR) ? " boost UVP" : "",
	    (st[2] & L41_PLL_LOCK) ? " PLL locked" : "");
}

/* both amplifiers on (the AoC's TDM running) or off: Linux's "Main AMP" PRE_PMU / POST_PMD (cs35l41_global_enable) */
int lynx_amp_speaker(int on);
int
lynx_amp_speaker(int on)
{
	static bool said;

	if (!amps_ready) {
		return -1;
	}
	if (on && !speaker_allowed) {
		/* nothing audible unless asked for: the speakers only with speaker=1 on the command line */
		if (!said) {
			said = true;
			lynx_tlog("lynx amp: asked to play, but speaker=1 is not given: the amplifiers stay off (silent)\n");
		}
		return -1;
	}
	for (int chip = 0; chip < 2; chip++) {
		uint32_t st = 0;
		int ms;

		if (on) {
			amp_test_key(chip, true);
			amp_write(chip, 0x2084, 0x002f1aa0);    /* cs35l41_pup_patch */
			amp_test_key(chip, false);
			amp_update(chip, L41_PWR_CTRL1, L41_GLOBAL_EN_MASK, 1);
			for (ms = 0; ms < 100; ms++) {
				if (amp_read(chip, L41_IRQ1_STATUS1, &st) && (st & L41_PUP_DONE_MASK)) {
					break;
				}
				sleep_ms(1);
			}
			amp_write(chip, L41_IRQ1_STATUS1, L41_PUP_DONE_MASK);
			amp_update(chip, L41_PWR_CTRL2, L41_AMP_EN_MASK, L41_AMP_EN_MASK);
			lynx_tlog("lynx amp: %s: powered up (%s after %d ms)\n", chip ? "right" : "left",
			    (st & L41_PUP_DONE_MASK) ? "PUP_DONE" : "NO PUP_DONE", ms);
			amp_say_status(chip, "on");
		} else {
			amp_say_status(chip, "before off");
			amp_update(chip, L41_PWR_CTRL2, L41_AMP_EN_MASK, 0);
			amp_update(chip, L41_PWR_CTRL1, L41_GLOBAL_EN_MASK, 0);
			for (ms = 0; ms < 100; ms++) {
				if (amp_read(chip, L41_IRQ1_STATUS1, &st) && (st & L41_PDN_DONE_MASK)) {
					break;
				}
				sleep_ms(1);
			}
			amp_write(chip, L41_IRQ1_STATUS1, L41_PDN_DONE_MASK);
			amp_test_key(chip, true);
			amp_write(chip, 0x2084, 0x002f1aa3);    /* cs35l41_pdn_patch */
			amp_test_key(chip, false);
			lynx_tlog("lynx amp: %s: powered down (%s after %d ms)\n", chip ? "right" : "left",
			    (st & L41_PDN_DONE_MASK) ? "PDN_DONE" : "NO PDN_DONE", ms);
		}
	}
	return 0;
}

/*
 * A listening test for the AoC's driver (aocplay_matrix=1): lynx_amp_variant(K) powers both down, sets their audio port
 * up the K-th way, powers them up -- K beeps with each K name the setting that works. 1 as set up (DSP-A, 32-bit slots
 * and words, left slot 0 / right slot 1), 2 I2S, 3 slots 2 and 3, 4 SCLK inverted, 5 LRCLK inverted, 6 16-bit slots and
 * words. Anything else: as set up, powered down.
 */
int lynx_amp_variant(int k);
int
lynx_amp_variant(int k)
{
	static const char *what[] = { "as set up", "as set up (DSP-A, 32-bit, left slot 0 / right slot 1)", "I2S",
		                      "slots 2 and 3", "SCLK inverted", "LRCLK inverted", "16-bit slots and words" };

	if (!amps_ready || !speaker_allowed) {
		return -1;
	}
	lynx_amp_speaker(0);
	for (int chip = 0; chip < 2; chip++) {
		uint32_t fmt = 0, inv = 0, width = 32, wl = 32, slots = chip ? 0x0001 : 0x0100;

		switch (k) {
		case 2: fmt = 2; break;
		case 3: slots = chip ? 0x0203 : 0x0302; break;
		case 4: inv = L41_SCLK_INV_MASK; break;
		case 5: inv = L41_LRCLK_INV_MASK; break;
		case 6: width = 16; wl = 16; break;
		default: break;
		}
		amp_update(chip, L41_SP_FORMAT, L41_ASP_FMT_MASK | L41_SCLK_INV_MASK | L41_LRCLK_INV_MASK | L41_ASP_WIDTH_RX_MASK,
		    (fmt << L41_ASP_FMT_SHIFT) | inv | (width << L41_ASP_WIDTH_RX_SHIFT));
		amp_update(chip, L41_SP_FRAME_RX_SLOT, 0x3f3f, slots);
		amp_update(chip, L41_SP_RX_WL, L41_ASP_RX_WL_MASK, wl << L41_ASP_RX_WL_SHIFT);
	}
	lynx_tlog("lynx amp: VARIANT %d: %s\n", k, (k >= 1 && k <= 6) ? what[k] : what[0]);
	return (k >= 1 && k <= 6) ? lynx_amp_speaker(1) : 0;
}

/* chip CHIP out of reset and asked who it is (cs35l41_probe's first half) */
static bool
amp_identify(int chip)
{
	const char *name = chip ? "right" : "left";
	uint32_t st4 = 0, st3 = 0, devid = 0, revid = 0, otpid = 0;
	int waited;

	if (chip == 1) {
		/* which chip-select register value lets the right amplifier answer (alone): each tried for its device id */
		static const uint32_t tries[] = { CS_INACTIVE | CS_AUTO | CS_NSC_CNT_2, CS_AUTO | CS_NSC_CNT_2, CS_AUTO, 0 };
		for (unsigned t = 0; t < sizeof(tries) / sizeof(tries[0]); t++) {
			cs_reg_right = tries[t];
			devid = 0xdeadbeef;
			bool ok = amp_read(chip, CS35L41_DEVID, &devid);
			lynx_tlog("lynx amp: right, CS_REG 0x%x while selected: %s, device id 0x%x\n", tries[t],
			    ok ? "transfer done" : "TRANSFER TIMED OUT", devid);
			if (ok && (devid == CS35L41_CHIP_ID || devid == CS35L41R_CHIP_ID)) {
				break;
			}
		}
		lynx_tlog("lynx amp: pins: gpp22 (right's chip select) CON 0x%x DAT 0x%x, gpp17 (left's reset) CON 0x%x DAT 0x%x, gpp25 "
		    "(right's reset) CON 0x%x DAT 0x%x\n", rd(cs_r), rd(cs_r + 4), rd(rst_l), rd(rst_l + 4), rd(rst_r), rd(rst_r + 4));
	}
	for (waited = 0; waited < 100; waited++) {
		if (amp_read(chip, CS35L41_IRQ1_STATUS4, &st4) && (st4 & CS35L41_OTP_BOOT_DONE)) {
			break;
		}
		sleep_ms(1);
	}
	amp_read(chip, CS35L41_IRQ1_STATUS3, &st3);
	amp_read(chip, CS35L41_DEVID, &devid);
	amp_read(chip, CS35L41_REVID, &revid);
	amp_read(chip, CS35L41_OTPID, &otpid);
	lynx_tlog("lynx amp: %s: OTP boot %s after %d ms (IRQ1_STATUS4 0x%x, STATUS3 0x%x%s), device id 0x%x (%s), revision 0x%x "
	    "(metal %u), OTP id 0x%x\n", name, (st4 & CS35L41_OTP_BOOT_DONE) ? "done" : "NOT DONE", waited, st4, st3,
	    (st3 & CS35L41_OTP_BOOT_ERR) ? ": OTP BOOT ERROR" : "", devid,
	    devid == CS35L41_CHIP_ID ? "a CS35L41" : devid == CS35L41R_CHIP_ID ? "a CS35L41R" : "not a CS35L41",
	    revid, revid & 0xf, otpid & 0xf);
	return devid == CS35L41_CHIP_ID || devid == CS35L41R_CHIP_ID;
}

static void
lynx_amp_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	spi_clock_on();
	if (!spi_init()) {
		lynx_tlog("lynx amp: the SPI controller does not answer; stopping\n");
		return;
	}
	pin_out(cs_r, CS_R_PIN, 1);                     /* the right amplifier's chip select inactive */
	pin_out(rst_l, RST_L_PIN, 0);                   /* both in reset (cs35l41_probe: GPIOD_OUT_LOW, 2 ms) */
	pin_out(rst_r, RST_R_PIN, 0);
	sleep_ms(3);
	pin_out(rst_l, RST_L_PIN, 1);
	pin_out(rst_r, RST_R_PIN, 1);
	sleep_ms(3);
	bool l = amp_identify(0), r = amp_identify(1);
	lynx_tlog("lynx amp: %s\n", l && r ? "BOTH SPEAKER AMPLIFIERS ANSWER" : "an amplifier did not answer");
	if (mode >= 2 && l && r && amp_setup(0) && amp_setup(1)) {
		amps_ready = true;
		lynx_tlog("lynx amp: both set up for the AoC's TDM_0, powered down until lynx_amp_speaker(1)\n");
	}
}

__startup_func
static void
lynx_amp_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("amp", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	PE_parse_boot_argn("speaker", &speaker_allowed, sizeof(speaker_allowed));
	spi = ml_io_map(SPI_PHYS, 0x1000);
	cmu = ml_io_map(CMU_PHYS, 0x2000);
	sysreg = ml_io_map(SYSREG_PHYS, 0x1000);
	rst_l = ml_io_map(RST_L_PHYS, 0x1000);
	rst_r = ml_io_map(RST_R_PHYS, 0x1000);
	cs_r = ml_io_map(CS_R_PHYS, 0x1000);
	if (!spi || !cmu || !sysreg || !rst_l || !rst_r || !cs_r) {
		lynx_tlog("lynx amp: could not map its registers\n");
		return;
	}
	rst_l += RST_L_BANK;
	rst_r += RST_R_BANK;
	cs_r += CS_R_BANK;
	start = thread_call_allocate(lynx_amp_start, NULL);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);       /* after the USB log is up */
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_amp_init);
