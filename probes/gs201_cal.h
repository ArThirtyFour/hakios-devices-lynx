/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The gs201 PCIe PHY calibration of Google's kernel/gs (android-gs-lynx-5.10-android15),
 * drivers/pci/controller/dwc/pcie-exynos-gs201-rc-cal.c (SOURCES.md), "CAL ver 210802":
 * exynos_pcie_rc_phy_all_pwrdn(), exynos_pcie_rc_phy_all_pwrdn_clear() and exynos_pcie_rc_pcie_phy_config(),
 * statement for statement. Changed for the loader only: the three bases are file variables, the kernel's
 * logging is the loader's, and the UDBG "internal dump" settings at the end of phy_config are left out
 * (a debug trace, not part of bringing the link up).
 *
 * The including file provides: wr(addr, value), rd(addr), wait_us(us), log_hex(), and the bases
 * CAL_ELBI, CAL_PHY, CAL_PCS, and CAL_LANES (the FDT's num-lanes).
 */
#define writel(v, a)    wr((uint64_t)(a), (uint32_t)(v))
#define readl(a)        rd((uint64_t)(a))
#define udelay(n)       wait_us(n)
#define mdelay(n)       wait_us((n) * 1000u)

/* PHY all power down */
static void cal_phy_all_pwrdn(void)
{
	uint64_t phy_base_regs = CAL_PHY;
	uint64_t phy_pcs_base_regs = CAL_PCS;
	uint32_t val;

	writel(0xA8, phy_base_regs + 0x404);
	writel(0x20, phy_base_regs + 0x408);
	writel(0x0A, phy_base_regs + 0x40C);

	writel(0x0A, phy_base_regs + 0x800);
	writel(0xBF, phy_base_regs + 0x804);
	writel(0x02, phy_base_regs + 0xA40);
	writel(0x2A, phy_base_regs + 0xA44);
	writel(0xAA, phy_base_regs + 0xA48);
	writel(0xA8, phy_base_regs + 0xA4C);
	writel(0x80, phy_base_regs + 0xA50);

	writel(0x0A, phy_base_regs + 0x1000);
	writel(0xBF, phy_base_regs + 0x1004);
	writel(0x02, phy_base_regs + 0x1240);
	writel(0x2A, phy_base_regs + 0x1244);
	writel(0xAA, phy_base_regs + 0x1248);
	writel(0xA8, phy_base_regs + 0x124C);
	writel(0x80, phy_base_regs + 0x1250);

	/* Disable PHY PMA */
	val = readl(phy_base_regs + 0x400);
	val &= ~(0x1 << 7);
	writel(val, phy_base_regs + 0x400);

	/* disable rate switching */
	val = readl(phy_pcs_base_regs + 0x184);
	val &= ~0xB0;
	writel(val, phy_pcs_base_regs + 0x184);
}

/* PHY all power down clear */
static void cal_phy_all_pwrdn_clear(void)
{
	uint64_t phy_base_regs = CAL_PHY;
	uint64_t phy_pcs_base_regs = CAL_PCS;

	writel(0x28, phy_base_regs + 0xD8);
	mdelay(1);

	writel(0x00, phy_base_regs + 0x404);
	writel(0x00, phy_base_regs + 0x408);
	writel(0x00, phy_base_regs + 0x40C);

	writel(0x00, phy_base_regs + 0x800);
	writel(0x00, phy_base_regs + 0x804);
	writel(0x00, phy_base_regs + 0xA40);
	writel(0x00, phy_base_regs + 0xA44);
	writel(0x00, phy_base_regs + 0xA48);
	writel(0x00, phy_base_regs + 0xA4C);
	writel(0x00, phy_base_regs + 0xA50);

	writel(0x00, phy_base_regs + 0x1000);
	writel(0x00, phy_base_regs + 0x1004);
	writel(0x00, phy_base_regs + 0x1240);
	writel(0x00, phy_base_regs + 0x1244);
	writel(0x00, phy_base_regs + 0x1248);
	writel(0x00, phy_base_regs + 0x124C);
	writel(0x00, phy_base_regs + 0x1250);

	/* reset rate change and powerdown value */
	writel(0x0, phy_pcs_base_regs + 0x184);
}

/* Debug code added to check if PCIe PHY is being reset correctly. */
static const uint16_t cal_reset_off[6] = { 0x0010, 0x0014, 0x0040, 0x0044, 0x00D8, 0x1054 };
static const uint8_t cal_reset_val[6] = { 0x55, 0x51, 0x50, 0x0C, 0x28, 0x77 };

static void cal_pcie_phy_config(void)
{
	uint64_t elbi_base_regs = CAL_ELBI;
	uint64_t phy_base_regs = CAL_PHY;
	uint64_t phy_pcs_base_regs = CAL_PCS;
	int num_lanes = CAL_LANES;
	uint32_t val;
	uint32_t i;

	/* Debug code added to check if PCIe PHY is being reset correctly. */
	for (i = 0; i < 6; i++) {
		val = readl(phy_base_regs + cal_reset_off[i]);
		if (val != cal_reset_val[i]) {
			log_hex("phy reset value differs: off", cal_reset_off[i]);
			log_hex(" act", val);
		}
	}

	/* init. input clk path */
	writel(0x28, phy_base_regs + 0xD8);

	/* PCS MUX glitch W/A */
	val = readl(phy_pcs_base_regs + 0x008);
	val |= (1 << 7);
	writel(val, phy_pcs_base_regs + 0x008);
	if (num_lanes == 2) {
		val = readl(phy_pcs_base_regs + 0x808);
		val |= (1 << 7);
		writel(val, phy_pcs_base_regs + 0x808);
	}

	/* PHY CMN_RST, INIT_RST, PORT_RST Assert */
	writel(0x1, elbi_base_regs + 0x1404);
	writel(0x1, elbi_base_regs + 0x1408);
	writel(0x1, elbi_base_regs + 0x1400);
	writel(0x0, elbi_base_regs + 0x1404);
	writel(0x0, elbi_base_regs + 0x1408);
	writel(0x0, elbi_base_regs + 0x1400);
	udelay(10);

	writel(0x1, elbi_base_regs + 0x1404);
	udelay(10);

	/* pma_setting */
	/* Common */
	writel(0x50, phy_base_regs + 0x018);
	writel(0x33, phy_base_regs + 0x048);
	writel(0x01, phy_base_regs + 0x068);
	writel(0x12, phy_base_regs + 0x070);
	writel(0x00, phy_base_regs + 0x08C);
	writel(0x21, phy_base_regs + 0x090);
	writel(0x14, phy_base_regs + 0x0B0);
	writel(0x50, phy_base_regs + 0x0B8);
	writel(0x51, phy_base_regs + 0x0E0);
	writel(0x00, phy_base_regs + 0x100);
	writel(0x80, phy_base_regs + 0x104);
	writel(0x38, phy_base_regs + 0x140);
	writel(0xA4, phy_base_regs + 0x180);
	writel(0x03, phy_base_regs + 0x188); /* LCPLL 100MHz no divide */
	writel(0x38, phy_base_regs + 0x2A8);
	writel(0x12, phy_base_regs + 0x2E4);
	/* PMA enable and aggregation mode(bifurcation mode: 0xC0) */
	writel(0x80, phy_base_regs + 0x400);
	writel(0x20, phy_base_regs + 0x408);
	writel(0x00, phy_base_regs + 0x550);
	writel(0x00, phy_base_regs + 0x5A8);
	writel(0xFF, phy_base_regs + 0x5EC);

	/* PCIe_TYPE: RC */
	writel(0x02, phy_base_regs + 0x458); /* 100MHz CLK on */
	writel(0x34, phy_base_regs + 0x5B0); /* diff,control REFCLK source */
	writel(0x20, phy_base_regs + 0x450); /* when entering L1.2, add delay */

	/* PHY INPUT CLK 38.4 */
	writel(0x34, phy_base_regs + 0xAC);
	writel(0x34, phy_base_regs + 0xB4);
	writel(0x50, phy_base_regs + 0xE0);
	writel(0x00, phy_base_regs + 0xF4);
	writel(0x08, phy_base_regs + 0xF8);
	writel(0xA0, phy_base_regs + 0x104);
	writel(0x19, phy_base_regs + 0x11C);
	writel(0x17, phy_base_regs + 0x124);
	writel(0x41, phy_base_regs + 0x220);

	for (i = 0; i < (uint32_t)num_lanes; i++) {
		phy_base_regs += (i * 0x800);

		writel(0x08, phy_base_regs + 0x82C);
		writel(0x24, phy_base_regs + 0x830);
		writel(0x80, phy_base_regs + 0x878);
		writel(0x40, phy_base_regs + 0x894);
		writel(0x00, phy_base_regs + 0x8C0);
		writel(0x30, phy_base_regs + 0x8F4);
		writel(0x05, phy_base_regs + 0x908);
		writel(0xE0, phy_base_regs + 0x90C);
		writel(0xD4, phy_base_regs + 0x914);
		writel(0xD3, phy_base_regs + 0x91C);
		writel(0xCE, phy_base_regs + 0x920);
		writel(0x01, phy_base_regs + 0x924);
		writel(0x35, phy_base_regs + 0x928);
		writel(0xBA, phy_base_regs + 0x92C);
		writel(0x41, phy_base_regs + 0x930);
		writel(0x15, phy_base_regs + 0x934);
		writel(0x13, phy_base_regs + 0x938);
		writel(0x4E, phy_base_regs + 0x93C);
		writel(0x43, phy_base_regs + 0x948);
		writel(0xFC, phy_base_regs + 0x94C);
		writel(0x10, phy_base_regs + 0x954);
		writel(0x69, phy_base_regs + 0x958);
		writel(0x40, phy_base_regs + 0x964);
		writel(0xF6, phy_base_regs + 0x9B4);
		writel(0x2D, phy_base_regs + 0x9C0);
		writel(0xB7, phy_base_regs + 0x9C4);
		writel(0x3C, phy_base_regs + 0x9CC);
		writel(0x7E, phy_base_regs + 0x9DC);
		writel(0x02, phy_base_regs + 0xA40);
		writel(0x26, phy_base_regs + 0xA70);
		writel(0x00, phy_base_regs + 0xA74);
		writel(0x06, phy_base_regs + 0xB40);	/* hf_init */
		writel(0x06, phy_base_regs + 0xB44);
		writel(0x04, phy_base_regs + 0xB48);	/* value changed: 0x06 -> 0x04 */
		writel(0x03, phy_base_regs + 0xB4C);	/* value changed: 0x04 -> 0x03 */
		writel(0x03, phy_base_regs + 0xB50);	/* mf_init */
		writel(0x03, phy_base_regs + 0xB54);
		writel(0x03, phy_base_regs + 0xB58);
		writel(0x03, phy_base_regs + 0xB5C);
		writel(0x1B, phy_base_regs + 0xC10);
		writel(0x10, phy_base_regs + 0xC44);
		writel(0x10, phy_base_regs + 0xC48);
		writel(0x10, phy_base_regs + 0xC4C);
		writel(0x10, phy_base_regs + 0xC50);
		writel(0x02, phy_base_regs + 0xC54);
		writel(0x02, phy_base_regs + 0xC58);
		writel(0x02, phy_base_regs + 0xC5C);
		writel(0x02, phy_base_regs + 0xC60);
		writel(0x02, phy_base_regs + 0xC6C);
		writel(0x02, phy_base_regs + 0xC70);
		writel(0xE7, phy_base_regs + 0xCA8);
		writel(0x00, phy_base_regs + 0xCAC);
		writel(0x0E, phy_base_regs + 0xCB0);
		writel(0x1C, phy_base_regs + 0xCCC);
		writel(0x05, phy_base_regs + 0xCD4);
		writel(0x77, phy_base_regs + 0xCD8);
		writel(0x7A, phy_base_regs + 0xCDC);
		writel(0x2F, phy_base_regs + 0xDB4);

		/* RX tuning */
		writel(0x80, phy_base_regs + 0x8FC);
		writel(0xF4, phy_base_regs + 0x914);
		writel(0xD3, phy_base_regs + 0x91C);
		writel(0xCA, phy_base_regs + 0x920);
		writel(0x3D, phy_base_regs + 0x928);
		writel(0xB8, phy_base_regs + 0x92C);
		writel(0x41, phy_base_regs + 0x930);
		writel(0x17, phy_base_regs + 0x934);
		writel(0x4C, phy_base_regs + 0x93C);
		writel(0x73, phy_base_regs + 0x948);
		writel(0xFC, phy_base_regs + 0x94C);
		writel(0x55, phy_base_regs + 0x96C);
		writel(0x78, phy_base_regs + 0x988);
		writel(0x3B, phy_base_regs + 0x994);
		writel(0xF6, phy_base_regs + 0x9B4);
		writel(0xFF, phy_base_regs + 0x9C4);
		writel(0x20, phy_base_regs + 0x9C8);
		writel(0x2F, phy_base_regs + 0xA08);
		writel(0x3F, phy_base_regs + 0xB9C);

		/* for GEN4 TX termination resistor? */
		writel(0x00, phy_base_regs + 0x9CC);
		writel(0xFF, phy_base_regs + 0xCD8);
		writel(0x6E, phy_base_regs + 0xCDC);

		/* TX idrv for termination resistor */
		writel(0x0F, phy_base_regs + 0x82C);
		writel(0x60, phy_base_regs + 0x830);
		writel(0x7E, phy_base_regs + 0x834);

		/* Auto FBB */
		writel(0x00, phy_base_regs + 0xC08);
		writel(0x09, phy_base_regs + 0xC10);
		writel(0x04, phy_base_regs + 0xC40);
		writel(0x00, phy_base_regs + 0xC70);
		/* DFE */
		writel(0x76, phy_base_regs + 0x9B4);
	}

	phy_base_regs = CAL_PHY;
	for (i = 0; i < 2; i++) {
		phy_base_regs += (i * 0x800);

		/* PHY INPUT CLK 38.4 */
		writel(0x41, phy_base_regs + 0xBCC);
		writel(0x41, phy_base_regs + 0xBD4);
		writel(0x68, phy_base_regs + 0xBDC);
		writel(0x00, phy_base_regs + 0xBE0);
		writel(0xD0, phy_base_regs + 0xBE4);
	}

	/* PCS setting: pcie_pcs_setting() in F/W code */

	/* aggregation mdoe(bifurcation disabled) */
	writel(0x00, phy_pcs_base_regs + 0x004);
	if (num_lanes == 2)
		writel(0x00, phy_pcs_base_regs + 0x804);

	/* if RC */
	writel(0x700D5, phy_pcs_base_regs + 0x154);
	if (num_lanes == 2)
		writel(0x700D5, phy_pcs_base_regs + 0x954);

	/* add for L2 entry and power */
	writel(0x300FF, phy_pcs_base_regs + 0x150);
	if (num_lanes == 2)
		writel(0x300FF, phy_pcs_base_regs + 0x950);

	/* add L2 power down delay */
	writel(0x40, phy_pcs_base_regs + 0x170);
	if (num_lanes == 2)
		writel(0x40, phy_pcs_base_regs + 0x970);

	/* when entring L1.2, ERIO CLK gating */
	val = readl(phy_pcs_base_regs + 0x008);
	val &= ~((1 << 4) | (1 << 5));
	val |= ((1 << 4) | (1 << 5));
	writel(val, phy_pcs_base_regs + 0x008);
	if (num_lanes == 2) {
		val = readl(phy_pcs_base_regs + 0x808);
		val &= ~((1 << 4) | (1 << 5));
		val |= ((1 << 4) | (1 << 5));
		writel(val, phy_pcs_base_regs + 0x808);
	}

	/* PLL & BIAS EN off delay	*/
	writel(0x100B0808, phy_pcs_base_regs + 0x190);

	/* PHY CMN_RST, PORT_RST Release */
	writel(0x1, elbi_base_regs + 0x1400);
	writel(0x1, elbi_base_regs + 0x1408);

	/* Additional PMA Configurations */
	phy_base_regs = CAL_PHY;
	val = readl(phy_base_regs + 0x5D0);
	val |= (0x1 << 4);
	val &= ~(0x1 << 3);
	writel(val, phy_base_regs + 0x5D0);
	log_hex("cal: XO clock configuration phy+0x5D0", readl(phy_base_regs + 0x5D0));

	/* AFC cal mode by default uses the calibrated value from a previous
	 * run. However on some devices this causes a CDR failure because
	 * the AFC done status is set prematurely. Setting the cal mode to
	 * always start from an initial value (determined through simulation)
	 * ensures that AFC has enough time to complete.
	 */
	writel(0x4, phy_base_regs + 0xBF4);

	/* (the UDBG internal dump settings of the original follow here: left out) */
}
