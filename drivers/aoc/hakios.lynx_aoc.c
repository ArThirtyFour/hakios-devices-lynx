/*
 * The Pixel 7a's AoC ("always-on compute"), Google's audio and sensor DSP on gs201: every on-device audio path ends
 * there -- the two CS35L41 speaker amplifiers' data comes from its TDM_0 port, the three PDM microphones feed it, and
 * the sensor hub (accelerometer, light, proximity, ...) is behind it. This driver cold-starts it and feeds its
 * audio_playbackN rings for the speakers; drivers/audio/hakios.lynx_amp.c is the amplifiers' control side.
 *
 * Unlike the Pixel 8a (zuma), the bootloader does NOT leave the AoC running on gs201: stock's kernel loads and starts
 * it about 4 s into boot ("aoc 19000000.aoc: attempting to load firmware aoc.bin", then "did become online with 82
 * services"). So the full load-authenticate-start path is needed (Android's drivers/soc/google/aoc/aoc.c aoc_fw_callback,
 * and the reference notes in the omarchy cheetah/gs201 port, AOC-PLAN.md -- same SoC, same sequence):
 *
 *   1. request_aoc_on: write 1 to aoc_req (0x181d0000); the acknowledge is aoc_req + 0x40 (aoc_v1.c).
 *   2. Open the AoC's S2MPU (0x1a0b0000) and the GSA's (0x17c60000): CTRL0 = 0 passes everything. The GSA reads the
 *      image header by DMA, so its S2MPU must be open for the load, under a GSA wakelock (cmd 150/151). Stock's pKVM
 *      programs both; nothing does here, so this driver does. Only CTRL0 is touched -- other S2MPU registers can reset
 *      the SoC (s2mpu-regs.h).
 *   3. Install the AoC's System MMU (sysmmu_asoc, 0x1a090000, SysMMU v8.1) windows. The firmware addresses its carve-out
 *      at 0x98000000, which is physically the GXP region, so translation is REQUIRED -- the firmware's own image header
 *      lists six 1 MiB windows (AoC address -> physical): 0x98000000->0x94000000 (48 MiB carve-out), 0x9b000000->
 *      0x97000000 (4 MiB xhci-dma), 0x9e000000->0x18200000 (2 MiB mailboxes), 0x9e200000->0x17c00000 (1 MiB GSA),
 *      0x9e500000->0x11200000 (1 MiB USB PHY), 0x9e300000->0x40000000 (1 MiB PCIe outbound). All 1 MiB multiples, so a
 *      first-level-only table of sections covers them (samsung-iommu.c, VID 0).
 *   4. Copy the image body (aoc.bin without its 4 KiB auth header) into the carve-out at 0x94000000.
 *   5. GSA_MB_CMD_LOAD_AOC_FW_IMG (90) over the GSA mailbox (0x17c90000): the auth header in a coherent page of ours,
 *      the body's physical address; the GSA authenticates and then locks the body (the AP must not touch it again).
 *   6. Write the firmware's parameters at carve-out + image_size (the IPC area): magic 0x0a0cda7a, then key/size/value
 *      triples (board id/rev, carve-out, the three heaps, chip, a seed).
 *   7. GSA_AOC_START. On gs201 the bare mailbox command (91 START) is NOT enough -- the GSA accepts it but the A32
 *      faults silently on the secure side (state 4); stock sends START through Trusty's hwmgr.aoc service, which does
 *      secure-side setup first. So START goes through lynx_trusty_aoc_cmd() (weak: the Trusty client is not ported to
 *      lynx yet -- see the report), with the mailbox command 91 as a documented fallback.
 *   8. Wait for the firmware to write AOC_MAGIC (0xa0c00a0c) into its control block at carve-out + image_size.
 *
 * The GSA mailbox (gsa_mbox.c): 16 shared registers SR0-15 at +0x80; a request writes the command into SR0, the
 * argument count into SR1 and the arguments after; the response interrupt (bit 0) is cleared (INTCR0 +0x24), unmasked
 * (INTMR0 +0x28), the request raised (INTGR1 +0x40, bit 0), and the GSA answers in SR0 (command | bit 31), SR1 (error),
 * SR2 (argument count) and the arguments after, the response bit then set in INTSR0 (+0x2c). Polled here.
 *
 * The AoC's IPC: its control block holds a table of services (struct aoc_ipc_service_header: name[32], flags, and two
 * regions of offset, size, slots, tx, rx, wp, rp). A message is written at a region's write pointer and the AoC rung
 * with a doorbell: the service's flags bits 19-23 name one of the 48 mailbox channels, reached through the three
 * whitechapel mailboxes (0x18300000 A32, 0x18320000 F1, 0x18340000 P6; 16 channels each; INTGR0 +0x20, INTSR0 +0x2c).
 *
 *   aoc=1   a look: aoc_req and the AoC's state, the GSA mailbox's registers, one harmless request
 *           (GSA_MB_CMD_GET_CHIP_ID, 3) and the AoC state command (91 GET_STATE) -- can this kernel talk to the GSA at
 *           all. Nothing switched on, nothing written but the mailbox.
 *   aoc=2   ... and one word 6 MiB into the carve-out, where a running firmware keeps its control block by default --
 *           whether the AoC is already up (it is not, after a cold fastboot boot).
 *   aoc=3   THE AOC STARTED, as Android does: aoc_req, S2MPUs, SysMMU windows, the image (fw/aoc.bin, or
 *           /private/var/hakios/fw/aoc.bin left there by userland/src/init.c) checked and copied, the GSA load (90),
 *           the parameters, the start through Trusty (lynx_trusty_aoc_cmd) and the control block awaited.
 *
 * The sound (aocsound=1, with aoc=3 and amp=2): what the audio server mixes (userland/src/audio_plugin.c ->
 * /dev/hakios-audio, drivers/audio/hakios.audio.c) is offered to this driver as frames; a sink keeps them in a ring of
 * ours, a feeder thread keeps audio_playbackN's ring (80 ms) three 20 ms blocks full through the speakers' guard
 * (drivers/audio/hakios.guard.c, in place of the DSP's protection), so the AoC's stream never stops; the amplifiers are
 * powered up when a write is not silence and down after three seconds of silence. aocplay=N plays a test tone instead.
 *
 * Sources: Google's kernel/gs drivers/soc/google/aoc/{aoc,aoc_v1,gsa_mbox}.c and drivers/iommu/samsung-iommu.c
 * (android-gs-lynx-5.10-android15), and the Pixel 8a's hakios.akita_aoc.c, whose structure this follows.
 */
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <kern/thread.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <sys/hakios.audio.h>
#include <kern/kalloc.h>
#include <sys/vnode.h>
#include <sys/vnode_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, integer_t priority,
    thread_t *new_thread);
/* Every lynx driver logs through the touch driver's lynx_tlog (printf and the ramoops console the loader reads). */
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

/*
 * Trusty's hwmgr.aoc service is how gs201 starts the AoC (the bare mailbox START is not enough). The client is the Pixel
 * 8a's hakios.akita_trusty.c, not ported to lynx yet: this weak reference stays 0 until one is built, so the driver runs
 * its read-only probes (aoc=1/aoc=2) but aoc=3 stops at the start. Returns < 0 when not reachable, > 0 the GSA's error;
 * *state the AoC's state after (0 inactive, 1 loaded, 2 running).
 */
int lynx_trusty_aoc_cmd(uint32_t cmd, uint32_t *state) __attribute__((weak));

/*
 * ... and a weak definition of it, so that a kernel built without the Trusty client still links (a weak undefined
 * symbol is not enough for this kernel's link): it answers -2, "there is no client", which the start treats as
 * "cannot go further" rather than as the GSA's error.
 */
__attribute__((weak)) int
lynx_trusty_aoc_cmd(uint32_t cmd, uint32_t *state)
{
	(void)cmd;
	(void)state;
	return -2;
}

/* drivers/audio (amp=2): the CS35L41s */
int lynx_amp_speaker(int on) __attribute__((weak));
int lynx_amp_variant(int k) __attribute__((weak));

#define AOC_REQ_PHYS            0x181d0000ULL   /* aoc_req: +0 the request, +0x40 the AoC's acknowledge */
#define AOC_DRAM_PHYS           0x94000000ULL   /* the AoC's carve-out (aoc@94000000), 48 MiB, no-map */
#define AOC_DRAM_SIZE           (48u << 20)
#define AOC_DRAM_AOC            0x98000000ULL   /* the carve-out as the AoC sees it (its SysMMU translates) */
#define AOC_MAGIC               0xA0C00A0Cu
#define GSA_MBOX_PHYS           0x17c90000ULL   /* gsa-ns */
#define GSA_S2MPU_PHYS          0x17c60000ULL   /* the GSA's S2MPU */
#define AOC_S2MPU_PHYS          0x1a0b0000ULL   /* the AoC's S2MPU */
#define AOC_SYSMMU_PHYS         0x1a090000ULL   /* sysmmu_asoc, SysMMU v8.1 */
#define AOC_DOORBELL(n)         (0x18300000ULL + 0x20000ULL * (n))   /* A32, F1, P6 */
#define CHIPID_PHYS             0x10000000ULL

/* the GSA mailbox */
#define MBOX_INTCR0             0x0024
#define MBOX_INTMR0             0x0028
#define MBOX_INTSR0             0x002c
#define MBOX_INTMSR0            0x0030
#define MBOX_INTGR1             0x0040
#define MBOX_SR(n)              (0x0080 + (n) * 4)
#define MBOX_SR_NUM             16

#define GSA_MB_CMD_RSP          (1u << 31)
#define GSA_MB_CMD_GET_CHIP_ID  3
#define GSA_MB_CMD_LOAD_AOC_FW_IMG 90
#define GSA_MB_CMD_AOC_CMD      91      /* state commands (also sent through Trusty on stock) */
#define GSA_MB_CMD_UNLOAD_AOC_FW_IMG 92
#define GSA_MB_CMD_WAKELOCK_ACQUIRE 150
#define GSA_MB_CMD_WAKELOCK_RELEASE 151
#define GSA_AOC_GET_STATE       0
#define GSA_AOC_START           1
#define GSA_AOC_SHUTDOWN        4

/* the AoC's System MMU, SysMMU v8.1 (samsung-iommu.c): the VM register set, VID 0 */
#define SYSMMU_CTRL             0x0000
#define SYSMMU_CFG              0x0004
#define SYSMMU_VM_CTRL          0x8000
#define SYSMMU_VM_FLPT_BASE     0x800c
#define SYSMMU_VM_ALL_INV       0x8010
#define SYSMMU_CTRL_ENABLE      (1u << 0)
#define SYSMMU_CTRL_BLOCK       (1u << 1)
#define SYSMMU_CTRL_INT         (1u << 2)
#define SYSMMU_VID_ENABLE       (1u << 0)
#define SYSMMU_VID_STALL        (1u << 3)
#define SYSMMU_SECT             2               /* a first-level section (1 MiB) descriptor */
#define SYSMMU_SHAREABLE        (1u << 6)
#define SYSMMU_LV1_ENTRIES      65536           /* 64 GiB of 1 MiB sections */
#define SYSMMU_LV1_SIZE         (SYSMMU_LV1_ENTRIES * 4)

#define AOC_AUTH_HEADER_SIZE    4096u
#define AOC_PARAMETER_MAGIC     0x0a0cda7au

static vm_offset_t aoc_req, gsa, gsa_s2mpu, aoc_s2mpu, sysmmu;
static uint32_t mode;

static inline uint32_t
rd(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(base + off) = v;
}

/* one GSA request, polled (no interrupt): 0 and the response's words, or -1 when the GSA did not answer */
static int gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms);

static int
gsa_request(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err)
{
	return gsa_request_ms(cmd, args, argc, rsp, rsp_n, err, 500);
}

static int
gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms)
{
	if (argc + 2 > MBOX_SR_NUM) {
		return -1;
	}
	wr(gsa, MBOX_SR(0), cmd);
	wr(gsa, MBOX_SR(1), argc);
	for (uint32_t i = 0; i < argc; i++) {
		wr(gsa, MBOX_SR(i + 2), args[i]);
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) & ~1u);
	__builtin_arm_dsb(0xf);
	wr(gsa, MBOX_INTGR1, 1u);
	int answered = 0;
	for (int ms = 0; ms < timeout_ms; ms++) {
		if (rd(gsa, MBOX_INTSR0) & 1u) {
			answered = 1;
			break;
		}
		delay(1000);
	}
	uint32_t got = rd(gsa, MBOX_SR(0));
	*err = rd(gsa, MBOX_SR(1));
	uint32_t n = rd(gsa, MBOX_SR(2));
	*rsp_n = 0;
	if (answered && got == (cmd | GSA_MB_CMD_RSP) && n + 3 <= MBOX_SR_NUM) {
		for (uint32_t i = 0; i < n; i++) {
			rsp[i] = rd(gsa, MBOX_SR(i + 3));
		}
		*rsp_n = n;
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) | 1u);
	lynx_tlog("lynx aoc: GSA request %u: %s (SR0 0x%x, error %u, %u words)\n", cmd, answered ? "answered" : "NO ANSWER",
	    got, *err, n);
	return answered && got == (cmd | GSA_MB_CMD_RSP) ? 0 : -1;
}

/* the GSA's S2MPU held open (CTRL0 = 0) for one data transfer, under its wakelock */
static bool gsa_open;
static bool
gsa_s2mpu_open(bool open)
{
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;
	uint32_t before = rd(gsa_s2mpu, 0);

	if (open == gsa_open) {
		return true;
	}
	if (open && gsa_request(GSA_MB_CMD_WAKELOCK_ACQUIRE, NULL, 0, rsp, &n, &err) != 0) {
		lynx_tlog("lynx aoc: the GSA's wakelock not taken; its S2MPU left as it is (CTRL0 0x%x)\n", before);
		return false;
	}
	wr(gsa_s2mpu, 0, open ? 0 : 1);        /* 0 passes everything; bit 0 set blocks */
	__builtin_arm_dsb(0xf);
	if (!open) {
		(void)gsa_request(GSA_MB_CMD_WAKELOCK_RELEASE, NULL, 0, rsp, &n, &err);
	}
	gsa_open = open;
	lynx_tlog("lynx aoc: the GSA's S2MPU CTRL0 0x%x -> 0x%x (%s)\n", before, open ? 0 : 1, open ? "opened" : "closed");
	return true;
}

/* the AoC's S2MPU: CTRL0 = 0 while BLK_AOC is on (the bootloader may leave it blocking, 0x5) */
static void
aoc_s2mpu_open(void)
{
	uint32_t before = rd(aoc_s2mpu, 0);

	if (before != 0) {
		wr(aoc_s2mpu, 0, 0);
		__builtin_arm_dsb(0xf);
		lynx_tlog("lynx aoc: the AoC's S2MPU CTRL0 0x%x -> 0 (opened)\n", before);
	}
}

/*
 * The carve-out mapped once, all 48 MiB (write-combined) with pmap_map, not ml_io_map -- its whole io area is 8 MiB.
 * Reads are fine only before the GSA locks the body; after the load only the IPC area (past image_size) is ours.
 */
static vm_offset_t
carveout_map(void)
{
	static vm_offset_t va;

	if (va == 0) {
		vm_offset_t v = 0;

		if (kmem_alloc(kernel_map, &v, AOC_DRAM_SIZE, KMA_VAONLY | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			return 0;
		}
		pmap_map(v, (vm_offset_t)AOC_DRAM_PHYS, (vm_offset_t)(AOC_DRAM_PHYS + AOC_DRAM_SIZE), VM_PROT_READ | VM_PROT_WRITE,
		    VM_WIMG_WCOMB);
		va = v;
	}
	return va;
}

/* ---- the AoC's System MMU: six windows from the firmware's image header, VID 0 -------------------------------- */

/* a 1 MiB section descriptor: the physical base, the section type, and whether the memory is shareable */
static inline uint32_t
sysmmu_section(uint64_t pa, bool shareable)
{
	return (uint32_t)(pa >> 4) | SYSMMU_SECT | (shareable ? SYSMMU_SHAREABLE : 0);
}

static void
sysmmu_install(const uint32_t *windows, uint32_t count)
{
	vm_offset_t table = 0;

	if (kmem_alloc_contig(kernel_map, &table, SYSMMU_LV1_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lynx_tlog("lynx aoc: no 256 KiB for the System MMU's first-level table\n");
		return;
	}
	for (uint32_t i = 0; i < count; i++) {
		uint64_t aoc_va = (uint64_t)windows[3 * i], pa = (uint64_t)windows[3 * i + 1] | (uint64_t)windows[3 * i + 2] << 32;
		uint32_t idx = (uint32_t)(aoc_va >> 20);

		/* the carve-out is shareable (the AoC and the AP share it); the MMIO windows are device memory */
		*(volatile uint32_t *)(table + idx * 4) = sysmmu_section(pa, aoc_va == AOC_DRAM_AOC);
	}
	/* the table cleaned to memory: the System MMU reads it through its own bus master */
	for (vm_offset_t line = table; line < table + SYSMMU_LV1_SIZE; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__builtin_arm_dsb(0xf);

	/* the bootloader may leave the MMU off: block it while the table goes in, then enable VID 0 and the MMU */
	uint32_t ctrl = rd(sysmmu, SYSMMU_CTRL);
	wr(sysmmu, SYSMMU_CTRL, ctrl | SYSMMU_CTRL_BLOCK);
	wr(sysmmu, SYSMMU_VM_FLPT_BASE, (uint32_t)(kvtophys(table) >> 12));
	wr(sysmmu, SYSMMU_VM_ALL_INV, 1);
	wr(sysmmu, SYSMMU_VM_CTRL, SYSMMU_VID_STALL | SYSMMU_VID_ENABLE);  /* stall on fault, VID 0 on */
	wr(sysmmu, SYSMMU_CTRL, ctrl | SYSMMU_CTRL_ENABLE | SYSMMU_CTRL_INT);  /* enable, faults to the AP */
	__builtin_arm_dsb(0xf);
	lynx_tlog("lynx aoc: the AoC's System MMU: %u windows at VID 0 (table 0x%llx); CTRL 0x%x -> 0x%x\n", count,
	    kvtophys(table), ctrl, rd(sysmmu, SYSMMU_CTRL));
}

/* ---- the AoC's IPC: services, their queues, the doorbells (aoc_ipc_core.c, mailbox-wc.c) --------------------- */

static vm_offset_t ipc_base;                    /* the IPC block (carve-out + the image's size), mapped */
static uint32_t ipc_room;                       /* the bytes mapped from ipc_base on (the carve-out's rest) */
static vm_offset_t aoc_mbox[3];                 /* the AP->AoC doorbells: 0x18300000/18320000/18340000, 16 channels each */
#define AOC_UP                          0       /* the AoC to us */
#define AOC_DOWN                        1       /* us to the AoC */
#define MB_INTGR0                       0x20
#define MB_INTSR0                       0x2c

/* service NAME's header (struct aoc_ipc_service_header: name[32], flags, two regions of offset, size, slots, tx, rx,
 * wp, rp), or 0 */
static vm_offset_t
aoc_service(const char *name)
{
	uint32_t count = rd(ipc_base, 24), each = rd(ipc_base, 28), at = rd(ipc_base, 32);

	for (uint32_t k = 0; k < count && each >= 92; k++) {
		vm_offset_t h = ipc_base + at + k * each;
		char nm[33];

		for (int c = 0; c < 32; c++) {
			nm[c] = (char)*(volatile uint8_t *)(h + c);
		}
		nm[32] = 0;
		if (strcmp(nm, name) == 0) {
			return h;
		}
	}
	return 0;
}

static inline vm_offset_t
region(vm_offset_t svc, int dir)
{
	return svc + 36 + 28 * dir;     /* offset 0, size 4, slots 8, tx 12, rx 16, wp 20, rp 24 */
}

/* the AoC rung for SVC's channel (flags bits 19-23: the mailbox interrupt, 16 a mailbox) */
static void
aoc_signal(vm_offset_t svc)
{
	uint32_t irq = (rd(svc, 32) >> 19) & 0x1f;
	vm_offset_t mb = aoc_mbox[irq / 16];

	if (!mb) {
		return;
	}
	for (int i = 0; i < 100 && (rd(mb, MB_INTSR0) & (1u << (irq % 16))); i++) {
		delay(100);                     /* (the AoC not done with the last one yet) */
	}
	wr(mb, MB_INTGR0, 1u << (irq % 16));
}

/* a message onto SVC's queue towards the AoC: false when it is full or too big */
static bool
aoc_write(vm_offset_t svc, const void *msg, uint32_t len)
{
	vm_offset_t r = region(svc, AOC_DOWN);
	uint32_t off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12), rx = rd(r, 16);

	if (slots == 0 || len + 8 > size || tx - rx >= slots) {
		return false;
	}
	vm_offset_t slot = ipc_base + off + size * (tx % slots);
	*(volatile uint32_t *)slot = len;                       /* struct aoc_ipc_message_header: length, 3 reserved */
	*(volatile uint32_t *)(slot + 4) = 0;
	for (uint32_t i = 0; i < len; i++) {
		*(volatile uint8_t *)(slot + 8 + i) = ((const uint8_t *)msg)[i];
	}
	__builtin_arm_dsb(0xf);
	wr(r, 12, tx + 1);
	wr(r, 20, (rd(r, 20) + 1) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
	return true;
}

/* the next message from the AoC on SVC (polled, up to MS ms): its length, or -1 */
static int
aoc_read(vm_offset_t svc, void *buf, uint32_t cap, int ms)
{
	vm_offset_t r = region(svc, AOC_UP);

	for (int i = 0; i <= ms * 10; i++) {
		uint32_t off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12), rx = rd(r, 16);

		if (slots != 0 && tx != rx) {
			vm_offset_t slot = ipc_base + off + size * (rx % slots);
			uint32_t len = rd(slot, 0) & 0xffff;
			uint32_t n = len < cap ? len : cap;

			for (uint32_t k = 0; k < n; k++) {
				((uint8_t *)buf)[k] = *(volatile uint8_t *)(slot + 8 + k);
			}
			wr(r, 16, rx + 1);
			wr(r, 24, (rd(r, 24) + 1) % size);
			return (int)len;
		}
		delay(100);
	}
	return -1;
}

/* an audio command round trip on SERVICE (aoc_alsa_hw.c's aoc_audio_control): old replies dropped, the command, its reply */
static int
aoc_audio_cmd(const char *service, void *cmd, uint32_t len)
{
	static uint8_t cntr;
	vm_offset_t svc = aoc_service(service);
	uint8_t junk[64];

	if (!svc) {
		lynx_tlog("lynx aoc: no service %s\n", service);
		return -1;
	}
	while (aoc_read(svc, junk, sizeof(junk), 0) >= 0) {
	}
	uint8_t *c = cmd;
	c[0] = 0;                                       /* struct CONTAINER_HDR: DATA_TYPE_CMD, a counter, the length */
	c[1] = cntr++;
	*(uint16_t *)(c + 2) = (uint16_t)len;
	if (!aoc_write(svc, cmd, len)) {
		lynx_tlog("lynx aoc: %s: its queue is full\n", service);
		return -1;
	}
	uint16_t id = *(uint16_t *)(c + 4);
	uint8_t reply[128];
	int got = aoc_read(svc, reply, sizeof(reply), 1000);
	if (got < 0) {
		lynx_tlog("lynx aoc: %s: no reply to command %u in 1 s\n", service, id);
	} else if (got == 4) {                          /* (aoc_audio_control: a 4-byte reply is an error code) */
		lynx_tlog("lynx aoc: %s: command %u: error 0x%x\n", service, id, *(uint32_t *)reply);
		return -1;
	} else {
		memcpy(cmd, reply, (uint32_t)got < len ? (uint32_t)got : len);
	}
	return got;
}

static bool
aoc_out_cmd(const char *what, void *cmd, uint32_t len, uint16_t id)
{
	*(uint16_t *)((uint8_t *)cmd + 4) = id;
	int got = aoc_audio_cmd("audio_output_control", cmd, len);
	lynx_tlog("lynx aoc: %s (%u): %s, reply field %d\n", what, id, got < 0 ? "FAILED" : "done", got < 0 ? 0 :
	    *(int16_t *)((uint8_t *)cmd + 6));
	return got >= 0;
}

/* ---- a sound through the AoC (aocplay=SECONDS with aoc=3): Google's ALSA driver's playback, by hand ---------------
 * prepare: CMD_AUDIO_OUTPUT_EP_SETUP (206) and EP_SETUP2 (298) for entry point 0 (audio_playback0); the route:
 * CMD_AUDIO_OUTPUT_BIND2 (271) to SINK_SPEAKER (TDM_0_RX: the two CS35L41s); start: CMD_AUDIO_OUTPUT_SOURCE (201) on;
 * then 48 kHz 16-bit stereo into audio_playback0's ring (a ring service: bytes at wp, tx counts them; the AoC moves rx). */

static uint32_t aocplay;
static uint32_t aocplay_amp = 2000;             /* aocplay_amp=N: the tone's peak (2000: about -24 dBFS) */
static uint32_t aocplay_lr;                     /* aocplay_lr=1: the tone left/right by turns */
static uint32_t aocsink = ~0u;                  /* aocsink=M: the speaker sink's processing before each stream (0 idle, 1 active, 2 bypass) */
static uint32_t aocep = 1;                      /* aocep=N: the entry point played into (audio_playbackN); 1 = low-latency */
static uint32_t aocplay_matrix;                 /* aocplay_matrix=1: a listening test over amplifier settings K (lynx_amp_variant) */
#define MATRIX_SEG      (6u * 48000u)           /* frames for each setting */
#define MATRIX_N        6u

/* the tone's on/off in the listening test: K beeps of 0.2 s from 0.5 s and again from 3.5 s into setting K's six seconds */
static bool
aoc_tone_on(uint64_t frame)
{
	if (!aocplay_matrix) {
		return true;
	}
	uint32_t k = (uint32_t)(frame / MATRIX_SEG) + 1, f = (uint32_t)(frame % MATRIX_SEG);
	if (k > MATRIX_N) {
		return false;
	}
	for (uint32_t g = 0; g < 2; g++) {
		uint32_t start = 24000 + g * 144000;
		if (f >= start && f < start + k * 19200) {
			return (f - start) % 19200 < 9600;
		}
	}
	return false;
}

/* the entry point's volume at a sink (aoc_audio_volume_set: CMD_AUDIO_OUTPUT_SET_PARAMETER 209, packed -- header, block
 * 16 + sink, component 0, key the source, value 0-1000). Google's driver sends chip->volume, 1000, for every stream. */
static bool
aoc_volume(uint8_t source, uint8_t sink)
{
	uint32_t value = 1000, key = source;
	uint8_t p[18] = { 0 };

	PE_parse_boot_argn("aocvol", &value, sizeof(value));
	if (value > 1000) {
		value = 1000;
	}
	p[8] = (uint8_t)(16 + sink);                    /* AOC_AUDIO_SINK_BLOCK_ID_BASE + sink */
	p[9] = 0;                                       /* component */
	memcpy(p + 10, &key, 4);
	memcpy(p + 14, &value, 4);
	lynx_tlog("lynx aoc: volume %u of 1000 for entry point %u at sink %u\n", value, source, sink);
	return aoc_out_cmd("SET_PARAMETER (volume)", p, sizeof(p), 209);
}

/* the speaker sink's processing mode (aocsink=M), before a stream's EP_SETUP */
static void
aoc_sink_mode(void)
{
	if (aocsink <= 2) {
		uint8_t sk[11] = { 0 };                 /* struct CMD_AUDIO_OUTPUT_SINK2: sink, mode, timestamping */
		sk[8] = 0;                              /* SINK_SPEAKER */
		sk[9] = (uint8_t)aocsink;
		aoc_out_cmd(aocsink == 2 ? "SINK2 speaker BYPASS" : aocsink == 1 ? "SINK2 speaker active" : "SINK2 speaker idle", sk,
		    sizeof(sk), 270);
	}
}

/* a 257-entry quarter sine, interpolated, 32-bit phase */
static const int16_t sine_q[257] = {
	0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012,
	3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195,
	6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319,
	9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
	12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
	15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
	18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
	20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
	23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
	25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
	27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
	28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
	30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
	31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
	32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
	32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766,
	32767,
};

static uint32_t aocplay_hz = 440;

static int32_t
tone_sample(uint64_t frame)
{
	uint32_t inc = (uint32_t)(((uint64_t)aocplay_hz << 32) / 48000u);
	uint32_t phase = (uint32_t)(frame * (uint64_t)inc), u = phase & 0x3fffffffu, q = phase >> 30;

	if (q & 1) {
		u = 0x40000000u - u;
	}
	uint32_t idx = u >> 22, frac = (u >> 6) & 0xffffu;
	int32_t a = sine_q[idx], b = sine_q[idx < 256 ? idx + 1 : 256];
	int32_t v = a + (int32_t)(((int64_t)(b - a) * frac) >> 16);
	return q >= 2 ? -v : v;
}

/* the tone, frame FRAME onwards, BYTES of it at ring offset AT (wrapping): 16-bit stereo */
static void
aoc_tone(vm_offset_t ring, uint32_t size, uint32_t at, uint32_t bytes, uint64_t *frame)
{
	for (uint32_t i = 0; i + 4 <= bytes; i += 4) {
		int32_t v = aoc_tone_on(*frame) ? (int32_t)(((int64_t)tone_sample(*frame) * aocplay_amp) / 32767) : 0;
		uint16_t l = (uint16_t)(int16_t)v, r = l;
		if (aocplay_lr) {
			if ((*frame / 48000) % 2 == 0) {
				r = 0;
			} else {
				l = 0;
			}
		}
		uint32_t o = (at + i) % size;
		*(volatile uint32_t *)(ring + o) = l | ((uint32_t)r << 16);
		(*frame)++;
	}
}

static void
aoc_play(void)
{
	char svcname[24];

	snprintf(svcname, sizeof(svcname), "audio_playback%u", aocep);
	vm_offset_t svc = aoc_service(svcname);

	if (!svc) {
		lynx_tlog("lynx aoc: no %s\n", svcname);
		return;
	}
	vm_offset_t r = region(svc, AOC_DOWN);
	uint32_t size = rd(r, 4), off = rd(r, 0);
	lynx_tlog("lynx aoc: %s: flags 0x%x (type %u, mailbox %u), down: %u bytes at +0x%x, slots %u, tx %u rx %u wp %u rp %u\n",
	    svcname, rd(svc, 32), (rd(svc, 32) >> 16) & 7, (rd(svc, 32) >> 19) & 0x1f, size, off, rd(r, 8), rd(r, 12), rd(r, 16),
	    rd(r, 20), rd(r, 24));
	if (size < 7680 || ((rd(svc, 32) >> 16) & 7) != 1 || (uint64_t)off + size > ipc_room) {   /* (a ring: bytes, tx counts) */
		lynx_tlog("lynx aoc: %s is not the ring expected; no sound\n", svcname);
		return;
	}
	uint32_t period = 3840, buffer = (size / period) * period;  /* 20 ms periods */

	/* the ring's write pointer to its start (aoc_ring_reset_write_pointer) */
	if (rd(r, 20) != 0) {
		uint32_t n = size - rd(r, 20);
		wr(r, 12, rd(r, 12) + n);
		wr(r, 20, 0);
	}

	uint8_t ep[28] = { 0 };                         /* struct CMD_AUDIO_OUTPUT_EP_SETUP: metadata -- chan, bits, sr, format, offset */
	*(uint32_t *)(ep + 12) = 48000;                 /* watermark */
	ep[20] = 2;                                     /* CH_STEREO */
	ep[21] = 1;                                     /* WIDTH_16_BIT */
	ep[22] = 5;                                     /* SR_48KHZ */
	ep[23] = 1;                                     /* FRMT_FIXED_POINT */
	ep[25] = (uint8_t)aocep;                        /* the entry point */
	ep[26] = 1;                                     /* wraparound */
	ep[27] = 0;                                     /* ENTRYPOINT_MODE_PLAYBACK */
	uint8_t ep2[18] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_EP_SETUP2 */
	ep2[8] = (uint8_t)aocep;
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = period;
	ep2[17] = 0;
	uint8_t bind[11] = { 0 };                       /* struct CMD_AUDIO_OUTPUT_BIND: src, dst, bind */
	bind[8] = (uint8_t)aocep;
	bind[9] = 0;                                    /* SINK_SPEAKER */
	bind[10] = 1;
	uint8_t src[10] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_SOURCE: source, on */
	src[8] = (uint8_t)aocep;
	src[9] = 1;

	aoc_sink_mode();
	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return;
	}
	(void)aoc_volume((uint8_t)aocep, 0);

	/* the ring filled half before the source goes on, then kept half full for AOCPLAY seconds */
	vm_offset_t ring = ipc_base + off;
	uint64_t frame = 0;
	uint32_t ahead = buffer - period;               /* (three periods of four) */
	aoc_tone(ring, size, rd(r, 20), ahead, &frame);
	__builtin_arm_dsb(0xf);
	wr(r, 12, rd(r, 12) + ahead);
	wr(r, 20, (rd(r, 20) + ahead) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
	if (!aoc_out_cmd("SOURCE on", src, sizeof(src), 201)) {
		return;
	}
	aoc_sink_mode();

	uint32_t rx0 = rd(r, 16);
	uint64_t t0 = mach_absolute_time(), ns;
	bool amps = false;
	uint32_t seg = ~0u;
	for (uint32_t ms = 0; ms < aocplay * 1000; ms += 10) {
		delay(10000);
		uint32_t tx = rd(r, 12), rx = rd(r, 16);
		uint32_t queued = tx - rx;
		if (queued < ahead) {
			uint32_t n = ((ahead - queued) / 4) * 4;
			aoc_tone(ring, size, rd(r, 20), n, &frame);
			__builtin_arm_dsb(0xf);
			wr(r, 12, tx + n);
			wr(r, 20, (rd(r, 20) + n) % size);
			__builtin_arm_dsb(0xf);
			aoc_signal(svc);
		}
		if (ms == 200 && rd(r, 16) != rx0 && lynx_amp_speaker) {  /* the AoC reading: its TDM runs, the amps' PLL can lock */
			amps = (aocplay_matrix && lynx_amp_variant ? lynx_amp_variant(1) : lynx_amp_speaker(1)) == 0;
			seg = 0;
		}
		if (aocplay_matrix && lynx_amp_variant && seg != ~0u) {
			uint64_t now = frame - (uint64_t)((rd(r, 12) - rd(r, 16)) / 4);
			uint32_t s = (uint32_t)(now / MATRIX_SEG);
			if (s != seg && s < MATRIX_N) {
				seg = s;
				amps = lynx_amp_variant((int)s + 1) == 0;
			}
		}
		if (ms % 1000 == 990) {
			absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
			lynx_tlog("lynx aoc: playing: the AoC has read %u bytes in %llu ms (%llu bytes/s; 192000 is 48 kHz stereo 16-bit), "
			    "queued %u\n", rx - rx0, ns / 1000000, ns ? (uint64_t)(rx - rx0) * 1000000000ULL / ns : 0, queued);
		}
	}
	if (aocplay_matrix && lynx_amp_variant && seg != ~0u) {
		lynx_amp_variant(0);
		amps = false;
	}
	if (amps) {
		lynx_amp_speaker(0);
	}
	src[9] = 0;
	aoc_out_cmd("SOURCE off", src, sizeof(src), 201);
	bind[10] = 0;
	aoc_out_cmd("BIND2 off", bind, sizeof(bind), 271);
}

/* the first conversation: CMD_AUDIO_OUTPUT_GET_DSP_STATE (233) on audio_output_control */
static void
aoc_talk(void)
{
	vm_offset_t svc = aoc_service("audio_output_control");

	for (int i = 0; i < 3; i++) {
		aoc_mbox[i] = ml_io_map(AOC_DOORBELL(i), 0x1000);
	}
	uint8_t cmd[9] = { 0 };
	*(uint16_t *)(cmd + 4) = 233;                   /* CMD_AUDIO_OUTPUT_GET_DSP_STATE_ID */
	int got = aoc_audio_cmd("audio_output_control", cmd, sizeof(cmd));
	if (got >= 0) {
		lynx_tlog("lynx aoc: THE AOC ANSWERS: %d bytes, id %u, reply %d, DSP mode %u (0 idle, 1 playback, 2 voice)\n", got,
		    *(uint16_t *)(cmd + 4), *(int16_t *)(cmd + 6), cmd[8]);
	} else if (!svc) {
		lynx_tlog("lynx aoc: no audio_output_control service (the firmware has no audio?)\n");
	}
}

/* ---- Darwin's sound through the AoC (aocsound=1, with aoc=3 and amp=2) ----------------------------------------------
 * What the audio server mixes (userland/src/audio_plugin.c -> /dev/hakios-audio) to the speakers: the writer's frames
 * into a ring of ours (never waiting); a feeder thread keeps audio_playbackN's ring (80 ms) three 20 ms blocks full,
 * each block from our ring through the speakers' guard (drivers/audio/hakios.guard.c -- the CS35L41s run without their
 * protection DSP), so the AoC's stream never stops; the amplifiers are up while there is sound and down after three
 * seconds of silence. */

#define DRING_FRAMES            32768           /* 0.68 s; a power of two */
#define SILENCE                 8
#define BLOCK_FRAMES            960             /* 20 ms */
#define BLOCK_BYTES             (BLOCK_FRAMES * 4)
#define QUIET_BEFORE_OFF_US     3000000ull

static uint32_t aocsound, aocsound_gain = 0;
static int16_t dring[DRING_FRAMES * 2];
static volatile uint32_t dring_head, dring_tail;
static volatile uint64_t last_sound_us;
static volatile uint32_t darwin_peak;
static uint64_t darwin_frames;

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* from the writer's thread, under hakios.audio.c's lock: never waits */
static uint32_t
aoc_darwin_sink(const int16_t *frames, uint32_t count)
{
	uint32_t head = dring_head, room = DRING_FRAMES - (head - dring_tail), taken = count < room ? count : room;
	bool sound = false;

	for (uint32_t i = 0; i < taken; i++) {
		int16_t left = frames[2 * i], right = frames[2 * i + 1];
		uint32_t l = left < 0 ? (uint32_t)-(int32_t)left : (uint32_t)left, r = right < 0 ? (uint32_t)-(int32_t)right : (uint32_t)right;

		dring[2 * ((head + i) % DRING_FRAMES)] = left;
		dring[2 * ((head + i) % DRING_FRAMES) + 1] = right;
		sound |= l > SILENCE || r > SILENCE;
		if (l > darwin_peak || r > darwin_peak) {
			darwin_peak = l > r ? l : r;
		}
	}
	__builtin_arm_dmb(0xb);
	dring_head = head + taken;
	darwin_frames += taken;
	if (sound) {
		last_sound_us = now_us();
	}
	return taken;
}

/* BYTES of SRC into the AoC's ring service region R (ring RING of SIZE bytes) at its write pointer, wrapping */
static void
aoc_ring_put(vm_offset_t svc, vm_offset_t r, vm_offset_t ring, uint32_t size, const void *src, uint32_t bytes)
{
	uint32_t wp = rd(r, 20);
	const uint32_t *w = src;

	for (uint32_t i = 0; i < bytes; i += 4) {
		*(volatile uint32_t *)(ring + (wp + i) % size) = w[i / 4];
	}
	__builtin_arm_dsb(0xf);
	wr(r, 12, rd(r, 12) + bytes);
	wr(r, 20, (wp + bytes) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
}

/* the AoC's entry point set up for 48 kHz 16-bit stereo and bound to the speaker, its source on, a silent block ahead;
 * or its source off and unbound */
static bool
aoc_stream(bool on, vm_offset_t svc, vm_offset_t r, vm_offset_t ring, uint32_t size, uint32_t buffer)
{
	static int16_t silence[BLOCK_FRAMES * 2];
	uint8_t ep[28] = { 0 }, ep2[18] = { 0 }, bind[11] = { 0 }, src[10] = { 0 };

	bind[8] = (uint8_t)aocep;
	src[8] = (uint8_t)aocep;
	bind[9] = 0;                                    /* SINK_SPEAKER */
	if (!on) {
		src[9] = 0;
		bind[10] = 0;
		return aoc_out_cmd("SOURCE off", src, sizeof(src), 201) & aoc_out_cmd("BIND2 off", bind, sizeof(bind), 271);
	}
	if (rd(r, 20) != 0) {                           /* aoc_ring_reset_write_pointer */
		wr(r, 12, rd(r, 12) + size - rd(r, 20));
		wr(r, 20, 0);
	}
	*(uint32_t *)(ep + 12) = 48000;
	ep[20] = 2; ep[21] = 1; ep[22] = 5; ep[23] = 1; ep[25] = (uint8_t)aocep; ep[26] = 1; ep[27] = 0;     /* as aoc_play's */
	ep2[8] = (uint8_t)aocep;
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = BLOCK_BYTES;
	bind[10] = 1;
	src[9] = 1;
	aoc_sink_mode();
	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return false;
	}
	(void)aoc_volume((uint8_t)aocep, 0);
	aoc_ring_put(svc, r, ring, size, silence, BLOCK_BYTES);
	if (!aoc_out_cmd("SOURCE on", src, sizeof(src), 201)) {
		return false;
	}
	aoc_sink_mode();
	return true;
}

/*
 * The AoC's stream runs only while there is sound: started when a write is not silence, stopped -- the amplifiers first
 * -- after three seconds of silence. While it is stopped, Darwin's frames are dropped as they come.
 */
static void
aoc_sound_thread(__unused void *parameter, __unused wait_result_t result)
{
	static int16_t block[BLOCK_FRAMES * 2];
	char svcname[24];

	snprintf(svcname, sizeof(svcname), "audio_playback%u", aocep);
	vm_offset_t svc = aoc_service(svcname);
	vm_offset_t r = svc ? region(svc, AOC_DOWN) : 0;
	uint32_t size = r ? rd(r, 4) : 0, period = BLOCK_BYTES, buffer = (size / period) * period;
	vm_offset_t ring = r ? ipc_base + rd(r, 0) : 0;

	if (!svc || buffer < 3 * period) {
		lynx_tlog("lynx aoc: sound: no %s ring to play into (%u bytes)\n", svcname, size);
		return;
	}
	{
		/* the amplifiers' level under Android's (AMP PCM Gain 17): what the guard's limits are measured against */
		uint32_t g = 0;

		PE_parse_boot_argn("amp_gain", &g, sizeof(g));
		if (g > 20) {
			g = 0;
		}
		hakios_guard_configure(true, aocsound_gain);
		hakios_guard_attenuation(g < 17 ? (17 - g) * 8 : 0);
	}
	hakios_audio_set_sink("the speakers (the AoC and two CS35L41s, hakios.lynx_aoc.c)", aoc_darwin_sink);
	lynx_tlog("lynx aoc: sound: WHAT DARWIN PLAYS GOES TO THE SPEAKERS from here on (entry point %u, %s; 48 kHz 16-bit "
	    "stereo, three 20 ms blocks ahead; the guard's gain %u.%03u dB; the amplifiers %s)\n", aocep, svcname,
	    aocsound_gain / 8, aocsound_gain % 8 * 125, lynx_amp_speaker ? "on while there is sound (with speaker=1)" :
	    "NOT BUILT: unheard");

	bool streaming = false, on = false, muted = false, tried = false;
	uint32_t generation = 0, streams = 0, starts = 0, underruns = 0;
	uint64_t fed = 0, silent = 0, dropped = 0, last_report = now_us(), reported_frames = 0, stream_t0 = 0;
	for (;;) {
		uint64_t now = now_us(), sounded = last_sound_us;
		uint32_t below_now, generation_now;
		bool muted_now;

		if (hakios_audio_volume(&below_now, &muted_now, &generation_now) && generation_now != generation) {
			generation = generation_now;
			muted = muted_now;
			lynx_tlog("lynx aoc: sound: Darwin sets the volume: %s%u.%03u dB under the top\n", muted ? "muted (0), " : "",
			    below_now / 8, below_now % 8 * 125);
			hakios_guard_attenuation(muted ? HAKIOS_GUARD_SILENT : 0);
		}
		bool sound_now = sounded != 0 && now - sounded < 100000;

		if (!streaming) {
			if (sound_now && !muted) {
				uint64_t t0 = now_us();

				streaming = aoc_stream(true, svc, r, ring, size, buffer);
				streams++;
				tried = false;
				stream_t0 = now_us();
				lynx_tlog("lynx aoc: sound: Darwin plays: the AoC's stream %s in %llu ms\n", streaming ? "is on" : "DID NOT START",
				    (stream_t0 - t0) / 1000);
				if (!streaming) {
					aoc_stream(false, svc, r, ring, size, buffer);
					last_sound_us = 0;
				}
			} else {
				uint32_t head = dring_head;     /* nothing plays: what Darwin writes is dropped as it comes */

				dropped += head - dring_tail;
				dring_tail = head;
			}
		}
		if (streaming) {
			/* the AoC's ring kept three blocks full */
			uint32_t queued = rd(r, 12) - rd(r, 16);
			if (queued < period && now - stream_t0 > 100000) {
				underruns++;
			}
			while (queued + period <= 3 * period) {
				uint32_t tail = dring_tail, available = dring_head - tail, n = available < BLOCK_FRAMES ? available : BLOCK_FRAMES;

				__builtin_arm_dmb(0xb);
				for (uint32_t i = 0; i < n; i++) {
					block[2 * i] = dring[2 * ((tail + i) % DRING_FRAMES)];
					block[2 * i + 1] = dring[2 * ((tail + i) % DRING_FRAMES) + 1];
				}
				bzero(block + 2 * n, (BLOCK_FRAMES - n) * 4);
				__builtin_arm_dmb(0xb);
				dring_tail = tail + n;
				fed += n;
				silent += BLOCK_FRAMES - n;
				hakios_guard_process(block, BLOCK_FRAMES);      /* (silence too: its filter and averages go on) */
				if (n != 0 && hakios_audio_recording_fed()) {
					hakios_audio_record_fed(block, n);
				}
				aoc_ring_put(svc, r, ring, size, block, period);
				queued += period;
			}

			/* the amplifiers: on while there is sound, off after three seconds of silence and while Darwin's volume is 0 */
			if (lynx_amp_speaker && !on && !tried && !muted && sound_now) {
				tried = true;           /* (once a stream: they may refuse -- speaker=1 not given) */
				on = lynx_amp_speaker(1) == 0;
				starts += on;
			}
			if (muted || now - sounded > QUIET_BEFORE_OFF_US) {
				if (on) {
					lynx_amp_speaker(0);
				}
				aoc_stream(false, svc, r, ring, size, buffer);
				streaming = false;
				lynx_tlog("lynx aoc: sound: %s: %sthe AoC's stream is off; what Darwin wrote peaked at %u%% of full scale\n",
				    muted ? "muted" : "three seconds of silence", on ? "the amplifiers down, " : "", darwin_peak * 100 / 32768);
				on = false;
				darwin_peak = 0;
			}
		}
		if (now - last_report >= 10000000 && darwin_frames != reported_frames) {
			lynx_tlog("lynx aoc: sound: from Darwin %llu frames; %llu fed to the AoC (%llu of silence), %llu dropped while "
			    "nothing played, %u underruns; the stream started %u times, the amplifiers %u times\n", darwin_frames, fed,
			    silent, dropped, underruns, streams, starts);
			reported_frames = darwin_frames;
			last_report = now;
		}
		delay(5000);
	}
}

/* ---- the firmware on the data volume (11-15): aoc.bin is ~21 MB, half the RAM disk, so the boot image may leave it
 * out; userland/src/init.c keeps the RAM disk's copy at /private/var/hakios/fw/aoc.bin, read from here when the image
 * carries none (the RAM disk's own copy still wins). */
#define AOC_FW_DISK_PATH        "/private/var/hakios/fw/aoc.bin"

static uint8_t *
aoc_disk_file(uint64_t *size)
{
	char path[128] = AOC_FW_DISK_PATH;
	vfs_context_t context = vfs_context_kernel();
	vnode_t vnode = NULLVP;
	struct vnode_attr attributes;
	uint8_t *data;
	int left = 0, error;

	PE_parse_boot_argn("aoc_fw", path, sizeof(path));
	if (rootvnode == NULLVP || vnode_lookup(path, 0, &vnode, context) != 0) {
		lynx_tlog("lynx aoc: no %s on the disk either\n", path);
		return NULL;
	}
	VATTR_INIT(&attributes);
	VATTR_WANTED(&attributes, va_data_size);
	if (vnode_getattr(vnode, &attributes, context) != 0 || attributes.va_data_size < AOC_AUTH_HEADER_SIZE + 0x1000 ||
	    attributes.va_data_size > AOC_DRAM_SIZE) {
		vnode_put(vnode);
		lynx_tlog("lynx aoc: %s: not a size an AoC image has\n", path);
		return NULL;
	}
	data = kalloc_data(attributes.va_data_size, Z_WAITOK);
	if (data == NULL) {
		vnode_put(vnode);
		return NULL;
	}
	error = vn_rdwr(UIO_READ, vnode, (caddr_t)data, (int)attributes.va_data_size, 0, UIO_SYSSPACE, 0,
	    vfs_context_ucred(context), &left, vfs_context_proc(context));
	vnode_put(vnode);
	if (error != 0 || left != 0) {
		lynx_tlog("lynx aoc: %s: read error %d, %d bytes left\n", path, error, left);
		kfree_data(data, attributes.va_data_size);
		return NULL;
	}
	lynx_tlog("lynx aoc: the firmware from the disk: %s (%llu bytes)\n", path, (uint64_t)attributes.va_data_size);
	*size = attributes.va_data_size;
	return data;
}

/* the six System MMU windows the firmware's image header lists, before we have read it (its own copy says the same;
 * keeping them here means the SysMMU can be installed before the GSA load, as stock does). */
static const uint32_t sysmmu_windows[6][3] = {
	{ 0x98000000u, 0x94000000u, 0 },     /* the carve-out, 48 MiB */
	{ 0x9b000000u, 0x97000000u, 0 },     /* xhci-dma, 4 MiB */
	{ 0x9e000000u, 0x18200000u, 0 },     /* APM/AoC mailboxes, 2 MiB */
	{ 0x9e200000u, 0x17c00000u, 0 },     /* the GSA block, 1 MiB */
	{ 0x9e500000u, 0x11200000u, 0 },     /* USB PHY/dwc3, 1 MiB */
	{ 0x9e300000u, 0x40000000u, 0 },     /* PCIe ch0 outbound, 1 MiB */
};

static void aoc_boot_image(const uint8_t *f, uint64_t size);

static void
aoc_boot(void)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", "aoc.bin", &size, NULL);
	uint8_t *disk = NULL;

	if (f == NULL || size < AOC_AUTH_HEADER_SIZE + 0x1000) {
		lynx_tlog("lynx aoc: no fw/aoc.bin in the image; looking on the disk\n");
		if ((disk = aoc_disk_file(&size)) == NULL) {
			return;
		}
		f = disk;
	}
	aoc_boot_image(f, size);
	if (disk != NULL) {
		kfree_data(disk, size);         /* (the GSA works from the carve-out and the header page: the file is done with) */
	}
}

static void
aoc_boot_image(const uint8_t *f, uint64_t size)
{
	uint32_t state = 0xff;

	uint32_t auth_magic = *(const uint32_t *)(f + 1024), generation = *(const uint32_t *)(f + 1028);
	const uint8_t *cfg = f + 1024 + (generation == 2 ? 384 : 352);         /* header_v2: a 64-byte hash */
	uint32_t fw_start = *(const uint32_t *)(cfg + 4), fw_size = *(const uint32_t *)(cfg + 8);
	const uint8_t *sb = f + AOC_AUTH_HEADER_SIZE;
	uint32_t sb_magic = *(const uint32_t *)sb, image_size = *(const uint32_t *)(sb + 12);
	char version[65];

	memcpy(version, sb + 48, 64);
	version[64] = 0;
	lynx_tlog("lynx aoc: fw/aoc.bin: %llu bytes, auth magic 0x%x generation %u, locked at 0x%x + 0x%x; superbin 0x%x, image "
	    "0x%x, \"%s\"\n", size, auth_magic, generation, fw_start, fw_size, sb_magic, image_size, version);
	if (auth_magic != 0x00434f41 || sb_magic != 0xaabbccddu || fw_start != AOC_DRAM_PHYS ||
	    image_size + 0x10000 > AOC_DRAM_SIZE || size - AOC_AUTH_HEADER_SIZE > image_size) {
		lynx_tlog("lynx aoc: not an image this driver knows how to load\n");
		return;
	}

	/* request_aoc_on: BLK_AOC's power (the APM turns it on; the acknowledge is the AoC's state) */
	wr(aoc_req, 0, 1);
	for (int ms = 0; ms < 100; ms++) {
		if (rd(aoc_req, 0x40) != 0) {
			break;
		}
		delay(1000);
	}
	lynx_tlog("lynx aoc: aoc_req 1, the AoC's acknowledge 0x%x\n", rd(aoc_req, 0x40));

	aoc_s2mpu_open();
	sysmmu_install(&sysmmu_windows[0][0], 6);
	if (!lynx_trusty_aoc_cmd) {
		lynx_tlog("lynx aoc: no Trusty client (lynx_trusty_aoc_cmd is not built): the AoC cannot be started on gs201\n");
		return;
	}
	{
		int rc = lynx_trusty_aoc_cmd(GSA_AOC_GET_STATE, &state);

		if (rc == -2) {
			lynx_tlog("lynx aoc: no Trusty client built (lynx_trusty_aoc_cmd answers -2): the AoC cannot be started\n");
			return;
		}
		if (rc != 0) {
			lynx_tlog("lynx aoc: the GSA's AoC service not reached (Trusty, %d): not loaded\n", rc);
			return;
		}
	}
	lynx_tlog("lynx aoc: the AoC's state %u (0 inactive); copying the image ...\n", state);
	if (state != 0) {
		lynx_tlog("lynx aoc: the AoC is not inactive (%u): not loaded again\n", state);
		return;
	}

	/* the body at the carve-out's start (aligned 64-bit stores), the IPC area cleared */
	vm_offset_t dram = carveout_map();
	if (!dram) {
		lynx_tlog("lynx aoc: the carve-out could not be mapped\n");
		return;
	}
	uint64_t body = size - AOC_AUTH_HEADER_SIZE;
	const uint8_t *src = f + AOC_AUTH_HEADER_SIZE;
	uint64_t i = 0;
	for (; i + 8 <= body; i += 8) {
		uint64_t w;
		__builtin_memcpy(&w, src + i, 8);
		*(volatile uint64_t *)(dram + i) = w;
	}
	for (; i < body; i++) {
		*(volatile uint8_t *)(dram + i) = src[i];
	}
	for (uint32_t k = 0; k < 0x10000; k += 8) {
		*(volatile uint64_t *)(dram + image_size + k) = 0;
	}
	__builtin_arm_dsb(0xf);

	/* the header in a page of ours, cleaned to memory for the GSA to read by DMA */
	vm_offset_t hdr = 0;
	if (kmem_alloc_contig(kernel_map, &hdr, PAGE_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		lynx_tlog("lynx aoc: no memory for the header\n");
		return;
	}
	memcpy((void *)hdr, f, AOC_AUTH_HEADER_SIZE);
	for (vm_offset_t line = hdr; line < hdr + PAGE_SIZE; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__builtin_arm_dsb(0xf);
	uint64_t hdr_pa = kvtophys(hdr);
	uint32_t args[4] = { (uint32_t)hdr_pa, (uint32_t)(hdr_pa >> 32), (uint32_t)AOC_DRAM_PHYS, (uint32_t)(AOC_DRAM_PHYS >> 32) };
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;
	uint64_t t0 = mach_absolute_time(), ns;

	lynx_tlog("lynx aoc: the image copied (%llu MB); the GSA asked to authenticate and load it (header at 0x%llx)\n",
	    (size - AOC_AUTH_HEADER_SIZE) >> 20, hdr_pa);
	if (!gsa_s2mpu_open(true)) {
		lynx_tlog("lynx aoc: the GSA's S2MPU not opened: the load will not authenticate\n");
	}
	if (gsa_request_ms(GSA_MB_CMD_LOAD_AOC_FW_IMG, args, 4, rsp, &n, &err, 20000) != 0 || err != 0) {
		lynx_tlog("lynx aoc: THE GSA REFUSED THE IMAGE: error %u (1 invalid args, 2 auth failed, 3 busy, 4 running, 5 resources)\n",
		    err);
		gsa_s2mpu_open(false);
		return;
	}
	gsa_s2mpu_open(false);
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	lynx_tlog("lynx aoc: THE GSA LOADED THE IMAGE in %llu ms\n", ns / 1000000);

	/* the firmware's parameters at the IPC area (aoc_pass_fw_information): magic, then key/size/value triples */
	vm_offset_t chip = ml_io_map(CHIPID_PHYS, 0x1000);
	uint32_t chip_type = chip ? rd(chip, 0) & 0xf : 0, r10 = chip ? rd(chip, 0x10) : 0;
	uint32_t chip_rev = (((r10 >> 20) & 0xf) << 4) | ((r10 >> 16) & 0xf);
	uint32_t top = AOC_DRAM_SIZE, sensor = top - (4u << 20), playback = sensor - 0x4000, capture = playback - 0x10000;
	const uint32_t kv[][2] = {
		{ 0x1001, 0x30501 }, { 0x1002, 0x10000 },               /* the board (aoc-board-id, -rev: "L10") */
		{ 0x1003, 0 },                                          /* SRAM repaired */
		{ 0x1005, (uint32_t)AOC_DRAM_PHYS }, { 0x1006, AOC_DRAM_SIZE },
		{ 0x1007, (uint32_t)(AOC_DRAM_AOC + sensor) }, { 0x1008, 4u << 20 },
		{ 0x100C, (uint32_t)(AOC_DRAM_AOC + playback) }, { 0x100D, 0x4000 },
		{ 0x100E, (uint32_t)(AOC_DRAM_AOC + capture) }, { 0x100F, 0x10000 },
		{ 0x1009, 0 }, { 0x100A, 0 }, { 0x100B, 0 }, { 0x1010, 0 },     /* VNOM, monitor mode, UART, ultrasonic */
		{ 0x1011, (uint32_t)mach_absolute_time() },             /* a seed */
		{ 0x1012, chip_rev }, { 0x1013, chip_type }, { 0x1014, 1 }, { 0x1015, 0 },   /* the chip, gnss-type 1 */
	};
	volatile uint32_t *p = (volatile uint32_t *)(dram + image_size);
	uint32_t num = sizeof(kv) / sizeof(kv[0]);
	p[0] = AOC_PARAMETER_MAGIC;
	p[1] = num;
	p[2] = 12 + num * 12;
	for (uint32_t i = 0; i < num; i++) {
		p[3 + 3 * i] = kv[i][0];
		p[4 + 3 * i] = 4;
		p[5 + 3 * i] = kv[i][1];
	}
	__builtin_arm_dsb(0xf);
	lynx_tlog("lynx aoc: parameters written (chip type %u revision 0x%x); the AoC started through the GSA\n", chip_type,
	    chip_rev);

	if (lynx_trusty_aoc_cmd(GSA_AOC_START, &state) != 0) {      /* GSA_AOC_START */
		lynx_tlog("lynx aoc: GSA_AOC_START refused: the AoC not started (a Trusty client is needed on gs201)\n");
		return;
	}
	for (int ms = 0; ms < 10000; ms += 100) {
		if (rd(dram, image_size) == AOC_MAGIC) {
			char fw[33] = "";
			for (int i = 0; i < 32; i++) {
				fw[i] = (char)*(volatile uint8_t *)(dram + image_size + 36 + i);
				if (fw[i] && (fw[i] < 32 || fw[i] > 126)) {
					fw[i] = 0;
					break;
				}
			}
			lynx_tlog("lynx aoc: THE AOC IS UP after %d ms: its IPC block, fw 0x%x hw 0x%x status 0x%x, %u services of %u bytes "
			    "at +0x%x, \"%s\"\n", ms, rd(dram, image_size + 4), rd(dram, image_size + 8), rd(dram, image_size + 12),
			    rd(dram, image_size + 24), rd(dram, image_size + 28), rd(dram, image_size + 32), fw);
			ipc_base = dram + image_size;
			ipc_room = AOC_DRAM_SIZE - image_size;
			aoc_talk();

			PE_parse_boot_argn("aocsink", &aocsink, sizeof(aocsink));
			PE_parse_boot_argn("aocep", &aocep, sizeof(aocep));
			if (aocep > 6) {
				aocep = 0;
			}
			uint32_t play = 0;
			PE_parse_boot_argn("aocplay", &play, sizeof(play));
			if (PE_parse_boot_argn("aocplay", &aocplay, sizeof(aocplay)) && aocplay != 0) {
				PE_parse_boot_argn("aocplay_amp", &aocplay_amp, sizeof(aocplay_amp));
				if (aocplay_amp > 32767) {
					aocplay_amp = 32767;
				}
				PE_parse_boot_argn("aocplay_lr", &aocplay_lr, sizeof(aocplay_lr));
				PE_parse_boot_argn("aocplay_hz", &aocplay_hz, sizeof(aocplay_hz));
				if (aocplay_hz < 20 || aocplay_hz > 20000) {
					aocplay_hz = 440;
				}
				PE_parse_boot_argn("aocplay_matrix", &aocplay_matrix, sizeof(aocplay_matrix));
				lynx_tlog("lynx aoc: aocplay: a %u Hz tone for %u s, peak as aocplay_amp%s\n", aocplay_hz, aocplay,
				    aocplay_lr ? ", left only and right only by turns" : ", both sides");
				aoc_play();
			}
			if (!PE_parse_boot_argn("aocsound", &aocsound, sizeof(aocsound))) {
				uint32_t speaker = 0;

				if (PE_parse_boot_argn("speaker", &speaker, sizeof(speaker)) && speaker != 0) {
					aocsound = 1;
				}
			}
			if (aocsound != 0) {
				thread_t t;

				PE_parse_boot_argn("aocsound_gain", &aocsound_gain, sizeof(aocsound_gain));
				if (aocsound_gain > HAKIOS_GUARD_MAX_GAIN) {
					aocsound_gain = HAKIOS_GUARD_MAX_GAIN;
				}
				if (kernel_thread_start_priority(aoc_sound_thread, NULL, 93, &t) == KERN_SUCCESS) {
					thread_deallocate(t);
				}
			}
			return;
		}
		delay(100000);
	}
	lynx_tlog("lynx aoc: no IPC block in 10 s (word 0x%x, state %u)\n", rd(dram, image_size), state);
}

static void
lynx_aoc_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;

	lynx_tlog("lynx aoc: aoc_req 0x%x, the AoC's state 0x%x\n", rd(aoc_req, 0), rd(aoc_req, 0x40));
	lynx_tlog("lynx aoc: GSA mailbox: INTMR0 0x%x INTSR0 0x%x INTMSR0 0x%x; SR0-3 0x%x 0x%x 0x%x 0x%x\n", rd(gsa, MBOX_INTMR0),
	    rd(gsa, MBOX_INTSR0), rd(gsa, MBOX_INTMSR0), rd(gsa, MBOX_SR(0)), rd(gsa, MBOX_SR(1)), rd(gsa, MBOX_SR(2)),
	    rd(gsa, MBOX_SR(3)));
	if (gsa_request(GSA_MB_CMD_GET_CHIP_ID, NULL, 0, rsp, &n, &err) == 0) {
		lynx_tlog("lynx aoc: THE GSA ANSWERS: chip id (%u words) %08x %08x %08x %08x\n", n, n > 0 ? rsp[0] : 0, n > 1 ? rsp[1] : 0,
		    n > 2 ? rsp[2] : 0, n > 3 ? rsp[3] : 0);
	}
	if (mode >= 3) {
		aoc_boot();
		return;
	}
	if (mode < 2) {
		return;
	}
	/*
	 * aoc=2: a look 6 MiB into the carve-out, where a running firmware keeps its control block by default. One word, and
	 * only there: on gs201 the carve-out is no-map, and after a load the body is locked against every other master.
	 */
	vm_offset_t ipc = ml_io_map(AOC_DRAM_PHYS + (6u << 20), 0x1000);

	if (!ipc) {
		lynx_tlog("lynx aoc: 6 MB into the carve-out could not be mapped\n");
		return;
	}
	uint32_t magic = rd(ipc, 0);

	lynx_tlog("lynx aoc: 6 MB into the carve-out: 0x%08x%s\n", magic, magic == AOC_MAGIC ? " -- AN IPC BLOCK" : "");
	if (magic == AOC_MAGIC) {
		lynx_tlog("lynx aoc: fw_version 0x%x hw_version 0x%x status 0x%x, %u services of %u bytes at +0x%x\n", rd(ipc, 4),
		    rd(ipc, 8), rd(ipc, 12), rd(ipc, 24), rd(ipc, 28), rd(ipc, 32));
	}
}

__startup_func
static void
lynx_aoc_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("aoc", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	aoc_req = ml_io_map(AOC_REQ_PHYS, 0x1000);
	gsa = ml_io_map(GSA_MBOX_PHYS, 0x1000);
	gsa_s2mpu = ml_io_map(GSA_S2MPU_PHYS, 0x1000);
	aoc_s2mpu = ml_io_map(AOC_S2MPU_PHYS, 0x1000);
	sysmmu = ml_io_map(AOC_SYSMMU_PHYS, 0x10000);
	if (!aoc_req || !gsa || !gsa_s2mpu || !aoc_s2mpu || !sysmmu) {
		lynx_tlog("lynx aoc: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(lynx_aoc_start, NULL);
	/* (aoc=3 at 25 s: after the USB log is up -- a load that killed the kernel at 3 s would leave no line of it) */
	clock_interval_to_deadline(mode >= 3 ? 25 : 3, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_aoc_init);
