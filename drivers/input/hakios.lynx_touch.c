/*
 * The touch screen of the Pixel 7a (lynx, gs201): a Goodix GT9895 ("Berlin", firmware patch id 9895) on the SPI of
 * PERIC1 USI0 (0x10D10000), as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with boot-arg
 * lynx_touch=1.
 *
 * The loader (lynxld) has already done the parts that need the power controller: LDO25M and LDO26M of the S2MPG12 on
 * through ACPM, the touch bus taken from the AoC (gph1-0 low), the SPI pins, CMU_PERIC1's USI0 clock (oscillator,
 * gates on), USI in SPI mode, the controller initialised, and the chip out of reset (gpp23-2 high). This driver only
 * talks to it: the SPI controller polled, chip select on gpp20-3, the interrupt line gpa7-0 looked at.
 *
 * Seen on the phone, from the loader (probes/tspi.c): the internal chip select of the
 * controller must be active (SLAVE_SEL = 0) or nothing leaves the FIFO; reads are 0xF1, the address, three 0xFF, then the
 * data from byte 8; the chip answered brl_dev_confirm and reported "BERLIN" / "9895".
 *
 * The chip's protocol (the firmware version, the chip description that says where the touch report is, the report and its
 * sums) is that of the POCO's GT9916 in drivers/input/hakios.goodix.c, which is the same Berlin family: taken from there.
 * Register sources: Google's kernel/gs spi-s3c64xx.c and pinctrl-gs201.c, google-modules/touch/goodix_touch.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <stdarg.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

#define SPI_PHYS        0x10D10000ull
#define GPIO_PERIC1     0x10C40000ull   /* gpp20 at +0x0 (CS = pin 3), gpp23 at +0x60 (reset = pin 2) */
#define GPIO_ALIVE      0x180E0000ull   /* gpa7 at +0x20 (interrupt = pin 0) */
#define MAP_SIZE        0x4000

#define CH_CFG          0x00
#define SLAVE_SEL       0x0C
#define STATUS          0x14
#define TX_DATA         0x18
#define RX_DATA         0x1C
#define PACKET_CNT      0x20

static vm_offset_t spi, peric1, alive, oops;

/* Every message also goes to the ramoops console the loader writes (0xfd3ff000, read after a long-key reset as
 * /sys/fs/pstore/console-ramoops-0): header "DBGC", start, size; data from +12; the loader's count at +12+0x40000. */
#define OOPS_PHYS 0xfd3ff000ull
#define OOPS_CAP  0x40000u
void lynx_tlog(const char *fmt, ...) __printflike(1, 2);
#define tlog lynx_tlog
void
tlog(const char *fmt, ...)
{
	char line[256];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	printf("%s", line);
	if (!oops || n <= 0) {
		return;
	}
	volatile uint32_t *hdr = (volatile uint32_t *)oops, *cnt = (volatile uint32_t *)(oops + 12 + OOPS_CAP);
	volatile uint8_t *data = (volatile uint8_t *)(oops + 12);
	uint32_t at = *cnt;
	for (int i = 0; i < n && i < (int)sizeof(line) - 1 && at < OOPS_CAP; i++) {
		data[at++] = (uint8_t)line[i];
	}
	*cnt = at;
	hdr[0] = 0x43474244u; hdr[1] = at; hdr[2] = at;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }
static void sleep_us(uint32_t us) { delay_for_interval(us, NSEC_PER_USEC); }
static void sleep_ms(uint32_t ms) { delay_for_interval(ms, NSEC_PER_MSEC); }

static void
cs(bool active)
{
	uint32_t v = rd(peric1 + 0x04);

	wr(peric1 + 0x04, active ? (v & ~(1u << 3)) : (v | (1u << 3)));
}

static bool
irq_low(void)
{
	return (rd(alive + 0x20 + 0x04) & 1u) == 0;
}

/* One full-duplex transfer of any length: the FIFO (64 bytes) kept fed and drained, polled; rx may be NULL. */
static bool
spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	uint32_t sent = 0, got = 0, spins = 0;
	bool ok;

	if (length == 0 || length > 0xffff) {
		return false;
	}
	cs(true);
	sleep_us(2);
	wr(spi + CH_CFG, rd(spi + CH_CFG) & ~3u);
	wr(spi + CH_CFG, rd(spi + CH_CFG) | (1u << 5));         /* software reset: empty FIFOs */
	wr(spi + CH_CFG, rd(spi + CH_CFG) & ~(1u << 5));
	wr(spi + PACKET_CNT, (1u << 16) | length);
	wr(spi + CH_CFG, rd(spi + CH_CFG) | 3u);
	wr(spi + SLAVE_SEL, 0);
	while (got < length && spins < 200000) {
		uint32_t status = rd(spi + STATUS);
		uint32_t tx_level = (status >> 6) & 0x1ff, rx_level = (status >> 15) & 0x1ff;

		while (sent < length && tx_level < 32 && sent - got < 48) {
			wr(spi + TX_DATA, tx[sent++]);
			tx_level++;
		}
		while (rx_level-- && got < length) {
			uint8_t b = (uint8_t)rd(spi + RX_DATA);

			if (rx) {
				rx[got] = b;
			}
			got++;
		}
		spins++;
	}
	ok = got == length;
	wr(spi + SLAVE_SEL, 1);
	wr(spi + CH_CFG, rd(spi + CH_CFG) & ~3u);
	wr(spi + PACKET_CNT, 0);
	sleep_us(2);
	cs(false);
	return ok;
}

/* ---- the Goodix Berlin protocol, as drivers/input/hakios.goodix.c ----------------------------------------------- */

#define GOODIX_FW_VERSION       0x10014
#define GOODIX_IC_INFO          0x10070
#define GOODIX_MAX_READ         1024
#define GOODIX_MAX_TOUCHES      10
#define GOODIX_STATUS_TOUCH     0x80
#define GOODIX_STATUS_REQUEST   0x40
#define GOODIX_REQUEST_RESET    3

static uint8_t spi_tx[8 + GOODIX_MAX_READ], spi_rx[8 + GOODIX_MAX_READ];
static uint32_t touch_address, max_x = 1080, max_y = 2400, stride = 14;   /* point_struct_len: 14 on the GT9895, 8 on older Berlins */

static bool
goodix_read(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf1;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	spi_tx[5] = spi_tx[6] = spi_tx[7] = 0xff;
	memset(spi_tx + 8, 0, length);
	if (!spi_transfer(spi_tx, spi_rx, 8 + length)) {
		return false;
	}
	memcpy(data, spi_rx + 8, length);
	return true;
}

static bool
goodix_write(uint32_t address, const uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf0;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	memcpy(spi_tx + 5, data, length);
	return spi_transfer(spi_tx, NULL, 5 + length);
}

static bool
goodix_checksum_ok(const uint8_t *data, uint32_t length)
{
	uint32_t sum = 0;
	bool blank = true;

	if (length < 2) {
		return false;
	}
	for (uint32_t i = 0; i < length - 2; i++) {
		sum += data[i];
		blank &= data[i] == 0 || data[i] == 0xff;
	}
	return !blank && (uint16_t)sum == (uint16_t)(data[length - 2] | data[length - 1] << 8);
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static void
touch_reset(void)
{
	wr(peric1 + 0x60 + 0x04, rd(peric1 + 0x60 + 0x04) & ~(1u << 2));
	sleep_ms(2);
	wr(peric1 + 0x60 + 0x04, rd(peric1 + 0x60 + 0x04) | (1u << 2));
	sleep_ms(100);
}

static bool
goodix_probe(void)
{
	static uint8_t info[GOODIX_MAX_READ];
	uint8_t version[28];
	uint32_t length, offset;

	if (!goodix_read(GOODIX_FW_VERSION, version, sizeof(version))) {
		tlog("lynx_touch: SPI transfer failed\n");
		return false;
	}
	if (!goodix_checksum_ok(version, sizeof(version))) {
		tlog("lynx_touch: no valid firmware version: %02x %02x %02x %02x %02x %02x ... %02x %02x\n", version[0],
		    version[1], version[2], version[3], version[4], version[5], version[26], version[27]);
		return false;
	}
	tlog("lynx_touch: chip \"%.6s\", firmware \"%.8s\" %02x.%02x.%02x.%02x, sensor %u\n", (const char *)version,
	    (const char *)version + 10, version[18], version[19], version[20], version[21], version[23]);
	if (!goodix_read(GOODIX_IC_INFO, info, 2)) {
		return false;
	}
	length = le16(info);
	if (length < 40 || length >= GOODIX_MAX_READ || !goodix_read(GOODIX_IC_INFO, info, length) ||
	    !goodix_checksum_ok(info, length)) {
		tlog("lynx_touch: bad chip description (length %u)\n", length);
		return false;
	}
	offset = 2 + 16 + 10 + 4;
	for (int i = 0; i < 5; i++) {
		if (offset >= length) {
			return false;
		}
		offset += 1 + 2u * info[offset];
	}
	if (offset + 48 + 2 > length) {
		tlog("lynx_touch: chip description too short (%u, lists end at %u)\n", length, offset);
		return false;
	}
	touch_address = le32(info + offset + 44);
	if (offset + 122 + 20 + 2 <= length) {
		uint16_t x = le16(info + offset + 122 + 16), y = le16(info + offset + 122 + 18);

		if (x && y) {
			max_x = x;
			max_y = y;
		}
	}
	tlog("lynx_touch: touch report at %#x, coordinates up to %u x %u\n", touch_address, max_x, max_y);
	return touch_address != 0;
}

static struct {
	uint32_t polls, reports, touches, bad, last_x, last_y;
} seen;

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[GOODIX_MAX_TOUCHES];

static void
goodix_report(const uint8_t *touches, uint32_t count)
{
	static unsigned shown;

	for (int i = 0; i < GOODIX_MAX_TOUCHES; i++) {
		contact[i].seen = false;
	}
	for (uint32_t i = 0; i < count; i++) {
		const uint8_t *t = touches + stride * i;
		uint32_t id = t[0] >> 4, kind = t[0] & 0xf;
		uint32_t x = le16(t + 2), y = le16(t + 4);

		if (id >= GOODIX_MAX_TOUCHES || kind == 1 || kind == 3) {
			continue;
		}
		if (shown < 40) {
			shown++;
			tlog("lynx_touch: finger %u at %u, %u (of %u x %u)\n", id, x, y, max_x, max_y);
		}
		seen.touches++;
		seen.last_x = x;
		seen.last_y = y;
		x = x >= max_x ? 65535 : x * 65535 / max_x;
		y = y >= max_y ? 65535 : y * 65535 / max_y;
		contact[id].seen = true;
		if (!contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
	for (uint32_t id = 0; id < GOODIX_MAX_TOUCHES; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

static void
goodix_poll(void)
{
	static uint8_t report[8 + 14 * GOODIX_MAX_TOUCHES + 2];
	static unsigned bad;
	const uint8_t zero = 0;
	const uint32_t first = 8 + 14 * 2 + 2;
	uint32_t count;

	seen.polls++;
	if (!goodix_read(touch_address, report, first) || report[0] == 0) {
		return;
	}
	seen.reports++;
	count = report[2] & 0xf;
	if ((report[0] & GOODIX_STATUS_TOUCH) && count > 2 && count <= GOODIX_MAX_TOUCHES &&
	    !goodix_read(touch_address + 8, report + 8, 14 * count + 2)) {
		return;
	}
	goodix_write(touch_address, &zero, 1);
	if (!goodix_checksum_ok(report, 8)) {
		seen.bad++;
		if (bad++ < 8) {
			tlog("lynx_touch: bad report header: %02x %02x %02x %02x %02x %02x %02x %02x\n", report[0], report[1],
			    report[2], report[3], report[4], report[5], report[6], report[7]);
		}
		return;
	}
	if (report[0] & GOODIX_STATUS_REQUEST) {
		if ((report[2] & 0xf) == GOODIX_REQUEST_RESET || report[2] == GOODIX_REQUEST_RESET) {
			tlog("lynx_touch: the chip asks to be reset\n");
			touch_reset();
		}
		return;
	}
	if (!(report[0] & GOODIX_STATUS_TOUCH)) {
		return;
	}
	if (count && count <= GOODIX_MAX_TOUCHES && !goodix_checksum_ok(report + 8, count * stride + 2)) {
		uint32_t other = stride == 14 ? 8 : 14;

		if (goodix_checksum_ok(report + 8, count * other + 2)) {
			tlog("lynx_touch: touch points are %u bytes, not %u\n", other, stride);
			stride = other;
		}
	}
	if (count > GOODIX_MAX_TOUCHES || (count && !goodix_checksum_ok(report + 8, count * stride + 2))) {
		seen.bad++;
		if (bad++ < 8) {
			tlog("lynx_touch: bad report (%u touches): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    count, report[8], report[9], report[10], report[11], report[12], report[13], report[14], report[15],
			    report[16], report[17], report[18], report[19], report[20], report[21], report[22], report[23]);
		}
		return;
	}
	goodix_report(report + 8, count);
}

static bool
touching(void)
{
	for (uint32_t i = 0; i < GOODIX_MAX_TOUCHES; i++) {
		if (contact[i].down) {
			return true;
		}
	}
	return false;
}

/* The kernel's own log (loader_return.c) is only handed over when the kernel returns to the loader; the lines about
 * starting the other processors are copied to the ramoops console as they appear, in the log's first 256 KB. */
extern char loader_log[];
extern uint32_t loader_log_len;
extern unsigned int processor_avail_count, real_ncpus;

static void
oops_raw(const char *text, uint32_t n)
{
	volatile uint32_t *hdr = (volatile uint32_t *)oops, *cnt = (volatile uint32_t *)(oops + 12 + OOPS_CAP);
	volatile uint8_t *data = (volatile uint8_t *)(oops + 12);
	uint32_t at = *cnt;

	for (uint32_t i = 0; i < n && at < OOPS_CAP; i++) {
		data[at++] = (uint8_t)text[i];
	}
	*cnt = at;
	hdr[0] = 0x43474244u; hdr[1] = at; hdr[2] = at;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
kernel_log_scan(void)
{
	static uint32_t scanned;
	uint32_t end = loader_log_len < 256 * 1024 ? loader_log_len : 256 * 1024;

	while (scanned < end) {
		uint32_t line = scanned, n = 0;

		while (line + n < end && loader_log[line + n] != '\n') {
			n++;
		}
		if (line + n >= end) {
			return;         /* not a whole line yet */
		}
		n++;
		if (n > 6 && n < 400 && (strnstr(loader_log + line, "psci", n) || strnstr(loader_log + line, "GenericARMCPU", n) ||
		    strnstr(loader_log + line, "panic", n) || strnstr(loader_log + line, "secondary", n))) {
			oops_raw("K: ", 3);
			oops_raw(loader_log + line, n);
		}
		scanned = line + n;
	}
}

static void buttons_poll(void);
static void buttons_init(void);
static vm_offset_t alive_d;

static void
lynx_touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000;
	unsigned idle = 0;

	oops = ml_io_map(OOPS_PHYS, 0x41000);
	tlog("lynx_touch: thread started\n");
	{
		/* how fast is this core really: 500 million one-cycle loop turns, timed */
		uint64_t t0 = mach_absolute_time(), ns, n = 500000000ull, midr;

		while (n) {
			n--;
			__asm__ volatile ("" : "+r"(n));
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		__asm__ volatile ("mrs %0, midr_el1" : "=r"(midr));
		tlog("lynx_touch: cpu midr %#llx, 500M loop turns in %llu ms (about %llu MHz)\n", midr, ns / 1000000,
		    ns ? 500000000ull * 1000 / ns : 0);
	}
	spi = ml_io_map(SPI_PHYS, MAP_SIZE);
	peric1 = ml_io_map(GPIO_PERIC1, MAP_SIZE);
	alive = ml_io_map(GPIO_ALIVE, MAP_SIZE);
	buttons_init();
	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 1000 || poll_us > 100000) {
		poll_us = 8000;
	}
	for (int attempt = 0; attempt < 3 && !goodix_probe(); attempt++) {
		touch_reset();
	}
	if (touch_address == 0) {
		tlog("lynx_touch: no Goodix touch screen (did the loader bring it up?); buttons only\n");
		for (;;) {
			buttons_poll();
			sleep_us(poll_us);
		}
	}
	tlog("lynx_touch: touch screen ready, the interrupt line looked at every %u us\n", poll_us);
	for (unsigned tick = 1;; tick++) {
		buttons_poll();
		if (irq_low() || touching()) {
			idle = 0;
			goodix_poll();
		} else if (++idle >= 16000 / poll_us) {
			idle = 0;
			goodix_poll();
		}
		if (tick % (5 * (1000000 / poll_us)) == 0) {
			kernel_log_scan();
			tlog("lynx_touch: %u reports in %u looks (%u bad), %u finger positions, the last at %u, %u; cpus %u of %u\n",
			    seen.reports, seen.polls, seen.bad, seen.touches, seen.last_x, seen.last_y, processor_avail_count, real_ncpus);
		}
		sleep_us(poll_us);
	}
}

/* ---- buttons: plain GPIOs, active low (lynx FDT gpio_keys; power key-power pinctrl), polled with the touch ---------- */
/* alive_d: pinctrl 0x180D0000: gpa10 at +0xE0, bit 0 power, bit 1 volume down */
static struct {
	const char *name;
	uint32_t record, bit;
	bool far, down;
} keys[] = {
	{ "power", HAKIOS_BUTTON_LOCK, 0, false, false },
	{ "volume down", HAKIOS_BUTTON_VOLUME_DOWN, 1, false, false },
	{ "volume up", HAKIOS_BUTTON_VOLUME_UP, 5, true, false },     /* gpa8-5 at 0x180E0000 + 0x40 */
};

static void
buttons_init(void)
{
	alive_d = ml_io_map(0x180D0000ull, MAP_SIZE);
	/* volume up (gpa8-5) has no pull in the FDT and read 0 all the time here (v15b): its outside pull-up is not powered.
	 * The pad's own pull-up (PUD +0x08, 4 bits a pin, 3 = up), as power and volume down have. */
	tlog("lynx_touch: gpa8 before: con %#x dat %#x pud %#x\n", rd(alive + 0x40), rd(alive + 0x44), rd(alive + 0x48));
	wr(alive + 0x40 + 0x08, (rd(alive + 0x40 + 0x08) & ~(0xfu << 20)) | (0x3u << 20));
	sleep_ms(2);
	tlog("lynx_touch: gpa8 pull-up: con %#x dat %#x pud %#x\n", rd(alive + 0x40), rd(alive + 0x44), rd(alive + 0x48));
	wr(alive + 0x40, rd(alive + 0x40) | (0xfu << 20));      /* the FDT's pin-function 0xf (interrupt input), as Linux has it */
	sleep_ms(2);
	tlog("lynx_touch: gpa8 function f: con %#x dat %#x pud %#x\n", rd(alive + 0x40), rd(alive + 0x44), rd(alive + 0x48));
	tlog("lynx_touch: keys raw gpa10 %#x gpa8 %#x (con %#x %#x)\n", rd(alive_d + 0xe4), rd(alive + 0x44), rd(alive_d + 0xe0), rd(alive + 0x40));
	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		keys[i].down = ((((keys[i].far ? rd(alive + 0x44) : rd(alive_d + 0xe4)) >> keys[i].bit) & 1u) == 0);
	}
}

static void
buttons_poll(void)
{
	uint32_t a10 = rd(alive_d + 0xe0 + 0x04), a8 = rd(alive + 0x40 + 0x04);

	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		bool down = (((keys[i].far ? a8 : a10) >> keys[i].bit) & 1u) == 0;

		if (down != keys[i].down) {
			keys[i].down = down;
			tlog("lynx_touch: button %s %s\n", keys[i].name, down ? "down" : "up");
			hakios_touch_post(down ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, keys[i].record, 0, 0);
		}
	}
}

static thread_call_t start_call;

static void
start_thread(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(lynx_touch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
lynx_touch_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("lynx_touch", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	start_call = thread_call_allocate(start_thread, NULL);
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start_call, deadline);
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_touch_init);
