/*
 * The battery of the Pixel 7a (lynx): the MAX77759's fuel gauge (I2C 0x36) and charger (I2C 0x69) on the bus of
 * PERIC1 USI13 (hsi2c@10D60000 in the phone's FDT), read for HakiosBattery (drivers/power/hakios.battery.c,
 * hakios_battery_report()). Only with boot-arg lynx_battery=1. Reads only: nothing is written to either chip.
 *
 * The bus is driven by hand on its two pads (gpp25-0 SCL, gpp25-1 SDA, the FDT's gpio_scl / gpio_sda): a few
 * registers every five seconds do not need the controller, its clocks or its interrupt. A pad is either an input
 * (the line's pull-up makes it high) or an output of 0: open drain, as the bus wants.
 *
 * Registers: Google's google-modules/bms max77759_A1_regs.h (charger) and max1720x_battery.h (the gauge's ModelGauge m5
 * map and its units; the sense resistor is the FDT's maxim,rsense-default = 500, in 10 micro-ohms).
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

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

#define GPIO_PERIC1     0x10C40000ull
#define GPP25           0xA0            /* CON +0 (4 bits a pin), DAT +4 */
#define FG_ADDR         0x36
#define CHG_ADDR        0x69
#define RSENSE          500             /* 10 micro-ohms */

static vm_offset_t pads;
static uint32_t i2c_failures;

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }
static void d(void) { delay_for_interval(5, NSEC_PER_USEC); }

static void
line(unsigned pin, bool high)
{
	uint32_t v = rd(pads + GPP25) & ~(0xfu << (4 * pin));

	wr(pads + GPP25, high ? v : (v | (1u << (4 * pin))));
}

static bool line_in(unsigned pin) { return (rd(pads + GPP25 + 4) >> pin) & 1u; }

static void
scl_high(void)
{
	line(0, true);
	for (int i = 0; i < 200 && !line_in(0); i++) {  /* a chip may hold the clock */
		d();
	}
}

static void i2c_start(void) { line(1, true); scl_high(); d(); line(1, false); d(); line(0, false); d(); }
static void i2c_stop(void) { line(1, false); d(); scl_high(); d(); line(1, true); d(); }

static bool
i2c_put(uint8_t byte)
{
	bool ack;

	for (int bit = 7; bit >= 0; bit--) {
		line(1, (byte >> bit) & 1);
		d();
		scl_high();
		d();
		line(0, false);
	}
	line(1, true);
	d();
	scl_high();
	d();
	ack = !line_in(1);
	line(0, false);
	d();
	return ack;
}

static uint8_t
i2c_get(bool ack)
{
	uint8_t byte = 0;

	line(1, true);
	for (int bit = 0; bit < 8; bit++) {
		d();
		scl_high();
		d();
		byte = (uint8_t)((byte << 1) | (line_in(1) ? 1 : 0));
		line(0, false);
	}
	line(1, !ack);
	d();
	scl_high();
	d();
	line(0, false);
	line(1, true);
	d();
	return byte;
}

/* count bytes of a register, low byte first; false if a chip did not acknowledge */
static bool
i2c_read(uint8_t chip, uint8_t reg, uint8_t *data, unsigned count)
{
	bool ok;

	i2c_start();
	ok = i2c_put((uint8_t)(chip << 1)) && i2c_put(reg);
	if (ok) {
		i2c_start();
		ok = i2c_put((uint8_t)(chip << 1 | 1));
		for (unsigned i = 0; ok && i < count; i++) {
			data[i] = i2c_get(i + 1 < count);
		}
	}
	i2c_stop();
	if (!ok) {
		i2c_failures++;
	}
	return ok;
}

static bool
fg(uint8_t reg, uint16_t *value)
{
	uint8_t b[2];

	if (!i2c_read(FG_ADDR, reg, b, 2)) {
		return false;
	}
	*value = (uint16_t)(b[0] | b[1] << 8);
	return true;
}

static void
lynx_battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint16_t name = 0, status = 0, period = 0;
	int cap_lsb = 1;
	uint32_t con, dat;

	pads = ml_io_map(GPIO_PERIC1, 0x4000);
	con = rd(pads + GPP25);
	dat = rd(pads + GPP25 + 4);
	wr(pads + GPP25 + 4, dat & ~3u);                        /* what an output drives: 0 */
	wr(pads + GPP25, con & ~0xffu);                          /* both released */
	delay_for_interval(1, NSEC_PER_MSEC);
	lynx_tlog("lynx_battery: pads were con %#x dat %#x; released: SCL %d SDA %d\n", con, dat, line_in(0), line_in(1));
	if (!line_in(0) || !line_in(1)) {
		lynx_tlog("lynx_battery: the bus is not pulled up: nothing read\n");
		wr(pads + GPP25, con);
		thread_terminate(current_thread());
		return;
	}
	if (!fg(0x21, &name) || !fg(0x00, &status)) {
		lynx_tlog("lynx_battery: the fuel gauge at %#x does not answer\n", FG_ADDR);
		wr(pads + GPP25, con);
		thread_terminate(current_thread());
		return;
	}
	/* capacities count double when the gauge's task period is 351 ms (TaskPeriod 0x3C = 0x2D00): Google's max_m5_cap_lsb().
	 * Seen on the phone without it: 1861 "mAh" full where Android said 3910. */
	if (fg(0x3C, &period) && period == 0x2D00) {
		cap_lsb = 2;
	}
	lynx_tlog("lynx_battery: fuel gauge DevName %#x Status %#x TaskPeriod %#x (capacity x%d)\n", name, status, period, cap_lsb);
	for (unsigned tick = 0;; tick++) {
		uint16_t soc, vcell, current, temp, cap, full, design;
		uint8_t ok = 0, details = 0;
		bool charger = i2c_read(CHG_ADDR, 0xB4, &ok, 1) && i2c_read(CHG_ADDR, 0xB6, &details, 1);

		if (fg(0x06, &soc) && fg(0x09, &vcell) && fg(0x0A, &current) && fg(0x08, &temp) && fg(0x05, &cap) &&
		    fg(0x10, &full) && fg(0x18, &design)) {
			struct hakios_battery_state b;

			bzero(&b, sizeof(b));
			b.size = sizeof(b);
			b.percent_hundredths = soc * 100 / 256;
			b.voltage_uv = (int32_t)((uint32_t)vcell * 625 / 8);            /* 78.125 uV */
			b.current_ua = (int32_t)((int64_t)(int16_t)current * 156250 / RSENSE);
			b.temperature_hundredths = (int16_t)temp * 100 / 256;
			b.present = 1;
			b.charge_uah = (int32_t)((int64_t)cap * 500000 / RSENSE) * cap_lsb;
			b.full_uah = (int32_t)((int64_t)full * 500000 / RSENSE) * cap_lsb;
			b.design_uah = (int32_t)((int64_t)design * 500000 / RSENSE) * cap_lsb;
			if (charger) {
				unsigned dtls = details & 0xf;

				b.external = (ok >> 6) & 1;                              /* CHGIN_OK */
				b.status = !b.external ? 2 : dtls <= 3 ? 1 : dtls == 4 ? 4 : 3;
				b.usb_type = b.external ? 1 : 0;
			} else {
				b.external = b.current_ua > 0;
				b.status = b.external ? 1 : 2;
			}
			if (tick < 3 || tick % 12 == 0) {
				lynx_tlog("lynx_battery: %d.%02d%%, %d mV, %d mA, %d C, %d of %d mAh; charger INT_OK %#x DTLS_01 %#x "
				    "(%s): external %d status %d; %u failed transfers\n", b.percent_hundredths / 100,
				    b.percent_hundredths % 100, b.voltage_uv / 1000, b.current_ua / 1000,
				    b.temperature_hundredths / 100, b.charge_uah / 1000, b.full_uah / 1000, ok, details,
				    charger ? "read" : "no answer", b.external, b.status, i2c_failures);
			}
			hakios_battery_report(&b);
		} else if (tick < 3 || tick % 12 == 0) {
			lynx_tlog("lynx_battery: the fuel gauge did not answer (%u failed transfers)\n", i2c_failures);
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

static thread_call_t start_call;

static void
start_thread(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(lynx_battery_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
lynx_battery_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("lynx_battery", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	start_call = thread_call_allocate(start_thread, NULL);
	clock_interval_to_deadline(12, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start_call, deadline);
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_battery_init);
