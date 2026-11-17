/*
 * Bluetooth of the Pixel 7a (lynx): the Bluetooth half of its Qualcomm WCN6740 on the UART of APM USI1
 * (0x181B0000, serial@181B0000 in the phone's FDT), H4 at 115200 with the UART's automatic flow control.
 * Only with boot-arg lynx_bt=1, which the loader gives when it has started the chip.
 *
 * STEP ONE: the transport, and a look at the air. The loader (lynxld, bt_up.c) has done everything that needs the
 * firmware files: clocks, pins, power (BT_EN, the 0.95 V enable), the rampatch and the NVM, HCI_Reset. This driver
 * finds the chip running and talks to it: its version and address, then eight seconds of passive LE scanning, and
 * says in the log how many advertising reports came from how many devices. That is all it does with the radio.
 *
 * What is NOT here: anything Darwin can use. bluetoothd of this system looks for a serial port /dev/cu.bluetooth or
 * /dev/uart.bluetooth (its own strings) and is not started at all by this port (userland/mkjobs.py DENY); a tty of
 * that name over this transport, and what bluetoothd then asks of a chip that is not the Broadcom one it was built
 * for, are the next steps.
 *
 * Register layout: Google's kernel/gs drivers/tty/serial/exynos_tty.c and include/linux/serial_s3c.h (ULCON 0x00,
 * UCON 0x04, UFCON 0x08, UMCON 0x0c, UTRSTAT 0x10, UERSTAT 0x14, UFSTAT 0x18: RX count 7:0 and full bit 8, TX full
 * bit 24; UTXH 0x20, URXH 0x24), as used by the loader's probes on this phone (probes/btprobe.c, probes/btfw.c).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

#define UART_PHYS       0x181B0000ull
#define UFSTAT          0x18
#define UERSTAT         0x14
#define UTXH            0x20
#define URXH            0x24

static vm_offset_t uart;
static uint32_t rx_errors, rx_stray;

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000000;
}

/* a byte from the chip within that long: 1 with it, 0 without */
static int
rx_byte(uint8_t *b, uint64_t until_ms)
{
	for (;;) {
		if (rd(uart + UFSTAT) & 0x1ff) {
			rx_errors |= rd(uart + UERSTAT) & 0xf;
			*b = (uint8_t)rd(uart + URXH);
			return 1;
		}
		if (now_ms() >= until_ms) {
			return 0;
		}
		delay_for_interval(2, NSEC_PER_MSEC);
	}
}

static uint8_t pkt[4 + 1024];

/*
 * One H4 packet from the chip into pkt[] without its type byte; *type gets that byte. Its length, or -1 when
 * nothing whole came in time. Events (4): code, length, parameters. ACL data (2): handle, 16-bit length, data.
 */
static int
h4_read(uint8_t *type, uint32_t wait_ms)
{
	uint64_t until = now_ms() + wait_ms;
	uint32_t header, body;
	uint8_t b;

	for (;;) {
		if (!rx_byte(&b, until)) {
			return -1;
		}
		if (b == 0x04 || b == 0x02) {
			break;
		}
		rx_stray++;
	}
	*type = b;
	header = b == 0x04 ? 2 : 4;
	for (uint32_t i = 0; i < header; i++) {
		if (!rx_byte(&pkt[i], until + 200)) {
			return -1;
		}
	}
	body = b == 0x04 ? pkt[1] : (uint32_t)(pkt[2] | pkt[3] << 8);
	if (body > sizeof(pkt) - 4) {
		return -1;
	}
	for (uint32_t i = 0; i < body; i++) {
		if (!rx_byte(&pkt[header + i], until + 500)) {
			return -1;
		}
	}
	return (int)(header + body);
}

static bool
h4_write(const uint8_t *data, uint32_t length)
{
	uint64_t until = now_ms() + 500;

	for (uint32_t i = 0; i < length; i++) {
		while (rd(uart + UFSTAT) & (1u << 24)) {
			if (now_ms() >= until) {
				return false;           /* the chip holds its RTS */
			}
			delay_for_interval(1, NSEC_PER_MSEC);
		}
		wr(uart + UTXH, data[i]);
	}
	return true;
}

static uint32_t adv_reports, adv_devices;
static uint8_t adv_seen[32][6];

/* an LE advertising report (LE Meta event, subevent 2): counted, the first of each device shown */
static void
advertising(const uint8_t *p, uint32_t length)
{
	uint32_t count = p[0], at = 1;

	for (uint32_t r = 0; r < count; r++) {
		char name[32] = "";
		const uint8_t *addr;
		uint32_t dlen;
		bool known = false;

		if (at + 9 > length) {
			return;
		}
		addr = p + at + 2;
		dlen = p[at + 8];
		if (at + 9 + dlen + 1 > length) {
			return;
		}
		adv_reports++;
		for (uint32_t i = 0; i < adv_devices && i < 32; i++) {
			known |= memcmp(adv_seen[i], addr, 6) == 0;
		}
		if (!known) {
			for (uint32_t a = 0; a + 1 < dlen;) {           /* the device's name, if it sends one */
				uint32_t alen = p[at + 9 + a], atype = p[at + 9 + a + 1];

				if (alen == 0 || a + 1 + alen > dlen) {
					break;
				}
				if (atype == 0x09 || atype == 0x08) {
					uint32_t n = alen - 1 < sizeof(name) - 1 ? alen - 1 : (uint32_t)sizeof(name) - 1;

					for (uint32_t i = 0; i < n; i++) {
						uint8_t c = p[at + 9 + a + 2 + i];

						name[i] = (c >= 32 && c < 127) ? (char)c : '.';
					}
					name[n] = 0;
				}
				a += 1 + alen;
			}
			if (adv_devices < 32) {
				memcpy(adv_seen[adv_devices], addr, 6);
			}
			if (adv_devices < 12) {
				lynx_tlog("lynx_bt: heard %02x:%02x:%02x:%02x:%02x:%02x (%s address), %d dBm%s%s\n", addr[5], addr[4],
				    addr[3], addr[2], addr[1], addr[0], p[at + 1] ? "random" : "public", (int8_t)p[at + 9 + dlen],
				    name[0] ? ", name " : "", name);
			}
			adv_devices++;
		}
		at += 9 + dlen + 1;
	}
}

/* a command and its Command Complete: the event's length in pkt[] (code, length, packets, opcode, status ...), or -1 */
static int
command(const char *name, uint16_t opcode, const uint8_t *parameters, uint8_t length)
{
	static uint8_t cmd[4 + 255];
	uint8_t type;

	cmd[0] = 0x01;
	cmd[1] = (uint8_t)opcode;
	cmd[2] = (uint8_t)(opcode >> 8);
	cmd[3] = length;
	for (uint32_t i = 0; i < length; i++) {
		cmd[4 + i] = parameters[i];
	}
	if (!h4_write(cmd, 4u + length)) {
		lynx_tlog("lynx_bt: %s: the chip does not take the command\n", name);
		return -1;
	}
	for (int i = 0; i < 16; i++) {
		int n = h4_read(&type, 1000);

		if (n < 0) {
			break;
		}
		if (type == 0x04 && pkt[0] == 0x0e && n >= 6 && pkt[3] == cmd[1] && pkt[4] == cmd[2]) {
			if (pkt[5] != 0) {
				lynx_tlog("lynx_bt: %s: status %#x\n", name, pkt[5]);
			}
			return n;
		}
		if (type == 0x04 && pkt[0] == 0x3e && n >= 4 && pkt[2] == 0x02) {
			advertising(pkt + 3, (uint32_t)n - 3);
		}
	}
	lynx_tlog("lynx_bt: %s: no answer\n", name);
	return -1;
}

static void
lynx_bt_thread(__unused void *parameter, __unused wait_result_t result)
{
	static const uint8_t event_mask[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f };  /* with the LE Meta event */
	static const uint8_t scan_parameters[7] = { 0x00, 0x60, 0x00, 0x30, 0x00, 0x00, 0x00 };   /* passive, 60 ms / 30 ms */
	static const uint8_t scan_on[2] = { 0x01, 0x00 }, scan_off[2] = { 0x00, 0x00 };
	uint64_t until;
	uint8_t type;
	int n;

	uart = ml_io_map(UART_PHYS, 0x1000);
	lynx_tlog("lynx_bt: UART as the loader left it: ULCON %#x UCON %#x UFCON %#x UMCON %#x UFSTAT %#x UMSTAT %#x UBRDIV %#x\n",
	    rd(uart + 0x00), rd(uart + 0x04), rd(uart + 0x08), rd(uart + 0x0c), rd(uart + UFSTAT), rd(uart + 0x1c), rd(uart + 0x28));
	while (h4_read(&type, 50) >= 0) {
	}
	n = command("Read_Local_Version", 0x1001, NULL, 0);
	if (n < 14 || pkt[5] != 0) {
		lynx_tlog("lynx_bt: the chip does not answer: no Bluetooth (did the loader start it?)\n");
		thread_terminate(current_thread());
		return;
	}
	lynx_tlog("lynx_bt: controller HCI %u.%04x, LMP %u.%04x, manufacturer %#06x\n", pkt[6], pkt[7] | pkt[8] << 8, pkt[9],
	    pkt[12] | pkt[13] << 8, pkt[10] | pkt[11] << 8);
	n = command("Read_BD_ADDR", 0x1009, NULL, 0);
	if (n >= 12 && pkt[5] == 0) {
		lynx_tlog("lynx_bt: address %02x:%02x:%02x:%02x:%02x:%02x\n", pkt[11], pkt[10], pkt[9], pkt[8], pkt[7], pkt[6]);
	}

	/* a look at the air: passive LE scanning for eight seconds */
	if (command("Set_Event_Mask", 0x0c01, event_mask, sizeof(event_mask)) < 0 ||
	    command("LE_Set_Scan_Parameters", 0x200b, scan_parameters, sizeof(scan_parameters)) < 0 ||
	    command("LE_Set_Scan_Enable", 0x200c, scan_on, sizeof(scan_on)) < 0) {
		lynx_tlog("lynx_bt: scanning could not be started\n");
	} else {
		until = now_ms() + 8000;
		while (now_ms() < until) {
			n = h4_read(&type, 200);
			if (n >= 4 && type == 0x04 && pkt[0] == 0x3e && pkt[2] == 0x02) {
				advertising(pkt + 3, (uint32_t)n - 3);
			}
		}
		command("LE_Set_Scan_Enable (off)", 0x200c, scan_off, sizeof(scan_off));
		lynx_tlog("lynx_bt: RADIO WORKS: %u advertising reports from %u devices in 8 s (%#x receive errors, %u stray bytes)\n",
		    adv_reports, adv_devices, rx_errors, rx_stray);
	}
	for (;;) {                      /* nothing asks for more yet: whatever the chip says by itself is read and dropped */
		h4_read(&type, 1000);
	}
}

static thread_call_t start_call;

static void
start_thread(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(lynx_bt_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
lynx_bt_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("lynx_bt", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	start_call = thread_call_allocate(start_thread, NULL);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start_call, deadline);
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_bt_init);
