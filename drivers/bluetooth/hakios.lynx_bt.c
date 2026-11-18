/*
 * Bluetooth of the Pixel 7a (lynx): the Bluetooth half of its Qualcomm WCN6740 on the UART of APM USI1
 * (0x181B0000, serial@181B0000 in the phone's FDT), H4 at 115200 with the UART's automatic flow control.
 * Only with boot-arg lynx_bt=1, which the loader gives when it has started the chip.
 *
 * The loader (lynxld, bt_up.c) has done everything that needs the firmware files: clocks, pins, power (BT_EN, the
 * 0.95 V enable), the rampatch and the NVM with the phone's own address in it, HCI_Reset. This driver finds the chip
 * running and is its transport: whole H4 packets to and from it. What is above the transport is HakiOS's own
 * (drivers/bluetooth, BT_HOST: the kernel control and the Skywalk pipe of iOS's bluetoothd, the registry's
 * "bluetooth"), which this gives its operations to (hakios_bthost_start).
 *
 *   lynx_bt=1         the controller for bluetoothd (the image needs bluetoothd in it: userland/build.sh BLUETOOTH=1)
 *   lynx_bt_scan=N    before that, N seconds of passive LE scanning said in the log: proof that the radio hears,
 *                     whatever bluetoothd then makes of the controller (default 3; 0: none)
 *
 * Vendor commands: bluetoothd's are Broadcom's, for the chips of Apple's phones (OGF 0x3f: 0xfce2, 0xfd48 ...). On
 * this chip the same numbers are Qualcomm's and mean other things -- 0xfc00 is its firmware download, 0xfc0b writes
 * its NVM, 0xfc48 changes the baud rate -- so NONE of them reaches the chip: each is said in the log and answered
 * here "unknown command", the answer BT_HOST already knows how to turn into what bluetoothd's parsers take
 * (hakios.bthost.c, skel_quirks).
 *
 * NOT YET: the UART stays at 115200 baud (11 KB/s: enough for LE, keyboards, pairing; too slow for music), it is
 * polled (no interrupt), and the chip is never switched off.
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
#include <pexpert/device_tree.h>
#include <sys/errno.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

/* drivers/bluetooth/hakios.bthost.c: what a phone's driver gives it (the comments on each are there) */
struct hakios_bt_platform {
	const char      *name;
	int     (*power_on)(void);
	int     (*function_on)(void);
	void    (*function_off)(void);
	int     (*send)(const uint8_t *h4, unsigned length);
	int     (*receive)(uint8_t *h4, unsigned room, unsigned wait_ms);
	void    (*wake)(void);
	int     (*wifi_address)(uint8_t out[6]);
	bool    wakes;
};
extern void hakios_bthost_start(const struct hakios_bt_platform *platform);
extern void hakios_bthost_publish(const struct hakios_bt_platform *platform);

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

/* a byte from the chip before that time: 1 with it, 0 without */
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
		delay_for_interval(1, NSEC_PER_MSEC);
	}
}

/*
 * One whole H4 packet from the chip, its type byte first (4 event, 2 ACL, 3 SCO, 5 ISO): its length, 0 when none
 * began within wait_ms, less than 0 when one began and did not end (the stream is then out of step; what follows
 * is dropped byte by byte until a type byte, and counted).
 */
static int
h4_read(uint8_t *h4, unsigned room, uint32_t wait_ms)
{
	uint64_t until = now_ms() + wait_ms;
	uint32_t header, body;

	for (;;) {
		if (!rx_byte(&h4[0], until)) {
			return 0;
		}
		if (h4[0] >= 2 && h4[0] <= 5) {
			break;
		}
		rx_stray++;
	}
	header = (h4[0] == 4) ? 2 : (h4[0] == 3 ? 3 : 4);
	until = now_ms() + 300;
	for (uint32_t i = 0; i < header; i++) {
		if (!rx_byte(&h4[1 + i], until)) {
			return -EIO;
		}
	}
	switch (h4[0]) {
	case 4:  body = h4[2]; break;
	case 3:  body = h4[3]; break;
	case 5:  body = (uint32_t)(h4[3] | h4[4] << 8) & 0x3fff; break;
	default: body = (uint32_t)(h4[3] | h4[4] << 8); break;
	}
	if (1 + header + body > room) {
		return -EMSGSIZE;
	}
	until = now_ms() + 300 + body / 8;      /* 115200 baud is 11 bytes a millisecond */
	for (uint32_t i = 0; i < body; i++) {
		if (!rx_byte(&h4[1 + header + i], until)) {
			return -EIO;
		}
	}
	return (int)(1 + header + body);
}

static bool
h4_write(const uint8_t *data, uint32_t length)
{
	uint64_t until = now_ms() + 1000;

	for (uint32_t i = 0; i < length; i++) {
		while (rd(uart + UFSTAT) & (1u << 24)) {
			if (now_ms() >= until) {
				return false;           /* the chip holds its RTS */
			}
			delay_for_interval(1, NSEC_PER_MSEC);
		}
		wr(uart + UTXH, data[i]);
		until = now_ms() + 1000;
	}
	return true;
}

/* ---- a look at the air, before anything is given to iOS ---- */
static uint8_t pkt[1100];
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

/*
 * A command and its Command Complete, in pkt[] as it came (04 0e length packets opcode-low opcode-high status ...):
 * its length, or -1.
 */
static int
command(const char *name, uint16_t opcode, const uint8_t *parameters, uint8_t length)
{
	static uint8_t cmd[4 + 255];

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
		int n = h4_read(pkt, sizeof(pkt), 1000);

		if (n <= 0) {
			break;
		}
		if (pkt[0] == 0x04 && pkt[1] == 0x0e && n >= 7 && pkt[4] == cmd[1] && pkt[5] == cmd[2]) {
			if (pkt[6] != 0) {
				lynx_tlog("lynx_bt: %s: status %#x\n", name, pkt[6]);
			}
			return pkt[6] == 0 ? n : -1;
		}
		if (pkt[0] == 0x04 && pkt[1] == 0x3e && n >= 5 && pkt[3] == 0x02) {
			advertising(pkt + 4, (uint32_t)n - 4);
		}
	}
	lynx_tlog("lynx_bt: %s: no answer\n", name);
	return -1;
}

static void
look_at_the_air(uint32_t seconds)
{
	static const uint8_t event_mask[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f };  /* with the LE Meta event */
	static const uint8_t scan_parameters[7] = { 0x00, 0x60, 0x00, 0x30, 0x00, 0x00, 0x00 };   /* passive, 60 ms / 30 ms */
	static const uint8_t scan_on[2] = { 0x01, 0x00 }, scan_off[2] = { 0x00, 0x00 };
	uint64_t until;

	if (command("Set_Event_Mask", 0x0c01, event_mask, sizeof(event_mask)) < 0 ||
	    command("LE_Set_Scan_Parameters", 0x200b, scan_parameters, sizeof(scan_parameters)) < 0 ||
	    command("LE_Set_Scan_Enable", 0x200c, scan_on, sizeof(scan_on)) < 0) {
		lynx_tlog("lynx_bt: scanning could not be started\n");
		return;
	}
	until = now_ms() + seconds * 1000ull;
	while (now_ms() < until) {
		int n = h4_read(pkt, sizeof(pkt), 200);

		if (n >= 5 && pkt[0] == 0x04 && pkt[1] == 0x3e && pkt[3] == 0x02) {
			advertising(pkt + 4, (uint32_t)n - 4);
		}
	}
	command("LE_Set_Scan_Enable (off)", 0x200c, scan_off, sizeof(scan_off));
	lynx_tlog("lynx_bt: scan: %u advertising reports from %u devices in %u s (%#x receive errors, %u stray bytes)\n",
	    adv_reports, adv_devices, seconds, rx_errors, rx_stray);
}

/* ---- the operations BT_HOST asks for ---- */

/* the answers made here (to bluetoothd's vendor commands), waiting for the next receive: a ring of 7-byte events */
#define OURS    32u
static uint8_t ours[OURS][7];
static volatile uint32_t ours_in, ours_out;     /* one thread, BT_HOST's, calls both send and receive */

static int
lynx_bt_power_on(void)
{
	int n;

	/* (the loader did the power, the clocks and the firmware; here only what it left is looked at) */
	lynx_tlog("lynx_bt: UART as the loader left it: ULCON %#x UCON %#x UFCON %#x UMCON %#x UFSTAT %#x UMSTAT %#x UBRDIV %#x\n",
	    rd(uart + 0x00), rd(uart + 0x04), rd(uart + 0x08), rd(uart + 0x0c), rd(uart + UFSTAT), rd(uart + 0x1c), rd(uart + 0x28));
	while (h4_read(pkt, sizeof(pkt), 50) != 0) {
	}
	n = command("Read_Local_Version", 0x1001, NULL, 0);
	if (n < 15 || pkt[6] != 0) {
		lynx_tlog("lynx_bt: the chip does not answer: no Bluetooth (did the loader start it?)\n");
		return ENXIO;
	}
	lynx_tlog("lynx_bt: controller HCI %u.%04x, LMP %u.%04x, manufacturer %#06x\n", pkt[7], pkt[8] | pkt[9] << 8, pkt[10],
	    pkt[13] | pkt[14] << 8, pkt[11] | pkt[12] << 8);
	n = command("Read_BD_ADDR", 0x1009, NULL, 0);
	if (n >= 13 && pkt[6] == 0) {
		lynx_tlog("lynx_bt: address %02x:%02x:%02x:%02x:%02x:%02x\n", pkt[12], pkt[11], pkt[10], pkt[9], pkt[8], pkt[7]);
	}
	return 0;
}

static int
lynx_bt_function_on(void)
{
	uint32_t seconds = 3;

	PE_parse_boot_argn("lynx_bt_scan", &seconds, sizeof(seconds));
	if (seconds != 0 && seconds <= 60) {
		look_at_the_air(seconds);
	}
	return 0;       /* (BT_HOST's own first command is HCI_Reset: what the scan set is gone by then) */
}

static void
lynx_bt_function_off(void)
{
	/* the chip stays on: its power is the loader's (BT_EN), and nothing here can start it again */
}

static int
lynx_bt_send(const uint8_t *h4, unsigned length)
{
	/* t3: Apple's Write LE Host Support (01 01) gets status 0x11 on WCN6740.
	 * Use LE enabled with simultaneous-host support clear, as Linux's
	 * hci_write_le_host_supported_sync(hdev, 0x01, 0x00). Keep the real reply.
	 */
	if (length == 6 && memcmp(h4, "\x01\x6d\x0c\x02\x01\x01", 6) == 0) {
		static const uint8_t le_host[] = { 0x01, 0x6d, 0x0c, 0x02, 0x01, 0x00 };

		lynx_tlog("lynx_bt: Write LE Host Support: 01 01 -> 01 00\n");
		return h4_write(le_host, sizeof(le_host)) ? 0 : EIO;
	}
	if (length >= 4 && h4[0] == 0x01 && (h4[2] >> 2) == 0x3f) {
		/* a vendor command: Broadcom's number, which is another command on this chip. Not sent; "unknown command" */
		static uint32_t said;
		uint8_t *answer = ours[ours_in % OURS];

		if (said < 96) {
			char text[3 * 16 + 1] = "";
			unsigned n = length - 4 < 16 ? length - 4 : 16;

			for (unsigned i = 0; i < n; i++) {
				snprintf(text + 3 * i, 4, " %02x", h4[4 + i]);
			}
			said++;
			lynx_tlog("lynx_bt: vendor command %#06x (%u bytes:%s%s) not given to the chip\n", h4[1] | h4[2] << 8, length - 4,
			    text, length - 4 > n ? " ..." : "");
		}
		if (ours_in - ours_out >= OURS) {
			return ENOBUFS;
		}
		answer[0] = 0x04;       /* Command Complete, one more command may be sent, the opcode, status 1 */
		answer[1] = 0x0e;
		answer[2] = 0x04;
		answer[3] = 0x01;
		answer[4] = h4[1];
		answer[5] = h4[2];
		answer[6] = 0x01;
		ours_in++;
		return 0;
	}
	return h4_write(h4, length) ? 0 : EIO;
}

static int
lynx_bt_receive(uint8_t *h4, unsigned room, unsigned wait_ms)
{
	static uint32_t said;
	int n;

	if (ours_in != ours_out && room >= 7) {
		memcpy(h4, ours[ours_out % OURS], 7);
		ours_out++;
		return 7;
	}
	n = h4_read(h4, room, wait_ms);
	/*
	 * t15: the identity the daemon can place. Our BlueTool answers as Broadcom ("BCM4383A3"), and Apple's stack
	 * has a platform/chipset table with no Qualcomm entry at all: a controller that answers manufacturer 0x001d
	 * is "an unknown platform/chipset/manufacturer combination" and is not powered (the toggle springs back).
	 * What the chip says of itself is turned into the answer the same BlueTool claims -- Broadcom, LMP 5.3,
	 * subversion 0x4383. Nothing depends on the claim beyond this: the vendor commands that follow are answered
	 * here (the filter below) or in BT_HOST's quirks, and are never given to the chip.
	 */
	if (n >= 15 && h4[0] == 0x04 && h4[1] == 0x0e && h4[4] == 0x01 && h4[5] == 0x10 && h4[6] == 0x00 &&
	    (h4[11] != 0x0f || h4[12] != 0x00)) {
		static uint32_t said;

		if (said++ < 4) {
			lynx_tlog("lynx_bt: Read Local Version: manufacturer %#06x -> 0x000f (Broadcom, as BlueTool says)\n",
			    h4[11] | h4[12] << 8);
		}
		h4[11] = 0x0f;
		h4[12] = 0x00;
		h4[13] = 0x83;
		h4[14] = 0x43;
	}
	if (n < 0) {
		if (said < 16) {
			said++;
			lynx_tlog("lynx_bt: a packet of type %u began and did not end (%d; %#x receive errors, %u stray bytes)\n", h4[0], n,
			    rx_errors, rx_stray);
		}
		return 0;       /* (not an error BT_HOST can do anything about; the stream finds its step again by itself) */
	}
	return n;
}

/*
 * The address for the registry. BT_HOST asks for the Wi-Fi radio's and makes Bluetooth's of it, one up; this phone
 * has an address of its own for each (the loader's /chosen: mac-address-bluetooth0, from the bootloader's tree, and
 * the same one is in the chip's NVM), so what is given is Bluetooth's, one down.
 */
static int
lynx_bt_wifi_address(uint8_t out[6])
{
	DTEntry chosen;
	const void *mac = NULL;
	unsigned int size = 0;

	if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess ||
	    SecureDTGetProperty(chosen, "mac-address-bluetooth0", &mac, &size) != kSuccess || size != 6) {
		return ENOENT;
	}
	memcpy(out, mac, 6);
	out[5] = (uint8_t)(out[5] - 1);
	return 0;
}

static const struct hakios_bt_platform lynx_bt_platform = {
	.name = "gs201 / lynx / WCN6740",
	.power_on = lynx_bt_power_on,
	.function_on = lynx_bt_function_on,
	.function_off = lynx_bt_function_off,
	.send = lynx_bt_send,
	.receive = lynx_bt_receive,
	.wifi_address = lynx_bt_wifi_address,
	.wakes = false,
};

static void
lynx_bt_publish_thread(__unused void *parameter, __unused wait_result_t result)
{
	hakios_bthost_publish(&lynx_bt_platform);
}

static void
lynx_bt_bridge_thread(__unused void *parameter, __unused wait_result_t result)
{
	hakios_bthost_start(&lynx_bt_platform);
}

/* the registry's service a second into the boot (bluetoothd leaves for good without it), the controller at 20 s */
static void
lynx_bt_call(thread_call_param_t which, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(which != NULL ? lynx_bt_bridge_thread : lynx_bt_publish_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
lynx_bt_init(void)
{
	static thread_call_t publish, start;
	uint32_t enabled = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("lynx_bt", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	uart = ml_io_map(UART_PHYS, 0x1000);
	if (!uart) {
		return;
	}
	publish = thread_call_allocate(lynx_bt_call, NULL);
	clock_interval_to_deadline(1, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(publish, deadline);
	start = thread_call_allocate(lynx_bt_call, (thread_call_param_t)1);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_bt_init);
