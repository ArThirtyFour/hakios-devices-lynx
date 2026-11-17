/* Compile-only WCN6740 probe. Source snapshots alongside this file.
 * External sources (SOURCES.md says where each is; "*.source" below are those
 * files): gs201 cmucal-sfr.c, cmucal-node.c, pinctrl-gs201.c, exynos_tty.c.
 * No firmware download, PMIC writes, shared APM mux changes, or WLAN_EN writes.
 */
#include <stdint.h>
#include "log.h"

/* device-tree.dtb: /serial@181B0000, /syscon@180204E0 (offset 4),
 * /pinctrl@180D0000/uart18-bus: gpa3-0..3, function 2, PUD 0.
 * /bt-power gives GPIO pin indices. pinctrl-gs201.c:77,86,122,148
 * gives respective bank offsets 0x60,0,0x200,0.
 * GPIO register/nibble layout also tspi.c.
 */
#define UART 0x181b0000ull
#define SYS  0x180204e4ull
#define GPA3 0x180d0060ull
#define GPA6 0x180e0000ull
#define GPP16 0x10840200ull
#define GPH2 0x14440000ull
/* gs201-cmucal-sfr.c:22,188-191,644,1076,1104-1111,2950.
 * gate fields:7031-7054; divider ratio 7 bits:5847; QCH:12600-12603.
 * SYSREG gate:1060,6899-6901. Parent gate:6947-6949.
 */
#define CMU 0x18000000ull
#define DIV 0x1814
#define IP 0x20ac
#define PC 0x20b0
#define RST 0x2100
#define QCH 0x306c
#define FORCE ((1u<<20)|(1u<<21))
/* serial_s3c.h.source:19-29,35-44,114-146,168-193,224-230.
 * exynos_tty.c:3200-3220 confirms S5PV210 FIFO fields and raw fracval.
 */
#define LCON 0x00
#define CON 0x04
#define FCON 0x08
#define MCON 0x0c
#define TRSTAT 0x10
#define ERSTAT 0x14
#define FSTAT 0x18
#define MSTAT 0x1c
#define TX 0x20
#define RX 0x24
#define BRDIV 0x28
#define FRAC 0x2c
#define INTMASK 0x38
/* exynos_tty.c:284-295,2234-2254,2601-2602. */
#define USICON 0xc4
#define USIOPT 0xc8
static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a,uint32_t v) { *(volatile uint32_t *)a=v; __asm__ volatile("dsb sy":::"memory"); }
static void rmw(uint64_t a,uint32_t mask,uint32_t v) { wr(a,(rd(a)&~mask)|v); }
static uint64_t freq;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("isb; mrs %0,cntpct_el0":"=r"(t)); return t; }
static int elapsed(uint64_t t,uint32_t us) { return ticks()-t >= freq*us/1000000; }
static void wait_us(uint32_t us) { uint64_t t=ticks(); while(!elapsed(t,us)) {} }
static void stage(const char *s) { log_str(s); log_str("\n"); log_flush(); }
static void reglog(uint64_t a) { log_hex("addr",a); log_hex("value",rd(a)); }
static void banklog(uint64_t a) { reglog(a); reglog(a+4); reglog(a+8); log_flush(); }
static void uartlog(void) {
    for(unsigned i=0;i<=FRAC;i+=4) if(i!=TX && i!=RX) reglog(UART+i);
    reglog(UART+USICON); reglog(UART+USIOPT); log_flush();
}
static void lines(void) { log_hex("hostwake DAT",rd(GPA6+4)); log_hex("CTS UMSTAT",rd(UART+MSTAT)); log_flush(); }
static uint32_t parent_rate(void) {
    /* Read, never switch the shared APM source. node.c:145-155,3531-3536.
     * These are nominal fixed-rate descriptions, NOT a clock measurement.
     * PAD_CLK_APM is not established: abort instead of guessing.
     */
    unsigned m=rd(CMU+0x1000)&3, s=rd(CMU+0x1004)&3;
    if(m==0 || m==3) return 26000000;
    if(m!=1) return 0;
    if(s==0) return 394000000;
    if(s==1) return 197000000;
    if(s==2) return 50000000;
    return 99000000;
}
static int baud(uint32_t rate,uint32_t b) {
    /* exynos_tty.c:1774-1791,1872-1897; round fraction without crossing
     * integer quotient, exactly as that driver does. */
    uint32_t d=rate/b;
    if(d<16) { stage("baud unsupported by current parent"); return 0; }
    uint32_t q=d/16-1;
    if((d&15)!=15 && rate/d-b > b-rate/(d+1)) d++;
    wr(UART+CON,0); wr(UART+INTMASK,15);
    wr(UART+LCON,3); wr(UART+BRDIV,q); wr(UART+FRAC,d&15);
    wr(UART+FCON,7); wait_us(10); wr(UART+FCON,1);
    /* Manual RTS asserted, AFC off: serial_s3c.h.source:145-146.
     * hci_qca.c.source:1753-1769 resumes flow after port reopen.
     * This probe deliberately bypasses CTS gating; CTS is diagnostic only. */
    wr(UART+MCON,1); wr(UART+CON,5);
    log_hex("baud",b); log_hex("assumed ipclk Hz",rate);
    log_hex("UBRDIV",q); log_hex("UFRACVAL",d&15); log_hex("actual nominal baud",rate/d);
    log_flush(); return 1;
}
static unsigned capture(uint32_t us) {
    uint64_t t=ticks(), packed=0; unsigned count=0,n=0; uint32_t errors=0;
    while(!elapsed(t,us)) {
        /* RX count or full; every iteration checks deadline even on noise. */
        if(rd(UART+FSTAT)&0x1ff) {
            errors |= rd(UART+ERSTAT); packed=(packed<<8)|(rd(UART+RX)&255);
            count++; n++;
            if(n==8) { log_hex("rx8",packed); packed=0; n=0; }
        }
    }
    if(n) { log_hex("rx tail length",n); log_hex("rx tail",packed); }
    log_hex("rx total",count); log_hex("rx errors",errors); log_flush(); return count;
}
static int send(const uint8_t *p,unsigned n) {
    uint64_t t=ticks();
    for(unsigned i=0;i<n;i++) {
        while(rd(UART+FSTAT)&(1u<<24)) if(elapsed(t,20000)) { stage("TX FIFO timeout"); return 0; }
        wr(UART+TX,p[i]);
    }
    while(!(rd(UART+TRSTAT)&4)) if(elapsed(t,20000)) { stage("TX shift timeout"); return 0; }
    stage("TX complete"); return 1;
}
/* btqca.h.source:8,11-12; btqca.c.source:39-41. H4 command type:
 * hci.h.source:492; HCI_Reset opcode:1130; zero parameters per task. */
static const uint8_t version[]={1,0,0xfc,1,0x19};
static const uint8_t reset[]={1,3,0x0c,0};
static unsigned exchange(void) {
    stage("unsolicited drain 30ms"); capture(30000);
    stage("TX H4 01 00 fc 01 19"); send(version,sizeof(version)); unsigned n=capture(300000);
    stage("TX H4 01 03 0c 00"); send(reset,sizeof(reset)); n+=capture(300000); lines(); return n;
}
void lynxld_main(uint64_t fdt,uint64_t self) {
    (void)fdt; (void)self; __asm__ volatile("mrs %0,cntfrq_el0":"=r"(freq));
    log_init(); stage("BTPROBE v2");
    reglog(CMU+0x1000); reglog(CMU+0x1004); reglog(CMU+0x2084);
    reglog(CMU+DIV); reglog(CMU+IP); reglog(CMU+PC); reglog(CMU+RST); reglog(CMU+QCH); reglog(CMU+0x211c); log_flush();
    banklog(GPP16); banklog(GPH2); banklog(GPA6); banklog(GPA3);
    uint32_t c16=rd(GPP16),ch=rd(GPH2),ca=rd(GPA6),cu=rd(GPA3);
    uint32_t pa=rd(GPA6+8),pu=rd(GPA3+8),dw=rd(GPP16+4);
    uint32_t ip=rd(CMU+IP),pc=rd(CMU+PC),rs=rd(CMU+RST),qc=rd(CMU+QCH),dv=rd(CMU+DIV),sg=rd(CMU+0x211c);
    uint32_t rate=parent_rate(); int enabled=0;
    if(!rate) { stage("ABORT unknown PAD clock source"); goto end; }
    /* Parent gate must already be running for live APM; do not alter it. */
    if(!(rd(CMU+0x2084)&(1u<<21))) { stage("ABORT parent CG_VAL off"); goto end; }
    stage("opening UART and SYSREG gates before register reads");
    rmw(CMU+IP,0,FORCE); rmw(CMU+PC,0,FORCE); rmw(CMU+RST,0,FORCE); rmw(CMU+0x211c,0,FORCE);
    rmw(CMU+QCH,1,2); wait_us(100);
    if((rd(CMU+IP)&FORCE)!=FORCE || (rd(CMU+PC)&FORCE)!=FORCE || (rd(CMU+RST)&FORCE)!=FORCE || (rd(CMU+0x211c)&FORCE)!=FORCE) { stage("gate readback failed"); goto clocks; }
    stage("ABL UART registers (after gate forcing)"); reglog(SYS); uartlog();
    uint32_t sys=rd(SYS);
    /* Parent review: Android's debugfs on this phone says MUX_APM_FUNC 394000000, DOUT_CLK_APM_USI1_UART 197000000:
     * the divider runs at /2 (field 1, as the vclk LUT's parameter 1). Same here, not /1: 394 MHz into the UART is untried. */
    rmw(CMU+DIV,0x7f,1); rate/=2;
    uint64_t t=ticks(); while(rd(CMU+DIV)&(1u<<16)) if(elapsed(t,10000)) { stage("DIV busy timeout"); goto clocks; }
    rmw(SYS,7,1); wr(UART+USICON,1); wait_us(1); wr(UART+USICON,0); wait_us(1); wr(UART+USIOPT,2);
    rmw(GPA3,0xffff,0x2222); rmw(GPA3+8,0xffff,0);
    if(!baud(rate,115200)) goto tidy;
    uartlog(); lines();
    /* /bt-power default pinctrl: hostwake input / pull-down.
     * btpower.c.source:635-665: BT_EN low 100ms, XO high 5-7ms,
     * BT_EN high 50ms; 601-603: BT_WAKE high afterwards.
     * hci_qca.c.source:1817-1821 likewise uses 50ms after BT_EN.
     */
    rmw(GPA6,15u<<24,0); rmw(GPA6+8,15u<<24,1u<<24);
    rmw(GPP16+4,(1u<<2)|(1u<<3),0); rmw(GPP16,0xff00,0x1100);
    enabled=1; stage("BT_EN low 100ms"); wait_us(100000);
    rmw(GPH2+4,0,1u<<5); rmw(GPH2,15u<<20,1u<<20); wait_us(6000);
    rmw(GPP16+4,0,1u<<2); stage("BT_EN high 50ms"); wait_us(50000);
    rmw(GPP16+4,0,1u<<3); lines();
    if(!exchange()) {
        /* hci_qca.c.source:2101-2102: init 115200, operational 3000000.
         * Diagnostic fallback only: Linux first negotiates this speed. */
        stage("no response: diagnostic 3000000 try (not ROM default)");
        if(baud(rate,3000000)) exchange();
    }
tidy:
    stage("tidy");
    if(enabled) { rmw(GPP16+4,12,0); wait_us(50000); wait_us(5000); rmw(GPH2+4,1u<<5,0); }
    wr(UART+CON,0); wr(UART+MCON,0); wr(SYS,sys);
    rmw(GPP16+4,1u<<3,dw&(1u<<3));
    rmw(GPP16,0xff00,c16&0xff00); rmw(GPH2,15u<<20,ch&(15u<<20));
    rmw(GPA6,15u<<24,ca&(15u<<24)); rmw(GPA6+8,15u<<24,pa&(15u<<24));
    rmw(GPA3,0xffff,cu&0xffff); rmw(GPA3+8,0xffff,pu&0xffff);
clocks:
    wr(CMU+DIV,dv); wr(CMU+QCH,qc); wr(CMU+RST,rs); wr(CMU+IP,ip); wr(CMU+PC,pc); wr(CMU+0x211c,sg);
end:
    stage("BTPROBE-END"); for(;;) __asm__ volatile("wfe");
}
void lynxld_returned(void) {}
