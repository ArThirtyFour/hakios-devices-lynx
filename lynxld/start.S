/* lynxld entry from Image code1 (offset 4). Linux boot protocol: x0=FDT, EL2, MMU off. */
.section .text.boot
.global _start
_start:
    msr  daifset, #0xf
    mov  x19, x0
    ic   iallu
    dsb  sy
    mrs  x9, sctlr_el2
    orr  x9, x9, #0x1000          /* I-cache on (MMU stays off) */
    msr  sctlr_el2, x9
    isb
    movz x9, #0x0000
    movk x9, #0xfd5f, lsl #16      /* stack inside reserved ramoops (0xfd5f0000) */
    mov  sp, x9
    adr  x9, __bss_start             /* .bss lies over old kernel bytes: zero it */
    adr  x10, __bss_end
3:  cmp  x9, x10
    b.hs 4f
    strb wzr, [x9], #1
    b    3b
4:  mov  x0, x19
    adr  x1, _start
    bl   lynxld_main
1:  wfe
    b    1b

/* lynx_enter_el1(entry_pa, boot_args_pa): EL2 -> EL1h, MMU/caches off, x0 = boot_args. */
.text
.global lynx_enter_el1
lynx_enter_el1:
    msr  daifset, #0xf
    movz x9, #0x8000, lsl #16      /* HCR_EL2.RW only: AArch64 EL1, no stage 2, no traps */
    msr  hcr_el2, x9
    mov  x9, #3                    /* CNTHCTL_EL2: EL1PCTEN | EL1PCEN */
    msr  cnthctl_el2, x9
    msr  cntvoff_el2, xzr
    movz x9, #0x33ff               /* CPTR_EL2: RES1 bits, no FP/SIMD trap */
    msr  cptr_el2, x9
    msr  hstr_el2, xzr
    msr  vttbr_el2, xzr
    mrs  x9, midr_el1
    msr  vpidr_el2, x9
    mrs  x9, mpidr_el1
    msr  vmpidr_el2, x9
    mrs  x9, icc_sre_el2
    mov  x10, #0x9                 /* SRE | Enable: EL1 may use the GICv3 system registers */
    orr  x9, x9, x10
    msr  icc_sre_el2, x9
    isb
    movz x9, #0x0800
    movk x9, #0x30d0, lsl #16      /* SCTLR_EL1: RES1 bits, MMU/C/I off */
    msr  sctlr_el1, x9
    movz x9, #0x30, lsl #16        /* CPACR_EL1.FPEN = 0b11 */
    msr  cpacr_el1, x9
    isb
    ic   iallu
    tlbi alle1
    dsb  sy
    isb
    mov  x9, #0x3c5                /* EL1h, DAIF masked */
    msr  spsr_el2, x9
    msr  elr_el2, x0
    mov  x0, x1
    eret

/* XNU comes back here through its XNULDRST slot: EL1, MMU off.
 * x0 = log PA, x1 = log len, x2 = reason, x3 = ESR, x4 = ELR, x5 = FAR, x6 = LR, x7 = SP. */
.balign 64
.global lynx_return_stub
lynx_return_stub:
    msr  daifset, #0xf
    .inst 0xd500409f              /* msr PAN, #0 */
    mrs  x9, sctlr_el1
    bic  x9, x9, #0x1
    bic  x9, x9, #0x4
    msr  sctlr_el1, x9
    isb
    movz x9, #0x0000
    movk x9, #0xfd5f, lsl #16
    mov  sp, x9
    bl   lynxld_returned
2:  wfe
    b    2b

/* Secondary entry for the kernel's own core (bootcpu): PSCI CPU_ON starts it here at EL2, MMU off.
 * x0 = context id = address of struct { entry_pa, boot_args_pa }. Own stack, then the same EL1 hand-off. */
.balign 64
.global lynx_other_cpu_entry
lynx_other_cpu_entry:
    msr  daifset, #0xf
    ic   iallu
    dsb  sy
    isb
    movz x9, #0x0000
    movk x9, #0xfd5e, lsl #16      /* stack 0xfd5e0000, inside reserved ramoops, apart from core 0's */
    mov  sp, x9
    ldr  x1, [x0, #8]              /* boot_args */
    ldr  x0, [x0]                  /* entry */
    b    lynx_enter_el1
