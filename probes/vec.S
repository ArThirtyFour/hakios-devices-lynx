/* lynx probes: EL2 exception vectors that report instead of hanging.
 * A synchronous exception from the probe itself (current EL, SP_ELx) calls probe_fault(esr, elr, far) with the
 * caller-saved registers kept, then returns to the instruction AFTER the faulting one: a faulting load or store is
 * skipped and the probe goes on (it looks at the flag probe_fault() sets). Anything else parks the core. */
.text
.balign 2048
.global probe_vectors
probe_vectors:
    .balign 128             /* current EL, SP_EL0: sync / irq / fiq / serror */
    b    park
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128             /* current EL, SP_ELx: sync */
    b    sync_here
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128             /* SError */
    b    serror_here
    .balign 128             /* lower EL, AArch64 */
    b    park
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128             /* lower EL, AArch32 */
    b    park
    .balign 128
    b    park
    .balign 128
    b    park
    .balign 128
    b    park

sync_here:
    sub  sp, sp, #176
    stp  x0, x1, [sp, #0]
    stp  x2, x3, [sp, #16]
    stp  x4, x5, [sp, #32]
    stp  x6, x7, [sp, #48]
    stp  x8, x9, [sp, #64]
    stp  x10, x11, [sp, #80]
    stp  x12, x13, [sp, #96]
    stp  x14, x15, [sp, #112]
    stp  x16, x17, [sp, #128]
    stp  x18, x29, [sp, #144]
    str  x30, [sp, #160]
    mrs  x0, esr_el2
    mrs  x1, elr_el2
    mrs  x2, far_el2
    bl   probe_fault
    mrs  x1, elr_el2
    add  x1, x1, #4
    msr  elr_el2, x1
    ldp  x0, x1, [sp, #0]
    ldp  x2, x3, [sp, #16]
    ldp  x4, x5, [sp, #32]
    ldp  x6, x7, [sp, #48]
    ldp  x8, x9, [sp, #64]
    ldp  x10, x11, [sp, #80]
    ldp  x12, x13, [sp, #96]
    ldp  x14, x15, [sp, #112]
    ldp  x16, x17, [sp, #128]
    ldp  x18, x29, [sp, #144]
    ldr  x30, [sp, #160]
    add  sp, sp, #176
    eret

serror_here:
    mrs  x0, esr_el2
    mrs  x1, elr_el2
    bl   probe_serror
park:
    wfe
    b    park

/* probe_smc(x0..x4): one SMC, returns x0 */
.global probe_smc
probe_smc:
    smc  #0
    ret
