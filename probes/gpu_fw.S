/* The Mali-G710's CSF firmware as the phone's Android loads it (mali_csffw-r54p3.bin from its /vendor/firmware, in
 * the directory build.sh is given; Arm's, not in this repository and not redistributable). */
.section .rodata
.balign 8
.global gpu_fw, gpu_fw_end
gpu_fw:
    .incbin "mali_csffw-r54p3.bin"
gpu_fw_end:
