/* The first 512 KB of the phone's own Wi-Fi firmware (amss20.bin from its /vendor/firmware, in the directory
 * build.sh is given; not in this repository and not redistributable): the part MHI's BHI hands to the chip's
 * first loader (sbl_size = SZ_512K in cnss2 and ath11k). */
.section .rodata
.balign 4096
.global wifi_sbl, wifi_sbl_end
wifi_sbl:
    .incbin "amss20.bin", 0, 0x80000
wifi_sbl_end:
