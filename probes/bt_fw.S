/* The Bluetooth firmware of this phone's WCN6740 for the btfw probe (hpbtfw21.tlv and hpnv21.bin from its
 * /vendor/firmware, in the directory build.sh is given; not in this repository and not redistributable). */
.section .rodata
.balign 8
.global bt_patch, bt_patch_end, bt_nvm, bt_nvm_end
bt_patch:
    .incbin "hpbtfw21.tlv"
bt_patch_end:
.balign 8
bt_nvm:
    .incbin "hpnv21.bin"
bt_nvm_end:
