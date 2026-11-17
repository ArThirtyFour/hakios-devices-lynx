/* The Bluetooth firmware of this phone's WCN6740, pulled from its own /vendor/firmware into the directory
 * build.sh is given (FW, default ../fw-lynx). Not in this repository and not redistributable. */
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
