include '../lib/io.asm'

.stacktop:   equ 0   ; end of RAM + 1


    ;###################################################
    ; STARTING HERE, WE ARE RUNNING FROM RAM
    ;###################################################

    ld      sp,.stacktop
    call    sioa_init
    call    .helloa

.loop:
    HALT
    jp      .loop

;##############################################################
; Print 'Hello' on SIO_A
;##############################################################
.helloa:
    ld      c,0x0D
    call    sioa_tx_char
    ld      c,0x0A
    call    sioa_tx_char
    ld      c,'H'
    call    sioa_tx_char
    ld      c,'e'
    call    sioa_tx_char
    ld      c,'l'
    call    sioa_tx_char
    call    sioa_tx_char
    ld      c,'o'
    call    sioa_tx_char
    ld      c,0x0D
    call    sioa_tx_char
    ld      c,0x0A
    call    sioa_tx_char
    ret

include '../lib/sio.asm'
