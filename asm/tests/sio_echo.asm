include '../lib/io.asm'

.stacktop:   equ 0   ; end of RAM + 1

    ;###################################################
    ; STARTING HERE, WE ARE RUNNING FROM RAM
    ;###################################################

    ld      sp,.stacktop
    call    sioa_init


.loop:
    call    .spew_line
    call    .echo_char
    jp      .loop

;##############################################################
; Echo characters from SIO back after adding one.
;##############################################################
.echo_char:
    call    sioa_rx_char    ; get a character from the SIO
    ld      c,a
    ;inc    c               ; add 1 (A becomes B, ...)
    call    sioa_tx_char    ; print the character
    ret

;##############################################################
; Print all printable characters in an endless loop on SIO A
;##############################################################
.spew_line:
    ; start a new line
    ld      c,0x0D
    call    sioa_tx_char
    ld      c,0x0A
    call    sioa_tx_char


    ld      c,0x20      ; ascii space character
.spew_loop1:
    call    sioa_tx_char
    inc     c
    ld      a,0x7f      ; last graphic character + 1
    cp      c
    jp      nz,.spew_loop1

    ; start a new line
    ld      c,0x0D
    call    sioa_tx_char
    ld      c,0x0A
    call    sioa_tx_char

    ret

include '../lib/sio.asm'
