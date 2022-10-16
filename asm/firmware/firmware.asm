;****************************************************************************
;
;    Copyright (C) 2022 A.L. Faber
;
;    This library is free software; you can redistribute it and/or
;    modify it under the terms of the GNU Lesser General Public
;    License as published by the Free Software Foundation; either
;    version 2.1 of the License, or (at your option) any later version.
;
;    This library is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;    Lesser General Public License for more details.
;
;    You should have received a copy of the GNU Lesser General Public
;    License along with this library; if not, write to the Free Software
;    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
;    USA
;
;****************************************************************************

.debug:		equ	99		; Set to 1 to show debug printing, else 0 

include	'../lib/io.asm'
include	'../lib/memory.asm'

.load_blks:	equ	(0x10000-LOAD_BASE)/512
.stacktop:	equ	LOAD_BASE	; (so the SD loader does not overwrite)

	org		0x0000		; Cold reset Z80 entry point.
	ld	sp,.stacktop    ; setup stack pointer

	; Init the SIO 
	call	sioa_init

    ; startup message
    call iputs
    db 0x0D, 0x0A, "*** starting boot firmware ***", 0x0D, 0x0A, 0

    ; init block device
    ld A, 0
    out (blk_dev_init),a
    
	; Load code from the block device
	call	.boot_blkdev

    call iputs
    db 0x0D, 0x0A, "*** copied data from block device ***", 0x0D, 0x0A, 0

if .debug >= 4
	; dump the 16k area
	ld	hl,LOAD_BASE		; start address
	ld	bc,0x4000	        ; dump 16 K number of bytes
	ld	e,1		            ; fancy format
	call	hexdump
endif

    ; start code
    jp LOAD_BASE


;##############################################################################
; Load 16K from the first blocks of partition 1 on the SD card into
; memory starting at 'LOAD_BASE' and jump to it.
; If reading the block device  should fail then this function will return.
;##############################################################################
.boot_blkdev:
	call	iputs
	db 0x0D,0x0A,"Booting SD card partition 1", 0x0D,0x0A,0x0D,0x0A,0


    ; a number of blocks to copy
    ld a, LOAD_SIZE / SD_BLK_SIZE
    ; DE = block device physical block number
    ld de, BLK_PARITION_START

    ; turn on tracing
    ;ld a, 0xFF
    ;call dbg_set_trace_level

    ; HL = target address to copy bytes
    ld hl, LOAD_BASE

.copy_loop:
    push  af
	call  .blk_read		; read 1 sector
    pop   af
    inc   de
    dec   a
    jp    NZ, .copy_loop
    ret


.blk_read:
    ; DE block
    ; HL target address
    
if .debug >= 1
	call	iputs
	db	".blk_read entered: ",0

	call	iputs
	db	", block=0x",0
	ld	a,d
	call	hexdump_a
	ld	a,e
	call	hexdump_a

	call	iputs
	db	", target address =0x",0
	ld	a,h
	call	hexdump_a
	ld	a,l
	call	hexdump_a

	call	puts_crlf
endif

	; set active block which will be read
    ld a,0
    out (blk_dev_blk_b3),a
    out (blk_dev_blk_b2),a
    ld a,d
    out (blk_dev_blk_b1),a
    ld a,e
    out (blk_dev_blk_b0),a

    ; read block
    ld a,0
    out (blk_dev_rd_blk),a

    ; get all 512 bytes from block
    ld bc, 0x0200

rd_blk_char:
    in a,(blk_dev_rd_char)
    ld  (hl),a
    inc hl
    dec bc
    ld a,b
    or a,c
    jp NZ, rd_blk_char
    ret

include '../lib/sio.asm'
include '../lib/puts.asm'
include '../lib/hexdump.asm'
include '../lib/dbg_trace.asm'

;##############################################################################
; This marks the end of the data copied from FLASH into RAM during boot
;##############################################################################
.end:		equ	$
