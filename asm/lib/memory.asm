;****************************************************************************
;
;    Copyright (C) 2021 A.L. Faber
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

; Define the memory size to be used for the CP/M configuration
MEM:    equ 60

; The CPM origin will be at: (MEM-7)*1024
; This screwy convention is due to the way that that the CP/M origin is defined.
CPM_BASE:	equ	(MEM-7)*1024

; XXX This is a hack that won't work unless the disk partition < 0x10000
; XXX This has the SD card partition offset hardcoded in it!!!
; LOAD SIZE MUST BE A MULTIPLE OF 512
LOAD_SIZE:  equ 0x4000                  ; size the bootloader will load from the block device
LOAD_BASE:	equ	0x10000 - LOAD_SIZE		; where the boot loader reads the image from the block device

BLK_PARITION_START: equ 0x0800          ; first partition offset on block device (in blocks) 
SD_BLK_SIZE: equ 512                    ; block device block size in bytes