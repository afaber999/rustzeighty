

;##############################################################################
; Sets the trace level in the emulator
; Register A determines the trace level, a level of 0 will turn off
; all tracing (default after startup)
;##############################################################################

dbg_set_trace_level:
    ; start code
    out (dbg_trace), a
    ret

dbg_set_trace_off:
    push a
    ld a,0
    out (dbg_trace), a
    pop a
    ret

dbg_set_trace_on:
    push a
    ld a,0xFF
    out (dbg_trace), a
    pop a
    ret 