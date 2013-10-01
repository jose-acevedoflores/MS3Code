; ISR_SCI_COMM
;*********************************************************************
; $Id: isr_sci.s,v 1.85.2.2 2013/04/30 22:42:13 jsmcortina Exp $

; * Copyright 2007, 2008, 2009, 2010, 2011 James Murray and Kenneth Culver
; *
; * This file is a part of Megasquirt-3.
; *
; * Origin: Al Grippo
; * Major: Recode in ASM, new features.
; * Trash most of code and move to mainloop: James Murray
; * Add SCI1: James Murray
; * Majority: James Murray
; *
; * You should have received a copy of the code LICENSE along with this source, please
; * ask on the www.msextra.com forum if you did not.
; *

.sect .text
.globl ISR_SCI0, ISR_SCI1

             nolist               ;turn off listing
             include "ms3h.inc"
             list                 ;turn listing back on
;.equ SERIAL_TIMING, 1

; Serial communications. Revised for MS3 1.1
; All serial is packet based.
; Interrupt RX code fills buffer
; Interrupt TX code sends buffer
; All processing is handled by serial() in mainloop code
; Envelope format is
; [2 bytes big endian size of payload] [payload] [big-endian CRC32]


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;; SCI 0 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_SCI0:
    ldab    GPAGE   ; always stack RPAGE
    pshb
    movb    #0x13, GPAGE

    ; if RDRF register not set, => transmit interrupt
    ldab	SCI0SR1
    bitb    #0x0f
    bne     rx_err   ; Overrun, noise, framing error, parity fail -> kill serial
    bitb    #32      ; check RDRF to see if receive data available
    bne     is1
    bitb    #128      ; check TDRE to see if space to xmit
    lbne    XMT_INT

; dunno... exit
    ldab    SCI0DRL     ; read assumed garbage data to clear flags
    bra     is1_rti

;shouldn't get here

rx_err:
    bitb    #1
    bne     rx_err_pf
    bitb    #2
    bne     rx_err_fe
    bitb    #4
    bne     rx_err_nf
    bitb    #8
    bne     rx_err_or
    clr     next_txmode ; shouldn't happen..
    bra     srl_abort

rx_err_pf:
    movb    #1, next_txmode
    bra     srl_abort
rx_err_fe:
    movb    #2, next_txmode
    bra     srl_abort
rx_err_nf:
    movb    #3, next_txmode
    bra     srl_abort
rx_err_or:
    movb    #4, next_txmode
    bra     srl_abort
xsdata:
    movb    #6, next_txmode
    inc     srl_err_cnt
    bra     srl_abort

;-------------------------------
is1:
    ; Receive Interrupt
    ; Clear the RDRF bit by reading SCISR1 register (done above), then read data
    ;  (in SCIDRL reg).
    ; Check if we are receiving new input parameter update
    ldab   SCI0DRL   ; always read the data first

    ldaa   txmode
    bmi    rxerrtxmode   ; should not be receiving when txmode >= 128 - i.e. transmitting
    bra    is1ckmode

rxerrtxmode:
;    movb    #5, next_txmode
; for testing we'll swallow the erroneous byte
    jmp    isL4           ; bail out

srl_abort:
    clr    rxmode
    clr    txmode
    ldab    SCI0DRL     ; read assumed garbage data to clear flags
    bclr   SCI0CR2,#0xAC;   // rcv, xmt disable, interrupt disable
    bset   flagbyte3,#flagbyte3_kill_srl
    movw   lmms+2, srl_timeout
    jmp    isL4           ; bail out

;--------------
is1ckmode:
    movw   #0xffff, rcv_timeout+0x2
    movw   #0xffff, rcv_timeout

; set current port
    ldx    #SCI0BDH
    stx    port_sci

is1ckm1:
.ifdef SERIAL_TIMING
;debug
    ldaa    outpc.status4
    cmpa    #1
    beq     ckd1
    cmpa    #6
    beq     ckd5
    bra     is1ckm1b
ckd1:
    movw    TCNT, outpc.sensors0
    inc     outpc.status4
    bra     is1ckm1b
ckd5:
    ldx     TCNT
    subx    outpc.sensors0
    stx     outpc.sensors6
    movw    TCNT, outpc.sensors0
    inc     outpc.status4
is1ckm1b:
;end debug
.endif
    ldaa   rxmode
    beq    is1case0
    cmpa   #1
    beq    is1case1
    cmpa   #2
    beq    is1case2
;should not be reached
    jmp    srl_abort

is1case0:
;    movw    lmms+2, outpc.istatus5  ; debug for timing burn time
    clrw    txgoal
    gstab   0xf000
    jmp     is1itx

is1case1:
    gstab   0xf001
    gldd    0xf000
    beq     xsdata              ; zero length is invalid
    cmpd    #0x6100             ; start of "a 00 06"
    beq     mv1ok
    cmpd    #0x7200             ; start of "r 00 06"
    beq     mv1ok
    cmpd    #SRLDATASIZE + 8    ; check for buffer overflow
    bhi     xsdata
mv1ok:
    clrw    rxoffset
    jmp     is1itx

is1case2:
    ldx     rxoffset
    addx    #0xf002
    gstab   0,x
    incw    rxoffset

    gldx    0xf000
    addx    #4
    cpx     rxoffset    
    bhi     isL4
    clr     rxmode
    bset    flagbyte14, #FLAGBYTE14_SERIAL_PROCESS
.ifdef SERIAL_TIMING
;debug
    ldab    outpc.status4
    cmpb    #2
    bne     jis1r
    ldx     TCNT
    subx    outpc.sensors0
    stx     outpc.sensors1
    movw    TCNT, outpc.sensors0
    inc     outpc.status4
jis1r:
;end debug
.endif
; no checking for overrun or underrun etc. YET !  FIXME
    jmp     is1_rti

is1itx:   ; common piece of code
    inc     rxmode

isL4:
;	rcv_timeout = lmms + 781;  // 0.1 sec timeout (was 0.2)
    ldd     lmms+0x2
    ldx     lmms
    addd    #781
    bcc     isL4a
    inx
isL4a:
    std     rcv_timeout+0x2
    stx     rcv_timeout
    jmp     is1_rti

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;    // Transmit Interrupt
XMT_INT:

.ifdef SERIAL_TIMING
;debug
    ldab    outpc.status4
    cmpb    #4
    bne     xmi_done
    ldx     TCNT
    subx    outpc.sensors0
    stx     outpc.sensors4
    movw    TCNT, outpc.sensors0
    inc     outpc.status4
xmi_done:
;end debug
.endif
;    // Clear the TDRE bit by reading SCISR1 register (done), then write data
    ldx     txcnt
    inx
    stx     txcnt

    cpx     txgoal
    lbcc    txmcl

    ldab    txmode

    beq     err_txmode0 ; txmode = 0,   shouldn't be here
    bpl     err_txmode12 ; txmode < 128, shouldn't be here

    ; send back from global buffer
    addx    #0xf000
    gldaa   0,x
    staa    SCI0DRL

    bra     is1_rti

err_txmode0:
    bra     is1_rti

err_txmode12:
    movb    #5, next_txmode
    jmp     srl_abort

txmcl:
.ifdef SERIAL_TIMING
;debug
    ldab    outpc.status4
    cmpb    #5
    bne     txmcl2
    ldx     TCNT
    subx    outpc.sensors0
    stx     outpc.sensors5
    movw    TCNT, outpc.sensors0
    inc     outpc.status4
txmcl2:
;end debug
.endif
    clr     txmode
    clrw    txcnt
    bclr    SCI0CR2, #0x88 ; xmit disable & xmit interrupt disable
    bset    SCI0CR2, #0x24   ; re-enable received interrupt and receiver

    brclr   flagbyte14, #FLAGBYTE14_SERIAL_TL, is1_rti
    ; tooth/trigger loggers only
    bclr    flagbyte14, #FLAGBYTE14_SERIAL_TL ; clear it
    bset    flagbyte8, #FLAGBYTE8_LOG_CLR   ; function in misc clears page and restarts

is1_rti:
    pulb
    stab    GPAGE
    rti

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;; SCI 1 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Presently we have one serial input process, so can receive from one serial
; port or the other. On receipt of data, switch the port_sci pointer.

ISR_SCI1:
    ldab    GPAGE   ; always stack RPAGE
    pshb
    movb    #0x13, GPAGE

    ; if RDRF register not set, => transmit interrupt
    ldab	SCI1SR1
    bitb    #0x0f
    bne     rx_err_1   ; Overrun, noise, framing error, parity fail -> kill serial
    bitb    #32      ; check RDRF to see if receive data available
    bne     is1_1
    bitb    #128      ; check TDRE to see if space to xmit
    lbne    XMT_INT_1

; dunno... exit
    ldab    SCI1DRL     ; read assumed garbage data to clear flags
    bra     is1_rti_1

;shouldn't get here

rx_err_1:
    bitb    #1
    bne     rx_err_pf_1
    bitb    #2
    bne     rx_err_fe_1
    bitb    #4
    bne     rx_err_nf_1
    bitb    #8
    bne     rx_err_or_1
    clr     next_txmode ; shouldn't happen..
    bra     srl_abort_1

rx_err_pf_1:
    movb    #1, next_txmode
    bra     srl_abort_1
rx_err_fe_1:
    movb    #2, next_txmode
    bra     srl_abort_1
rx_err_nf_1:
    movb    #3, next_txmode
    bra     srl_abort_1
rx_err_or_1:
    movb    #4, next_txmode
    bra     srl_abort_1

;-------------------------------
is1_1:
    ; Receive Interrupt
    ; Clear the RDRF bit by reading SCISR1 register (done above), then read data
    ;  (in SCIDRL reg).
    ; Check if we are receiving new input parameter update
    ldab   SCI1DRL   ; always read the data first

    ldaa   txmode
    bmi    rxerrtxmode_1   ; should not be receiving when txmode >= 128 - i.e. transmitting
    bra    is1ckmode_1

rxerrtxmode_1:
;    movb    #5, next_txmode
; for testing we'll swallow the erroneous byte
    jmp    isL4_1           ; bail out

srl_abort_1:
    ldab    SCI1DRL     ; read assumed garbage data to clear flags
    clr    rxmode
    clr    txmode
    bclr   SCI1CR2,#0xAC;   // rcv, xmt disable, interrupt disable
    bset   flagbyte3,#flagbyte3_kill_srl
    movw   lmms+2, srl_timeout
    jmp    isL4_1           ; bail out

;--------------
is1ckmode_1:
    movw   #0xffff, rcv_timeout+0x2
    movw   #0xffff, rcv_timeout

; set current port
    ldx    #SCI1BDH
    stx    port_sci

is1ckm1_1:
    ldaa   rxmode
    beq    is1case0_1
    cmpa   #1
    beq    is1case1_1
    cmpa   #2
    beq    is1case2_1
;should not be reached
    jmp    srl_abort_1

is1case0_1:
    clrw    txgoal
    gstab   0xf000
    jmp     is1itx_1

is1case1_1:
    gstab   0xf001
    gldd    0xf000
    beq     xsdata              ; zero length is invalid
    cmpd    #0x6100             ; start of "a 00 06"
    beq     mv1ok_1
    cmpd    #0x7200             ; start of "r 00 06"
    beq     mv1ok_1
    cmpd    #SRLDATASIZE + 8    ; check for buffer overflow
    bhi     xsdata
mv1ok_1:
    clrw    rxoffset
    jmp     is1itx_1

is1case2_1:
    ldx     rxoffset
    addx    #0xf002
    gstab   0,x
    incw    rxoffset

    gldx    0xf000
    addx    #4
    cpx     rxoffset    
    bhi     isL4_1
    clr     rxmode
    bset    flagbyte14, #FLAGBYTE14_SERIAL_PROCESS
; no checking for overrun or underrun etc. YET !  FIXME
    jmp     is1_rti_1

is1itx_1:   ; common piece of code
    inc     rxmode

isL4_1:
;	rcv_timeout = lmms + 781; // 0.1 sec timeout (was 0.2)
    ldd     lmms+0x2
    ldx     lmms
    addd    #781
    bcc     isL4a_1
    inx
isL4a_1:
    std     rcv_timeout+0x2
    stx     rcv_timeout
    jmp     is1_rti_1

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;    // Transmit Interrupt
XMT_INT_1:

;    // Clear the TDRE bit by reading SCISR1 register (done), then write data
    ldx     txcnt
    inx
    stx     txcnt

    cpx     txgoal
    lbcc    txmcl_1

    ldab    txmode

    beq     err_txmode0_1 ; txmode = 0,   shouldn't be here
    bpl     err_txmode12_1 ; txmode < 128, shouldn't be here

    ; send back from global buffer
    addx    #0xf000
    gldaa   0,x
    staa    SCI1DRL

    bra     is1_rti_1

err_txmode0_1:
    bra     is1_rti_1

err_txmode12_1:
    movb    #5, next_txmode
    jmp     srl_abort_1

txmcl_1:
    clr     txmode
    clrw    txcnt
    bclr    SCI1CR2, #0x88 ; xmit disable & xmit interrupt disable
    bset    SCI1CR2, #0x24   ; re-enable received interrupt and receiver

    brclr   flagbyte14, #FLAGBYTE14_SERIAL_TL, is1_rti_1
    ; tooth/trigger loggers only
    bclr    flagbyte14, #FLAGBYTE14_SERIAL_TL ; clear it
    bset    flagbyte8, #FLAGBYTE8_LOG_CLR   ; function in misc clears page and restarts

is1_rti_1:
    pulb
    stab    GPAGE
    rti

