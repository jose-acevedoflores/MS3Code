;*********************************************************************
; ISR_pit
;*********************************************************************
; $Id: isr_pit.s,v 1.19 2012/11/03 15:57:44 jsmcortina Exp $

; * Copyright 2007, 2008, 2009, 2010, 2011 James Murray and Kenneth Culver
; *
; * This file is a part of Megasquirt-3.
; *
; * Origin: James Murray
; * Majority: James Murray
; *
; * You should have received a copy of the code LICENSE along with this source, please
; * ask on the www.msextra.com forum if you did not.
; *

.sect .text3
.globl ISR_pit0, ISR_pit1
             nolist               ;turn off listing
             include "ms3h.inc"
             list                 ;turn listing back on
;*********************************************************************
;50us periodic interrupt
ISR_pit0:
    ldaa    PITTF
    movb    #1, PITTF

;generate 16 bit x 50us timer
   incw    pit_16bits

;VSS1
    tst     pin_vss1
    beq     no_change1 ; not enabled
    tst     ram4.vss1_an
    bne     no_change1 ; using analogue input instead
    ldx     port_vss1
    ldab    0,x
    andb    pin_vss1
    brset   flagbyte7, #FLAGBYTE7_VSS1ACT, was_high1
;likely want polarity switch option..
was_low1:
    tstb
    bne     now_high1
    bra     no_change1
was_high1:
    tstb
    beq     now_low1
    bra     no_change1

now_low1:
    bclr    flagbyte7, #FLAGBYTE7_VSS1ACT
    bra     no_change1

now_high1:
    bset    flagbyte7, #FLAGBYTE7_VSS1ACT

calc_vss1:
   ldd     pit_16bits
   subd    vss1_last
; noise filter
    movw    vss1_time, 2,-sp
    asrw    0,sp
    asrw    0,sp
    cmpd    2,sp+
    blo     no_change1  ; if new period < 25% of previous, then ignore as false trigger
    
    brset    flagbyte8, #FLAGBYTE8_SAMPLE_VSS1, done_record1 ; don't record anything if waiting mainloop

    incw    vss1_teeth
    std     vss1_time
    addd    vss1_time_sum
    std     vss1_time_sum
    cmpd    #2000   ; 20ms
    blo     done_record1
    bset    flagbyte8, #FLAGBYTE8_SAMPLE_VSS1   ; reached at least Xms of data

done_record1:
   movw    pit_16bits,vss1_last
   clrw    vss1_stall

no_change1:

;VSS2
    tst     pin_vss2
    beq     no_change2 ; not enabled
    tst     ram4.vss2_an
    bne     no_change2 ; using analogue input instead
    ldx     port_vss2
    ldab    0,x
    andb    pin_vss2
    brset   flagbyte7, #FLAGBYTE7_VSS2ACT, was_high2
;likely want polarity switch option..
was_low2:
    tstb
    bne     now_high2
    bra     no_change2
was_high2:
    tstb
    beq     now_low2
    bra     no_change2

now_low2:
    bclr    flagbyte7, #FLAGBYTE7_VSS2ACT
    bra     no_change2

now_high2:
    bset    flagbyte7, #FLAGBYTE7_VSS2ACT

calc_vss2:
   ldd     pit_16bits
   subd    vss2_last
; noise filter
    movw    vss2_time, 2,-sp
    asrw    0,sp
    asrw    0,sp
    cmpd    2,sp+
    blo     no_change2  ; if new period < 25% of previous, then ignore as false trigger
 
    brset    flagbyte8, #FLAGBYTE8_SAMPLE_VSS2, done_record2 ; don't record anything if waiting mainloop

    incw    vss2_teeth
    std     vss2_time
    addd    vss2_time_sum
    std     vss2_time_sum
    cmpd    #2000   ; 20ms
    blo     done_record2
    bset    flagbyte8, #FLAGBYTE8_SAMPLE_VSS2   ; reached at least Xms of data

done_record2:
   movw    pit_16bits,vss2_last
   clrw    vss2_stall

no_change2:

;Shaft speed 1
    tst     pin_ss1
    beq     no_change3 ; not enabled
    ldx     port_ss1
    ldab    0,x
    andb    pin_ss1
    brset   flagbyte7, #FLAGBYTE7_SS1ACT, was_high3
;likely want polarity switch option..
was_low3:
    tstb
    bne     now_high3
    bra     no_change3
was_high3:
    tstb
    beq     now_low3
    bra     no_change3

now_low3:
    bclr    flagbyte7, #FLAGBYTE7_SS1ACT
    bra     no_change3

now_high3:
    bset    flagbyte7, #FLAGBYTE7_SS1ACT

calc_ss3:
   ldd     pit_16bits
   subd    ss1_last
   std     ss1_time

   movw    pit_16bits,ss1_last
   clrw    ss1_stall
   bset    flagbyte8, #FLAGBYTE8_SAMPLE_SS1

no_change3:

;Shaft speed 2
    tst     pin_ss2
    beq     no_change4 ; not enabled
    ldx     port_ss2
    ldab    0,x
    andb    pin_ss2
    brset   flagbyte7, #FLAGBYTE7_SS2ACT, was_high4
;likely want polarity switch option..
was_low4:
    tstb
    bne     now_high4
    bra     no_change4
was_high4:
    tstb
    beq     now_low4
    bra     no_change4

now_low4:
    bclr    flagbyte7, #FLAGBYTE7_SS2ACT
    bra     no_change4

now_high4:
    bset    flagbyte7, #FLAGBYTE7_SS2ACT

calc_ss4:
   ldd     pit_16bits
   subd    ss2_last
   std     ss2_time

   movw    pit_16bits,ss2_last
   clrw    ss2_stall
   bset    flagbyte8, #FLAGBYTE8_SAMPLE_SS2

no_change4:
   brset   flagbyte7, #FLAGBYTE7_SS_INIT, end_vss_ss
   bset    flagbyte7, #FLAGBYTE7_SS_INIT
   ; first time around say that all inputs are stalled to avoid spurious initial spike
   movw    #30000, vss1_stall
   movw    #30000, vss2_stall
   movw    #30000, ss1_stall
   movw    #30000, ss2_stall

end_vss_ss:

;water injection 'fast' valve runs in 50us units
   brclr    ram4.water_freq,#0x10,end_water

   ldy      water_pw_cnt
   beq      end_water

   ldab     ram4.water_freq
   andb     #0x60
   cmpb     #0x20   ; high speed
   bne      end_water

   ldx      port_wivalve
   
   dey
   bne      water_set

    SSEM0
   ldaa     pin_wivalve
   coma
   anda     0,x     ; set the pin
   staa     0,x
    CSEM0
   bra      water_store

water_set:
    SSEM0
   ldaa     pin_wivalve
   oraa     0,x     ; set the pin
   staa     0,x
    CSEM0
water_store:
   sty      water_pw_cnt

end_water:

    ldx     vssout_match
    beq     end_vssout
    ldab    pin_vssout
    beq     end_vssout

    ldx     vssout_cnt
    inx
    cpx     vssout_match
    blo     vssout_stx
    ldy     port_vssout
SSEM0
    ldab    0,y
    eorb    pin_vssout  ; flip the bit
    stab    0,y
CSEM0
    clrx
vssout_stx:
    stx     vssout_cnt
end_vssout:


;    // read fuel sensor data, measure period. Calc to freq in _inj.c
    brclr   ram4.FlexFuel, #1, end_ff ; skip if not set

    ldab    PORTE         ; all happen to be on PORTE in MS3
    andb    pin_flex

;                // wait for pin hi -> lo transition to start period(freq) count
    tst     last_fsensdat ;
    beq     ff1          ;
; was high, see if now low
    tstb                  ; B contains port value from above
    bne     ff2          ;

    movw    FPdcounter, FSens_Pd      ; save period
    clrw    FPdcounter                ; start new period count
    bra     ff3

ff1:
    ; was low
    tstb                  ; B contains port value from above
    beq     ff2          ; still low, next!
; now high, record this time
    incw    FPdcounter
    movw    FPdcounter, ff_pw
    
ff2:
   incw    FPdcounter
   bne     ff3
   movw    #20000, FSens_Pd ; wrapped to zero - wire fell off or sensor failed somehow. Report 1Hz

ff3:
   stab    last_fsensdat    ;  last_fsensdat = fsensdat

end_ff:

;I2C state machine
;;debug
;    ldx     pit_16bits
;    andx    #0x000f
;    beq     do_i2c
;    rti
;
;do_i2c:
;debug
    tst     i2cstate
    beq     end_i2c
    ldaa    i2csubstate
    bita    #127
    bne     i2c_sub

    ldab    i2cstate
    cmpb    #1
    beq     i2cs1
    cmpb    #2
    beq     i2cs2
    cmpb    #3
    beq     i2cs3
    cmpb    #4
    beq     i2cs4
    cmpb    #5
    beq     i2cs5
    cmpb    #6
    beq     i2cs6
    cmpb    #7
    beq     i2cs7
    cmpb    #8
    beq     i2cs8
    cmpb    #9
    beq     i2cs9
    cmpb    #10
    beq     i2cs10
    cmpb    #11
    beq     i2cs11
    cmpb    #12
    beq     i2cs12
    cmpb    #13
    beq     i2cs13
    cmpb    #14
    beq     i2cs14
    cmpb    #15
    beq     i2cs15
    cmpb    #16
    beq     i2cs16
    cmpb    #17
    beq     i2cs17
    cmpb    #18
    beq     i2cs18
    cmpb    #19
    beq     i2cs19
    cmpb    #20
    beq     i2cs20
    cmpb    #21
    beq     i2cs21
    cmpb    #22
    beq     i2cs22
    cmpb    #23
    beq     i2cs23
    cmpb    #24
    beq     i2cs24
    cmpb    #25
    beq     i2cs25
    cmpb    #26
    beq     i2cs26

    cmpb    #30
    beq     i2cs30
    cmpb    #31
    beq     i2cs31
    cmpb    #32
    beq     i2cs32
    cmpb    #33
    beq     i2cs33
    cmpb    #34
    beq     i2cs34
    cmpb    #35
    beq     i2cs35
    cmpb    #36
    beq     i2cs36
    cmpb    #37
    beq     i2cs37
    cmpb    #38
    beq     i2cs38
    cmpb    #39
    beq     i2cs39
    cmpb    #40
    beq     i2cs40
    cmpb    #41
    beq     i2cs41
    cmpb    #42
    beq     i2cs42
    cmpb    #43
    beq     i2cs43
    cmpb    #44
    beq     i2cs44
    cmpb    #45
    beq     i2cs45
    cmpb    #46
    beq     i2cs46
    cmpb    #47
    beq     i2cs47
    cmpb    #48
    beq     i2cs48
    cmpb    #49
    beq     i2cs49
    cmpb    #50
    beq     i2cs50
    cmpb    #51
    beq     i2cs51

;invalid
    clr     i2cstate
    bra     end_i2c

i2c_sub:
    bita    #128
    bne     i2c_subrx

;------------------------------
;transmit subroutine
    ldab    i2csubstate
    andb    #3
    cmpb    #0
    beq     scllow1
    cmpb    #3
    beq     scllow2    
    cmpb    #2
    beq     sclhigh1    
    cmpb    #1
    beq     sclhigh1

sclhigh1:
    bset    PORTK, #8 ; SCL high
    bra     sclset

scllow1:
    bclr    PORTK, #8 ; SCL low
    bra     sclset

scllow2:
    asl     i2csubbyte
    bcs     sdahigh
    bclr    PORTK, #2 ; SDA low
    bra     sclset
sdahigh:
    bset    PORTK, #2 ; SDA high

sclset:
    dec     i2csubstate
    bra     end_i2c
;------------------------------
;receive subroutine
i2c_subrx:
    ldab    i2csubstate
    andb    #3
    cmpb    #0
    beq     scllow1rx
    cmpb    #3
    beq     scllow2rx  
    cmpb    #2
    beq     sclhigh1rx   
    cmpb    #1
    beq     sclhigh1rx

sclhigh1rx:
    bset    PORTK, #8 ; SCL high
    bra     sclsetrx

scllow1rx:
    bclr    PORTK, #8 ; SCL low
    bra     sclset

scllow2rx:
    brset   PORTK, #2, i2crx1
    clc
    bra     i2crx_store
i2crx1:
    sec
i2crx_store:
    rol     i2csubbyte
    bclr    PORTK, #8 ; SCL low

sclsetrx:
    dec     i2csubstate
    bra     end_i2c

;-------------------------------

i2cs1:
    brclr   PORTK, #8, i2cs1ok
    ; SCL is high
    brset   PORTK, #2, i2cs1ok ; both already high, so carry on that way

    bclr    PORTK, #8 ; SCL low
    bra     end_i2c
i2cs1ok:
    bset    PORTK, #2 ; SDA high
    bset    DDRK, #2  ; set SDA to output
    bra     i2c_incexit

i2cs2:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs3:
    bclr    PORTK, #2 ; SDA low
    movb    #0xde, i2csubbyte ; SRAM/RTCC READ
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs4:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs5:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs6:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs7:
    ; wait for ACK here...
    brclr   PORTK, #2, i2c_add
    ; no ACK yet... implement timeout
;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c

i2c_add:
    bclr    PORTK, #2 ; SDA low
    bra     i2c_incexit

i2cs8:
    bset    DDRK, #2    ; set SDA to output
    movb    i2caddr, i2csubbyte  ; send address byte 0 = seconds
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs9:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs10:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs11:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs12:
    ; wait for ACK here...
    brclr   PORTK, #2, go_incexit
    ; no ACK yet... implement timeout
;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c
go_incexit:
    jmp     i2c_incexit

i2cs13:
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs14:
    bset    DDRK, #2  ; set SDA to output
    bset    PORTK, #2 ; SDA high
    bra     i2c_incexit

i2cs15:
;start bit
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs16:
    bclr    PORTK, #2 ; SDA low
    movb    #0xdf, i2csubbyte
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs17:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs18:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs19:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs20:
    ; wait for ACK here...
    brclr   PORTK, #2, go_incexit
    ; no ACK yet... implement timeout
    ;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c

i2cs21:
    bclr    PORTK, #8 ; SCL low
    movb    #128+32, i2csubstate
    clr     i2csubbyte
    bra     i2c_incexit

i2cs22:
    movb    i2csubbyte, i2cbyte
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs23:
    bset    DDRK, #2 ; SDA output
    bclr    PORTK, #2 ; SDA low
    bra     i2c_incexit

i2cs24:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs25:
    bset    PORTK, #2 ; SDA high
    bra     i2c_incexit

i2cs26:
    bclr    PORTK, #8 ; SCL low
    clr     i2cstate
    bra     end_i2c

;-------------

i2cs30:
    brclr   PORTK, #8, i2cs30ok
    bclr    PORTK, #8 ; SCL low
    bra     end_i2c
i2cs30ok:
    bset    PORTK, #2 ; SDA high
    bset    DDRK, #2  ; set SDA to output
    bra     i2c_incexit

i2cs31:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs32:
    bclr    PORTK, #2 ; SDA low
    movb    #0xde, i2csubbyte
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs33:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs34:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs35:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs36:
    ; wait for ACK here...
    brclr   PORTK, #2, i2cs36add
    ; no ACK yet... implement timeout
;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c
i2cs36add:
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs37:
    bset    DDRK,#2 ; set SDA as output
    movb    i2caddr, i2csubbyte  ; send address byte
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs38:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs39:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs40:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs41:
    ; wait for ACK here...
    brclr   PORTK, #2, i2cs41add
    ; no ACK yet... implement timeout
;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c
i2cs41add:
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs42:
    bset    DDRK,#2 ; set SDA as output
    movb    i2cbyte, i2csubbyte  ; set WRTC bit, others zero
    movb    #32, i2csubstate
    bra     i2c_incexit

i2cs43:
    ;only gets back here after sending all 8 bits
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs44:
    bclr    DDRK, #2    ; set SDA to input
    bra     i2c_incexit

i2cs45:
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs46:
    ; wait for ACK here...
    brclr   PORTK, #2, i2c_incexit
    ; no ACK yet... implement timeout
;    movb    #1, i2cstate ; try again perhaps
    bra     end_i2c

i2cs47:
    bclr    PORTK, #8 ; SCL low
    bra     i2c_incexit

i2cs48:
    bset    DDRK,#2 ; set SDA as output
    bclr    PORTK, #2 ; SDA low
    bra     i2c_incexit

i2cs49:
    ; stop bit
    bset    PORTK, #8 ; SCL high
    bra     i2c_incexit

i2cs50:
    bset    PORTK, #2 ; SDA high
    bra     i2c_incexit

i2cs51:
    bclr    PORTK, #8 ; SCL low
    clr     i2cstate
    bra     end_i2c

i2c_incexit:
    inc     i2cstate

end_i2c:
    rti

;*********************************************************************
; 125us periodic interrupt
ISR_pit1:
    ldaa    PITTF
    movb    #2, PITTF

   brclr   flagbyte6, #FLAGBYTE6_DONEINIT, no_new_adc  ; skip during init
;Check ADC for new sequence. Expected to occur every time
   brclr   ATD0STAT0, #0x80, no_new_adc ; check if sequence complete
   movb    #0x10, ATD0CTL5              ; start new sequence and clear flags
no_new_adc:
;stream logging
   brclr    flagbyte6, #FLAGBYTE6_STREAM_CONT, stream_norm
   jmp      stream_cont
stream_norm:

   brset    flagbyte6, #FLAGBYTE6_STREAM_HOLD, j_ns     ; skip if on hold
   brset    outpc.sd_status, #0x08, stream_on       ; are we logging
j_ns:
   jmp      no_stream
stream_on:
   ldab     ram4.log_style
   andb     #0x03
   cmpb     #0x01
   bne      no_stream

   ldab     RPAGE
   movb     #SDLOGPAGE, RPAGE
   tfr      d,y
   ldx      port_stream
   ldd      0,x
   ldx      sd_stream_ad
   lsrd ; make eight bit value
   lsrd
   stab     0,x

   tfr      y,d
   stab     RPAGE

   incx
   stx      sd_stream_ad
   tfr      x,y
   andy     #0x01ff
; do three datablock per sector. Control them from here
   cpy      #0x0081
   beq      grab_dat
   cpy      #0x0140
   bne      no_grab
grab_dat:
;stream logging
   bset     flagbyte6, #FLAGBYTE6_SD_GO ; tell mainloop code to grab some data
no_grab:
   cpx      #0x1200
   beq      set_1300
   cpx      #0x1400
   bne      no_stream
   movw     #0x1080, sd_stream_ad
   bra      schd_write
set_1300:
   movw     #0x1280, sd_stream_ad

schd_write: ; better only get here when logging!
   tst      sd_int_phase
   beq      schd_write_ok
; if this happens it means the last write is still happening - can't do two at once
   bset     flagbyte6, #FLAGBYTE6_STREAM_HOLD  ; stop any more audio sampling until card unblocked
schd_write_ok:
   movb     #0x43, sd_phase  ; make mainloop code write out the sector
   bra      no_stream

stream_cont:
; send stream straight to the serial port...
   ldy      port_sci
   brclr    4,y, #0x80, no_stream ; part of flag clearing process. Ought to always have gone (SCIxSR1)
   ldx      port_stream
   ldd      0,x
   lsrd ; make eight bit value
   lsrd
   stab     7,y ;SCIxDRL

no_stream:
; is SD stream logging enabled at all?
   ldab     ram4.log_style
   bitb     #0xc0
   beq      str_level_low
   andb     #3
   cmpb     #1
   bne      str_level_low

;Now we maintain the rolling avg even if not YET logging, so we are ready
   ldx      port_stream
   ldd      0,x
   lsrd
   lsrd

;take rolling average for zero value
; first get 255/256 of existing
   ldaa     sd_stream_avg+1   
   suba     sd_stream_avg
   staa     sd_stream_avg+1
   bcc      sd_ss_nod
   dec      sd_stream_avg
sd_ss_nod:
; now add on the 1/256th
   clra
   addd     sd_stream_avg
   std      sd_stream_avg

; now calculate level - first old version decayed later
;now find abs(stream)
   ldx      port_stream
   ldd      0,x
   lsrd
   lsrd
   subb     sd_stream_avg
   bpl      str_ok_pos
   negb
str_ok_pos:
   cmpb     outpc.stream_level
   blo      str_level_low
   stab     outpc.stream_level

str_level_low:

    rti
