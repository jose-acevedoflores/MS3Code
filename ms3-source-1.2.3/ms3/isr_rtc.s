;*********************************************************************
; ISR_Timer_Clock
;*********************************************************************
; $Id: isr_rtc.s,v 1.156.2.7 2013/05/22 13:48:01 jsmcortina Exp $

; * Copyright 2007, 2008, 2009, 2010, 2011 James Murray and Kenneth Culver
; *
; * This file is a part of Megasquirt-3.
; *
; * Origin: Al Grippo
; * Major: Recode in ASM, new features. James Murray / Kenneth Culver
; * Majority: James Murray / Kenneth Culver
; *
; * You should have received a copy of the code LICENSE along with this source, please
; * ask on the www.msextra.com forum if you did not.
; *

.sect .text3
.globl ISR_Timer_Clock
             nolist               ;turn off listing
             include "ms3h.inc"
             list                 ;turn listing back on

;**************************************************************************
; "0.1ms" section periodic interrupt
; maintains clocks amongst other functions
; **************************************************************************

ISR_Timer_Clock:
;    // .128 ms clock interrupt - clear flag immediately to resume count
   movb    #128, CRGFLG ; clear RTI interrupt flag

;    // also generate 1.024 ms, .10035 sec and 1.0035 sec clocks
    SSEM1 ; use sem 1 for this for very fine lock in XGATE
    incw    lmms+0x02
    bne     lm3b
    incw    lmms
lm3b:
    CSEM1

    inc      mms

; in SD readback, we'll skip all of these timers to release some CPU cycles.
    brclr   flagbyte7, #FLAGBYTE7_SENDFILE, rtc_normal
    incw    sd_timeout
    bra     ck_milli  ; bypass it all and go to millisecond section

rtc_normal:

    ldab    map_deadman
    cmpb    #4
    bhs     deadmap
    brset   outpc.engine,#0x1,norm_map_sample ; force map sample when engine not running.

map_nonwindow:
    ldy     mapport
    ldx     0,y
    stx     map_temp
    brclr	flagbyte0, #FLAGBYTE0_MAPLOG, no_ml1
    jsr     do_maplog1
no_ml1:
    clrw    map_sum
    stx     map_sum+2
    movw    #1,map_cnt
    bset    flagbyte3,#flagbyte3_samplemap   
    bset    flagbyte15,#FLAGBYTE15_MAP_AVG_RDY
    clr     map_deadman ; acknowledge that we have taken a sample
    bra     done_min ; don't forget the chaos (MAF)

deadmap:
   ; something went wrong with MAP sampling and samples not happening
    bset    outpc.status6, #STATUS6_MAPERROR
    bra     map_nonwindow

norm_map_sample:
   brclr    flagbyte15,#FLAGBYTE15_MAP_AVG_TRIG,continue_norm_map
   ; if I'm here, reached a spark event, so just store my sum and count in their vars, clear
   ; the temp ones, and continue
   movw     map_temp_sum,map_sum
   movw     map_temp_sum+2,map_sum+2
   movw     map_temp_cnt,map_cnt
   clrw     map_temp_sum
   clrw     map_temp_sum+2
   clrw     map_temp_cnt
   bset     flagbyte15,#FLAGBYTE15_MAP_AVG_RDY
   bclr     flagbyte15,#FLAGBYTE15_MAP_AVG_TRIG

continue_norm_map:
   ; add up for average per-cycle map
   ldy      mapport
   ldx      0,y
    brclr	flagbyte0, #FLAGBYTE0_MAPLOG, no_ml2
    jsr     do_maplog1
no_ml2:
   addx     map_temp_sum+2
   stx      map_temp_sum+2
   ldx      map_temp_sum
   adex     #0
   stx      map_temp_sum
   incw     map_temp_cnt 

   ldd      map_start_countdown
   beq      cont_map_check

   ; MAP sampling time?

   subd     #1
   std      map_start_countdown
   bne      cont_map_check ; allow window while counting down
   movw     map_window_set,map_window_countdown
   movw     #0xFFFF,map_temp
   bset     flagbyte15, #FLAGBYTE15_MAPWINDOW
cont_map_check:
   ldd      map_window_countdown
   beq      done_map_check 
   subd     #1
   std      map_window_countdown
   ldy      mapport
   ldx      0,y 
    brclr	flagbyte0, #FLAGBYTE0_MAPLOG, no_ml3
    jsr     do_maplog1
no_ml3:
    cmpx    mapadc_thresh
    bhs     no_mt1
    bset    flagbyte1,#flagbyte1_trig2active  ; MAP phase detection less than threshold
no_mt1:
   cmpx     map_temp
   bhi      cont_map_check2
   stx      map_temp
cont_map_check2:

; now check if 2nd MAP sensor in use and if lower than sample
    ldy     port_map2
    cpy     #dummyReg
    beq     cont_map_check3
    ldx     0,y
    brclr	flagbyte0, #FLAGBYTE0_MAPLOG, no_ml4
    jsr     do_maplog2
no_ml4:
    cmpx    map_temp
    bhi     cont_map_check3
    stx     map_temp

cont_map_check3:
   cpd      #0
   bne      done_map_check
   bclr     flagbyte15, #FLAGBYTE15_MAPWINDOW
   ; done sampling window, tell main loop to take our value
   bset     flagbyte3, #flagbyte3_samplemap
   clr      map_deadman ; acknowledge that we have taken a sample
    brclr   ram4.mapsample_opt, #3, done_min ; bypass

; 4 element ring buffer
    clra
    ldab    map_temps_cnt
    aslb
    tfr     d,y
    stx     map_temps, y
    brclr   ram4.mapsample_opt, #2, scmp1

    cmpb    #6
    blo     cmc2a
    clr     map_temps_cnt
    bra     cmc2b
scmp1:
    cmpb    #2
    blo     cmc2a
    clr     map_temps_cnt
    bra     cmc2b

cmc2a:
    inc     map_temps_cnt
cmc2b:

;pick smallest value
    ldy     #map_temps
    ldd     2,y+
    emind   2,y+
    brclr   ram4.mapsample_opt, #2, str_min
    emind   2,y+
    emind   0,y
str_min:
    std     map_temp

done_min:

   ; while we're here, take a sample of the MAF if it's set up
   brclr    flagbyte8,#FLAGBYTE8_USE_MAF,done_map_check 
   ; MAF enabled, so sample it.
   ldx      mafport
   ldy      0,x
   sty      maf_temp

done_map_check:

;knock
    tst     pin_knock_out
    beq     done_knock
    ldd     knock_start_countdown
    beq     cont_knock_check

   ; knock sampling time?

    subd    #1
    std     knock_start_countdown
    bne     done_knock
    movw    knock_window_set,knock_window_countdown
    tst     knock_state     ; don't set pin if presently communicating as it would break comms
    bne     done_knock
    ;Start of window - set pin
    ldx     port_knock_out
    SSEM0
    ldab    0,x
    orab    pin_knock_out
    stab    0,x
    CSEM0
    clrw    knock_temp

cont_knock_check:

    brset   ram4.knk_option, #0x08, knk_nopeak ; internal SPI mode
    brset   ram4.knk_option, #0x04, knk_an ; on/off or analogue (internal uses SPI not ADC so treat as on/off)
    ldx     port_knk
    ldab    0,x
    andb    pin_knk
    cmpb    pin_knk_match
    bne     knk_nopeak

    movw    #1000, knock_temp
    bra     knk_nopeak

knk_an:
    brclr   ram4.knk_option, #0x80, knk_nopeak
    ; peak mode measure over whole window
    ldx     port_knock_in
    ldd     knock_temp
    emaxd   0,x         ; max of temp or port
    std     knock_temp

knk_nopeak:
    ldd     knock_window_countdown
    beq     done_knock 
    subd    #1
    std     knock_window_countdown
    bne     done_knock

    brset   ram4.knk_option, #0x08, knk_clr ; internal SPI mode
    brclr   ram4.knk_option, #0x04, knk_noend ; digi skip
    brset   ram4.knk_option, #0x80, knk_noend
    ; for end mode we grab the sample right here at the end
    ldx     port_knock_in
    movw    0,x, knock_temp

knk_noend:
    ; End of window - store ADC value
    movw    knock_temp, knockin

knk_clr:
    ; and clear pin
    ldx     port_knock_out
    SSEM0
    ldab    pin_knock_out
    comb
    andb    0,x
    stab    0,x
    CSEM0
    bset    flagbyte11, #FLAGBYTE11_KNOCK ; say we've completed the window
    movb    knock_chan_tmp, knock_chan_sample

    brclr   ram4.knk_option, #0x08, done_knock ; not SPI
    tst     knock_state
    bne     done_knock
    movb    #20, knock_state ; fire off a digi read if SPI

done_knock:

    tst     knock_state
    beq     no_ks
    STACK_SOFT
    call    knock_spi
    UNSTACK_SOFT
no_ks:

    tst     spi2_state
    beq     no_spi2
    STACK_SOFT
    call    do_spi2
    UNSTACK_SOFT
no_spi2:

; tps_ring
    ldab    tps_ring_cnt
    incb
    cmpb    #7
    bls     tpsrs
    clrb
tpsrs:
    stab    tps_ring_cnt
    lslb
    sex     b,x
    ldy     ATD0DR3
    sty     tps_ring,x
;end tps_ring

   brset    flagbyte1,#2,do_testmode
   jmp      ir_notest

; --------- test mode in here, normally skipped  --------
do_testmode:
; test mode

   ldaa     testmode_glob
   cmpa     #3
   beq      tst_pulsedio

;FP removed
   ldx      testcnt
   beq      tst_choose
   dex
   stx      testcnt
   bra      tst_inj_done
tst_choose:
   movw     ram5.testint,testcnt

   ldaa     testmode_glob
   cmpa     #1
   beq      tst_coils
   cmpa     #2
   beq      tst_inj

   bra      ir_notest

tst_coils:
   movb     #0xff, maxdwl

   ldab     ram5.testop
   andb     #0x03
   lbeq     tst_inj_done  ; coil testing not actually enabled
;-----------------------
;   cmpb     #0x03
;   beq      tst_coil_all - not supported
   cmpb     #0x02
   bne      tst_coil_single

;sequence through coils up to coil selected
   inc      tst_tmp
   ldaa     tst_tmp
   ldab     ram5.testsel
   andb     #0xf0 
   addb     #0x10 ; 16th output won't work, but doesn't exist yet !! FIXME !!
   lsla
   lsla
   lsla
   lsla
   cba
   blo      tst_coil_exp
   clra
   staa     tst_tmp
   bra      tst_coil_exp

tst_coil_single:
   ldaa     ram5.testsel
   anda     #0xf0
tst_coil_exp:
    lsra
    lsra
    lsra
    lsra
    ; values now 00-0f
    tab
    clra
    std     tmp_coil
    ldd     #tmp_coil
    movw    #coilsel, 2,-sp
    jsr     set_coil
    leas    2,sp    ; restore stack
    ; expands coil to bits in coilsel
SSEM0
    movw    coilsel, dwellsel ; same
CSEM0
tdsdone:
;blocking XGATE, but only occurs in test mode
SSEM0
   ldd      TCNT
   addd     #48 ; slight delay
   std      TC7 ; set up dwell O/C
   bset     TIE,#0x80
   movb     #0x80, TFLG1

   ldaa     ram5.testdwell
   ldab     #100
   mul
   addd     #48
   addd     TCNT
   std      TC1  ; set up spark O/C
   bset     TIE,#0x02
   movb     0x02, TFLG1
CSEM0
   bra      ir_notest

tst_inj:
   bclr     outpc.squirt, #0x03
; injection counter
   ldx      testmode_cnt
   stx      outpc.istatus5 ; counter in testmode
   beq      LM25
   dex
   stx      testmode_cnt

   ldab     ram5.testop
   andb     #0x60
   lbeq     tst_inj_done  ; injector testing not enabled
;-----------------------
   cmpb     #0x60
   beq      tst_inj_all
   cmpb     #0x40
   bne      tst_inj_single

;sequence through injectors up to injector selected
   inc      tst_tmp
   ldaa     tst_tmp
   ldab     ram5.testsel
   andb     #0x0f
   incb
   cba
   blo      tst_inj_seq

    brset   ram4.hardware,#HARDWARE_MS3XFUEL, tis2   ; ok, MS3X pri
    ldaa    #8
    bra     tis3
tis2:
   clra
tis3:
   staa     tst_tmp
 
tst_inj_seq:
    tab
    bra     tst_inj_do
tst_inj_single:
   ldab     ram5.testsel
   andb     #0x0f
tst_inj_do:
; check not firing MS3X inj by accident
    brset   ram4.hardware,#HARDWARE_MS3XFUEL, tid2   ; ok, MS3X pri
    cmpb    #0x08
    blo     tst_inj_done    ; skip it

tid2:
    clra
    lsld
    tfr     d,x

;countdown
    addx    #inj_cnt
    movw    #1, 0,x

; pulsewidth
    tfr     d,x
    addx    #seq_pw
    movw    ram5.testpw, 0,x

    bra     tst_inj_done

tst_inj_all:
    brset   ram4.hardware,#HARDWARE_MS3XFUEL, tia2   ; ok, MS3X pri
;mainboard primaries, fire both
    ldx     #inj_cnt+16 ; inj1,2
    ldy     #seq_pw+16 ; inj1,2
;inj1
    movw    #1, 2,x+
    movw    ram5.testpw, 2,y+
;inj2
    movw    #1, 2,x+
    movw    ram5.testpw, 2,y+
    bra     tst_inj_done

tia2:
    ldab    num_inj
    ldx     #inj_cnt
    ldy     #seq_pw
tia_settime:
    movw    #1, 2,x+
    movw    ram5.testpw, 2,y+
    dbne    b, tia_settime

tst_inj_done:
   nop ; space for next mode
;   bra      LM25 ; skip over normal inj PWM code
    bra     injpwm  ; ignore cranking test

; --------- pulsed i/o test mode --------

tst_pulsedio:
   inc	    mmsDiv_testio
   ldaa	    ram5.pwm_testio  ; bits 2-3... 78 Hz = 1, 39 Hz = 2, 26 = 3, 19.5 = 4, 15.6 = 5, etc...
   anda	    #0x7
   cmpa	    mmsDiv_testio	
   bhi      end_testio
   clr      mmsDiv_testio
   ldaa	    clk_testio
   inca
   cmpa     #100
   blo      cont_pwm_clktest
   clra
cont_pwm_clktest:
   staa     clk_testio

   ldaa     ram5.duty_testio
   beq	    testio_off
   cmpa	    clk_testio
   blo      testio_off

testio_on:
    ldx	    port_testio
SSEM0
   ldaa     0,X
   oraa     pin_testio
   staa     0,X

CSEM0
   bra      end_testio

testio_off:
    ldx      port_testio
    ldab     pin_testio
    comb
SSEM0
    andb     0,X 
    stab     0,X
CSEM0
end_testio:

; --------- normal code continues --------

ir_notest:

;    check for turning on pwm duty cycle
   brset   outpc.engine,#2,LM25 ; skip if engine is cranking

injpwm:
   tst     pwm1_on ; skip if zero
   beq     LM20

   dec     pwm1_on
   ldaa    pwm1_on
   bne     LM20

LM17:
   ldab    InjPWMPd1
   stab    PWMPER0 ; set PWM period (us)
   movb    pwmd1,PWMDTY0

LM20:
   tst     pwm2_on ; skip if zero
   beq     LM25

   dec     pwm2_on
   ldaa    pwm2_on
   bne     LM25

LM22:
   ldab    InjPWMPd2
   stab    PWMPER1 ; set PWM period (us)
   movb    pwmd2,PWMDTY1

LM25:
;    // check for re-enabling IC interrupt
;    if(lmms > t_enable_IC)  {
   ldd     lmms
   cpd     t_enable_IC
   bcs     LM7
   bhi     LM6
   ldd     lmms+0x2
   cpd     t_enable_IC+0x2
   bls     LM7

LM6:
SSEM0
   movb    #1, TFLG1   ; clear Ignition IC interrupt flag
   bset    TIE, #1     ; enable Ignition IC interrupt
CSEM0
;   bclr    PTS, #0x04 ; turn off mask debug output

   movw    #0xffff, t_enable_IC+0x2
   movw    #0xffff, t_enable_IC

LM7:
; XXXX should only do this if actually using the second trigger
;    // check for re-enabling IC2 interrupt
   ldd     lmms
   cpd     t_enable_IC2
   bcs     LM9
   bhi     LM8
   ldd     lmms+0x2
   cpd     t_enable_IC2+0x2
   bls     LM9

LM8:
SSEM0
   movb    TFLG_trig2, TFLG1   ; clear 2nd trig IC interrupt flag
   brset   ram4.hardware, #HARDWARE_MS3XFUEL, LM8_TC2
   bset    TIE, #0x20    ; enable TC5 IC interrupt
   bra     LM8_TC_ok
LM8_TC2:
   bset    TIE, 0x08     ; enable TC2 IC interrupt
LM8_TC_ok:
CSEM0
;   bclr    PTS, #0x08 ; turn off mask debug output

   movw    #0xffff, t_enable_IC2+0x2
   movw    #0xffff, t_enable_IC2

LM9:
L31:
   brclr    flagbyte1,#flagbyte1_tstmode,dwell_timers
   jmp      dwell_queue   ; don't do dwell timers, boost, nitrous, tacho or genpwm if in test mode

;dwell "actual" timers (always on)
;if zero then counter is skipped
;if>0 and <ff then increments
;code can then determine how long a coil has actually charged for
; compared to how long it thought it was being charged for
;In here also see if maxdwl exceeded
dwell_timers:
    tst     maxdwl
    beq     done_overdwell ; do not check if zero

    brclr   dwl, #0xff, LM112b
   ldaa    dwl
   inc     dwl
   cmpa    maxdwl
   blo     LM112b
   clr     dwl
   ldd    #COILABIT
    bsr fire_n_check
LM112b:
    brclr   dwl+1, #0xff, LM112c
   ldaa    dwl+1
   inc     dwl+1
   cmpa    maxdwl
   blo     LM112c
   clr     dwl+1
   ldd    #COILBBIT
    bsr fire_n_check
LM112c:
    brclr   dwl+2, #0xff, LM112d
   ldaa    dwl+2
   inc     dwl+2
   cmpa    maxdwl
   blo     LM112d
   clr     dwl+2
   ldd    #COILCBIT
    bsr fire_n_check
LM112d:
    brclr   dwl+3, #0xff, LM112e
   ldaa    dwl+3
   inc     dwl+3
   cmpa    maxdwl
   blo     LM112e
   clr     dwl+3
   ldd    #COILDBIT
    bsr fire_n_check
LM112e:
    brclr   dwl+4, #0xff, LM112f
   ldaa    dwl+4
   inc     dwl+4
   cmpa    maxdwl
   blo     LM112f
   clr     dwl+4
   ldd    #COILEBIT
    bsr fire_n_check
LM112f:
    brclr   dwl+5, #0xff, LM112g
   ldaa    dwl+5
   inc     dwl+5
   cmpa    maxdwl
   blo     LM112g
   clr     dwl+5
   ldd    #COILFBIT
    bsr fire_n_check
LM112g:
    brclr   dwl+6, #0xff, LM112h
   ldaa    dwl+6
   inc     dwl+6
   cmpa    maxdwl
   blo     LM112h
   clr     dwl+6
   ldd    #COILGBIT
    bsr fire_n_check
LM112h:
    brclr   dwl+7, #0xff, LM112i
   ldaa    dwl+7
   inc     dwl+7
   cmpa    maxdwl
   blo     LM112i
   clr     dwl+7
   ldd    #COILHBIT
    bsr fire_n_check

    brclr    ram4.EngStroke, #2, LM112i
    ldab    spkmode
    cmpb    #14
    bne     LM112i
    bra     rot_overdwell

    ; not rotary or twin-trigger, check for sparks 9-16
LM112i:
    brclr   dwl+8, #0xff, LM112j
   ldaa    dwl+8
   inc     dwl+8
   cmpa    maxdwl
   blo     LM112j
   clr     dwl+8
   ldd    #COILIBIT
    bsr fire_n_check
LM112j:
    brclr   dwl+9, #0xff, LM112k
   ldaa    dwl+9
   inc     dwl+9
   cmpa    maxdwl
   blo     LM112k
   clr     dwl+9
   ldd    #COILJBIT
    bsr fire_n_check
LM112k:
    brclr   dwl+10, #0xff, LM112l
   ldaa    dwl+10
   inc     dwl+10
   cmpa    maxdwl
   blo     LM112l
   clr     dwl+10
   ldd    #COILKBIT
    bsr fire_n_check
LM112l:
    brclr   dwl+11, #0xff, LM112m
   ldaa    dwl+11
   inc     dwl+11
   cmpa    maxdwl
   blo     LM112m
   clr     dwl+11
   ldd    #COILLBIT
    bsr fire_n_check
LM112m:
    brclr   dwl+12, #0xff, LM112n
   ldaa    dwl+12
   inc     dwl+12
   cmpa    maxdwl
   blo     LM112n
   clr     dwl+12
   ldd    #COILMBIT
    bsr fire_n_check
LM112n:
    brclr   dwl+13, #0xff, LM112o
   ldaa    dwl+13
   inc     dwl+13
   cmpa    maxdwl
   blo     LM112o
   clr     dwl+13
   ldd    #COILNBIT
    bsr fire_n_check
LM112o:
    brclr   dwl+14, #0xff, LM112p
   ldaa    dwl+14
   inc     dwl+14
   cmpa    maxdwl
   blo     LM112p
   clr     dwl+14
   ldd    #COILOBIT
    bsr fire_n_check
LM112p:
    brclr   dwl+15, #0xff, rot_overdwell
   ldaa    dwl+15
   inc     dwl+15
   cmpa    maxdwl
   blo     rot_overdwell
   clr     dwl+15
   ldd    #COILPBIT
    bsr fire_n_check
    bra     done_overdwell

;------------

rot_overdwell:
    brclr   rdwl, #0xff, rd2
   ldaa    rdwl
   inc     rdwl
   cmpa    maxdwl
   blo     rd2
   clr     dwl
   ldab    #COILABIT
    bsr rfire_n_check
rd2:
    brclr   rdwl+1, #0xff, rd3
   ldaa    rdwl+1
   inc     rdwl+1
   cmpa    maxdwl
   blo     rd3
   clr     rdwl+1
   ldab    #COILBBIT
    bsr rfire_n_check
rd3:
    brclr   rdwl+2, #0xff, rd4
   ldaa    rdwl+2
   inc     rdwl+2
   cmpa    maxdwl
   blo     rd4
   clr     rdwl+2
   ldab    #COILCBIT
    bsr rfire_n_check
rd4:
    brclr   rdwl+3, #0xff, rd5
   ldaa    rdwl+3
   inc     rdwl+3
   cmpa    maxdwl
   blo     rd5
   clr     rdwl+3
   ldab    #COILDBIT
    bsr rfire_n_check
rd5:
    brclr   rdwl+4, #0xff, rd6
   ldaa    rdwl+4
   inc     rdwl+4
   cmpa    maxdwl
   blo     rd6
   clr     rdwl+4
   ldab    #COILEBIT
    bsr rfire_n_check
rd6:
    brclr   rdwl+5, #0xff, rd7
   ldaa    rdwl+5
   inc     rdwl+5
   cmpa    maxdwl
   blo     rd7
   clr     rdwl+5
   ldab    #COILFBIT
    bsr rfire_n_check
rd7:
    brclr   rdwl+6, #0xff, rd8
   ldaa    rdwl+6
   inc     rdwl+6
   cmpa    maxdwl
   blo     rd8
   clr     rdwl+6
   ldab    #COILGBIT
    bsr rfire_n_check
rd8:
    brclr   rdwl+7, #0xff, done_overdwell
   ldaa    rdwl+7
   inc     rdwl+7
   cmpa    maxdwl
   blo     done_overdwell
   clr     rdwl+7
   ldab    #COILHBIT
    bsr rfire_n_check

;**
    bra     done_overdwell

  ;***************************************
fire_n_check:
    ldx     coilsel
    std     coilsel
    FIRE_COIL
fnc2:
    tstw    coilsel ; has XGATE fired the coil/s yet?
    bne     fnc2
    stx     coilsel
    rts

  ;***************************************
rfire_n_check:
    ldaa     rotaryspksel
    stab     rotaryspksel
    FIRE_COIL_ROTARY
rfnc2:
    tst     rotaryspksel ; has XGATE fired the coil/s yet?
    bne     rfnc2
    staa    rotaryspksel
    rts
  ;***************************************

done_overdwell:

;;;; boost
check_boost:
    tst     boostpin
    beq     j_skip_boost ; if not yet configured then don't try to run it
   brset    ram4.boost_ctl_settings,#0x8,do_boost
    bra     skip_boost
do_boost:
   brset    ram4.boost_ctl_pwm, #0x20, boost_sw ; outputs selected to use software

;channel 1
   ldx	    boostport
   movb     boost_ctl_duty_pwm, 0,x

   brclr    ram5.boost_ctl_settings2,#0x8,j_skip_boost
;channel 2
    tst     pin_boost2
    beq     j_skip_boost ; if not yet configured then don't try to run it
   ldx	    port_boost2
   movb     boost_ctl_duty_pwm+1, 0,x

j_skip_boost:
   bra      skip_boost

boost_sw:
   inc	    mmsDiv
   ldaa	    ram4.boost_ctl_settings  ; bits 0-2... 78 Hz = 1, 39 Hz = 2, 26 = 3, 19.5 = 4, 15.6 = 5, etc...
   anda	    #0x7
   cmpa	    mmsDiv
   bhi      skip_boost
   clr      mmsDiv
   ldaa	    boost_ctl_clock
   inca
   cmpa     #100
   blo      cont_boost_clk
   clra
cont_boost_clk:
   staa     boost_ctl_clock

;channel 1
   ldx	    boostport
   ldaa     boost_ctl_duty
   beq	    boostOff
   cmpa	    boost_ctl_clock
   blo      boostOff

boostOn:
   brset    ram4.boost_ctl_settings,#0x20,bcClrout
   bra      bcSetout

boostOff:
   brset    ram4.boost_ctl_settings,#0x20,bcSetout
   bra	    bcClrout

bcSetout:
SSEM0
   ldaa     0,X
   oraa     boostpin
   staa     0,X
CSEM0
   bra      bc_chan2

bcClrout:
   ldab     boostpin
   comb
SSEM0
   andb     0,X
   stab     0,X
CSEM0

bc_chan2:
;channel 2
   brclr    ram5.boost_ctl_settings2,#0x8,skip_boost
   ldx	    port_boost2
   ldaa     boost_ctl_duty+1
   beq	    boostOff2
   cmpa	    boost_ctl_clock
   blo      boostOff2

boostOn2:
   brset    ram4.boost_ctl_settings,#0x20,bcClrout2 ; polarity setting shared with chan 1
   bra      bcSetout2

boostOff2:
   brset    ram4.boost_ctl_settings,#0x20,bcSetout2 ; polarity setting shared with chan 1
   bra	    bcClrout2

bcSetout2:
SSEM0
   ldaa     0,X
   oraa     pin_boost2
   staa     0,X
CSEM0
   bra      skip_boost

bcClrout2:
   ldab     pin_boost2
   comb
SSEM0
   andb     0,X
   stab     0,X
CSEM0

skip_boost:

; nitrous stage 1 pwm
check_nitrous1:
   brset    ram4.N2Oopt2,#0x1,check_nitrous1b
   jmp      skip_nitrous
check_nitrous1b:
   inc	    mmsDivn2o
   ldaa	    ram4.N2Oopt2  ; bits 2-3... 78 Hz = 1, 39 Hz = 2, 26 = 3, 19.5 = 4, 15.6 = 5, etc...
   lsra
   lsra
   anda	    #0x7
   cmpa	    mmsDivn2o	
   bhi      skip_nitrous
   clr      mmsDivn2o
   ldaa	    nitrous_pwm_clock
   inca
   cmpa     #100
   blo      cont_nitrous_clk
   clra
cont_nitrous_clk:
   staa     nitrous_pwm_clock

;stage 1
   ldaa     outpc.nitrous1_duty
   beq	    nitrous1off
   cmpa	    nitrous_pwm_clock
   blo      nitrous1off

nitrous1on:
   ldx	    port_n2o1n
SSEM0
   ldaa     0,X
   oraa     pin_n2o1n
   staa     0,X
   ldx	    port_n2o1f
   ldaa     0,X
   oraa     pin_n2o1f
   staa     0,X
CSEM0
   bra      nitrousstage2

nitrous1off:
   ldx      port_n2o1n
   ldab     pin_n2o1n
   comb
SSEM0
   andb     0,X 
   stab     0,X
   ldx      port_n2o1f
   ldab     pin_n2o1f
   comb
   andb     0,X 
   stab     0,X
CSEM0
;stage 2
nitrousstage2:
   ldaa     outpc.nitrous2_duty
   beq	    nitrous2off
   cmpa	    nitrous_pwm_clock
   blo      nitrous2off

nitrous2on:
   ldx	    port_n2o2n
SSEM0
   ldaa     0,X
   oraa     pin_n2o2n
   staa     0,X
   ldx	    port_n2o2f
   ldaa     0,X
   oraa     pin_n2o2f
   staa     0,X
CSEM0
   bra      skip_nitrous

nitrous2off:
SSEM0
   ldx      port_n2o2n
   ldab     pin_n2o2n
   comb
   andb     0,X 
   stab     0,X
   ldx      port_n2o2f
   ldab     pin_n2o2f
   comb
   andb     0,X 
   stab     0,X
CSEM0
skip_nitrous:

; water inj pwm
check_wi:
   brclr    ram4.water_freq,#0x10,skip_wi
   ldab     ram4.water_freq
   andb     #0x60
   cmpb     #0x40   ; slow mode
   bne      skip_wi
check_wib:
   inc	    mmsDivwi
   ldaa	    ram4.water_freq  ; bits 0-2... 78 Hz = 1, 39 Hz = 2, 26 = 3, 19.5 = 4, 15.6 = 5, etc...
   anda	    #0x7
   cmpa	    mmsDivwi	
   bhi      skip_wi
   clr      mmsDivwi
   ldaa	    water_pwm_clock
   inca
   cmpa     #100
   blo      cont_wi_clk
   clra
cont_wi_clk:
   staa     water_pwm_clock

   ldaa     outpc.water_duty
   beq	    wi1off
   cmpa	    water_pwm_clock
   blo      wi1off

wi1on:
   ldx	    port_wivalve
SSEM0
   ldaa     0,X
   oraa     pin_wivalve
   staa     0,X
CSEM0
   bra      skip_wi

wi1off:
   ldx      port_wivalve
   ldab     pin_wivalve
   comb
SSEM0
   andb     0,X 
   stab     0,X
CSEM0
skip_wi:

; generic pwm outputs
    clry
generic_pwm:
    ldaa     pin_pwm,y
    beq      end_pwm
    cmpa     #255 ; see if the magic number
    bne      gp1

;store straight to the (remote) port
    ldx	    port_pwm,y
    ldaa    outpc.duty_pwm,y
    staa    0,x
    bra     end_pwm

gp1:
    tfr     y,x
    lslx                 ; Y doubled ; need more efficient way
    decw    gp_clk,x
    bne     end_pwm

; now decide whether this is an on or off event
    brclr   gp_stat,y, #2, gen_pwm_off

gen_pwm_on:
    ldd     gp_max_on,x
    beq     gen_pwm_off  ; 0% duty
    std     gp_clk,x ; next target timer
    bclr    gp_stat,y, #2

    ldx	    port_pwm,x
SSEM0
    ldaa    0,X
    oraa    pin_pwm,y
    staa    0,X
CSEM0
    bra     end_pwm

gen_pwm_on_sanity:
    ldd     gp_max_on,x
    bne     gen_pwm_on
    bra     end_pwm  ; BOTH are zero...oops

gen_pwm_off:
    ldd     gp_max_off,x
    beq     gen_pwm_on_sanity  ; 100% duty
    std     gp_clk,x ; next target timer
    bset    gp_stat,y, #2

    ldx     port_pwm,x
    ldab    pin_pwm,y
    comb
SSEM0
    andb    0,X 
    stab    0,X
CSEM0

end_pwm:
    iny
    cmpy    #6
    bne     generic_pwm

;create 0.128ms period timer like MS1 did. Used for tacho out. Bypassed in test mode.
   ldx     lowres_ctr
   inx
   bne     str_lowres   ; railing to ffff
   dex
str_lowres:
   stx     lowres_ctr

;tacho /squirt out
   cpx     tacho_targ
   bne     no_tachoff
; turn off tacho output
   ldab    pin_tacho
   beq     injled
   ldy     port_tacho
   comb
SSEM0
   andb    0,y
   stab    0,y
CSEM0
injled:
   ; Inj LED still want to toggle pPTMpin3
   ldy    pPTMpin3
SSEM0
   bclr   0,y, #0x8
CSEM0   

no_tachoff:

; removed and replaced by dwell queue

dwell_queue:
; array is of ints
; dwellq[0].sel = dwellq
; dwellq[0].time_us  = dwellq+2
; dwellq[0].time_mms = dwellq+4
; dwellq[1].sel = dwellq+6
; dwellq[1].time_us  = dwellq+8
; dwellq[1].time_mms = dwellq+10

dq1:
    tstw    dwellq
    beq     dq2
    decw    dwellq+4
    bne     dq2
    ldx     dwellq
    clrw    dwellq
    ldd     dwellq+2
    bsr     dq_fire

dq2:
    tstw    dwellq+6
    beq     dq_done
    decw    dwellq+10
    bne     dq_done
    ldx     dwellq+6
    clrw    dwellq+6
    ldd     dwellq+8
    bsr     dq_fire

    bra     dq_done

dq_fire:
    stx     dwellsel
    std     TC7
    SSEM0
    movb    #0x80, TFLG1
    bset    TIE, #0x80
    CSEM0
    rts

dq_done:

spark_queue:
; array is of ints
; spkq[0].sel = spkq
; spkq[0].time_us  = spkq+2
; spkq[0].time_mms = spkq+4
; spkq[1].sel = spkq+6
; spkq[1].time_us  = spkq+8
; spkq[1].time_mms = spkq+10

sq1:
    tstw    spkq
    beq     sq2
    decw    spkq+4
    bne     sq2
    ldx     spkq
    clrw    spkq
    ldd     spkq+2
    bsr     sq_fire

sq2:
    tstw    spkq+6
    beq     sq_done
    decw    spkq+10
    bne     sq_done
    ldx     spkq+6
    clrw    spkq+6
    ldd     spkq+8
    bsr     sq_fire

    bra     sq_done

sq_fire:
    stx     coilsel
    std     TC1
    SSEM0
    movb    #0x02, TFLG1
    bset    TIE, #0x02
    CSEM0
    rts

sq_done:
;;;; start of clocks section

;rtc_clocks:
; are we within 2 rtc ticks of the overflow? if so, let IC know
   ldd TCNT
   cpd #0xFE7F
   blo  chk_timtcnt 
   bset flagbyte1,#flagbyte1_ovfclose

chk_timtcnt:
   ldd TIMTCNT
   cpd #0xFE7F
   blo  really_rtc_clocks
   bset flagbyte6, #FLAGBYTE6_INJ_OVFCLOSE

really_rtc_clocks:

ck_milli:
;    // check mms to generate other clocks
   ldaa	   mms
   cmpa    #7   ; reset every 8 ticks
   bls     CLK_DONE
   clr     mms

; -------------------- 1ms code section --------------------
;milliseconds    (actually 1.024 ms)
   inc     millisec

;    // check for IAC step pulses
   ldab    IdleCtl
   beq     no_idle
   cmpb    #4
   beq	   no_idle ; PWM idle handled in foreground
   cmpb    #6
   beq     no_idle

; Stepper idle
   ldab    IAC_moving
   beq     no_idle

; Time to step?
   tst     motor_time_ms
   beq     iac_move     ; Already zero
   dec     motor_time_ms
   bne     no_idle      ; Not zero yet.

iac_move:
   cmpb    #2
   beq     iacm2

; IAC_moving = 1
   clr     IAC_moving
   bra     no_idle

iacm2:
; IAC_moving = 2
   movb    ram4.IACtstep, motor_time_ms

LM45:
   ldx    #PTJ ;  Always PTJ

   ldd     IACmotor_pos
   cpd     outpc.iacstep
   bgt     step_anticlockwise
   blt     step_clockwise
   bra     LM50

step_clockwise:
   dec     motor_step
   ldab    motor_step
   clra
   andb    #0x7
   stab    motor_step
   decw    outpc.iacstep

   tfr     D,Y

   ldab    IACCoilA,Y
   tstb
   beq     LM47a
   bset    0,X,#0x01
   bra     LM47b
LM47a:
   bclr    0,X,#0x01
LM47b:
   ldab    IACCoilB,Y
   tstb
   beq     LM47c
   bset    0,X,#0x02
   bra     LM50
LM47c:
   bclr    0,X,#0x02
   bra     LM50

step_anticlockwise:
   inc     motor_step
   ldab    motor_step
   clra
   andb    #0x7
   stab    motor_step
   incw    outpc.iacstep

   tfr     D,Y

   ldab    IACCoilA,Y
   tstb
   beq     LM49a
   bset    0,X,#0x01
   bra     LM49b
LM49a:
   bclr    0,X,#0x01
LM49b:
   ldab    IACCoilB,Y
   tstb
   beq     LM49c
   bset    0,X,#0x02
   bra     LM50
LM49c:
   bclr    0,X,#0x02

LM50:
   ldd     outpc.iacstep
   cpd     IACmotor_pos
   bne     L31 ; skip if not equal

   ldab    IACmotor_reset
   bne     LM57 ; skip if non-zero

   movb    #1, IACmotor_reset
   movw    #0xf380, last_iacclt ; -3200
    ; this will cause to go to temperature based position after initial reset
   clr     IAC_moving
   bra     LM58

LM57:
   movb	#1, IAC_moving

LM58:
   ldab    IdleCtl
   cmpb    #2
   beq     turn_off_IAC
   cmpb    #7
   bne	   no_idle
turn_off_IAC:
   bset    PTJ, #0x40 ; disable current to stepper motor(bit=1)

no_idle:

   ldx     ac_idleup_timer
   bmi     fan_idleup
   inx
   stx     ac_idleup_timer

fan_idleup:
   ldx     fan_idleup_timer
   bmi     do_maxafr
   inx
   stx     fan_idleup_timer

do_maxafr:
;maxafr
   tstw    maxafr_timer
   beq     do_maxegt
   incw    maxafr_timer

do_maxegt:
;maxafr
   tstw    egt_timer
   beq     do_boosttime
   incw    egt_timer

do_boosttime:
   ldy     launch_timer
   bmi     do_nitroustime
   iny
   sty     launch_timer

do_nitroustime:
   ldy     nitrous_timer
   bmi     do_tb_timer
   iny
   sty     nitrous_timer

do_tb_timer:
   ldy     tb_timer
   bmi     do_perfect
   iny
   sty     tb_timer
 
do_perfect:
   ldy     perfect_timer
   beq     ms_c
   bmi     ms_c
   iny
   sty     perfect_timer

ms_c:
   brclr   ram4.boost_ctl_settings,#0x8,vvt_ctl_timer
   decw    boost_ctl_timer
   bne     vvt_ctl_timer
   movw    ram4.boost_ctl_ms, boost_ctl_timer
   bset    flagbyte3,#flagbyte3_runboost

vvt_ctl_timer:
    brclr   flagbyte16, #FLAGBYTE16_VVT_TIMED, idle_ctl_timer
    decw    vvt_timer
    bne     idle_ctl_timer
    movb    #0xFF,vvt_run   ; Just tell all of them to run.
    ldab    RPAGE
    movb    #0xfb,RPAGE
    movw    ram_window.pg24.vvt_ctl_ms, vvt_timer
    stab    RPAGE

idle_ctl_timer:
   ldab    IdleCtl
   cmpb    #6
   bhs     continue_idle_ctl_timer  ; 6,7,8
   bra     continue_millisec

continue_idle_ctl_timer:
   decw    pwmidle_timer
   bne     continue_millisec
   movw    ram4.pwmidle_ms, pwmidle_timer
   bset    flagbyte2,#flagbyte2_runidle
   bset    flagbyte15,#FLAGBYTE15_DPCNT
   
continue_millisec:
    dec     adc_ctr   ; faster as downcounter
   bne     milli_cont
    movb    #10,adc_ctr ; every 10ms

; -------------------- 10ms code section --------------------

    brset   ram4.feature7, #0x08, no_tps_aclk
   inc     tpsaclk
no_tps_aclk:

;stream level decay
   tst      outpc.stream_level
   beq      nodec
   dec      outpc.stream_level
nodec:

    tst     shift_cut_timer
    beq     no_shift_timer
    dec     shift_cut_timer
no_shift_timer:

   bset    flagbyte0,#FLAGBYTE0_50MS  ; set flag so mainloop can do lag factors

;run nitrous down timer here at 1/100s
   tst     n2o_act_timer
   beq     n2chk2
   dec     n2o_act_timer

n2chk2:
   tst     n2o2_act_timer
   beq     do_slip_timer
   dec     n2o2_act_timer
 
do_slip_timer:
    ldy     sliptimer
    beq     milli_cont
    iny
    cpy     #0xffff
    bhi     milli_cont
    sty     sliptimer

; -------------------- end of 10ms code section --------------------

milli_cont:
   ldaa    millisec
   cmpa    #99
   bls     CLK_DONE
   clr     millisec

; -------------------- 100ms code section --------------------

   incw   vss1_stall
   incw   vss2_stall
   incw   ss1_stall
   incw   ss2_stall

   tst     conf_err                 ; check for code config error
   beq     L47  ; skip if no error
    ldab    conf_err
    cmpb    #200
    bhs     L47
   STACK_SOFT
   call     ign_reset
   UNSTACK_SOFT
   movw    outpc.seconds, outpc.clt ; flag up something weird
   ldd     #60                      ;
   subd    outpc.seconds            ; to catch user's attention
   std     outpc.mat
   clra
   ldab    conf_err
   addd    #65000
   std     outpc.rpm ; rpm = 65000 + conf_err
L47:

    bset    flagbyte16, #FLAGBYTE16_CHKSENS

    ; ASE
    brclr   outpc.engine, #1, noasecnt
    brclr   ram4.feature3, #2, noasecnt
    incw    asecount
noasecnt:

   inc     knk_clk
    bne     kn_ok
    movb    #255, knk_clk ; prevent rollover
kn_ok:

   tst     fc_counter
   beq     no_fc
   dec     fc_counter
no_fc:

;anti-lag timer
    tst     als_timer
    beq     altd
    dec     als_timer
altd:

;tclu timer
    tst     tclu_timer
    beq     tclutd
    dec     tclu_timer
tclutd:

; -------------------- runs every 0.128ms --------------------
CLK_DONE:

; this runs on every hit
   ldx    sec_timer  ;    // Get display seconds from 0.128ms clock
   inx
   stx    sec_timer
   cpx    #7812 ; really 7812.5
   blo    DONE1s
   beq    cd_7812
   cpx    #15625 ; 2*7812.5. The second second..otherwise every second hour we lose one second
   blo    DONE1s
   clrw   sec_timer
cd_7812:

; -------------------- one second code section --------------------

   incw   outpc.seconds ;        // update seconds to send back to PC
   bset    flagbyte9, #FLAGBYTE9_GETRTC ; grab new RTC once per second

    bclr    outpc.status6, #STATUS6_MAPERROR ; reset every 1 second

   ldab outpc.seconds+1
    bitb    #0x3f
    bne     fuelcons
    bset    flagbyte15, #FLAGBYTE15_LTT64s ; set every 64 seconds

    ;check SPI and reset timers
    ldd    0xfefe
    cmpd    #0x37f
    bhi     fuelcons
    brclr   PTS, #0xe0, fuelcons
    clr    TIE

fuelcons:
   ; fuel cons
    ldaa    outpc.seconds+1
    anda    #1
    beq     do_bl   ; every 2 seconds, take rolling average

    movw    flowsum_accum, flowsum
    movw    flowsum_accum+2, flowsum+2
    movw    flowsum_accum+4, flowsum+4
    movw    flowsum_accum+6, flowsum+6
    clrw    flowsum_accum
    clrw    flowsum_accum+2
    clrw    flowsum_accum+4
    clrw    flowsum_accum+6

    bset    flagbyte14, #FLAGBYTE14_FUELFLOW_CALC

do_bl:
   ldaa   bl_timer
   beq    pwmidle_wait_timer
   cmpa   #255
   beq    pwmidle_wait_timer
   inca
   staa   bl_timer

pwmidle_wait_timer:
   ldaa   idle_wait_timer
   cmpa   #255
   beq    idle_advance_tmer 
   inca
   staa   idle_wait_timer

idle_advance_tmer:
   brclr  ram4.idle_special_ops,#IDLE_SPECIAL_OPS_IDLEADVANCE,idle_ve_tmer
   ldaa   idle_advance_timer
   cmpa   #255
   beq    idle_ve_tmer 
   inca
   staa   idle_advance_timer

idle_ve_tmer:
   brclr  ram4.idle_special_ops,#IDLE_SPECIAL_OPS_IDLEVE,idle_shift_timer
   ldaa   idle_ve_timer
   cmpa   #255
   beq    idle_shift_timer
   inca
   staa   idle_ve_timer

idle_shift_timer:
   ldaa   pwmidle_shift_timer
   cmpa   #255
   beq    running_timer
   inca
   staa   pwmidle_shift_timer

running_timer:
   ldd    outpc.rpm  ; if rpm is 0, don't increment this counter
   beq	  fc_ego_timer 
   ldaa   running_seconds
   cmpa   #255
   beq    fc_ego_timer
   inca
   staa   running_seconds

fc_ego_timer:
   ldaa   fc_off_time
   cmpa   ram4.fc_ego_delay
   bhs    ac_last_on_timer 
   inca
   staa   fc_off_time

ac_last_on_timer:
   ldaa   ac_time_since_last_on
   cmpa   #255 
   beq    srl_err_counter
   inca
   staa   ac_time_since_last_on

srl_err_counter:
   tst    srl_err_cnt
   beq    DONE1s
   dec    srl_err_cnt

DONE1s:

   inc  sci_lock_timer
    
   rti
