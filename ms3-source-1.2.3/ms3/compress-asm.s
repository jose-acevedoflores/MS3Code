;*********************************************************************
; asm for compress
;*********************************************************************
; $Id: compress-asm.s,v 1.12 2012/10/16 19:05:10 jsmcortina Exp $
;
; * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
; *
; * This file is a part of Megasquirt-3.
; *
; * comp_writesd
;    Origin: James Murray
;    Majority: James Murray
; * comp_readhash
;    Origin: James Murray
;    Majority: James Murray
; * comp_writehash
;    Origin: James Murray
;    Majority: James Murray
; * comp_clearhash
;    Origin: James Murray
;    Majority: James Murray
; * comp_readcode
;    Origin: James Murray
;    Majority: James Murray
; * comp_writecode
;    Origin: James Murray
;    Majority: James Murray
; * comp_memnmagic
;    Origin: arr. from public domain C code.
;    Major: coded in ASM. James Murray
;    Majority: Public domain
; * comp_cp_remainder
;    Origin: arr. from public domain C code.
;    Major: coded in ASM. James Murray
;    Majority: Public domain
; * comp_output3
;    Origin: arr. from public domain C code.
;    Major: coded in ASM. James Murray
;    Majority: Public domain
; *
; * You should have received a copy of the code LICENSE along with this source, please
; * ask on the www.msextra.com forum if you did not.
; *
;

.sect .textfb
.globl comp_writesd, comp_readhash, comp_writehash, comp_clearhash, 
.globl comp_readcode, comp_writecode, comp_cp_remainder, comp_memnmagic, comp_output3
.globl SENDBUF

    nolist               ;turn off listing
    include "ms3h.inc"
;*********************************************************************
; Hard coded equates for global addresses of tables
.equ    HTAB,   0x0800  ; in GPAGE 0x0f
.equ    CODETAB,0x5700  ; in GPAGE 0x0f
.equ    SENDBUF,0xf000  ; in GPAGE 0x13

;*********************************************************************

comp_writesd:
;int comp_writesd(unsigned int outbuf, unsigned int OUTBUFSIZ)
; foreground code to write out buffer until empty
;on entry
;outbuf D
;OUTBUFSIZ 3,SP
;on exit
;num bytes written D
    ldy     3,SP
    tfr     d,x

    leas    -4,sp
    sei
    movw    lmms, 0,sp
    movw    lmms+2, 2,sp
    cli

is_sci_ready:
    brset   flagbyte7, #FLAGBYTE7_SF_GO, sci_ready  ; wait until SCI ready for us
    pshx
    pshy
    call    serial  ; or else we never will be ready !
    puly
    pulx
    bra     is_sci_ready

; check for timeout
    ; old code brought back from the dead
    ldd     sd_timeout
    cmpd    #39000
    blo     is_sci_ready
;;over 5 seconds... bail out
    clr     sd_phase
    clrw    7,sp
    bra     compwsd_exit

sci_ready:
    bclr    flagbyte7, #FLAGBYTE7_SF_GO ; clear it again

    movb    #0x13, GPAGE    ; new serial

    clrw    sd_timeout

; this has changed a little to newserialise it.
    ldd     sd_rb_block
    gstd    SENDBUF+3            ; store block num into buffer after code
    incw    sd_rb_block

    clra
    gstaa   SENDBUF+2           ; ok code 0

    addy    #3
    tfr     y,d
    gsty    SENDBUF            ; store num bytes into buffer, first two bytes
    sty     txgoal      ; number of bytes of payload - irrespective of what requested

;copy from outbuf to send buf
; 0,1 = num bytes
; 2 = return code
; 3,4 = block num
    ldy     #SENDBUF+5

wsdlp2:
; going to be slow with the GPAGE switching... think of another way ? map to RPAGE and copy wordwise?
    pshd
    movb    #0x0f, GPAGE
    gldaa   1,x+
    movb    #0x13, GPAGE
    gstaa   1,y+
    puld
    dbne    d,wsdlp2

;            g_write32(g_crc32buf(0, 0xf002, txgoal), 0xf002 + txgoal);
    movw    txgoal, 0x2,-SP
    movw	#0xf002, 0x2,-SP
    clra
    clrb
    clrx
    call    g_crc32buf
    leas    4,sp
    ldy     txgoal
    leay    0xf002,y
    gstx    2,y+
    gstd    0,y
    ldy     txgoal
    leay    6,y     ; add on the 2 prefix and 4 crc suffix bytes
    sty     txgoal

    movb    #129, txmode

; send first byte
    gldaa   SENDBUF
    clrw   txcnt
    ldx    port_sci
    bset   3,x, #0x08   ; xmit enable enable    ; SCIxCR2
    staa   7,x                                  ; SCIxDRL
    bset   3,x, #0x80   ; xmit interrupt enable ; SCIxCR2

; now serial work done can restore GPAGE
    movb    #0x0f, GPAGE

    sei
    ldd     lmms+0x2
    ldx     lmms
    cli
    addd    #3906
    bcc     wsdlp3
    inx
wsdlp3:
    sei
    std     rcv_timeout+0x2
    stx     rcv_timeout
    cli

compwsd_exit:
    leas    4,sp
    ldd     3,SP    ;return code to say all ok
    rtc

comp_readhash:
;unsigned long readhash(unsigned int index)
;on entry
;index  D
;on exit
;long value X:D
    movb    #0x0f, GPAGE
    lsld
    lsld
    addd    #HTAB
    tfr     d,y
    gldx    2,y+
    gldd    0,y
    
    rtc

comp_writehash:
;void writehash(unsigned int index, unsigned long value)
; on entry
; index   D
; value   3,SP

    movb    #0x0f, GPAGE
    lsld
    lsld
    addd    #HTAB
    tfr     d,y
    ldx     3,SP
    gstx    2,y+
    ldx     5,SP
    gstx    0,y

    rtc

comp_clearhash:
; on entry
; HSIZE     D

    ldx     #HTAB
    tfr     d,y

    movb    #0x0f, GPAGE
    ldd     #0xffff
mslp3:
    gstd    2,x+
    gstd    2,x+ ; long
    dey
    bne     mslp3
    rtc

comp_readcode:
;unsigned int readcode(unsigned int index)
;on entry
;index  D
;on exit
;int value D
    movb    #0x0f, GPAGE
    lsld
    addd    #CODETAB
    tfr     d,y
    gldd    0,y
    
    rtc

comp_writecode:
;void writecode(unsigned int index, unsigned int value)
; on entry
; index   D
; value   3,SP

    movb    #0x0f, GPAGE
    lsld
    addd    #CODETAB
    tfr     d,y
    ldx     3,SP
    gstx    0,y
    rtc

comp_memnmagic:
;void memnmagic(unsigned int outbuf, unsigned int SIZEOFOUTBUF)
; on entry
; outbuf        D
; SIZEOFOUTBUF  3,SP
; clears outbuf and sets first three bytes to magics and block mode

    tfr     d,x
    ldy     3,SP

    movb    #0x0f, GPAGE
    lsry

    ldaa    #0x1f      ; // MAGIC1
    gstaa   1,x+
    ldaa    #0x9d      ;  // MAGIC2
    gstaa   1,x+
    ldaa    #0x8c      ;  // BLOCKMODE | 0c bits
    gstaa   1,x+
    clra
    gstaa   1,x+
    dey
    dey
    clra
    clrb
mslp:
    gstd    2,x+
    dey
    bne     mslp

    rtc

comp_cp_remainder:
;comp_cp_remainder(unsigned int outbuf, unsigned int OBUFSIZ, unsigned int(outbits>>3)+1)
;On entry 
; outbuf D
; OBUFSIZ 5,sp
; (outbits>>3+1) 3,sp
;   "x"(outbuf), "y"(outbuf+OBUFSIZ), "d"((unsigned int)(outbits>>3)+1)
    movb    #0x0f, GPAGE
    tfr     d,x
    addd    3,sp
    tfr     d,y
    ldd     5,sp
    pshx    ; save outbuf value

mslp2:
    psha
    gldaa    1,y+
    gstaa    1,x+
    pula
    subd    #1
    bne     mslp2
    
; Next section required to fill rest of outbuf with zeros.. possibly not required if output3 uses = instead of |= on final byte

    pulx    ; outbuf from above
    addx    5,sp
    ldy     3,sp
    lsry
    clra
    clrb
mslp4:
    gstd    2,x+
    dey
    bne mslp4

    rtc

comp_output3:
;void output(unsigned int addr, unsigned long value)
;input addr = (outbuf+offset) D
;i        3,sp

;#define	output(b,o,c,n)	{	uint8_t	*p = &(b)[(o)>>3];				\
;							int32_t		 i = ((int32_t)(c))<<((o)&0x7);	\
;							p[0] |= (uint8_t)(i);							\
;							p[1] |= (uint8_t)(i>>8);						\
;							p[2] |= (uint8_t)(i>>16);						\
;							(o) += (n);										\
;						}

;(o)>>3
    movb    #0x0f, GPAGE
    tfr     d,x

    gldaa   0,x
    oraa    6,SP
    gstaa   1,x+

    gldaa   0,x
    oraa    5,SP
    gstaa   1,x+

    gldaa   0,x
    oraa    4,SP
    gstaa   1,x+

    rtc
