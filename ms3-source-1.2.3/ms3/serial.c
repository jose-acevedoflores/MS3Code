/* $Id: serial.c,v 1.56.2.3 2013/01/27 15:31:27 jsmcortina Exp $
 * Copyright 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * Origin: James Murray
 * Majority: James Murray
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
 */

/* mainloop portion of serial processing */

#include "ms3.h"

#define GWR(a) g_write_generic(a, &x, sizeof(a));
//#define SERIAL_TIMING 1
//#define SERIAL_TIMING2 1

#define DB_BUFADDR 0xf900
#define DB_BUFSIZE 0x0700

void g_write_generic(unsigned long val, unsigned int* ad, unsigned char size)
{
    if (size == 1) {
        g_write8((unsigned char)val, *ad);
        *ad += 1;
    } else if (size == 2) {
        g_write16((unsigned int)val, *ad);
        *ad += 2;
    } else if (size == 4) {
        DISABLE_INTERRUPTS;
        g_write32(val, *ad);
        ENABLE_INTERRUPTS;
        *ad += 4;
    }
    return;
}

void serial()
{
    unsigned int size, ad, x, r_offset, r_size;
    unsigned long crc;
    unsigned char cmd, r_canid, r_table, rp, compat = 0;

#ifdef SERIAL_TIMING
    /* debug */
    if ((outpc.status4 == 0) && ((outpc.seconds % 20) == 5)) {
        outpc.status4 = 1;
    }
    if ((outpc.seconds % 20) == 6) {
        outpc.status4 = 0;
    }
    /* end debug */
#endif
    ck_log_clr();

    GPAGE = 0x13;
    ad = 0xf002;

#if 0
    if ((MONVER >= 0x380) && (MONVER <= 0x38f)) {
        /* Only on MS3pro */
        if ((flagbyte2 & FLAGBYTE2_SCI_CONFIRMED) && (sci_lock_timer > 10)) {
            flagbyte2 &= ~FLAGBYTE2_SCI_CONFIRMED;
            /* had locked but nothing for over 10 seconds, unlock */
            (void)SCI0SR1; // clear any flags
            (void)SCI0DRL;
            SCI0CR1 = 0x00;
            SCI0CR2 = 0x24;             // TIE=0,RIE = 1; TE=0,RE =1

            /* now set SCI1 the same */
            (void)SCI1SR1; // clear any flags
            (void)SCI1DRL;
            SCI1CR1 = 0x00;
            SCI1CR2 = 0x24;             // TIE=0,RIE = 1; TE=0,RE =1
        }
    }
#endif

    if (flagbyte3 & flagbyte3_kill_srl) {
        unsigned int lmms_tmp, timeout;
        if (next_txmode == 7) {
            timeout = 7812; // 1 second
        } else {
            timeout = 390; // about 50ms seconds of timeout
        }
        lmms_tmp = (unsigned int)lmms;

        if ((lmms_tmp - srl_timeout) > timeout) {
            unsigned char dummy;
            if ((!(flagbyte2 & FLAGBYTE2_SCI_CONFIRMED)) || ((unsigned int)port_sci == (unsigned int)SCI0BDH)) {
                if (SCI0SR1 & 0x20) { // data available
                    dummy = SCI0DRL; // gobble it up and wait again            
                    srl_timeout = (unsigned int)lmms;
                    return;
                }
                dummy = SCI0SR1; // step 1, clear any pending interrupts
                dummy = SCI0DRL; // step 2
                SCI0CR2 |= 0x24;        // rcv, rcvint re-enable
            }

            if ((MONVER >= 0x380) && (MONVER <= 0x38f)) {
                if (!(flagbyte2 & FLAGBYTE2_SCI_CONFIRMED) || ((unsigned int)port_sci == (unsigned int)SCI1BDH)) {
                    if (SCI1SR1 & 0x20) { // data available
                        dummy = SCI1DRL; // gobble it up and wait again            
                        srl_timeout = (unsigned int)lmms;
                        return;
                    }

                    dummy = SCI1SR1; // step 1, clear any pending interrupts
                    dummy = SCI1DRL; // step 2
                    SCI1CR2 |= 0x24;        // rcv, rcvint re-enable
                }
            }

            rcv_timeout = 0xFFFFFFFF;
            flagbyte3 &= ~flagbyte3_kill_srl;
            if (next_txmode == 1) {
                g_write8(0x8c, 0xf002); // parity error
            } else if (next_txmode == 2) {
                g_write8(0x8d, 0xf002); // framing error
            } else if (next_txmode == 3) {
                g_write8(0x8e, 0xf002); // noise
            } else if (next_txmode == 4) {
                g_write8(0x81, 0xf002); // overrun
            } else if (next_txmode == 5) {
                g_write8(0x8f, 0xf002); // Transmit txmode range
            } else if (next_txmode == 6) {
                g_write8(0x84, 0xf002); // Out of range
            } else if (next_txmode == 7) {
                int x;
                const char errstr[] = "Too many bad requests! Stop doing that!";
                g_write8(0x91, 0xf002); // Too many!

                for (x = 0; x < sizeof(errstr); x++) {
                    g_write8(errstr[x], 0xf003 + x); 
                }
                txgoal = sizeof(errstr);
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            } else {
                g_write8(0x90, 0xf002); // unknown
            }
            txgoal = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
    }
    
    /* check for under-run or old commands */
    if (rxmode) {
        unsigned long lmms_tmp;

        DISABLE_INTERRUPTS;
        lmms_tmp = lmms;
        ENABLE_INTERRUPTS;

        if (lmms_tmp > rcv_timeout) {
            /* had apparent under-run, check for old commands */
            cmd = g_read8(0xf000);
            if ((rxmode == 1) && (cmd == 'Q')) {
                ad = 0xf000;
                size = 1;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'S')) {
                ad = 0xf000;
                size = 1;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'C')) {
                ad = 0xf000;
                size = 1;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'F')) {
                ad = 0xf000;
                size = 1;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'I')) {
                ad = 0xf000;
                size = 1;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'A')) {
                ad = 0xf000;
                size = 1;
                compat = 1; // full A
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 1) && (cmd == 'D')) {
                unsigned int cnt, cnt2, pt1;
                txgoal = 0;
                /* debug mode */
                if (flagbyte15 & FLAGBYTE15_DB_WRAP) {
                    cnt2 = DB_BUFSIZE;
                    pt1 = db_ptr;
                } else {
                    cnt2 = db_ptr;
                    pt1 = 0;
                }
                for (cnt = 0 ; cnt < cnt2 ; cnt++) {
                    unsigned char c;
                    c = g_read8(DB_BUFADDR + pt1);
                    pt1++; /* gcc makes better code with this on its own line */
                    if (pt1 >= DB_BUFSIZE) {
                        pt1 = 0;
                    }
                    /* change 0x0a into 0x0d 0x0a */
                    if (c) {
                        if (c == 0x0a) {
                            g_write8(0x0d, 0xf000 + txgoal);
                            txgoal++;
                        } else if ((c != 0x0a) && (c != 0x0d) && ((c < 32) || (c > 127))) {
                            c = '.';
                        }
                        g_write8(c, 0xf000 + txgoal);
                        txgoal++;
                    }
                }
                rxmode = 0;
                /* send as unwrapped data */
                txcnt = 0;
                txmode = 129;
                *(port_sci + 7) = g_read8(0xf000); // SCIxDRL
                *(port_sci + 3) |= 0x88; // xmit enable & xmit interrupt enable // SCIxCR2
                flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;

            } else if ((rxmode == 2) && (cmd == 'r') && (g_read8(0xf001) == 0x00) && (g_read8(0xf002) == 0x04)) {
                /* Megaview request config data command */
                ad = 0xf000;
                size = 7;
                compat = 1;
                rxmode = 0;
                goto SERIAL_OK;
            } else if ((rxmode == 2) && (cmd == 'a') && (g_read8(0xf001) == 0x00) && (g_read8(0xf002) == 0x06)) {
                /* Megaview read command */
                ad = 0xf000;
                g_write8('A', ad); /* fake A command */
                size = 1;
                compat = 2; // MV read
                rxmode = 0;
                goto SERIAL_OK;
            } else {
                /* had some undefined under-run situation */
                g_write8(0x80, 0xf002);
                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                rxmode = 0;
                goto EXIT_SERIAL;
            }
        }
    }

    if ((flagbyte14 & FLAGBYTE14_SERIAL_BURN) && (burnstat == 0)) {
        /* burn now completed */
        flagbyte14 &= ~FLAGBYTE14_SERIAL_BURN;
        g_write8(0x04, 0xf002); // burn OK code
        txgoal = 0;
        flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
//        outpc.sensors[15] = (unsigned int)lmms - outpc.istatus5; // time the burn operation
        goto EXIT_SERIAL;
    }

    if (flagbyte14 & FLAGBYTE14_SERIAL_OK) {
        flagbyte14 &= ~FLAGBYTE14_SERIAL_OK;
        goto RETURNOK_SERIAL;
    }

    if (flagbyte11 & FLAGBYTE11_CANRX) {
        flagbyte11 &= ~FLAGBYTE11_CANRX;
        g_write8(0x06, 0xf002); // OK code
        txgoal = g_read16(0xf000); // what was stored there previously
        flagbyte3 &= ~flagbyte3_getcandat;
        cp_targ = 0;
        flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
        goto EXIT_SERIAL;
    }

    if (flagbyte3 & flagbyte3_getcandat) {
        unsigned int ti;
        DISABLE_INTERRUPTS;
        ti = (unsigned int)lmms - cp_time;
        ENABLE_INTERRUPTS;

        if (ti > 1000) {
            /* taking too long to get a reply... bail on it */
            g_write8(0x8a, 0xf002); // CAN timeout
            txgoal = 0;
            flagbyte3 &= ~flagbyte3_getcandat;
            cp_targ = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else if (flagbyte14 & FLAGBYTE14_CP_ERR) {
            flagbyte3 &= ~flagbyte3_getcandat;
            flagbyte14 &= ~FLAGBYTE14_CP_ERR;
            g_write8(0x8b, 0xf002); // CAN failure
            txgoal = 0;
            cp_targ = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
        if (flagbyte14 & FLAGBYTE14_SERIAL_PROCESS) {
            g_write8(0x85, 0xf002); // BUSY - in the middle of a CAN transaction

            txgoal = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
    }

    if (flagbyte3 & flagbyte3_sndcandat) {
        unsigned int ti;
        DISABLE_INTERRUPTS;
        ti = (unsigned int)lmms - cp_time;
        ENABLE_INTERRUPTS;

        if (ti > 1000) {
            g_write8(0x8a, 0xf002);
            txgoal = 0;
            flagbyte3 &= ~flagbyte3_sndcandat;
            cp_targ = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else if (flagbyte14 & FLAGBYTE14_CP_ERR) {
            flagbyte3 &= ~flagbyte3_sndcandat;
            flagbyte14 &= ~FLAGBYTE14_CP_ERR;
            g_write8(0x8b, 0xf002); // CAN failure - not sure that TS will be expecting this reply now...
            txgoal = 0;
            cp_targ = 0;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
    }

    if (!(flagbyte14 & FLAGBYTE14_SERIAL_PROCESS)) {
        if (flagbyte14 & FLAGBYTE14_SERIAL_FWD) {
            /* pass on forwarded CAN data if not doing any other transaction */
            flagbyte14 &= ~FLAGBYTE14_SERIAL_FWD;
            txgoal = canbuf[16];
            g_write8(6, 0xf002); /* CAN data */
            for (x = 0; x < txgoal; x++) {
                g_write8(canbuf[x], 0xf003 + x);
            }
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;

        } else {
            return;
        }
#ifdef SERIAL_TIMING
        /* debug */
    } else {
        if (outpc.status4 == 3) {
            outpc.sensors[2] = TCNT - outpc.sensors[0];
            outpc.sensors[0] = TCNT;
        }
    /* end debug */
#endif
    }

    size = g_read16(0xf000); // size of payload
    /* calc CRC of data */
    crc = g_crc32buf(0, 0xf002, size);    // incrc, buf, size

    if (crc != g_read32(0xf002 + size)) {
        /* CRC did not match - build return error packet */
        g_write8(0x82, 0xf002); // CRC failed code
        txgoal = 0;
        flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
        goto EXIT_SERIAL;
    }

    /* Get here when packet looks ok */
SERIAL_OK:;

    /* We got something real, so lock to this serial port only. */
    sci_lock_timer = 0;
    if (!(flagbyte2 & FLAGBYTE2_SCI_CONFIRMED)) {
        flagbyte2 |= FLAGBYTE2_SCI_CONFIRMED;
        if ((unsigned int)port_sci == (unsigned int)SCI0BDH) {
            /* Disable SCI1 */
            SCI1CR1 = 0x00;
            SCI1CR2 = 0x00;
        } else {
            /* Disable SCI0 */
            SCI0CR1 = 0x00;
            SCI0CR2 = 0x00;
        }
    }
/*
;**************************************************************************
; **
; ** SCI Communications - within size/CRC32 wrapper
; **
; ** Communications is established when the PC communications program sends
; ** a command character - the particular character sets the mode:
; **
; was "a"+<canid>+<table id> = send all of the realtime display variables (outpc structure) via txport.
; now "A" send all realtime vars
; ** "w"+<canid>+<table id>+<offset lsb>+<offset msb>+<nobytes>+<newbytes> =
; **    receive updated data parameter(s) and write into offset location
; **    relative to start of data block
; ** "e" = same as "w" above, followed by "r" below to echo back value
; ** "r"+<canid>+<table id>+<offset lsb>+<offset msb>+<nobytes>+<newbytes> = read and
; **    send back value of a data parameter or block in offset location
; **   offset means data table. FIXME. No specific page change command.
; not implemented: "y" = verify inpram data block = inpflash data block, return no. bytes different.
; ** "b"+<canid>+<table id> = jump to flash burner routine and burn a ram data block into a flash
; **    data block.
; ** "c" = Test communications - echo back Seconds
; ** "t" = receive new table data for local sensor data
; ** "T" = receive new table data for CAN re-transmission to GPIO
; ** "Q" = Send over Embedded Code Revision Number
; ** "S" = Send program title.
; **
; **************************************************************************
*/

    cmd = g_read8(ad);
    if (size == 1) {
        /* single character commands */
        if (cmd == 'A') {
            if (!compat) {

                if (conf_err) {
                    /* if there's a configuration error, then we'll send that back instead of the realtime data */
                    g_write8(0x03, ad);    /* config error packet */
                    ad++;
                    for (x = 0 ; x < 256 ; x++) {
                        cmd = g_read8(0xf700 + x);
                        g_write8(cmd, ad++);
                        if (cmd == 0) {
                            break;
                        }
                    }
                    txgoal = x;
                    if (conf_err >= 200) {
                        conf_err = 0;
                    }
                    flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                    goto EXIT_SERIAL;
                }

                g_write8(1, ad);    /* realtime data packet */
                ad++;
                // copy outpc to txbuf one by one to allow interrupts
            }

            x = ad;
            if (compat < 2) {
                txgoal = sizeof(outpc);
            } else {
                txgoal = 112;
            }
            // takes ~50us to copy the lot over
            cp2serialbuf(ad, (unsigned int)&outpc, txgoal);

            if (compat && (ram4.Temp_Units & 1)) { /* re-write these two for Megaview if needed */
                x = 0xf003 + 20;
                g_write_generic(((outpc.mat - 32) * 5) / 9, &x, sizeof(outpc.mat));
                g_write_generic(((outpc.clt - 32) * 5) / 9, &x, sizeof(outpc.clt));
            }

            if (compat < 2) {
                /* MS3 different from now */
                if (outpc.syncreason) {
                    outpc.syncreason = 0;       // send the sync loss reason code once then reset it
                }
            } else {
                x = 0xf003 + 72;
                GWR(outpc.EAEfcor1);
                GWR(outpc.egoV1);
                GWR(outpc.egoV2);
                g_write_generic(0, &x, 2); /* amc Updates */
                g_write_generic(0, &x, 2); /* kpaix */
                GWR(outpc.EAEfcor2);
                g_write_generic(0, &x, 2); /* spare1 */
                g_write_generic(0, &x, 2); /* spare2 */
                g_write_generic(0, &x, 2); /* trigfix */
                g_write_generic(0, &x, 2); /* spare4 */
                g_write_generic(0, &x, 2); /* spare5 */
                g_write_generic(0, &x, 2); /* spare6 */
                g_write_generic(0, &x, 2); /* spare7 */
                g_write_generic(0, &x, 2); /* spare8 */
                g_write_generic(0, &x, 2); /* spare9 */
                g_write_generic(0, &x, 2); /* spare10 */
                g_write_generic(0, &x, 2); /* tachcount */
                g_write_generic(0, &x, 2); /* ospare/cksum */
                g_write_generic(0, &x, 2); /* deltaT */
                g_write_generic(0, &x, 2); /* deltaT pt.2 */
            }

            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;

        } else if (cmd == 'C') {
            /* test data packet */
            if (!compat) {
                g_write8(0, ad); /* OK */
                ad++;
            }
            x = ad;
            // copy outpc to txbuf one by one to allow interrupts
            GWR(outpc.seconds);
            txgoal = 2;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;

        } else if (cmd == 'Q') {
            /* serial format */
            if (!compat) {
                g_write8(0, ad); /* OK */
                ad++;
            }
            for (x = 0; x < SIZE_OF_REVNUM; x++) {
                g_write8(RevNum[x], ad + x);
            }
            txgoal = SIZE_OF_REVNUM;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else if (cmd == 'S') {
            /* string for title bar */
            if (!compat) {
                g_write8(0, ad);
                ad++;
            }
            for (x = 0; x < SIZE_OF_SIGNATURE; x++) {
                g_write8(Signature[x], ad + x);
            }
            txgoal = SIZE_OF_SIGNATURE;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else if (cmd == 'F') {
            /* format i.e. 001 = newserial */
            if (!compat) {
                g_write8(0, ad);
                ad++;
            }
            g_write8('0', ad++);
            g_write8('0', ad++);
            g_write8('1', ad);
            txgoal = 3;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else if (cmd == 'I') {
            /* return my CANid */
            if (!compat) {
                g_write8(0, ad);
                ad++;
            }
            g_write8(ram4.mycan_id, ad);
            txgoal = 1;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }

    } else if ((size == 3) && (cmd == 'b')) {
        /* need a way of avoiding clashes between remotely and locally initiated burn */
        r_canid = g_read8(0xf003);
        r_table = g_read8(0xf004);

        if ((r_canid >= MAX_CANBOARDS) || (r_table >= NO_TBLES)) {
            goto INVALID_OUTOFRANGE;
        }

        if (r_canid != ram4.mycan_id) {
            unsigned int r;
            if (flagbyte3 & (flagbyte3_getcandat)) {
                g_write8(0x85, 0xf002); // BUSY - in the middle of a CAN transaction

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            /* pass on via CAN */
            DISABLE_INTERRUPTS;
            r = can_sendburn(r_table, r_canid); // ring0
            ENABLE_INTERRUPTS;
            if (r) {
                g_write8(0x89, 0xf002); // CAN overflow code

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            goto RETURNOK_SERIAL; // ok code - command is in queue but we don't know if it _actually_ worked
        } else {
            /* local burn */
            if (burnstat) {
                /* burn already in progress ! */
                g_write8(0x85, 0xf002); // busy code
                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            if (flagbyte1 & flagbyte1_tstmode) {
                /* no more burning during test mode */
                g_write8(0x85, 0xf002); // say we are busy
                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            burn_idx = r_table;
            burnstat = 1;
            flocker = 0xdd;
            flagbyte14 |= FLAGBYTE14_SERIAL_BURN; // burn mode
            flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND; // don't send anything yet
            goto EXIT_SERIAL;
        }

    } else if ((size == 7) && (cmd == 'k')) {
        /* CRC of 1k ram page */
        /* or of 16k memory region - future */
        r_canid = g_read8(0xf003);
        r_table = g_read8(0xf004);
        /* ignore the other 4 bytes */

        if ((r_canid >= MAX_CANBOARDS) || (r_table < 0x70 && (r_table >= NO_TBLES))
            || (r_table > 0x7f)) {
            goto INVALID_OUTOFRANGE;
        }

        if (r_canid != ram4.mycan_id) {
            unsigned int r;

            if (flagbyte3 & (flagbyte3_getcandat)) {
                g_write8(0x85, 0xf002); // BUSY - in the middle of a CAN transaction

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            /* pass on via CAN */
            canrxad = 0xf003;
            canrxgoal = 4;
            g_write16(canrxgoal, 0xf000);

            DISABLE_INTERRUPTS;
            r = can_crc32(r_table, r_canid); // ring 0
            ENABLE_INTERRUPTS;
            if (r) {
                g_write8(0x89, 0xf002); // CAN overflow code

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }
            flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND; // don't send anything yet
            goto EXIT_SERIAL;
        } else {
            /* local CRC */
            g_write8(0, ad);
            ad++;
            if (r_table < 0x40) {
                if ((unsigned int)tables[r_table].addrRam == 0) {
                    unsigned char gp_tmp;
                    /* Has no RAM address, check in flash */
                    gp_tmp = GPAGE;
                    GPAGE = 0x10; // eeprom
                    crc = g_crc32buf(0, tables[r_table].addrFlash, tables[r_table].n_bytes);    // incrc, buf, size
                    GPAGE = gp_tmp;
                } else {
                    /* CRC of RAM page */
                    rp = tables[r_table].rpg;
                    if (rp) {
                        RPAGE = rp;
                    }
                    crc = crc32buf(0, tables[r_table].adhi << 8, 0x400);    // incrc, buf, size
                }
            } else {
                /* 'table' is GPAGE value and calc CRC of a l<<8 bytes */
                unsigned int ad;
                unsigned long ll;
                ad = g_read8(0xf005) << 8;
                /* without the double cast, gcc erroneously sign extends the intermediate 16bit result */
                ll = (unsigned long)((unsigned int)(g_read8(0xf006) << 8));
                if (ll == 0) {
                    ll = 0x10000;
                }
                if ((r_table == 0x7f) && (((unsigned long)ad + ll) > 0xf000)) {
                    outpc.sensors[0] = ad;
                    outpc.sensors[1] = ll >> 16;
                    outpc.sensors[2] = (unsigned int)ll;

                    /* avoid the monitor  */
                    goto INVALID_OUTOFRANGE;
                }
                GPAGE = r_table;
                crc = g_crc32buf(0, ad, (unsigned int)ll);
                GPAGE = 0x13;
            }
            g_write32(crc, ad);
            txgoal = 4;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }

    } else if ((size == 7) && (cmd == 'r')) {
        ad++;
        r_canid = g_read8(ad);
        ad++;
        r_table = g_read8(ad);
        ad++;
        r_offset = g_read16(ad);
        ad += 2;
        r_size = g_read16(ad);

        if ((r_canid >= MAX_CANBOARDS)
            || (r_size > 0x4000) || (r_offset >= 0x4000) /* Prevent wrap-around overflow */
            || ((r_table >= NO_TBLES) && ((r_table < 0xf0) || (r_table > 0xf4)))) {
            goto INVALID_OUTOFRANGE;
        }

        if (r_canid != ram4.mycan_id) {

            if (flagbyte3 & (flagbyte3_getcandat)) {
                g_write8(0x85, 0xf002); // BUSY - in the middle of a CAN transaction

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            /* pass on via CAN */
            unsigned int by, r = 0;

            canrxad = 0xf003;
            canrxgoal = r_size;
            g_write16(canrxgoal, 0xf000);

            if (r_size <= 8) {
                by = r_size;
            } else {
                by = 8;
            }

            DISABLE_INTERRUPTS;
            r = can_reqdata(r_table, r_canid, r_offset, by); // uses ring0
            ENABLE_INTERRUPTS;
            if (r) {
                g_write8(0x89, 0xf002); // CAN overflow code

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            if (r_size > 8) {
                /* set up other passthrough read, variables handled in CAN TX ISR */
                cp_id = r_canid;
                cp_table = r_table;
                cp_targ = r_size - 8;
                cp_cnt = 0;
                cp_offset = r_offset + 8;
            }
            flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND; // don't send anything yet
            goto EXIT_SERIAL;
        } else {
            unsigned int ram_ad;
            if ((r_table >= 0xf0) && (r_table <= 0xf4)) {
                /* logger pages
                0xf0 = tooth logger
                0xf1 = trigger logger
                0xf2 = composite logger
                0xf3 = sync error composite logger
                0xf4 = map logger (not yet implemented in TS)
                 */
//                if ((r_table == 0xf0) && (ram4.feature3 & 0x04)) {
//                    r_table = 0xf4; /* dev option to log MAP in place of tooth logger */
//                }
                flagbyte0 &= ~(FLAGBYTE0_TTHLOG | FLAGBYTE0_TRGLOG | FLAGBYTE0_COMPLOG | FLAGBYTE0_MAPLOG | FLAGBYTE0_MAPLOGARM);
                /* ensure tooth logger off, ensure trigger logger off, ensure composite tooth logger off */

                outpc.status3 &= ~STATUS3_DONELOG; /* clear 'complete/available' flag */

                RPAGE = TRIGLOGPAGE;
                ram_ad = TRIGLOGBASE;
                if (page != r_table) {
                    /* first read - zap page to prevent garbage */
                    for (x = TRIGLOGBASE; x < (TRIGLOGBASE + 0x400); x++) {
                        *(unsigned char*)x = 0;
                    }
                } else if (r_table == 0xf4) {
                    /* process teeth into angles in map logger */
                    for (x = TRIGLOGBASE; x < (TRIGLOGBASE + 0x400); x+=2) {
                        unsigned int t;
                        t = *(unsigned int*)x;
                        if ((t & 0xc000) == 0x8000) {
                            t = t & 0x3fff;
                            if (t == no_teeth) {
                                t = 0;
                            }
                            t = tooth_absang[t];
                            t |= 0x8000;
                            *(unsigned int*)x = t;
                        }
                    }
                }
                page = r_table;
                flagbyte14 |= FLAGBYTE14_SERIAL_TL;
                log_offset = 0;
            } else {
                /* local read */
                if ((r_table != 0x14) && ((r_table > NO_TBLES)
                    || ((r_table != 7) && ((r_offset + r_size) > tables[r_table].n_bytes))
                    || ((r_table == 7) && (r_offset < 0x200) && ((r_offset + r_size) > tables[7].n_bytes))
                    || ((r_table == 7) && (r_offset >= 0x200) && ((r_offset + r_size) > (0x200 + sizeof(datax1))))
                    )) {
                    /* last two lines to handle outpc vs. datax1 in same 'page' */
                    goto INVALID_OUTOFRANGE;
                }

                if (r_table == 0x14) { /* SDcard file readback */
                    /* not yet documented
                        sd_sync2 does this for new-serial
                        buf[2] = 'r';
                        buf[3] = canid;
                        buf[4] = 0x14; // sd_buf
                        buf[5] = blk_num >> 8; // start
                        buf[6] = blk_num & 0xff; 
                        buf[7] = 0x08; // ignored anyway
                        buf[8] = 0x00;
                    */
                    if (r_offset == sd_rb_block) {
                        /* asked for next block */
                        flagbyte7 |= FLAGBYTE7_SF_GO; // actual send setup is handled in the compress
                        goto EXIT_SERIAL;
                    } else {
                        /* not yet supporting old ones */
                        /* do want to add this to allow retrying without requesting the file all over again */
                        goto INVALID_SERIAL;
                    }
                } else if ((r_table == 0x10) && (flagbyte8 & FLAGBYTE8_MODE10)) { // special handling
                    /* restart timeout */
                    mode10_time = (unsigned int)lmms;
                }

                RPAGE = tables[r_table].rpg;
                if ((r_table == 7) && (r_offset >= 0x200)) {
                    ram_ad = (unsigned int)&datax1;
                    r_offset -= 0x200;
                } else {
                    ram_ad = (unsigned int)tables[r_table].addrRam;
                }
            }

            if (ram_ad == 0) {
                /* No valid RAM copy to read back
                    This will happen if user tries to read calibration tables.
                    Could extend code to pull back flash, but not yet. */
                goto INVALID_OUTOFRANGE;
            }

            if (!compat) {
                g_write8(0, 0xf002); // OK
                ad = 0xf003;
            } else {
                ad = 0xf000;
            }
            for (x = 0; x < r_size; x++) {
                g_write8(*(unsigned char*)(ram_ad + r_offset + x), ad + x);
            }
            txgoal = r_size;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
        
    } else if ((size > 7) && ((cmd == 'w') || (cmd == 'x'))) {
        r_canid = g_read8(0xf003);
        r_table = g_read8(0xf004);
        r_offset = g_read16(0xf005);
        r_size = g_read16(0xf007);

        if ((r_canid >= MAX_CANBOARDS)
            || (r_size > 0x4000) || (r_offset >= 0x4000) /* Prevent wrap-around overflow */
            || (r_table >= NO_TBLES)) {
            goto INVALID_OUTOFRANGE;
        }

        if (r_canid != ram4.mycan_id) {
            unsigned int by, r = 0;

            if (flagbyte3 & (flagbyte3_getcandat)) {
                g_write8(0x85, 0xf002); // BUSY - in the middle of a CAN transaction

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            /* pass on via CAN */
            if (r_size <= 8) {
                by = r_size;
            } else {
                by = 8;
            }
            DISABLE_INTERRUPTS;
            r = can_snddata(r_table, r_canid, r_offset, by, 0xf009); // ring0
            ENABLE_INTERRUPTS;
            if (r) {
                g_write8(0x89, 0xf002); // CAN overflow code

                txgoal = 0;
                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                goto EXIT_SERIAL;
            }

            if (r_size > 8) {
                /* set up a passthrough write, handled in CAN TX ISR */
                /* when all packets in queue, flag will be set to tell PC */
                flagbyte3 |= flagbyte3_sndcandat;
                cp_id = r_canid;
                cp_table = r_table;
                cp_targ = r_size;
                cp_cnt = 8;
                cp_offset = r_offset;
            } else {
                /* for a single packet say ok now */
                if (cmd == 'x') {
                    /* no-ACK command */
                    flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND;
                    goto EXIT_SERIAL;
                } else {
                    goto RETURNOK_SERIAL;
                }
            }
            flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        } else {
            unsigned char ch = 0;
            unsigned int ram_ad;
            /* local write */
            if (cmd != 'w') {
                goto INVALID_SERIAL;
            }

            if ((r_table > NO_TBLES)
                || ((r_table != 7) && ((r_offset + r_size) > tables[r_table].n_bytes))
                || ((r_table == 7) && (r_offset < 0x200) && ((r_offset + r_size) > tables[7].n_bytes))
                || ((r_table == 7) && (r_offset >= 0x200) && ((r_offset + r_size) > (0x200 + sizeof(datax1))))
                ) {
                goto INVALID_OUTOFRANGE;
            }

            /* copy to ram page */
            /* FIXME - validate page numbers using a field in tables? */
            if ((r_table == 4) || (r_table == 5) || (r_table > 6)) { 
                if (r_table == 0x10) { // special handling
                    if (r_size != 0x40) {
//                        goto INVALID_OUTOFRANGE;
                        g_write8(0x87, 0xf002); // FIXME 0x87
                        txgoal = 0;
                        flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                        goto EXIT_SERIAL;
                    }

                    if (outpc.rpm) {
                        g_write8(0x85, 0xf002); // busy
                        txgoal = 0;
                        flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                        goto EXIT_SERIAL;
                    }

                    if (!(flagbyte8 & FLAGBYTE8_MODE10)) {
                        for (x = 0xf009 ; x < 0xf049 ; x++) {
                            if (g_read8(x)) {
                                g_write8(0x88, 0xf002); // FIXME 0x88
                                txgoal = 0;
                                flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
                                goto EXIT_SERIAL;
                            }
                        }
                        flagbyte8 |= FLAGBYTE8_MODE10;
                    }
                    mode10_time = (unsigned int)lmms;
                    flagbyte1 |=FLAGBYTE1_EX;
                }

                rp = tables[r_table].rpg;
                if (rp) {
                    RPAGE = rp;
                }

                if (r_table == 0x11) { /* SDcard operations */
                    /* use r_offset as command no. */

                    if (r_offset == 4) { // sd_stream:
                        /* unlike other newserial, this just spews back raw data */
                        if (g_read8(0xf009)) {
                            flagbyte6 |= FLAGBYTE6_STREAM_CONT;
                            *(port_sci + 3) &= ~0x80; //xmit interrupt disable // SCIxCR2
                            *(port_sci + 3) |= 0x08; // xmit enable // SCIxCR2
                            goto EXIT_SERIAL;

                        } else {
                            /* unlikely to be able to see this command */
                            flagbyte6 &= ~FLAGBYTE6_STREAM_CONT;
                            goto RETURNOK_SERIAL;
                        }
                    /* all following commands require a card to be present */
                    } else if (!(outpc.sd_status & 1)) {
                        goto BUSY_SERIAL;

                    } else if (r_offset == 0) { /* do command */
                        unsigned char cmd2;
                        cmd2 = g_read8(0xf009);
                        if (cmd2 == 0) { // sd_resetgo:
                            sd_phase = 0;
                            flagbyte6 &= ~FLAGBYTE6_SD_MANUAL;
                            flagbyte15 &= ~FLAGBYTE15_SDLOGRUN;
                            flagbyte10 &= ~FLAGBYTE10_SDERASEAUTO;
                            goto RETURNOK_SERIAL;
                        } else if (cmd2 == 1) { // sd_resetwait:
                            sd_phase = 0;
                            flagbyte6 |= FLAGBYTE6_SD_MANUAL;
                            flagbyte15 &= ~FLAGBYTE15_SDLOGRUN;
                            goto RETURNOK_SERIAL;
                        } else if (cmd2 == 2) { // sd_stoplog:
                            if (!(outpc.sd_status & 8)) { // not allowed while NOT logging
                                goto BUSY_SERIAL;
                            }
                            flagbyte6 |= FLAGBYTE6_SD_MANUAL;
                            flagbyte15 &= ~FLAGBYTE15_SDLOGRUN;
                            sd_phase = 0x48;
                            goto RETURNOK_SERIAL;
                        } else if (cmd2 == 3) { // sd_startlog:
                            if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                                goto BUSY_SERIAL;
                            }
                            flagbyte6 |= FLAGBYTE6_SD_MANUAL;
                            flagbyte15 |= FLAGBYTE15_SDLOGRUN;
                            /* FIXME - need to check if we are allowed to do this */
                            sd_phase = 0x30; // update time in dirent first
                            goto RETURNOK_SERIAL;
                        } else if (cmd2 == 4) { // sd_sendstat:
                            if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                                goto BUSY_SERIAL;
                            }
                            // put data into buffer ready for fetch
                            *(unsigned char*)0x1000 = outpc.sd_status;
                            *(unsigned char*)0x1001 = outpc.sd_error;
                            *(unsigned int*)0x1002 = sd_sectsize;
                            *(unsigned int*)0x1004 = u32MaxBlocks >> 16;
                            *(unsigned int*)0x1006 = (unsigned int)u32MaxBlocks;
                            *(unsigned int*)0x1008 = sd_max_roots;
                            *(unsigned int*)0x100a = sd_dir_start >> 16;
                            *(unsigned int*)0x100c = (unsigned int)sd_dir_start;
                            *(unsigned int*)0x100e = 0;
                            goto RETURNOK_SERIAL;
                        } else if (cmd2 == 5) { // sd_init:
                            flagbyte15 &= ~FLAGBYTE15_SDLOGRUN;
                            sd_phase = 0x00;
                            goto RETURNOK_SERIAL;
                        } else {
                            goto INVALID_SERIAL;
                        }
                    } else if (r_offset == 1) { // sd_readdir:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 2) {
                            goto INVALID_SERIAL;
                        }
                        sd_pos = sd_dir_start + g_read16(0xf009);
                        sd_phase = 0x50;
                        sd_uitmp = 0;
                        outpc.sd_status &= ~0x04;
                        goto RETURNOK_SERIAL;

                    } else if (r_offset == 2) { // sd_readsect:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 4) {
                            goto INVALID_SERIAL;
                        }
                        sd_pos = g_read32(0xf009);
                        sd_phase = 0x54;
                        outpc.sd_status &= ~0x04;
                        goto RETURNOK_SERIAL;

                    } else if (r_offset == 3) { // sd_writesect:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 0x204) {
                            goto INVALID_SERIAL;
                        }
                        sd_pos = g_read32(0xf209);
                        /* copy from serial buffer to SDcard buffer */
                        for (x = 0; x < 0x200; x+=2) {
                            unsigned int ad1, v;
                            v = g_read16(0xf009 + x);
                            ad1 = 0x1000 + x;
                            *(unsigned int*)ad1 = v;
                        }
                        sd_phase = 0x58;
                        outpc.sd_status &= ~0x04;
                        sd_ledstat = 2;
                        goto RETURNOK_SERIAL;

                    } else if (r_offset == 5) { // sd_readfile:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 8) {
                            goto INVALID_SERIAL;
                        }
                        outpc.sd_status &= ~0x04; // not ready
                        sd_pos = g_read32(0xf009);
                        sd_filesize_bytes = g_read32(0xf00d);
                        flagbyte7 |= FLAGBYTE7_SENDFILE; // trigger mainloop sending
                        flagbyte7 &= ~FLAGBYTE7_SF_GO; // wait until asked
                        goto RETURNOK_SERIAL; // FIXME - needs review !

                    } else if (r_offset == 6) { // sd_erasefile:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 6) {
                            goto INVALID_SERIAL;
                        }
                        *(unsigned long*)0x1200 = g_read32(0xf009);
                        sd_uitmp2 = g_read16(0xf00d);
                        sd_pos = sd_dir_start + sd_uitmp2;
                        sd_phase = 0x88;
                        goto RETURNOK_SERIAL;

                    } else if (r_offset == 7) { // sd_speedtest:
                        if ((outpc.sd_status & 8) || (!(outpc.sd_status & 4))) { // logging || !ready
                            goto BUSY_SERIAL;
                        }
                        if (r_size != 8) {
                            goto INVALID_SERIAL;
                        }
                        sd_pos = g_read32(0xf009);
                        *(unsigned long*)0x1000 = g_read32(0xf00d);
                        sd_phase = 0xa0;
                        outpc.sd_status &= ~0x04; // not ready

                        goto RETURNOK_SERIAL;
                    }
                    /* if nothing caught yet, must be invalid */
                    goto INVALID_SERIAL;
                }
                /* back to normal transactions */

                if ((r_table == 7) && (r_offset >= 0x200)) {
                    ram_ad = (unsigned int)&datax1 + r_offset - 0x200;
                } else {
                    ram_ad = (unsigned int)tables[r_table].addrRam + r_offset;
                }

                ch = g_read_copy(0xf009, r_size, ram_ad);// src, count, dest

                if ((r_table == 7) && (r_offset >= 0x200)) {
                    /* test mode special code */
                        if (datax1.testmodelock == 12345) {
                            if (datax1.testmodemode == 0) {
                                flagbyte1 &= ~flagbyte1_tstmode; /* disable test mode */
                                outpc.status3 &= ~STATUS3_TESTMODE;
                                /* Put injector port/pin mappings back to normal (same code as in init) */
                                inj_event_array_portpin(0);
                                if (do_dualouts) {
                                    inj_event_array_portpin(INJ_FILL_STAGED);
                                }
                                testmode_glob = 0;
                            } else if (datax1.testmodemode == 1) {
                                flagbyte1 |= flagbyte1_tstmode; /* enable test mode */
                                outpc.status3 |= STATUS3_TESTMODE;
                                inj_event_array_portpin(INJ_FILL_TESTMODE);
                                testmode_glob = 0;
                                // accept the test mode and return ok
                                goto RETURNOK_SERIAL;
                            } else if (flagbyte1 & flagbyte1_tstmode) {
                                // only allow test modes to run when enabled
                                if (datax1.testmodemode == 2) { // coil testing
                                    if (outpc.rpm != 0) {
                                        goto BUSY_SERIAL;
                                    } else {
                                        // enable the mode
                                        testmode_glob = 1;
                                        goto RETURNOK_SERIAL;
                                    }
                                } else if (datax1.testmodemode == 3) { // inj testing
                                    if (outpc.rpm != 0) {
                                        goto BUSY_SERIAL;
                                    } else {
                                        // enable the mode
                                        testmode_cnt = ram5.testinjcnt;
                                        testmode_glob = 2;
                                        goto RETURNOK_SERIAL;
                                    }
                                // 4 not used
                                } else if (datax1.testmodemode == 5) { // FP on
                                    SSEM0;
                                    *port_fp |= pin_fp;
                                    CSEM0;
                                    outpc.engine |= ENGINE_READY;
                                } else if (datax1.testmodemode == 6) { // FP off
                                    SSEM0;
                                    *port_fp &= ~pin_fp;
                                    CSEM0;
                                    outpc.engine &= ~ENGINE_READY;
                                } else if (datax1.testmodemode == 7) { // Cancel inj or spk
                                    testmode_glob = 0;
                                    outpc.istatus5 = 0; // testmode injection counter
/* plenty more */
/*"PM3 - Injection LED D14", "PM4 - Accel LED D16", "PM5 - Warmup LED D15", "PJ0 - IAC2", "PJ1 - IAC1", "PJ7 - JS11", "PP2 - Idle", "PP3 - Boost", "PP4 - Nitrous 1", "PP5 - Nitrous 2", "PP6 - VVT", "PP7 - Fidle", "PT1 - V3 Inj 1", "PT3 - V3 Inj 2", "PT5 - JS10", "PK0 - Tacho", "PA0 - Inj A", "PA1 - Inj B", "PA2 - Inj C", "PA3 - Inj D", "PA4 - Inj E", "PA5 - Inj F", "PA6 - Inj G", "PA7 - Inj H", 
*/
                                } else if (datax1.testmodemode & 0x80) { // I/O ports
                                    unsigned char ioport, ioport_mode;
                                    ioport = datax1.testmodemode & 0x7c;
                                    ioport_mode = datax1.testmodemode & 0x03;
                                    if (ioport == 0x00) { // PP3
                                        PWME &= ~0x08;
                                        DDRP |= 0x08;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x08;
                                    } else if (ioport == 0x04) { // PP4
                                        PWME &= ~0x10;
                                        DDRP |= 0x10;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x10;
                                    } else if (ioport == 0x08) { // PP5
                                        PWME &= ~0x20;
                                        DDRP |= 0x20;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x20;
                                    } else if (ioport == 0x0c) { // PP6
                                        PWME &= ~0x40;
                                        DDRP |= 0x40;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x40;
                                    } else if (ioport == 0x10) { // PP7
                                        PWME &= ~0x80;
                                        DDRP |= 0x80;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x80;
                                    } else if (ioport == 0x14) { // PK0
                                        DDRK |= 0x01;
                                        port_testio = (unsigned char*)&PORTK;
                                        pin_testio = 0x01;
                                    } else if (ioport == 0x18) { // PP2
                                        PWME &= ~0x04;
                                        DDRP |= 0x04;
                                        port_testio = (unsigned char*)&PTP;
                                        pin_testio = 0x04;
                                    } else {
                                        port_testio = (unsigned char*)&dummyReg;
                                        pin_testio = 0;
                                    }

                                    testmode_glob = 0;
                                    if (ioport_mode == 0) {
                                        *port_testio &= ~pin_testio;
                                    } else if (ioport_mode == 2) {
                                        testmode_glob = 3;
                                    } else if (ioport_mode == 3) {
                                        *port_testio |= pin_testio;
                                    }

                                } else { // invalid mode
                                    goto INVALID_OUTOFRANGE;
                                }
                            } else {
                                /* IAC tests do not require and not possible when in inj/spk test mode */
                                if (datax1.testmodemode == 8) { // Cancel IAC test
                                    iactest_glob = 0;
                                    if ((outpc.rpm == 0) && ((IdleCtl == 4) || (IdleCtl == 6))) {
                                        pwmidle_reset = 4;
                                    }
                                } else if (datax1.testmodemode == 9) { // IAC test home
                                    iactest_glob = 1;
                                } else if (datax1.testmodemode == 10) { // IAC test run
                                    iactest_glob = 3; // nb 2 is used as a holding phase after homing
                                } else {
                                    // tried to run an invalid test when not in test mode?!
                                    goto INVALID_OUTOFRANGE;
                                }
                            }
                        } else {
                            flagbyte1 &= ~flagbyte1_tstmode; /* disable test mode */
                            outpc.status3 &= ~STATUS3_TESTMODE;
                        }
                }

                if (ch &&
                    ((r_table == 4) || (r_table == 5) || (r_table == 8)
                    || (r_table == 9) || (r_table == 10) || (r_table == 11)
                    || (r_table == 12) || (r_table == 13) || (r_table == 18)
                    || (r_table == 19) || (r_table == 21) || (r_table == 22)
                    || (r_table == 23) || (r_table == 24) )) { /* only applies to tuning data pages */
                    if (!(flagbyte1 & flagbyte1_tstmode)) {
                        outpc.status1 |= STATUS1_NEEDBURN; /* flag burn needed if anything changed */
                    }
                }
                g_write8(0x00, 0xf002); // ok code
            } else {
                goto INVALID_OUTOFRANGE;
            }

            txgoal = 0;

            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }

    } else if ((size == 14)
        && (g_read8(0xf002) == '!')
        && (g_read8(0xf003) == '!')
        && (g_read8(0xf004) == '!')
        && (g_read8(0xf005) == 'S')
        && (g_read8(0xf006) == 'a')
        && (g_read8(0xf007) == 'f')
        && (g_read8(0xf008) == 'e')
        && (g_read8(0xf009) == 't')
        && (g_read8(0xf00a) == 'y')
        && (g_read8(0xf00b) == 'F')
        && (g_read8(0xf00c) == 'i')
        && (g_read8(0xf00d) == 'r')
        && (g_read8(0xf00e) == 's')
        && (g_read8(0xf00f) == 't')) {
        /* jumperless reboot, shut down and enter monitor through COP timeout */
        DISABLE_INTERRUPTS;
        // turn off fuel pump. User is instructed to have coils wired via the relay
        *port_fp &= ~pin_fp;
        PORTA = 0; // turn off injectors - FIXME - ASSUMES NOTHING ELSE ON THOSE PORTS

        coilsel = 0xffff;
        FIRE_COIL;
        //Insert code here to reset ports to their default values - reading from D-flash

        // Monitor will check for this string on reboot
        *(unsigned char*)0x3ff0 = 'R';
        *(unsigned char*)0x3ff1 = 'u';
        *(unsigned char*)0x3ff2 = 'n';
        *(unsigned char*)0x3ff3 = 'M';
        *(unsigned char*)0x3ff4 = 'o';
        *(unsigned char*)0x3ff5 = 'n';
        *(unsigned char*)0x3ff6 = 'i';
        *(unsigned char*)0x3ff7 = 't';
        *(unsigned char*)0x3ff8 = 'o';
        *(unsigned char*)0x3ff9 = 'r';
        COPCTL = 0x41;
        while (1) { ; };

    } else if (cmd == 't') {
        r_table = g_read8(0xf003);
        if (size != (tables[r_table].n_bytes + 2)) {
            goto INVALID_OUTOFRANGE;
        }

        if ((ram5.flashlock & 1) == 0) {
            g_write8(0x86, 0xf002); // invalid command code
            txgoal = 1;
            flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
            goto EXIT_SERIAL;
        }
    
        burn_idx = r_table;
        flocker = 0xcc;
        erasefactor();
        burnstat = 5; // writing handle in dribble_burn
        flocker = 0xdd;
        flagbyte14 |= FLAGBYTE14_SERIAL_BURN; // burn mode
        flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND; // don't send anything yet
        goto EXIT_SERIAL;
    }

/* ------------------------------------------------------------------------- */
INVALID_SERIAL:;
/* Should not be reached - must be invalid command - build return error packet */
    srl_err_cnt++;
    g_write8(0x83, 0xf002); // invalid command code
    txgoal = 0;
    flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
    goto EXIT_SERIAL;

/* ------------------------------------------------------------------------- */
BUSY_SERIAL:;
    g_write8(0x85, 0xf002); // busy
    txgoal = 0;
    flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
    goto EXIT_SERIAL;

/* ------------------------------------------------------------------------- */
RETURNOK_SERIAL:;
    g_write8(0x00, 0xf002); // ok
    txgoal = 0;
    flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
    goto EXIT_SERIAL;

/* ------------------------------------------------------------------------- */
INVALID_OUTOFRANGE:;
/* Should not be reached - must be invalid command - build return error packet */
    srl_err_cnt++;
    g_write8(0x84, 0xf002); // out of range command code
    txgoal = 0;
    flagbyte14 |= FLAGBYTE14_SERIAL_SEND;
    goto EXIT_SERIAL;

/* ------------------------------------------------------------------------- */

EXIT_SERIAL:;
    flagbyte14 &= ~FLAGBYTE14_SERIAL_PROCESS;

    if (flagbyte14 & FLAGBYTE14_SERIAL_SEND) {
        if (srl_err_cnt > 3) {
            next_txmode = 7;
            rxmode = 0;
            txmode = 0;
            (void)SCI0DRL; // read assumed garbage data to clear flags
            SCI0CR2 &= ~0xAC;   // rcv, xmt disable, interrupt disable
            flagbyte3 |= flagbyte3_kill_srl;
            srl_timeout = (unsigned int)lmms;
            return;
        }

        if (!compat) {
            txgoal++; /* 1 byte return code */
#ifdef SERIAL_TIMING2
    outpc.istatus5 = TCNT;
#endif
            g_write16(txgoal, 0xf000);
            g_write32(g_crc32buf(0, 0xf002, txgoal), 0xf002 + txgoal);
#ifdef SERIAL_TIMING2
    outpc.istatus5 = TCNT - outpc.istatus5;
#endif
            txgoal += 6; /* 2 bytes size, 4 bytes CRC */
        }
        next_txmode = 0;
        txcnt = 0;
        txmode = 129;
        *(port_sci + 7) = g_read8(0xf000); // SCIxDRL
        *(port_sci + 3) |= 0x88; // xmit enable & xmit interrupt enable // SCIxCR2
        flagbyte14 &= ~FLAGBYTE14_SERIAL_SEND;
#ifdef SERIAL_TIMING
        /* debug */
        if (outpc.status4 == 3) {
            outpc.sensors[3] = TCNT - outpc.sensors[0];
            outpc.sensors[0] = TCNT;
            outpc.status4++;
        }
        /* end debug */
#endif
    }
    return;
}

/* ------------------------------------------------------------------------- */

void debug_init(void)
{
    db_ptr = 0;
    debug_str("MS3 debug buffer\n");
    debug_str("================\n");
}

void debug_str(unsigned char* str)
{
    unsigned char c, m = 0;
    GPAGE = 0x13;
    do {
        c = *str;
        str++;
        if (c) {
            g_write8(c, DB_BUFADDR + db_ptr);
            db_ptr++;
            if (db_ptr >= DB_BUFSIZE) {
                db_ptr = 0;
                flagbyte15 |= FLAGBYTE15_DB_WRAP;
            }
        }
        m++;
    } while ((c != 0) && (m < 80));
}

void debug_byte(unsigned char c)
{
    GPAGE = 0x13;
    g_write8(c, DB_BUFADDR + db_ptr);
    db_ptr++;
    if (db_ptr >= DB_BUFSIZE) {
        db_ptr = 0;
        flagbyte15 |= FLAGBYTE15_DB_WRAP;
    }
}

void debug_bytehex(unsigned char b)
{
    /* logs byte b in hex */
    unsigned char c;

    c = b >> 4;
    if (c < 10) {
        c += 48;
    } else {
        c += 55;
    }
    debug_byte(c);

    c = b & 0x0f;
    if (c < 10) {
        c += 48;
    } else {
        c += 55;
    }
    debug_byte(c);
}

void debug_bytedec(unsigned char b)
{
    /* logs byte b in decimal, 3 digits */
    unsigned char c;

    c = b / 100;
    c += 48;
    debug_byte(c);

    c = (b / 10) % 10;
    c += 48;
    debug_byte(c);

    c = b % 10;
    c += 48;
    debug_byte(c);
}

void debug_byte2dec(unsigned char b)
{
    /* logs byte b in decimal, 2 digits */
    unsigned char c;

    c = (b / 10) % 10;
    c += 48;
    debug_byte(c);

    c = b % 10;
    c += 48;
    debug_byte(c);
}

void debug_inthex(unsigned int b)
{
    debug_bytehex((b >> 8) & 0xff);
    debug_bytehex((b >> 0) & 0xff);
}

void debug_longhex(unsigned long b)
{
    debug_bytehex((b >> 24) & 0xff);
    debug_bytehex((b >> 16) & 0xff);
    debug_bytehex((b >> 8) & 0xff);
    debug_bytehex((b >> 0) & 0xff);
}
