/* $Id: ms3_can_isr.c,v 1.34.2.1 2013/01/25 11:40:28 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * CanTxIsr()
    Origin: Al Grippo.
    Moderate: Changes for MS2/Extra and MS3 by James Murray
    Majority: Al Grippo / James Murray
 * CanRxIsr()
    Origin: Al Grippo.
    Moderate: Changes for MS2/Extra and MS3 by James Murray
    Majority: Al Grippo / James Murray
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/
#include "ms3.h"

INTERRUPT void CanTxIsr(void)
{
    unsigned char ix, jx, kx, sw;

    /* CAN Xmit Interrupt */
    CAN0TBSEL = CAN0TFLG;       // select MSCAN xmit buffer
    // Check ring buffers and xfer to MSCAN buffer
    for (ix = 0; ix < 2; ix++) {
        if (can[ix].cxno) {
            jx = can[ix].cxno_out;
            /* Set up identifier registers */
            CAN0_TB0_IDR0 =
                (unsigned char) (can[ix].cx_destvaroff[jx] >> 3);
            // 8 high bits in IDR0, 3 low bits in IDR1
            CAN0_TB0_IDR1 = (unsigned char) ((can[ix].cx_destvaroff[jx] & 0x0007) << 5) | 0x18 |        // SRR=IDE=1
                (can[ix].cx_msg_type[jx] & 0x07);        // 3 bits
            CAN0_TB0_IDR2 = (ram4.mycan_id << 4) | can[ix].cx_dest[jx];
//            CAN0_TB0_IDR3 = (can[ix].cx_destvarblk[jx] << 4);
            CAN0_TB0_IDR3 = (can[ix].cx_destvarblk[jx] << 4) | ((can[ix].cx_destvarblk[jx] & 0x10) >> 1);
            /* Set xmt buffer priorities (lower is > priority) */
            CAN0_TB0_TBPR = 0x02;

            /* set data in buffer */
            // this switch is likely broken because GCC puts jump table in banked memory (duh!)
            // switch changed to if due to bug
            sw = can[ix].cx_msg_type[jx];
            if ((sw == MSG_CMD) || (sw == MSG_RSP)) {
                //case MSG_CMD:  // msg for dest ecu to set a variable to val in msg
                //case MSG_RSP:  // msg in reply to dest ecu's request for a var val
                CAN0_TB0_DLR = can[ix].cx_varbyt[jx];
                for (kx = 0; kx < CAN0_TB0_DLR; kx++) {
                    *(&CAN0_TB0_DSR0 + kx) = can[ix].cx_datbuf[jx][kx];
                }
                //break;
            } else if (sw == MSG_REQ) {
                //case MSG_REQ:  // msg to send back current value of variable(s)
                // this 1st byte holds var blk for where to put rcvd data
                // 2nd,3rd bytes hold var offset rel. to var blk and how
                // many consecutive bytes to be sent back
                CAN0_TB0_DLR = 3;
                CAN0_TB0_DSR0 = can[ix].cx_myvarblk[jx];
                CAN0_TB0_DSR1 = (unsigned char) (can[ix].cx_myvaroff[jx] >> 3);
                CAN0_TB0_DSR2 = (unsigned char) ((can[ix].cx_myvaroff[jx] & 0x0007) << 5) | can[ix].cx_varbyt[jx];
                //break;

            } else if (sw == MSG_XSUB) {
                //case MSG_XSUB:
                CAN0_TB0_DLR = 0;
                //break;

            } else if (sw == MSG_BURN) {
                //case MSG_BURN:
                CAN0_TB0_DLR = 0;
                //break;
            } else if (sw == MSG_CRC) {
                // convert to extended format
                CAN0_TB0_IDR1 = (CAN0_TB0_IDR1 & 0xf8) | MSG_XTND; 
                CAN0_TB0_DLR = 4;
                CAN0_TB0_DSR0 = MSG_CRC;
                CAN0_TB0_DSR1 = can[ix].cx_myvarblk[jx];
                CAN0_TB0_DSR2 = (unsigned char)(can[ix].cx_myvaroff[jx] >> 3);
                CAN0_TB0_DSR3 = (unsigned char)((can[ix].cx_myvaroff[jx] & 0x0007) << 5) | can[ix].cx_varbyt[jx];

            } else if (sw == MSG_STD) {
                // msg format 11 bit STD - do not let MSII mess with it
                // reset identifier registers for standard ID
                CAN0_TB0_IDR0 = (unsigned char)(can[ix].cx_destvaroff[jx] >> 3);
                // 8 high bits in IDR0, 3 low bits in IDR1
                CAN0_TB0_IDR1 = (unsigned char)((can[ix].cx_destvaroff[jx] & 0x0007) << 5);
                CAN0_TB0_IDR2 = (unsigned char)(0x00);
                CAN0_TB0_IDR3 = (unsigned char)(0x01);
                CAN0_TB0_DLR = (unsigned char) (can[ix].cx_varbyt[jx]);
                for (kx = 0;kx < CAN0_TB0_DLR;kx++)  {
                    *(&CAN0_TB0_DSR0 + kx) = can[ix].cx_datbuf[jx][kx];
                }
            }
            // This is where (in xmt buffer) to get next outgoing message
            if (can[ix].cxno_out < (NO_CANMSG - 1)) {
                can[ix].cxno_out++;
            } else {
                can[ix].cxno_out = 0;
            }
            if (can[ix].cxno > 0) {
                can[ix].cxno--; // TB buf loaded for xmit - decrement ring buf count
            }
            can_status &= CLR_XMT_ERR;
            /* set CAN0 xmt interrupt bit and clear flag to initiate transmit */
            CAN0TIER |= CAN0TBSEL;
            CAN0TFLG = CAN0TBSEL;       // 1 clears(buffer full), 0 is ignored
            break;              // only handle 1 buffer entry at time
        }
        if (ix == 1) {          // nothing left in either ring buffer
            CAN0TIER = 0x00;    // leave CAN0TFLG as buffer empty, but disable interrupt
            // Note: if this last xmt in buf, will re-neter ISR, but 
            //  then exit because can[0,1].cxno = 0. 
            break;
        }
    }                           // end for loop

    /* check for any passthrough write messages */
    if ((flagbyte3 & flagbyte3_sndcandat) && cp_targ) {
        int num;
        unsigned int r;
        unsigned char gp_tmp = GPAGE;

        num = cp_targ - cp_cnt;
        if (num > 8) {
            num = 8;
        } else {
            /* final block, prevent sending any more*/
            flagbyte14 |= FLAGBYTE14_SERIAL_OK; // serial code to return ack code that all bytes were stuck into pipe
            flagbyte3 &= ~flagbyte3_sndcandat;
            cp_targ = 0;
        }
        
        GPAGE = 0x13;
        r = can_snddata(cp_table, cp_id, cp_offset + cp_cnt, num, 0xf009 + cp_cnt); // ring0
        GPAGE = gp_tmp;
        if (r) {
            /* flag up the error so serial.c can act*/
            flagbyte14 |= FLAGBYTE14_CP_ERR;
        }
        cp_cnt += num;
    }

    /* check for any passthrough read messages */
    if ((flagbyte3 & flagbyte3_getcandat) && cp_targ) {
        int num;

        num = cp_targ - cp_cnt;
        if (num > 8) {
            num = 8;
        } else {
            /* final block, prevent sending any more*/
            cp_targ = 0;
        }

        if (can_reqdata(cp_table, cp_id, cp_offset + cp_cnt, num)) {
            /* flag up the error so serial.c can act*/
            flagbyte14 |= FLAGBYTE14_CP_ERR;
        }
        cp_cnt += num;
        cp_time = (unsigned int)lmms;
    }

    if ((CAN0RFLG & 0x0C) != 0) {
        // Xmt error count
        can_status |= XMT_ERR;
        //can_reset = 1;
    }
    return;
}


void do_getcan_delayed()
{
    /* called from RTC after delay timer */
        int num;

        num = cp_targ - cp_cnt;
        if (num > 8) {
            num = 8;
        } else {
            /* final block, prevent sending any more*/
            cp_targ = 0;
        }

        if (can_reqdata(cp_table, cp_id, cp_offset + cp_cnt, num)) {
            /* flag up the error so serial.c can act*/
            flagbyte14 |= FLAGBYTE14_CP_ERR;
        }
        cp_cnt += num;
        cp_time = (unsigned int)lmms;
}

INTERRUPT void CanRxIsr(void)
{
    unsigned char rcv_id, msg_type, var_blk, var_byt;
    unsigned short var_off, dvar_off, jx, kx;

    /* CAN0 Recv Interrupt */
    if (CAN0RFLG & 0x01) {
        var_off = ((unsigned short) CAN0_RB_IDR0 << 3) |
            ((CAN0_RB_IDR1 & 0xE0) >> 5);
        msg_type = (CAN0_RB_IDR1 & 0x07);
        if(msg_type == MSG_XTND)
            msg_type = CAN0_RB_DSR0;
        rcv_id = CAN0_RB_IDR2 >> 4;     // message from device rcv_id
//        var_blk = CAN0_RB_IDR3 >> 4;
        var_blk = (CAN0_RB_IDR3 >> 4) | ((CAN0_RB_IDR3 & 0x08) << 1);
        var_byt = (CAN0_RB_DLR & 0x0F);

        if (var_off >= 0x4000) {
            /* Impossible! */
            /* in ver 2 protocol will send back error reply */
        } else if ((msg_type == MSG_CMD) || (msg_type == MSG_RSP)) {
            //case MSG_CMD:  // msg for this ecu to set a variable to val in msg
            //case MSG_RSP:  // msg in reply to this ecu's request for a var val
            // value in the data buffer in recvd msg

            if ((msg_type == MSG_RSP) && (flagbyte3 & flagbyte3_getcandat) && (var_blk == 6))  {
                // only forward on via serial if doing CAN stuff and requested to dump into txbuf
                // set up for serial xmit of the recvd CAN bytes (<= 8)
                //  MT getting back data requested from aux board

                if (var_byt && (var_byt <= 8)) {
                    unsigned char gp_tmp = GPAGE;
                    GPAGE = 0x13;
                    for (jx = 0; jx < var_byt; jx++) {
                        g_write8(*(&CAN0_RB_DSR0 + jx), canrxad);
                        canrxad++;
                        if (canrxgoal) {
                            canrxgoal--;
                        }
                    }
                    if (canrxgoal == 0) {
                        /* not expecting any more data */
                        flagbyte11 |= FLAGBYTE11_CANRX; /* tell mainloop to do something with the data */
                    }
                    GPAGE = gp_tmp;
                } else {
                    /* how can we get here ? */
                    /* turn off CAN receive mode */
                    flagbyte3 &= ~flagbyte3_getcandat;
                }

                ltch_CAN = 0xFFFFFFFF;
                //break;
            } else {
                unsigned int dest_addr = 0;
                // update variable value with received data
                // Will never pay any attention to writes to the sensor data

                if ((var_blk > 3) && (var_blk < 32)) {

                    // jump to common code used by serial code
                    // concurrent access to tuning data via serial and CAN will cause corruption
                    flagbyte4 |= flagbyte4_cantuning;

                    dest_addr = (unsigned int)tables[var_blk].addrRam;
                
                    // special case for receiving CAN data
                    if (var_blk == 7) {
                        if (var_off >= 0x200) { // datax1 sharing same block no. as outpc
                            var_off -= 0x200; // but at addresses 0x200 and beyond
                            if ((var_off + var_byt) > sizeof(datax1)) {
                                flagbyte3 |= flagbyte3_can_reset;
                            } else {
                                dest_addr = (unsigned int) &datax1;
                            }
                        } else {
                            if ((var_off + var_byt) > sizeof(outpc)) {
                                flagbyte3 |= flagbyte3_can_reset;
                            } else {
                                dest_addr = (unsigned int) &outpc;
                            }
                        }
                    } else if ((var_off + var_byt) > tables[var_blk].n_bytes) {
                        flagbyte3 |= flagbyte3_can_reset;
                        dest_addr = 0;
                    }
                }

                if (dest_addr) {
                    unsigned char save_rpage;
                    unsigned int ad_tmp;
                    ad_tmp = dest_addr + var_off;
                    if ((ad_tmp < 0x1000) || (ad_tmp > 0x3e00) || (var_byt > 8)) {
                        flagbyte3 |= flagbyte3_can_reset;
                    } else {
                        save_rpage = RPAGE;
                        if (tables[var_blk].rpg) {
                            RPAGE = tables[var_blk].rpg;
                        }
                        for (jx = 0; jx < var_byt; jx++) {
                            if (((var_blk == 4) || (var_blk == 5) || (var_blk == 8) || (var_blk == 9) || (var_blk == 10))
                                && (*((unsigned char *) (ad_tmp + jx)) != *(&CAN0_RB_DSR0 + jx))) {
                                outpc.status1 |= STATUS1_NEEDBURN;  // we changed some tuning data
                            }
                            *((unsigned char *) (ad_tmp + jx)) = *(&CAN0_RB_DSR0 + jx);
                        }
                        RPAGE = save_rpage;
                    }
                }
            }

        } else if (msg_type == MSG_REQ) {
            // Update related parameters for xmt ring buffer
            jx = can[0].cxno_in;
            can[0].cx_msg_type[jx] = MSG_RSP;
            // destination var blk
            can[0].cx_destvarblk[jx] = CAN0_RB_DSR0;
            dvar_off = ((unsigned short) CAN0_RB_DSR1 << 3) |
                ((CAN0_RB_DSR2 & 0xE0) >> 5);
            can[0].cx_destvaroff[jx] = dvar_off;
            can[0].cx_dest[jx] = rcv_id;
            var_byt = CAN0_RB_DSR2 & 0x1F;
            can[0].cx_varbyt[jx] = var_byt;
            // put variable value(s) in xmit ring buffer

            unsigned int dest_addr = 0;
            // update variable value with received data
            // see if this variable block is in ram at present
            // if being tuned by remote CAN then we should copy it over,
            // if being tuned by serial and it isn't in ram then we'll have
            // to ignore the CAN device
            // Will never pay any attention to writes to the sensor data

            if ((var_blk > 3) && (var_blk < 32)) {
                flagbyte4 |= flagbyte4_cantuning;
                dest_addr = (unsigned int)tables[var_blk].addrRam;
            }

            if ((dest_addr) ) {
                unsigned int ad_tmp;
                ad_tmp = dest_addr + var_off;
                if ((ad_tmp < 0x1000) || (ad_tmp > 0xeff7) || (var_byt > 8) || ((var_off + var_byt) > tables[var_blk].n_bytes) ) {
                    flagbyte3 |= flagbyte3_can_reset;
                } else {
                    unsigned char save_rpage;
                    save_rpage = RPAGE;
                    if (tables[var_blk].rpg) {
                        RPAGE = tables[var_blk].rpg;
                    }

                    for (kx = 0; kx < var_byt; kx++) {
                        can[0].cx_datbuf[jx][kx] = *((unsigned char *) (ad_tmp + kx));
                    }
                    RPAGE = save_rpage;
    /*                // page 0x10 check. CAN doesn't allow up to 16 at the moment
                    if ((tble_idx == 16) && ((var_off + kx) == 64)) {
                        flagbyte1 |= FLAGBYTE1_EX;
                    }
    */
                    // end page 0x10 check
                }
            }

            // This is where (in xmt buffer) to put next messge
            if (can[0].cxno_in < (NO_CANMSG - 1)) {
                can[0].cxno_in++;
            } else {
                can[0].cxno_in = 0;
            }
            // increment counter
            if (can[0].cxno < NO_CANMSG) {
                can[0].cxno++;
            } else {
                can[0].cxno = NO_CANMSG;
            }
            if (!(CAN0TIER & 0x07)) {
                // Following will cause entry to TxIsr without sending msg
                // since when CAN0TIER = 0, CAN0TFLG left as buff empty.
                // If CAN0TIER has at least 1 int buf enabled, will enter
                // TxIsr automatically. 
                CAN0TBSEL = CAN0TFLG;
                CAN0TIER = CAN0TBSEL;
            }

        } else if (msg_type == MSG_XSUB) {
            switch (rcv_id) {
            case 1:            // message to this ecu from device 1
                can_xsub01();   // execute sub immediately here (set
                // flag if can execute in main loop) 
                break;
            }

        } else if (msg_type == MSG_BURN) {
            Flash_Init();       //check FDIV written to (should be at reset)
            flocker = 0xdd;
            burn_idx = var_blk;
            burnstat = 1;       //set flag byte to enable mainloop erase and burn
        } else if (msg_type == MSG_FWD) {  // msg to forward data to the serial port
				// set up for serial xmit of the recvd CAN bytes (<= 7)
				// Unsolicited data
				if ((var_byt > 0) && (var_byt <= 7))  {
                    canbuf[16] = var_byt - 1; // serial.c will use this
					for (jx = 0;jx < canbuf[16];jx++)  {
						canbuf[jx] = *(&CAN0_RB_DSR1 + jx);
					}
                    flagbyte14 |= FLAGBYTE14_SERIAL_FWD; // mainloop should send when safe
				}
        } else if (msg_type == MSG_CRC) {  // msg to calc CRC32 of page - actually done in misc
            if ((var_blk > 3) && (var_blk < 32)) {
                flagbyte4 |= flagbyte4_cantuning;
                tble_idx = var_blk;
                flagbyte9 |= FLAGBYTE9_CRC_CAN; // set up to do the biz from the mainloop
                // txbuf seems a fair place to store this data. Hoping nothing else tries to write it.
                canbuf[4] = CAN0_RB_DSR1; // dest var blk
                canbuf[5] = rcv_id;
            }

        }                       // end msg_type switch
        can_status &= CLR_RCV_ERR;
    }
    if ((CAN0RFLG & 0x72) != 0) {
        // Rcv error or overrun on receive
        can_status |= RCV_ERR;
        //can_reset = 1;
    }
    /* clear RX buf full, err flags to ready for next rcv int */
    /*  (Note: can't clear err count bits) */
    CAN0RFLG = 0xC3;

    return;
}
