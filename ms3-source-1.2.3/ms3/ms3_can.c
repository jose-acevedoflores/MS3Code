/* $Id: ms3_can.c,v 1.39 2012/11/04 14:12:39 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * CanInit()
    Origin: Al Grippo.
    Minor: XEPisms by James Murray
    Majority: Al Grippo
 * can_incring()
    Origin: Al Grippo.
    Minor: own function James Murray
    Majority: Al Grippo
 * can_sendburn()
    Origin: Al Grippo.
    Minor: XEPisms by James Murray
    Majority: Al Grippo
 * can_reqdata()
    Origin: Al Grippo.
    Minor: XEPisms by James Murray
    Majority: Al Grippo
 * can_snddata()
    Origin: Al Grippo.
    Minor: XEPisms by James Murray
    Majority: Al Grippo
 * can_t()
    Origin: Al Grippo.
    Minor: XEPisms by James Murray
    Majority: Al Grippo
 * can_xsub01
    Origin: Al Grippo.
    Majority: Al Grippo
 * can_poll()
    Origin: Jean Belanger (in MS2/Extra)
    Moderate: XEPisms by James Murray
    Majority: Jean Belanger / James Murray
 * can_crc32()
    Origin: Code by James Murray using MS-CAN framework.
    Majority: James Murray
 * ECANSendMessage()
    Origin: "stevevp" used with permission
    Minor: James Murray
    Majority: stevevp
 * can_broadcast()
    Origin: "stevevp" used with permission
    Minor: James Murray
    Majority: stevevp
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
 */
#include "ms3.h"

void CanInit(void)
{
    unsigned int ix;
    /* Set up CAN communications */
    /* Enable CAN0, set Init mode so can change registers */
    CAN0CTL1 |= 0x80;
    CAN0CTL0 |= 0x01;

    /* clear ring buffers */
    for (ix = 0; ix < 2; ix++) {
        can[ix].cxno = 0;
        can[ix].cxno_in = 0;
        can[ix].cxno_out = 0;
    }
    can_status = 0;
    flagbyte3 &= ~(flagbyte3_can_reset | flagbyte3_sndcandat | flagbyte3_getcandat);

    while (!(CAN0CTL1 & 0x01)); // make sure in init mode (possible lockup?)

    /* Set Can enable, use Oscclk (8 MHz),clear rest */
    CAN0CTL1 = 0x80;
    /* Set timing for .5Mbits/ sec */
// Use same timing as MS2
    CAN0BTR0 = 0xC0;            /* SJW=4,BR Prescaler= 1(8MHz CAN0 clk) */
    CAN0BTR1 = 0x1C;            /* Set time quanta: tseg2 =2,tseg1=13 
                                   (16 Tq total including sync seg (=1)) */


    CAN0IDAC = 0x00;            /* 2 32-bit acceptance filters */
    /* CAN message format:
     * Reg Bits: 7 <-------------------- 0
     * IDR0:    |---var_off(11 bits)----|  (Header bits 28 <-- 21)
     * IDR1:    |cont'd 1 1 --msg type--|  (Header bits 20 <-- 15)
     * IDR2:    |---From ID--|--To ID---|  (Header bits 14 <--  7)
     * IDR3:    |--var_blk-|--spare--rtr|  (Header bits  6 <-- 0,rtr)
     */
    /* Set identifier acceptance and mask registers to accept 
       messages only for can_id or device #15 (=> all devices) */
    /* 1st 32-bit filter bank-to mask filtering, set bit=1 */
    CAN0IDMR0 = 0xFF;           // anything ok in IDR0(var offset)
    CAN0IDAR1 = 0x18;           // 0,0,0,SRR=IDE=1
    CAN0IDMR1 = 0xE7;           // anything ok for var_off cont'd, msgtype
    CAN0IDAR2 = ram4.mycan_id;  // rcv msg must be to can_id, but
    CAN0IDMR2 = 0xF0;           // can be from any other device
    CAN0IDMR3 = 0xFF;           // any var_blk, spare, rtr
    /* 2nd 32-bit filter bank */
    CAN0IDMR4 = 0xFF;           // anything ok in IDR0(var offset)
    CAN0IDAR5 = 0x18;           // 0,0,0,SRR=IDE=1
    CAN0IDMR5 = 0xE7;           // anything ok for var_off cont'd, msgtype
    CAN0IDAR6 = 0x0F;           // rcv msg can be to everyone (id=15), and
    CAN0IDMR6 = 0xF0;           // can be from any other device
    CAN0IDMR7 = 0xFF;           // any var_blk, spare, rtr

    /* clear init mode */
    CAN0CTL0 &= 0xFE;
    /* wait for synch to bus */
    ix = 0;
    while ((!(CAN0CTL0 & 0x10)) && (ix < 0xfffe)) {
        ix++;
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
    if (ix == 0xfffe) { // normal behaviour ~38
        // CAN broken
        conf_err = 190;
    }

    /* no xmit yet */
    CAN0TIER = 0x00;
    /* clear RX flag to ready for CAN0 recv interrupt */
    CAN0RFLG = 0xC3;
    /* set CAN0 rcv full interrupt bit */
    CAN0RIER = 0x01;
    ltch_CAN = 0xffffffff;

	CANid = ram4.mycan_id;    // this may/not be required
//    PTS |= 0x20;                // demo board - enable CAN transceiver
    cp_targ = 0;
    cp_cnt = 0;
    flagbyte14 &= ~FLAGBYTE14_CP_ERR;
    return;
}

unsigned int can_inc_ring(unsigned int ring)
{
    unsigned int ret = 0;
    // This is where (in xmt ring buffer) to put next message
    if (can[ring].cxno_in < (NO_CANMSG - 1)) {
        can[ring].cxno_in++;
    } else {
        can[ring].cxno_in = 0;     // overwrite oldest msg in queue
    }
    // increment counter
    if (can[ring].cxno < NO_CANMSG) {
        can[ring].cxno++;
    } else {
        can[ring].cxno = NO_CANMSG;
        ret = 1;
    }
    if (!(CAN0TIER & 0x07)) {
        // Following will cause entry to TxIsr without sending msg
        // since when CAN0TIER = 0, CAN0TFLG left as buff empty(>0).
        // If CAN0TIER has at least 1 int buf enabled, will enter
        // TxIsr automatically.
        CAN0TBSEL = CAN0TFLG;
        CAN0TIER = CAN0TBSEL;
    }
    return ret;
}

unsigned int can_sendburn(unsigned int table, unsigned int id)
{
    int ix;
    // set up single CAN message for "burn" & forward to aux board
    // (called from mainloop serial)
    ix = can[0].cxno_in;
    can[0].cx_msg_type[ix] = MSG_BURN;
    can[0].cx_destvarblk[ix] = table;
    can[0].cx_destvaroff[ix] = 0;
    can[0].cx_dest[ix] = id;
    can[0].cx_varbyt[ix] = 0;   // no data bytes
    return can_inc_ring(0);
}


unsigned int can_crc32(unsigned int table, unsigned int id)
{
    int ix;
    unsigned int ret;
    // set up single CAN message & forward to aux board
    //   PC requesting CRC from aux board.
    // (called from mainloop serial with ints off)
    ix = can[0].cxno_in;
    can[0].cx_msg_type[ix] = MSG_CRC; 
    can[0].cx_destvarblk[ix] = table;
    can[0].cx_myvarblk[ix] = 6;  // txbuf
    // varblks for the aux boards
    can[0].cx_destvaroff[ix] = 0;
    can[0].cx_myvaroff[ix] = 0;
    can[0].cx_dest[ix] = id;
    can[0].cx_varbyt[ix] = 4; // 4 bytes of CRC
    ret = can_inc_ring(0);
    flagbyte3 |= flagbyte3_getcandat;
    cp_time = (unsigned int)lmms;
    return ret;
}

unsigned int can_reqdata(unsigned int table, unsigned int id, unsigned int offset, unsigned int num)
{
    int ix;
    unsigned int ret;
    // set up single CAN message & forward to aux board
    //   MT requesting data from aux board.
    //   Note: MT must only deal with 1 board at a time and wait til
    //   data back or timeout.
    // (called from mainloop serial)
    ix = can[0].cxno_in;
    can[0].cx_msg_type[ix] = MSG_REQ;
    can[0].cx_destvarblk[ix] = table;
    can[0].cx_myvarblk[ix] = 6; // txbuf
    // varblks for the aux boards
    can[0].cx_destvaroff[ix] = offset;
    can[0].cx_myvaroff[ix] = 0;
    can[0].cx_dest[ix] = id;
    can[0].cx_varbyt[ix] = (unsigned char) num;
    ret = can_inc_ring(0);
    flagbyte3 |= flagbyte3_getcandat;
    cp_time = (unsigned int)lmms;
    return ret;
}

unsigned int can_snddata(unsigned int table, unsigned int id, unsigned int offset, unsigned int num, unsigned int bufad)
{
    unsigned int ix;
    // set up CAN message & forward to aux board, can be part of chain of packets
    //   PC sending data to aux board
    // called from serial.c and can_isr.c - both use _global_ bufad - requires GPAGE setting externally
    for (ix = 0; ix < num; ix++) {
        can[0].cx_datbuf[can[0].cxno_in][ix] = g_read8(bufad + ix);
    }
    ix = can[0].cxno_in;
    can[0].cx_msg_type[ix] = MSG_CMD;
    can[0].cx_destvarblk[ix] = table;
    can[0].cx_destvaroff[ix] = offset;
    can[0].cx_dest[ix] = id;
    can[0].cx_varbyt[ix] = num;
    cp_time = (unsigned int)lmms;
    return can_inc_ring(0);
}

/* This function is not presently used. Tcntr is not initialised. */
unsigned int can_t(unsigned int table, unsigned int id, unsigned int num, unsigned int bufad)
{
    unsigned int ix;
    // set up single CAN message & forward to aux board
    //   MT sending data to aux board
    //  'T' command for remote sensor data
    // (called from mainloop serial)
    for (ix = 0; ix < Tcntr; ix++) {
        can[0].cx_datbuf[can[0].cxno_in][ix] = *((unsigned char *)(bufad + ix));
    }
    ix = can[0].cxno_in;
    can[0].cx_msg_type[ix] = MSG_CMD;
    can[0].cx_destvarblk[ix] = table;
    can[0].cx_destvaroff[ix] = 0;
    can[0].cx_dest[ix] = id;
    can[0].cx_varbyt[ix] = num;
    return can_inc_ring(0);
}

void can_xsub01(void)
{
    return;
}

/***************************************************************************
 **
 **  CAN polling from I/O slave device
 **
 **************************************************************************/
/* CAN message set up as follows
 * cx_msg_type = type of message being sent. MSG_REQ is a request for data
 * cx_dest = remote CANid that we are sending message to
 * cx_destvarblk = which variable block (page) to get data from. page 6 = realtime data, outpc
 * cx_destvaroff = offset within that datablock
 * cx_datbuf = a buffer for the data we are sending, none for a request, eight maximum.
 * cx_varbyt = number of bytes sent or requested
 * cx_myvarblk = the variable block to put the reply in (the other end sends a message back with this block no)
 * cx_myvaroff = the variable offset to put the reply in (the other end sends a message back with this block no)
 *
 * With MSG_REQ we send all of that info off and then the other end replies with a MSG_RSP including the variable block and offset
 * to tell us where to deposit the data, kind of like a "push" of the data. We then (blindly) store it where the MSG_RSP asked us to.
 * In the code below we are asking the GPIO board for the raw ADC data and then store it in datax1.gpioadc[] which are then available for
 * use in the code. These can be transformed in the code and then available as outpc.sensors[] as part of the realtime data pack.
 */

void can_poll(void)
{
    int ix;

    if (flagbyte3 & (flagbyte3_sndcandat | flagbyte3_getcandat))  {
        /* while doing a passthrough read/write, do not add any other requests to the queue */
        /* need a timeout to reset if it doesn't complete somehow? */
        return;
    }
    // CAN polling code.
    // If enabled it sends a request to a GPIO type board which then sends data back to us.
    // This gets ADC values from a GPIO with user defined CAN_ID= ram4.can_poll_id
    DISABLE_INTERRUPTS;
    ultmp = lmms;
    ENABLE_INTERRUPTS;
    if (ultmp > cansendclk) {      // enable clauses checked below
        cansendclk = ultmp + 78;        // 10ms ( 1s = 7812 x .128 ms) clk
        // load ring buffer - send request for data
        // can[1] is used for mainloop requests

        if (ram4.enable_poll & 0x06) {
            // copy previous to outpc
            if (ram4.enable_poll & 0x04) { // 32 bit
                unsigned long pwmtmp;
                unsigned char l;
                for (l = 0 ; l < 4 ; l++) {
                    unsigned long *ad;
                    ad = (unsigned long*)&datax1.pwmin32[l];
                    DISABLE_INTERRUPTS;
                    pwmtmp = *ad;
                    ENABLE_INTERRUPTS;
                    if (pwmtmp & 0xffff0000) {
                        outpc.gpiopwmin[l] = 0xffff;
                    } else {
                        outpc.gpiopwmin[l] = (unsigned int)pwmtmp;
                    }
                }
            } else { // 16bit
                outpc.gpiopwmin[0] = datax1.pwmin16[0];
                outpc.gpiopwmin[1] = datax1.pwmin16[1];
                outpc.gpiopwmin[2] = datax1.pwmin16[2];
                outpc.gpiopwmin[3] = datax1.pwmin16[3];
            }

	        // Get remote 16 bit PWM data from Generic board or first 2 x 32 bits
	        can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
	        can[1].cx_dest[can[1].cxno_in] = ram4.can_poll_id;            // send to device

	        can[1].cx_destvarblk[can[1].cxno_in] = ram4.poll_tables[1];  // fetch from user defined table
	        can[1].cx_destvaroff[can[1].cxno_in] = ram4.poll_offset[1];  // fetch raw pwm from user defined offset
	        can[1].cx_varbyt[can[1].cxno_in] = 8;    // 8 bytes to be returned

	        // where should the resulting data be stored
	        can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc
	        can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.pwmin16[0]) - (unsigned short)(&datax1); // this is offset of where to store it

            can_inc_ring(1);
        }

        if (ram4.enable_poll & 0x04) {
	        // Get second 2 x 32 bit PWM data from Generic board
	        can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
	        can[1].cx_dest[can[1].cxno_in] = ram4.can_poll_id;            // send to device

	        can[1].cx_destvarblk[can[1].cxno_in] = ram4.poll_tables[1];  // fetch from user defined table
	        can[1].cx_destvaroff[can[1].cxno_in] = 8 + ram4.poll_offset[1];  // fetch raw pwm from user defined offset
	        can[1].cx_varbyt[can[1].cxno_in] = 8;    // 8 bytes to be returned

	        // where should the resulting data be stored
	        can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc
	        can[1].cx_myvaroff[can[1].cxno_in] = 512 + 8 + (unsigned short)(&datax1.pwmin16[0]) - (unsigned short)(&datax1); // this is offset of where to store it

            can_inc_ring(1);
        }

        // CAN digital port input
        if (ram4.enable_poll & 0x08) {
	        // Get remote ports data from Generic board
	        can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
	        can[1].cx_dest[can[1].cxno_in] = ram4.can_poll_id_ports;

	        can[1].cx_destvarblk[can[1].cxno_in] = ram4.poll_tables[2];  // fetch from user defined table
	        // 1 input
	        can[1].cx_destvaroff[can[1].cxno_in] = ram4.poll_offset[2];  // fetch raw port from user defined offset
	        can[1].cx_varbyt[can[1].cxno_in] = 1;    // 1 bytes to be returned

	        // where should the resulting data be stored
	        can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc
	        can[1].cx_myvaroff[can[1].cxno_in] = (unsigned short)(&outpc.canin1_8) - (unsigned short)(&outpc); // this is offset of where to store it

            can_inc_ring(1);
        }

        // CAN digital port outputs
        if (ram4.enable_poll & 0x30) {
	        // Send remote ports data to Generic board
	        can[1].cx_msg_type[can[1].cxno_in] = MSG_CMD; 
	        can[1].cx_dest[can[1].cxno_in] = ram4.can_poll_id_ports;

	        can[1].cx_destvarblk[can[1].cxno_in] = ram4.poll_tables[2];  // send to user defined table
	        can[1].cx_destvaroff[can[1].cxno_in] = ram4.poll_offset[3];  // raw port from user defined offset
            if (ram4.enable_poll & 0x20) {
	            can[1].cx_varbyt[can[1].cxno_in] = 2;
                can[1].cx_datbuf[can[1].cxno_in][1] = outpc.canout9_16;
            } else {
    	        can[1].cx_varbyt[can[1].cxno_in] = 1;
            }
            can[1].cx_datbuf[can[1].cxno_in][0] = outpc.canout1_8;

	        // clear these fields not used for MSG_RSP
	        can[1].cx_myvarblk[can[1].cxno_in] = 0;
	        can[1].cx_myvaroff[can[1].cxno_in] = 0;

            can_inc_ring(1);
        }

         // CAN PWM outputs
        if (ram4.enable_poll & 0x40) {
	        // Set remote PWM data on Generic board
	        can[1].cx_msg_type[can[1].cxno_in] = MSG_CMD; 
	        can[1].cx_dest[can[1].cxno_in] = ram4.can_pwmout_id;

	        can[1].cx_destvarblk[can[1].cxno_in] = ram4.can_pwmout_tab;
	        can[1].cx_destvaroff[can[1].cxno_in] = ram4.can_pwmout_offset;
	        can[1].cx_varbyt[can[1].cxno_in] = 8;

	        for(ix = 0;ix < 8;ix++)  {
		        can[1].cx_datbuf[can[1].cxno_in][ix] = datax1.canpwmout[ix];
	        }

            can_inc_ring(1);
        }

        //CAN ADCs
        if (ram4.enable_poll & 0x01) {
            for (ix = 0 ; ix < 6 ; ix++) {
                if (ram4.canadc_opt & (1 << ix)) { // are these group of 4 enabled?
	                can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
	                can[1].cx_dest[can[1].cxno_in] = ram4.canadc_id[ix];            // send to device

	                can[1].cx_destvarblk[can[1].cxno_in] = ram4.canadc_tab[ix];  // fetch from user defined table
	                can[1].cx_destvaroff[can[1].cxno_in] = ram4.canadc_off[ix];  // fetch raw pwm from user defined offset
	                can[1].cx_varbyt[can[1].cxno_in] = 8;    // 8 bytes to be returned

	                // where should the resulting data be stored
	                can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in 'outpc'
	                can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.adc[ix << 2]) - (unsigned short)(&datax1); // this is offset of where to store it

                    can_inc_ring(1);
                }
            }
        }

        // Innovate EGO via CAN
        // currently only supported by Extender
        if (ram4.can_poll2 & 0x01) {
            for (ix = 0 ; ix < 2 ; ix++) {
	            can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
	            can[1].cx_dest[can[1].cxno_in] = ram4.can_ego_id;

	            can[1].cx_destvarblk[can[1].cxno_in] = ram4.can_ego_table;
	            // 1 input
	            can[1].cx_destvaroff[can[1].cxno_in] = ram4.can_ego_offset + (ix * 8);
	            can[1].cx_varbyt[can[1].cxno_in] = 8;

	            // where should the resulting data be stored
	            can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc (datax1)
	            can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.ego[ix*4]) - (unsigned short)(&datax1);

                can_inc_ring(1);
            }
        }

        if ((ram4.vss_opt & 0x0f) == 0x0e) {
            can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
            can[1].cx_dest[can[1].cxno_in] = ram4.vss1_can_id;

            can[1].cx_destvarblk[can[1].cxno_in] = ram4.vss1_can_table;
            // 1 input
            can[1].cx_destvaroff[can[1].cxno_in] = ram4.vss1_can_offset;
            // where should the resulting data be stored
            can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc (datax1)

            if (ram4.vss_can_size & 0x01) { // 16 bit
                can[1].cx_varbyt[can[1].cxno_in] = 2;
                can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.vss1_16) - (unsigned short)(&datax1);
            } else {
                can[1].cx_varbyt[can[1].cxno_in] = 1;
                can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.vss1_8) - (unsigned short)(&datax1);
            }

            can_inc_ring(1);
        }

        if ((ram4.vss_opt & 0xf0) == 0xe0) {
            can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
            can[1].cx_dest[can[1].cxno_in] = ram4.vss1_can_id;

            can[1].cx_destvarblk[can[1].cxno_in] = ram4.vss1_can_table;
            // 1 input
            can[1].cx_destvaroff[can[1].cxno_in] = ram4.vss2_can_offset;
            // where should the resulting data be stored
            can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc (datax1)

            if (ram4.vss_can_size & 0x02) { // 16 bit
                can[1].cx_varbyt[can[1].cxno_in] = 2;
                can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.vss2_16) - (unsigned short)(&datax1);
            } else {
                can[1].cx_varbyt[can[1].cxno_in] = 1;
                can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.vss2_8) - (unsigned short)(&datax1);
            }

            can_inc_ring(1);
        }

        if ((ram4.gear_method & 0x03) == 0x03) {
            can[1].cx_msg_type[can[1].cxno_in] = MSG_REQ; 
            can[1].cx_dest[can[1].cxno_in] = ram4.vss1_can_id;

            can[1].cx_destvarblk[can[1].cxno_in] = ram4.vss1_can_table;
            // 1 input
            can[1].cx_destvaroff[can[1].cxno_in] = ram4.gear_can_offset;
            // where should the resulting data be stored
            can[1].cx_myvarblk[can[1].cxno_in] = 7;  // store returned data in outpc (datax1)

            can[1].cx_varbyt[can[1].cxno_in] = 1;
            can[1].cx_myvaroff[can[1].cxno_in] = 512 + (unsigned short)(&datax1.gear) - (unsigned short)(&datax1);

            can_inc_ring(1);
        }
    }

    // CAN RTC. Assumes same 8 byte format as extender
/*
    unsigned char rtc_sec, rtc_min, rtc_hour, rtc_day, rtc_date, rtc_month;
    unsigned int rtc_year;
*/
    if ((ram4.opt142 & 0x03) == 2) {
        if (flagbyte9 & FLAGBYTE9_GETRTC) {
            flagbyte9 &= ~FLAGBYTE9_GETRTC;

            ix = can[1].cxno_in;
            // Get time/date
            can[1].cx_msg_type[ix] = MSG_REQ; 
            can[1].cx_dest[ix] = ram4.can_poll_id_rtc;

            can[1].cx_destvarblk[ix] = ram4.poll_tables[0];  // fetch from user defined table
            can[1].cx_destvaroff[ix] = ram4.poll_offset[0];  // fetch raw port from user defined offset
            can[1].cx_varbyt[ix] = 8;

            // where should the resulting data be stored
            can[1].cx_myvarblk[ix] = 7;  // store returned data in outpc
            can[1].cx_myvaroff[ix] = 512+ (unsigned short)(&datax1.rtc_sec) - (unsigned short)(&datax1); // this is offset of where to store it

        } else if (datax1.setrtc_lock == 0x5a) {
            unsigned char tr;
            datax1.setrtc_lock = 0;

            ix = can[1].cxno_in;
            can[1].cx_msg_type[ix] = MSG_CMD;
            /* HARDCODED: per JB, send to table 0x10, offset 0 and encode in BCD */
            can[1].cx_destvarblk[ix] = 0x10;
            can[1].cx_destvaroff[ix] = 0;
            can[1].cx_dest[ix] = ram4.can_poll_id_rtc;
            can[1].cx_varbyt[ix] = 8;
            // unsigned char setrtc_sec, setrtc_min, setrtc_hour, setrtc_day, setrtc_date, setrtc_month
            // unsigned int setrtc_year;
            can[1].cx_datbuf[ix][0] = (datax1.setrtc_sec % 10) | ((datax1.setrtc_sec / 10) << 4);
            can[1].cx_datbuf[ix][1] = (datax1.setrtc_min % 10) | ((datax1.setrtc_min / 10) << 4);
            can[1].cx_datbuf[ix][2] = (datax1.setrtc_hour % 10) | ((datax1.setrtc_hour / 10) << 4);
            can[1].cx_datbuf[ix][3] = datax1.setrtc_day;
            can[1].cx_datbuf[ix][4] = (datax1.setrtc_date % 10) | ((datax1.setrtc_date / 10) << 4);
            can[1].cx_datbuf[ix][5] = (datax1.setrtc_month % 10) | ((datax1.setrtc_month / 10) << 4);
            can[1].cx_datbuf[ix][6] = (unsigned char)(datax1.setrtc_year % 100);
            if (ram4.rtc_trim < 0) {
                if (ram4.rtc_trim <= -62) {
                    tr = 63;  // 31 plus sign bit
                } else {
                    tr = 0x20 | ((-ram4.rtc_trim)>>1);  // Set trim to positive value, divide by 2, add sign bit
                }
            } else {
                 tr = ram4.rtc_trim >> 2; // Divide trim by 4
            }
            can[1].cx_datbuf[ix][7] = tr;
        }

        can_inc_ring(1);
    }
}

/* The following function was written by 'stevep' and donated to MS2/Extra / MS3 */
void ECANSendMessage(unsigned int id, unsigned char *data, unsigned char dataLen, unsigned char msgType) {
    int ix, kx;
    /* CAN message format:
     * Reg Bits: 7<------------------- 0
     * IDR0:    |----STD ID(11 bits)----|  (Header bits 28 <-- 21)
     * IDR1:    |cont'd 0 0 ------------|  (Header bits 20 <-- 15)
     * IDR2:    |-----------------------|  (Header bits 14 <--  7)
     * IDR3:    |----------------------0|  (Header bits  6 <-- 0,rtr)
     */ 

    ix = can[0].cxno_in;

    // Set to Standard 11Bit ID Format
    can[0].cx_msg_type[ix] = msgType; // MSG_STD = 11

    // set CANID and DLC of Message
    can[0].cx_destvaroff[ix] = id; // CANID
    can[0].cx_varbyt[ix] = dataLen; // 8 bytes DLC

    // load Message Data
                    for (kx = 0;kx < dataLen ;kx++)  {
                        can[0].cx_datbuf[ix][kx] = data[kx];
                    }

    // set following to zero as not used in Standard CAN Header
    can[0].cx_destvarblk[ix] = 0;
    can[0].cx_dest[ix] = 0;

    // This is where (in xmt ring buffer) to put next message
    if (can[0].cxno_in < (NO_CANMSG - 1)) {
        can[0].cxno_in++;
    } else {
        can[0].cxno_in = 0; // overwrite oldest msg in queue
    }

    can_inc_ring(0);
}

/* The following function is derived from code written by 'stevep' and donated to MS2/Extra / MS3 */
void can_broadcast(void)
{
    /* Broadcast selected CAN messages using STD CAN*/

    // Send Message Variables
    unsigned int id, val;
    unsigned char data[8];
    unsigned char dataLen;
  
    DISABLE_INTERRUPTS;
    val = (unsigned int)lmms;
    ENABLE_INTERRUPTS;
    if ((val - can_bcast_last) > ram5.can_bcast_int) {
        can_bcast_last = val; // rollover safe
    } else {
        return;
    }
   
    if (ram5.can_bcast1 & 0x02) {
        id = Engine_RPM; //0x280 - Motorsteuergerat 640 - engine control unit RPM is 4:1 ratio = outpc.rpm*4
        val = outpc.rpm * 4;
        dataLen = 0x08;
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = (unsigned char)(val & 0xff); //byte 3 = RPM, L
        data[3] = (unsigned char)(val >> 8);   //byte 4 = RPM, H
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
        ECANSendMessage(id, data, dataLen, MSG_STD);
    }

    if (ram5.can_bcast1 & 0x04) {
        /* id definition says 1:1 ?? */
        id = Engine_RPM; //0x280 - Motorsteuergerat 640 - engine control unit RPM is 1:1 ratio = outpc.rpm*1
        val = outpc.rpm * 1;
        dataLen = 0x08;
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = (unsigned char)(val & 0xff); //byte 3 = RPM, L
        data[3] = (unsigned char)(val >> 8);   //byte 4 = RPM, H
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
        ECANSendMessage(id, data, dataLen, MSG_STD);
    }

    if (ram5.can_bcast1 & 0x08) {
        long t1, t2;
        id = Engine_Temp;//     0x289 // 996 - Motorsteuerger�t 649 - Engine Temp /y=46.14+1.67*x where x is degrees C
        // quadratic to match guage position curve y = -.0336*x*x + 7.7986*x + -225.2571
        val = (((outpc.clt - 320) * 5) / 9)/10;
        if (val > 74) {
            t1 = (-336L * val) * val;
            t2 = 77986L * val;
            val = (unsigned int)((t1 + t2 -2252571) / 10000);
        }
        else {
            val = (5447 + (154 * val)) / 100;
        }
        dataLen = 0x08;
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = (unsigned char)val;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
        ECANSendMessage(id, data, dataLen, MSG_STD);
    }
}
