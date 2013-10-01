/* $Id: ms3_main.c,v 1.298.2.4 2013/04/24 20:42:13 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * main()
    Origin: Al Grippo
    Major: Re-write, split up. James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/
/*************************************************************************
 **************************************************************************
 **   M E G A S Q U I R T  II - 2 0 0 4 - V1.000
 **
 **   (C) 2003 - B. A. Bowling And A. C. Grippo
 **
 **   This header must appear on all derivatives of this code.
 **
 ***************************************************************************
 **************************************************************************/

/*************************************************************************
 **************************************************************************
 **   GCC Port
 **
 **   (C) 2004,2005 - Philip L Johnson
 **
 **   This header must appear on all derivatives of this code.
 **
 ***************************************************************************
 **************************************************************************/

/*************************************************************************
 **************************************************************************
 **   MS2/Extra
 **
 **   (C) 2006,2007,2008 - Ken Culver, James Murray (in either order)
 **
 **   This header must appear on all derivatives of this code.
 **
 ***************************************************************************
 **************************************************************************/

#include "ms3.h"
#include "cltfactor.inc"
#include "matfactor.inc"
#include "egofactor.inc"
#include "maffactor.inc"
#include "gmfactor.inc"

// the factor tables are only accessed via global calls which cause a compiler warning
// See memory.x for where these are. See ms3.h and ms3h.inc

#include "ms3_main_vectors.h"
#include "ms3_main_defaults.h"
#include "ms3_main_decls.h"

int main(void)
{
    unsigned int utmp1;
    long lsum = 0, lsum1 = 0, lsum2 = 0;
    char localflags = 0;

    tmp_pw1 = tmp_pw2 = 0;
    injbits = 0;

    main_init();                // set up all timers, initialise stuff

    mltimestamp = TCNT;
    //  main loop
    for (;;) {
        if (conf_err != 0) {
            configerror();          // only returns for errors >= 190
        }
        outpc.looptime = (outpc.looptime + TCNT - mltimestamp) >> 1;    // rolling avg

        mltimestamp = TCNT;

        stack_watch();

        //reset COP timer (That's Computer Operating Properly, not Coil On Plug...)
        ARMCOP = 0x55;
        ARMCOP = 0xAA;

        //check XGATE
        if ((unsigned char)XGMCTL != 0x81) {
            // XGATE went and did a silly thing - shouldn't happen any more
            XGMCTL = 0xA383; // same as XGMCTL_ENABLE in config_xgate()
        }

        // For debug - check if this setting got randomly changed
        if (ram4.no_skip_pulses > 10) {
            conf_err = 99;
        }

        if ((ram4.opt142 & 0x03) == 1) {
            poll_i2c_rtc();
        } else if ((ram4.opt142 & 0x03) == 0) {
            /* no real RTCC, but accept any set-time requests */
            if (datax1.setrtc_lock == 0x5a) {
                datax1.setrtc_lock = 0;

                datax1.rtc_sec = datax1.setrtc_sec;
                datax1.rtc_min = datax1.setrtc_min;
                datax1.rtc_hour = datax1.setrtc_hour;
                datax1.rtc_day = datax1.setrtc_day;
                datax1.rtc_date = datax1.setrtc_date;
                datax1.rtc_month = datax1.setrtc_month;
                datax1.rtc_year = datax1.setrtc_year;
            } else if (flagbyte9 & FLAGBYTE9_GETRTC) {
                flagbyte9 &= ~FLAGBYTE9_GETRTC;
                /* maintain some time locally */
                datax1.rtc_sec++;

                if (datax1.rtc_sec == 60) {
                    datax1.rtc_sec = 0;
                    datax1.rtc_min++;
                }

                if (datax1.rtc_min == 60) {
                    datax1.rtc_min = 0;
                    datax1.rtc_hour++;
                }

                if (datax1.rtc_hour == 24) {
                    datax1.rtc_hour = 0;
                    datax1.rtc_day++;
                }
                /* let days count up */
            }
        }

        serial();
        do_sdcard();

        if (flagbyte8 & FLAGBYTE8_MODE10) {
            if (((unsigned int)lmms - mode10_time) > 8000) { // 1 second
                flagbyte8 &= ~FLAGBYTE8_MODE10;
            }
        }

        if (flagbyte1 & FLAGBYTE1_EX) { // XGATE re-init
            flagbyte1 &= ~FLAGBYTE1_EX;
            DISABLE_INTERRUPTS;
            __asm__ __volatile__("pshy\n"
                                 "jsr 0xFEF0\n" "puly\n"::"y"(&buf2));
            XGMCTL = 0xA383;
            ENABLE_INTERRUPTS;
        }

        chk_crc();
        dribble_burn();

        if (flagbyte1 & flagbyte1_tstmode) {
            injpwms();

            goto SKIP_THE_LOT;  // in special test mode don't do any of normal mainloop
        }

        sample_map_tps(&localflags);
        serial();

        /* Noise Filter Table Lookup */
        if (ram4.NoiseFilterOpts & 1) {
            unsigned int noiserpm;
            if (outpc.rpm < 10) {
                noiserpm = 32767;
            } else {
                noiserpm = outpc.rpm;
            }
            utmp1 = intrp_1ditable(noiserpm, 4,
                               (unsigned int *) ram_window.pg8.NoiseFilterRpm, 0,
                               (unsigned int *) ram_window.pg8.NoiseFilterLen, 8);

            DISABLE_INTERRUPTS;
            NoiseFilterMin = (unsigned long) utmp1;
            ENABLE_INTERRUPTS;
        }

        if (((unsigned int)lmms - adc_lmms) > 78) {   // every 10 ms (78 x .128 ms clk)
            adc_lmms = (unsigned int)lmms;
            // read 10-bit ADC results, convert to engineering units and filter
            next_adc++;
            if (next_adc > 7)
                next_adc = 1;
                do_sensors();
            if ((next_adc == 1) || (next_adc == 2) || (next_adc == 4)) {
                get_adc(next_adc, next_adc);    // get one channel on each pass
            } else if (next_adc >= 6) {
                get_adc(next_adc, next_adc);    // get one channel on each pass
            }
            gearpos(); // gear selection
            calcvssdot();
        }

        if ((localflags & LOCALFLAGS_RUNFUEL) || (outpc.engine & ENGINE_CRANK)) {
            calc_baro_mat_load();
            serial();

            // check for stall
            if (((outpc.engine & ENGINE_READY) == 0) || (outpc.rpm == 0))
                goto END_FUEL;

            utmp1 = crank_calcs();

            if (utmp1 == 1) {
                goto KNK;
            } else if (utmp1 == 2) {
                goto CHECK_STAGED;
            }

            warmup_calcs();
            serial();
            do_sdcard();
            normal_accel();
            serial();

            ego_get_targs_gl();
            ego_get_targs();
            ego_calc();
            serial();
            main_fuel_calcs(&lsum1, &lsum2); /* lsum1 and lsum2 set in here */

            flex_fuel_calcs(&lsum1, &lsum2);

            dribble_burn();
            serial();

            if (ram4.sequential & SEQ_TRIM) { // MS3X and/or V3
                calc_fuel_trims();
            }

            new_accel(&lsum1, &lsum2);
            /**************************************************************************
             **
             ** Calculation of Fuel Pulse Width
             **
             **************************************************************************/
            if (!(ram5.AE_options & USE_NEW_AE)) {
                lsum = (long) outpc.tpsaccel * 10000;       // .01 usec
                if (lsum1 > 0) {
                    lsum1 += lsum;  // usec
                }
                if (lsum2 > 0) {
                    lsum2 += lsum;  // usec
                }
            }

            tmp_pw1 = lsum1;
            tmp_pw2 = lsum2;

        /* for fuel table blending could do the blend here if ve1+2 -> lsum1 and ve3+4 -> lsum2 in inj.c */

            do_overrun();

            if (outpc.status3 & STATUS3_CUT_FUEL) {
                tmp_pw1 = 0;
                tmp_pw2 = 0;
            }

            n2o_launch_additional_fuel();

CHECK_STAGED:
            if (ram4.staged & 0x7) {
                calc_staged_pw(tmp_pw1);

                tmp_pw1 = pw_staged1;
                tmp_pw2 = pw_staged2;
            }
END_FUEL:;
        }
        /* fuel is finished off down below EVERY_TOOTH */

        serial();

        if ((pwmidle_reset == 4) && !als_state) {       /* just came out of init */
            /* Wait for the valve to stop moving, then go to the crank position */
            if (!IAC_moving) {
                unsigned int pos;

                pos =
                    intrp_1ditable(outpc.clt, 4,
                                   (int *) ram_window.pg8.
                                   pwmidle_crank_clt_temps, 0,
                                   (unsigned int *) ram_window.pg8.
                                   pwmidle_crank_dutyorsteps, 8);

                DISABLE_INTERRUPTS;
                IACmotor_pos = pos;
                move_IACmotor();
                ENABLE_INTERRUPTS;
                pwmidle_reset = 5;
            }
        }
        if ((IdleCtl == 4) || (IdleCtl == 6)) {
            (void) move_IACmotor(); // ensure that duty is set now that code removed from isr_rtc
        }

        if (IACmotor_reset && iactest_glob) {
            idle_test_mode();
        }

        speed_sensors();

        water_inj();
        fan_ctl_idleup();

        // check for spark cut rev limiting
        flagbyte10 &= ~FLAGBYTE10_SPKCUTTMP; // status var set later
        flagbyte10 &= ~FLAGBYTE10_FUELCUTTMP;

        // check for stall
        if (((outpc.engine & ENGINE_READY) == 0) || (outpc.rpm == 0))
            goto EVERY_TOOTH;

        idle_ac_idleup();
        idle_target_lookup();
        idle_ctl();

        /* BOOST CONTROL */
        if (ram4.boost_ctl_settings & BOOST_CTL_ON) {
            boost_ctl();
        }

        serial();
        do_sdcard();
        /**************************************************************************
         **
         ** Calculation of Knock retard, Distributor Advance & coil charge time correction
         **
         **************************************************************************/
      KNK:
        do_knock();
        calc_advance(&lsum); /* lsum is set in here */
        do_launch(&lsum);

        dribble_burn();
        serial();
        nitrous();
        traction(&lsum);
        lsum -= outpc.n2o_retard;
        lsum -= outpc.launch_retard;

      	lsum += datax1.SpkAdj;     // degx10 - adjustment from remote CAN device (dangerous)
        if (outpc.status7 & STATUS7_LIMP) {
            lsum -= ram5.cel_retard;
        }

        if (ram4.spk_mode3 & SPK_MODE3_SPKTRIM) {
            calc_spk_trims();
        }
        /**************************************************************************/
        if (spkmode == 31) {
            lsum = 0; // fuel only
        }

        if ((spkmode == 2) || (spkmode == 3) || (spkmode == 14)) { // dizzy or twin trigger
            if (ram4.adv_offset < 200) { // next cyl
                // allow at least 1 deg ahead of trigger
                if (lsum < (ram4.adv_offset + 10)) {
                    lsum = ram4.adv_offset + 10;
                }
            } else { // this cyl
                // allow at least 5 deg after trigger
                if (lsum > (ram4.adv_offset - 50)) {
                    lsum = ram4.adv_offset - 50;
                }
            }
        }

        if (lsum < MIN_IGN_ANGLE) {
            lsum = MIN_IGN_ANGLE;
        }

        outpc.adv_deg = (int) lsum;     // Report advance before we fudge it

        /* check for difference between flash and ram trigger angles (trigger wizard?) for on-the-fly adjustment */
        if (spkmode != 4) {
            if (ram4.adv_offset != flash_adv_offset) {
                int local_offset;
                local_offset = ram4.adv_offset;
                if (spkmode > 3) {
                    /* only permit +/-20 here too */
                    if (local_offset < -200) {
                        local_offset = -200;
                    } else if (local_offset > 200) {
                        local_offset = 200;
                    }
                }
                lsum += flash_adv_offset - local_offset;
            }
        }
        if (spkmode < 2) { // EDIS isn't adjusted in ign_wheel_init like other modes, do it here
            lsum -= flash_adv_offset;
        }

        /* ONLY FOR TRIGGER WHEEL - check for difference between flash and ram tooth #1 angles for on-the-fly adjustment */
        if ((spkmode == 4) && (ram4.Miss_ang != flash_Miss_ang)) {
            lsum += flash_Miss_ang - ram4.Miss_ang;
        }

        /* ROTARY: No function here, very small so leave it here */
        if (((ram4.EngStroke & 0x03) == 0x03)) {
            if (outpc.engine & ENGINE_CRANK) {
                lsum1 = ram4.crank_timing;
            } else if (ram4.timing_flags & TIMING_FIXED) {
                lsum1 = ram4.fixed_timing;
            } else {
                lsum1 =
                    intrp_2ditable(outpc.rpm, outpc.fuelload, 8, 8,
                                   &ram_window.pg8.RotarySplitRPM[0],
                                   &ram_window.pg8.RotarySplitMAP[0],
                                   (int *) &ram_window.pg8.
                                   RotarySplitTable[0][0], 8);
                if (!(ram4.RotarySplitMode & ROTARY_SPLIT_ALLOW_NEG_SPLIT)) {
                    if (lsum1 < 0) {
                        lsum1 = 0;
                    }
                }

                /* now since split is with reference to the leading timing, subtract split to leading timing */
                lsum1 = lsum - lsum1;
            }
        }

        /* --------------------------------------------------------------------------------------- */
        /* EVERY TOOTH WHEEL DECODER */
        /* --------------------------------------------------------------------------------------- */
EVERY_TOOTH:
        do_everytooth_calcs(&lsum, &lsum1, &localflags);
        do_tach_mask();
        dribble_burn();

        // that's most of the spark calcs done
        serial();
        do_sdcard();

        do_revlim_overboost_maxafr();

        if ((localflags & LOCALFLAGS_RUNFUEL) || (outpc.engine & ENGINE_CRANK)) {
            do_final_fuelcalcs();
            localflags &= ~LOCALFLAGS_RUNFUEL;
        }

        do_sequential_fuel();
        dribble_burn();

        shifter();
        generic_pwm();
        antilag();
        generic_pwm_outs();
        vvt();
        tclu();

        if (!(flagbyte10 & FLAGBYTE10_SPKCUTTMP)) {
            outpc.status2 &= ~STATUS2_SPKCUT;
            spk_cutx = 0;
            spk_cuty = 0;
        } else {
            outpc.status2 |= STATUS2_SPKCUT;
        }

        if (!(flagbyte10 & FLAGBYTE10_FUELCUTTMP)) {
            fuel_cutx = 0;
            fuel_cuty = 0;
        }

        handle_ovflo();


        /***************************************************************************
         **
         ** Check whether to burn flash
         ** (do this in SCI now)
         **************************************************************************/
        serial();
        ckstall();              // also calc PWM duties
        handle_spareports();

        /***************************************************************************
         **
         **  Check for CAN receiver timeout
         **
         **************************************************************************/
        DISABLE_INTERRUPTS;
        ultmp2 = ltch_CAN;
        ENABLE_INTERRUPTS;
        if ((ultmp > ultmp2) || flagbyte3 & flagbyte3_can_reset) {
            CanInit();          // start over again and clear flags
        }

        can_poll();
        if (ram5.can_bcast1 & 1) {
            can_broadcast();
        }

        do_egt();
        accelerometer();
        dribble_burn();

        // check for delayed ignition kill
        if (flagbyte6 & FLAGBYTE6_DELAYEDKILL) {
            unsigned long ult3;
            DISABLE_INTERRUPTS;
            ult3 = lmms;
            ENABLE_INTERRUPTS;
            if ((ult3 - sync_loss_stamp) > 39) {        // 5ms hardcoded max dwell before trying to fire coil
                FIRE_COIL;
            }
            if ((ult3 - sync_loss_stamp) > sync_loss_time) {
                ign_kill();
            }
        }

        /* if required, call read and compress file from sd */
        if (flagbyte7 & FLAGBYTE7_SENDFILE) {
            sd_timeout = 0;
            read_compress();    // inbuf, outbuf
            cp_flash_ram(); // re-copy data back again because compress used that space.
            flagbyte7 &= ~FLAGBYTE7_SENDFILE;
            outpc.sd_status |= 0x04; // ready again
        }

        /* calculate fuel flow */
        if (flagbyte14 & FLAGBYTE14_FUELFLOW_CALC) {
            flagbyte14 &= ~FLAGBYTE14_FUELFLOW_CALC;
            unsigned long ft;
            unsigned int inj1, inj2;
            if (ram4.hardware & HARDWARE_MS3XFUEL) {
                inj1 = ram4.staged_pri_size;
                if (ram4.sequential & SEQ_SEMI) {
                    inj1 *= 2;
                }
                if (ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3) {
                    inj2 = (ram4.staged_sec_size * ram4.NoInj) / 2;
                } else {
                    inj2 = ram4.staged_sec_size;
                    if (ram4.sequential & SEQ_SEMI) {
                        inj2 *= 2;
                    }
                }
            } else {
                inj1 = (ram4.staged_pri_size * ram4.NoInj) / 2;
                inj2 = (ram4.staged_sec_size * ram4.NoInj) / 2;
            }

            ft = ((flowsum[0] / 200) * inj1) / 10000;
            ft += ((flowsum[1] / 200) * inj2) / 10000;
            
            outpc.fuelflow = (unsigned int)ft;
        }

        if (pin_tsw_rf && (!(*port_tsw_rf & pin_tsw_rf))) {  // Reqfuel switching
            if ((orig_ReqFuel != ram4.ReqFuel_alt) || (orig_divider != ram4.Divider) || (orig_alternate != ram4.Alternate)) {
                /* User changed ReqFuel, recalculate the one we actually use */
                calc_reqfuel(ram4.ReqFuel_alt);
            }
        } else {
            if ((orig_ReqFuel != ram4.ReqFuel) || (orig_divider != ram4.Divider) || (orig_alternate != ram4.Alternate)) {
                /* User changed ReqFuel, recalculate the one we actually use */
                calc_reqfuel(ram4.ReqFuel);
            }
        }

        if (flagbyte16 & FLAGBYTE16_CHKSENS) {
            flagbyte16 &= ~FLAGBYTE16_CHKSENS;
            if  (ram5.cel_opt & 0x01) {
                check_sensors();
            }
        }

        /* 'user defined' */

        user_defined(); // call the user defined function - put your code in there

        /* end user defined section */

SKIP_THE_LOT:;

        /* port status */
        outpc.porta = PORTA;
        outpc.porta = PORTB;
        outpc.porteh = (PORTE & 0x17) | (PTH & 0xc0);
        outpc.portk = PORTK;
        outpc.portmj = (PTM & 0x3c) | (PTJ & 0x83);
        outpc.portp = PTP;
        outpc.portt = PTT;
   }                           //  END Main while(1) Loop
}
