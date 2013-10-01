/* $Id: ms3_misc.c,v 1.338.2.11 2013/04/06 15:35:08 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * boost_ctl_init()
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * boost_ctl()
    Origin: Kenneth Culver
    Moderate: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * get_adc()
    Origin: Al Grippo
    Major:  James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * barocor_eq()
    Origin: Al Grippo
    Majority: Al Grippo
 * aircor_eq()
    Origin: Al Grippo
    Moderate: James Murray
    Majority: Al Grippo / James Murray
 * CW_table()
    Origin: Al Grippo
    Minor: James Murray
    Majority: Al Grippo
 * set_spr_port()
    Trace: Al Grippo
    Major: Rewrite. James Murray
    Majority: James Murray
 * Flash_Init()
    Trace: Al Grippo
    Major: Rewrite. James Murray
    Majority: James Murray
 * realtime()
    Origin: James Murray
    Majority: James Murray
 * twoptlookup()
    Origin: James Murray
    Majority: James Murray
 * dribble_burn()
    Origin: James Murray
    Majority: James Murray
 * wheel_fill_map_event_array
    Origin: Kenneth Culver
    Moderate: James Murray
    Majority: Kenneth Culver / James Murray
 * speed_sensors()
    Origin: James Murray
    Majority: James Murray
 * nitrous()
    Origin: James Murray
    Majority: James Murray
 * median3pt
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * sample_map_tps()
    Origin: Al Grippo
    Major:  James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * calc_ITB_load()
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * calc_baro_mat_load()
    Origin: Al Grippo
    Major:  James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * water_inj()
    Origin: James Murray
    Majority: James Murray
 * do_launch()
    Origin: James Murray
    Majority: James Murray
 * do_revlim_overboost_maxafr
    Origin: Kenneth Culver
    Moderate: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * handle_ovflo()
    Origin: Kenneth Culver
    Moderate: MS3. James Murray
    Majority: James Murray / Kenneth Culver
 * handle_spareports()
    Origin: Al Grippo
    Minor: MS3. James Murray
    Majority: Al Grippo
 * do_egt()
    Origin: James Murray
    Majority: James Murray
 * do_sensors()
    Origin: James Murray
    Majority: James Murray
 * gearpos()
    Origin: James Murray
    Majority: James Murray
 * calcvssdot()
    Origin: James Murray
    Majority: James Murray
 * accelerometer()
    Origin: James Murray
    Majority: James Murray
 * ck_log_clr()
    Origin: James Murray
    Majority: James Murray
 * chk_crc()
    Origin: James Murray
    Majority: James Murray
 * long_abs()
    Origin: James Murray
    Majority: James Murray
 * shifter()
    Origin: James Murray
    Majority: James Murray
 * generic_pwm()
    Origin: James Murray
    Majority: James Murray
 * generic_pwm_out()
    Origin: James Murray
    Majority: James Murray
 * poll_i2c_rtc()
    Origin: James Murray
    Majority: James Murray
 * antilag()
    Origin: James Murray
    Majority: James Murray
 * vvt_ctl_pid_init()
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * vvt_pid()
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * vvt()
    Origin:  Kenneth Culver / James Murray
    Majority: Kenneth Culver / James Murray
 * tclu()
    Origin: James Murray
    Majority: James Murray
 * traction()
    Origin: James Murray
    Majority: James Murray
 * check_sensors(), check_sensors_reset(), check_sensors_init()
    Origin: James Murray
    Majority: James Murray
 * do_spi2()
    Origin: James Murray
    Majority: James Murray
 * ckstall(), clear_all()
    Origin: James Murray (was in ASM)
    Majority: James Murray
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/
#include "ms3.h"

void boost_ctl_init(void)
{
    boost_ctl_last_pv[0][0] = boost_ctl_last_pv[0][1] = 0;
    boost_ctl_last_pv[1][0] = boost_ctl_last_pv[1][1] = 0;
    boost_ctl_last_error[0] = 0;
    boost_ctl_last_error[1] = 0;
    boost_ctl_timer = ram4.boost_ctl_ms;
    boost_ctl_duty[0] = 100;
    outpc.boostduty = boost_ctl_duty[0];
    boost_ctl_duty[1] = 100;
    outpc.boostduty2 = boost_ctl_duty[1];
    boost_PID_enabled[0] = 0;
    boost_PID_enabled[1] = 0;
}

void boost_ctl_cl(int channel, int lowerlimit, int Kp, int Ki, int Kd, unsigned char closeduty,
                  unsigned char openduty, char *outputptr)
{
    /* CLOSED LOOP BOOST */
    int targ_load, maxboost;
    unsigned char flags = PID_TYPE_B, start_duty = openduty;
    int tmp1, tmp2;

    if (pin_tsw_ob && (!(*port_tsw_ob & pin_tsw_ob))) {
        maxboost = ram4.OverBoostKpa2;
    } else {
        maxboost = ram4.OverBoostKpa;
    }

    if (channel == 1) {
        targ_load = intrp_2ditable(outpc.rpm, outpc.tps, 8, 8,
                &ram_window.pg8.boost_ctl_loadtarg_rpm_bins[0],
                &ram_window.pg8.boost_ctl_loadtarg_tps_bins[0],
                &ram_window.pg8.boost_ctl_load_targets[0][0], 8);
        if (ram4.boost_ctl_settings & BOOST_CTL_USE_INITIAL) {
            start_duty = intrp_2dctable(outpc.rpm, targ_load, 8, 8,
                    &ram_window.pg25.boost_ctl_cl_pwm_rpms2[0],
                    &ram_window.pg25.boost_ctl_cl_pwm_targboosts2[0],
                    &ram_window.pg25.boost_ctl_cl_pwm_targs2[0][0], 0, 25);
        }
        outpc.boost_targ_2 = targ_load;
    } else {
        if ((ram4.boost_feats & 0x20) && (outpc.status2 & STATUS2_LAUNCH)) { 
            targ_load = ram4.boost_launch_target;
        } else if ((ram4.boost_vss & 0x03) &&
                   (outpc.tps > ram4.boost_vss_tps)) {
            unsigned int tmp_vss;
            if ((ram4.boost_vss & 0x03) == 2) {
                tmp_vss = outpc.vss2 / 10;
            } else {
                tmp_vss = outpc.vss1 / 10;
            }
            targ_load =
                intrp_1ditable(tmp_vss, 6,
                        (int *) ram_window.pg10.boostvss_speed, 0,
                        (int *) ram_window.pg10.boostvss_target, 10);
        } else {
            unsigned char w;
            tmp2 = tmp1 = 0; // keep compiler happy
            if ((pin_boost_tsw && (!(*port_boost_tsw & pin_boost_tsw))) ||
                (((ram4.boost_feats & 0x1f) == 15) &&
                 (outpc.gear >= ram4.boost_gear_switch))) {
                w = 2;
            } else if ((ram4.boost_feats & 0x1f) == 14) {
                w = 3;
            } else {
                w = 1;
            }

            if (w & 1) {
                tmp1 = intrp_2ditable(outpc.rpm, outpc.tps, 8, 8,
                        &ram_window.pg8.boost_ctl_loadtarg_rpm_bins[0],
                        &ram_window.pg8.boost_ctl_loadtarg_tps_bins[0],
                        &ram_window.pg8.boost_ctl_load_targets[0][0], 8);
            }
            if (w & 2) {
                tmp2 = intrp_2ditable(outpc.rpm, outpc.tps, 8, 8,
                        &ram_window.pg10.boost_ctl_loadtarg_rpm_bins2[0],
                        &ram_window.pg10.boost_ctl_loadtarg_tps_bins2[0],
                        &ram_window.pg10.boost_ctl_load_targets2[0][0], 10);
            }

            if (w == 1) {
                targ_load = tmp1;
            } else if (w == 2) {
                targ_load = tmp2;
            } else { // blended
                unsigned char blend;
                blend = intrp_1dctable(blend_xaxis(ram5.blend_opt[5] & 0x1f), 9, 
                        (int *) ram_window.pg25.blendx[5], 0, 
                        (unsigned char *)ram_window.pg25.blendy[5], 25);
                targ_load = (unsigned int)((((long)tmp1 * (100 - blend)) + ((long)tmp2 * blend)) / 100);
            }

            if ((ram4.boost_feats & 0x40) && pin_launch) {  // launch delay and launch enabled
                unsigned char boost_pct;
                unsigned int timer;

                if (outpc.status2 & STATUS2_LAUNCH) {
                    timer = 0; // hold at start boost
                } else if (launch_timer < 32760) {
                    timer = launch_timer;
                } else {
                    goto LD_SKIP; // not in launch or timed, so skip down-scaling
                }
                boost_pct = intrp_1dctable(timer, 6, 
                                           (int *) ram_window.pg10.boost_timed_time,
                                           0, (unsigned char *)ram_window.pg10.boost_timed_pct,
                                           10);
                /* scale gauge boost, not absolute pressure */
                targ_load = outpc.baro + ((unsigned long) (targ_load - outpc.baro) * 
                             (unsigned int) boost_pct) / 100;
                LD_SKIP:;
            }
        }

        /* traction control down-scaling */
        if (tc_boost != 100) {
            targ_load = outpc.baro + (int) (((targ_load - outpc.baro) * (long)tc_boost) / 100);
        }

        if (ram4.boost_ctl_settings & BOOST_CTL_USE_INITIAL) {
            start_duty = intrp_2dctable(outpc.rpm, targ_load, 8, 8,
                    &ram_window.pg25.boost_ctl_cl_pwm_rpms1[0],
                    &ram_window.pg25.boost_ctl_cl_pwm_targboosts1[0],
                    &ram_window.pg25.boost_ctl_cl_pwm_targs1[0][0], 0, 25);
        }

        outpc.boost_targ_1 = targ_load;
    }

    /* do not try to control boost below user-specified load or CLT*/
    if (((outpc.map < outpc.baro)) ||
        (outpc.map < (targ_load - lowerlimit))) {
        if (outpc.clt < ram4.boost_ctl_clt_threshold) {
            DISABLE_INTERRUPTS;
            boost_ctl_duty[channel] = closeduty;
            ENABLE_INTERRUPTS;
        } else {
            DISABLE_INTERRUPTS;
            boost_ctl_duty[channel] = start_duty;
            ENABLE_INTERRUPTS;
        }
        *outputptr = boost_ctl_duty[channel];
        boost_ctl_last_pv[channel][0] = boost_ctl_last_pv[channel][1] =
            boost_ctl_last_error[channel] = 0;
        boost_PID_enabled[channel] = 0;
        return;
    }

    if (!boost_PID_enabled[channel]) {
        flags |= PID_INIT;
        boost_PID_enabled[channel] = 1;
        boost_ctl_duty_100[channel] = boost_ctl_duty[channel] * 100;
    }

    tmp1 = boost_ctl_duty_100[channel] + (((long) (generic_pid_routine(1000, maxboost,
                        targ_load, outpc.map,
                        Kp, Ki, Kd/10, ram4.boost_ctl_ms,
                        boost_ctl_last_pv[channel], boost_ctl_last_error, 
                        flags)) * (openduty - closeduty)) / 1000);


    tmp2 = tmp1 / 100;

    if (tmp2 < closeduty) {
        tmp1 = closeduty * 100;
        tmp2 = closeduty;
    } else if (tmp2 > openduty) {
        tmp1 = openduty * 100;
        tmp2 = openduty;
    }

    boost_ctl_duty_100[channel] = tmp1;

    boost_ctl_duty[channel] = tmp2;
}

void boost_ctl_ol(int channel, unsigned char coldduty, char *outputptr)
{
    /* OPEN LOOP BOOST
     * lookup duty based on TPS,RPM
     */

    if (outpc.clt < ram4.boost_ctl_clt_threshold) {
        DISABLE_INTERRUPTS;
        boost_ctl_duty[channel] = coldduty;
        *outputptr = boost_ctl_duty[channel];
        ENABLE_INTERRUPTS;
        return;
    }

    if (channel == 1) {
        /* removed launch duty, vss, time based */
        boost_ctl_duty[1] = intrp_2dctable(outpc.rpm, outpc.tps, 8, 8,
                &ram_window.pg10.boost_ctl_pwmtarg_rpm_bins2[0],
                &ram_window.pg10.boost_ctl_pwmtarg_tps_bins2[0],
                &ram_window.pg10.boost_ctl_pwm_targets2[0][0],
                0, 10);
    } else {
        if ((ram4.boost_feats & 0x20) && pin_launch && (outpc.status2 & STATUS2_LAUNCH)) {
            boost_ctl_duty[channel] = ram4.boost_launch_duty;
        } else if ((ram4.boost_vss & 0x03)
                && (outpc.tps > ram4.boost_vss_tps)) {
            unsigned int tmp_vss;
            if ((ram4.boost_vss & 0x03) == 2) {
                tmp_vss = outpc.vss2;
            } else {
                tmp_vss = outpc.vss1;
            }
            boost_ctl_duty[channel] =
                intrp_1dctable(tmp_vss, 6,
                        (int *) ram_window.pg10.boostvss_speed,
                        0,
                        (unsigned char *) ram_window.
                        pg10.boostvss_duty, 10);
        } else {
            unsigned char w;
            int tmp1 = 0, tmp2 = 0;
            if ((pin_boost_tsw && (!(*port_boost_tsw & pin_boost_tsw))) ||
                (((ram4.boost_feats & 0x1f) == 15) &&
                 (outpc.gear >= ram4.boost_gear_switch))) {
                w = 2;
            } else if ((ram4.boost_feats & 0x1f) == 14) {
                w = 3;
            } else {
                w = 1;
            }

            if (w & 1) {
                tmp1 = intrp_2dctable(outpc.rpm, outpc.tps, 8, 8,
                            &ram_window.pg8.boost_ctl_pwmtarg_rpm_bins[0],
                            &ram_window.pg8.boost_ctl_pwmtarg_tps_bins[0],
                            &ram_window.pg8.boost_ctl_pwm_targets[0][0], 0,
                            8);
            }
            if (w & 2) {
                tmp2 = intrp_2dctable(outpc.rpm, outpc.tps, 8, 8,
                            &ram_window.pg10.boost_ctl_pwmtarg_rpm_bins2[0],
                            &ram_window.pg10.boost_ctl_pwmtarg_tps_bins2[0],
                            &ram_window.pg10.boost_ctl_pwm_targets2[0][0],
                            0, 10);
            }

            if (w == 1) {
                boost_ctl_duty[0] = tmp1;
            } else if (w == 2) {
                boost_ctl_duty[0] = tmp2;
            } else { // blended
                unsigned char blend;
                blend = intrp_1dctable(blend_xaxis(ram5.blend_opt[5] & 0x1f), 9, 
                        (int *) ram_window.pg25.blendx[5], 0, 
                        (unsigned char *)ram_window.pg25.blendy[5], 25);
                boost_ctl_duty[0] = (unsigned int)((((long)tmp1 * (100 - blend)) + ((long)tmp2 * blend)) / 100);
            }

            if (pin_launch && (ram4.boost_feats & 0x40)) {  // launch delay
                unsigned char boost_pct;
                boost_pct = intrp_1dctable(launch_timer, 6, (int *) ram_window.pg10.boost_timed_time,
                        0, (unsigned char *)ram_window.pg10.boost_timed_pct, 10);

                boost_ctl_duty[channel] = (unsigned char) (((unsigned int) boost_ctl_duty[channel] *
                            (unsigned int) boost_pct) / 100);
            }
        }
    }
}

void boost_ctl(void)
{
    int tmp1;

    if (outpc.status7 & STATUS7_LIMP) {
        outpc.boostduty = ram5.cel_boost_duty;
        outpc.boostduty2 = ram5.cel_boost_duty2;
        return;
    }

    if (flagbyte3 & flagbyte3_runboost) {
        DISABLE_INTERRUPTS;
        flagbyte3 &= ~flagbyte3_runboost;
        ENABLE_INTERRUPTS;
        if (ram4.boost_ctl_settings & BOOST_CTL_CLOSED_LOOP) {
            boost_ctl_cl(0, ram4.boost_ctl_lowerlimit, ram4.boost_ctl_Kp,
                         ram4.boost_ctl_Ki, ram4.boost_ctl_Kd,
                         ram4.boost_ctl_closeduty, ram4.boost_ctl_openduty,
                         &outpc.boostduty);
        } else {
            boost_ctl_ol(0, 0, &outpc.boostduty); /* , 0 was ram4.boost_ctl_openduty */
        }

        outpc.boostduty = boost_ctl_duty[0];
        tmp1 = (boost_ctl_duty[0] * 255) / 100;
        boost_ctl_duty_pwm[0] = (unsigned char) tmp1;

        /* Boost control second channel - no table switching support,
         * no boost vs vss, timer. Always uses second tables */
        /* Incomplete implementation at present, worth reviewing for
         * sequential boost or true twin channel */

        if (ram5.boost_ctl_settings2 & BOOST_CTL_ON) {
            if (ram5.boost_ctl_settings2 & BOOST_CTL_CLOSED_LOOP) {
                boost_ctl_cl(0, ram5.boost_ctl_lowerlimit2, ram5.boost_ctl_Kp2,
                             ram5.boost_ctl_Ki2, ram5.boost_ctl_Kd2,
                             ram5.boost_ctl_closeduty2,
                             ram5.boost_ctl_openduty2, &outpc.boostduty2);
            } else {
                /* OPEN LOOP BOOST
                 * lookup duty based on TPS,RPM
                 */
                boost_ctl_ol(1, 0, &outpc.boostduty2); /* , 0 was ram5.boost_ctl_openduty2 */
            }
            outpc.boostduty2 = boost_ctl_duty[1];
            tmp1 = (boost_ctl_duty[1] * 255) / 100;
            boost_ctl_duty_pwm[1] = (unsigned char) tmp1;
        }
    }
}

void get_adc(char chan1, char chan2)
{
    char chan;
    //long adcval;
    int adcvalv, tmp1, tmp3;
    unsigned int tmpadc;

    for (chan = chan1; chan <= chan2; chan++) {
        //    switch(chan)  {
        // when using switch, gcc puts lookup table around 0x5000 and then linker
        // gets all upset because we are in page 0x3c and jump table isn't. Replace
        // with ifs instead. No functional difference. This is known bug in gcc
        if (chan == 0) {
            unsigned int mapadc;
            mapadc = *mapport;

            //removed lag code and first_adc comparison as only called here from init
            __asm__ __volatile__("ldd    %1\n"
                                 "subd   %2\n"
                                 "ldy    %3\n"
                                 "emul\n"
                                 "ldx    #1023\n"
                                 "ediv\n" "addy   %2\n":"=y"(outpc.map)
                                 :"m"(ram4.mapmax), "m"(ram4.map0),
                                 "m"(mapadc)
                                 :"d", "x");

        } else if ((chan == 1) && (!burnstat) && (!ltt_fl_state)) {
            int i;
            unsigned int avg_adc;
            // sliding window
            avg_adc = 0;
            for (i = MAT_N - 1 ; i > 0 ; i--) {
                mat_data[i] = mat_data[i-1];
                avg_adc += mat_data[i];
            }
            mat_data[0] = ATD0DR1;
            avg_adc += mat_data[0];

            if (first_adc) { // fill with same value
                for (i = MAT_N - 1 ; i > 0 ; i--) {
                    mat_data[i] = mat_data[0];
                }
                avg_adc = mat_data[0];
            } else {
                avg_adc /= MAT_N;
            }

            //        adcval = (long)ram4.mat0 +
            //          ((long)ram4.matmult * matfactor_table[ATD0DR1]) / 100; // deg F or C x 10
            GPAGE = 0x10;
            __asm__ __volatile__("aslx\n" "addx   #0x4800\n"    // matfactor_table address
                                 "gldy   0,x\n"
                                 "ldd    %2\n"
                                 "emul\n"
                                 "ldx    #100\n"
                                 "ediv\n" "addy   %3\n":"=y"(tmp1)
                                 :"x"(avg_adc), "m"(ram4.matmult),
                                 "m"(ram4.mat0)
                                 :"d");

            if (first_adc) {
                outpc.mat = tmp1;
            } else {
                //          outpc.mat += (short)((ram4.adcLF * (adcval - outpc.mat)) / 100);
                __asm__ __volatile__(
                                        //                "ldd    %1\n"
                                        "subd   %3\n" "tfr    d,y\n" "clra\n" "ldab    %2\n"    // it is a uchar
                                        "emuls\n"
                                        "ldx     #100\n"
                                        "edivs\n"
                                        "addy    %3\n":"=y"(outpc.mat)
                                        :"d"(tmp1), "m"(ram4.adcLF),
                                        "m"(outpc.mat)
                                        :"x");
            }

            if (stat_mat && (ram5.cel_action1 & 0x02)) { /* MAT sensor is broken */
                outpc.mat = ram5.cel_mat_default;
            }


        } else if ((chan == 2) && (!burnstat) && (!ltt_fl_state)) {
            int i;
            unsigned int avg_adc;
            // sliding window
            avg_adc = 0;
            for (i = CLT_N - 1 ; i > 0 ; i--) {
                clt_data[i] = clt_data[i-1];
                avg_adc += clt_data[i];
            }
            clt_data[0] = ATD0DR2;
            avg_adc += clt_data[0];

            if (first_adc) { // fill with same value
                for (i = CLT_N - 1 ; i > 0 ; i--) {
                    clt_data[i] = clt_data[0];
                }
                avg_adc = clt_data[0];
            } else {
                avg_adc /= CLT_N;
            }
            //        adcval = (long)ram4.clt0 +
            //          ((long)ram4.cltmult * cltfactor_table[ATD0DR2]) / 100; // deg F or C x 10
            GPAGE = 0x10;
            __asm__ __volatile__("aslx\n" "addx   #0x4000\n"    // cltfactor_table address
                                 "gldy   0,x\n"
                                 "ldd    %2\n"
                                 "emul\n"
                                 "ldx    #100\n"
                                 "ediv\n" "addy   %3\n":"=y"(tmp1)
                                 :"x"(avg_adc), "m"(ram4.cltmult),
                                 "m"(ram4.clt0)
                                 :"d");
            if (first_adc)
                outpc.clt = tmp1;
            else {
                //          outpc.clt += (short)((ram4.adcLF * (adcval - outpc.clt)) / 100);
                __asm__ __volatile__("ldd    %1\n" "subd   %3\n" "tfr    d,y\n" "clra\n" "ldab    %2\n" // it is a uchar
                                     "emuls\n"
                                     "ldx     #100\n"
                                     "edivs\n"
                                     "addy    %3\n":"=y"(outpc.clt)
                                     :"m"(tmp1), "m"(ram4.adcLF),
                                     "m"(outpc.clt)
                                     :"d", "x");
            }

            if (stat_clt && (ram5.cel_action1 & 0x04)) { /* CLT sensor is broken */
                if (outpc.seconds > ram5.cel_warmtime) {
                    outpc.clt = ram5.cel_clt_warm;
                } else {
                    /* interpolate a faked warmup */
                    outpc.clt = ram5.cel_clt_cold + (((ram5.cel_clt_warm - ram5.cel_clt_cold) * (long)outpc.seconds) / ram5.cel_warmtime);
                } 
            }
        } else if (chan == 3) {
            // only used by init?
            outpc.tpsadc = ATD0DR3;
            if (first_adc) {
                /* fill ring */
                tps_ring[0] = outpc.tpsadc;
                tps_ring[1] = outpc.tpsadc;
                tps_ring[2] = outpc.tpsadc;
                tps_ring[3] = outpc.tpsadc;
                tps_ring[4] = outpc.tpsadc;
                tps_ring[5] = outpc.tpsadc;
                tps_ring[6] = outpc.tpsadc;
                tps_ring[7] = outpc.tpsadc;

                __asm__ __volatile__
                ("ldd %3\n"
                 "subd %2\n"
                 "tfr  d,y\n"
                 "ldd  %1\n"
                 "subd %2\n"
                 "pshd\n"
                 "ldd #1000\n"
                 "emuls\n"
                 "pulx\n"
                 "edivs\n"
                 :"=y"(outpc.tps)
                 :"m"(ram4.tpsmax), "m"(tps0_auto), "m"(ATD0DR3)
                 :"d", "x");
            }
        } else if (chan == 4) {
            int i;
            unsigned int avg_adc;
            // sliding window
            avg_adc = 0;
            for (i = BATT_N - 1 ; i > 0 ; i--) {
                batt_data[i] = batt_data[i-1];
                avg_adc += batt_data[i];
            }
            batt_data[0] = ATD0DR4;
            avg_adc += batt_data[0];

            if (first_adc) { // fill with same value
                for (i = BATT_N - 1 ; i > 0 ; i--) {
                    batt_data[i] = batt_data[0];
                }
                avg_adc = batt_data[0];
            } else {
                avg_adc /= BATT_N;
            }
            //        adcval = (long)ram4.batt0 +
            //          ((long)(ram4.battmax - ram4.batt0) * ATD0DR4) / 1023; // V x 10
            __asm__ __volatile__("ldd    %1\n"
                                 "subd   %2\n"
                                 "ldy    %3\n"
                                 "emul\n"
                                 "ldx    #1023\n"
                                 "ediv\n"
                                 "addy   %2\n":"=y"(tmp1)
                                 :"m"(ram4.battmax), "m"(ram4.batt0), "m"(avg_adc)
                                 :"d", "x");
            if (first_adc)
                outpc.batt = tmp1;
            else {
                //        outpc.batt += (short)((ram4.adcLF * (adcval - outpc.batt)) / 100);
                __asm__ __volatile__("ldy    %1\n" "suby   %3\n" "clra\n" "ldab    %2\n"        // it is a uchar
                                     "emuls\n"
                                     "ldx     #100\n"
                                     "edivs\n"
                                     "addy    %3\n":"=y"(outpc.batt)
                                     :"m"(tmp1), "m"(ram4.adcLF),
                                     "m"(outpc.batt)
                                     :"x", "d");
            }
        } else if ((chan == 5) && (!burnstat) && (!ltt_fl_state)) {
            int start, egonum, ix;
            // do all EGO sensors in here.

            if (ram4.EgoOption & 0x03) {
                egonum = ram4.egonum;
            } else {
                egonum = 1;
            }

            start = 0;

            for (ix = start; ix < egonum ; ix++) {
                tmpadc = *egoport[ix];

                // check if sensor bad (near limits)
                if ((tmpadc < 3) || (tmpadc > 1020)) {
                    bad_ego_flag |= (0x01 << ix);
                } else {
                    bad_ego_flag &= ~(0x01 << ix);
                }
                
                if ((ram5.egoport[ix] & 0x1f) == 7) {
                    // presently only Innovate 0.5 - 1.523 lambda = 0 to 1023
                    // counts supported convert to AFR
                    unsigned long ul;

                    ul = (tmpadc + 500) * (unsigned long)ram4.stoich;
                    tmp3 = ul / 1000;

                    adcvalv = 0; // not used

                } else {

                    GPAGE = 0x10;
                    __asm__ __volatile__
                    ("addx   #0x5000\n"     //egofactor_table address
                     "gldab 0,x\n"
                     "clra\n"
                     "ldy    %2\n"
                     "emul\n"
                     "ldx    #100\n"
                     "ediv\n"
                     "addy   %3\n"
                     :"=y"(tmp3)
                     :"x"(tmpadc), "m"(ram4.egomult), "m"(ram4.ego0)
                     :"d");

                    __asm__ __volatile__
                    ("ldd #250\n"
                     "emul\n"
                     "ldx #1023\n"
                     "ediv\n"
                     :"=y"(adcvalv)
                     :"y"(tmpadc)
                     :"d", "x");
                }

                if (first_adc) {
                    outpc.afr[ix] = tmp3;
                    outpc.egov[ix] = tmpadc;
                } else {
                    int tmp2, tmp4;
// lag factor on AFR
                    tmp4 = outpc.afr[ix];
                    //            outpc.ego1 += (short)((ram4.egoLF * (adcval - outpc.ego1)) / 100);
                    __asm__ __volatile__
                    ("ldy    %1\n"
                     "suby   %3\n"
                     "clra\n"
                     "ldab    %2\n"        // it is a uchar
                     "emuls\n"
                     "ldx     #100\n"
                     "edivs\n"
                     "addy    %3\n"
                     :"=y"(outpc.afr[ix])
                     :"m"(tmp3), "m"(ram4.egoLF),"m"(tmp4)
                     :"x", "d");
// lag factor on EGO volts
                    tmp1 = (int) ram4.egoLF;
                    tmp2 = tmpadc - (int)outpc.egov[ix];
                    __asm__ __volatile__
                    ("emuls\n"
                     "ldx #100\n"
                     "edivs\n"
                     :"=y"(adcvalv)
                     :"d"(tmp1), "y"(tmp2)
                     :"x");

                    tmp4 = (int)outpc.egov[ix] + adcvalv; // Vx100
                    outpc.egov[ix] = tmp4;
                }
            }
            // for compatability
            outpc.ego1 = outpc.afr[0];
            outpc.ego2 = outpc.afr[1];
            outpc.egoV1 = outpc.egov[0];
            outpc.egoV2 = outpc.egov[1];
        }                       // end of switch
    }                           // end of for loop

    // if GPIO slave copy these raw ADCs to a convenient place so master can grab them
    if (ram4.mycan_id) {
        outpc.sensors[0] = ATD0DR0;
        outpc.sensors[1] = ATD0DR1;
        outpc.sensors[2] = ATD0DR2;
        outpc.sensors[3] = ATD0DR3;
        outpc.sensors[4] = ATD0DR4;
        outpc.sensors[5] = ATD0DR5;
        outpc.sensors[6] = ATD0DR6;
        outpc.sensors[7] = ATD0DR7;
    }
    // now calculate other stuff that uses optional ADC inputs
    if (ram4.BaroOption == 2) {

       tmpadc = *baroport; 

        //          adcval = (long)ram4.baro0 +
        //            ((long)(ram4.baromax - ram4.baro0) * tmpadc) / 1023; // kPa x 10
        __asm__ __volatile__("ldd    %1\n"
                             "subd   %2\n"
                             "ldy    %3\n"
                             "emul\n"
                             "ldx    #1023\n"
                             "ediv\n" "addy   %2\n":"=y"(tmp1)
                             :"m"(ram4.baromax), "m"(ram4.baro0),
                             "m"(tmpadc)
                             :"x", "d");

        if (first_adc) {
            outpc.baro = tmp1;
        } else {
            //            outpc.baro += (short)((ram4.adcLF * (adcval - outpc.baro)) / 100);
            __asm__ __volatile__("ldy    %1\n" "suby   %3\n" "clra\n" "ldab    %2\n"    // it is a uchar
                                 "emuls\n"
                                 "ldx     #100\n"
                                 "edivs\n" "addy    %3\n":"=y"(outpc.baro)
                                 :"m"(tmp1), "m"(ram4.adcLF),
                                 "m"(outpc.baro)
                                 :"x", "d");
        }
        if (outpc.baro < ram4.baro_lower) {
            outpc.baro = ram4.baro_lower;
        } else if (outpc.baro > ram4.baro_upper) {
            outpc.baro = ram4.baro_upper;
        }
    }

    return;
}

int barocor_eq(int baro)
{
    // returns baro correction in % (100 is no correction)
    // baro in kPa x 10
    return ((int)
            (ram4.bcor0 * 10 + (((long) ram4.bcormult * baro) / 100)));
}

int CW_table(int clt, int *table, int *temp_table, unsigned char page)
{
    int ix;
    long interp, interp3;

    RPAGE = tables[page].rpg;
    // returns values for various cold warmup table interpolations
    // bound input arguments
    if (clt > temp_table[NO_TEMPS - 1]) {
        return (table[NO_TEMPS - 1]);
    }
    if (clt < temp_table[0]) {
        return (table[0]);
    }
    for (ix = NO_TEMPS - 2; ix > -1; ix--) {
        if (clt > temp_table[ix]) {
            break;
        }
    }
    if (ix < 0)
        ix = 0;

    interp = temp_table[ix + 1] - temp_table[ix];
    if (interp != 0) {
        interp3 = (clt - temp_table[ix]);
        interp3 = (100 * interp3);
        interp = interp3 / interp;
    }
    return ((int)
            (table[ix] + interp * (table[ix + 1] - table[ix]) / 100));
}

//*****************************************************************************
// Function to set clear output port bit based
// port = software port no.
// lookup actual hardware port in table
// ports 0 = PM, 1 = PJ, 2 = PP, 3 = PT, 4 = PA, 5 = PB, 6 = PK
// val = 0 or 1 for off/on
//*****************************************************************************
void set_spr_port(char port, char val)
{
    unsigned char hw_mask, *hw_addr;

    hw_mask = spr_port_hw[2][(int) port];
    hw_addr = (unsigned char *) spr_port_addr[(int) port];

// wrap XGATE semaphore around here in case a port is shared with XGATE

    if (val) {
        SSEM0SEI;
        *hw_addr |= hw_mask;
        CSEM0CLI;
    } else {
        SSEM0SEI;
        *hw_addr &= ~hw_mask;
        CSEM0CLI;
    }
}

//*****************************************************************************
//* Function Name: Flash_Init
//* Description : Initialize Flash NVM for HCS12 by programming
//* FCLKDIV based on passed oscillator frequency, then
//* uprotect the array, and finally ensure PVIOL and
//* ACCERR are cleared by writing to them.
//*
//*****************************************************************************
void Flash_Init()
{
    if (!(FSTAT & 0x80)) {
        conf_err = 50;          // flash busy error - should not be busy!
    } else {
        /* Next, initialize FCLKDIV register to ensure we can program/erase */
        if ((FCLKDIV & 0x80) == 0) {    // if not already set, then set it.
            FCLKDIV = 7;        // for 8MHz crystal OSCCLK
        }
        //  FPROT = 0xFF; /* Disable all protection (only in special modes)*/
        // commented as it will likely spew out error normally
        FSTAT = 0x30;  // clear ACCERR or FPVIOL if set
    }
    return;
}

int twoptlookup(unsigned int x, unsigned int x0, unsigned int x1,
            int y0, int y1)
{
    long interp, interp3;
    int result;

    // bound input arguments
    if (x >= x1) {
        return (y1);
    }

    if (x <= x0) {
        return (y0);
    }

    interp = (long) x1 - (long) x0;
    interp3 = ((long) x - (long) x0);
    interp3 = (100 * interp3);
    interp = interp3 / interp;

    interp = interp * ((long) y1 - (long) y0) / 100;
    result = (int) ((long) y0 + interp);

    return (result);
}

void dribble_burn()
{                               /* dribble burning */
    if ((burnstat) && (flocker == 0xdd)) {
        if ((burnstat > 1) && ((FSTAT & 0x80) == 0)) {
            goto DB_END;        // previous flash command not completed, so skip
        }
/*
this is trapped in serial.c
        if ((burn_idx < 4) && ((ram5.flashlock & 0x01) == 0)) {
            // shouldn't get here normally
            // could occur on illegal write to sensor calibrations
            flagbyte10 |= FLAGBYTE10_LOCKEDFLASH;
            goto DB_END;
        }
*/
        if ((burnstat >= 1) && (burnstat <= 4)) {
            if (burnstat == 1) {
                Flash_Init();
            }
            //erase all d-flash sectors corresponding to this data page
            // first identify the global address for the first sector
            unsigned int ad150;
            ad150 = (unsigned int) tables[burn_idx].addrFlash;

            if ((tables[burn_idx].n_bytes == 0)
                || (tables[burn_idx].n_bytes > 1024) || (ad150 > 0x8000)) {
                //invalid or unrecognised page sent here (not foolproof detection)
                burnstat = 0;
                flocker = 0;
                goto DB_END;
            }
            ad150 += (burnstat - 1) * 0x100;
            // erase the specified flash block
            DISABLE_INTERRUPTS;
            if ((FSTAT & 0x30) != 0) {
                FSTAT = 0x30;  // clear ACCERR or FPVIOL if set
            }

            FCCOBIX = 0;
            FCCOBHI = 0x12;     // erase D-flash sector command
            FCCOBLO = 0x10;     // global address of all D-flash

            FCCOBIX = 1;
            FCCOB = ad150;      // address within sector

            FSTAT = 0x80;       // initiate flash command (datasheet very confusing)
            ENABLE_INTERRUPTS;

            burnstat++;

        } else if (burnstat >= 5) {
            // write data to flash page, a word at a time
            unsigned char rp;
            unsigned int ad150, *ad_loc;

            ad150 = tables[burn_idx].addrFlash;

            if ((tables[burn_idx].n_bytes == 0)
                || (tables[burn_idx].n_bytes > 2048) || (ad150 >= 0x8000)) {
                //invalid or unrecognised page sent here (not foolproof detection)
                burnstat = 0;
                flocker = 0;
                goto DB_END;
            }
            rp = tables[burn_idx].rpg;
            // sanity check in case of corruption, but allow calibration tables
            if ((rp < 0xf0) && (tables[burn_idx].adhi != 0xf0)) {
                // how did this happen?
                burnstat = 0;
                flocker = 0;
                goto DB_END;
            }
            ad_loc = (unsigned int *) (tables[burn_idx].adhi << 8);
            ad150 += (burnstat - 5) * 8;        // we prog 8 bytes at a time
            ad_loc += (burnstat - 5) * 4;       // only 4 because uint so gets doubled

            if ((FSTAT & 0x30) != 0) {
                FSTAT = 0x30;  // clear ACCERR or FPVIOL if set
            }

            DISABLE_INTERRUPTS;
            RPAGE = rp;

            if (ad150 >= 0x8000) {
                // how did this happen?
                burnstat = 0;
                flocker = 0;
                goto DB_END;
            }

            FCCOBIX = 0;
            FCCOBHI = 0x11;     // prog D-flash
            FCCOBLO = 0x10;     // global addr

            FCCOBIX = 1;
            FCCOB = ad150;      //global addr
            if (burn_idx < 4) {
                /* calibration tables, handled differently */
                ad_loc += 2; /* adds four for 0xf004*/

                FCCOBIX = 2;
                FCCOB = g_read16((unsigned int)ad_loc);
                ad_loc++;           // (words)

                FCCOBIX = 3;
                FCCOB = g_read16((unsigned int)ad_loc);
                ad_loc++;

                FCCOBIX = 4;
                FCCOB = g_read16((unsigned int)ad_loc);
                ad_loc++;

                FCCOBIX = 5;
                FCCOB = g_read16((unsigned int)ad_loc);
            } else {
                FCCOBIX = 2;
                FCCOB = *ad_loc;
                ad_loc++;           // (words)

                FCCOBIX = 3;
                FCCOB = *ad_loc;
                ad_loc++;

                FCCOBIX = 4;
                FCCOB = *ad_loc;
                ad_loc++;

                FCCOBIX = 5;
                FCCOB = *ad_loc;
            }

            FSTAT = 0x80;       // initiate flash command
            ENABLE_INTERRUPTS;

            if ((((burn_idx >= 4) || (burn_idx == 2)) && (burnstat < 132))
                || (((burn_idx < 2) || (burn_idx == 3)) && (burnstat < 260))) {
                burnstat++;
            } else {
                // finished
                burnstat = 0;
                flocker = 0;
                outpc.status1 &= ~(STATUS1_NEEDBURN + STATUS1_LOSTDATA);        // clear NeedBurn,LostData on burn completion
            }
        }
    } else if (((burnstat == 0) && (flocker == 0xdd)) || ((burnstat) && (flocker != 0xdd))) {
        // shouldn't happen
        burnstat = 0;
        flocker = 0;
        // might want to kill serial too
    }
  DB_END:;
}

void wheel_fill_map_event_array(map_event * map_events_fill, int map_ang,
                                ign_time last_tooth_time,
                                unsigned int last_tooth_ang, unsigned int duration)
{
    char iterate, tth, wfe_err;
    int tth_ang, tmp_ang, i;
    unsigned int map_time, map_window_time;

    if (map_ang < 0) {
        map_ang += cycle_deg;
    }

    map_window_time =
        (unsigned
         int) (((duration * last_tooth_time.time_32_bits) /
                last_tooth_ang) / 128L);
    if (map_window_time < 1) {
        map_window_time = 1;
    }

    for (i = 0; i < no_triggers; i++) {
        wfe_err = 0;
        iterate = 0;
        tth_ang = trig_angs[i];
        tth = trigger_teeth[i];
        while (!iterate) {
            if (tth_ang > map_ang) {
                iterate = 1;
            } else {
                //how far do we step back in deg
                tth--;
                if (tth < 1) {
                    tth = last_tooth;
                    wfe_err++;
                    if (wfe_err > 1) {
                        iterate = 2;
                    }
                }
                tth_ang += deg_per_tooth[tth - 1];
            }
        }
        if (iterate == 2) {
            DISABLE_INTERRUPTS;
            asm("nop\n");       // something screwed up, place for breakpoint
            ENABLE_INTERRUPTS;
            // can't continue as didn't find a valid tooth
            return;
        }

        tmp_ang = tth_ang - map_ang;

        map_time =
            (unsigned
             int) (((tmp_ang * last_tooth_time.time_32_bits) /
                    last_tooth_ang) / 128L);
        wfe_err = 0;

        while ((wfe_err < 2) && (map_time < 1)) {
            // too soon after tooth, need to step back
            tth--;
            if (tth < 1) {
                tth = last_tooth;
                wfe_err++;
            }
            tth_ang += deg_per_tooth[tth - 1];
            // recalc
            tmp_ang = tth_ang - map_ang;
            map_time =
                (unsigned
                 int) (((tmp_ang * last_tooth_time.time_32_bits) /
                        last_tooth_ang) / 128L);
        }

        if (wfe_err > 1) {
            DISABLE_INTERRUPTS;
            asm("nop\n");       // something screwed up, place for breakpoint
            ENABLE_INTERRUPTS;
            // can't continue as didn't find a valid tooth
            return;
        }

        map_events_fill[i].tooth = tth;
        map_events_fill[i].time = map_time;
        map_events_fill[i].map_window_set = map_window_time;
        map_events_fill[i].evnum = i;
    }
}

void speed_sensors()
{
    unsigned int vss_sample_time, tmp_speed;
    unsigned char dl;
    // VSS code. Derived from Microsquirt trans code

    // Adaptive averaging on VSS
    if (outpc.vss1 > 10) {
        vss_sample_time = 15000 / outpc.vss1;
    } else {
        vss_sample_time = 1500;
    }

    if (((unsigned int)lmms - vss_time) > vss_sample_time) {
        vss_time = (unsigned int)lmms;
        if ((ram4.vss1_an & 0x1f) || (!(ram4.vss1_an & 0x1f) && ((ram4.vss_opt & 0x0f) >= 0x0e))) {
            flagbyte8 |= FLAGBYTE8_SAMPLE_VSS1;
        }
        if ((ram4.vss2_an & 0x1f) || (!(ram4.vss2_an & 0x1f) && ((ram4.vss_opt & 0xf0) >= 0xe0))) {
            flagbyte8 |= FLAGBYTE8_SAMPLE_VSS2;
        }
    }

    dl = 0;
    if (ram4.vss1_an & 0x1f) {
        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS1) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS1;
            tmp_speed =
                (unsigned int) (*(unsigned int *) &(*port_vss1) *
                                (unsigned long) ram4.vss1_an_max /
                                (unsigned long) 1023);
            dl = 1;
        }
    } else if (pin_vss1) {
        if (vss1_stall < VSS_STALL_TIMEOUT) {
            if ((flagbyte8 & FLAGBYTE8_SAMPLE_VSS1) && (vss1_time_sum > 0)) {
                tmp_speed = (unsigned int) ((vss1_coeff * vss1_teeth) / vss1_time_sum);
                vss1_time_sum = 0;
                vss1_teeth = 0;
                flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS1;
                dl = 1;
            }
        } else {
            vss1_stall = VSS_STALL_TIMEOUT + 1; // keep it from rolling over
            outpc.vss1 = 0;
            vss1_time = 0;
        }
    } else if ((ram4.vss_opt & 0x0f) == 0x0e) {
        unsigned int tmp_vss;

        if (ram4.vss_can_size & 0x01) { // 16 bit
            tmp_vss = datax1.vss1_16;
        } else {
            tmp_vss = datax1.vss1_8;
        }

        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS1) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS1;
            tmp_speed = (unsigned int)(((unsigned long)tmp_vss * (unsigned long)ram4.vss1_can_scale) / 1000L);
            dl = 1 ;
        }
    } else if ((ram4.vss_opt & 0x0f) == 0x0f) {
        unsigned long tmppwm;

        if (ram4.enable_poll & 0x04) { // 32 bit
            unsigned long *tmp_ad;
            tmp_ad = (unsigned long*)&datax1.pwmin32[ram4.vss_pwmseq[0]-1];
            DISABLE_INTERRUPTS;
            tmppwm = *tmp_ad;
            ENABLE_INTERRUPTS;
        } else {
            tmppwm = (unsigned long)datax1.pwmin16[ram4.vss_pwmseq[0]-1];
        }

        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS1) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS1;
            tmp_speed = (unsigned int) (vss1_coeff / tmppwm);
            dl = 1;
        }
    }

    if (dl) {
        __asm__ __volatile__("ldd    %1\n"
                             "subd   %3\n"
                             "tfr    d,y\n"
                             "clra\n"
                             "ldab    %2\n" // it is a uchar
                             "emuls\n"
                             "ldx     #100\n"
                             "edivs\n"
                             "addy    %3\n"
                            :"=y"(outpc.vss1)
                             :"m"(tmp_speed), "m"(ram4.vss1LF), "m"(outpc.vss1)
                             :"d", "x");
        if ((outpc.vss1 < 20) && (tmp_speed == 0)) {
            outpc.vss1 = 0;
        }

    }

    dl = 0;
    if (ram4.vss2_an & 0x1f) {
        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS2) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS2;
            tmp_speed =
                (unsigned int) (*(unsigned int *) &(*port_vss2) *
                                (unsigned long) ram4.vss2_an_max /
                                (unsigned long) 1023);
            dl = 1;
        }
    } else if (pin_vss2) {
        if (vss2_stall < VSS_STALL_TIMEOUT) {
            if ((flagbyte8 & FLAGBYTE8_SAMPLE_VSS2) && (vss2_time_sum > 0)) {
                tmp_speed = (unsigned int) ((vss2_coeff * vss2_teeth) / vss2_time_sum);
                vss2_time_sum = 0;
                vss2_teeth = 0;
                flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS2;
                dl = 1;
            }
        } else {
            vss2_stall = VSS_STALL_TIMEOUT + 1; // keep it from rolling over
            outpc.vss2 = 0;
            vss2_time = 0;
        }
    } else if ((ram4.vss_opt & 0xf0) == 0xe0) {
        unsigned int tmp_vss;

        if (ram4.vss_can_size & 0x02) { // 16 bit
            tmp_vss = datax1.vss2_16;
        } else {
            tmp_vss = datax1.vss2_8;
        }
        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS2) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS2;

            tmp_speed = (unsigned int)(((unsigned long)tmp_vss * (unsigned long)ram4.vss2_can_scale) / 1000L);
            dl = 1;
        }
    } else if ((ram4.vss_opt & 0xf0) == 0xf0) {
        unsigned long tmppwm;

        if (ram4.enable_poll & 0x04) { // 32 bit
            tmppwm = datax1.pwmin32[ram4.vss_pwmseq[1]-1];
        } else {
            tmppwm = (unsigned long)datax1.pwmin16[ram4.vss_pwmseq[1]-1];
        }

        if (flagbyte8 & FLAGBYTE8_SAMPLE_VSS2) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_VSS2;
            tmp_speed = (unsigned int) (vss2_coeff / tmppwm);
            dl = 1;
        }
    }

    if (dl) {
        __asm__ __volatile__("ldd    %1\n"
                             "subd   %3\n"
                             "tfr    d,y\n"
                             "clra\n"
                             "ldab    %2\n" // it is a uchar
                             "emuls\n"
                             "ldx     #100\n"
                             "edivs\n"
                             "addy    %3\n"
                            :"=y"(outpc.vss2)
                             :"m"(tmp_speed), "m"(ram4.vss2LF), "m"(outpc.vss2)
                             :"d", "x");
        if ((outpc.vss2 < 20) && (tmp_speed == 0)) {
            outpc.vss2 = 0;
        }
    }

    // Shaft Speed
    if (pin_ss1) {
        if (ss1_stall < VSS_STALL_TIMEOUT) {
            if (flagbyte8 & FLAGBYTE8_SAMPLE_SS1) {
            flagbyte8 &= ~FLAGBYTE8_SAMPLE_SS1;
                if (ss1_time > 0) {

                    tmp_speed = (unsigned int) (ss1_coeff / ss1_time);
                    __asm__ __volatile__("ldd    %1\n"
                                         "subd   %3\n"
                                         "tfr    d,y\n"
                                         "clra\n"
                                         "ldab    %2\n" // it is a uchar
                                         "emuls\n"
                                         "ldx     #100\n"
                                         "edivs\n"
                                         "addy    %3\n"
                                        :"=y"(outpc.ss1)
                                         :"m"(tmp_speed), "m"(ram4.ss1LF), "m"(outpc.ss1)
                                         :"d", "x");
                }
            }
        } else {
            ss1_stall = VSS_STALL_TIMEOUT + 1;  // keep it from rolling over
            outpc.ss1 = 0;
            ss1_time = 0;
        }
    }

    if (pin_ss2) {
        if (ss2_stall < VSS_STALL_TIMEOUT) {
            if (flagbyte8 & FLAGBYTE8_SAMPLE_SS2) {
                flagbyte8 &= ~FLAGBYTE8_SAMPLE_SS2;
                if (ss2_time > 0) {

                    tmp_speed = (unsigned int) (ss2_coeff / ss2_time);
                    __asm__ __volatile__("ldd    %1\n"
                                         "subd   %3\n"
                                         "tfr    d,y\n"
                                         "clra\n"
                                         "ldab    %2\n" // it is a uchar
                                         "emuls\n"
                                         "ldx     #100\n"
                                         "edivs\n"
                                         "addy    %3\n"
                                        :"=y"(outpc.ss2)
                                         :"m"(tmp_speed), "m"(ram4.ss2LF), "m"(outpc.ss2)
                                         :"d", "x");
                }
            }
        } else {
            ss2_stall = VSS_STALL_TIMEOUT + 1;  // keep it from rolling over
            outpc.ss2 = 0;
            ss2_time = 0;
        }
    }

    if (pin_vssout) {
        if (outpc.vss1 == 0) {
            vssout_match = 0; // don't flip output
        } else {
            if ((ram4.vssout_opt & 0xc0) == 0x00) {
                /* time */
                vssout_match = ram4.vssout_scale / outpc.vss1;
            } else if ((ram4.vssout_opt & 0xc0) == 0x40) {
                /* pulses per mile
                factor is 16093.44 * 20000 / 2 
                (miles to metresX10, 20000 ticks per second, but need half a period) */
                vssout_match = (1609344000L / ram4.vssout_scale) / outpc.vss1;
            } else if ((ram4.vssout_opt & 0xc0) == 0x80) {
                /* pulses per km
                factor is 10000.00 * 20000 / 2 
                (miles to metresX10, 20000 ticks per second, but need half a period) */
                vssout_match = (1000000000L / ram4.vssout_scale) / outpc.vss1;
            } else {
                vssout_match = 0;
            }
        }
    } else {
        vssout_match = 0;
    }
}

void nitrous()
{
    /**************************************************************************
     **
     ** Nitrous
     **
     **************************************************************************/
    if (ram4.N2Oopt & 0x04) {   // are we using nitrous at all
        if ((outpc.status2 & (STATUS2_LAUNCH | STATUS3_3STEP)) || (flagbyte10 & FLAGBYTE10_TC_N2O)
            || (outpc.status6 & STATUS6_AFRSHUT) ) { // if in launch or 3step or TC or AFRsafety cut nitrous
            outpc.status2 &= ~STATUS2_NITROUS1;
            outpc.status2 &= ~STATUS2_NITROUS2;
            outpc.nitrous1_duty = 0;
            outpc.nitrous2_duty = 0;
            outpc.n2o_addfuel = 0;
            outpc.n2o_retard = 0;
            outpc.nitrous_timer_out = 0;
            goto NITROUS_OUTPUTS;
        }
        // is enable input on
        //check if the chosen pin is low
        if (((*port_n2oin & pin_n2oin) == pin_match_n2oin) && pin_n2oin)
            goto DO_NITROUS;

        // no valid input so turn it off
        outpc.status2 &= ~STATUS2_NITROUS1;
        outpc.status2 &= ~STATUS2_NITROUS2;
        outpc.nitrous1_duty = 0;
        outpc.nitrous2_duty = 0;
        outpc.n2o_addfuel = 0;
        outpc.n2o_retard = 0;
        outpc.nitrous_timer_out = 0;
        goto NITROUS_OUTPUTS;

      DO_NITROUS:
        // selection logic
        if ((outpc.rpm > ram4.N2ORpm) && (outpc.rpm < ram4.N2ORpmMax)
            && (outpc.tps > ram4.N2OTps) && (outpc.clt > ram4.N2OClt)
            && (n2o_act_timer == 0)) {
            if (!(outpc.status2 & STATUS2_NITROUS1)) {
                n2o2_act_timer = ram4.N2O2delay;        // if first change then set delay to stage 2
                nitrous_timer = 0;      // progressive up counter
            }
            outpc.status2 |= STATUS2_NITROUS1;
            if (ram4.N2Oopt2 & 1) {     // progressive
                if (ram4.N2Oopt2 & 2) { // time based
                    outpc.n2o_addfuel =
                        intrp_1ditable(nitrous_timer, 6,
                                       (int *) ram_window.pg11.n2o1_time,
                                       1, (int *) ram_window.pg11.n2o1_pw,
                                       11);
                    outpc.n2o_retard =
                        intrp_1ditable(nitrous_timer, 6,
                                       (int *) ram_window.pg11.n2o1_time,
                                       0,
                                       (int *) ram_window.pg11.n2o1_retard,
                                       11);
                    outpc.nitrous1_duty =
                        intrp_1dctable(nitrous_timer, 6,
                                       (int *) ram_window.pg11.n2o1_time,
                                       0, ram_window.pg11.n2o1_duty, 11);
                    outpc.nitrous_timer_out = nitrous_timer;
                } else {
                    outpc.n2o_addfuel =
                        intrp_1ditable(outpc.rpm, 6,
                                       (int *) ram_window.pg11.n2o1_rpm, 1,
                                       (int *) ram_window.pg11.n2o1_pw,
                                       11);
                    outpc.n2o_retard =
                        intrp_1ditable(outpc.rpm, 6,
                                       (int *) ram_window.pg11.n2o1_rpm, 0,
                                       (int *) ram_window.pg11.n2o1_retard,
                                       11);
                    outpc.nitrous1_duty =
                        intrp_1dctable(outpc.rpm, 6,
                                       (int *) ram_window.pg11.n2o1_rpm, 0,
                                       ram_window.pg11.n2o1_duty, 11);
                }
            } else {
                outpc.n2o_addfuel =
                    twoptlookup(outpc.rpm, ram4.N2ORpm, ram4.N2ORpmMax,
                                ram4.N2OPWLo, ram4.N2OPWHi);
                outpc.n2o_retard = ram4.N2OAngle;       // retard timing
            }

            // if stage 1 on, then check for stage 2
            if ((ram4.N2Oopt & 0x08)
                && ((n2o2_act_timer == 0)
                    || ((ram4.N2Oopt2 & 1) && (ram4.N2Oopt2 & 2)))
                && ((ram4.N2Oopt2 & 1)
                    || ((outpc.rpm > ram4.N2O2Rpm)
                        && (outpc.rpm < ram4.N2O2RpmMax)))) {

                // if stage2 and (time expired OR progressive time-based) and (progressive or within rpm window)  
                outpc.status2 |= STATUS2_NITROUS2;

                if (ram4.N2Oopt2 & 1) { // progressive
                    if (ram4.N2Oopt2 & 2) {     // time based
                        outpc.n2o_addfuel +=
                            intrp_1ditable(nitrous_timer, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_time, 1,
                                           (int *) ram_window.pg11.n2o2_pw,
                                           11);
                        outpc.n2o_retard +=
                            intrp_1ditable(nitrous_timer, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_time, 0,
                                           (int *) ram_window.
                                           pg11.n2o2_retard, 11);
                        outpc.nitrous2_duty =
                            intrp_1dctable(nitrous_timer, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_time, 0,
                                           ram_window.pg11.n2o2_duty, 11);
                    } else {
                        outpc.n2o_addfuel +=
                            intrp_1ditable(outpc.rpm, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_rpm, 1,
                                           (int *) ram_window.pg11.n2o2_pw,
                                           11);
                        outpc.n2o_retard +=
                            intrp_1ditable(outpc.rpm, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_rpm, 0,
                                           (int *) ram_window.
                                           pg11.n2o2_retard, 11);
                        outpc.nitrous2_duty =
                            intrp_1dctable(outpc.rpm, 6,
                                           (int *) ram_window.
                                           pg11.n2o2_rpm, 0,
                                           ram_window.pg11.n2o2_duty, 11);
                    }
                } else {
                    outpc.n2o_addfuel +=
                        twoptlookup(outpc.rpm, ram4.N2O2Rpm,
                                    ram4.N2O2RpmMax, ram4.N2O2PWLo,
                                    ram4.N2O2PWHi);
                    outpc.n2o_retard += ram4.N2O2Angle; // retard timing
                }
            } else {
                outpc.status2 &= ~STATUS2_NITROUS2;     // if stage1 is off then so is stage2
                outpc.nitrous2_duty = 0;
            }

        } else {
            outpc.status2 &= ~STATUS2_NITROUS1;
            outpc.status2 &= ~STATUS2_NITROUS2; // if stage1 is off then so is stage2
            outpc.nitrous1_duty = 0;
            outpc.nitrous2_duty = 0;
            outpc.n2o_addfuel = 0;
            outpc.n2o_retard = 0;
            outpc.nitrous_timer_out = 0;
        }

      NITROUS_OUTPUTS:
        // here we flip the bits for chosen output pins

        if (!(ram4.N2Oopt2 & 1)) {      // not progressive
            // stage 1
            if (outpc.status2 & STATUS2_NITROUS1) {
                SSEM0SEI;
                *port_n2o1n |= pin_n2o1n;
                *port_n2o1f |= pin_n2o1f;
                CSEM0CLI;
            } else {
                SSEM0SEI;
                *port_n2o1n &= ~pin_n2o1n;
                *port_n2o1f &= ~pin_n2o1f;
                CSEM0CLI;
            }

            if (ram4.N2Oopt & 0x08) {
                // stage 2
                if (outpc.status2 & STATUS2_NITROUS2) {
                    SSEM0SEI;
                    *port_n2o2n |= pin_n2o2n;
                    *port_n2o2f |= pin_n2o2f;
                    CSEM0CLI;
                } else {
                    SSEM0SEI;
                    *port_n2o2n &= ~pin_n2o2n;
                    *port_n2o2f &= ~pin_n2o2f;
                    CSEM0CLI;
                }
            }
        }
    }
}

int median3pt(int array[3], int value)
{
    int val0, val1, val2, tmp;
    /* insert new value into master array and local copy */
    val2 = array[2] = array[1];
    val1 = array[1] = array[0];
    val0 = array[0] = value;
    /* sort */
    if (val0 > val1) {
        tmp = val0;
        val0 = val1;
        val1 = tmp;
    }
    if (val1 > val2) {
        tmp = val1;
        val1 = val2;
        val2 = tmp;
    }
    if (val0 > val1) {
        tmp = val0;
        val0 = val1;
        val1 = tmp;
    }

    /* median */
    return val1;
}

void sample_map_tps(char *localflags)
{
    unsigned int utmp1;
    int tmp1;
    unsigned long map_sum_local;
    unsigned int map_cnt_local;
    int map_local, maf_local, map_local2;

    RPAGE = RPAGE_VARS1;

    if (((flagbyte3 & flagbyte3_samplemap) && !(ram4.mapsample_opt & 0x4)) || 
        ((flagbyte15 & FLAGBYTE15_MAP_AVG_RDY) && (ram4.mapsample_opt & 0x4))) {
        unsigned int tmp_mapsample_time;

        *localflags |= LOCALFLAGS_RUNFUEL;

        DISABLE_INTERRUPTS;
        map_sum_local = map_sum;
        map_cnt_local = map_cnt;
        map_local = map_temp;
        flagbyte3 &= ~flagbyte3_samplemap;
        flagbyte15 &= ~FLAGBYTE15_MAP_AVG_RDY;
        ENABLE_INTERRUPTS;
        map_local2 = (int)(map_sum_local / (long)map_cnt_local);

        /* Using average over period or if first time in window.
        (First time into window code can return a zero) */
        if ((ram4.mapsample_opt & 0x4) || (map_local == 0)) {
            map_local = map_local2;
        }

        DISABLE_INTERRUPTS;
        maf_local = maf_temp;
        ENABLE_INTERRUPTS;

        old_map = outpc.map;

        if (ram4.mapport & 0x20) {
            /* For frequency sensor, convert period to frequency to 0-1023 */
            unsigned int f;

            /* calc freq x 5  (0.2Hz units) */
            if (flagbyte12 & FLAGBYTE12_MAP_FSLOW) {
                f =  1250000UL / map_local; /* in slow mode time is reported in 4us units */
            } else {
                f = 5000000UL / map_local;
            }

            if (f < ram5.map_freq0) {
                map_local = 0;
            } else if (f > ram5.map_freq1) {
                map_local = 1023;
            } else {
                map_local = (int)((1023UL * (f - ram5.map_freq0)) / (ram5.map_freq1 - ram5.map_freq0));
            }
        }

        __asm__ __volatile__("ldd    %1\n"
                             "subd   %2\n"
                             "ldy    %3\n"
                             "emul\n"
                             "ldx    #1023\n"
                             "ediv\n"
                             "addy   %2\n"
                             :"=y"(utmp1)
                             :"m"(ram4.mapmax), "m"(ram4.map0), "m"(map_local)
                             :"x");

        if (stat_map && (ram5.cel_action1 & 0x01) && (ram5.cel_action1 & 0x80)) {
            /* broken MAP and using fallback A-N lookup table */
            utmp1 = intrp_2ditable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg25.amap_rpm[0],
    	      &ram_window.pg25.amap_tps[0], &ram_window.pg25.alpha_map_table[0][0], 25); // will alter RPAGE
        }

        /* new MAP value held in tmp1 at this point */

        tmp1 = median3pt(&map_data[0], tmp1);  // <-- wrong var!

        __asm__ __volatile__("ldd    %1\n"
                             "subd   %3\n"
                             "tfr    d,y\n"
                             "clra\n"
                             "ldab    %2\n" // it is a uchar
                             "emuls\n"
                             "ldx     #100\n"
                             "edivs\n"
                             "addy    %3\n"
                             :"=y"(outpc.map)
                             :"m"(utmp1), "m"(ram4.mapLF), "m"(outpc.map)
                             :"x");

        /* Sample MAF */

        if ((flagbyte8 & FLAGBYTE8_USE_MAF) && (!burnstat) && (!ltt_fl_state)) { /* but not while eflash in use */

            if (ram4.MAFOption & 0x20) {
                /* For frequency sensor, convert period to frequency to 0-1023 */
                unsigned int f;

                /* calc freq x 4 - note different units for MAF (0.25Hz) */
                if (flagbyte12 & FLAGBYTE12_MAF_FSLOW) {
                    f =  1000000UL / maf_local; /* in slow mode time is reported in 4us units */
                } else {
                    f = 4000000UL / maf_local;
                }

                if (f < ram5.maf_freq0) {
                    maf_local = 0;
                    outpc.maf_volts = 0;
                    stat_maf |= 1;
                } else if (f > ram5.maf_freq1) {
                    maf_local = 1023;
                    outpc.maf_volts = 5000;
                    stat_maf |= 1;
                } else {
                    maf_local = (int)((1023UL * (f - ram5.maf_freq0)) / (ram5.maf_freq1 - ram5.maf_freq0));
                    /* need to calculate fully or rounding gets the better of us */
                    outpc.maf_volts = (int)((5000UL * (f - ram5.maf_freq0)) / (ram5.maf_freq1 - ram5.maf_freq0));
                    stat_maf &= ~1; // only valid while no other checks are in place
                }
            } else {
                outpc.maf_volts = (5000L * maf_local) / 1023; // 0.001V
            }

            if (ram4.feature7 & 0x04) { /* old calibration table */
                GPAGE = 0x10;
                __asm__ __volatile__("aslx\n"
                                     "addx   #0x5400\n"    // maffactor_table address
                                     "gldy   0,x\n"
                                     :"=y"(utmp1)
                                     :"x"(maf_local));
            } else {
                utmp1 = intrp_1ditable(outpc.maf_volts, 64, (int *)ram_window.pg25.mafv, 0, (unsigned int *)
                    ram_window.pg25.mafflow, 25);
            }

            utmp1 = median3pt(&maf_data[0], utmp1);

            __asm__ __volatile__("ldd    %1\n"
                    "subd   %3\n"
                    "tfr    d,y\n"
                    "clra\n"
                    "ldab    %2\n" // it is a uchar
                    "emuls\n"
                    "ldx     #100\n"
                    "edivs\n"
                    "addy    %3\n"
                    :"=y"(mafraw)
                    :"m"(utmp1), "m"(ram4.mafLF), "m"(mafraw)
                    :"x");

            if (ram4.feature7 & 0x04) { /* old correction table */
                /* this is the new code with the unsigned X axis lookup */
                utmp1 = intrp_1dctableu(mafraw, 12, (int *)ram_window.pg11.MAFFlow, 0, (unsigned char *)
                    ram_window.pg11.MAFCor, 11);
                outpc.maf = ((unsigned long)mafraw * utmp1) / 100;
            } else {
                outpc.maf = mafraw;
            }

            if (outpc.rpm > 5) { // ensure that 1,2 rpm don't give a bogus huge MAFload value
                unsigned long scaled_maf = outpc.maf * ((ram4.maf_range & 0x03) + 1);
                outpc.mafload = (int)(((unsigned long)(MAFCoef / outpc.aircor) * scaled_maf) / outpc.rpm);
                if (outpc.mafload < 100) {
                    outpc.mafload = 100;
                }
                mafload_no_air = (int)(((unsigned long)(MAFCoef / 1000) * scaled_maf) / outpc.rpm);
                if (mafload_no_air < 100) {
                    mafload_no_air = 100;
                }
            } else {
                outpc.mafload = 1000;
                mafload_no_air = 1000;
            }
        }

        // mapdot
        tmp_mapsample_time = (unsigned int)lmms - mapsample_time;
        if (tmp_mapsample_time > 78) { // minimum 10ms
            DISABLE_INTERRUPTS;
            mapsample_time = (unsigned int)lmms;
            ENABLE_INTERRUPTS;
            if (flagbyte8 & FLAGBYTE8_USE_MAF_ONLY) {
                /* not using real MAP anywhere, use mafload for mapdot */
                map_local = outpc.mafload;
            } else {
                map_local = outpc.map;
            }

            RPAGE = RPAGE_VARS1;

            /* sliding windows */
            /* mapdot calc - sliding window - variable dataset and max sample rate */
            v1.mapdot_data[0][0] = (unsigned int)lmms; /* store actual 16 bit time to first pt in array */
            v1.mapdot_data[0][1] = map_local; /* store map - note this is _after_ the lags*/

            /* minimum 10ms between samples */
            int mapi, samples, a, b;
            long mapdot_sumx, mapdot_sumx2, mapdot_sumy, mapdot_sumxy;
            long toprow, btmrow;
            unsigned long mapdot_x; // was uint

            mapdot_sumx = 0;
            mapdot_sumx2 = 0;
            mapdot_sumy = 0;
            mapdot_sumxy = 0;

            /* decide how many samples to use based on rate from last two points
                miniumum is three, max is MAPDOT_N (20)
            */
            a = v1.mapdot_data[0][1] - v1.mapdot_data[1][1];
            b = v1.mapdot_data[0][0] - v1.mapdot_data[1][0];
            a = (int)((7812L * a) / b);
            a = long_abs(a);

            if (a > 2500) { // 250%/s
                samples = 3;
            } else if (a > 1000) {
                samples = 4;
            } else {
                samples = 5;
            }

    /* note that this only uses first and last, so doesn't fully benefit from the over-sampling */
            mapdot_x = 0; // not used
            toprow = (int)v1.mapdot_data[0][1] - (int)v1.mapdot_data[samples - 1][1];
            btmrow = v1.mapdot_data[0][0] - v1.mapdot_data[samples - 1][0];    

            toprow = (toprow * 781) / btmrow;
            if (toprow > 32767) {
                toprow = 32767;
            } else if (toprow < -32767) {
                toprow = -32767;
            }

            toprow = 50L * (toprow - outpc.mapdot); /* now 50% lag */
            if ((toprow > 0) && (toprow < 100)) {
                toprow = 100;
            } else if ((toprow < 0) && (toprow > -100)) {
                toprow = -100;
            }
            outpc.mapdot += (int)(toprow / 100);

            /* shuffle data forwards by one */
            for (mapi = MAPDOT_N - 1; mapi > 0 ; mapi--) {
                v1.mapdot_data[mapi][0] = v1.mapdot_data[mapi - 1][0];
                v1.mapdot_data[mapi][1] = v1.mapdot_data[mapi - 1][1];
            }

            last_map = map_local;
        }
    }

    if (flagbyte0 & FLAGBYTE0_50MS) { // actually every 10ms
        unsigned int tmp_tpssample_time, raw_tps;
        unsigned int raw_tps_accum;
        DISABLE_INTERRUPTS;
        flagbyte0 &= ~FLAGBYTE0_50MS;   // clear flag
        tmp_tpssample_time = (unsigned int)lmms - tpssample_time;
        tpssample_time = (unsigned int)lmms;
        ENABLE_INTERRUPTS;

        if (stat_tps) {
            /* broken TPS*/
            outpc.tps = 0;
            outpc.tpsdot = 0;
            outpc.tpsadc = ATD0DR3;
        } else {
            /* normal calculations */

            /* raw_tps = average of 8 pt ring buffer tps_ring[] */
            DISABLE_INTERRUPTS;
            raw_tps_accum = tps_ring[0];
            raw_tps_accum += tps_ring[1];
            raw_tps_accum += tps_ring[2];
            raw_tps_accum += tps_ring[3];
            raw_tps_accum += tps_ring[4];
            raw_tps_accum += tps_ring[5];
            raw_tps_accum += tps_ring[6];
            raw_tps_accum += tps_ring[7];
            ENABLE_INTERRUPTS;

            raw_tps = raw_tps_accum >> 3;

            // get map, tps
            // map, tps lag(IIR) filters

            outpc.tpsadc = raw_tps; // send back for TPS calib

            __asm__ __volatile__("ldd %3\n"
                                 "subd %2\n"
                                 "tfr  d,y\n"
                                 "ldd  %1\n"
                                 "subd %2\n"
                                 "pshd\n"
                                 "ldd #1000\n"
                                 "emuls\n" "pulx\n" "edivs\n":"=y"(tmp1)
                                 :"m"(ram4.tpsmax), "m"(tps0_auto),
                                 "m"(raw_tps)
                                 :"d", "x");

            tmp1 = median3pt(&tps_data[0], tmp1);

            __asm__ __volatile__("ldd    %1\n" "subd   %3\n" "tfr    d,y\n" "clra\n" "ldab    %2\n" // it is a uchar
                                 "emuls\n"
                                 "ldx     #100\n"
                                 "edivs\n" "addy    %3\n":"=y"(outpc.tps)
                                 :"m"(tmp1), "m"(ram4.tpsLF), "m"(outpc.tps)
                                 :"d", "x");

            if (ram4.tps0 != tps0_orig) {
                // if user sends a new calibration, cancel out any auto-zero
                tps0_auto = tps0_orig = ram4.tps0;
            }

            /* ----------- Experimental ----------- */
            if (ram4.feature7 & 0x10) {
                /* use TPSwot curve to ignore TPS > WOT vs. RPM */
                int tps_max;
                tps_max = intrp_1ditable(outpc.rpm, 6,
                         (unsigned int *) ram_window.pg25.tpswot_rpm, 1,
                         (int *) ram_window.pg25.tpswot_tps, 25);
                if (tmp1 > tps_max) {
                    tmp1 = tps_max;
                }

                /* scale tmp1 as 0-tps_max */
                tmp1 = (tmp1 * 1000L) / tps_max;

            }
            /* ----------- end experimental ----------- */

            RPAGE = RPAGE_VARS1;
            /* sliding windows */

            /* tpsdot calc - sliding window - variable dataset and max sample rate */
            v1.tpsdot_data[0][0] = (unsigned int)lmms; /* store actual 16 bit time to first pt in array */
            v1.tpsdot_data[0][1] = tmp1; /* w/o extra bits */

            /* minimum 10ms between samples */
            if ((v1.tpsdot_data[0][0] - v1.tpsdot_data[1][0]) > 78) {
                int tpsi, samples, a, b;
                long toprow, btmrow;

                /* decide how many samples to use based on rate from last two points
                    miniumum is three, max is TPSDOT_N (20)
                */
                a = v1.tpsdot_data[0][1] - v1.tpsdot_data[1][1];
                b = v1.tpsdot_data[0][0] - v1.tpsdot_data[1][0];
                a = (int)((7812L * a) / b);
                a = long_abs(a);

                if (a > 2500) { // 250%/s
                    samples = 3;
                } else if (a > 1000) {
                    samples = 4;
                } else {
                    samples = 5;
                }

    /* note that this only uses first and last, so doesn't fully benefit from the over-sampling */
                toprow = (int)v1.tpsdot_data[0][1] - (int)v1.tpsdot_data[samples - 1][1];
                btmrow = v1.tpsdot_data[0][0] - v1.tpsdot_data[samples - 1][0];    

                btmrow = btmrow / 10; /* allows top row to be 10x less to reduce overflow. */
                toprow = (toprow * 781) / btmrow;
                if (toprow > 32767) {
                    toprow = 32767;
                } else if (toprow < -32767) {
                    toprow = -32767;
                }

                toprow = 50L * (toprow - outpc.tpsdot); /* now 50% lag */
                if ((toprow > 0) && (toprow < 100)) {
                    toprow = 100;
                } else if ((toprow < 0) && (toprow > -100)) {
                    toprow = -100;
                }

                outpc.tpsdot += (int)(toprow / 100);

                /* shuffle data forwards by one */
                for (tpsi = TPSDOT_N - 1; tpsi > 0 ; tpsi--) {
                    v1.tpsdot_data[tpsi][0] = v1.tpsdot_data[tpsi - 1][0];
                    v1.tpsdot_data[tpsi][1] = v1.tpsdot_data[tpsi - 1][1];
                }
            }

            /* end sliding window */
        }
        last_tps = outpc.tps;

        ego_get_sample();
    }
}

int calc_ITB_load(int percentbaro)
{
    int tmp3, tmp4, tmp5; 

    tmp3 = intrp_1ditable(outpc.rpm, 12,
                         (unsigned int *) ram_window.pg19.ITB_load_rpms, 1,
                         (int *) ram_window.pg19.ITB_load_loadvals, 19);

    if ((outpc.tps <= ram5.ITB_load_idletpsthresh) || 
        (percentbaro < ram5.ITB_load_mappoint)) {
        /* Make MAP fit in to 0-tmp3 % load... so if user selects 60%,
         * 0-90kPa would be 0-60% load, and the throttle position above
         * that point to 100% throttle will be 60% load to 100% load
         */
        tmp3 = (((long) percentbaro * tmp3) / ram5.ITB_load_mappoint);
    } else {
        tmp4 = intrp_1ditable(outpc.rpm, 12,
                             (unsigned int *) ram_window.pg19.ITB_load_rpms, 1,
                             (int *) ram_window.pg19.ITB_load_switchpoints, 19);
        /* Make TPS fit in tmp3 - 100% */
        if (outpc.tps >= tmp4) {
            /* This is the amt of load TPS has to fit into */
            tmp5 = 1000 - tmp3;

            /* This is the actual percent above the
             * switchpoint TPS is at */
            tmp4 = ((long)(outpc.tps - tmp4) * 1000) / (1000 - tmp4);

            /* Now scale that into what's left above the 90 % baro 
             * point
             */
            tmp3 = (((long) tmp4 * tmp5) / 1000) + tmp3;
        }
        /* IF TPS hasn't gone above the setpoint, load should
         * stay at the user-set setpoint */
    }

    /* Make 10 the lowest possible load */

    if (tmp3 < 100) {
        tmp3 = 100;
    }

    if (tmp3 > 1000) {
        tmp3 = 1000;
    }

    return tmp3;
}

void calc_baro_mat_load(void)
{
    int tmp1, tmp2, tmp3 = 0;
    unsigned char uctmp1, uctmp2, uctmp3, uctmp4, uctmp5, uctmp6;

    if (ram4.BaroOption == 0) {
        outpc.barocor = 1000;
    } else {                    // barometric correction (%)
        outpc.barocor = barocor_eq(outpc.baro) + intrp_1ditable(outpc.baro, NO_BARS,
                        (int *)ram_window.pg8.BaroVals, 1,
                        (unsigned int *)ram_window.pg8.BaroCorDel, 8);
    }

    if ((ram4.feature3 & 0x40) && (outpc.engine & ENGINE_ASE)) {
        outpc.aircor = 1000;
        outpc.airtemp = outpc.mat;  // use raw MAT in this mode
    } else {
        unsigned int flow; // beware of signed ness of function if flow > 32767
        unsigned long flowtmp;

        flowtmp = (unsigned long)outpc.fuelload * outpc.rpm;
        flowtmp /= 1000; // 10 for fuelload. 100 for 0.01 final units
        flowtmp *= outpc.vecurr1;

        flow = (unsigned int) (flowtmp / 1000UL); // normalise VE

        // use MAT/CLT scaling for a temporary mat value
        tmp1 = intrp_1ditable(flow, 6, ram_window.pg8.matclt_flow, 0, ram_window.pg8.matclt_pct, 8);    // returns %clt * 100

        outpc.airtemp = (int) ((((long) outpc.mat * (10000 - tmp1)) + ((long) outpc.clt * tmp1)) / 10000L);  // 'corrected' MAT

        // airdensity correction (%)
        outpc.aircor = intrp_1ditable(outpc.airtemp, NO_MATS, (int *)ram_window.pg8.MatVals, 1,
                        (unsigned int *)ram_window.pg8.AirCorDel,8);
        // ...  = aircor_eq(tmp2) + ... removed
    }

    tmp1 = outpc.map;

    serial();
    uctmp1 = ram4.FuelAlgorithm & 0xF;
    uctmp2 = ram4.FuelAlgorithm & 0xF0;
    uctmp3 = ram4.IgnAlgorithm & 0xF;
    uctmp4 = ram4.IgnAlgorithm & 0xF0;
    uctmp5 = ram4.extra_load_opts & 0xF;
    uctmp6 = ram4.extra_load_opts & 0xF0;
    if ((uctmp1 == 2) || (uctmp2 == 0x20) ||
        (uctmp1 == 6) || (uctmp2 == 0x60) ||
        (uctmp3 == 2) || (uctmp4 == 0x20) ||
        (uctmp3 == 6) || (uctmp4 == 0x60) ||
        (uctmp5 == 2) || (uctmp5 == 6) ||
        (uctmp6 == 0x20) || (uctmp6 == 0x60)) {

        tmp2 = (int) (((long) tmp1 * 1000) / outpc.baro);
    } else {
        tmp2 = 1000;
    }

    if ((uctmp1 == 6) || (uctmp2 == 0x60) ||
        (uctmp3 == 6) || (uctmp4 == 0x60) ||
        (uctmp5 == 6) || (uctmp6 == 0x60)) {
        tmp3 = calc_ITB_load(tmp2);
    }

    /* Fuel Load 1 */
    if (uctmp1 == 1) {
        outpc.fuelload = tmp1;  // kpa x 10, speed density
    } else if (uctmp1 == 2) {
        outpc.fuelload = tmp2; // % baro
    } else if (uctmp1 == 3) {
        outpc.fuelload = outpc.tps; // alpha-n
    } else if (uctmp1 == 4 || uctmp1 == 5) { // maf or mafload
        outpc.fuelload = outpc.mafload;
    } else if (uctmp1 == 6) { // ITB
        outpc.fuelload = tmp3;
    } else {
        /* somehow something is set wrong, just use map */
        outpc.fuelload = outpc.map;
    }

    if (uctmp2 == 0x10) {
        outpc.fuelload2 = tmp1;
    } else if (uctmp2 == 0x20) {
        outpc.fuelload2 = tmp2;
    } else if (uctmp2 == 0x30) {
        outpc.fuelload2 = outpc.tps;
    } else if (uctmp2 == 0x40 || uctmp2 == 0x50) {
        outpc.fuelload2 = outpc.mafload;
    } else if (uctmp2 == 0x60) {
        outpc.fuelload2 = tmp3;
    }

    /* Ign Load 1 */
    if (uctmp3 == 1) {
        outpc.ignload = outpc.map;
    } else if (uctmp3 == 2) {
        outpc.ignload = tmp2;
    } else if (uctmp3 == 3) {
        outpc.ignload = outpc.tps;
    } else if (uctmp3 == 5) {
        outpc.ignload = outpc.mafload;
    } else if (uctmp3 == 6) {
        outpc.ignload = tmp3;
    } else {
        outpc.ignload = outpc.fuelload;
    }

    if (uctmp4 == 0x10) {
        outpc.ignload2 = outpc.map;
    } else if (uctmp4 == 0x20) {
        outpc.ignload2 = tmp2;
    } else if (uctmp4 == 0x30) {
        outpc.ignload2 = outpc.tps;
    } else if (uctmp4 == 0x50) {
        outpc.ignload2 = outpc.mafload;
    } else if (uctmp4 == 0x60) {
        outpc.ignload2 = tmp3;
    }

    /* AFR load */
    if (uctmp5 == 1) {
        outpc.afrload = outpc.map;
    } else if (uctmp5 == 2) {
        outpc.afrload = tmp2;
    } else if (uctmp5 == 3) {
        outpc.afrload = outpc.tps;
    } else if (uctmp5 == 5) {
        outpc.afrload = outpc.mafload;
    } else if (uctmp5 == 6) {
        outpc.afrload = tmp3;
    } else {
        outpc.afrload = outpc.fuelload;
    }

    /* EAEload */
    if (uctmp6 == 0x10) {
        outpc.eaeload = outpc.map;
    } else if (uctmp6 == 0x20) {
        outpc.eaeload = tmp2;
    } else if (uctmp6 == 0x30) {
        outpc.eaeload = outpc.tps;
    } else if (uctmp6 == 0x50) {
        outpc.eaeload = outpc.mafload;
    } else if (uctmp6 == 0x60) {
        outpc.eaeload = tmp3;
    } else {
        outpc.eaeload = outpc.fuelload;
    }
}

void water_inj()
{
    /**************************************************************************
     **
     ** Water injection
     ** Can:
     ** Turn pump on/off at set points
     ** read from level/pressure or whatever switch to shutdown on system failure
     ** use slow-speed (duty based) or high-speed (injector type) valves
     **
     ** Uses various set points and a table
     ** in high-speed mode, valve runs as fast as tacho output
     **************************************************************************/
    if (ram4.water_freq & 0x10) {    // system enabled?
        if ((outpc.tps > ram4.water_tps)
            && (outpc.rpm > ram4.water_rpm)
            && (outpc.map > ram4.water_map)
            && (outpc.mat > ram4.water_mat)) {  // system active?

            if (pin_wipump) {  // using pump output
                SSEM0SEI;
                *port_wipump |= pin_wipump;
                CSEM0CLI;
            }

            if ((ram4.maxafr_opt1 & 1)
                && pin_wiin && ((*port_wiin & pin_wiin) != pin_match_wiin)) {
                // safety input NOT on, e.g. low water level etc. (Might want to change polarity?)
                if (maxafr_stat == 0) {
                    maxafr_stat = 1;    // mainloop code reads the stat and cut appropriately
                    maxafr_timer = 1;   // start counting
                }
            }

            if (ram4.water_freq & 0x60) {    // using valve output
                outpc.water_duty =
                    intrp_2dctable(outpc.rpm, outpc.map, 8, 4,
                                   &ram_window.pg10.waterinj_rpm[0],
                                   &ram_window.pg10.waterinj_map[0],
                                   &ram_window.pg10.waterinj_duty[0][0], 0,
                                   10);
                if ((ram4.water_freq & 0x60) == 0x20) {      // using high speed output (see isr_pit.s)
                    unsigned long dtpred_local;
                    DISABLE_INTERRUPTS;
                    dtpred_local = dtpred;
                    ENABLE_INTERRUPTS;
                    if (outpc.water_duty > 99) {
                        water_pw = 10 + (dtpred_local / 50);    // make sure full on (bodge)
                    } else {
                        water_pw = 1 + (unsigned int) ((unsigned long) outpc.water_duty * dtpred_local / 5000); // 100 for % and 50 to convert to 50us units
                    }
                } else {    // low speed (see isr_rtc.s)
                    water_pw = 0;
                    water_pw_cnt = 0;
                }
            }


        } else {                // system inactive
            if (pin_wipump) {  // using pump output
                SSEM0SEI;
                *port_wipump &= ~pin_wipump;
                CSEM0CLI;
            }

            outpc.water_duty = 0;
            water_pw = 0;
            water_pw_cnt = 0;

            if ((ram4.water_freq & 0x60) && pin_wivalve) {    // using valve output
                SSEM0SEI;
                *port_wivalve &= ~pin_wivalve;
                CSEM0CLI;
            }
        }
    }
}

void do_launch(long *lsum)
{
    // 3 step
    if (pin_3step && (!(*port_3step & pin_3step))) {
        // feature on and switch active (low)
        outpc.status3 |= STATUS3_3STEPIN;
        if (outpc.rpm > ram4.launch_sft_lim3) {
            *lsum = (long) ram4.launch_sft_deg3;
            outpc.status3 |= STATUS3_3STEP;
        } else {
            outpc.status3 &= ~STATUS3_3STEP;
        }

        if (outpc.rpm > ram4.launch_hrd_lim3) {
            if (ram4.launchlimopt & 1) {
                spk_cutx = ram4.launchcutx;
                spk_cuty = ram4.launchcuty;
                flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
            }
            if (ram4.launchlimopt & 2) {
                tmp_pw1 = 0;
                tmp_pw2 = 0;
            }
        }
    } else {
        outpc.status3 &= ~(STATUS3_3STEPIN | STATUS3_3STEP);
    }
    // do launch in here too

    outpc.status3 &= ~STATUS3_LAUNCHON; // clear bit first
    if (ram4.launch_opt & 0x40) {
        /* pin status for UI only */
        if ((*port_launch & pin_launch) == pin_match_launch) {
            outpc.status2 |= STATUS2_LAUNCHIN;
        } else {
            outpc.status2 &= ~STATUS2_LAUNCHIN;
        }

        if (outpc.tps > ram4.launch_tps) {
            //check if the chosen pin is low
            if (((*port_launch & pin_launch) == pin_match_launch)
                && (!(((ram4.launch_opt & 0xc0) == 0x40) && ((ram4.vss_opt & 0x0f) || (ram4.vss1_an & 0x1f))
                    && (outpc.vss1 > ram4.launch_maxvss)))) {
                // pin is active but VSS low enough (when enabled) and launch mode (not flat shift)
                goto DO_LAUNCH;
            }
            // nothing active
            if (outpc.status2 & STATUS2_LAUNCH) {   // if we were on, then set nitrous delay timer
                outpc.launch_retard = 0;
                if ((ram4.launch_opt & 0x80) && (outpc.status2 & STATUS2_FLATSHIFT)) {
                    n2o_act_timer = ram4.N2Odel_flat;       //flat shift timer
                } else {
                    n2o_act_timer = ram4.N2Odel_launch;     //launch timer
                    /* Note that this has moved, time is from end of LAUNCH only, not flat-shift */
                    launch_timer = 0;        // boost/ retard launch delay timer
                }
                if ((outpc.vss1 <= ram5.tc_minvss) && ((outpc.status2 & STATUS2_FLATSHIFT) == 0)) {
                    // only if going slow enough and weren't in flatshift
                    perfect_timer = 1; // set it rolling (strictly off by one)
                }
            }

            outpc.status2 &= ~(STATUS2_LAUNCH | STATUS2_FLATSHIFT); // turn off both
            goto NO_LAUNCH;

          DO_LAUNCH:
            launch_timer = 0x7fff;   // boost launch delay timer
            perfect_timer = 0;
            if (!(outpc.status2 & STATUS2_LAUNCH)) {
                outpc.status2 |= STATUS2_LAUNCH;    // turn it on (code below shuts down nitrous immediately)
                if ((ram4.launch_opt & 0x80) && (outpc.rpm > ram4.flats_arm)) {
                    outpc.status2 |= STATUS2_FLATSHIFT;     // turn it on
                }
            }
            if (outpc.status2 & STATUS2_FLATSHIFT) {
                // use flat shift limits
                if (outpc.rpm > ram4.flats_sft) {
                    outpc.status3 |= STATUS3_LAUNCHON; // actually in launch mode, not just armed
                    *lsum = (long) ram4.flats_deg;
                }
                if (outpc.rpm > ram4.flats_hrd) {
                    if (ram4.launchlimopt & 1) {
                        spk_cutx = ram4.launchcutx;
                        spk_cuty = ram4.launchcuty;
                        flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
                    }
                    if ((ram4.launchlimopt & 2) || (outpc.rpm > ram4.launch_fcut_rpm)) {
                        tmp_pw1 = 0;
                        tmp_pw2 = 0;
                    }
                }
            } else {
                unsigned int soft_lim, hard_lim;
                if (ram4.launch_var_on & 0x1f) {       // using variable launch
                    unsigned tmp_adc;
                    tmp_adc = *port_launch_var;
                    if (tmp_adc > 1023) {   // shouldn't ever happen
                        tmp_adc = 0;        // set low rpm limit as safety measure
                    }
                    hard_lim = ram4.launch_var_low
                        + (unsigned int) (((ram4.launch_var_up - ram4.launch_var_low)
                                * (unsigned long) tmp_adc) / 1024);
                    soft_lim = hard_lim - ram4.launch_var_sof;
                } else {
                    hard_lim = ram4.launch_hrd_lim;
                    soft_lim = ram4.launch_sft_lim;
                }
                // use launch limits

                if (outpc.rpm > soft_lim) {
                    outpc.status3 |= STATUS3_LAUNCHON;
                    *lsum = (long) ram4.launch_sft_deg;
                }
                if (outpc.rpm > hard_lim) {
                    if (ram4.launchlimopt & 1) {
                        spk_cutx = ram4.launchcutx;
                        spk_cuty = ram4.launchcuty;
                        flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
                    }
                    if ((ram4.launchlimopt & 2) || (outpc.rpm > ram4.launch_fcut_rpm)) {
                        tmp_pw1 = 0;
                        tmp_pw2 = 0;
                    }
                }
            }
        } else {
            /* ensure show as off when outside conditions */
            outpc.status2 &= ~(STATUS2_LAUNCH | STATUS2_FLATSHIFT); // turn off both
        }
    }
  NO_LAUNCH:;
    /* apply launch retard timer */
    if ( ((outpc.status2 & (STATUS2_LAUNCH | STATUS2_FLATSHIFT)) == 0)
            && (ram4.launch_opt & 0x08) && (launch_timer < 0x7ffe)) {
        outpc.launch_retard = intrp_1ditable(launch_timer, 6,
        (int *) ram_window.pg11.launch_time, 0,
        (int *) ram_window.pg11.launch_retard, 11);
    } else {
        outpc.launch_retard = 0;
    }
    outpc.launch_timer = launch_timer;

    /* throttle stops */
    if (pin_tstop && (ram5.tstop_out & 0x1f)) {
        unsigned char st = 0;
        if (launch_timer > (ram5.tstop_delay + ram5.tstop_hold)) {
            st = 0;
        } else if (launch_timer > ram5.tstop_delay) {
            st = 1;
        }
        if (st) {
            *port_tstop |= pin_tstop;
        } else {
            *port_tstop &= ~pin_tstop;
        }
    }

    /* timed pin delay
        This need not actually be linked to launch, but it is for now
    */
    if (pin_timed1_in && pin_timed1_out && (ram5.timedout1_in & 0x1f)) {
        if (!(flagbyte10 & FLAGBYTE10_TBRAKE)) {
            /* off at the moment */
            if (!(*port_timed1_in & pin_timed1_in)) {
                flagbyte10 |= FLAGBYTE10_TBRAKE;
                *port_timed1_out |= pin_timed1_out;
            } else if (tb_timer > ram5.timedout1_offdelay) {
                *port_timed1_out &= ~pin_timed1_out;
            }
        } else {
            /* was on */
            if (*port_timed1_in & pin_timed1_in) {
                /* now off */
                flagbyte10 &= ~FLAGBYTE10_TBRAKE;
                tb_timer = 0;
            }
        }
    }
}

void do_revlim_overboost_maxafr(void)
{
    if (ram4.RevLimOption & 0x8) {
        //check if within bounds of table, otherwise use normal limits
        RPAGE = tables[8].rpg;
        if (outpc.clt < ram_window.pg8.RevLimLookup[7]) {
            int limit;
            if (outpc.tps < ram4.TpsBypassCLTRevlim) {
                limit = intrp_1ditable(outpc.clt, 8,
                                       (int *) ram_window.pg8.RevLimLookup, 0,
                                       (unsigned int *) ram_window.pg8.RevLimRpm1, 8);
            } else {
                limit = ram4.RevLimTPSbypassRPM;
            }
            DISABLE_INTERRUPTS;
            RevLimRpm1 = limit;
            RevLimRpm2 = limit + ram4.RevLimRpm2;
            ENABLE_INTERRUPTS;
        } else {
            RevLimRpm1 = ram4.RevLimNormal1;
            RevLimRpm2 = ram4.RevLimNormal2;
        }
    } else {
        RevLimRpm1 = ram4.RevLimNormal1;
        RevLimRpm2 = ram4.RevLimNormal2;
    }

    if (outpc.status7 & STATUS7_LIMP) {
        RevLimRpm2 = ram5.cel_revlim; // hard fuel cut limit
        RevLimRpm2 = ram5.cel_revlim - 200; // soft retard limit
    }

    // This is the same method as MS1/Extra. Would be nice to do adaptive spark cut
    // and/or vary cylinders cut. Should ensure that spk_cuty is not a multiple of num_cyl

    if (ram4.RevLimOption & 4) {
        if (outpc.rpm > ram4.RevLimNormal3) {
            spk_cutx = ram4.revlimcutx;
            spk_cuty = ram4.revlimcuty;
            flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
        }
    }

    if (ram4.OverBoostOption & 0x03) {
        int maxboost;

        if (pin_tsw_ob && (!(*port_tsw_ob & pin_tsw_ob))) {
            maxboost = ram4.OverBoostKpa2;
        } else {
            maxboost = ram4.OverBoostKpa;
        }

        if (outpc.map > maxboost) {
            outpc.status2 |= STATUS2_OVERBOOST_ACTIVE;
        }

        if (outpc.map < (maxboost - ram4.OverBoostHyst)) {
            outpc.status2 &= ~STATUS2_OVERBOOST_ACTIVE;
        }
    }

    if (ram4.OverBoostOption & 0x02) {
        if (outpc.status2 & STATUS2_OVERBOOST_ACTIVE) {
            spk_cutx = ram4.overboostcutx;
            spk_cuty = ram4.overboostcuty;
            flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
        }
    }

    /* Now check rev limits for fuel cut*/
    if (outpc.rpm > RevLimRpm2) {
        // Cut fuel for Over Rev
        flagbyte17 |= FLAGBYTE17_REVLIMFC;
    } else if (outpc.rpm < (RevLimRpm2 - ram4.RevLimNormal2_hyst)) {
        // restore fuel
        flagbyte17 &= ~FLAGBYTE17_REVLIMFC;
    }

    do_maxafr();
    if (maxafr_stat) {
        spk_cutx = 255;
        spk_cuty = 255;
        flagbyte10 |= FLAGBYTE10_SPKCUTTMP;
    }

    /* Check what bits are wanting to cut fuel and set/clear the master setbit */
    if ( (flagbyte17 & FLAGBYTE17_REVLIMFC) || (flagbyte17 & FLAGBYTE17_OVERRUNFC)
        || (maxafr_stat == 2)
        || ((ram4.OverBoostOption & 0x01) && (outpc.status2 & STATUS2_OVERBOOST_ACTIVE))
        || ((outpc.engine & ENGINE_CRANK) && (outpc.tps > ram4.TPSWOT)) ) {
        outpc.status3 |= STATUS3_CUT_FUEL;
    } else {
        outpc.status3 &= ~STATUS3_CUT_FUEL;
    }

}

void handle_ovflo(void)
{
    // check for long timers
    if (wheeldec_ovflo) {
        unsigned long wotmp;

        while ((TCNT > 0xfff8) || (TCNT < 6));
        // make sure not right at rollover

        wotmp = ((unsigned long) swtimer << 16) | TCNT;

        /* OVFLO_SPK and OVFLO_DWL removed as no longer used */

        if (wheeldec_ovflo & OVFLO_ROT_DWL) {
            if ((dwl_time_ovflo_trl.time_32_bits - wotmp) < 0x10000) {
                TC6 = dwl_time_ovflo_trl.time_16_bits[1];
                SSEM0SEI;
                TFLG1 = 0x40;       // clear ign OC interrupt flag
                TIE |= 0x40;
                CSEM0CLI;
                wheeldec_ovflo &= ~OVFLO_ROT_DWL;   // done overflow
            }
        }

        if (wheeldec_ovflo & OVFLO_ROT_SPK) {
            if ((spk_time_ovflo_trl.time_32_bits - wotmp) < 0x10000) {
                TC3 = spk_time_ovflo_trl.time_16_bits[1];
                SSEM0SEI;
                TFLG1 = 0x08;
                TIE |= 0x08;
                CSEM0CLI;
                wheeldec_ovflo &= ~OVFLO_ROT_SPK;
            }
        }
    }
}

/***************************************************************************
 **
 ** Determine spare port settings
 **
 **************************************************************************/
void handle_spareports(void)
{
    int ix, tmp1, tmp2 = 0;
    char ctmp1 = 0, ctmp2 = 0, cond1 = 0, cond12 = 0, cond2 = 0;

    RPAGE = tables[23].rpg; // need access to the tuning data in paged ram

    for (ix = 0; ix < NPORT; ix++) {
        if (ram_window.pg23.spr_port[ix]) {
            cond1 = ram_window.pg23.condition1[ix];
            cond12 = ram_window.pg23.cond12[ix];
            cond2 = ram_window.pg23.condition2[ix];
            // Evaluate first condition
            if (ram_window.pg23.out_byte1[ix] == 1) {
                tmp1 = *((char *) (&outpc) + ram_window.pg23.out_offset1[ix]);
            } else {
                tmp1 = *(int *) ((char *) (&outpc) + ram_window.pg23.out_offset1[ix]);
            }

            if (cond1 == '&') { // bitwise AND
                tmp1 = tmp1 & ram_window.pg23.thresh1[ix];
            } else {
                tmp1 = tmp1 - ram_window.pg23.thresh1[ix];
            }

            if (cond1 == '<') {
                tmp1 = -tmp1;   //  convert < condition to same as > condition
            }
            if (cond1 == '=') {
                if ((tmp1 >= -ram_window.pg23.hyst1[ix]) && (tmp1 <= ram_window.pg23.hyst1[ix]))
                    ctmp1 = 1;  // 1st condition true
            } else if (tmp1 > 0) {
                ctmp1 = 1;      // 1st condition true
            } else {
                ctmp1 = 0;      // 1st condition false
            }
            // Evaluate second condition if there is one
            if (cond12 != ' ') {
                if (ram_window.pg23.out_byte2[ix] == 1) {
                    tmp2 = *((char *) (&outpc) + ram_window.pg23.out_offset2[ix]);
                } else {
                    tmp2 =
                        *(int *) ((char *) (&outpc) +
                                  ram_window.pg23.out_offset2[ix]);
                }

                if (cond2 == '&') { // bitwise AND
                    tmp2 = tmp2 & ram_window.pg23.thresh2[ix];
                } else {
                    tmp2 = tmp2 - ram_window.pg23.thresh2[ix];
                }

                if (cond2 == '<') {
                    tmp2 = -tmp2;       //  convert < condition to same as > condition
                }
                if (cond2 == '=') {
                    if ((tmp2 >= -ram_window.pg23.hyst2[ix])
                        && (tmp2 <= ram_window.pg23.hyst2[ix]))
                        ctmp2 = 1;      // 2nd condition true
                } else if (tmp2 > 0) {
                    ctmp2 = 1;  // 2nd condition true
                } else {
                    ctmp2 = 0;  // 2nd condition false
                }
            }
            // Evaluate final condition
            if (((cond12 == '&') && (ctmp1 && ctmp2)) ||
                ((cond12 == '|') && (ctmp1 || ctmp2)) ||
                ((cond12 == ' ') && ctmp1)) {
                char pv = ram_window.pg23.port_val[ix];
                if (lst_pval[ix] != pv) {
                    set_spr_port((char) ix, pv);
                    lst_pval[ix] = pv;
                }
            } else {
                // Evaluate hysteresis conditions
                if ((cond1 == '>')
                    || (cond1 == '<')) {
                    tmp1 = -tmp1 - ram_window.pg23.hyst1[ix];
                }
                if ((cond1 == '=') || (cond1 == '&')) {
                    ctmp1 = 1 - ctmp1;  // 1st hyst. condition opposite of set cond
                } else if (tmp1 > 0) {
                    ctmp1 = 1;  // 1st hysteresis condition true
                } else {
                    ctmp1 = 0;  // 1st hysteresis condition false
                }
                if (cond12 != ' ') {
                    if ((cond2 == '>')
                        || (cond2 == '<'))
                        tmp2 = -tmp2 - ram_window.pg23.hyst2[ix];
                    if ((cond2 == '=') || (cond2 == '&')) {
                        ctmp2 = 1 - ctmp2;      // 2nd hyst. condition opposite of set cond
                    } else if (tmp2 > 0) {
                        ctmp2 = 1;      // 2nd hysteresis condition true
                    } else {
                        ctmp2 = 0;      // 2nd hysteresis condition false
                    }
                }
                // Evaluate final hysteresis condition
                if (((cond12 == '&') && (ctmp1 || ctmp2)) ||
                    ((cond12 == '|') && (ctmp1 && ctmp2)) ||
                    ((cond12 == ' ') && ctmp1)) {
                    char pv = 1 - ram_window.pg23.port_val[ix];
                    if (lst_pval[ix] != pv) {
                        set_spr_port((char) ix, pv);
                        lst_pval[ix] = pv;
                    }
                }
            }                   // end eval of hysteresis conditions
        }                       // end if spr_port
    }
}

/***************************************************************************
 **
 ** EGT
 **
 **************************************************************************/
void do_egt(void)
{
    int ix;
    unsigned char add = 0, melt =0;

    if (!ram4.egt_num) {
        return;
    }

    for (ix = 0; ix < ram4.egt_num ; ix++) {
        unsigned int egt;
        //convert to temp units
        egt = ram4.egt_temp0 + (((long)*port_egt[ix] * (long)(ram4.egt_tempmax - ram4.egt_temp0)) / 1023);
        if ((egt > ram4.egt_tempmax) || (egt < ram4.egt_temp0)) {
            egt = -1000; // nonsense failsafe
        }
        outpc.egt[ix] = egt;
        if (ram4.egt_conf & 0x01) { // are 'actions' enabled
            if (stat_egt[ix] == 0) { // ignore any broken channels
                if (egt > ram4.egt_warn) {
                    add = 1;
                }
                if (egt > ram4.egt_max) {
                    melt = 1;
                }
            }
        }
    }

    if (add) {
        flagbyte9 |= FLAGBYTE9_EGTADD;
        outpc.status6 |= STATUS6_EGTWARN;
        if ((egt_timer == 0) && (maxafr_stat == 0) && (ram4.egt_conf & 0x02)) {
            egt_timer = 1; // only start this timer if not in shutdown mode
        }
    } else {
        flagbyte9 &= ~FLAGBYTE9_EGTADD;
        outpc.status6 &= ~STATUS6_EGTWARN;
        egt_timer = 0; // cancel it
    }

    if (ram4.egt_conf & 0x02) { // shutdown enabled
        if (melt || (add && (egt_timer > ram4.egt_time))) {
            flagbyte9 |= FLAGBYTE9_EGTMELT;
            outpc.status6 |= STATUS6_EGTSHUT;
            egt_timer = 0; // cancel it, go to shut down
            if (maxafr_stat == 0) {
                maxafr_stat = 1;    // mainloop code reads the stat and cut appropriately
                maxafr_timer = 1;   // start counting
            }
            if (pin_cel) { // turn on as soon as we can
                SSEM0SEI;
                *port_cel |= pin_cel;
                CSEM0CLI;
            }
            outpc.status7 |= STATUS7_CEL;
        } else {
            flagbyte9 &= ~FLAGBYTE9_EGTMELT;
            outpc.status6 &= ~STATUS6_EGTSHUT;
        }
    }
}


/***************************************************************************
 **
 ** Generic sensors
 **
 **************************************************************************/
void do_sensors()
{
    unsigned char i;
    if (((unsigned int)lmms - sens_time) > 78) { // every 10ms
        sens_time = (unsigned int)lmms;
    } else {
        return;
    }
    for (i = 0; i <= 15 ; i++) {
        unsigned char s = ram5.sensor_source[i];
#ifdef MS3PRO
        if (i == 15) {
            s = 1;
        }
#endif
        if (s) {
            int val, last_val;
            unsigned char t;
            // grab raw value
            val = *port_sensor[i];

            // grab last val
            last_val = outpc.sensors[i];

            // do transformation
            t = ram5.sensor_trans[i];
#ifdef MS3PRO
            if (i == 15) {
                /* force this to internal temp sensor */
                val = ATD0DR6;
                t = 7;
            }
#endif
            if (t == 1) {
                int gcc_stupid_val0;
                gcc_stupid_val0 = ram5.sensor_val0[i];
                __asm__ __volatile__
                ("ldd %1\n"
                 "subd %2\n"
                 "ldy %3\n"
                 "emuls\n"
                 "ldx #1023\n"
                 "edivs\n"
                 "addy  %2\n"
                :"=y"(val)
                :"m"(ram5.sensor_max[i]), "m"(gcc_stupid_val0), "m"(val)
                :"d", "x");
                
            } else if (t == 2) {
                __asm__ __volatile__
                ("ldd    %1\n"
                 "subd   %2\n"
                 "ldy    %3\n"
                 "emul\n"
                 "ldx    #1023\n"
                 "ediv\n"
                 "addy   %2\n"
                 :"=y"(val)
                 :"m"(ram4.mapmax), "m"(ram4.map0), "m"(val)
                 :"d", "x");


            } else if (t == 3) {
                GPAGE = 0x10;
                __asm__ __volatile__
                ("aslx\n"
                 "addx #0x4000\n"    // cltfactor_table address
                 "gldy 0,x\n"
                 "ldd %2\n"
                 "emul\n"
                 "ldx #100\n"
                 "ediv\n"
                 "addy   %3\n"
                :"=y"(val)
                :"x"(val), "m"(ram4.cltmult),"m"(ram4.clt0)
                :"d");
                if (ram5.sensor_temp & 0x01) {
                    val = ((val -320) * 5) / 9;
                }

            } else if (t == 4) {
                GPAGE = 0x10;
                __asm__ __volatile__
                ("aslx\n"
                 "addx #0x4800\n"    // matfactor_table address
                 "gldy 0,x\n"
                 "ldd %2\n"
                 "emul\n"
                 "ldx #100\n"
                 "ediv\n"
                 "addy   %3\n"
                :"=y"(val)
                :"x"(val), "m"(ram4.matmult),"m"(ram4.mat0)
                :"d");
                if (ram5.sensor_temp & 0x01) {
                    val = ((val -320) * 5) / 9;
                }

            } else if (t == 5) {
                GPAGE = 0x10;
                __asm__ __volatile__
                ("addx #0x5000\n"     //egofactor_table address
                 "gldab 0,x\n"
                 "clra\n"
                 "ldy %2\n"
                 "emul\n"
                 "ldx #100\n"
                 "ediv\n"
                 "addy %3\n"
                :"=y"(val)
                :"x"(val), "m"(ram4.egomult), "m"(ram4.ego0)
                :"d");

            } else if (t == 6) {
                GPAGE = 0x10;
                __asm__ __volatile__
                ("aslx\n"
                 "addx #0x5400\n"     //matfactor_table address
                 "gldy 0,x\n"
                :"=y"(val)
                :"x"(val)
                :"d");

            } else if (t == 7) {
                GPAGE = 0x10;
                __asm__ __volatile__
                ("aslx\n"
                 "addx #0x5c00\n"     //gmfactor_table address
                 "gldy 0,x\n"
                :"=y"(val)
                :"x"(val)
                :"d");

#ifdef MS3PRO
                if (i == 15) {
                    /* this adjustment makes the temp sensor very close to the GM calibration over the working range */
                    val -= 110;
                }
#endif
                if (ram5.sensor_temp & 0x01) {
                    val = ((val -320) * 5) / 9;
                }

            } else { //no transformation = raw
                val *= 10; // (display is in 0.1 units)
            }

            // lag factor
            __asm__ __volatile__
            ("ldd %1\n"
             "subd %3\n"
             "tfr d,y\n"
             "clra\n"
             "ldab %2\n" // it is a uchar
             "emuls\n"
             "ldx #100\n"
             "edivs\n"
             "addy %3\n"
            :"=y"(val)
            :"m"(val), "m"(ram5.sensorLF[i]), "m"(last_val)
            :"d", "x");

            // store result
            outpc.sensors[i] = val;
        }
    }
}

void gearpos()
{
/***************************************************************************
 **
 ** Gear detection
 **
 ** Selection options: Off, RPM/VSS, Analogue, CAN VSS
 ** For RPM/VSS the gear ratio and final drive ratio table is used
 ** With analogue, gear position is indicated by input voltage
 ** CAN is a direct gear number captured from a remote board
 **************************************************************************/
    if ((ram4.gear_method & 0x03) == 1) {
        // in this mode calculate gear from RPM/VSS factor
        // output shaft speed = mph / wheel circumference * fdratio (* scaler)
        // gear ratio = rpm / output shaft speed
        // then compare to gear ratio table
        unsigned int expect_rpm[6], x;

        if (outpc.vss1 < 2) {
            outpc.gear = 0;
            return;
        } else if (outpc.vss1 < 5) {
            return;
        }

        /* build array of expected rpms per gear */
        for (x = 0 ; x < ram4.gear_no ; x++) {
            expect_rpm[x] = (unsigned int)((ram4.gear_ratio[x] * (unsigned long)outpc.vss1 * gear_scale) / 10000);
        }

        if (outpc.rpm > expect_rpm[0]) {
            /* first gear or free-revving */
            outpc.gear = 1;
            return;
        }

        if (outpc.rpm < expect_rpm[ram4.gear_no - 1]) {
            /* top gear or coasting in neutral */
            outpc.gear = ram4.gear_no;
            return;
        }

        // now compare against actual rpms
        // Exit as soon as decided on gear or indeterminate
        // note in gear_ratio[] 0 = 1st etc.
        
        // loop to check other gears
        for (x = 0 ; x < ram4.gear_no ; x++) {
            unsigned int low_rpm, high_rpm;
            if (x > 0) {
                high_rpm = expect_rpm[x] + ((expect_rpm[x - 1] - expect_rpm[x]) / 3);
            } else {
                high_rpm = expect_rpm[0];
            }
            if (x < (ram4.gear_no - 1)) {
                low_rpm = expect_rpm[x] - ((expect_rpm[x] - expect_rpm[x + 1]) / 3);
            } else {
                low_rpm = expect_rpm[ram4.gear_no - 1];
            }

            if ((outpc.rpm > low_rpm) && (outpc.rpm < high_rpm)) {
                outpc.gear = x + 1;
                return;
            }
        }
        /* if in-between, then outpc.gear remains unchanged */
        return;

    } else if (((ram4.gear_method & 0x03) == 2) && port_gearsel) {
        unsigned int gear_v, gear_lowv, gear_highv, x, plus;
        
        // in this mode the 0-5V signal tells us the gear we are in

//       gear_v  = *port_gearsel / 1023 * 500
        __asm__ __volatile__ (
        "ldy #500\n"
        "emul\n"
        "ldx #1023\n"
        "ediv\n"
        : "=y" (gear_v)
        : "d" (*port_gearsel)
        );

        if (ram4.gearv[1] < ram4.gearv[ram4.gear_no]) {
            gear_lowv = 1;
            gear_highv = ram4.gear_no;
            plus = 1;
        } else {
            gear_lowv = ram4.gear_no;
            gear_highv = 1;
            plus = 0xffff;
        }

        // is neutral low or high?
        // note in gearv[] 0 = neutral, 1 = 1st etc.
        if (ram4.gearv[0] > ram4.gearv[gear_highv]) {
            unsigned int v_diff, v_high;
            // high neutral
            v_diff = (ram4.gearv[0] - ram4.gearv[gear_highv]) >> 2;
            v_high = ram4.gearv[0] - v_diff;
            if (gear_v > v_high) {
                outpc.gear = 0;
                return;
            }
        } else {
            unsigned int v_diff, v_low;
            // low neutral
            v_diff = (ram4.gearv[gear_lowv] - ram4.gearv[0]) >> 2;
            v_low = ram4.gearv[0] + v_diff;
            if (gear_v < v_low) {
                outpc.gear = 0;
                return;
            }
        }

        // loop to selection value
        for (x = 1; x <= ram4.gear_no ; x++) {
            int v_diff, v_low, v_high;
            // gap below
            if (x == gear_lowv) { // lowest value
                v_diff = (ram4.gearv[x + plus] - ram4.gearv[x]) >> 2; // 25% // use gap above only
                v_low = ram4.gearv[gear_lowv] - v_diff;
                v_high = ram4.gearv[gear_lowv] + v_diff;
            } else if (x == gear_highv) { // highest value
                v_diff = (ram4.gearv[x] - ram4.gearv[x - plus]) >> 2; // 25% // use gap below only
                v_low = ram4.gearv[gear_highv] - v_diff;
                v_high = ram4.gearv[gear_highv] + v_diff;
            } else {
                v_diff = (ram4.gearv[x] - ram4.gearv[x - plus]) >> 2; // 25% // use gap below
                v_low = ram4.gearv[x] - v_diff;
                v_diff = (ram4.gearv[x + plus] - ram4.gearv[x]) >> 2; // 25% // use gap above
                v_high = ram4.gearv[x] + v_diff;
            }

            if ((gear_v > v_low) && (gear_v < v_high)) {
                outpc.gear = x;
                return;
            }
        } // end for
    } else if ((ram4.gear_method & 0x03) == 3) {
        outpc.gear = datax1.gear; // grab the CAN gear defined on the CAN parameters
    }
}

void calcvssdot()
{
    RPAGE = RPAGE_VARS1;

    vss_cnt++;

    if (vss_cnt < ram4.vssdot_int) { // sample every X*10ms
        return;
    } else {
        int vsscalcno;
        vss_cnt = 0;

        for (vsscalcno = 0; vsscalcno < 2 ; vsscalcno++) {
            int vssi;
            long vssdot_sumx, vssdot_sumx2, vssdot_sumy, vssdot_sumxy;
            long toprow, btmrow;
            unsigned long vssdot_x; // was uint

            v1.vssdot_data[0][0][vsscalcno] = (unsigned int)lmms; // store actual 16 bit time to first pt in array
            v1.vssdot_data[0][1][vsscalcno] = *((unsigned int*)&outpc.vss1 +vsscalcno); // store speed

            vssdot_sumx = 0;
            vssdot_sumx2 = 0;
            vssdot_sumy = 0;
            vssdot_sumxy = 0;

            vssi = 0;
            // only use six datapoints for this. Array defined as larger
            while ((vssi < 6) && (vssdot_sumxy < 10000000)) {
                vssdot_x = v1.vssdot_data[0][0][vsscalcno] - v1.vssdot_data[vssi][0][vsscalcno]; // relative time to keep numbers smaller
                vssdot_sumx += vssdot_x;
                vssdot_sumy += v1.vssdot_data[vssi][1][vsscalcno];
                vssdot_sumx2 += ((unsigned long)vssdot_x * vssdot_x); // overflow if vssdot_x > 65535
                vssdot_sumxy += ((unsigned long)vssdot_x * v1.vssdot_data[vssi][1][vsscalcno]);
                vssi++;
            }

            toprow = vssdot_sumxy - (vssdot_sumx * (vssdot_sumy / vssi)); // divide sumy to help overflow
            btmrow = vssdot_sumx2 - (vssdot_sumx * vssdot_sumx / vssi);

            btmrow = btmrow / 10; // allows top row to be 10x less to reduce overflow.
            toprow = (-toprow * 781) / btmrow;
            if (toprow > 32767) {
                toprow = 32767;
            } else if (toprow < -32767) {
                toprow = -32767;
            }

/*
            // very low vssdot
            if ((toprow > -150) && (toprow < 150)) { // these factors would need changing if code ever used
                long tmp_top, tmp_btm;
                // see how it compares to 20 positions
                tmp_btm = vssdot_data[0][0][vsscalcno] - vssdot_data[VSSDOT_N-1][0][vsscalcno];
                tmp_top = (int)vssdot_data[0][1][vsscalcno] - (int)vssdot_data[VSSDOT_N-1][1][vsscalcno];
                tmp_btm = tmp_btm / 10; // allows top row to be 10x less to reduce overflow.
                tmp_top = (tmp_top * 781) / tmp_btm;
                if (tmp_top > 32767) {
                    tmp_top = 32767;
                } else if (tmp_top < -32767) {
                    tmp_top = -32767;
                }
                // use lesser magnitude of two
                if (long_abs(tmp_top) < long_abs(toprow)) {
                    toprow = tmp_top;
                }
            }
*/
            toprow = 30L * (toprow - *((int*)&outpc.vss1dot +vsscalcno)); // relies on vss1dot, vss2dot following each other
            if ((toprow > 0) && (toprow < 100)) {
                toprow = 100;
            } else if ((toprow < 0) && (toprow > -100)) {
                toprow = -100;
            }
            *((int*)&outpc.vss1dot +vsscalcno) += (int)(toprow / 100);

            //shuffle data forwards by one
            for (vssi = VSSDOT_N - 1; vssi > 0 ; vssi--) {
                v1.vssdot_data[vssi][0][vsscalcno] = v1.vssdot_data[vssi - 1][0][vsscalcno];
                v1.vssdot_data[vssi][1][vsscalcno] = v1.vssdot_data[vssi - 1][1][vsscalcno];
            }
        }
    }
}

void accelerometer()
{
    signed int tmp1, tmp2, tmp3;
//need (more!) lag factors
    if (((unsigned int)lmms - accxyz_time) > 78) { // every 10ms
        accxyz_time = (unsigned int)lmms;
    } else {
        return;
    }
    if (ram4.accXport) {
        tmp1 = (ram4.accXcal1 + ram4.accXcal2) >> 1; // mid point
        tmp2 = (ram4.accXcal2 - ram4.accXcal1) >> 1; // 1g range
//        tmp3 = (((int)*accXport - tmp1) * 1000L) / tmp2; // gives g x 1000
        __asm__ __volatile__ (
        "ldy #9810\n" // build in 'g'
        "emuls\n"
        "ldx %2\n"
        "edivs\n"
        :"=y" (tmp3)
        :"d" ((int)*accXport - tmp1), "m" (tmp2)
        :"x"
        );

        //lag factor
        __asm__ __volatile__ (
        "suby   %3\n"
        "clra\n"
        "ldab    %2\n" // it is a uchar
        "emuls\n"
        "ldx     #100\n"
        "edivs\n"
        "addy    %3\n"
        :"=y"(outpc.accelx)
        :"y"(tmp3), "m"(ram4.accxyzLF), "m"(outpc.accelx)
        :"d", "x");

    }
    if (ram4.accYport) {
        tmp1 = (ram4.accYcal1 + ram4.accYcal2) >> 1; // mid point
        tmp2 = (ram4.accYcal2 - ram4.accYcal1) >> 1; // 1g range
//        outpc.accely = (((int)*accYport - tmp1) * 1000L) / tmp2; // gives g x 1000
        __asm__ __volatile__ (
        "ldy #9810\n"
        "emuls\n"
        "ldx %2\n"
        "edivs\n"
        :"=y" (tmp3)
        :"d" ((int)*accYport - tmp1), "m" (tmp2)
        :"x"
        );

        //lag factor
        __asm__ __volatile__ (
        "suby   %3\n"
        "clra\n"
        "ldab    %2\n" // it is a uchar
        "emuls\n"
        "ldx     #100\n"
        "edivs\n"
        "addy    %3\n"
        :"=y"(outpc.accely)
        :"y"(tmp3), "m"(ram4.accxyzLF), "m"(outpc.accely)
        :"d", "x");

    }
    if (ram4.accZport) {
        tmp1 = (ram4.accZcal1 + ram4.accZcal2) >> 1; // mid point
        tmp2 = (ram4.accZcal2 - ram4.accZcal1) >> 1; // range
//        outpc.accelz = (((int)*accZport - tmp1) * 1000L) / tmp2; // gives g x 1000
        __asm__ __volatile__ (
        "ldy #9810\n"
        "emuls\n"
        "ldx %2\n"
        "edivs\n"
        :"=y" (tmp3)
        :"d" ((int)*accZport - tmp1), "m" (tmp2)
        :"x"
        );

        //lag factor
        __asm__ __volatile__ (
        "suby   %3\n"
        "clra\n"
        "ldab    %2\n" // it is a uchar
        "emuls\n"
        "ldx     #100\n"
        "edivs\n"
        "addy    %3\n"
        :"=y"(outpc.accelz)
        :"y"(tmp3), "m"(ram4.accxyzLF), "m"(outpc.accelz)
        :"d", "x");

    }
}

void ck_log_clr(void)
{
/* Check for clearing trigger/tooth logger buffer */
    if (flagbyte8 & FLAGBYTE8_LOG_CLR) {
        if ((page >= 0xf0) && (page <= 0xf4)) { // double check
            unsigned char tmp_rpage;
            tmp_rpage = RPAGE;
            RPAGE = TRIGLOGPAGE;
            __asm__ __volatile__ (
            "ldd   #512\n"
            "tthclr:\n"
            "clrw   2,y+\n"
            "dbne   d, tthclr\n"
            : 
            : "y"(TRIGLOGBASE)
            : "d");
            RPAGE = tmp_rpage;
        }

        if (page == 0xf0) {
            flagbyte0 |= FLAGBYTE0_TTHLOG;
        } else if (page == 0xf1) {
            flagbyte0 |= FLAGBYTE0_TRGLOG;
        } else if ((page == 0xf2) || (page == 0xf3)) {
            flagbyte0 |= FLAGBYTE0_COMPLOG;
        } else if (page == 0xf4) {
            flagbyte0 |= FLAGBYTE0_MAPLOGARM; // gets started in ISR
        }
        flagbyte8 &= ~FLAGBYTE8_LOG_CLR;
    }
}

void chk_crc(void)
{
    unsigned long crc = 0;
    /* if required, calc crc of ram page */
    /* local CRC handled in serial.c now */
    if (flagbyte9 & FLAGBYTE9_CRC_CAN) {
        int i;
        RPAGE = tables[tble_idx].rpg;
        crc = crc32buf(0, tables[tble_idx].adhi << 8, 0x400);    // incrc, buf, size

        flagbyte9 &= ~FLAGBYTE9_CRC_CAN;
// only check ram copy, irrespective of page number
        i = can[1].cxno_in;
        *(unsigned long*)&can[1].cx_datbuf[i][0] = crc;
        // tell the CAN system to send it somewhere
		can[1].cx_msg_type[i] = MSG_RSP; 
		can[1].cx_dest[i] = canbuf[5];   // send to device stored in ISR

		can[1].cx_destvarblk[i] = canbuf[4];  // table stored in ISR
		can[1].cx_destvaroff[i] = 0;  // assume offset is zero
		can[1].cx_varbyt[i] = 4;    // 4 bytes

		// This is where (in xmt ring buffer) to put next message
		if(can[1].cxno_in < (NO_CANMSG - 1)) {
			can[1].cxno_in++;
		} else {
			can[1].cxno_in = 0;
		}
		// increment counter
		if(can[1].cxno < NO_CANMSG) {
			can[1].cxno++;
		} else {
			can[1].cxno = NO_CANMSG;
		}
        if(!(CAN0TIER & 0x07))  {
            CAN0TBSEL = CAN0TFLG;
            CAN0TIER = CAN0TBSEL;
        }
    }
}

long long_abs(long in)
{
    if (in < 0) {
        return -in;
    } else {
        return in;
    }
}


void shifter()
{
    /**************************************************************************
     ** Bike type ignition cut on shift and optional air-shift output
     **************************************************************************/
	if ((ram5.shift_cut_on & 1) == 0) {
        outpc.status3 &= ~STATUS3_BIKESHIFT;
        return;
    }
    // state machine
    if (shift_cut_phase == 0) {
        if ((outpc.rpm > ram5.shift_cut_rpm) && (outpc.tps > ram5.shift_cut_tps) &&   // base conditions AND
            ((pin_shift_cut_in && ((*port_shift_cut_in & pin_shift_cut_in) == pin_shift_cut_match))  // button pressed
            || ((ram5.shift_cut_on & 2) && (outpc.gear) && (outpc.gear < 6) && (outpc.rpm > ram5.shift_cut_rpmauto[outpc.gear - 1])) ) ) { // OR auto
            shift_cut_phase = 1;
            SSEM0SEI;
            *port_shift_cut_out |= pin_shift_cut_out; // enable solenoid
            CSEM0CLI;
            shift_cut_timer = ram5.shift_cut_delay;
        }
    } else if ((shift_cut_phase == 1) && (shift_cut_timer == 0)) {
        shift_cut_timer = ram5.shift_cut_time;
        if (ram5.shift_cut_on & 4) {
            shift_cut_timer += ram5.shift_cut_add[outpc.gear - 1];
        }
        shift_cut_phase = 2;
    } else if (shift_cut_phase == 2) {            
        if (shift_cut_timer == 0) {
            shift_cut_timer = ram5.shift_cut_soldelay;
            shift_cut_phase = 3;
        } else {
            flagbyte10 |= FLAGBYTE10_SPKCUTTMP; // have to do this every loop
            spk_cutx = 255;
            spk_cuty = 255;
        }
    } else if ((shift_cut_phase == 3) && (shift_cut_timer == 0)) {
        SSEM0SEI;
        *port_shift_cut_out &= ~pin_shift_cut_out; // disable solenoid
        CSEM0CLI;
        shift_cut_timer = ram5.shift_cut_reshift;
        shift_cut_phase = 4;
    } else if ((shift_cut_phase == 4) && (shift_cut_timer == 0)) {
        //check button not already pressed
        if (pin_shift_cut_in && ((*port_shift_cut_in & pin_shift_cut_in) != pin_shift_cut_match)) {
            shift_cut_phase = 0; // back to the start
        }
    }
    if (shift_cut_phase) {
        outpc.status3 |= STATUS3_BIKESHIFT;
    } else {
        outpc.status3 &= ~STATUS3_BIKESHIFT;
    }

}

void generic_pwm()
{
    /**************************************************************************
     ** Generic PWM open-loop outputs - calculate duty
     **************************************************************************/
    int i;
    for (i = 0; i < 6 ; i++) {
        if (ram5.pwm_opt[i] & 1) {
            unsigned char duty, load_opt;
            int load;
            load_opt = ram5.pwm_opt[i] >> 5;
            //[5:7], "INVALID", "MAP", "% baro", "TPS", "MAFload", "CLT", "ITB", "MAT"
            if (load_opt == 2) {
                load = (outpc.map * 1000L) / outpc.baro;
            } else if (load_opt == 3) {
                load = outpc.tps;
            } else if (load_opt == 4) {
                load = outpc.mafload;
            } else if (load_opt == 5) {
                load = outpc.clt;
                if (ram5.sensor_temp & 0x01) {
                    load = ((load -320) * 5) / 9;
                }
            } else if (load_opt == 6) {
                load = outpc.batt;
            } else if (load_opt == 7) {
                load = outpc.mat;
                if (ram5.sensor_temp & 0x01) {
                    load = ((load -320) * 5) / 9;
                }
            } else {
                load = outpc.map; // default
            }
            duty = intrp_2dctable(outpc.rpm, load, 6, 6,
                   &ram_window.pg21.pwm_rpms[i][0],
                   &ram_window.pg21.pwm_loadvals[i][0],
                   &ram_window.pg21.pwm_duties[i][0][0], 0, 21);
            if (ram5.pwm_opt[i] & 2) { // variable
                outpc.duty_pwm[i] = duty;
            } else { // on/off
                if (duty > ram5.pwm_onabove[i]) {
                    outpc.duty_pwm[i] = 100;
                } else if (duty < ram5.pwm_offbelow[i]) {
                    outpc.duty_pwm[i] = 0;
                }
            }
        }
    }
}

void generic_pwm_outs()
{
    /**************************************************************************
     ** Generic PWM open-loop outputs - calculate on/off times for RTC
     ** handled here so other features (e.g. ALS) can re-use the outputs
     **************************************************************************/
    int i;
    for (i = 0; i < 6 ; i++) {
        if (pin_pwm[i] && (pin_pwm[i] < 255)) {
            /* figure out PWM parameters for isr_rtc.s */
            unsigned char mult;
            unsigned int max, trig;
            mult = (ram5.pwm_opt[i] >> 2) & 0x7;
    
            if (mult) {
                max = mult * 100;
                trig = outpc.duty_pwm[i] * mult;
            } else {
                // gives 250Hz with 3% duty steps
                max = 31;
                trig = (outpc.duty_pwm[i] * 10) / 32; //  divide by 3.2
            }
            gp_max_on[i] = trig; // on time
            gp_max_off[i] = max - trig; // off time
            if ((gp_stat[i] & 1) == 0) {
                gp_clk[i] = 1; // bring it back to earth
                gp_stat[i] |= 1; // enabled
            }
        } else {
            gp_stat[i] = 0;
        }
    }
}

void poll_i2c_rtc()
{
/* code to read/write MCP79410 */
    if (i2cstate2 == 0) {
        i2caddr = 0;    // SR
        i2cstate = 1; // read
        i2cstate2++;

    } else if (i2cstate2 == 1) {
        if (i2cstate == 0) {
            if (i2cbyte & 0x80) { // osc is running
                i2cstate2 = 100;
            } else {
                // osc wasn't running, set time as well
                datax1.setrtc_sec = 0;
                datax1.setrtc_min = 0;
                datax1.setrtc_hour = 0;
                datax1.setrtc_month = 0;
                datax1.setrtc_date = 1;
                datax1.setrtc_day = 1;
                datax1.setrtc_year = 2001;
                i2cstate2 = 120;
            }
        }

// -------------- wait phase
    } else if (i2cstate2 == 99) {
        if (flagbyte9 & FLAGBYTE9_GETRTC) {
            flagbyte9 &= ~FLAGBYTE9_GETRTC;
            i2cstate2 = 100;

        } else if (datax1.setrtc_lock == 0x5a) {
            datax1.setrtc_lock = 0;
            i2cstate2 = 120;
//            outpc.istatus5++;
        }

// -------------- read phase

    } else if (i2cstate2 == 100) {
        i2caddr = 0;
        i2cstate = 1; // start off a read
        i2cstate2++;

    } else if (i2cstate2 == 101) {
        if (i2cstate == 0) {
            datax1.rtc_sec = (i2cbyte & 0x0f) + (10 * ((i2cbyte >> 4) & 0x07));
            i2caddr = 1;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 102) {
        if (i2cstate == 0) {
            datax1.rtc_min = (i2cbyte & 0x0f) + (10 * (i2cbyte >> 4));
            i2caddr = 2;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 103) {
        if (i2cstate == 0) {
            datax1.rtc_hour = (i2cbyte & 0x0f) + (10 * ((i2cbyte >> 4) & 0x03));
            i2caddr = 4;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 104) {
        if (i2cstate == 0) {
            datax1.rtc_date = (i2cbyte & 0x0f) + (10 * ((i2cbyte >> 4) & 0x03));
            i2caddr = 5;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 105) {
        if (i2cstate == 0) {
            datax1.rtc_month = (i2cbyte & 0x0f) + (10 * ((i2cbyte >> 4) & 0x01));
            i2caddr = 6;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 106) {
        if (i2cstate == 0) {
            datax1.rtc_year = 2000 + (i2cbyte & 0x0f) + (10 * (i2cbyte >> 4)); // not year 2100 safe
            i2caddr = 3;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }

    } else if (i2cstate2 == 107) {
        if (i2cstate == 0) {
            datax1.rtc_day = i2cbyte & 0x07;
//            i2cstate2 = 99;
            i2caddr = 8;
            i2cstate = 1; // start off a read
            i2cstate2++;
        }
    } else if (i2cstate2 == 108) {
        if (i2cstate == 0) {
            outpc.status4 = i2cbyte;
            i2cstate2 = 99;
        }

// -------------- set time phase

    } else if (i2cstate2 == 120) {
        i2cstate2++;

    } else if (i2cstate2 == 121) {
        if (i2cstate == 0) {
            i2caddr = 0;    // seconds
            i2cbyte = 0x80 | (datax1.setrtc_sec % 10) | ((datax1.setrtc_sec / 10) << 4);
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 122) {
        if (i2cstate == 0) {
            i2caddr = 1;
            i2cbyte = (datax1.setrtc_min % 10) | ((datax1.setrtc_min / 10) << 4);
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 123) {
        if (i2cstate == 0) {
            i2caddr = 2;
            i2cbyte = ((datax1.setrtc_hour % 10) | ((datax1.setrtc_hour / 10) << 4)) & 0x3f; // ensure 24hr
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 124) {
        if (i2cstate == 0) {
            i2caddr = 3;
            i2cbyte = (datax1.setrtc_day & 0x07) | 0x38;
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 125) {
        if (i2cstate == 0) {
            i2caddr = 4;
            i2cbyte = (datax1.setrtc_date % 10) | ((datax1.setrtc_date / 10) << 4);
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 126) {
        if (i2cstate == 0) {
            i2caddr = 5;
            i2cbyte = (datax1.setrtc_month % 10) | ((datax1.setrtc_month / 10) << 4);
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 127) {
        if (i2cstate == 0) {
            i2caddr = 6;
            i2cbyte = (datax1.setrtc_year % 10) | (((datax1.setrtc_year / 10) % 10) << 4);
            i2cstate = 30; // write
            i2cstate2++;
        }
    /* century not stored */


    } else if (i2cstate2 == 128) {
        if (i2cstate == 0) {
            i2caddr = 8;
            if (ram4.rtc_trim < 0) {
                i2cbyte = 128 - ram4.rtc_trim; // 0x80 | abs(rtc_trim)
            } else {
                i2cbyte = ram4.rtc_trim;
            }
            i2cstate = 30; // write
            i2cstate2++;
        }

    } else if (i2cstate2 == 129) {
        if (i2cstate == 0) {
            i2cstate2 = 99; // done
        }
    }
}

void antilag()
{
    unsigned char newstat = 0;
    RPAGE = 0xfb; // HARDCODING for page 24
    // safe to alter RPAGE in mainloop
    if (ram_window.pg24.als_in_pin & 0x1f) {
 
        if ((als_state == 0) && als_timer) { // off, pause timer running
            newstat = 0;

        } else if (als_state == 2) {
            /* was on but now turned off, wait for switch released */

// per user feedback, if any conditions not met then reset
//            if (!(pin_alsin && !(*port_alsin & pin_alsin))) {
            // switch released, so fully off again
            // required to prevent cycling
                als_state = 0;
//            }
            return;

        } else if ((outpc.clt > ram_window.pg24.als_minclt) && (outpc.clt < ram_window.pg24.als_maxclt)
            /* && (outpc.tps > ram_window.pg24.als_mintps)*/ && (outpc.tps < ram_window.pg24.als_maxtps)
            && (outpc.rpm > ram_window.pg24.als_minrpm) && (outpc.rpm < ram_window.pg24.als_maxrpm)
            && (outpc.mat < ram_window.pg24.als_maxmat)
            && (pin_alsin && ((*port_alsin & pin_alsin) == pin_match_alsin)) ) {

            newstat = 1;
        } else {
            newstat = 0;
        }

        if (newstat) {
            if (als_state == 0) { // was off, now on
                als_state = 1;
                als_timer = ram_window.pg24.als_maxtime;
                if (pin_alsout) {
                    SSEM0SEI;
                    *port_alsout |= pin_alsout;
                    CSEM0CLI;
                }
                als_iacstep = IACmotor_pos; // save it
            } else { // still on
                if (!als_timer) { // reached zero
                    newstat = 0; // turn it off
                    goto ALS_CHKSTAT;
                }
            }

            als_timing = intrp_2ditable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg24.als_rpms[0],
                   &ram_window.pg24.als_tpss[0], &ram_window.pg24.als_timing[0][0], 24);
            als_addfuel = intrp_2ditable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg24.als_rpms[0],
                   &ram_window.pg24.als_tpss[0], &ram_window.pg24.als_addfuel[0][0], 24);

            /* spark cut handled in rev limit spark cut section, calculate spark cut */
            if (ram_window.pg24.als_opt & 0x02) {
                unsigned int sc_pct;
                sc_pct = intrp_2dctable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg24.als_rpms[0],
                   &ram_window.pg24.als_tpss[0], &ram_window.pg24.als_sparkcut[0][0], 0, 24);
                // rough percentage makes setting easier to comprehend
                spk_cutx = (unsigned char)(((num_cyl + 1) * sc_pct) / 100);
                spk_cuty = num_cyl + 1;
                flagbyte10 |= FLAGBYTE10_SPKCUTTMP; // needed on each pass
            }

            /* fuel cut handled in ign_in only, calculate cyclic fuel cut */
            /* only permitted with a sequential variant on MS3X outputs, would be wholly dangerous
               with batch fire outputs */
            if ((ram_window.pg24.als_opt & 0x01) && (ram4.hardware & HARDWARE_MS3XFUEL) && (ram4.sequential & (SEQ_SEMI | SEQ_FULL))) {
                unsigned int fc_pct;
                fc_pct = intrp_2dctable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg24.als_rpms[0],
                   &ram_window.pg24.als_tpss[0], &ram_window.pg24.als_fuelcut[0][0], 0, 24);
                // rough percentage makes setting easier to comprehend
                fuel_cutx = (unsigned char)(((num_cyl + 1) * fc_pct) / 100);
                fuel_cuty = num_cyl + 1;
                flagbyte10 |= FLAGBYTE10_FUELCUTTMP;
            }

            if (ram_window.pg24.als_opt & 0x04) {
//                IACmotor_pos = outpc.iacstep = ram_window.pg24.als_iac_dutysteps;
                IACmotor_pos = ram_window.pg24.als_iac_dutysteps;

            }
            if (ram_window.pg24.als_opt & 0x10) {
                outpc.duty_pwm[5] = ram_window.pg24.als_pwm_duty;
            }
        }
    }

ALS_CHKSTAT:;
    // ensure in all cases that status gets reset if ALS was on
    if ((!newstat) && (als_state == 1)) { // was on, now off
        // clear any settings
        als_state = 0; // ready to restart, (was wait for true de-activation  = 2)
/* change pending user feedback on forum */
//        if (als_timer) {
//            als_timer = 0; /* hadn't reached max time, so ok to restart immediately */
//        } else {
            als_timer = ram_window.pg24.als_pausetime; /* hit timeout, ensure delay */
//        }
        spk_cutx = 0;
        spk_cuty = 0;
        fuel_cutx = 0;
        fuel_cuty = 0;
//        IACmotor_pos = outpc.iacstep = als_iacstep; // restore
        IACmotor_pos = als_iacstep; // restore target (RTC code will move if required)
        als_iacstep = 32000;
        if (pin_alsout) {
            SSEM0SEI;
            *port_alsout &= ~pin_alsout;
            CSEM0CLI;
        }
        if (ram_window.pg24.als_opt & 0x10) {
            outpc.duty_pwm[5] = 0;
        }
    }

    if (als_state == 0) {
        /* Roving idle / fuel cut when ALS inactive */
        /* fuel cut handled in ign_in only, calculate cyclic fuel cut */
        /* only permitted with a sequential variant on MS3X outputs, would be wholly dangerous
           with batch fire outputs */
        if ((ram_window.pg24.als_opt & 0x20) && (ram4.hardware & HARDWARE_MS3XFUEL) && (ram4.sequential & (SEQ_SEMI | SEQ_FULL))) {
            unsigned int fc_pct;
            fc_pct = intrp_2dctable(outpc.rpm, outpc.tps, 6, 6, &ram_window.pg24.als_rirpms[0],
               &ram_window.pg24.als_ritpss[0], &ram_window.pg24.als_rifuelcut[0][0], 0, 24);
            // rough percentage makes setting easier to comprehend
            fuel_cutx = (unsigned char)(((num_cyl + 1) * fc_pct) / 100);
            fuel_cuty = num_cyl + 1;
            flagbyte10 |= FLAGBYTE10_FUELCUTTMP;
        }
    }
}

void vvt_ctl_pid_init(void)
{
    int i;

    RPAGE = 0xfb;

    vvt_timer = ram_window.pg24.vvt_ctl_ms;
    if (ram_window.pg24.vvt_opt1 & 0x8) {
        flagbyte16 |= FLAGBYTE16_VVT_TIMED;
    }
    vvt_PID_enabled = 0;

    for (i = 0; i < 4; i++) {
        vvt_ctl_last_pv[i][0] = vvt_ctl_last_pv[i][1] = 0;
        vvt_ctl_last_error[i] = 0;
        vvt_last_run[i] = 0;
        if (ram_window.pg24.vvt_out[i] & 0x80) {
            outpc.vvt_duty[i] = 255;
        } else {
        outpc.vvt_duty[i] = 0;
    }
}
}

void vvt_pid(unsigned char numvvt)
{
    int i, hold_duty;
    long tmp1;
    unsigned long lmms_ltch, looptime;
    unsigned char flags[4] = {PID_TYPE_B, PID_TYPE_B, PID_TYPE_B, PID_TYPE_B};
    unsigned char exhaust_or_intake, Kp, Ki, Kd;

    for (i = 0; i < numvvt; i++) {
        if (vvt_run & twopow[i]) {
            DISABLE_INTERRUPTS;
            vvt_run &= ~(twopow[i]);
            lmms_ltch = lmms;
            ENABLE_INTERRUPTS;
            looptime = lmms_ltch - vvt_last_run[i];
            vvt_last_run[i] = lmms_ltch;

            if (ram_window.pg24.vvt_opt5 & (0x10 << i)) {
                exhaust_or_intake = 1;
            } else {
                exhaust_or_intake = 0;
            }

            if (exhaust_or_intake) { /* Exhaust */
                if (ram_window.pg24.vvt_opt2 & 0x4) {
                    hold_duty = ram_window.pg24.vvt_hold_duty_exh;
                } else {
                    hold_duty = -1;
                }
                Kp = ram_window.pg24.vvt_ctl_Kp_exh;
                Ki = ram_window.pg24.vvt_ctl_Ki_exh;
                Kd = ram_window.pg24.vvt_ctl_Kd_exh;
            } else {
                if (ram_window.pg24.vvt_opt2 & 0x2) {
                    hold_duty = ram_window.pg24.vvt_hold_duty;
                } else {
                    hold_duty = -1;
                }
                Kp = ram_window.pg24.vvt_ctl_Kp;
                Ki = ram_window.pg24.vvt_ctl_Ki;
                Kd = ram_window.pg24.vvt_ctl_Kd;
            }

            if (!(outpc.engine & ENGINE_CRANK)) {
                if (!(vvt_PID_enabled & twopow[i]) && outpc.rpm) {
                    vvt_PID_enabled |= twopow[i];
                    flags[i] |= PID_INIT;
                    vvt_duty_store[i] = outpc.vvt_duty[i] * 10000L / 255;
                }
            } else {
                vvt_PID_enabled &= ~twopow[i];
                /* set Duty to 0 to make sure the cam is fully retarded on crank */
                if (ram_window.pg24.vvt_out[i] & 0x80) {
                    outpc.vvt_duty[i] = 255; // have to do this as output polarity is swapped
                } else {
                outpc.vvt_duty[i] = 0;
                }
                return;
            }

            tmp1 = vvt_duty_store[i];

            tmp1 = tmp1 + (generic_pid_routine(0, ram_window.pg24.vvt_max_ang[i] - ram_window.pg24.vvt_min_ang[i],
                                               outpc.vvt_target[i], outpc.vvt_ang[i],
                                               Kp,
                                               Ki,
                                               Kd,
                                               looptime,
                                               vvt_ctl_last_pv[i],
                                               &vvt_ctl_last_error[i],
                                               flags[i]) / 
                                               100L);

            if (hold_duty != -1) {
                if ((outpc.vvt_target[i] / 10) == (outpc.vvt_ang[i] / 10)) {
                    DISABLE_INTERRUPTS;
                    outpc.vvt_duty[i] = hold_duty;
                    vvt_duty_store[i] = (long)hold_duty * 10000/255L;
                    ENABLE_INTERRUPTS;
                    return;
                }
            }


            if (tmp1 < 0) {
                tmp1 = 0;
            }

            if (tmp1 > 10000) {
                tmp1 = 10000;
            }

            vvt_duty_store[i] = tmp1;

            outpc.vvt_duty[i] = (tmp1*255)/10000L; // scale up to 0-255
        }
    }
}

void vvt()
{
    int target[2];
    unsigned char numvvt;
    RPAGE = 0xfb; // HARDCODING for page 24
    // safe to alter RPAGE in mainloop
    vvt_inj_timing_adj = 0; // clear now, will get set if needed

    if (((outpc.engine & ENGINE_READY) == 0) || (outpc.rpm == 0)) {
        int i;
//        outpc.vvt_duty[3] = outpc.vvt_duty[2] = outpc.vvt_duty[1] = outpc.vvt_duty[0] = 0;
        /* Not that simple, have to handle output polarities */
        for (i = 0; i < 4; i++) {
            if (ram_window.pg24.vvt_out[i] & 0x80) {
                outpc.vvt_duty[i] = 255;
            } else {
                outpc.vvt_duty[i] = 0;
            }
        }
        goto VVT_NONRUN;
    }

    numvvt = ram_window.pg24.vvt_opt1 & 0x03;
    if (numvvt) {
        if (numvvt > 2) {
            numvvt = 4;
        }
        target[0] = intrp_2ditable(outpc.rpm, outpc.fuelload, 8, 8,
                               &ram_window.pg24.vvt_timing_rpm[0],
                               &ram_window.pg24.vvt_timing_load[0],
                               (unsigned int *) &ram_window.pg24.vvt_timing[0][0][0], 24);
//        if (numvvt > 1) {
            target[1] = intrp_2ditable(outpc.rpm, outpc.fuelload, 8, 8,
                               &ram_window.pg24.vvt_timing_rpm[0],
                               &ram_window.pg24.vvt_timing_load[0],
                               (unsigned int *) &ram_window.pg24.vvt_timing[1][0][0], 24);
//        }

        if (ram_window.pg24.vvt_opt2 & 0x01) {
            if (ram_window.pg24.vvt_opt5 & 0x04) { // actual
                if (ram_window.pg24.vvt_opt5 & 0x01) {
                    vvt_inj_timing_adj = outpc.vvt_ang[0] - ram_window.pg24.vvt_min_ang[0];
                } else if (ram_window.pg24.vvt_opt5 & 0x02) {
                    vvt_inj_timing_adj = outpc.vvt_ang[1] - ram_window.pg24.vvt_min_ang[1];
                } else {
                    vvt_inj_timing_adj = 0;
                }
            } else { // commanded
                if (ram_window.pg24.vvt_opt5 & 0x01) {
                    vvt_inj_timing_adj = target[0];
                } else if (ram_window.pg24.vvt_opt5 & 0x02) {
                    vvt_inj_timing_adj = target[1];
                } else {
                    vvt_inj_timing_adj = 0;
                }
            }

// VVTs use target[0/1] depending on whether intake or exhaust

            /* convert relative to absolute angles */
            outpc.vvt_target[0] = target[(ram_window.pg24.vvt_opt5 & 0x10) == 0x10];
            if (numvvt > 1) {
                outpc.vvt_target[1] = target[(ram_window.pg24.vvt_opt5 & 0x20) == 0x20];
            } else {
                outpc.vvt_target[1] = 0;
            }
            if (numvvt > 2) {
                outpc.vvt_target[2] = target[(ram_window.pg24.vvt_opt5 & 0x40) == 0x40];
            } else {
                outpc.vvt_target[2] = 0;
            }
            if (numvvt > 3) {
                outpc.vvt_target[3] = target[(ram_window.pg24.vvt_opt5 & 0x80) == 0x80];
            } else {
                outpc.vvt_target[3] = 0;
            }

            vvt_pid(numvvt);
        } else {
            // on/off only - using PWM E table (presently)
            /* is this the best way to handle on/off ?
             * better way might be to use the same normal VVT table and overcome the interpolation problem ??
             * using the same table is more consistent
             */
            unsigned char duty;
            duty = intrp_2dctable(outpc.rpm, outpc.fuelload, 6, 6,
                   &ram_window.pg21.pwm_rpms[4][0],
                   &ram_window.pg21.pwm_loadvals[4][0],
                   &ram_window.pg21.pwm_duties[4][0][0], 0, 21);
 
            if (duty > 60) { // fixed threshold
                outpc.vvt_duty[0] = 255;
                if (ram_window.pg24.vvt_opt1 & 0x80) {
                    vvt_inj_timing_adj = ram_window.pg24.vvt_onoff_ang;
                } else {
                    vvt_inj_timing_adj = 0;
                }
            } else if (duty < 40) {
                outpc.vvt_duty[0] = 0;
                vvt_inj_timing_adj = 0;
            }
        }
VVT_NONRUN:;

        // check test modes
        if (ram_window.pg24.vvt_opt1 & 0x70) {
            outpc.vvt_duty[((ram_window.pg24.vvt_opt1 & 0x70) >> 4) - 1] = ram_window.pg24.vvt_test_duty;
        }

        *port_vvt[0] = outpc.vvt_duty[0];
        *port_vvt[1] = outpc.vvt_duty[1];
        *port_vvt[2] = outpc.vvt_duty[2];
        *port_vvt[3] = outpc.vvt_duty[3];
    }
}

/* Torque convertor lockup
 * written with 700R4 in mind
 */
void tclu()
{
    unsigned char v = 0;
    RPAGE = 0xfb; // HARDCODING for page 24
    // safe to alter RPAGE in mainloop
    if (ram_window.pg24.tclu_outpin & 0x1f) {
            // see if conditions are be met
            if ( (!(((ram_window.pg24.tclu_opt & 0x01) && (outpc.vss1 < ram_window.pg24.tclu_vssmin))
                || ((ram_window.pg24.tclu_opt & 0x02) && (outpc.vss2 < ram_window.pg24.tclu_vssmin))))
                && (!((ram_window.pg24.tclu_opt & 0x04) && (outpc.gear < ram_window.pg24.tclu_gearmin)))
                && (outpc.tps >= ram_window.pg24.tclu_tpsmin)
                && (outpc.tps <= ram_window.pg24.tclu_tpsmax)
                && (outpc.map >= ram_window.pg24.tclu_mapmin)
                && (outpc.map <= ram_window.pg24.tclu_mapmax)
                && ((*port_tcluen & pin_tcluen) == 0)
                && (!(pin_tclubr && ((*port_tclubr & pin_tclubr) == 0)))
                ) {
                v = 1;
            }

        if (tclu_state == 0) { // presently off
            if (v == 1) {
                tclu_timer = ram_window.pg24.tclu_delay;
                tclu_state = 1;
            } else {
                SSEM0SEI;
                *port_tcluout &= ~pin_tcluout; // ensure output is off
                CSEM0CLI;
            }
        } else if (tclu_state == 1) { // pending delay
            if (v == 0) {
                tclu_state = 0; // cancel
            } else if (tclu_timer == 0) {
                tclu_state = 2; // turn it on
                SSEM0SEI;
                *port_tcluout |= pin_tcluout;
                CSEM0CLI;
            }
        } else if (tclu_state == 2) { // locked
            v = 1;
            /* recheck conditions with some hyst */
            if (pin_tclubr && ((*port_tclubr & pin_tclubr) == 0)) { // brake switch cancels immediately
                v = 0;
            }
            if (*port_tcluen & pin_tcluen) { // lack of enable input
                v = 0;
            }
            if ( ((ram_window.pg24.tclu_opt & 0x01) && (outpc.vss1 < (ram_window.pg24.tclu_vssmin - 10)))
                || ((ram_window.pg24.tclu_opt & 0x02) && (outpc.vss2 < (ram_window.pg24.tclu_vssmin -10)))) {
                // 1.0 ms-1 hyst
                v = 0;
            }
            if ((ram_window.pg24.tclu_opt & 0x04) && (outpc.gear < ram_window.pg24.tclu_gearmin)) { // wrong gear
                v = 0;
            }
            if ( (outpc.tps < (ram_window.pg24.tclu_tpsmin - 10))
                || (outpc.tps > (ram_window.pg24.tclu_tpsmax + 10)) ) { // TPS with 1.0% hyst
                v = 0;
            }
            if ( (outpc.map < (ram_window.pg24.tclu_mapmin - 10))
                || (outpc.map > (ram_window.pg24.tclu_mapmax + 10)) ) { // MAP with 1.0% hyst
                v = 0;
            }
            if (v == 0) {
                tclu_state = 0;
                SSEM0SEI;
                *port_tcluout &= ~pin_tcluout; // ensure output is off
                CSEM0CLI;
            }
        }
   }
}

void traction(long *lsum)
{
    /**************************************************************************
     ** Traction control
     **************************************************************************/
    // only present methods are 'perfect run' and 'vss1 vs vss2'
    unsigned int slipxtime, tmp_tc_retard;
    unsigned char tmp_tc_spkcut;
    unsigned int tmp_tc_addfuel;
    unsigned long ul;
 
    slipxtime = 0;
    if (((ram5.tc_opt & 1) == 0) || ((pin_tcenin != 0) && (*port_tcenin & pin_tcenin))) {
        // disabled or enable input on but inactive
        tc_addfuel = 0;
        tc_nitrous = 100;
        tc_boost = 100;
        sliptimer = 0;
        flagbyte10 &= ~FLAGBYTE10_TC_N2O;
        return;
    }
    if (((ram5.tc_opt & 6) == 0) && perfect_timer && (outpc.tps > ram5.tc_mintps) && (outpc.vss1 > ram5.tc_minvss)) { // perfect run method
        unsigned int perfect_vss;
        // see if time within range
        RPAGE = tables[19].rpg;
        if (perfect_timer > ram_window.pg19.tc_perfect_time[9]) {
            perfect_timer = 0; // done
            sliptimer = 0;
        } else {
            perfect_vss = intrp_1ditable(perfect_timer, 10, (int *) ram_window.pg19.tc_perfect_time,
               0, (unsigned int *) ram_window.pg19.tc_perfect_vss, 19);
            if (outpc.vss1 > perfect_vss) {
                slipxtime = ((outpc.vss1 - perfect_vss) * 100) / perfect_vss;
                if (sliptimer == 0) {
                    sliptimer = 1;
                }
            } else {
                sliptimer = 0;
            }
        }
    } else if ((ram5.tc_opt & 6) == 2) { // vss1 vs vss2
        // assumes vss1 is driven, vss2 is undriven
        if ((outpc.vss1 > outpc.vss2) && (outpc.vss1 > ram5.tc_minvss) && (outpc.vss2 > ram5.tc_minvss)
            && (outpc.tps > ram5.tc_mintps) && (outpc.map > ram5.tc_minmap)) {
            long lt;
            unsigned char slipth;
            if (ram5.tc_opt & 0x10) {
                slipth = intrp_1dctable(*port_tc_knob, 9, 
                        (int *) ram_window.pg25.tcslipx, 0, 
                        (unsigned char *)ram_window.pg25.tcslipy, 25);
            } else {
                slipth = ram5.tc_slipthresh;
            }

            lt = ((outpc.vss1 - outpc.vss2) * 100L) / outpc.vss2;
            if (lt > slipth) {
                slipxtime = (unsigned int)lt - slipth;
                if (sliptimer == 0) {
                    sliptimer = 1;
                }
            } else {
                sliptimer = 0;
            }
        } else {
            sliptimer = 0;
        }
    } else if (((ram5.tc_opt & 6) == 4) && perfect_timer && (outpc.tps > ram5.tc_mintps)) { // perfect run RPM method
        unsigned int perfect_rpm;
        // see if time within range
        RPAGE = tables[19].rpg;
        if (perfect_timer > ram_window.pg19.tc_perfect_time[9]) {
            perfect_timer = 0; // done
            sliptimer = 0;
        } else {
            perfect_rpm = intrp_1ditable(perfect_timer, 10, (int *) ram_window.pg19.tc_perfect_time,
               0, (unsigned int *) ram_window.pg19.tc_perfect_rpm, 19);
            if (outpc.rpm > perfect_rpm) {
                slipxtime = ((outpc.rpm - perfect_rpm) * 100) / perfect_rpm;
                if (sliptimer == 0) {
                    sliptimer = 1;
                }
            } else {
                sliptimer = 0;
            }
        }

    } else if ((ram5.tc_opt & 6) == 6) { // switch input
        // External system flags up traction loss using "enable button", so act on it.
        if ((outpc.tps > ram5.tc_mintps) && (outpc.map > ram5.tc_minmap)) {
            slipxtime = 100;
            if (sliptimer == 0) {
                sliptimer = 1;
            }
        }
    }
    ul = slipxtime * (sliptimer / 10);
    if (ul > 0x8000) {
        slipxtime = 0x8000;
    } else {
        slipxtime = (unsigned int)ul;
    }

    if (slipxtime == 0) {
        tc_addfuel = 0;
        tc_nitrous = 100;
        tc_boost = 100;
        tc_boost_duty_delta = 0;
        flagbyte10 &= ~FLAGBYTE10_TC_N2O;
        return;
    }

    // Now figure out what actions to take
    // for all of these a small number in the table means less action.
    tmp_tc_retard = intrp_1ditable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned int *) ram_window.pg19.tc_retard, 19);
    tmp_tc_spkcut = intrp_1dctable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned char *) ram_window.pg19.tc_spkcut, 19);
    tmp_tc_addfuel = (unsigned int)intrp_1dctable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned char *) ram_window.pg19.tc_addfuel, 19);
    tc_nitrous = 100 - intrp_1dctable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned char *) ram_window.pg19.tc_nitrous, 19);
    tc_boost = 100 - intrp_1dctable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned char *) ram_window.pg19.tc_boost, 19);
    tc_boost_duty_delta = intrp_1dctable(slipxtime, 4, (int *) ram_window.pg19.tc_react_x,
           0, (unsigned char *) ram_window.pg19.tc_boost_duty_delta, 19);

    if (tmp_tc_addfuel) {
        /* do multiply/divide and check for overflow */
        unsigned long ultmp_cr;
        ultmp_cr = (unsigned long)tmp_tc_addfuel * ReqFuel;
        if (ultmp_cr > 6500000) {
            tc_addfuel = 65000;
        } else {
            tc_addfuel = (unsigned int)(ultmp_cr / 100);
        }
    } else {
        tc_addfuel = 0;
    }

    if (tmp_tc_spkcut) {
        // rough percentage makes setting easier to comprehend
        spk_cutx = (unsigned char)(((num_cyl + 1) * tmp_tc_spkcut) / 100);
        spk_cuty = num_cyl + 1;
        flagbyte10 |= FLAGBYTE10_SPKCUTTMP; // needed on each pass
    }

    *lsum -= tmp_tc_retard;
    if (*lsum < -100) {
        *lsum = -100;
    }

//    if (ram5.tc_opt & 0x20) - progressive not yet supported, so either nitrous is on (100%) or off.
    if (tc_nitrous < 40) {
        flagbyte10 |= FLAGBYTE10_TC_N2O;
    } else if (tc_nitrous > 60) {
        flagbyte10 &= ~FLAGBYTE10_TC_N2O;
    }

}

void do_spi2(void)
{
}

void ckstall(void)
{
    unsigned long stalltime, stalltime2;

    if (flagbyte2 & flagbyte1_tstmode) {
        return;
    }

    SSEM0SEI;
    stalltime = lmms - ltch_lmms;
    CSEM0CLI;
    if ((outpc.rpm < 3) || (flagbyte2 & flagbyte2_crank_ok)) {
        /* Longer non-running timeout or not yet out of crank mode
            Saves pump bouncing on and off during starting attempt. */
        if (outpc.seconds > (2 + (ram4.primedelay / 10))) {
            stalltime2 = 3906; // 0.5s, was 2s
        } else {
            /* right at start include the primedelay too */
            stalltime2 = 15625 + (781 * ram4.primedelay);
        }

    } else {
        /* quick timeout while running */
        stalltime2 = stall_timeout;
    }

    if (stalltime > stalltime2) {
        if (synch & SYNC_SYNCED) {
            ign_reset();
        }
        clear_all();
    }
}

void clear_all(void)
{
    SSEM0;
    *port_fp &= ~pin_fp;
    CSEM0;   // Turn off fuel Pump
    // idle?
    outpc.engine &= ~1;
    outpc.pw1 = 0;
    outpc.pw2 = 0;
    outpc.pwseq[0] = 0;
    outpc.pwseq[1] = 0;
    outpc.pwseq[2] = 0;
    outpc.pwseq[3] = 0;
    outpc.pwseq[4] = 0;
    outpc.pwseq[5] = 0;
    outpc.pwseq[6] = 0;
    outpc.pwseq[7] = 0;
    outpc.pwseq[8] = 0;
    outpc.pwseq[9] = 0;
    outpc.pwseq[10] = 0;
    outpc.pwseq[11] = 0;
    outpc.pwseq[12] = 0;
    outpc.pwseq[13] = 0;
    outpc.pwseq[14] = 0;
    outpc.pwseq[15] = 0;
    asecount = 0;
    tcrank_done = 0xffff;
    running_seconds = 0;
    flagbyte2 |= flagbyte2_crank_ok;
    if (!(ram4.EAEOption & 0x01)) {
        outpc.EAEfcor1 = 100;
        outpc.EAEfcor2 = 100;
        outpc.wallfuel1 = 0;
        outpc.wallfuel2 = 0;
        WF1 = 0;
        AWA1 = 0;
        SOA1 = 0;
        WF2 = 0;
        AWA2 = 0;
        SOA2 = 0;
    }
}

int abs_int(int in)
{
    if (in < 0) {
        return -in;
    } else {
        return in;
    }
}

void check_sensors(void)
{
    int x;
    unsigned char stat_egt_all = 0;
    #define CK_SENS_SAMPLES 30
    #define CK_SENS_THRESH 10

    RPAGE = RPAGE_VARS1;

    if (outpc.seconds) {
        /* only check after ECU has been on for a second */
        v1.ck_map_sum += abs_int(*mapport - v1.ck_map_last);
        v1.ck_mat_sum += abs_int(ATD0DR1 - v1.ck_mat_last);
        v1.ck_clt_sum += abs_int(ATD0DR2 - v1.ck_clt_last);
        v1.ck_tps_sum += abs_int(ATD0DR3 - v1.ck_tps_last);
        v1.ck_batt_sum += abs_int(outpc.batt - v1.ck_batt_last);
        v1.ck_afr0_sum += abs_int(outpc.afr[0] - v1.ck_afr0_last);

        if (*mapport < ram5.map_minadc) {
            v1.ck_map_min_cnt++;
        } else if (*mapport > ram5.map_maxadc) {
            v1.ck_map_max_cnt++;
        }

        if (ATD0DR1 < ram5.mat_minadc) {
            v1.ck_mat_min_cnt++;
        } else if (ATD0DR1 > ram5.mat_maxadc) {
            v1.ck_mat_max_cnt++;
        }

        if (ATD0DR2 < ram5.clt_minadc) {
            v1.ck_clt_min_cnt++;
        } else if (ATD0DR1 > ram5.clt_maxadc) {
            v1.ck_clt_max_cnt++;
        }

        if (ATD0DR3 < ram5.tps_minadc) {
            v1.ck_tps_min_cnt++;
        } else if (ATD0DR3 > ram5.tps_maxadc) {
            v1.ck_tps_max_cnt++;
        }

        if ((outpc.batt < ram5.batt_minv) && (running_seconds > 5)) { // expect low battery at and immediately after start so wait 5 seconds
            v1.ck_batt_min_cnt++;
        } else if (outpc.batt > ram5.batt_maxv) {
            v1.ck_batt_max_cnt++;
        }

        if (outpc.afr[0] < ram5.afr_min) {
            v1.ck_afr0_min_cnt++;
        } else if (outpc.afr[0] > ram5.afr_max) {
            v1.ck_afr0_max_cnt++;
        }

        if ((ram5.cel_opt2 & 0x80) && ram4.egt_num) {
            int n = ram4.egt_num > NUM_EGT_CHECK ? NUM_EGT_CHECK : ram4.egt_num;
            for (x = 0; x < n ; x++) {
                unsigned long t;
                if ((outpc.egt[x] < ram5.egt_minvalid)
                 && (!((outpc.engine & ENGINE_CRANK) || (flagbyte2 & flagbyte2_crank_ok)))) {
                    /* running minimum will detect dead cylinder too */
                    v1.ck_egt_min_cnt[x]++;
                } else if (outpc.egt[x] > ram5.egt_maxvalid) {
                    v1.ck_egt_max_cnt[x]++;
                }
                t = v1.ck_egt_sum[x] + abs_int(outpc.egt[x] - v1.ck_egt_last[x]);
                if (t > 65535) {
                    t = 65535;
                }
                v1.ck_egt_sum[x] = t;
            }
        }
    }

    /* Always populate these here so sensible data is ready for first pass.*/

    /* use raw data so we can check sensor health even in fallback */
    v1.ck_map_last = *mapport;
    v1.ck_mat_last = ATD0DR1;
    v1.ck_clt_last = ATD0DR2;
    v1.ck_tps_last = ATD0DR3;

    v1.ck_batt_last = outpc.batt;
    v1.ck_afr0_last = outpc.afr[0];
    if ((ram5.cel_opt2 & 0x80) && ram4.egt_num) {
        int n = ram4.egt_num > NUM_EGT_CHECK ? NUM_EGT_CHECK : ram4.egt_num;
        for (x = 0; x < n ; x++) {
            v1.ck_egt_last[x] = outpc.egt[x];
        }
    }

    v1.ck_cnt++;
    if (v1.ck_cnt >= CK_SENS_SAMPLES) {
        outpc.cel_status = 0; // start with zero and set shortly.

        /* process data */
        v1.ck_cnt = 0;

        /* validation allows 8 samples out of range */
        /* MAP */
        stat_map <<= 1;
        if ((ram5.cel_opt2 & 0x01)
            && ( ((running_seconds > ram5.cel_runtime) && ((v1.ck_map_sum > ram5.map_var_upper) || (v1.ck_map_sum < ram5.map_var_lower)))
            || (v1.ck_map_min_cnt > CK_SENS_THRESH)
            || (v1.ck_map_max_cnt > CK_SENS_THRESH)) ) {
            stat_map |= 1;
        }

        /* MAT */
        stat_mat <<= 1;
        if ((ram5.cel_opt2 & 0x02)
            && ( ((running_seconds > ram5.cel_runtime) && (v1.ck_mat_sum > ram5.mat_var_upper))
            || (v1.ck_mat_min_cnt > CK_SENS_THRESH)
            || (v1.ck_mat_max_cnt > CK_SENS_THRESH)) ) {
            stat_mat |= 1;
        }

        /* CLT */
        stat_clt <<= 1;
        if ((ram5.cel_opt2 & 0x04)
            && ( ((running_seconds > ram5.cel_runtime) && (v1.ck_clt_sum > ram5.clt_var_upper))
            || (v1.ck_clt_min_cnt > CK_SENS_THRESH)
            || (v1.ck_clt_max_cnt > CK_SENS_THRESH)) ) {
            stat_clt |= 1;
        }

        /* TPS */
        stat_tps <<= 1;
        if ((ram5.cel_opt2 & 0x08)
            && ( ((running_seconds > ram5.cel_runtime) && (v1.ck_tps_sum > ram5.tps_var_upper))
            || (v1.ck_tps_min_cnt > CK_SENS_THRESH)
            || (v1.ck_tps_max_cnt > CK_SENS_THRESH)) ) {
            stat_tps |= 1;
        }

        /* Batt */
        stat_batt <<= 1;
        if ((ram5.cel_opt2 & 0x10)
            && ( ((running_seconds > ram5.cel_runtime) && (v1.ck_batt_sum > ram5.batt_var_upper))
            || (v1.ck_batt_min_cnt > CK_SENS_THRESH)
            || (v1.ck_batt_max_cnt > CK_SENS_THRESH)) ) {
            stat_batt |= 1;
        }

        /* AFR0 */
        stat_afr0 <<= 1;
        if ((ram5.cel_opt2 & 0x20)
            && ( ((running_seconds > ram5.cel_runtime) && ((v1.ck_afr0_sum > ram5.afr_var_upper) || (v1.ck_afr0_sum < ram5.afr_var_lower)))
            || (v1.ck_afr0_min_cnt > CK_SENS_THRESH)
            || (v1.ck_afr0_max_cnt > CK_SENS_THRESH)) ) {
            stat_afr0 |= 1;
        }

        /* EGTs */
        stat_egt_all = 0;
        if ((ram5.cel_opt2 & 0x80) && ram4.egt_num) {
            int n = ram4.egt_num > NUM_EGT_CHECK ? NUM_EGT_CHECK : ram4.egt_num;
            for (x = 0; x < n ; x++) {
                stat_egt[x] <<= 1;
                /* no checks at all during initial period to allow engine to warmup */
                if ((running_seconds > ram5.cel_runtime) && ((v1.ck_egt_sum[x] > ram5.egt_var_upper) || (v1.ck_egt_sum[x] < ram5.egt_var_lower)
                    || (v1.ck_egt_min_cnt[x] > CK_SENS_THRESH)
                    || (v1.ck_egt_max_cnt[x] > CK_SENS_THRESH))) {
                    stat_egt[x] |= 1;
                }
                if (stat_egt[x]) {
                    stat_egt_all = 1;
                    outpc.cel_status |= 0x80;
                }
            }
        }

        stat_sync <<= 1;
        if (outpc.engine & ENGINE_CRANK) {
            if ((outpc.synccnt - v1.ck_sync_last) > 5) { // arbitrary 5 sync loss limit during cranking only
                stat_sync |= 1;
            }
        } else if ((!(outpc.engine & ENGINE_CRANK)) && (flagbyte2 & flagbyte2_crank_ok) && (v1.ck_sync_last == 0)) {
            v1.ck_sync_last = 1 + outpc.synccnt; // during crank to run period, ignore any sync losses and reset last counter
        } else if ((outpc.engine & ENGINE_READY) && ((outpc.synccnt + 1 - v1.ck_sync_last) > ram5.cel_synctol)) { // sync loss during running
            stat_sync |= 1;
        }

        stat_flex <<= 1; // this is checked and set in Flex code
        stat_maf <<= 1; // this is set in MAF code
        stat_knock <<= 1; // this is set in Knock code

        /* CEL is only a warning */
        if ((outpc.engine & ENGINE_READY)
            && (stat_map || stat_mat || stat_clt || stat_tps || stat_batt || stat_afr0 || stat_sync
            ||  stat_egt_all || stat_flex || stat_maf || stat_knock
            || (outpc.status6 & STATUS6_EGTSHUT) || (outpc.status6 & STATUS6_AFRSHUT) )) {
            if (pin_cel) {
                SSEM0SEI;
                *port_cel |= pin_cel;
                CSEM0CLI;
            }
            outpc.status7 |= STATUS7_CEL;
        } else {
            if (pin_cel) {
                SSEM0SEI;
                *port_cel &= ~pin_cel;
                CSEM0CLI;
            }
            outpc.status7 &= ~STATUS7_CEL;
        }

        /* Limp mode is an action. Validate which faults trigger limp mode */
        if ((outpc.engine & ENGINE_READY)
            && ((stat_map && (ram5.cel_action1 & 0x01))
             || (stat_mat && (ram5.cel_action1 & 0x02))
            || (stat_clt && (ram5.cel_action1 & 0x04))
            || (stat_tps && (ram5.cel_action1 & 0x08))
            || (stat_batt && (ram5.cel_action1 & 0x10))
            || (stat_afr0 && (ram5.cel_action1 & 0x20))
            || (stat_sync && (ram5.cel_action1 & 0x40))
            || (stat_flex && (ram5.cel_action2 & 0x01))
            || (stat_egt_all && (ram5.cel_action2 & 0x02))
            )) {
            outpc.status7 |= STATUS7_LIMP;
        } else {
            outpc.status7 &= ~STATUS7_LIMP;
        }

        /* monitor */
//    cel_opt_stat =  bits, 274, [1:3], "Off", "MAP", "MAT", "CLT", "TPS", "Batt", "EGO", "EGT1"
        if ((ram5.cel_opt & 0x0e) == 0x02) {
            outpc.istatus5 = v1.ck_map_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x04) {
            outpc.istatus5 = v1.ck_mat_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x06) {
            outpc.istatus5 = v1.ck_clt_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x08) {
            outpc.istatus5 = v1.ck_tps_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x0a) {
            outpc.istatus5 = v1.ck_batt_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x0c) {
            outpc.istatus5 = v1.ck_afr0_sum;
        } else if ((ram5.cel_opt & 0x0e) == 0x0e) {
            outpc.istatus5 = v1.ck_egt_sum[0];
        }

        /* Let the user know */
        if (stat_map) {
            outpc.cel_status |= 1;
        }
        if (stat_mat) {
            outpc.cel_status |= 2;
        }
        if (stat_clt) {
            outpc.cel_status |= 4;
        }
        if (stat_tps) {
            outpc.cel_status |= 8;
        }
        if (stat_batt) {
            outpc.cel_status |= 0x10;
        }
        if (stat_afr0) {
            outpc.cel_status |= 0x20;
        }
        if (stat_sync) {
            outpc.cel_status |= 0x40;
        }
        /* EGT status set above */
        if (stat_flex) {
            outpc.cel_status |= 0x100;
        }
        if (stat_maf) {
            outpc.cel_status |= 0x200;
        }
        if (stat_knock) {
            outpc.cel_status |= 0x400;
        }
        /* reset counters */
        check_sensors_reset();
    }

    return;
}

void check_sensors_reset(void)
{
    int x;

    RPAGE = RPAGE_VARS1;

    v1.ck_cnt = 0;
    v1.ck_map_sum = 0;
    v1.ck_mat_sum = 0;
    v1.ck_clt_sum = 0;
    v1.ck_tps_sum = 0;
    v1.ck_afr0_sum = 0;
    v1.ck_batt_sum = 0;
    v1.ck_map_min_cnt = 0;
    v1.ck_mat_min_cnt = 0;
    v1.ck_clt_min_cnt = 0;
    v1.ck_tps_min_cnt = 0;
    v1.ck_afr0_min_cnt = 0;
    v1.ck_batt_min_cnt = 0;
    v1.ck_map_max_cnt = 0;
    v1.ck_mat_max_cnt = 0;
    v1.ck_clt_max_cnt = 0;
    v1.ck_tps_max_cnt = 0;
    v1.ck_afr0_max_cnt = 0;
    v1.ck_batt_max_cnt = 0;
    {
        int n = ram4.egt_num > NUM_EGT_CHECK ? NUM_EGT_CHECK : ram4.egt_num;
        for (x = 0 ; x < n ; x++) {
            v1.ck_egt_sum[x] = 0;
            v1.ck_egt_min_cnt[x] = 0;
            v1.ck_egt_max_cnt[x] = 0;
        }
    }
}

void check_sensors_init(void)
{
    int x;
    check_sensors_reset();
    stat_map = 0;
    stat_mat = 0;
    stat_clt = 0;
    stat_tps = 0;
    stat_afr0 = 0;
    stat_batt = 0;
    stat_sync = 0;
    stat_flex = 0;
    stat_maf = 0;
    stat_knock = 0;
    {
        int n = ram4.egt_num > NUM_EGT_CHECK ? NUM_EGT_CHECK : ram4.egt_num;
        for (x = 0 ; x < n ; x++) {
            stat_egt[x] = 0;
        }
    }
    v1.ck_sync_last = outpc.synccnt;
}

/* The following variable is a global local, it should be the only one and as
   a result will be the last variable allocated, so most exposed to the stack. */
unsigned int stack_watch_var;

void stack_watch_init(void)
{
    stack_watch_var = 0;
}

void stack_watch(void)
{
    if (stack_watch_var != 0) {
        conf_err = 153;
    }
}

int blend_xaxis(unsigned char opt)
{
    if (opt == 0) {
        return outpc.tps / 10;
    } else if (opt == 1) {
        return outpc.map / 10;
    } else if (opt == 2) {
        return outpc.rpm;
    } else if (opt == 3) {
        return outpc.mafload / 10;
    } else if (opt == 4) {
        return outpc.fuel_pct / 10;
    } else if (opt == 5) {
        return outpc.vss1 / 10;
    } else if (opt == 6) {
        return outpc.gear;
    } else if (opt == 7) {
        return (int)(((long)outpc.fuelload * outpc.rpm) / 1000L);
    } else if ((opt >= 16) && (opt <= 31)) {
        return outpc.sensors[opt - 16] / 10;
    } else {
        return 0;
    }
}
