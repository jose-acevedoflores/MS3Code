/* $Id: ms3_idle.c,v 1.89.2.1 2013/04/19 14:32:12 culverk Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * idle_ac_idleup()
    Majority: Kenneth Culver
 * fan_ctl_idleup()
    Majority: Kenneth Culver
 * idle_ctl_init
    Majority: Kenneth Culver
 * idle_test_mode
    Majority: James Murray
 * idle_on_off
    Origin: Al Grippo
    Moderate: MS3 hardware. James Murray
    Majority: Al Grippo / James Murray
 * idle_iac_warmup
    Origin: Al Grippo
    Minor: Fan. Kenneth Culver
    Majority: Al Grippo
 * idle_pwm_warmup
    Origin: Al Grippo
    Minor: Fan. Kenneth Culver
    Majority: Al Grippo
 * idle_closed_loop_throttlepressed
    Majority: Kenneth Culver
 * idle_closed_loop_throttlelifted
    Majority: Kenneth Culver
 * idle_closed_loop_pid
    Majority: Kenneth Culver
 * idle_closed_loop_newtarg
    Majority: Kenneth Culver
 * idle_closed_loop
    Majority: Kenneth Culver
 * idle_ctl
    Majority: Kenneth Culver
 * move_IACmotor
    Origin: Al Grippo
    Minor: MS3 hardware. James Murray. Bug Fixes (from ms2/extra) Kenneth Culver
    Majority: Al Grippo
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/

#include "ms3.h"
#define LAST_DIRECTION_UNDEF 0
#define LAST_DIRECTION_CLOSE 1
#define LAST_DIRECTION_OPEN  2

#define PWMIDLE_RESET_PID         0x1 
#define PWMIDLE_RESET_JUSTLIFTED  0x2 
#define PWMIDLE_RESET_DONOTHING   0x4 
#define PWMIDLE_RESET_JUSTCRANKED 0x8 
#define PWMIDLE_RESET_CALCNEWTARG 0x10 
#define PWMIDLE_RESET_DPADDED     0x20

void idle_voltage_compensation(void)
{
    idle_voltage_comp = intrp_1ditable(outpc.batt, 6, ram5.idle_voltage_comp_voltage,
                                       1, ram5.idle_voltage_comp_delta, 5);
}

void idle_ac_idleup(void)
{
    if (ram4.ac_idleup_settings & 0x80) {

        if(outpc.engine & ENGINE_CRANK) {
            SSEM0SEI;
            *port_ac_out &= ~pin_ac_out;
            CSEM0CLI;
            ac_time_since_last_on = 0;
            return;
        }

        /* Check AC idleup input pin */
        if ((*port_ac_in & pin_ac_in) == pin_match_ac_in) {

            /* If AC on, check for TPS/VSS shutoff */
            if (flagbyte16 & FLAGBYTE16_AC_ENABLE) {
                unsigned char ac_disable = 0;
                /* presently on */
                if (outpc.tps > ram4.ac_idleup_tps_offpoint) {
                    flagbyte16 |= FLAGBYTE16_AC_TPSHYST;
                    ac_disable = 1;
                } else if ((flagbyte16 & FLAGBYTE16_AC_TPSHYST) 
                    && (outpc.tps > (ram4.ac_idleup_tps_offpoint - ram4.ac_idleup_tps_hyst))) {
                    ac_disable = 1;
                } else {
                    flagbyte16 &= ~FLAGBYTE16_AC_TPSHYST;
                }

                if ((ram4.vss_opt & 0x0f) || (ram4.vss1_an & 0x1f)) {
                    /* same thing as above for VSS */
                    if (outpc.vss1 >= ram4.ac_idleup_vss_offpoint) {
                        flagbyte16 |= FLAGBYTE16_AC_VSSHYST;
                        ac_disable = 1;
                    } else if ((flagbyte16 & FLAGBYTE16_AC_VSSHYST)
                        && (outpc.vss1 >= (ram4.ac_idleup_vss_offpoint - ram4.ac_idleup_vss_hyst))) {
                            ac_disable = 1;
                    } else {
                        flagbyte16 &= ~FLAGBYTE16_AC_VSSHYST;
                    }
                }

                if (ac_disable) {
                    if (outpc.status7 & STATUS7_ACOUT) {
                        ac_time_since_last_on = 0;
                    }
                    SSEM0SEI;
                    *port_ac_out &= ~pin_ac_out;
                    CSEM0CLI;
                    outpc.status7 &= ~STATUS7_ACOUT;
                    return;
                }
            }

            if ( (outpc.tps < ram4.ac_idleup_tps_offpoint)
                && (!((ram4.vss_opt & 0x0F) && (outpc.vss1 > ram4.ac_idleup_vss_offpoint)))
                && (ac_time_since_last_on > ram4.ac_delay_since_last_on) ) {
                /* button pressed and conditions met, start timer */
                if (last_acbutton_state == 0) {
                    last_acbutton_state = 1;
                    ac_idleup_timer = 0;
                    ac_idleup_adder = ram4.ac_idleup_adder;
                    ac_idleup_mapadder = ram4.ac_idleup_cl_lockout_mapadder;
                    ac_idleup_cl_targetadder = ram4.ac_idleup_cl_targetadder;
                    flagbyte16 &= ~FLAGBYTE16_AC_TPSHYST;
                    flagbyte16 &= ~FLAGBYTE16_AC_VSSHYST;
                    return;
                }

                if (ac_idleup_timer >= ram4.ac_idleup_delay) {
                    SSEM0SEI;
                    *port_ac_out |= pin_ac_out;
                    CSEM0CLI;
                    outpc.status7 |= STATUS7_ACOUT;
                    flagbyte16 |= FLAGBYTE16_AC_ENABLE;
                }
            }
        } else {
            if (last_acbutton_state == 1) {
                last_acbutton_state = 0;
                ac_idleup_timer = 0;
                ac_idleup_adder = 0;
                ac_idleup_mapadder = 0;
                ac_idleup_cl_targetadder = 0;
                ac_time_since_last_on = 0;
                return;
            }

            if (ac_idleup_timer >= ram4.ac_idleup_delay) {
                SSEM0SEI;
                *port_ac_out &= ~pin_ac_out;
                CSEM0CLI;
                outpc.status7 &= ~STATUS7_ACOUT;
                flagbyte16 &= ~FLAGBYTE16_AC_ENABLE;
            }
        }
    }
}

void fan_ctl_idleup(void)
{
    unsigned char fan_disable = 0;
         
    if (ram4.fanctl_settings & 0x80) {
        if ((ram4.fanctl_opt2 & 0x01) == 0) { 
            if ((outpc.engine & ENGINE_READY) == 0) { // don't run when engine stalled
                SSEM0SEI;
                *port_fanctl_out &= ~pin_fanctl_out;
                CSEM0CLI;
                outpc.status6 &= ~STATUS6_FAN;
                return;
            }
        } else { // do
            if ((outpc.engine & ENGINE_CRANK) || (outpc.batt < 100)) {
                SSEM0SEI;
                *port_fanctl_out &= ~pin_fanctl_out; // but off when cranking
                CSEM0CLI;
                outpc.status6 &= ~STATUS6_FAN;
                return;
            }
        }

        if (flagbyte16 & FLAGBYTE16_FAN_ENABLE) {
            /* presently on */
            if (outpc.tps > ram4.fan_idleup_tps_offpoint) {
                flagbyte16 |= FLAGBYTE16_FAN_TPSHYST;
                fan_disable = 1;
            } else if ((flagbyte16 & FLAGBYTE16_FAN_TPSHYST) 
                && (outpc.tps > (ram4.fan_idleup_tps_offpoint - ram4.fan_idleup_tps_hyst))) {
                fan_disable = 1;
            } else {
                flagbyte16 &= ~FLAGBYTE16_FAN_TPSHYST;
            }

            if ((ram4.vss_opt & 0x0f) || (ram4.vss1_an & 0x1f)) {
                /* same thing as above for VSS */
                if (outpc.vss1 >= ram4.fan_idleup_vss_offpoint) {
                    flagbyte16 |= FLAGBYTE16_FAN_VSSHYST;
                    fan_disable = 1;
                } else if ((flagbyte16 & FLAGBYTE16_FAN_VSSHYST)
                    && (outpc.vss1 >= (ram4.fan_idleup_vss_offpoint - ram4.fan_idleup_vss_hyst))) {
                    fan_disable = 1;
                } else {
                    flagbyte16 &= ~FLAGBYTE16_FAN_VSSHYST;
                }
            }

            if (fan_disable) {
                SSEM0SEI;
                *port_fanctl_out &= ~pin_fanctl_out;
                CSEM0CLI;
                outpc.status6 &= ~STATUS6_FAN;
            }
        }

        /* Is CLT above threshold? */
        /* For AC button, only turn on fan if the user wants the fan on with AC */
        if ((outpc.clt >= ram4.fanctl_ontemp) || ((last_acbutton_state == 1) && 
                                                   ram4.fan_ctl_settings2 & 0x01)) {
            /* Not disabled, so start timers and such. */
            if (last_fan_state == 0) {
                last_fan_state = 1;
                fan_idleup_timer = 0;
                if ((ram4.fanctl_settings & 0x40) || (ram4.ac_idleup_settings & 0x80)) {
                    fan_idleup_adder = ram4.fanctl_idleup_adder;
                    fan_idleup_cl_targetadder = ram4.fan_idleup_cl_targetadder;
                    flagbyte16 &= ~FLAGBYTE16_FAN_TPSHYST;
                    flagbyte16 &= ~FLAGBYTE16_FAN_VSSHYST;
                }
                return;
            }

            if ((fan_idleup_timer >= ram4.fanctl_idleup_delay) && (!fan_disable)) {
                SSEM0SEI;
                *port_fanctl_out |= pin_fanctl_out;
                CSEM0CLI;
                outpc.status6 |= STATUS6_FAN;
                flagbyte16 |= FLAGBYTE16_FAN_ENABLE;
            }
        }

        if ((outpc.clt <= ram4.fanctl_offtemp) && ((last_acbutton_state == 0) || (!(ram4.fan_ctl_settings2 & 0x01)))) {
            /* cool enough and AC off if linked to fan */
            if (last_fan_state == 1) {
                last_fan_state = 0;
                fan_idleup_timer = 0;
                if (ram4.fanctl_settings & 0x40) {
                    fan_idleup_adder = 0;
                    fan_idleup_cl_targetadder = 0;
                }
                return;
            }

            if (fan_idleup_timer >= ram4.fanctl_idleup_delay) {
                SSEM0SEI;
                *port_fanctl_out &= ~pin_fanctl_out;
                CSEM0CLI;
                outpc.status6 &= ~STATUS6_FAN;
                flagbyte16 &= ~FLAGBYTE16_FAN_ENABLE;
            }
        }
    }
}

void idle_ctl_init(void)
{
    IACmotor_reset = 2;
    pwmidle_stepsize = 0;
    pwmidle_numsteps = 0;
    pwmidle_targ_stepsize = 0;
    pwmidle_targ_numsteps = 0;
    pwmidle_targ_last = 0;
    PV_last[0] = PV_last[1] = 0;
    last_direction = LAST_DIRECTION_UNDEF;
    ac_idleup_adder = last_acbutton_state = 0;
    fan_idleup_adder = last_fan_state = 0;
    ac_idleup_mapadder = 0;
    ac_idleup_timer = fan_idleup_timer = 0;
    idle_wait_timer = ram4.pwmidle_pid_wait_timer;
    pwmidle_shift_timer = ram4.pwmidle_shift_open_time;
    idle_voltage_comp = 0;
}

void idle_test_mode(void)
{
    int pos_tmp;
    pos_tmp = ram5.iacpostest;  // adding this temp var halves the code size for this if {} section
    //for PWM valves, scale the result
    if ((ram4.IdleCtl & 0xd) == 4) {   // 4 or 6
        if (pos_tmp > 100) {
            pos_tmp = 100;
        }
        pos_tmp = (pos_tmp * 256) / 100;        // means user enters 0-100% // sensible code with temp var, crap without
        if (pos_tmp > 255) {
            pos_tmp = 255;
        }
        IACmotor_pos = pos_tmp;
    } else {
        IACmotor_pos = pos_tmp;
    }

    if (iactest_glob == 1) {   // home feature
        iactest_glob = 2; // done it
        if ((IdleCtl == 2) || (IdleCtl == 3) ||
            (IdleCtl == 5) || (IdleCtl == 7) || (IdleCtl == 8)) {
            DISABLE_INTERRUPTS;
            IAC_moving = 0;
            motor_step = -1;
            IACmotor_reset = 0;
            IACmotor_pos = 0;
            outpc.iacstep = ram5.iachometest;   // set current motor step position
            ENABLE_INTERRUPTS;
            (void) move_IACmotor();
        }
    } else if (iactest_glob == 3) { // run mode
        if ((outpc.iacstep != IACmotor_pos)) {
            // move IAC motor to new step position
            move_IACmotor();
        }
    }
}

void idle_on_off(void)
{
    if (outpc.clt < (ram4.FastIdle - ram4.IdleHyst)) {
        SSEM0SEI;
        *port_idleonoff |= pin_idleonoff;
        CSEM0CLI;
    } else if (outpc.clt > ram4.FastIdle) {
        SSEM0SEI;
        *port_idleonoff &= ~pin_idleonoff;
        CSEM0CLI;
    }
}

void idle_iac_warmup(void)
{
    int tmp1, tmp2;
    unsigned int pos;

    pos =
        intrp_1ditable(outpc.clt, 4,
                       (int *) ram_window.pg8.pwmidle_crank_clt_temps, 0,
                       (unsigned int *) ram_window.
                       pg8.pwmidle_crank_dutyorsteps, 8);

    // In cranking mode. The very first time we crank we want to
    //  open to crankpos, but after cranking we don't want to do
    // this because of an ignition reset (in which rpm = 0)
    if ((flagbyte3 & flagbyte3_firstIAC) && (outpc.engine & ENGINE_CRANK)) {
        tmp1 = pos;
    } else {
        // after cranking flare back to normal temperature dependent pos
        if ((outpc.seconds >= tcrank_done) &&
            (outpc.seconds <= tcrank_done + ram4.IACcrankxt)) {
            // 0 steps is fully closed (fast idle) and IACStart steps is fully closed
            tmp1 =
                CW_table(outpc.clt, (int *) ram_window.pg10.iacstep_table,
                        (int *) ram_window.pg10.temp_table, 10);
            if (tmp1 <= pos) {
                tmp2 = (int) ((long) (pos - tmp1) *
                        (tcrank_done + ram4.IACcrankxt - outpc.seconds) /
                        ram4.IACcrankxt);
                tmp1 += tmp2;
            }
            tble_motor_pos = tmp1;
            flagbyte3 &= ~flagbyte3_firstIAC;

            //  switch to time based control if cold enough at startup
        } else if ((outpc.seconds >= tcold_pos) &&
                (outpc.seconds <= tcold_pos + ram4.IACcoldxt)) {
            tmp1 =
                CW_table(outpc.clt, (int *) ram_window.pg10.iacstep_table,
                        (int *) ram_window.pg10.temp_table, 10);
            if (tmp1 <= ram4.IACcoldpos) {
                RPAGE = tables[10].rpg;     //HARDCODED
                tmp2 = (int) ((long)
                        (ram_window.pg10.iacstep_table[NO_TEMPS - 1] -
                         ram4.IACcoldpos)
                        * (tcold_pos + ram4.IACcoldxt -
                            outpc.seconds) / ram4.IACcoldxt);
                tmp1 = ram_window.pg10.iacstep_table[NO_TEMPS - 1] - tmp2;
            }
            tble_motor_pos = tmp1;

            // check if there has been a significant change in clt temp
        } else if ((outpc.clt < last_iacclt - ram4.IdleHyst) ||
                (outpc.clt > last_iacclt)) {
            tble_motor_pos =
                CW_table(outpc.clt, (int *) ram_window.pg10.iacstep_table,
                        (int *) ram_window.pg10.temp_table, 10);
        }
        tmp1 = tble_motor_pos + datax1.IdleAdj; // from remote CAN device
        if (tmp1 < 0)
            tmp1 = 0;

        if (ac_idleup_adder > 0) {
            tmp1 += ac_idleup_adder;
        }
        if (fan_idleup_adder > 0) {
            tmp1 += fan_idleup_adder;
        }
    }
    IACmotor_pos = tmp1;

    if (outpc.iacstep != IACmotor_pos) {
        // move IAC motor to new step position
        if (IdleCtl != 4) {
            if (move_IACmotor())
                last_iacclt = outpc.clt;
        } else {
            (void)move_IACmotor();
            last_iacclt = outpc.clt;
        }
    }
    // check if/ when to start extended time-based idle control
    if (start_clt < ram4.IACcoldtmp) {
        if ((outpc.seconds > tcrank_done) && (tcold_pos == 0xFFFF) &&
            (IACmotor_pos > ram4.IACcoldpos))
            tcold_pos = outpc.seconds;
    }
    if ((IdleCtl == 5) && (outpc.seconds > 900)) {
        IdleCtl = 2;
        if (!IAC_moving)
            SSEM0;
            PTJ |= 0x40;        // disable current to stepper motor(bit=1)
            CSEM0;
    }
}

void idle_pwm_warmup(void)
{
    int tmp1, tmp2;
    unsigned int pos;

    pos =
        intrp_1ditable(outpc.clt, 4,
                       (int *) ram_window.pg8.pwmidle_crank_clt_temps, 0,
                       (unsigned int *) ram_window.
                       pg8.pwmidle_crank_dutyorsteps, 8);

    //pwmidle warmup only simplified
    if (outpc.engine & ENGINE_CRANK) {
        DISABLE_INTERRUPTS;
        IACmotor_pos = pos;
        ENABLE_INTERRUPTS;
    } else {
        // after cranking taper back to normal temperature dependent pos
        // 0% duty is fully closed
        RPAGE = tables[10].rpg; //HARDCODED
        if (outpc.clt < ram_window.pg10.temp_table[0]) {
            tmp2 = ram_window.pg10.pwmidle_table[0];
        } else if (outpc.clt > ram_window.pg10.temp_table[9]) {
            tmp2 = ram_window.pg10.pwmidle_table[9];
        } else {
            tmp2 =
                CW_table(outpc.clt, (int *) ram_window.pg10.pwmidle_table,
                         (int *) ram_window.pg10.temp_table, 10);
        }

        if ((outpc.seconds >= tcrank_done) &&
            (outpc.seconds <= tcrank_done + ram4.pwmidlecranktaper)) {
            if (tmp2 <= pos) {
                tmp1 = (int) ((long) (pos - tmp2) *
                        (tcrank_done + ram4.pwmidlecranktaper - outpc.seconds) /
                        ram4.pwmidlecranktaper);
                tmp2 += tmp1;
            }
        }

        if (ac_idleup_adder > 0) {
            tmp2 += ac_idleup_adder;
        }
        if (fan_idleup_adder > 0) {
            tmp2 += fan_idleup_adder;
        }
        tmp2 += idle_voltage_comp;

        //check range
        DISABLE_INTERRUPTS;
        if (tmp2 > 255) {
            IACmotor_pos = 255;
        } else if (tmp2 < 0) {
            IACmotor_pos = 0;
        } else {
            IACmotor_pos = tmp2;
        }
        ENABLE_INTERRUPTS;
        (void)move_IACmotor();
    }
}

#define VALVE_CLOSED 1
#define VALVE_CLOSED_PARTIAL 2

void idle_closed_loop_throttlepressed(int rpm_thresh)
{
    int pos = IACmotor_pos;

    outpc.status2 &= ~STATUS2_CLIDLE;

    if (outpc.rpm > rpm_thresh) {       /* should we close the valve? */
        if (IACmotor_pos_tmp == IACmotor_last) {
            /* IF so, the number of steps is the delay in ms divided by how 
             * often we will run, and the step size is the amount to move total
             * divided by the number of steps
             */
            if (ram4.pwmidle_close_delay) {
                pwmidle_numsteps =
                    ((long) (ram4.pwmidle_close_delay * 1000)) /
                    ram4.pwmidle_ms;
                if (pwmidle_numsteps > 0) {
                    pwmidle_stepsize = (IACmotor_pos - ram4.pwmidle_closed_duty) /
                        pwmidle_numsteps;
                    if (pwmidle_stepsize == 0) {
                        pwmidle_stepsize = 1;
                    }
                } else {
                    pos = ram4.pwmidle_closed_duty;
                    valve_closed = VALVE_CLOSED;
                }

                if (pwmidle_stepsize == 0) {
                    pwmidle_stepsize = 1;
                }
            } else {
                pos = IACmotor_last;
                valve_closed = VALVE_CLOSED;
            }
        }
        if (flagbyte2 & flagbyte2_runidle) {
            if ((pos > ram4.pwmidle_closed_duty)) {
                pos -= pwmidle_stepsize;
                valve_closed = VALVE_CLOSED;
            }
            if (pos <= ram4.pwmidle_closed_duty) {
                pos = ram4.pwmidle_closed_duty;
            }

            DISABLE_INTERRUPTS;
            flagbyte2 &= ~flagbyte2_runidle;
            IACmotor_pos = pos;
            ENABLE_INTERRUPTS;
            IACmotor_pos_tmp = pos;
        }
    }

    pwmidle_reset = PWMIDLE_RESET_JUSTLIFTED;

    idle_wait_timer = 0;
    pwmidle_shift_timer = 0;
}

void idle_closed_loop_throttlelifted(int rpm_thresh, int rpm_targ)
{
    int y_val_lookup;
    char tmp = 0;

    PV_last[0] = PV_last[1] = 0;

    if (ram4.pwmidle_cl_opts & PWMIDLE_CL_USE_INITVALUETABLE) {
        if (ram4.pwmidle_cl_opts & PWMIDLE_CL_INITVAL_CLT) {
            y_val_lookup = outpc.clt;
        } else {
            y_val_lookup = outpc.mat;
        }
        IACmotor_last = intrp_2dctable(rpm_targ, y_val_lookup, 5, 5,
                                       &ram_window.pg19.pwmidle_cl_initialvalue_rpms[0],
                                       &ram_window.pg19.pwmidle_cl_initialvalue_matorclt[0],
                                       &ram_window.pg19.pwmidle_cl_initialvalues[0][0],
                                       0, 19);
    }

    if (((!ram4.pwmidle_close_delay) ||
        (outpc.rpm <= ram4.pwmidle_shift_lower_rpm) ||
        (pwmidle_shift_timer > ram4.pwmidle_shift_open_time)) && 
        (!(pwmidle_reset & PWMIDLE_RESET_DPADDED))) {

        if (valve_closed == VALVE_CLOSED) {
            IACmotor_pos_tmp = IACmotor_last + ram4.pwmidle_dp_adder;
            tmp = PWMIDLE_RESET_DPADDED;
        } else {
            IACmotor_pos_tmp = IACmotor_last;
        }
    }

    if (IACmotor_pos_tmp > (int) ram4.pwmidle_open_duty) {
        IACmotor_pos_tmp = ram4.pwmidle_open_duty;
    }

    /* Using VSS for Idle speed control?
     * Wait till it says 0 and then go into PID
     */
    if (ram4.IdleCtl & 0x10) {
        if (outpc.vss1 == 0) {
            idle_wait_timer = 0;
            pwmidle_reset = PWMIDLE_RESET_CALCNEWTARG;
        }
        goto throttlelifted_end;
    }

    /* if rpm is below rpm threshold, just enable PID */
    if (outpc.rpm <= rpm_thresh) {
        idle_wait_timer = 0;
        pwmidle_reset = PWMIDLE_RESET_CALCNEWTARG;
        valve_closed = 0;
        goto throttlelifted_end;
    }

    /* check for PID lockout */
    if (flagbyte2 & flagbyte2_runidle) {
        int rpmdot, tmpload;

        DISABLE_INTERRUPTS;
        flagbyte2 &= ~flagbyte2_runidle;
        ENABLE_INTERRUPTS;

        rpmdot = outpc.rpmdot;

        if (rpmdot < 0) {
            rpmdot = -rpmdot;
        }

        if ((ram4.FuelAlgorithm & 0xf) == 3) {
            tmpload = outpc.map;
        } else {
            tmpload = outpc.fuelload;
        }

        if (idle_wait_timer >= ram4.pwmidle_pid_wait_timer) {
            if (rpmdot < ram4.pwmidle_rpmdot_threshold) {
                if (tmpload > ram4.pwmidle_decelload_threshold + ac_idleup_mapadder) {
                    pwmidle_reset = PWMIDLE_RESET_CALCNEWTARG;
                    valve_closed = 0;
                }
            }
        } else {
            /* If the timer is not yet expired... reset the timer to
             * zero if rpmdot or load are outside the acceptable range
             * in order to keep this feature from false triggering
             */
            if ((rpmdot >= ram4.pwmidle_rpmdot_threshold) || 
                (tmpload <= ram4.pwmidle_decelload_threshold)) {
                    idle_wait_timer = 0;
            }
        }
    }
throttlelifted_end:
    pwmidle_reset |= tmp;
}

void idle_closed_loop_pid(int targ_rpm, int rpm_thresh, char savelast)
{
    long last_error, tmp1, tmp2;
    int rpm;
    unsigned char flags = PID_TYPE_C;

    DISABLE_INTERRUPTS;
    flagbyte2 &= ~flagbyte2_runidle;
    ENABLE_INTERRUPTS;

    outpc.status2 |= STATUS2_CLIDLE;      // turn on CL indicator

    /* Convert the rpms to generic percents in % * 10000 units.
     * Avoid going below the min rpm set by the user.
     * That will cause a large duty cycle spike
     */
    rpm = outpc.rpm;

    if (rpm < ram4.pwmidle_min_rpm) {
        rpm = ram4.pwmidle_min_rpm;
    }

    if ((pwmidle_reset & PWMIDLE_RESET_CALCNEWTARG) ||
        (pwmidle_reset & PWMIDLE_RESET_JUSTCRANKED)) {
        flags |= PID_INIT;
        IACmotor_100 = IACmotor_pos_tmp * 100;
    }

    tmp1 = IACmotor_100 - ((generic_pid_routine(ram4.pwmidle_min_rpm,
                                                ram4.pwmidle_max_rpm,
                                                targ_rpm, rpm,
                                                ram4.pwmidle_Kp,
                                                ram4.pwmidle_Ki,
                                                ram4.pwmidle_Kd,
                                                ram4.pwmidle_ms,
                                                PV_last, &last_error,
                                                flags) *
                           (ram4.pwmidle_open_duty -
                            ram4.pwmidle_closed_duty)) / 100000);

    tmp2 = tmp1 / 100;

    if (tmp2 < ram4.pwmidle_min_duty) {
        tmp2 = ram4.pwmidle_min_duty;
        tmp1 = tmp2 * 100;
    } else if (tmp2 > ram4.pwmidle_open_duty) {
        tmp2 = ram4.pwmidle_open_duty;
        tmp1 = tmp2 * 100;
    }

    IACmotor_100 = tmp1;
    IACmotor_pos_tmp = tmp2;

    if ((outpc.rpmdot < ram4.pwmidle_rpmdot_disablepid) || (ram4.IdleCtl & 0x10)) {
        if (savelast && (outpc.rpm < rpm_thresh)) {
            IACmotor_last = IACmotor_pos_tmp;
        }
    } else {
        if ((outpc.rpm >= rpm_thresh) &&
            (pwmidle_reset & PWMIDLE_RESET_PID)) {
            pwmidle_reset = PWMIDLE_RESET_JUSTLIFTED;
            outpc.status2 &= ~STATUS2_CLIDLE;
            idle_wait_timer = 0;
            return;
        }
    }

    pwmidle_reset = PWMIDLE_RESET_PID;
}

int idle_closed_loop_newtarg(int targ_rpm)
{
    int new_targ, rpm;

    /* This function gradually drops the target from where it is now
     * to the end target
     */
    if ((pwmidle_reset & PWMIDLE_RESET_CALCNEWTARG) ||
            (pwmidle_reset & PWMIDLE_RESET_JUSTCRANKED)) {
        /* first time here, figure out number of steps
         * and the amount per step
         */

        if (outpc.rpm > targ_rpm) {
            rpm = outpc.rpm;
            if (rpm > ram4.pwmidle_max_rpm) {
                rpm = ram4.pwmidle_max_rpm;
            }
            pwmidle_targ_numsteps = (ram4.pwmidle_targ_ramptime * 1000) /
                ram4.pwmidle_ms;
            if (pwmidle_targ_numsteps > 0) {
                pwmidle_targ_stepsize =
                    (rpm - targ_rpm) / pwmidle_targ_numsteps;
                pwmidle_targ_last = rpm;
                return rpm;
            } else {
                pwmidle_targ_last = 0;
                return targ_rpm;
            }
        } else {
            pwmidle_targ_last = 0;
            return targ_rpm;
        }
    }

    if (pwmidle_targ_last != 0) {
        new_targ = pwmidle_targ_last - pwmidle_targ_stepsize;
        if (new_targ >= targ_rpm) {
            pwmidle_targ_last = new_targ;
            return new_targ;
        }
        pwmidle_targ_last = 0;
    }
    return targ_rpm;
}

void idle_closed_loop_calc_dp_decay(void)
{
    int subtractor, tmp1;

    if (outpc.rpmdot >= 0) {
        return;
    }

    subtractor = (ram4.pwmidle_dp_decay_factor/-outpc.rpmdot);

    tmp1 = IACmotor_pos_tmp - subtractor;

    if (tmp1 < IACmotor_last) {
        tmp1 = IACmotor_last;
    }

    IACmotor_pos_tmp = tmp1;
}

void idle_target_lookup(void)
{
    if ((IdleCtl == 6) || (IdleCtl == 7) || (IdleCtl == 8) ||
        (ram4.idle_special_ops & IDLE_SPECIAL_OPS_CLIDLE_TIMING_ASSIST)) {
        targ_rpm = intrp_1ditable(outpc.clt, 8,
                (int *) ram_window.pg8.pwmidle_clt_temps, 0,
                (unsigned int *) ram_window.pg8.pwmidle_target_rpms,
                8);
        targ_rpm += ac_idleup_cl_targetadder + fan_idleup_cl_targetadder;

        outpc.cl_idle_targ_rpm = targ_rpm;
    } 
}


void idle_closed_loop(void)
{
    unsigned int rpm_thresh, adj_targ;
    int pos;

    rpm_thresh = targ_rpm + ram4.pwmidle_engage_rpm_adder;

    if (outpc.engine & ENGINE_CRANK) {
        pos =
            intrp_1ditable(outpc.clt, 4,
                    (int *) ram_window.pg8.pwmidle_crank_clt_temps,
                    0,
                    (unsigned int *) ram_window.
                    pg8.pwmidle_crank_dutyorsteps, 8);
        /* cranking */
        pos += idle_voltage_comp;
        DISABLE_INTERRUPTS;
        IACmotor_pos = pos;
        IACmotor_last = pos;
        IACmotor_pos_tmp = pos;
        ENABLE_INTERRUPTS;
        idle_wait_timer = 0;
        valve_closed = 0;
        outpc.status2 &= ~STATUS2_CLIDLE;
        pwmidle_reset = PWMIDLE_RESET_JUSTCRANKED;
    } else {
        if (outpc.tps >= (int) ram4.pwmidle_tps_thresh) {
            idle_closed_loop_throttlepressed(rpm_thresh);
        } else {
            if ((pwmidle_reset & PWMIDLE_RESET_DPADDED) &&
                (flagbyte15 & FLAGBYTE15_DPCNT)) {
                DISABLE_INTERRUPTS;
                flagbyte15 &= ~FLAGBYTE15_DPCNT;
                ENABLE_INTERRUPTS;
                idle_closed_loop_calc_dp_decay();
            }
            if (pwmidle_reset & PWMIDLE_RESET_JUSTLIFTED) {
                idle_closed_loop_throttlelifted(rpm_thresh, targ_rpm);
            } else {
                /* just after crank */
                unsigned char timer;

                if (pwmidle_reset & PWMIDLE_RESET_JUSTCRANKED) {
                    timer = ram4.pwmidlecranktaper;
                } else {
                    timer = ram4.pwmidle_pid_wait_timer;
                }

                if ((flagbyte2 & flagbyte2_runidle) &&
                        (idle_wait_timer >= timer) && !IAC_moving) {

                    adj_targ = idle_closed_loop_newtarg(targ_rpm);
                    idle_closed_loop_pid(adj_targ, rpm_thresh, adj_targ == targ_rpm);
                } 
            }

            pos = IACmotor_pos_tmp;

            if ((!ram4.pwmidle_close_delay) ||
                    (outpc.rpm <= ram4.pwmidle_shift_lower_rpm) ||
                    (pwmidle_shift_timer > ram4.pwmidle_shift_open_time)) {
                pos += ac_idleup_adder;
                pos += fan_idleup_adder;
                pos += idle_voltage_comp;
            }

            IACmotor_pos = pos;
        }

        if (IdleCtl >= 6) {
            (void) move_IACmotor();
        }
    }
}

void idle_ctl(void)
{
    if (IdleCtl == 4 || IdleCtl == 6) {
        idle_voltage_compensation();
    }
    if (als_state == 1) {
        // ALS alters idle position, move stepper, but skip other idle routines
        (void) move_IACmotor();
    } else if (IACmotor_reset && iactest_glob) {
        ;                       // handled in mainloop
    } else {
        if (IdleCtl == 1) {
            idle_on_off();
        } else if (((IdleCtl == 2) || (IdleCtl == 3) || (IdleCtl == 5))
                   && IACmotor_reset) {
            idle_iac_warmup();
        } else if (IdleCtl == 4) {
            idle_pwm_warmup();
        } else if ((IdleCtl == 6) || (IdleCtl == 7) || (IdleCtl == 8)) {
            idle_closed_loop();
        }
    }
}

int move_IACmotor(void)
{
//    unsigned char coils;
    short del;

    if (IAC_moving || (outpc.iacstep == IACmotor_pos)) {
        return 0;
    }

    if ((IdleCtl == 4) || (IdleCtl == 6)) {
        /* PWM idle */
        if (IACmotor_pos > 255) {
            IACmotor_pos = 255;
        }
        *port_idlepwm = (unsigned char)IACmotor_pos;
        *port_idlepwm3 = (unsigned char)IACmotor_pos;
        outpc.iacstep = IACmotor_pos;
        return 0;
    }

    /* The rest is only for stepper motors */
    del = IACmotor_pos - outpc.iacstep;
    if (del < 0) {
        del = -del;
    }
    if (del < ram4.IACminstep) {
        return 0;
    }

    // set up the motor move
    motor_time_ms = ram4.IAC_tinitial_step; // units changed to 1 ms
    SSEM0SEI;
    PTJ &= ~0x40;               // enable current to motor(bit=0)
    CSEM0CLI;
    IAC_moving = 2;

    return 1;
}
