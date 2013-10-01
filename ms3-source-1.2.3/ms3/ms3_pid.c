/* $Id: ms3_pid.c,v 1.4 2012/05/25 14:13:46 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 * Origin: Kenneth Culver
 * Majority: Kenneth Culver
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
 */

#include "ms3.h"

void convert_unitless_percent(int min, int max, int targ, int raw_PV,
                              long *PV, long *SP)
{
    *PV = ((long) (raw_PV - min) * 10000) /
         (max - min);
    *SP = ((long) (targ - min) * 10000) /
          (max - min);
}

long generic_pid_routine(int min, int max, int targ, int raw_PV,
                         int set_Kp, int set_Ki, int set_Kd, int looptime,
                         long *PV_last_arg, long *last_error, 
                         unsigned char config)
{
    long Kp, Ki, Kd, PV, SP, error, pid_deriv, tmp1;
    int divider, multiplier;

    if (config & PID_LOOPTIME_RTC) {
        divider = 7812;
        multiplier = 781;
    } else {
        divider = 1000;
        multiplier = 100;
    }

    convert_unitless_percent(min, max, targ, raw_PV, &PV, &SP);

    error = SP - PV;

    /* Reset previous PV vals to same as PV to avoid
     * bad behavior on the first time through the loop
     */
    if (config & PID_INIT) {
        PV_last_arg[0] = PV_last_arg[1] = PV;
        *last_error = error;
    }

    pid_deriv = PV - (2 * PV_last_arg[0]) + PV_last_arg[1];

    if (config & PID_TYPE_C) {
        Kp = ((long) ((PV - PV_last_arg[0]) * set_Kp));
    } else {
        Kp = ((long) ((error - *last_error) * set_Kp));
        *last_error = error;
    }
    Ki = ((((long) error * looptime) / (long)divider) * set_Ki);
    Kd = ((long) pid_deriv * (((long) set_Kd * multiplier) / looptime));

    PV_last_arg[1] = PV_last_arg[0];
    PV_last_arg[0] = PV;

    if (config & PID_TYPE_C) {
        tmp1 = Kp - Ki + Kd;
    } else {
        tmp1 = Kp + Ki - Kd;
    }

    return tmp1;
}

