/* $Id: ms3_inj.c,v 1.183.2.5 2013/05/10 17:17:37 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 *
 * calc_opentime
    Origin: James Murray
    Majority: James Murray
 * smallpw
    Origin: James Murray
    Majority: James Murray
 * setup_staging
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * calc_duty
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * staging_on
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * calc_staged_pw
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * run_EAE_calcs
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * run_xtau_calcs
    Origin: Al Grippo
    Majority: Al Grippo
 * wheel_fill_inj_event_array
    Origin: Kenneth Culver
    Moderate: James Murray
    Majority: Kenneth Culver / James Murray
 * inj_event_array_portpin
    Origin: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * calc_reqfuel
    Origin: Kenneth Culver
    Minor: James Murray
    Majority: Kenneth Culver
 * crank_calcs
    Origin: Al Grippo
    Moderate: James Murray / Kenneth Culver
    Majority: Al Grippo / Kenneth Culver / James Murray
 * warmup_calcs
    Origin: Al Grippo
    Moderate: James Murray / Kenneth Culver
    Majority: Al Grippo / Kenneth Culver / James Murray
 * normal_accel
    Origin: Al Grippo
    Minor: AE %ages. James Murray
    Majority: Al Grippo
 * new_accel_calc_percent
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * new_accel
    Origin: Kenneth Culver
    Majority: Kenneth Culver
 * main_fuel_calcs
    Origin: Al Grippo
    Moderate: James Murray / Kenneth Culver
    Majority: Al Grippo / Kenneth Culver / James Murray
 * flex_fuel_calcs
    Origin: Al Grippo
    Majority: Al Grippo
 * do_overrun
    Origin: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * n2o_launch_additional_fuel
    Origin: James Murray
    Majority: James Murray
 * injpwms
    Trace: Al Grippo
    Majority: James Murray
 * do_final_fuelcalcs
    Origin: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * do_seqpw_calcs
    Origin: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 * do_sequential_fuel
    Origin: Kenneth Culver
    Minor: James Murray
    Majority: Kenneth Culver
 * calc_fuel_trims
    Origin: James Murray
    Majority: James Murray
 * calc_divider
    Origin: James Murray / Kenneth Culver
    Majority: James Murray / Kenneth Culver
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/

#include "ms3.h"

static unsigned int staging_percent;
static unsigned char squirts_per_rev;

/**************************************************************************
 **
 ** Calculation of Injector opening time.
 **
 ** Uses a base time setting * correction curve from battery voltage
 **
 **
 **************************************************************************/
signed int calc_opentime(unsigned char inj_no)
{
    int tmp1, tmp2, curve_no;

    // pick curve and lookup base opentime
    if (pin_dualfuel && (ram5.dualfuel_sw2 & 0x04) && (!(*port_dualfuel & pin_dualfuel))) {
        if ((inj_no < 8) && ((ram5.opentime2_opt[0] & 0x80) == 0)) {
            // shared settings from MS3X inj A
            inj_no = 0;
        }
        curve_no = ram5.opentime2_opt[(int)inj_no]& 0x03;
        tmp2 = ram5.inj2Open[inj_no];
    } else {
        if ((inj_no < 8) && ((ram4.opentime_opt[0] & 0x80) == 0)) {
            // shared settings from MS3X inj A
            inj_no = 0;
        }
        curve_no = ram4.opentime_opt[(int)inj_no]& 0x03;
        tmp2 = ram5.injOpen[inj_no];
    }
    // get correction %age
    tmp1 = intrp_1ditable(outpc.batt, 6, (int *) ram_window.pg10.opentimev, 0,
      (int *) &ram_window.pg10.opentime[curve_no][0], 10);
    if (tmp1 < 0) {
        tmp1 = 0;
    } else if (tmp1 > 5000) {
        tmp1 = 5000;
    }
    // scale
    __asm__ __volatile__ (
    "emuls\n"
    "ldx #1000\n"
    "edivs\n"
    :"=y"(tmp1)
    :"d"(tmp1), "y"(tmp2)
    :"x"
    );
    return tmp1;
}

/**************************************************************************
 **
 ** Calculation of non-linear small pulsewidth
 **
 **************************************************************************/
unsigned int smallpw(unsigned int pw_in, unsigned char inj_no)
{
    int curve_no;
    RPAGE = tables[10].rpg;     // load page into ram window, to check upper value

    // pick curve and lookup base opentime
    if (pin_dualfuel && (ram5.dualfuel_sw2 & 0x08) && (!(*port_dualfuel & pin_dualfuel))) {
        curve_no = ram5.smallpw2_opt[(int) inj_no] & 0x03;
    } else {
        curve_no = ram4.smallpw_opt[(int) inj_no] & 0x03;
    }
    if ((ram4.smallpw_opt[0] & 0x80)
        && (pw_in < ram_window.pg10.smallpwpw[5])) {
        return intrp_1ditable(pw_in, 6, (int *) ram_window.pg10.smallpwpw,
                          0,(int *) &ram_window.pg10.smallpw[curve_no][0], 10);
    } else {
        return pw_in;
    }
}

/* staging parameters from user are size of primaries and size of secondaries.
 * we will calculate a percentage (100% means same-size injectors) and store
 * in RAM for the staged pw calc
 */
void setup_staging(void)
{
    if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
        staging_percent = ((unsigned long) ram4.staged_pri_size * 100) /
                          ram4.staged_sec_size;
    } else {
        staging_percent = ((unsigned long) ram4.staged_pri_size * 100) /
                           (ram4.staged_pri_size + ram4.staged_sec_size);
    }
    flagbyte4 &= ~flagbyte4_staging_on;
    squirts_per_rev = (num_cyl / divider) >> 1;
    if ((ram4.sequential & SEQ_SEMI) && (ram4.hardware & HARDWARE_MS3XFUEL)
        && (num_cyl > 4) && !(ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3)
        && (cycle_deg == 7200)) { // not enough events to do this in 1.0
            conf_err = 109;
    }
}

unsigned char calc_duty(unsigned long base_pw)
{
    unsigned long time_per_rev;
    unsigned long dtpred_time;
    unsigned int duty;

    /* add this since it's really part of the duty...
     * Divide by 100 because we were doing our calcs in
     * usec * 100 resolution before.
     */
    base_pw = (base_pw / 100) + calc_opentime(8);

    base_pw *= squirts_per_rev;

    /* how long is 1 rev right now */
    DISABLE_INTERRUPTS;
    dtpred_time = dtpred;
    ENABLE_INTERRUPTS;

    time_per_rev = (num_cyl >> 1) * dtpred_time;

    /* rolled over, but we don't want to stage b/c of that */
    if (time_per_rev < dtpred_time) {
        return 0;
    }

    duty = (unsigned int) ((base_pw * 100) / time_per_rev);

    if (duty > 255) {
        duty = 255;
    }

    return (unsigned char) duty;
}

/* determine if staging should be on or not */
unsigned char staging_on(unsigned long base_pw)
{
    unsigned char param;
    static unsigned char staged_first_param_on = 0,
                         staged_second_param_on = 0;
    int param_to_check;

    param = ram4.staged & 0x7;
    /* First parameter */

    if (param == 0x1) {
        param_to_check = (int)outpc.rpm;
    } else if (param == 0x2) {
        param_to_check = outpc.map / 10;
    } else if (param == 0x3) {
        param_to_check = outpc.tps / 10;
    } else {
        /* calc duty and check against setting */
        param_to_check = calc_duty(base_pw);
    }

    if (param_to_check >= ram4.staged_param_1) {
        staged_first_param_on = 1;
    } else if (param_to_check < (ram4.staged_param_1 - ram4.staged_hyst_1)) {
        staged_first_param_on = 0;
    }

    param = ram4.staged & 0x38;

    /* only one param, so return the result */
    if (!param) {
        return (staged_first_param_on);
    }

    if (param == 0x8) {
        param_to_check = (int)outpc.rpm;
    } else if (param == 0x10) {
        param_to_check = outpc.map / 10;
    } else if (param == 0x18) {
        param_to_check = outpc.tps / 10;
     } else {
        param_to_check = calc_duty(base_pw);
    }

    if (param_to_check >= ram4.staged_param_2) {
        staged_second_param_on = 1;
    } else if (param_to_check < (ram4.staged_param_2 - ram4.staged_hyst_2)) {
        staged_second_param_on = 0;
    }

    if (ram4.staged & 0x80) {
        return (staged_first_param_on && staged_second_param_on);
    } else {
        return (staged_first_param_on || staged_second_param_on);
    }

    return 0;
}

/* now we calculate the staged pulse-width based on the staging percent, and
 * conditions for staging
 */
void calc_staged_pw(unsigned long base_pw)
{
    long calculated_pw1, calculated_pw2, calculated_base_pw;
    long per_ignevent_amt_pw1, per_ignevent_amt_pw2, calculated_secondary_enriched; 
    unsigned char staged_num_events_pri;
    unsigned int staged_transition_percent;

    /* percent in 1% units */
    calculated_base_pw = (base_pw * (unsigned long) staging_percent) / 100;


    if ((ram4.staged & 0x7) == 5) {
        /* This mode is a bit more complicated... we look up a value in an loadxRPM table,
         * and use the percent there to figure out what percentage through the staging process
         * the user wants to be at... 0% is no staging, and 100% is fully staged
         */
        staged_transition_percent = intrp_2dctable(outpc.rpm, outpc.fuelload, 8, 8,
                                                   ram_window.pg11.staged_rpms,
                                                   ram_window.pg11.staged_loads,
                                                   (unsigned char *) ram_window.
                                                   pg11.staged_percents, 1, 11);


        /* Hard code 0% and 100% to avoid doing the long math when we can */
        if (staged_transition_percent == 0) {
            pw_staged1 = base_pw;
            pw_staged2 = 0;
        } else if (staged_transition_percent == 1000) {
            if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
                pw_staged1 = 0;
                pw_staged2 = calculated_base_pw;
            } else {
                pw_staged1 = pw_staged2 = calculated_base_pw;
            }
        } else {
            /* pw1 is (base pw - (starting pw - the ending pw) * the commanded percent)
             * so that pw1 is scaled down by the same amt that pw2 is scaled up by given the
             * size difference between the injectors
             */
            if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
                calculated_pw1 = base_pw - (((long) staged_transition_percent * base_pw) /
                                                    1000L);
            } else {
                calculated_pw1 = base_pw - (((long) staged_transition_percent *
                                 (base_pw - calculated_base_pw)) / 1000L);
            }

            if (calculated_pw1 < 0) {
                calculated_pw1 = 0;
            }

            /* pw2 is (commanded percent * base pw) */

            calculated_pw2 = (calculated_base_pw * staged_transition_percent) / 1000L;

            pw_staged1 = calculated_pw1;
            pw_staged2 = calculated_pw2;
        }
    } else if (staging_on(base_pw)) {
        /* did staging just come on? if so, pw_staged2 will be 0... use that to reset
         * the transition count if necessary */

        if ((ram4.staged & 0x40) && (!(flagbyte4 & flagbyte4_transition_done))) {
            if (pw_staged2 == 0) {
                DISABLE_INTERRUPTS;
                staged_num_events = 1;
                ENABLE_INTERRUPTS;
            }

            calculated_secondary_enriched = (calculated_base_pw + ram4.staged_secondary_enrichment);

            if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
                per_ignevent_amt_pw1 = base_pw / ram4.staged_transition_events;
            } else {
                per_ignevent_amt_pw1 = (base_pw - calculated_base_pw) / ram4.staged_transition_events;
            }
            per_ignevent_amt_pw2 = calculated_secondary_enriched / ram4.staged_transition_events;

            staged_num_events_pri = staged_num_events - ram4.staged_primary_delay;
            if (staged_num_events_pri > staged_num_events) {
                /* rolled over */
                staged_num_events_pri = 0;
            }

            calculated_pw2 = (staged_num_events * per_ignevent_amt_pw2);
            calculated_pw1 = base_pw -
                             (staged_num_events_pri * per_ignevent_amt_pw1);

            if (calculated_pw1 < 0) {
                calculated_pw1 = 0;
            }

            if (!(ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF)) {
                if (calculated_pw1 <= calculated_base_pw) {
                    calculated_pw1 = calculated_base_pw;
                }
            }

            if (staged_num_events_pri >= ram4.staged_transition_events) {
                flagbyte4 |= flagbyte4_transition_done;
            }

            if (flagbyte4 & flagbyte4_transition_done) {
                if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
                    calculated_pw2 = calculated_base_pw;
                    calculated_pw1 = 0;
                } else {
                    calculated_pw2 = calculated_pw1 = calculated_base_pw;
                }
            }
        } else {
            if (ram4.staged_extended_opts & STAGED_EXTENDED_STAGEPW1OFF) {
                calculated_pw2 = calculated_base_pw;
                calculated_pw1 = 0;
            } else {
                calculated_pw2 = calculated_pw1 = calculated_base_pw;
            }
        }

        pw_staged1 = calculated_pw1;
        pw_staged2 = calculated_pw2;
    } else {
        /* staging should be off, just set the pulse-width to the base_pw */

        pw_staged1 = base_pw;
        pw_staged2 = 0;
        flagbyte4 &= ~flagbyte4_transition_done;
    }
}

/**************************************************************************
 **
 **  EAE Transient Enrichment Section:
 **
 BF = Basic Fuel amount
 P = MAP value
 EXC = EGO correction factor
 TCC = Corrections for temperature, barometric pressure, etc...
 DFC = Desired Amount of Fuel
 WF = total amt of fuel adhering to walls
 AWC = Adhere to wall coefficient: proportion of the fuel injected in next pulse which will adhere.
 AWA = actual amount that'll adhere in the next pulse
 SOC = proportion of fuel injected in next pulse which will be sucked off the walls
 SOA = actual amt of fuel injected in next pulse which will be sucked off the walls
 SQF = actual amt of fuel squirted for this pulse
 AFC = time that injector is commanded to be open
 DT = injector opening time
 BAWC = AWC but only with reference to manifold pressure.
 BSOC = same as BAWC but for SOC
 AWW = correction to AWC based on CLT
 SOW = correction to SOC based on CLT
 AWN = correction to AWC based on engine speed
 SON = correction to SOC based on engine speed
 AWF = correction to AWC based on air velocity
 SOF = correction to SOC based on air velocity

 AWC = BAWC*AWW*AWN*AWF
 SOC = BSOC*SOW*SON*SOF

 SOA = SOC * WF
 SQF = (DFC - SOA)/(1-AWC)
 AWA = SQF * AWC

 if (fuelcut)
 WF = WF - SOA
 else
 WF = WF + AWA - SOA

 AFC = SQF + DT

 calculation of WF should be done in ISR, the rest can and will be done
 in main loop.
 **
 **************************************************************************/

void run_EAE_calcs(void)
{
    if (((ram4.EAEOption == 1) || (ram4.EAEOption == 2)) &&
        (outpc.rpm != 0)) {
        unsigned long wflocal;
        unsigned long SQF, SOAtmp, AWAtmp;
        unsigned char BAWC, AWN, SOC, BSOC, SON, AWC, AWW, SOW;


        BAWC =
            (unsigned char) intrp_1dctable(outpc.eaeload, NO_FMAPS,
                                           (int *) ram_window.pg8.
                                           EAEAWCKPAbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAEBAWC, 8);
        AWN =
            (unsigned char) intrp_1dctable(outpc.rpm, NO_FRPMS,
                                           (int *) ram_window.pg8.
                                           EAEAWCRPMbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAEAWN, 8);
        AWW =
            (unsigned char) intrp_1dctable(outpc.clt, 12,
                                           (int *) ram_window.pg8.
                                           EAEAWWCLTbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAEAWW, 8);
        /* BAWC in 1% units, so 100 = 100%, but make end result in .1% units */

        AWC = ((unsigned long) BAWC * AWN * AWW) / 10000;
        if (AWC >= 100)
            AWC = 99;           /* avoid div by 0 below */

        BSOC =
            (unsigned char) intrp_1dctable(outpc.eaeload, NO_FMAPS,
                                           (int *) ram_window.pg8.
                                           EAESOCKPAbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAEBSOC, 8);
        SON =
            (unsigned char) intrp_1dctable(outpc.rpm, NO_FRPMS,
                                           (int *) ram_window.pg8.
                                           EAESOCRPMbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAESON, 8);
        SOW =
            (unsigned char) intrp_1dctable(outpc.clt, 12,
                                           (int *) ram_window.pg8.
                                           EAESOWCLTbins, 0,
                                           (unsigned char *)
                                           ram_window.pg8.EAESOW, 8);

        /* units here are .1% */
        SOC = ((unsigned long) BSOC * SON * SOW) / 10000;

        /* table lookups done, do calcs */

        /* calculate actual amt "sucked off" the walls */

        DISABLE_INTERRUPTS;
        wflocal = WF1;
        ENABLE_INTERRUPTS;

        SOAtmp = (SOC * wflocal) / 1000;

        DISABLE_INTERRUPTS;
        SOA1 = SOAtmp;
        ENABLE_INTERRUPTS;

        if (!(outpc.engine & ENGINE_CRANK)) {
            tmp_pw1 *= EAE_multiplier;
        }

        if (SOAtmp > tmp_pw1)
            SOAtmp = tmp_pw1;

        /* Calc actual amt of fuel to be injected in next pulse */
        SQF =
            ((unsigned long) ((unsigned long) (tmp_pw1 - SOAtmp) * 100)) /
            (100 - AWC);

        if (SQF > 3200000) {
            SQF = 3200000; // limit to half PW range
        }

        /* do % calc */

        outpc.EAEfcor1 = ((unsigned long) SQF * 100) / tmp_pw1;

        AWAtmp = ((unsigned long) SQF * AWC) / 100;

        DISABLE_INTERRUPTS;
        AWA1 = AWAtmp;
        ENABLE_INTERRUPTS;
        outpc.wallfuel1 = wflocal;

        if (!(outpc.engine & ENGINE_CRANK)) {
            tmp_pw1 = (SQF / EAE_multiplier);
        }

        /* do this optionally to speed up the code when we don't need it */

        if (do_dualouts) {
            BAWC =
                (unsigned char) intrp_1dctable(outpc.eaeload, NO_FMAPS,
                        (int *) ram_window.pg19.
                        EAEAWCKPAbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAEBAWC2, 8);
            AWN =
                (unsigned char) intrp_1dctable(outpc.rpm, NO_FRPMS,
                        (int *) ram_window.pg19.
                        EAEAWCRPMbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAEAWN2, 8);
            AWW =
                (unsigned char) intrp_1dctable(outpc.clt, 12,
                        (int *) ram_window.pg19.
                        EAEAWWCLTbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAEAWW2, 8);
            /* BAWC in 1% units, so 100 = 100%, but make end result in .1% units */

            AWC = ((unsigned long) BAWC * AWN * AWW) / 10000;
            if (AWC >= 100)
                AWC = 99;           /* avoid div by 0 below */

            BSOC =
                (unsigned char) intrp_1dctable(outpc.eaeload, NO_FMAPS,
                        (int *) ram_window.pg19.
                        EAESOCKPAbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAEBSOC2, 8);
            SON =
                (unsigned char) intrp_1dctable(outpc.rpm, NO_FRPMS,
                        (int *) ram_window.pg19.
                        EAESOCRPMbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAESON2, 8);
            SOW =
                (unsigned char) intrp_1dctable(outpc.clt, 12,
                        (int *) ram_window.pg19.
                        EAESOWCLTbins2, 0,
                        (unsigned char *)
                        ram_window.pg19.EAESOW2, 8);
            DISABLE_INTERRUPTS;
            wflocal = WF2;
            ENABLE_INTERRUPTS;

            SOAtmp = (SOC * wflocal) / 1000;

            DISABLE_INTERRUPTS;
            SOA2 = SOAtmp;
            ENABLE_INTERRUPTS;

            if (SOAtmp > tmp_pw2)
                SOAtmp = tmp_pw2;

            /* Calc actual amt of fuel to be injected in next pulse */
            SQF =
                ((unsigned long) ((unsigned long) (tmp_pw2 - SOAtmp) *
                                  100)) / (100 - AWC);

            if (SQF > 3200000) {
                SQF = 3200000; // limit to half PW range
            }

            /* do % calc */

            outpc.EAEfcor2 = ((unsigned long) SQF * 100) / tmp_pw2;

            AWAtmp = ((unsigned long) SQF * AWC) / 100;

            DISABLE_INTERRUPTS;
            AWA2 = AWAtmp;
            ENABLE_INTERRUPTS;
            outpc.wallfuel2 = wflocal;

            if (!(outpc.engine & ENGINE_CRANK)) {
                tmp_pw2 = SQF;
            }
        } else {
            DISABLE_INTERRUPTS;
            AWA2 = AWAtmp;
            SOA2 = SOAtmp;
            outpc.EAEfcor2 = outpc.EAEfcor1;
            ENABLE_INTERRUPTS;
            tmp_pw2 = tmp_pw1;
        }

        /* WF calc in interrupt */
    }
}

/**************************************************************************
**
**  X,Tau Transient Enrichment Section:
**    
**     fi = [ mi  -  (Mi / (tau / dltau)) ] / (1 - X)
**
**     Mi+1 = Mi + X * fi - (Mi / (tau / dltau))
**
**   where,
**          fi = total fuel injected
**
**          mi = total fuel going directly into combustion chamber (want this 
**                to be the calculated fuel)
**          
**          Mi = net fuel entering/leaving port wall puddling
**          
**          dltau = time (msx10) between squirts
**
**          X = fraction of fuel injected which goes into port wall puddling
**          tau = puddle fuel dissipation time constant (ms) as function 
**                 of rpm and coolant temp.
**                 
**         XTfcor = % correction to calculated fuel to ensure the calculated
**                 amount gets into the comb chamber.  
**
**************************************************************************/
void run_xtau_calcs(void)
{
    int ix;
    long tmp1, tmp2, tmp3, tmp4, beta, ltmp;
    unsigned long utmp1 = 0, utmp2, ultmp;
    unsigned long ppw[2];
    char Tau0;

    if (((ram4.EAEOption == 3) || (ram4.EAEOption == 4))
        && (flagbyte5 & FLAGBYTE5_RUN_XTAU)) {
        ppw[0] = tmp_pw1 / 100;
        ppw[1] = tmp_pw2 / 100;
        // calculate X(%x10), tau(ms) as function of rpm, accel/decel, clt
        tmp1 = -(int) ram4.MapThreshXTD;
        tmp2 = -(int) ram4.MapThreshXTD2;

        if (outpc.mapdot > tmp1) {      // use accel tables
            XTX =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.XAccTable, 11);
            Tau =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.TauAccTable, 11);
        } else if (outpc.mapdot < tmp2) {       // use decel tables
            XTX =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.XDecTable, 11);
            Tau =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.TauDecTable, 11);
        } else {                // transition from accel -> decel
            utmp1 =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.XAccTable, 11);
            utmp2 =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.XDecTable, 11);
            tmp3 = outpc.mapdot - tmp2;
            tmp4 = tmp1 - tmp2;
            tmp3 = ((long) tmp3 * 100L) / tmp4;

            // save tmp3 = ((outpc.mapdot - tmp2)* 100) / (tmp1 - tmp2)
            tmp4 = utmp1 - utmp2;
            tmp4 = ((long) tmp3 * tmp4) / 100;
            XTX = utmp2 + tmp4;
            utmp1 =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.TauAccTable, 11);
            utmp2 =
                intrp_1ditable(outpc.rpm, NO_XTRPMS,
                               ram_window.pg11.XTrpms, 0,
                               ram_window.pg11.TauDecTable, 11);
            tmp4 = (utmp1 - utmp2);
            tmp4 = ((long) tmp3 * tmp4) / 100;
            Tau = utmp2 + tmp4;
        }

        if (ram4.EAEOption == 4) {
            // use X-Tau for warmup based on clt temp
            utmp1 = (unsigned short) CW_table(outpc.clt, (int *) ram_window.pg11.XClt, ram_window.pg11.XClt_temps, 11); // %
            XTX = ((long) XTX * utmp1) / 100;
            // use X-Tau for warmup based on clt temp
            utmp1 = (unsigned short) CW_table(outpc.clt, (int *) ram_window.pg11.TauClt, ram_window.pg11.TauClt_temps, 11);     // %
            Tau = ((long) Tau * utmp1) / 100;
        }

        if (Tau < 1) {
            Tau = 1;
            Tau0 = 1;           // continue x-tau calc with tau= 1, but make no correction.
        } else {
            Tau0 = 0;
        }

        for (ix = 0; ix < 2; ix++) {
            XTm = ppw[ix];

            DISABLE_INTERRUPTS;
            ultmp = sum_dltau[ix];
            sum_dltau[ix] = 0;
            ENABLE_INTERRUPTS;

            ltmp = (Tau * 1000) / ultmp;        // tau x 1000 (us) / dltau(us)

            if (ltmp < 1) {     // port wall is totally dry
                ltmp = 0;       // These derived from setting (1-X)fi = mi
                XTM[ix] = 0;
            } else {
                ltmp = XTM[ix] / ltmp;
            }

            if (ltmp > XTm) {
                ltmp = XTm;
            }

            beta = ((long) (XTm - ltmp) * 1000) / (long) (1000 - XTX);
            if (beta > 65000) {
                beta = 65000;
            }
            XTf = beta;
            utmp2 = ((long) XTX * XTf) / 1000;
            XTM[ix] = XTM[ix] + utmp2 - ltmp;
            if ((XTm > 0) && !Tau0) {
                tmp1 = ((long) XTf * 100L) / XTm;
            } else {
                tmp1 = 100;
            }
            if (ix == 0) {
                outpc.EAEfcor1 = tmp1;
            } else {
                outpc.EAEfcor2 = tmp1;
            }
            if (!Tau0) {
                ppw[ix] = XTf;
            }
//            if (ppw[ix] > 32000) {
//                ppw[ix] = 32000;        // DON'T rail at 32 ms
//            }
        }
        tmp_pw1 = ppw[0] * 100;
        tmp_pw2 = ppw[1] * 100;
    }
}

void wheel_fill_inj_event_array(inj_event * inj_events_fill, int inj_ang,
                                ign_time last_tooth_time,
                                unsigned int last_tooth_ang,
                                unsigned char num_events,
                                unsigned char num_outs,
                                unsigned char flags)
{
    char iterate, tth, wfe_err, oldout_staged = 0;
    int tth_ang, tmp_ang, i, k;
    ign_time inj_time;
    unsigned int increment, start, end;

    increment = no_triggers / num_events;

    if ((flags & INJ_FILL_STAGED) &&
       (((num_cyl > 4) && (ram4.sequential & SEQ_FULL)) ||
         (ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3))) {
        oldout_staged = 1;
    }

    /* This will be bounded at init time */
    if((flags & INJ_FILL_STAGED) && !oldout_staged) {
        start = num_events;
        end = start << 1;
    } else if (oldout_staged) {
        start = 8;
        if (ram4.staged_extended_opts & STAGED_EXTENDED_SIMULT) {
            increment = no_triggers;
            end = 9;
        } else {
            increment = no_triggers>>1;
            end = 10;
        }
    } else {
        start = 0;
        end = num_events;
    }

/* new tests */
        // handle the negative angle case
        if (inj_ang < 0) {
            inj_ang += cycle_deg;
        }

        // handle the excessive angle case
        if (inj_ang > cycle_deg) {
            inj_ang -= cycle_deg;
        }
/* end new */

    for (i = start, k = 0; i < end; i++, k += increment) {
        int tmp_inj_ang;

        wfe_err = 0;
        iterate = 0;
        tth_ang = trig_angs[k];
        tth = trigger_teeth[k];
        tmp_inj_ang = inj_ang;


//////////////////////
// Is this valid???
        // handle the negative angle case
        if (tth_ang > tmp_inj_ang) {
            tmp_inj_ang += cycle_deg;
        }
//////////////////////

        while (!iterate) {
            if (tth_ang > tmp_inj_ang) {
                iterate = 1;
            } else {
                //how far do we step back in deg
                tth--;
                if (tth < 1) {
                    tth = last_tooth;
                    wfe_err++;
                    if (wfe_err > 2) {
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
            debug_str("Iterate==2 @ ms3_inj.c:");
            debug_byte2dec(__LINE__ / 100); debug_byte2dec(__LINE__ % 100);
            debug_str("\r");
            return;
        }

        tmp_ang = tth_ang - tmp_inj_ang;

        inj_time.time_32_bits = (tmp_ang * last_tooth_time.time_32_bits) /
            last_tooth_ang;

        wfe_err = 0;

        while ((wfe_err < 2) && (inj_time.time_32_bits < SPK_TIME_MIN)) {
            // too soon after tooth, need to step back
            tth--;
            if (tth < 1) {
                tth = last_tooth;
                wfe_err++;
            }
            tth_ang += deg_per_tooth[tth - 1];
            // recalc
            tmp_ang = tth_ang - tmp_inj_ang;
            inj_time.time_32_bits =
                (tmp_ang * last_tooth_time.time_32_bits) / last_tooth_ang;
        }

        if (wfe_err > 2) {
            DISABLE_INTERRUPTS;
            asm("nop\n");       // something screwed up, place for breakpoint
            ENABLE_INTERRUPTS;
            // can't continue as didn't find a valid tooth
            debug_str("Iterate==2 @ ms3_inj.c:");
            debug_byte2dec(__LINE__ / 100); debug_byte2dec(__LINE__ % 100);
            debug_str("\r");
            return;
        }

        inj_events_fill[i].tooth = tth;
        inj_events_fill[i].time = inj_time.time_32_bits/50;
        if (inj_events_fill[i].time == 0) {
            inj_events_fill[i].time = 1;
        }
    }

    if ((oldout_staged) && (ram4.staged_extended_opts & STAGED_EXTENDED_SIMULT)) {
        inj_events_fill[9].tooth = inj_events_fill[8].tooth;
        inj_events_fill[9].time = inj_events_fill[8].time;
    }
}

void inj_event_array_portpin(unsigned char flags)
{
    char oldout_staged = 0;
    unsigned char num_outs;
    int i, j = 0, k;
    unsigned int increment, start, end, pin;

    for (i = 0 ; i < NUM_TRIGS ; i++) {
        injch[i] = 0; // primary
    }

    num_outs = num_inj_outs; /* gets changed */
    increment = no_triggers / num_inj_events;

    if (flags & INJ_FILL_STAGED) {
        if ((((num_cyl > 4) && (ram4.sequential & SEQ_FULL)) ||
         (ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3))) {
            oldout_staged = 1;
            injch[8] = 1;
            injch[9] = 1;
        }

        if ((num_cyl > 8) && !oldout_staged) {
            conf_err = 109;
            return;
        }
    } else {
        // always set V3 injectors to defaults on first call
        OCPD |= 0xa;
        next_inj[8].port = next_inj[9].port = &PORTT;
        next_inj[8].pin = 0x2;
        next_inj[9].pin = 0x8;
    }

    if (flags & INJ_FILL_TESTMODE) {
        /* force injector to hardware mapping */
        num_outs = 1;
        oldout_staged = 0;
        start = 0;
        end = 12;
        goto IEAPP1;
    }

    /* This will be bounded at init time */
    if((flags & INJ_FILL_STAGED) && !oldout_staged) {
        start = num_inj; // was num_inj_events
        end = start << 1;
    } else if (oldout_staged) {
        start = 8;
        if (ram4.staged_extended_opts & STAGED_EXTENDED_SIMULT) {
            increment = no_triggers;
            end = 10; // was 9
        } else {
            increment = no_triggers>>1;
            end = 10;
        }
    } else {
        start = 0;
        if ((ram4.sequential & SEQ_SEMI) && (num_inj_events > 8)) {
            end = num_inj_events >> 1; // V10, V12, V16 semi-seq must run 2 inj per output at the moment
        } else if ((ram4.sequential & 0x03) == 0) {
            /* for non sequential all injectors are individually timed so trim works
                this means that >8 cyl must be alternating - checked elsewhere */
            end = num_inj;
        } else {
            end = num_inj_events;
        }
    }

    // semi-seq MS3X dual fuel cases
    if ((ram4.sequential & SEQ_SEMI) && (ram4.hardware & HARDWARE_MS3XFUEL) && (do_dualouts)
        && (((num_inj == 4) && (num_cyl == 4)) || (num_cyl == 6) || (num_cyl == 8)) && !(ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3)) {
        if (flags & INJ_FILL_STAGED) {
            oldout_staged = 1;
            start = 1;
            end = 0; // don't do loop below
            for (i = 0; i < NUM_TRIGS; i++) {
                next_inj[i].port = &PORTA; // FIXME may change in the future
            }
            // 4 cyl uses one wire per injector like normal. 8 events in 720 deg, 4 events in 360
            // special cases for 6 and 8 cyl using MS3X for pri and sec. - wire 2 injectors per output
            // in 720 deg case, two events point to each timer used
            if ((num_inj == 4) && (num_cyl == 4)) {
                if (cycle_deg == 7200) {
                    // hardcode the output to get them right
                    next_inj[0].pin = 0x05; // pri
                    next_inj[1].pin = 0x0a;
                    next_inj[2].pin = 0x05;
                    next_inj[3].pin = 0x0a;

                    next_inj[4].pin = 0x50; // sec
                    next_inj[5].pin = 0xa0;
                    next_inj[6].pin = 0x50;
                    next_inj[7].pin = 0xa0;
                    injch[4] = 1;
                    injch[5] = 1;
                    injch[6] = 1;
                    injch[7] = 1;
                } else {
                    // hardcode the output to get them right
                    next_inj[0].pin = 0x05; // pri
                    next_inj[1].pin = 0x0a;

                    next_inj[2].pin = 0x50; // sec
                    next_inj[3].pin = 0xa0;
                    injch[2] = 1;
                    injch[3] = 1;
                }
            } else if (num_cyl == 6) {
                if (cycle_deg == 7200) {
                    inj_cnt_xref[0] = 0;
                    inj_cnt_xref[1] = 1;
                    inj_cnt_xref[2] = 2;
                    inj_cnt_xref[3] = 0;
                    inj_cnt_xref[4] = 1;
                    inj_cnt_xref[5] = 2;

                    inj_cnt_xref[6] = 3;
                    inj_cnt_xref[7] = 4;
                    inj_cnt_xref[8] = 5;
                    inj_cnt_xref[9] = 3;
                    inj_cnt_xref[10]= 4;
                    inj_cnt_xref[11]= 5;
                }
                // hardcode the output to get them right
                next_inj[0].pin = 0x01; // pri
                next_inj[1].pin = 0x02;
                next_inj[2].pin = 0x04;

                next_inj[3].pin = 0x08; // sec
                next_inj[4].pin = 0x10;
                next_inj[5].pin = 0x20;
                injch[3] = 1;
                injch[4] = 1;
                injch[5] = 1;
                num_outs = 1; // looks like seq
            } else if (num_cyl == 8) {
                if (cycle_deg == 7200) {
                    inj_cnt_xref[0] = 0;
                    inj_cnt_xref[1] = 1;
                    inj_cnt_xref[2] = 2;
                    inj_cnt_xref[3] = 3;
                    inj_cnt_xref[4] = 0;
                    inj_cnt_xref[5] = 1;
                    inj_cnt_xref[6] = 2;
                    inj_cnt_xref[7] = 3;

                    inj_cnt_xref[8] = 4; // sec
                    inj_cnt_xref[9] = 5;
                    inj_cnt_xref[10]= 6;
                    inj_cnt_xref[11]= 7;
                    inj_cnt_xref[12]= 4;
                    inj_cnt_xref[13]= 5;
                    inj_cnt_xref[14]= 6;
                    inj_cnt_xref[15]= 7;
                }
                // hardcode the output to get them right
                next_inj[0].pin = 0x01; // pri
                next_inj[1].pin = 0x02;
                next_inj[2].pin = 0x04;
                next_inj[3].pin = 0x08;

                next_inj[4].pin = 0x10; // sec
                next_inj[5].pin = 0x20;
                next_inj[6].pin = 0x40;
                next_inj[7].pin = 0x80;
                injch[4] = 1;
                injch[5] = 1;
                injch[6] = 1;
                injch[7] = 1;
                num_outs = 1; // looks like seq
            }
        }
    }

IEAPP1:;

    for (i = start, k = 0; i < end; i++, k += increment) {
        if ((ram4.hardware & HARDWARE_MS3XFUEL) && (!oldout_staged)) {
            if (flags & INJ_FILL_STAGED) {
            injch[i] = 1;
            }
            pin = 1;

            if ((num_outs > 1) && (num_inj_events < 9) && (num_cyl < 9)) { // i.e. semi-seq but don't enable outputs beyond injH on MS3
                if (cycle_deg == 7200) {
                        pin <<= (num_inj_events >> 1);
                        pin |= 1;
                    if (i >=  (num_inj_events >> 1)) {
                        j = i - (num_inj_events >> 1);
                    } else {
                        j = i;
                    }
                    pin <<= j;
                } else { // 360 deg case
                        pin <<= num_inj_events;
                        pin |= 1;
                    pin <<= i;
                }
            } else { // seq
                pin <<= i;
            }

            if (i < 8) { // 0-7 only
                next_inj[i].pin = pin;
                next_inj[i].port = pPTA;
            } else if (i == 8) {
                if ((ram4.hardware & 0x20) 
                    && (!((MONVER >= 0x380) && (MONVER <= 0x3ff)))) { // only on MS3
                    next_inj[8].pin = 0x08; // PK3 / H3
                    next_inj[8].port = pPTK;
                } else { // optional or non MS3 uses PT1 for injI
                    next_inj[8].pin = 0x02;
                    next_inj[8].port = pPTT;
                }
            } else if (i == 9) {
                if ((ram4.hardware & 0x20) 
                    && (!((MONVER >= 0x380) && (MONVER <= 0x3ff)))) { // only on MS3
                    next_inj[9].pin = 0x02; // PK1 / H4
                    next_inj[9].port = pPTK;
                } else { // optional or non MS3 uses PT3 for injJ
                    next_inj[9].pin = 0x08;
                    next_inj[9].port = pPTT;
                }
            } else if (i == 10) {
                if ((MONVER >= 0x380) && (MONVER <= 0x3ff)) { // non MS3
                    next_inj[10].pin = 0x10;
                    next_inj[10].port = &PTP;
                } else {
                    next_inj[10].pin = 0x80;
                    next_inj[10].port = &PORTK;
                }
            } else if (i == 11) {
                if ((MONVER >= 0x380) && (MONVER <= 0x3ff)) { // non MS3
                    next_inj[11].pin = 0x20;
                    next_inj[11].port = &PTP;
                } else {
                    next_inj[11].pin = 0x04;
                    next_inj[11].port = pPTM;
                }
            }

        } else {
            next_inj[i].port = pPTT;
            if ((ram4.Alternate && !oldout_staged) ||
               (oldout_staged)
                || (!(ram4.hardware & HARDWARE_MS3XFUEL) && (ram4.staged & 0x7))) { // V3 only + staged
                if (i & 0x01) {
                    /* ODD, use inj2 */
                    next_inj[i].pin = 0x08;
                } else {
                    /* EVEN, use inj1 */
                    next_inj[i].pin = 0x02;
                }
            } else {
                next_inj[i].pin = 0x0a;
            }
        }
    }
}

void calc_reqfuel(unsigned int reqfuel_in)
{
    unsigned char reqfuel_multiplier;
    unsigned long utmp1;

    orig_ReqFuel = reqfuel_in;
    orig_divider = ram4.Divider;
    orig_alternate = ram4.Alternate;

    calc_divider();
    /* In here we set up some critical internal variables that are used in other parts of ms3_inj
     * and ms3_ign_in. These and others are:
     * num_inj_outs is the number of outputs per event
     * num_inj_events is the number of expected events
     * num_inj is the number of actual injectors
     * do_dualouts is set for dual fuel
     */

    if (ram4.sequential & 0x3) {
        if (ram4.sequential & SEQ_FULL) {
            num_inj_outs = 1;   // 1 out per event;
            num_inj_events = num_cyl;
            num_inj = num_cyl;

            reqfuel_multiplier = num_cyl / divider;

            if ((ram4.Alternate & 0x01) && (reqfuel_multiplier > 1)) {
                reqfuel_multiplier >>= 1;
            }

            ReqFuel = (unsigned long) reqfuel_in * reqfuel_multiplier;

            if (num_cyl < num_inj) {
                utmp1 = num_cyl * (unsigned long)ReqFuel;
                ReqFuel = utmp1 / num_inj;
            } else {
                utmp1 = num_inj * (unsigned long)ReqFuel;
                ReqFuel = utmp1 / num_cyl;
            }

            EAE_multiplier = 1;
        } else if ((ram4.sequential & SEQ_SEMI) && (ram4.hardware & HARDWARE_MS3XFUEL)) {
            if ((ram4.EngStroke & 0x01) && (!(flagbyte6 & FLAGBYTE6_DONEINIT))) {
                conf_err = 112; // semi-seq doesn't make sense
            }
            if (cycle_deg == 7200) {
                num_inj_events = num_cyl; // need to output twice in semi-seq
            } else {
                num_inj_events = num_cyl >> 1;
            }

            if ((do_dualouts) && (num_cyl > 4) && !(ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3)) {
                if ((num_cyl == 6) || (num_cyl == 8)) {
                    // 2 injectors per output case - now supported
                    num_inj_outs = 2;
                    num_inj = num_cyl >> 1;
                } else {
                    conf_err = 109; // other combinations not supported
                }
            } else {
                num_inj_outs = 2;
                num_inj = num_cyl;
            }

            reqfuel_multiplier = num_cyl / divider;

            if ((ram4.Alternate & 0x01) && (reqfuel_multiplier > 1)) {
                reqfuel_multiplier >>= 1;
            }

            ReqFuel = (unsigned long) reqfuel_in * reqfuel_multiplier;
            ReqFuel >>= 1; // semi-seq squirts half the fuel every 360deg compared to seq every 720deg

            if (num_cyl < num_inj) {
                utmp1 = num_cyl * (unsigned long)ReqFuel;
                ReqFuel = utmp1 / num_inj;
            } else {
                utmp1 = num_inj * (unsigned long)ReqFuel;
                ReqFuel = utmp1 / num_cyl;
            }

            EAE_multiplier = 1;
        } else { /* Semi-seq on standard v3 injectors */
            OCPD |= 0xA;
            /* For this mode, just use whatever the tuning
             * software sets. It is correct.
             */
            num_inj_events = num_cyl / divider;
            if (cycle_deg == 3600) {
                num_inj_events >>= 1;
            }

            if ((num_inj_events < 2) && (!(flagbyte6 & FLAGBYTE6_DONEINIT))) {
                /* These don't make sense... need 720 degrees of data to do properly, but
                 * only have 360, or need to skip an entire half the altogether
                 */
                conf_err = 102; // only report during init phase
            }

            num_inj_outs = num_inj_events >> 1;
            ReqFuel = reqfuel_in;

            EAE_multiplier = num_inj_outs;

            if (!(ram4.Alternate & 0x1)) {
                EAE_multiplier <<= 1;
            }

            EAE_multiplier /= (num_cyl / (ram4.NoInj & 0x1f));

            if (!EAE_multiplier) {
                EAE_multiplier = 1;
            }

            num_inj = 2;
        }
    } else { /* Non seq */
        num_inj_events = num_cyl / divider;
        num_inj_outs = num_inj_events >> 1;
        ReqFuel = reqfuel_in;

        EAE_multiplier = num_inj_outs;

        if (!(ram4.Alternate & 0x1)) {
            EAE_multiplier <<= 1;
        }

        EAE_multiplier /= (num_cyl / (ram4.NoInj & 0x1f));

        if (!EAE_multiplier) {
            EAE_multiplier = 1;
        }

        if (ram4.hardware & HARDWARE_MS3XFUEL) {
            num_inj_outs = 1; /* fix for Mariob 4sq -> 8inj */
            num_inj = ram4.NoInj & 0x1f;
            if ((ram4.EngStroke & 0x01) && (!(flagbyte6 & FLAGBYTE6_DONEINIT))) {
                if (spkmode != 14) { // not twin trigger
                    conf_err = 112; // non-seq MS3X doesn't make sense with rotary or 2-stroke
                }
            }
        } else {
            num_inj = 2;
        }
    }
}

/**************************************************************************
 **
 ** Cranking Mode
 **
 ** Pulsewidth is directly set by the coolant temperature to a value of
 **  CWU (at temp_table[0]) and CWH (at temp_table[NO_TEMPS -1]).
 **  The value is interpolated at clt.
 **
 **************************************************************************/

char crank_calcs(void)
{
    unsigned int utmp1 = 0, utmp2 = 0;

    if (((outpc.rpm < ram4.crank_rpm)
         && (flagbyte2 & flagbyte2_crank_ok))
        /*|| (!(flagbyte3 & flagbyte3_toothinit)) */
        ) {
        unsigned char w = 0;

        tcrank_done = 0xFFFF;
        DISABLE_INTERRUPTS;
        running_seconds = 0;
        ENABLE_INTERRUPTS;
        outpc.engine |= ENGINE_CRANK;   // set cranking bit
        outpc.engine &= ~(ENGINE_ASE | ENGINE_WUE);  // clr starting warmup bit & warmup bit
        if (outpc.tps > ram4.TPSWOT) {
            tmp_pw1 = 0;        // usec  (0 ms for Flood Clear)
            tmp_pw2 = 0;
            return 1;
        }
        // cranking pulse width now uses a %age lookup table calculated from ReqFuel.
        // Hopefully new users can leave this table alone and engine will start
        // Changing injector size is corrected by ReqFuel

        if (ram4.Alternate & 0x04) {
            w = 3;
        } else if (pin_dualfuel && (ram5.dualfuel_sw2 & 0x02) && (!(*port_dualfuel & pin_dualfuel))) {
            w = 2;
        } else {
            w = 1;
        }

        if (w & 1) {
            utmp1 = CW_table(outpc.clt, (int *) ram_window.pg8.CrankPctTable,
                         (int *) ram_window.pg8.temp_table_p5, 8);
        }
        if (w & 2) {
            utmp2 = CW_table(outpc.clt, (int *) ram_window.pg21.CrankPctTable2,
                         (int *) ram_window.pg21.temp_table_p21, 21);
        }

        if (w == 2) { // switched
            utmp1 = utmp2;
        } else if (w == 3) {  // blended
            unsigned char blend;
            blend = intrp_1dctable(blend_xaxis(ram5.blend_opt[6] & 0x1f), 9, 
                    (int *) ram_window.pg25.blendx[6], 0, 
                    (unsigned char *)ram_window.pg25.blendy[6], 25);
            utmp1 = (unsigned int)((((unsigned long)utmp1 * (100 - blend)) + ((unsigned long)utmp2 * blend)) / 100);
        }
        // else just drop through using crank%1

        if ((ram4.sequential & SEQ_SEMI) && (ram4.hardware & HARDWARE_MS3XFUEL)) {
            utmp1 >>= 1;
        } else if ((ram4.sequential & SEQ_SEMI) && (!(ram4.hardware & HARDWARE_MS3XFUEL))) {
            utmp1 /= num_inj_events;

            if (ram4.Alternate & 0x01) {
                utmp1 <<= 1;
            }
        } else if (!(ram4.sequential & 0x03)) {
            if (ram4.Alternate & 0x02) {
                utmp1 <<= 1;
            }

            utmp1 /= divider;

            if (ram4.Alternate & 0x01) {
                utmp1 >>= 1;
            }
        }

        {
            /* do crank multiply/divide and check for overflow */
            unsigned long ultmp_cr;
            ultmp_cr = (unsigned long)utmp1 * ReqFuel;
            if (ultmp_cr > 6500000) {
                utmp1 = 65000;
            } else {
                utmp1 = (unsigned int)(ultmp_cr / 100);
            }
        }

        if ((ram5.dualfuel_sw & 0x1) && !(ram5.dualfuel_opt & 0x4) && (ram5.dualfuel_opt & 0x8) && pin_dualfuel) {
            // Dual Fuel on, Switching, different outputs
            if (!(*port_dualfuel & pin_dualfuel)) {
                // if active, then kill PW1
                tmp_pw1 = 0;
                tmp_pw2 = (unsigned long) utmp1 *100;
            } else {
                // else kill PW2
                tmp_pw1 = (unsigned long) utmp1 *100;
                tmp_pw2 = 0;
            }
        } else { // normal mode
        tmp_pw1 = (unsigned long) utmp1 *100;
        tmp_pw2 = (unsigned long) utmp1 *100;
        }
        return 2;
    } else if (tcrank_done == 0xFFFF) {
        tcrank_done = outpc.seconds;
    } else {
        if (outpc.seconds > tcrank_done + 5) {
            flagbyte2 &= ~flagbyte2_crank_ok;
            if ((spkmode == 2) && (ram4.spk_conf2 & 1)) {
                // Set HEI bypass to 5v
                if (ram4.hardware & 0x02) {
                    SSEM0SEI;
                    PORTB |= 0x02;
                    CSEM0CLI;
                } else {
                    SSEM0SEI;
                    PTM &= ~0x10;
                    CSEM0CLI;
                }
            }
        }
    }

    return 0;
}

/**************************************************************************
 **
 ** Warm-up and After-start Enrichment Section
 **
 ** The Warm-up enrichment is a linear interpolated value for the current clt
 ** temperature from warmen_table[NO_TEMPS] vs temp_table[NO_TEMPS]. The
 ** interpolation is done in subroutine warmcor_table.
 **
 ** Also, the after-start enrichment value is calculated and applied here - it
 **  is an added percent value on top of the warmup enrichment, and it is applied
 **  for the number of ignition cycles specified in AWC. This enrichment starts
 **  at a value of AWEV at first, then it linearly interpolates down to zero
 **  after AWC cycles.
 **
 **  If (startw, engine is set) then:
 **   compare if (AWC > 0) then:
 **    interpolate for warmup enrichment
 **   else clear startw bit in engine
 **
 **************************************************************************/

void warmup_calcs(void)
{
    int wrmtmp;
    unsigned int utmp1;
    static unsigned int tASTol = 0;

    wrmtmp = outpc.warmcor;
    if (outpc.engine & ENGINE_CRANK) {  // if engine cranking
        outpc.engine &= ~ENGINE_CRANK;  // clear crank bit
        outpc.engine |= ENGINE_ASE | ENGINE_WUE;       // set starting warmup bit (ase) & warmup bit
//            asecount = 0;
        tASTol = outpc.seconds;

        set_prime_ASE();
        DISABLE_INTERRUPTS;
        running_seconds = 0;
        ENABLE_INTERRUPTS;
    }
    if (pin_dualfuel && (ram5.dualfuel_sw & 0x40) && (!(*port_dualfuel & pin_dualfuel))) {
        wrmtmp = intrp_1dctable(outpc.clt, NO_TEMPS, (int *) ram_window.pg21.temp_table_p21,
                                0, (unsigned char *) ram_window.pg21.warmen_table2, 21);     // %
    } else {
        wrmtmp = intrp_1dctable(outpc.clt, NO_TEMPS, (int *) ram_window.pg10.temp_table,
                                0, (unsigned char *) ram_window.pg10.warmen_table, 8);     // %
    }
    outpc.cold_adv_deg = CW_table(outpc.clt, (int *) ram_window.pg10.cold_adv_table,
                                 (int *) ram_window.pg10.temp_table, 10);   // deg x 10

    if (wrmtmp == 100) {        // done warmup
        outpc.engine &= ~ENGINE_WUE;  // clear start warmup bit (ase) & warmup bit
        SSEM0SEI;
        *pPTMpin5 &= ~0x20;   // clear warmup led
        CSEM0CLI;
    } else {
        SSEM0SEI;
        *pPTMpin5 |= 0x20;    // set warmup led
        CSEM0CLI;
        outpc.engine |= ENGINE_WUE;   // set warmup bit
    }

    if (!(outpc.engine & ENGINE_ASE)) {       // if starting warmup bit clear
        goto END_WRM;
    }

    if ((asecount > AWC) || (AWEV == 0)) {
        outpc.engine &= ~ENGINE_ASE;  // clear start warmup bit
        goto END_WRM;
    }

    if (AWC > 0) {
//        utmp1 = (unsigned long)(AWEV * (AWC - asecount)) / AWC;
        __asm__ __volatile__ (
        "tfr d,x\n"
        "subd %1\n"
        "ldy  %2\n"
        "emul\n"
        "ediv\n"
        : "=y" (utmp1)
        : "m" (asecount), "m" (AWEV), "d" (AWC)
        : "x"
        );

        wrmtmp += utmp1;
    }
  END_WRM:
    outpc.warmcor = wrmtmp;
}

/**************************************************************************
 **
 **  Throttle Position Acceleration Enrichment
 **
 **   Method is the following:
 **
 **
 **   ACCELERATION ENRICHMENT:
 **   If (tpsdot < 0) goto DEACCELERATION_ENRICHMENT
 **   If tpsdot > tpsthresh and TPSAEN bit = 0 then (acceleration enrichment):
 **   {
 **    1) Set acceleration mode
 **    2) Continuously determine rate-of-change of throttle, and perform
 **        interpolation of table values to determine acceleration
 **        enrichment amount to apply.
 **   }
 **   If (TPSACLK > TpsAsync) and TPSAEN is set then:
 **   {
 **    1) Clear TPSAEN bit in engine
 **    2) Set TPSACCEL to 0
 **    3) Go to EGO Delta Step Check Section
 **   }
 **   Enrichment tail-off pulsewidth:
 **
 **   ------------------ tpsaccel
 **   |            |\
 **   |            |   \
 **   |            |      \
 **   |            |         \          ____ TpsAccel2
 **   |            |            |
 **   |            |            |
 **   ---------------------------
 **   <--TpsAsync--><-TpsAsync2->
 **
 **
 **   DEACCELERATION ENRICHMENT:
 **   If (-tpsdot) > tpsthresh then (deceleration fuel cut)
 **   {
 **    If (TPSAEN = 1) then:
 **    {
 **      1) TPSACCEL = 0 (no acceleration)
 **      2) Clear TPSAEN bit in ENGINE
 **      3) Go to EGO Delta Step
 **    }
 **    If (RPM > 1500 then (fuel cut mode):
 **    {
 **      1) Set TPSACCEL value to TPSDQ
 **      2) Set TPSDEN bit in ENGINE
 **      3) Go to EGO Delta Step Check Section
 **    }
 **   }
 **   else
 **   {
 **    If (TPSDEN = 1) then
 **    {
 **     1) Clear TPSDEN bit in ENGINE
 **     2) TPSACCEL = 0
 **     3) Go to EGO Delta Step Check Section
 **    }
 **   }
 **
 **************************************************************************/

void normal_accel(void)
{
    int tpsatmp, ae_scaler;
    int tmp1, tmp2, tmp3, tmp4;

    if (!(ram5.AE_options & USE_NEW_AE)) {
        tpsatmp = outpc.tpsaccel;
        DISABLE_INTERRUPTS;
        tpsdot_ltch = outpc.tpsdot;
        mapdot_ltch = outpc.mapdot;
        ENABLE_INTERRUPTS;

        if ((tpsdot_ltch < 0) && (ram4.Tps_acc_wght != 0)) {
            goto TDE;               // decelerating
        }
        if ((mapdot_ltch < 0) && (ram4.Tps_acc_wght != 100)) {
            goto TDE;
        }
        if (outpc.engine & ENGINE_TPSACC) {
            goto AE_COMP_SHOOT_AMT; // if accel enrich bit set
        }
        if (outpc.engine & ENGINE_MAPACC) {
            goto AE_COMP_SHOOT_AMT;
        }
        if (((ram4.Tps_acc_wght == 0) || (tpsdot_ltch < ram4.TpsThresh))
                && ((ram4.Tps_acc_wght == 100) || (mapdot_ltch < ram4.MapThresh))) {
            goto TAE_CHK_TIME;
        }
        // start out using first element - determine actual next time around

        tpsatmp = (((short) ram4.tpsen_table[0] * ram4.Tps_acc_wght) +
                ((short) ram4.mapen_table[0] *
                 (100 - ram4.Tps_acc_wght))) / 100;
        tpsatmp = (unsigned char) (((unsigned long) tpsatmp * ReqFuel) / (unsigned long) 10000);    // .1 ms units

        tpsaccel_end = tpsatmp;     // latch last tpsaccel before tailoff
        tpsaclk = 0;                // incremented in .1 sec timer

        if ((tpsdot_ltch > ram4.TpsThresh) && (ram4.Tps_acc_wght != 0)) {
            outpc.engine |= ENGINE_TPSACC;   // set tpsaen bit
        }
        if ((mapdot_ltch > ram4.MapThresh) && (ram4.Tps_acc_wght != 100)) {
            outpc.engine |= ENGINE_MAPACC;
        }
        outpc.engine &= ~ENGINE_TPSDEC;      // clear tpsden bit
        outpc.engine &= ~ENGINE_MAPDEC;
        SSEM0SEI;
        *pPTMpin4 |= 0x10;        // set accel led
        CSEM0CLI;
        goto END_TPS;
        /* First, calculate Cold temperature add-on enrichment value from coolant value
           TPSACOLD at min temp & 0 at high temp.

           Then determine cold temperature multiplier value ACCELMULT (in percent).

           Next, Calculate Shoot amount (quantity) for acceleration enrichment from table.
           Find bins (between) for corresponding TPSDOT and MAPDOT, and linear interpolate
           to find enrichment amount from table. This is continuously
           checked every time thru main loop while in acceleration mode,
           and the highest value is latched and used.

           The final acceleration enrichment (in .1 ms units) applied is AE =
           (((Alookup(TPSDOT)*Tps_acc_wght + Alookup(MAPDOT)*(100 - Tps_acc_wght))
           /100) * ACCELMULT/100) + TPSACOLD.
         */

AE_COMP_SHOOT_AMT:
        if (tpsaclk < ram4.TpsAsync) { // was <=
            // direct use of table data without calling interpolation routine
            RPAGE = tables[10].rpg; // HARDCODED
            tmp1 = (short) ram4.Tpsacold - (short) ((ram4.Tpsacold *
                        (long) (outpc.clt - ram_window.pg10.temp_table[0])
                        / (ram_window.pg10.temp_table[NO_TEMPS - 1] - ram_window.pg10.temp_table[0])));   // in .1 ms
            tmp2 = (short) ram4.AccMult + (short) (((100 - ram4.AccMult) *
                        (long) (outpc.clt - ram_window.pg10.temp_table[0])
                        / (ram_window.pg10.temp_table[NO_TEMPS - 1] - ram_window.pg10.temp_table[0])));    // in %

            if (ram4.Tps_acc_wght > 0) {
                tmp3 = (unsigned char) (((unsigned long) intrp_1dctable(tpsdot_ltch, NO_TPS_DOTS, (int *) ram4.tpsdot_table, 0, (unsigned char *) ram4.tpsen_table, 10) * ReqFuel) / (unsigned long) 10000);
            } else {
                tmp3 = 0;
            }

            if (ram4.Tps_acc_wght < 100) {
                tmp4 = (unsigned char) (((unsigned long) intrp_1dctable(mapdot_ltch, NO_MAP_DOTS, (int *) ram4.mapdot_table, 0, (unsigned char *) ram4.mapen_table, 10) * ReqFuel) / (unsigned long) 10000);
            } else {
                tmp4 = 0;
            }
            tmp3 = ((tmp3 * ram4.Tps_acc_wght) + (tmp4 * (100 - ram4.Tps_acc_wght))) / 100;

            tmp3 = (tmp3 * tmp2) / 100;

            if (tmp3 > 200) {
                tmp3 = 200;         // rail at 200%
            }

            tmp3 += tmp1;
            if (tmp3 > tpsatmp) {
                tpsatmp = tmp3;     // make > tps/mapen_table entry for lowest tps/mapdot
            }
            // plus latch and hold largest pw
            //        tpsaccel_end = tpsatmp; // latch last tpsaccel before tailoff (original non working line)
            if (tpsatmp > tpsaccel_end) {
                tpsaccel_end = tpsatmp;
            } else {
                tpsatmp = tpsaccel_end;
            }      
        } else {                    // tailoff enrichment pulsewidth
TAILOFF:
            if (ram4.TpsAsync2 > 0) {
                unsigned int tmp_endpw;
                tmp_endpw = (unsigned int) (((unsigned long) ram4.TpsAccel2 * ReqFuel) / (unsigned long) 10000);
                tpsatmp = tpsaccel_end + ((((int)tmp_endpw - (int)tpsaccel_end)
                            * (long) (tpsaclk - ram4.TpsAsync)) / ram4.TpsAsync2);
            } else {
                tpsatmp = 0;
            }
        }

TAE_CHK_TIME:
        // check if accel is done
        // if tps decel bit not set, accel bit is
        if ((!(outpc.engine & ENGINE_TPSDEC) && (outpc.engine & ENGINE_TPSACC)) ||
                (!(outpc.engine & ENGINE_MAPDEC) && (outpc.engine & ENGINE_MAPACC))) {
            if (tpsaclk < (ram4.TpsAsync + ram4.TpsAsync2)) {
                goto END_TPS;
            }
        }
        goto KILL_ACCEL;

TDE:                         // tpsdot < 0
        if ((outpc.engine & ENGINE_TPSACC) || (outpc.engine & ENGINE_MAPACC)) {       // if tps accel bit set
            if ((-tpsdot_ltch) > ram4.TpsThresh) {
                goto KILL_ACCEL;
            }
            if ((-mapdot_ltch) > ram4.MapThresh) {
                goto KILL_ACCEL;
            }
            /* on the bench this next section kills off AE once TPS stops increasing */
            if (tpsaclk < ram4.TpsAsync) {
                tpsaclk = ram4.TpsAsync;    // just tail off AE
            }
            goto TAILOFF;
        }
        // decel
        if (((-tpsdot_ltch) > ram4.TpsThresh) && (ram4.TPSDQ != 0) &&
                (ram4.Tps_acc_wght > 0)) {
            if (outpc.rpm < 1500) {
                goto END_TPS;
            }
            outpc.tpsfuelcut = ram4.TPSDQ;  // in %
            outpc.engine |= ENGINE_TPSDEC;   // set tps decel bit
            goto END_TPS;
        }
        if (((-mapdot_ltch) > ram4.MapThresh) && (ram4.TPSDQ != 100) &&
                (ram4.Tps_acc_wght < 100)) {
            if (outpc.rpm < 1500) {
                goto END_TPS;
            }
            outpc.tpsfuelcut = ram4.TPSDQ;
            outpc.engine |= ENGINE_MAPDEC;
            goto END_TPS;
        }
        if (outpc.engine & (ENGINE_TPSACC | ENGINE_TPSDEC | ENGINE_MAPACC | ENGINE_MAPDEC)) {  // if decel or just finished accel
KILL_ACCEL:
            outpc.engine &= ~(ENGINE_TPSACC | ENGINE_TPSDEC | ENGINE_MAPACC | ENGINE_MAPDEC);  // clear tps decel, accel bits
            outpc.tpsfuelcut = 100;
            SSEM0SEI;
            *pPTMpin4 &= ~0x10;   // clear accel led
            CSEM0CLI;
            tpsatmp = 0;
        }
END_TPS:
   
        // apply AE rpm-based scaler
        if ((outpc.rpm <= ram4.ae_lorpm) || (ram4.feature7 & 0x08)) { // if events then ignore down-scaling
            ae_scaler = 100;  
        } else if (outpc.rpm >= ram4.ae_hirpm) {
            ae_scaler = 0;
        } else {
            ae_scaler = (short) (((long) 100 * (ram4.ae_hirpm - outpc.rpm)) /
                    (ram4.ae_hirpm - ram4.ae_lorpm));
        }

        outpc.tpsaccel = (tpsatmp * ae_scaler) / 100;
    }
}

int new_accel_calc_percent(int x, int *x_table, int *z_table, int dot_threshold,
                           unsigned char engine_acc_flag,
                           unsigned char engine_dec_flag)

{
    int tmpdot, accel_percent = 0, tmp2;

    if (x < 0) {
        tmpdot = -x;
    } else {
        tmpdot = x;
    }

    if (tmpdot >= dot_threshold) {
        accel_percent = intrp_1ditable(x, 8, x_table, 1, 
                z_table, 5);
        // direct use of table data without calling interpolation routine
        RPAGE = tables[10].rpg; // HARDCODED
        tmp2 = (int)ram5.accel_CLT_multiplier + (int) (((1000 - ram5.accel_CLT_multiplier) *
                    (long) (outpc.clt - ram_window.pg10.temp_table[0])
                    / (ram_window.pg10.temp_table[NO_TEMPS - 1] -
                        ram_window.pg10.temp_table[0])));

        accel_percent = ((long)accel_percent * tmp2) / 1000;

        if (x > 0) {
            outpc.engine |= engine_acc_flag;
            outpc.engine &= ~engine_dec_flag;
        } else {
            outpc.engine &= ~engine_acc_flag;
            outpc.engine |= engine_dec_flag;
        }
    } else {
        outpc.engine &= ~engine_acc_flag;
        outpc.engine &= ~engine_dec_flag;
    }

    return accel_percent;
}

void new_accel(long *lsum1, long *lsum2)
{
    /* This is the new accel idea. Basically it is a percentage of the
     * ReqFuel for this iteration only. No timers or
     * anything like that. Just measures TPSdot, looks up the appropriate
     * value, sets the percentage, done.
     */

    int tps_percent, map_percent, ae_scaler;
    long ltmp;

    if (ram5.AE_options & USE_NEW_AE) {
        if (ram5.accel_blend_percent > 0) {
            tps_percent = new_accel_calc_percent(outpc.tpsdot, ram5.accel_tpsdots,
                    ram5.accel_tpsdot_amts,
                    ram5.accel_tpsdot_threshold,
                    ENGINE_TPSACC,
                    ENGINE_TPSDEC);
            outpc.tps_accel = (ram5.accel_blend_percent * (long)tps_percent) / 1000;
        } else {
            /* 0% TPS AE */
            outpc.tps_accel = 0;
        }
        if (ram5.accel_blend_percent < 1000) {
            map_percent = new_accel_calc_percent(outpc.mapdot, ram5.accel_mapdots,
                    ram5.accel_mapdot_amts,
                    ram5.accel_mapdot_threshold,
                    ENGINE_MAPACC,
                    ENGINE_MAPDEC);

            outpc.map_accel = ((1000 - ram5.accel_blend_percent) * (long)map_percent) / 1000;
        }

        if (outpc.rpm <= ram4.ae_lorpm) {
            ae_scaler = 1000;
        } else if (outpc.rpm >= ram4.ae_hirpm) {
            ae_scaler = 0;
        } else {
            ae_scaler = (short) (((long) 1000 * (ram4.ae_hirpm - outpc.rpm)) /
                    (ram4.ae_hirpm - ram4.ae_lorpm));
        }

        outpc.total_accel = (int)(((long)(outpc.tps_accel + outpc.map_accel) *
                    ae_scaler) / 1000L);

        ltmp = ((outpc.total_accel * (long)ReqFuel) / 10L);

        *lsum1 += ltmp;
        /* bound to 0-65535 usec */
        if (*lsum1 > 6553500) {
            *lsum1 = 6553500;
        } else if (*lsum1 < 0) {
            *lsum1 = 0;
        }

        *lsum2 += ltmp;
        /* bound to 0-65535 usec */
        if (*lsum2 > 6553500) {
            *lsum2 = 6553500;
        } else if (*lsum2 < 0) {
            *lsum2 = 0;
        }

        /* Calculate old variable for user interface consistency.
           The addition to PW in the mainloop isn't run when Accel-pump is enabled. */
        outpc.tpsaccel = (unsigned int) (ltmp / 10000);
    }
}

void main_fuel_calcs(long *lsum1, long *lsum2)
{
    unsigned char uctmp, v = 0, w = 0;
    int ve1 = 1000, ve2 = 1000, ve3 = 1000, ve4 = 1000;

    /**************************************************************************
     **  Look up volumetric efficiency as function of rpm and map (tps
     **   in alpha N mode)
     **************************************************************************/
    uctmp = 0;                  //uctmp is local tmp var
    if ((ram5.dualfuel_sw & 0x03) == 0x03) { // on and fuel
        if (pin_tsf && !(*port_tsf & pin_tsf)) {       // Hardware table switching
            uctmp = 1;
        }
    } else {
        if (ram4.feature5_0 & 2) {      //  table switching
            if ((ram4.feature5_0 & 0x0c) == 0) {
                if (pin_tsf && !(*port_tsf & pin_tsf)) {       // Hardware table switching
                    uctmp = 1;
                }
            } else if ((ram4.feature5_0 & 0x0c) == 4) {
                if (outpc.rpm > ram4.tsf_rpm) {
                    uctmp = 1;
                }
            } else if ((ram4.feature5_0 & 0x0c) == 8) {
                if (outpc.map > ram4.tsf_kpa) {
                    uctmp = 1;
                }
            } else if ((ram4.feature5_0 & 0x0c) == 0xc) {
                if (outpc.tps > ram4.tsf_tps) {
                    uctmp = 1;
                }
            }
        }
    }

    if (uctmp) {
        outpc.status1 |= STATUS1_FTBLSW;
    } else {
        outpc.status1 &= ~STATUS1_FTBLSW;
    }

    if ((ram5.dualfuel_sw & 0x1) && (ram5.dualfuel_opt & 0x4)) {  /* dual outputs */
        w = 7;
    } else if ((ram4.feature5_0 & 2) && ((ram4.feature5_0 & 0x0c) == 0) && (ram4.tsw_pin_f & 0x1f) == 14) { /* blend from VE1+2 to VE3+4 */
        w = 3;
    } else if (uctmp == 1) { /* table switched */
        w = 2;
    } else {
        w = 1;
    }

    if (((ram4.FuelAlgorithm & 0xf) == 5) && (!(ram4.feature7 & 0x02))) {
        /* Pure MAF algorithm takes VE1 as 100% */
        v = 0;
    } else {
        /* non MAF or MAF with VE tweak table */
        v = w;
    }

    if (v & 1) {
        ve1 = intrp_2ditable(outpc.rpm, outpc.fuelload, NO_EXFRPMS, NO_EXFMAPS, &ram_window.pg19.frpm_tablev1[0],
                  &ram_window.pg19.fmap_tablev1[0], (unsigned int *) &ram_window.pg12.ve_table1[0][0], 12);
    }

    if (v & 2) {
        ve3 = intrp_2ditable(outpc.rpm, outpc.fuelload, NO_EXFRPMS, NO_EXFMAPS, &ram_window.pg19.frpm_tablev3[0],
                  &ram_window.pg19.fmap_tablev3[0], (unsigned int *) &ram_window.pg18.ve_table3[0][0], 18);
    }

    if (v == 2) {
        ve1 = ve3; /* store switched result to ve1 */
    }
    /* No change for Single table unswitched, Dual table or ve1+2 -> ve3+4 blend (1) */

    /* Secondary load options (VE2 + VE4) */
    if (ram4.FuelAlgorithm & 0xf0) {
        if (((ram4.FuelAlgorithm & 0xf0) == 0x50) && (!(ram4.feature7 & 0x02))) {
            /* Pure MAF algorithm takes VE2 as 100% */
            v = 0;
        } else {
            /* non MAF or MAF with VE tweak table */
            v = w;
        }

        if (w & 1) {
            ve2 = intrp_2ditable(outpc.rpm, outpc.fuelload2, NO_EXFRPMS, NO_EXFMAPS, &ram_window.pg19.frpm_tablev2[0],
                    &ram_window.pg19.fmap_tablev2[0], (unsigned int *) &ram_window.pg12.ve_table2[0][0], 12);
        }
        if (w & 2) {
            ve4 = intrp_2ditable(outpc.rpm, outpc.fuelload2, NO_EXFRPMS, NO_EXFMAPS, &ram_window.pg21.frpm_tablev4[0],
                    &ram_window.pg21.fmap_tablev4[0], (unsigned int *) &ram_window.pg22.ve_table4[0][0], 22);
        }

        if (w == 2) {
            ve2 = ve4; /* store switched result to ve2 */
        }

        if ((ram4.loadopts & 0x3) == 1) {
            /* multiply */
            ve1 = ((long)ve1 * ve2) / 1000;
            if (w == 7) {
                ve3 = ((long)ve3 * ve4) / 1000;
            }
        } else if ((ram4.loadopts & 0x3) == 0) {
            /* add */
            ve1 = ve1 + ve2;
            if (w == 7) {
                ve3 = ve3 + ve4;
            }
        }
    }

    /* Idle advance overrides VE1 and VE3 calcs */
    if (ram4.idle_special_ops & IDLE_SPECIAL_OPS_IDLEVE) {
        if ((!(ram4.idleveadv_to_pid & IDLEVEADV_TO_PID_IDLEVE) && 
            (outpc.tps < ram4.idleve_tps) &&
            (outpc.rpm < ram4.idleve_rpm) &&
            (outpc.fuelload > ram4.idleve_load) &&
            (outpc.clt > ram4.idleve_clt) &&
            (((ram4.idle_special_ops & 0x0c) == 0) ||
             (((ram4.idle_special_ops & 0x0c) == 0x04) && (outpc.vss1 < 20)) || // less that 2.0 ms-1
             (((ram4.idle_special_ops & 0x0c) == 0x08) && (outpc.vss2 < 20)))) ||
             ((ram4.idleveadv_to_pid & IDLEVEADV_TO_PID_IDLEVE) &&
              (outpc.status2 & 0x80))
            ) {
            if (((idle_ve_timer >= ram4.idleve_delay) && 
                (flagbyte4 & flagbyte4_idlevereset)) || 
                ((ram4.idleveadv_to_pid & IDLEVEADV_TO_PID_IDLEVE) &&
                 (outpc.status2 & 0x80))) {
                outpc.status6 |= STATUS6_IDLEVE;
                if (w & 1) {
                    ve1 = intrp_2ditable(outpc.rpm, outpc.fuelload, 4, 4,
                                       &ram_window.pg19.idleve_rpms[0][0],
                                       &ram_window.pg19.idleve_loads[0][0],
                                       (unsigned int *) &ram_window.pg19.idleve_table1[0][0], 19);
                }

                if (w & 2) {
                    ve3 = intrp_2ditable(outpc.rpm, outpc.fuelload, 4, 4,
                                           &ram_window.pg19.idleve_rpms[1][0],
                                           &ram_window.pg19.idleve_loads[1][0],
                                           (unsigned int *) &ram_window.pg19.idleve_table2[0][0], 19);
                }

                if (w == 2) {
                    ve1 = ve3;  /* store switched result to ve1 */
                }

            } else {
                outpc.status6 &= ~STATUS6_IDLEVE;
                if (!(flagbyte4 & flagbyte4_idlevereset)) {
                    DISABLE_INTERRUPTS;
                    idle_ve_timer = 0;
                    ENABLE_INTERRUPTS;
                    flagbyte4 |= flagbyte4_idlevereset;
                }
            }
        } else {
            flagbyte4 &= ~flagbyte4_idlevereset;
            outpc.status6 &= ~STATUS6_IDLEVE;
        }
    }
    /* end of idle advance */

    /* If blending, at this point we've calculated individual VE1,2,3,4 values */
    if (w == 3) {
        /* ve1/ve3 and ve2/ve4 blend (3) into single ve1 and ve2
          this is the up/down blend - incompatible with dual outputs*/
        unsigned char blend;

        blend = intrp_1dctable(blend_xaxis(ram5.blend_opt[2] & 0x1f), 9, 
                (int *) ram_window.pg25.blendx[2], 0, 
                (unsigned char *)ram_window.pg25.blendy[2], 25);
        ve1 = (int)((((long)ve1 * (100 - blend)) + ((long)ve3 * blend)) / 100);
        ve2 = (int)((((long)ve2 * (100 - blend)) + ((long)ve4 * blend)) / 100);
    }
    if (w != 7) { // everything except dual outputs
        ve3 = ve1;
        ve4 = ve2;
    }
    /* Now the VE1,3 are combined to VE1  and VE2,4 are combined to VE2 */
    /* With dual outputs, w == 5 so all individuals are still there */

    outpc.vecurr1 = ve1;

    if (w == 7) { // dual outputs
        outpc.vecurr2 = ve3;
    } else if (((ram4.loadopts & 0x3) == 2) && (ram4.FuelAlgorithm & 0xf0)) { // blended (1) and secondary algo defined
        outpc.vecurr2 = ve2;
    } else { // single output
        outpc.vecurr2 = outpc.vecurr1;
    }

    serial();
    /**************************************************************************
     **
     ** Computation of Fuel Parameters
     ** Note that GAMMAE only includes Warm, Tpsfuelcut, Barocor, and Aircor
     ** (EGO no longer included)
     **
     **************************************************************************/
    {
        int inj_divider1, inj_load1,  inj_divider2, inj_load2, gammae1, gammae2;
        long fuel_tmp1, fuel_tmp2, local_lsum1, local_lsum2, local_lsum3, local_lsum4;

        fuel_tmp1 = ((outpc.warmcor * (long) outpc.tpsfuelcut) / 10L);

        //Dual fuel correction tables
        /* now included in GammaE (2012-10-30) */
        if (pin_dualfuel && (!(*port_dualfuel & pin_dualfuel))) { // dual fuel switch active
            if (ram5.dualfuel_opt & 0x01) { // dual fuel switch - temperature table
                int adj, temperature, i;
                unsigned char t;
                i = ram5.dualfuel_temp_sens & 0x0f;
                temperature = outpc.sensors[i];

                t = ram5.sensor_trans[i];
                if (((t == 3) || (t == 4)) && (ram5.sensor_temp & 0x01)) {
                    // if using CLT, MAT sensor tranform and in degC need to reverse calc back to degF
                    temperature = ((temperature * 9) / 5) + 320;
                }
                adj = intrp_1ditable(temperature, 10,
                   (int *) ram_window.pg21.dualfuel_temp, 1,
                   (unsigned int *) ram_window.pg21.dualfuel_temp_adj, 21);
                if (adj < -1000) {
                    adj = -1000; // can't be less than -100%
                }
                fuel_tmp1 = (fuel_tmp1 * (1000 + adj)) / 1000L;
            }

            if (ram5.dualfuel_opt & 0x02) { // dual fuel switch - pressure table
                int adj, pressure;
                pressure = outpc.sensors[ram5.dualfuel_press_sens & 0x0f];

                // That gives gauge pressure. Now convert to differential
                pressure = pressure + outpc.baro - outpc.map;

                adj = intrp_1ditable(pressure, 10,
                   (int *) ram_window.pg21.dualfuel_press, 1,
                   (unsigned int *) ram_window.pg21.dualfuel_press_adj, 21);
                if (adj < -1000) {
                    adj = -1000; // can't be less than -100%
                }
                fuel_tmp1 = (fuel_tmp1 * (1000 + adj)) / 1000L;
            }
        }

        fuel_tmp2 = fuel_tmp1;

        /* ALGORITHM 1 */

        if ((ram4.FuelAlgorithm & 0xf) != 5) {
            /* Use baro and aircor for everything except primary MAF mode */
            fuel_tmp1 = (fuel_tmp1 * ((outpc.barocor * (long) outpc.aircor) / 1000L) / 1000L);
        }

        gammae1 = (int) (fuel_tmp1 / 10);

        if (((ram4.FuelAlgorithm & 0xf) == 5) || ((ram4.FuelAlgorithm & 0xf) == 4)) {
            if (ram4.feature7 & 0x01) {
                /* uses old-style +/- MAT correction curve instead of new exposed calibration */
                inj_load1 = (int)( ((long)mafload_no_air * 
                    (1000 + intrp_1ditable(outpc.mat, NO_MAFMATS, (int *)ram_window.pg25.MatVals, 1,
                            (unsigned int *)ram_window.pg25.AirCorDel,25) )) / 1000);
            } else {
                inj_load1 = mafload_no_air;
            }

            inj_divider1 = 1000;
            /* MAF algorithm uses MAF to calculate mafload_no_air (calc is in ms3_misc.c)
              mafload is a 'synthetic' load.
              mafload_no_air excludes the aircor variable to avoid cancelling it out again here.
              VE is taken as 100.0% as MAF ideally gives a perfect mass flow number.
              The rest of the fuel calc follows as normal.
              i.e. PW = RF * mafload_no_air * other_corrections
              cf. Speed density where PW = RF * map * VE(rpm,map) * airden * other_corrections
            */
        } else {
            if (ram4.loadopts & 0x4) { // multiply map
                inj_load1 = outpc.map;
            } else {
                inj_load1 = 1000;          // normalizes to ~1 when divide by baro
            }
            inj_divider1 = outpc.baro;
        }

        /* Above here fuel_tmp and inj_load are parts of the same calculation
           and do not refer to different 'banks' */

        /* now calculate by bank. lsum1 = bank1, lsum2 = bank2 for dual outputs */
        if ((ram4.EgoOption & 0x03) && (ram4.egonum > 1)) {
            // > 2 widebands, no ego here, apply to seq_pw at cylinder level calc
            local_lsum1 = fuel_tmp1 * ((1000 * (long)inj_load1) / inj_divider1) / 100L;
            local_lsum2 = local_lsum1;
        } else {
            // Do it the old way, apply egocor here
            local_lsum1 = fuel_tmp1 * ((outpc.egocor1 * (long)inj_load1) / inj_divider1) / 100L;
            local_lsum2 = fuel_tmp1 * ((outpc.egocor2 * (long)inj_load1) / inj_divider1) / 100L;
        }

        /* Now lsum1 and lsum2 have started to become tmp_pw1 and tmp_pw2 */

        /* ve will be 1000 for pure MAF */
        local_lsum1 = (local_lsum1 * ((ve1 * (long) ReqFuel) / 1000L)) / 100L;      // .01 usec
        local_lsum2 = (local_lsum2 * ((ve3 * (long) ReqFuel) / 1000L)) / 100L;      // .01 usec

        /* ALGORITHM 2 */
        if (((ram4.loadopts & 0x3) == 2) && (ram4.FuelAlgorithm & 0xf0)) { // blended (1) and secondary algo defined
            unsigned char blend;
            /* Note that local_lsum numbers don't match VE table numbers.
                local_lsum1 = VE1, local_lsum2 = VE3, local_lsum3 = VE2, local_lsum4 = VE4 */

            if ((ram4.FuelAlgorithm & 0xf0) != 0x50) {
                /* Use baro and aircor for everything except MAF mode */
                fuel_tmp2 = (fuel_tmp2 * ((outpc.barocor * (long) outpc.aircor) / 1000L) / 1000L);
            }

            gammae2 = (int) (fuel_tmp2 / 10);

            if (((ram4.FuelAlgorithm & 0xf0) == 0x50) || ((ram4.FuelAlgorithm & 0xf0) == 0x40)) {
                if (ram4.feature7 & 0x01) {
                    /* uses old-style +/- MAT correction curve instead of new exposed calibration */
                    inj_load2 = (int)( ((long)mafload_no_air * 
                        (1000 + intrp_1ditable(outpc.mat, NO_MAFMATS, (int *)ram_window.pg25.MatVals, 1,
                                (unsigned int *)ram_window.pg25.AirCorDel,25) )) / 1000);
                } else {
                    inj_load2 = mafload_no_air;
                }

                inj_divider2 = 1000;
            } else {
                if (ram4.loadopts & 0x20) { // multiply map (2nd)
                    inj_load2 = outpc.map;
                } else {
                    inj_load2 = 1000;          // normalizes to ~1 when divide by baro
                }
                inj_divider2 = outpc.baro;
            }

            /* Above here fuel_tmp and inj_load are parts of the same calculation
               and do not refer to different 'banks' */

            if ((ram4.EgoOption & 0x03) && (ram4.egonum > 1)) {
                // > 2 widebands, no ego here, apply to seq_pw at cylinder level calc
                local_lsum3 = fuel_tmp2 * ((1000 * (long)inj_load2) / inj_divider2) / 100L;
                local_lsum4 = local_lsum1;
            } else {
                // Do it the old way, apply egocor here
                local_lsum3 = (fuel_tmp2 * ((outpc.egocor1 * (long)inj_load2) / inj_divider2) / 100L);
                local_lsum4 = (fuel_tmp2 * ((outpc.egocor2 * (long)inj_load2) / inj_divider2) / 100L);
            }

            /* Now lsum1 and lsum2 have started to become tmp_pw1 and tmp_pw2 */

            /* vecurr will be 1000 for pure MAF */
            local_lsum3 = (local_lsum3 * ((ve2 * (long) ReqFuel) / 1000L)) / 100L;      // .01 usec
            local_lsum4 = (local_lsum4 * ((ve4 * (long) ReqFuel) / 1000L)) / 100L;      // .01 usec

            /* have now calculated both banks with two algos */
            /* ve1->2 blend and ve3->4 blend (1)
             this is the side/side blend */
            blend = intrp_1dctable(blend_xaxis(ram5.blend_opt[0] & 0x1f), 9, 
                    (int *) ram_window.pg25.blendx[0], 0, 
                    (unsigned char *)ram_window.pg25.blendy[0], 25);
            *lsum1 = (((local_lsum1 * (100 - blend)) + (local_lsum3 * blend)) / 100);
            *lsum2 = (((local_lsum2 * (100 - blend)) + (local_lsum4 * blend)) / 100);
            outpc.gammae = (int)((((long)gammae1 * (100 - blend)) + ((long)gammae2 * blend)) / 100);

        } else {
            /* normal un-blended output */
            *lsum1 = local_lsum1;
            *lsum2 = local_lsum2;
            outpc.gammae = gammae1;
        }
    }

    if ((ram5.dualfuel_sw & 0x1) && !(ram5.dualfuel_opt & 0x4) && (ram5.dualfuel_opt & 0x8) && pin_dualfuel) {
        // Dual Fuel on, Switching, different outputs
        if (!(*port_dualfuel & pin_dualfuel)) {
            outpc.vecurr1 = 0; // if active, then kill VE1 (secondary outputs only)
            *lsum1 = 0;
        } else {
            outpc.vecurr2 = 0; // else kill VE2 (primary outputs only)
            *lsum2 = 0;
        }
    }

    if (ram4.loadopts & 0x8) {  /* factor in afr target */
        int tmp_stoich;
        if (pin_tsw_stoich && (!(*port_tsw_stoich & pin_tsw_stoich))) {  // Stoich switching
            tmp_stoich = ram4.stoich_alt;
        } else {
            tmp_stoich = ram4.stoich;
        }
        *lsum1 = (*lsum1 * (long) tmp_stoich) / (long) gl_afrtgt1;
        *lsum2 = (*lsum2 * (long) tmp_stoich) / (long) gl_afrtgt2; /* NB. gl_afrtgt2 is always the same as 1 (2012-10-29) */
    }
}

/**************************************************************************
 **
 ** Calc. of Flex Fuel Sensor %alcohol and PW,spk correction (fuelcor,ffspkdel)
 **
 **************************************************************************/
void flex_fuel_calcs(long *lsum1, long *lsum2)
{
    unsigned char tmp_opt;
    int t;
    unsigned int FSensFreq;

    if ((ram4.FlexFuel & 0x01) && (FSens_Pd > 0)) {
        FSensFreq = 20000 / FSens_Pd;    // Hz, (FSens_Pd in .05 tics)

        if ((FSensFreq >= ram4.fuelFreq[0]) && (FSensFreq <= ram4.fuelFreq[1])) {
            unsigned int freq_delta, freq_max_min;
            freq_delta = FSensFreq - ram4.fuelFreq[0];
            freq_max_min = ram4.fuelFreq[1] - ram4.fuelFreq[0];

            outpc.fuelcor = ram4.fuelCorr[0] + ((freq_delta * (ram4.fuelCorr[1] - ram4.fuelCorr[0])) / freq_max_min);      // %
            ffspkdel = ram4.ffSpkDel[0] + (((long)freq_delta * (ram4.ffSpkDel[1] - ram4.ffSpkDel[0])) / freq_max_min);    // degx10
            outpc.fuel_pct = ram5.fuel_pct[0] + (((unsigned long) freq_delta * (ram5.fuel_pct[1] - ram5.fuel_pct[0])) / freq_max_min);      // %
            stat_flex &= ~1; // clear current bit
        } else {                // sensor reading bad - use default
            if ((outpc.seconds > 3) && (ram5.cel_opt3 & 0x01)) { // give it a chance to start. Only report if CEL is on.
                stat_flex |= 1;
            }
            outpc.fuel_pct = 10; // fault condition. 1% is never going to happen. Safe default for anyone used blended spark tables.
            outpc.fuelcor = ram5.fuelCorr_default;
            ffspkdel = ram5.fuelSpkDel_default;

            /* old behaviour was
            outpc.fuelcor = 100;        // %
            ffspkdel = 0;       // degx10
            outpc.fuel_pct = 0;
            */
        }
        *lsum1 = (*lsum1 * outpc.fuelcor) / 100;        // usec
        *lsum2 = (*lsum2 * outpc.fuelcor) / 100;        // usec
    } else {                    // no flex fuel
        outpc.fuelcor = 100;    // %
        ffspkdel = 0;           // degx10
        outpc.fuel_pct = 0;
    }

    /* fuel temperature */
    tmp_opt = (ram5.fueltemp1 & 0x1f);
    if (tmp_opt == 1) {
        /* bound limits */
        if (ff_pw <= ram5.ff_tpw0) {
            t = ram5.ff_temp0;
        } else if (ff_pw >= ram5.ff_tpw1) {
            t = ram5.ff_temp1;
        } else {
            t = ram5.ff_temp0 + (int)(((ram5.ff_temp1 - ram5.ff_temp0) * (long)(ff_pw - ram5.ff_tpw0)) / (ram5.ff_tpw1 - ram5.ff_tpw0));
        }
        outpc.fuel_temp = t;
    } else if (tmp_opt > 15) {
        outpc.fuel_temp = outpc.sensors[tmp_opt - 16];
    } else {
        outpc.fuel_temp = 0;
    }
}

/**************************************************************************
 **
 ** Over Run Fuel Cut
 **
 **************************************************************************/
void do_overrun(void)
{
    flagbyte17 &= ~FLAGBYTE17_OVERRUNFC; // only checked in mainloop code
    if (ram4.feature5_0 & 0x1) {
        if ((outpc.rpm > ram4.fc_rpm) && (outpc.map < ram4.fc_kpa)
            && (outpc.tps < ram4.fc_tps) && (outpc.clt > ram4.fc_clt)) {
            if (fc_counter == 0) {
                flagbyte17 |= FLAGBYTE17_OVERRUNFC;
                fc_off_time = 0xff;
            } else if (fc_counter > ram4.fc_delay) {
                fc_counter = ram4.fc_delay;
            }
        } else if ((outpc.rpm < ram4.fc_rpm_lower) || (outpc.map >= ram4.fc_kpa) ||
                   (outpc.tps >= ram4.fc_tps) || (outpc.clt <= ram4.fc_clt)) {
            fc_counter = 0xff;
            if (fc_off_time == 0xff) {
                fc_off_time = 0;
            }
        } else {
            /* if we're here, we're in hysteresis, so if the counter is < 0
             * and none of the other conditions are above the threshold
             * yet, continue cutting fuel
             */
            if (fc_counter == 0) {
                flagbyte17 |= FLAGBYTE17_OVERRUNFC;
                fc_off_time = 0xff;
            }
        }
    } else {
        fc_counter = 0xff;
        fc_off_time = 0xff;
    }
}

/**************************************************************************
 ** Additional fuel from nitrous/launch. Zero if unused.
 **************************************************************************/
void n2o_launch_additional_fuel(void)
{
    long tmp_addfuel;
/*
    if (ram4.dual_tble_optn) { <-- unused option
        if (ram4.N2Oopt & 1) {
            tmp_pw1 += (unsigned long) outpc.n2o_addfuel * 100;
            goto DONE_ADD_N2O;
       } else if (ram4.N2Oopt & 2) {
            tmp_pw2 += (unsigned long) outpc.n2o_addfuel * 100;
            goto DONE_ADD_N2O;
        }
    }
*/
    tmp_pw1 += (unsigned long) outpc.n2o_addfuel * 100;
    tmp_pw2 += (unsigned long) outpc.n2o_addfuel * 100;

//  DONE_ADD_N2O:

    if (outpc.status3 & STATUS3_LAUNCHON) {
        tmp_addfuel = (long) ram4.launch_addfuel * 100L;
        if (ram4.launch_opt & 0x10) {
            tmp_pw1 += tmp_addfuel;
        }
        if (ram4.launch_opt & 0x20) {
            tmp_pw2 += tmp_addfuel;
        }
    }

    if (flagbyte9 & FLAGBYTE9_EGTADD) {
        tmp_addfuel = (long) ram4.egt_addfuel * 100L;
        if (ram4.egt_conf & 0x04) {
            tmp_pw1 += tmp_addfuel;
        }
        if (ram4.egt_conf & 0x08) {
            tmp_pw2 += tmp_addfuel;
        }
    }

//tc
    tmp_addfuel = (long) tc_addfuel * 100L;
    if (ram5.tc_opt & 0x40) {
        tmp_pw1 += tmp_addfuel;
    }
    if (ram5.tc_opt & 0x80) {
        tmp_pw2 += tmp_addfuel;
    }

}

void injpwms(void)
{
    if (flagbyte1 & flagbyte1_tstmode) { /* in testmode */
        if (ram5.testcoil & 0x80) { /* custom PWM params, otherwise normal calcs */
            InjPWMDty1 = ram5.testInjPWMDty;
            InjPWMPd1 = ram5.testInjPWMPd;
            InjPWMTim1 = ram5.testInjPWMTim;

            InjPWMDty2 = InjPWMDty1;
            InjPWMPd2 = InjPWMPd1;
            InjPWMTim2 = InjPWMTim1;

            goto INJPWM_COM;
        }
    }

    /* Calc injector PWM parameters */
    if (pin_dualfuel && (ram5.dualfuel_sw2 & 0x04) && (!(*port_dualfuel & pin_dualfuel))) { // dual fuel switch
        InjPWMDty1 = ram5.Inj2PWMDty1;
        InjPWMPd1 = ram5.Inj2PWMPd1;
        if (ram5.opentime2_opt[8] & 0x10) { // PWM enabled or not
            InjPWMTim1 = ram5.Inj2PWMTim1;
        } else {
            InjPWMTim1 = 255;
        }
        if (ram5.opentime2_opt[9] & 0x20) { // own bank 2 settings
            InjPWMDty2 = ram5.Inj2PWMDty2;
            InjPWMPd2 = ram5.Inj2PWMPd2;
            if (ram5.opentime2_opt[9] & 0x10) { // PWM enabled or not
                InjPWMTim2 = ram5.Inj2PWMTim2;
            } else {
                InjPWMTim2 = 255;
            }
        } else {
            InjPWMDty2 = InjPWMDty1;
            InjPWMPd2 = InjPWMPd1;
            InjPWMTim2 = InjPWMTim1;
        }
    } else { // normal
        InjPWMDty1 = ram4.InjPWMDty;
        InjPWMPd1 = ram4.InjPWMPd;
        if (ram4.opentime_opt[8] & 0x10) { // PWM enabled or not
            InjPWMTim1 = ram4.InjPWMTim;
        } else {
            InjPWMTim1 = 255;
        }
        if (ram4.opentime_opt[9] & 0x20) { // own bank 2 settings
            InjPWMDty2 = ram4.InjPWMDty2;
            InjPWMPd2 = ram4.InjPWMPd2;
            if (ram4.opentime_opt[9] & 0x10) { // PWM enabled or not
                InjPWMTim2 = ram4.InjPWMTim2;
            } else {
                InjPWMTim2 = 255;
            }
        } else { // bank2 = bank1
            InjPWMDty2 = InjPWMDty1;
            InjPWMPd2 = InjPWMPd1;
            InjPWMTim2 = InjPWMTim1;
        }
    }
    
    INJPWM_COM:;

    {
        unsigned int t1;
        t1 = (unsigned int)InjPWMPd1 * InjPWMDty1;
        t1 /= 100; /* <- check this out for lousy assembly output */
        pwmd1 = t1;

        pwmd2 = ((unsigned int)InjPWMPd2 * InjPWMDty2) / 100;
    }
}

void do_final_fuelcalcs(void)
{
    unsigned int uitmp1, uitmp2;
    unsigned long tmp_pw_sf, ultmp;

    run_EAE_calcs();
    run_xtau_calcs();
    injpwms();

/* Anti-lag pw modification */
    if (als_state == 1) {
        tmp_pw1 = (tmp_pw1 * (1000 + als_addfuel)) / 1000;
        tmp_pw2 = (tmp_pw2 * (1000 + als_addfuel)) / 1000;
    }
/* end anti-lag */

// limit PW to 65ms (or 65x4 ms)
    if (tmp_pw1 > 6500000) {
        tmp_pw1 = 6500000;
    }

    if (tmp_pw2 > 6500000) {
        tmp_pw2 = 6500000;
    }

    if (ram4.EAEOption == 2) {
        int source;

        /* bank 1 */
        ultmp = tmp_pw1 / EAEdivider; // EAE divider is always divider * 100

        if (ultmp) {
            ultmp += calc_opentime(8);
        }

        if (ultmp > 65000) {
            ultmp = 65000;
        }

        DISABLE_INTERRUPTS;
        pwcalc_eae1 = (unsigned int)ultmp;
        ENABLE_INTERRUPTS;

        /* bank 2 */
        ultmp = tmp_pw2 / EAEdivider;

        if (ultmp) {
            if (ram4.opentime_opt[9] & 0x20) {
                ultmp += calc_opentime(9);
            } else {
                ultmp += calc_opentime(8);      // bank1 settings
            }
        }

        if (ultmp > 65000) {
            ultmp = 65000;
        }

        DISABLE_INTERRUPTS;
        pwcalc_eae2 = (unsigned int)ultmp;
        ENABLE_INTERRUPTS;

        /* calcs */
        if (ram4.EAElagsource) {
            /* use mapdot, otherwise, tpsdot */
            source = outpc.mapdot;
        } else {
            source = outpc.tpsdot / 10;
        }

        if ((source > ram4.EAElagthresh) &&
            ((outpc.EAEfcor1 > 100) || (outpc.EAEfcor2 > 100)) &&
            (outpc.rpm < ram4.EAElagRPMmax)) {
            DISABLE_INTERRUPTS;
            flagbyte2 |= flagbyte2_EAElag;
            ENABLE_INTERRUPTS;
        } else {
            DISABLE_INTERRUPTS;
            flagbyte2 &= ~flagbyte2_EAElag;
            ENABLE_INTERRUPTS;
        }

        DISABLE_INTERRUPTS;
        injtime_EAElagcomp = injtime / divider;
        ENABLE_INTERRUPTS;
    }

    // If only using V3 inj, this might not be ideal, but result is stored to pwcalc1
    // would be better if using pwseq[9]
    if (!(ram4.hardware & HARDWARE_MS3XFUEL)) { 
        if ((ram4.EgoOption & 0x03) && (ram4.egonum > 1)) {
            // EGO correction on and more than one sensor, do calc in loop
            tmp_pw_sf = (tmp_pw1 * outpc.egocor[ram5.egomap[8] & 0x07]) / 1000;
        } else {
            tmp_pw_sf = tmp_pw1;
        }
    } else {
        tmp_pw_sf = tmp_pw1;
    }

    if ((ram4.sequential & SEQ_TRIM) && (!(ram4.hardware & HARDWARE_MS3XFUEL))) { // V3 inj trim - should this be within above (if) ?
        ultmp = (tmp_pw_sf * (500UL + fuel_trim[8])) / 500UL;
    } else {
        ultmp = tmp_pw_sf;
    }

    /* rail pre-OT PW to 65ms */
    if (ultmp > 6500000) {
        uitmp2 = 65000;
    } else {
        uitmp2 = ultmp / 100;
    }

    uitmp2 = smallpw(uitmp2, 8);

    rawpw[16] = uitmp2;
    uitmp1 = calc_opentime(8);

    /* See if we can add any OT */
    if (uitmp1 > ((unsigned int)65000 - uitmp2)) {
        uitmp2 = 65000;
    } else if (uitmp2) {
        uitmp2 += uitmp1;
    }

    pwcalc1 = uitmp2;
    // let ign_in set outpc.pwseq[9]

    // If only using V3 inj, this might not be ideal, but result is stored to pwcalc1
    // would be better if using pwseq[9]
    if (!(ram4.hardware & HARDWARE_MS3XFUEL)) { 
        if ((ram4.EgoOption & 0x03) && (ram4.egonum > 1)) {
            // EGO correction on and more than one sensor, do calc in loop
            tmp_pw_sf = (tmp_pw2 * outpc.egocor[ram5.egomap[9] & 0x07]) / 1000;
        } else {
            tmp_pw_sf = tmp_pw2;
        }
    } else {
        tmp_pw_sf = tmp_pw2;
    }

    if ((ram4.sequential & SEQ_TRIM) && (!(ram4.hardware & HARDWARE_MS3XFUEL))) { // V3 inj trim
        ultmp = (tmp_pw_sf * (500UL + fuel_trim[9])) / 500UL;
    } else {
        ultmp = tmp_pw_sf;
    } 

    /* rail pre-OT PW to 65ms */
    if (ultmp > 6500000) {
        uitmp2 = 65000;
    } else {
        uitmp2 = ultmp / 100;
    }

    uitmp2 = smallpw(uitmp2, 9);
    rawpw[17] = uitmp2;

    if (ram4.opentime_opt[9] & 0x20) {
        uitmp1 = calc_opentime(9);
    } else {
        uitmp1 = calc_opentime(8);  // bank1 settings
    }

    /* See if we can add any OT */
    if (uitmp1 > ((unsigned int)65000 - uitmp2)) {
        uitmp2 = 65000;
    } else if (uitmp2) {
        uitmp2 += uitmp1;
    }

    pwcalc2 = uitmp2;
    // let ign_in set outpc.pwseq[10]
}

void do_seqpw_calcs(unsigned long pw, int start, int end)
{
    unsigned long ultmp, tmp_pw_sf = 0;
    int fire, i, ix;
    unsigned int uitmp1, uitmp2;

    if (!((ram4.EgoOption & 0x03) && (ram4.egonum > 1)) || (start >= 8)) {
        /* if nil or single EGO sensor then do calc out here and save doing
         * it every loop iteration
         */
        tmp_pw_sf = pw;
    }

    for (ix = start, i = 0; ix < end; ix++, i++) {
        if (pw) {
            if ((ram4.EgoOption & 0x03) && (ram4.egonum > 1) && (ix < NUM_TRIGS)) {
                // EGO correction on and more than one sensor, do calc in loop
                tmp_pw_sf = (pw * outpc.egocor[ram5.egomap[i] & 0x0f]) / 1000;
            }
            if ((ix < NUM_TRIGS) && (ram4.sequential & SEQ_TRIM) && (ram4.hardware & HARDWARE_MS3XFUEL)) {
                fire = ram4.fire[i];
                if ((fire > 0) && (fire <= NUM_TRIGS)) {
                    fire--;

                    /* Previous code was:
                     * ultmp = (tmp_pw_sf * (1000 + (fuel_trim[fire] << 1))) / 1000L;
                     * But that can overflow unsigned long. This one just fits. */
                    ultmp = (tmp_pw_sf * (500UL + fuel_trim[fire])) / 500UL;

                    /* rail pre-OT PW to 65ms */
                    if (ultmp > 6500000) {
                        uitmp2 = 65000;
                    } else {
                        uitmp2 = ultmp / 100;
                    }

                    uitmp2 = smallpw(uitmp2, fire);
                    rawpw[ix] = uitmp2;

                    uitmp1 = calc_opentime(fire);

                    /* See if we can add any OT */
                    if (uitmp1 > ((unsigned int)65000 - uitmp2)) {
                        uitmp2 = 65000;
                    } else if (uitmp2) {
                        uitmp2 += uitmp1;
                    }

                    seq_pw[ix] = uitmp2;
                    outpc.pwseq[fire+start] = uitmp2;
                }
            } else {

                if ((ram4.sequential & SEQ_TRIM) && (!(ram4.hardware & HARDWARE_MS3XFUEL)) && (ix >= 8)) {
                    ultmp = (tmp_pw_sf * (500UL + fuel_trim[ix])) / 500UL;
                } else {
                    ultmp = tmp_pw_sf;
                }

                /* rail pre-OT PW to 65ms */
                if (ultmp > 6500000) {
                    uitmp2 = 65000;
                } else {
                    uitmp2 = ultmp / 100;
                }

                uitmp2 = smallpw(uitmp2, ix);
                rawpw[ix] = uitmp2;

                uitmp1 = calc_opentime(ix);

                /* See if we can add any OT */
                if (uitmp1 > ((unsigned int)65000 - uitmp2)) {
                    uitmp2 = 65000;
                } else if (uitmp2) {
                    uitmp2 += uitmp1;
                }

                seq_pw[ix] = uitmp2;
                outpc.pwseq[ix] = uitmp2;
            }
        } else {
            outpc.pwseq[ix] = 0;
            seq_pw[ix] = 0;
        }
    }
}

void do_sequential_fuel(void)
{
    inj_event *events_to_fill = NULL;
    unsigned char last_tooth_no;
    ign_time last_tooth_time;
    int last_tooth_ang, fuel_ang, inttmp;
    long dsf_lsum;

    if (outpc.rpm) {
        if (ram4.hardware & HARDWARE_MS3XFUEL) {
            do_seqpw_calcs(tmp_pw1, 0, num_inj);

            if (do_dualouts) {
                unsigned char start, end;

                if (((num_inj > 4) && (ram4.sequential & SEQ_FULL)) ||
                        ((num_inj > 4) && !(ram4.sequential & 0x3)) ||
                        (ram4.staged_extended_opts & STAGED_EXTENDED_USE_V3)) {

                    start = 8;
                    end = 10;
                } else {
                    if ((cycle_deg == 3600)
                     && (!(ram4.EngStroke & 0x01)) && (num_cyl < 5)) {
                        start = num_inj >> 1; // like wasted-spark with injectors
                    } else {
                        start = num_inj;
                    }
                    end = start << 1;
                }

                do_seqpw_calcs(tmp_pw2, start, end);
            }
        } else {
            if (do_dualouts) {
                do_seqpw_calcs(tmp_pw1, 8, 9);
                do_seqpw_calcs(tmp_pw2, 9, 10);
            } else {
                do_seqpw_calcs(tmp_pw1, 8, 10);
            }
        }

        /* also do timing lookup */

        if (ram4.sequential & 0x3) {

            dsf_lsum =
                intrp_2ditable(outpc.rpm, outpc.fuelload, 12, 12,
                        &ram_window.pg9.inj_timing_rpm[0],
                        &ram_window.pg9.inj_timing_load[0],
                        (unsigned int *) &ram_window.
                        pg9.inj_timing[0][0], 9);

            dsf_lsum += vvt_inj_timing_adj;
            outpc.inj_timing_pri = dsf_lsum;

            DISABLE_INTERRUPTS;
            last_tooth_time = tooth_diff_rpm;
            last_tooth_no = tooth_no_rpm;
            ENABLE_INTERRUPTS;

            if (inj_events == inj_events_a) {
                events_to_fill = inj_events_b;
            } else {
                events_to_fill = inj_events_a;
            }

            if (last_tooth_no == 0) {
                goto END_SEQUENTIAL;
            }
            /* figure out angle between this tooth and previous one
             * array holds angle _ahead_ of the tooth, so step back a tooth
             */
            last_tooth_no--;
            if ((char) last_tooth_no <= 0) {
                last_tooth_no = last_tooth;
            }
            /* make sure we don't land on a zero one */
            while ((last_tooth_ang = deg_per_tooth[last_tooth_no - 1]) == 0) {
                last_tooth_no--;
                if ((char) last_tooth_no <= 0) { /* limits us to 127 teeth */
                    last_tooth_no = last_tooth;
                }
            }

            if (ram4.sequential & SEQ_BEGIN) {
                fuel_ang = (int) dsf_lsum;
            } else {
                unsigned long seql;
                if (ram4.sequential & SEQ_MIDDLE) {
                    seql = pwcalc1 / 2; /* FIXME review for smallpw */
                } else {
                    seql = pwcalc1;
                }

                if (ram4.feature3 & 0x10) {
                    seql *= 4; // PW really 4x length
                }

                fuel_ang =
                    (int) ((seql * last_tooth_ang) / last_tooth_time.time_32_bits);
                fuel_ang = dsf_lsum + fuel_ang;
            }

            /* bound positive */
            while (fuel_ang > 3600) {
                fuel_ang -= cycle_deg;
            }

            /* bound negative */
            while (fuel_ang < -3600) {
                fuel_ang += cycle_deg;
            }

            /* Add calcs for trim and angle here, for now hardcoded angle */
            wheel_fill_inj_event_array(events_to_fill, fuel_ang,
                    last_tooth_time, last_tooth_ang,
                    num_inj_events, num_inj_outs, 0);

            if (do_dualouts) {

                dsf_lsum = intrp_2ditable(outpc.rpm, outpc.fuelload, 12, 12,
                                       &ram_window.pg21.inj_timing_sec_rpm[0],
                                       &ram_window.pg21.inj_timing_sec_load[0],
                                       (unsigned int *) &ram_window.
                                       pg21.inj_timing_sec[0][0], 21);

                dsf_lsum += vvt_inj_timing_adj;
                outpc.inj_timing_sec = dsf_lsum;

                if (ram4.sequential & SEQ_BEGIN) {
                    fuel_ang = (int) dsf_lsum;
                } else {
                    if (ram4.sequential & SEQ_MIDDLE) {
                        inttmp = pwcalc2 / 2;
                    } else {
                        inttmp = pwcalc2;
                    }

                    fuel_ang = (int) (((long) inttmp * last_tooth_ang) /
                            (long) last_tooth_time.time_32_bits);
                    fuel_ang = dsf_lsum + fuel_ang;
                }

                while (fuel_ang > 3600) {
                    fuel_ang -= cycle_deg;
                }

                while (fuel_ang < -3600) {
                    fuel_ang += cycle_deg;
                }

                wheel_fill_inj_event_array(events_to_fill, fuel_ang,
                        last_tooth_time, last_tooth_ang,
                        num_inj_events, num_inj_outs, INJ_FILL_STAGED);
            }

            if (events_to_fill == inj_events_a) {
                DISABLE_INTERRUPTS;
                inj_events = inj_events_a;
                ENABLE_INTERRUPTS;
            } else {
                DISABLE_INTERRUPTS;
                inj_events = inj_events_b;
                ENABLE_INTERRUPTS;
            }
        }
    }
END_SEQUENTIAL:;
}

void calc_fuel_trims(void)
{
    char trimtmp;

    // update one trim on each pass
    if (fuel_trim_cnt >= NUM_TRIGS) { // range check first
        fuel_trim_cnt = 0;
    }

    trimtmp = (char) intrp_2dcstable(outpc.rpm, outpc.fuelload, 6, 6,
            &ram_window.pg9.inj_trim_rpm[0],
            &ram_window.pg9.inj_trim_load[0],
            &ram_window.pg9.inj_trim[fuel_trim_cnt][0][0],
            9);
    fuel_trim[fuel_trim_cnt] = trimtmp;
    // the array is per table NOT adjusted for firing order
    fuel_trim_cnt++;
}

/*
 *    Calculates num_cyl and divider. At init and on the fly.
 */
void calc_divider(void)
{
    unsigned char tmp_divider, tmp_num_cyl;

    // initialise num_cyl variable.
/*
    *** this seems to do more harm than good ***
    if (ram4.EngStroke & 1) {
        tmp_num_cyl = (ram4.no_cyl & 0x1f) << 1;     // two stroke so double it
    } else */ {
        tmp_num_cyl = (ram4.no_cyl & 0x1f);
    }

    tmp_divider = ram4.Divider; // same to start with

    // handle odd configurations
    if ((ram4.no_cyl & 0x1f) == 1) {
        if (ram4.ICIgnOption & 8) {
            conf_err = 33; // No such thing as oddfire 1 cyl? 
        }
        if ((ram4.EngStroke & 1) == 0) { // 4 stroke
            if ((ram4.spk_mode3 & 0xc0) == 0x40) { // wasted
                tmp_num_cyl = 2; // would have been 1 // tach every 360deg
                tmp_divider = ram4.Divider << 1;
            // probably in error to be anything else
            }
        }
    } else if ((ram4.no_cyl & 0x1f) == 2) {
/* Does this make any sense?
        if (((ram4.EngStroke & 0x03) == 1) && ((ram4.ICIgnOption & 8) == 0) && ((ram4.spk_mode3 & 0xc0) == 0x00)) {
            // 2 cyl, 2 stroke, even fire and both cyl spark at the same time
            tmp_num_cyl = 2; // would have been 4 // tach every 360deg
            if ((ram4.Divider & 1) == 0) {
                tmp_divider = ram4.Divider >> 1;
            } else {
                conf_err = 72; // need more squirts, divider can't be halved
            }
        } else */
if ( ((ram4.EngStroke & 1) == 0) && (ram4.ICIgnOption & 8) && ((ram4.spk_mode3 & 0xc0) == 0x40) ) {
            // 2 cyl, 4 stroke, odd fire, wasted spark (twice as often)
            tmp_num_cyl = 4;
        }

    } else if ((ram4.no_cyl & 0x1f) == 3) {
        if (ram4.ICIgnOption & 8) {
            conf_err = 33; // Unsure how to support oddfire 3 cyl 
        }
        if ((ram4.spk_mode3 & 0xc0) == 0x40) { // wasted
            if (spkmode != 4) {
                conf_err = 71;
            } else {
                tmp_num_cyl = 6; /* this causes a problem with fuel */
                tmp_divider = ram4.Divider << 1;
            }
        }
    } else if ((ram4.no_cyl & 0x1f) == 5) {
        if (ram4.ICIgnOption & 8) {
            conf_err = 33; // Unsure how to support oddfire 5 cyl
        }
        if ((ram4.spk_mode3 & 0xc0) == 0x40) { // wasted
            if (spkmode != 4) {
                conf_err = 71;
            } else {
                tmp_num_cyl = 10;
                tmp_divider = ram4.Divider << 1;
            }
        }
    }
    num_cyl = tmp_num_cyl;
    divider = tmp_divider;
}
