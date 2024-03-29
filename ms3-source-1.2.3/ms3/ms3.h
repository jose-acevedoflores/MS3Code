/* $Id: ms3.h,v 1.455.2.4 2013/01/31 15:05:08 jsmcortina Exp $
 * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
 *
 * This file is a part of Megasquirt-3.
 *
 *
    Origin: Al Grippo
    Major: James Murray / Kenneth Culver
    Majority: Al Grippo / James Murray / Kenneth Culver
 *
 * You should have received a copy of the code LICENSE along with this source, please
 * ask on the www.msextra.com forum if you did not.
 *
*/


/* This header should be included by all other C source files in the
 * ms3 project.
 */

#ifndef _MS3_H
#define _MS3_H

#include "hcs12xedef.h"         /* common defines and macros */
#include <string.h>

#include "opt.h"

#include "SD.h"
#include "ms3_defines.h" /* generated from #define list in ms3_vars.h */

extern void _start(void);       /* Startup routine */

/* Explanation of page names.
 * To keep us guessing the XEP uses global mapping as well as PPAGE mapping,
 * these aren't the same of course...
 * The paged areas can still appear in the 0x8000-0xBFFF window using PPAGE
 *
 * To be doubly confusing, gcc has(?) to use its own address mapping which must be
 * observed to get the correct PPAGE in CALL instructions - these are in memory.x
 * These gcc addresses are then converted to global addresses at the end of Makefile
 * and saved to the s19. The downloader and monitor use global addresses.
 *
 * section, PPAGE, gcc address, global addr, HIWAVE
 *   textf2   F2,    0x3d8000     7c_8000    F28000'l
 *   textf3   F3,    0x3dc000     7c_c000    F38000'l
 *   textf4   F4,    0x3e0000     7d_0000    F48000'l
 *   textf5   F5,    0x3e4000     7d_4000    F58000'l
 *   textf6   F6,    0x3e8000     7d_8000    F68000'l
 *   textf7   F7,    0x3ec000     7d_c000    F78000'l
 *   textf8   F8,    0x3f0000     7e_0000    F88000'l
 *   textf9   F9,    0x3f4000     7e_4000    F98000'l
 *   textfa   FA,    0x3f8000     7e_8000    FA8000'l
 *   textfb   FB,    0x3fc000     7e_c000    FB8000'l
 *   textfc   FC,    0x400000     7f_0000    FC8000'l
 *   text3    FD,    0x004000,    7f_4000      4000'l   non-banked
 *   textfe   FE,    0x408000     7f_8000    FE8000'l
 *   text     FF,    0x00c000,    7f_c000      C000'l   non-banked
 *
 *   cnfdata  --,    0x100000,    10_0000    100000'g   D-flash copy of tuning data
 *   cnfdata2 --,    0x103000,    10_3000    103000'g   D-flash copy of tuning data
 *   lookup   --,    0x104000,    10_4000    104000'g   D-flash copy of sensor data
 *
 * RPAGES - 64k
 * Pages F0-FF available for S12X globally these are 0xf_0000-0xf_ffff
 * Pages F8-FF available to XGATE globally these are 0xf_8000-0xf_ffff appears as 0x8000-0xffff in XGATE memory space.
 2k of registers appear at the same place S12X and XGATE
 * For S12X, all pages can be accessed through the window at 0x1000-0x1fff by setting RPAGE
 * Page usage: Global addresses 0x0f_
 * RPAGE - global - usage
 * F0 - 0x0000 - sd card logging (first 1k) : (1k ram25) : RTvars, trim (1k) : tooth/trigger logging (last 1k) @
 * F1 - 0x1000 - free @ These pages F0-F7 are all used by compressed SD readback
 * F2 - 0x2000 - free @
 * F3 - 0x3000 - free @
 * F4 - 0x4000 - free @
 * F5 - 0x5000 - free @
 * F6 - 0x6000 - free @
 * F7 - 0x7000 - free @
 * F8 - 0x8000 - XGATE programme space. All next pages available simultaneously in XGATE memory map.
 * F9 - 0x9000 - XGATE cont'd
 * FA - 0xa000 - ram12,ram13,ram18,ram19 / @
 * FB - 0xb000 - ram21,ram22,ram23,ram24 
 * FC - 0xc000 - ram8,ram9,ram10,ram11
 * FD - 0xd000 - vars1 + free
 * FE - 0xe000 - non-banked RAM. First 2k are ram4,ram5. Upper 2k variable space. (0x2000-0x2fff)
 * FF - 0xf000 - non-banked RAM. Variable space, stack at top. (0x3000-0x3fff)
 *
 * @ are all used when reading back compressed data from SD card.
 *
 * PPAGES - 1024k
 * There's a ton of these. All pages can be accessed through the window at 0x8000-0xbfff by setting PPAGE
 * Global addresses 0x70_0000-0x7f_ffff
 * C0-F4 - free
 * F5 - progs
 * F6 - progs
 * F7 - progs
 * F8 - XGATE can access the top 14k of this page, the bottom 2k are covered by the registers. 
 * F9 - progs
 * FA - progs
 * FB - progs
 * FC - progs
 * FD - non-banked. 0x4000-0x7fff. "text3".
 * FE - progs
 * FF - non-banked. 0xc000-0xffff. "text". 0xf000-0xffff is the monitor
 *
 * EPAGES -32k
 * Global addresses 0x10_0000-0x10_7fff
 * EEPROM window at 0x0800-0x0bff. Non-banked at 0x0c00-0x0fff
 * Not accessible by XGATE
 * 00 - flash8
 * 01 - flash9
 * 02 - flash10
 * 03 - flash11
 * 04 - flash12
 * 05 - flash13
 * 06 - flash18
 * 07 - flash19
 * 08 - flash21
 * 09 - flash22
 * 0a - flash23
 * 0b - flash24
 * 0c - flash4
 * 0d - flash5
 * 0e - flash25
 * 0f - free
 * 10 - cltfactor
 * 11 - cltfactor cont'd
 * 12 - matfactor
 * 13 - matfactor cont'd
 * 14 - egofactor
 * 15 - maffactor
 * 16 - maffactor cont'd
 * 17 - GM curve
 * 18 - GM curve cont'd
 * 19 - Long Term Trim, 2 copies + 512b spare.
 * 1a-1e free-ish
 * 1f   initial DDR and port values
 */
#define FAR_TEXTf2_ATTR __attribute__ ((far)) __attribute__ ((section (".textf2")))
#define FAR_TEXTf3_ATTR __attribute__ ((far)) __attribute__ ((section (".textf3")))
#define FAR_TEXTf4_ATTR __attribute__ ((far)) __attribute__ ((section (".textf4")))
#define FAR_TEXTf5_ATTR __attribute__ ((far)) __attribute__ ((section (".textf5")))
#define FAR_TEXTf6_ATTR __attribute__ ((far)) __attribute__ ((section (".textf6")))
#define FAR_TEXTf7_ATTR __attribute__ ((far)) __attribute__ ((section (".textf7")))
#define FAR_TEXTf8_ATTR __attribute__ ((far)) __attribute__ ((section (".textf8")))
#define FAR_TEXTf9_ATTR __attribute__ ((far)) __attribute__ ((section (".textf9")))
#define FAR_TEXTfa_ATTR __attribute__ ((far)) __attribute__ ((section (".textfa")))
#define FAR_TEXTfb_ATTR __attribute__ ((far)) __attribute__ ((section (".textfb")))
#define FAR_TEXTfc_ATTR __attribute__ ((far)) __attribute__ ((section (".textfc")))
#define FAR_TEXTfe_ATTR __attribute__ ((far)) __attribute__ ((section (".textfe")))
#define LOOKUP_ATTR __attribute__ ((section (".lookup")))
#define CNFDATA_ATTR __attribute__ ((section (".cnfdata")))
#define CNFDATA2_ATTR __attribute__ ((section (".cnfdata2")))
#define TEXTfc_ATTR __attribute__ ((section (".textfc")))
#define TEXT3_ATTR __attribute__ ((section (".text3")))
#define TEXT_ATTR __attribute__ ((section (".text")))
#define INTERRUPT
#define POST_INTERRUPT __attribute__((interrupt))
#define POST_INTERRUPT_TEXT3 __attribute__ ((section (".text3"))) __attribute__((interrupt))
#define POST_INTERRUPT_TEXT3d __attribute__ ((section (".text3d"))) __attribute__((interrupt))
#define ENABLE_INTERRUPTS __asm__ __volatile__ ("cli");
#define DISABLE_INTERRUPTS __asm__ __volatile__ ("sei");
#define NEAR
#define VECT_ATTR __attribute__ ((section (".vectors")))
#define DATA1_ATTR __attribute__ ((section (".data1")))
#define DATA2_ATTR __attribute__ ((section (".data2")))
#define XGATE_ALIGN_ATTR __attribute__ ((aligned (2)))
/* gcc insists on using indexed addressing for the following even if I don't, so follow suit */
#define SSEM0 __asm__ __volatile__ ("movw #0x0101, 0,x\n" "brclr 1,x, #0x01, -9\n" ::"x" (&XGSEMM) );
#define CSEM0 __asm__ __volatile__ ("movw #0x0100, %0\n" ::"m" (XGSEMM) );
#define CSEM0X __asm__ __volatile__ ("movw #0x0100, 0,x\n" ::"x" (&XGSEMM) );
#define SSEM0SEI __asm__ __volatile__ ("sei\n ""movw #0x0101, 0,x\n" "brclr 1,x, #0x01, -9\n" ::"x" (&XGSEMM) );
#define CSEM0CLI __asm__ __volatile__ ("movw #0x0100, %0\n" "cli\n" ::"m" (XGSEMM) );
#define SSEM1 __asm__ __volatile__ ("movw #0x0202, 0,x\n" "brclr 1,x, #0x02, -9\n" ::"x" (&XGSEMM) );
#define CSEM1 __asm__ __volatile__ ("movw #0x0200, %0\n" ::"m" (XGSEMM) );
#define SSEM2 __asm__ __volatile__ ("movw #0x0404, 0,x\n" "brclr 1,x, #0x04, -9\n" ::"x" (&XGSEMM) );
#define CSEM2 __asm__ __volatile__ ("movw #0x0400, %0\n" ::"m" (XGSEMM) );
#define FIRE_COIL XGSWTM=0x0101;
#define DWELL_COIL XGSWTM=0x0202;
#define FIRE_COIL_ROTARY XGSWTM=0x0404;
#define DWELL_COIL_ROTARY XGSWTM=0x0808;

INTERRUPT void UnimplementedISR(void) POST_INTERRUPT;
INTERRUPT void ISR_Ign_TimerIn(void) POST_INTERRUPT_TEXT3;
INTERRUPT void exec_TimerIn(void) POST_INTERRUPT_TEXT3;
unsigned char ISR_Ign_TimerIn_paged(void) FAR_TEXTf6_ATTR;
unsigned char ISR_Ign_TimerIn_paged2(unsigned char, unsigned char) FAR_TEXTf5_ATTR;
void ISR_Ign_TimerIn_part2(void) FAR_TEXTf5_ATTR;
INTERRUPT void ISR_TC2(void) POST_INTERRUPT;
INTERRUPT void ISR_TC4(void) POST_INTERRUPT;
INTERRUPT void ISR_TC5(void) POST_INTERRUPT;
INTERRUPT void ISR_TC6(void) POST_INTERRUPT;
INTERRUPT void ISR_Inj1_TimerOut(void) POST_INTERRUPT;
INTERRUPT void ISR_Inj2_TimerOut(void) POST_INTERRUPT;
INTERRUPT void ISR_TimerOverflow(void) POST_INTERRUPT;
INTERRUPT void ISR_Timer_Clock(void) POST_INTERRUPT_TEXT3;
INTERRUPT void ISR_SCI0(void) POST_INTERRUPT;
INTERRUPT void ISR_SCI1(void) POST_INTERRUPT;
INTERRUPT void CanTxIsr(void) POST_INTERRUPT;
INTERRUPT void CanRxIsr(void) POST_INTERRUPT;
INTERRUPT void ISR_TIMTimerOverflow(void) POST_INTERRUPT;
INTERRUPT void ISR_SPI(void) POST_INTERRUPT_TEXT3;
INTERRUPT void ISR_pit0(void) POST_INTERRUPT;
INTERRUPT void ISR_pit1(void) POST_INTERRUPT;
INTERRUPT void xgswe_handler(void) POST_INTERRUPT;
INTERRUPT void xg_crank_logger(void) POST_INTERRUPT_TEXT3;
INTERRUPT void xg_cam_logger(void) POST_INTERRUPT_TEXT3;

typedef void (*NEAR tIsrFunc) (void);

int intrp_1dctable(int x, unsigned char n, int *x_table,
                   char sgn, unsigned char *z_table,
                   unsigned char page);
int intrp_1dctableu(unsigned int x, unsigned char n, int *x_table,
                   char sgn, unsigned char *z_table,
                   unsigned char page);
int intrp_1ditable(int x, unsigned char n, int *x_table, char sgn,
                   unsigned int *z_table,
                   unsigned char page);
int intrp_2dctable(unsigned int x, int y, unsigned char nx,
                   unsigned char ny, unsigned int *x_table, int *y_table,
                   unsigned char *z_table, unsigned char hires,
                   unsigned char page);
int intrp_2dcstable(unsigned int x, int y, unsigned char nx,
                   unsigned char ny, unsigned int *x_table, int *y_table,
                   char *z_table, unsigned char page);
int twoptlookup(unsigned int x, unsigned int x0, unsigned int x1, int y0,
                int y1) FAR_TEXTfc_ATTR;

typedef struct {
    unsigned int *addrRam;
    unsigned int addrFlash;
    unsigned int n_bytes;
    unsigned char rpg;          // RPAGE for this page
    unsigned char adhi;         // high byte of address in RPAGE
} tableDescriptor;

// page4 - variables mainly. This will be kept in non-paged RAM
typedef struct _page4_data {
    unsigned char no_cyl, no_skip_pulses,       // skip >= n pulses at startup
        ICIgnOption,            // Bit 0: Input capture: 0 = falling edge, 1 rising
        // bit 3: 0 = evenfire, 1 = oddfire
        // bit 4 = spk out hi/lo 0 = spk low (gnd)
        // bit 5 = inj2 uses own parameters
    max_coil_dur, max_spk_dur, XXDurAcc;  // ms x 15 (10* 3/2)
    int crank_rpm,              // rpm at which cranking is through (~300 - 400 rpm)
        adv_offset,             // all in deg x 10
        //     adv_offset is no deg(x10) between trigger (Input Capture) and TDC.
        //     It may be +btdc (IC1 to TDC) or -atdc (TDC to IC2).
        //
        //    |               |     |
        //    |               |     |
        //   -|---------------|-----|-
        //   IC1             TDC    IC2
    TpsBypassCLTRevlim, RevLimRpm2;     // Optn 1:Retard spk by 0 deg at Rpm1 and increase
    //    to RevLimMaxRtd at Rpm2
    // Optn 2:Cut fuel above Rpm2, restore below Rpm1
    int map0, mapmax, clt0, cltmult, mat0, matmult, tps0, tpsmax, batt0,
        battmax,
        // kPa x 10, deg F or C x 10, adc counts, volts x 10
    ego0, egomult, baro0, baromax, bcor0, bcormult, knock0, knockmax;
    // afr x 10, kpa x 10, volts x 100
    unsigned char CrnkTol, ASTol, PulseTol,     // % tolerance for next input pulse
        // timing during cranking, after start/warmup and normal running
        IdleCtl,                // idle: 0 = none, 1= solenoid, 2= iac stepper motor
        //  - enabled only when moving, 3 = iac motor - always enabled.
        // 4 = Ford pwm, 5 = 3 for 15 min, then = 2.
        IACtstep,               // signatiac stepper motor nominal time between steps (.1 ms units)
        IAC_tinitial_step,      // Moving only initial step delay after valve powerup.
        IACminstep,             // iac stepper motor no. of accel/decel steps - future use
        dwellduty;              // dwell duty%
    int IACStart,               // no. of steps to send at startup to put stepper
        //    motor at reference (wide open) position
        IdleHyst,               // amount clt temp must move before Idle position is changed
        IACcrankpos,            // IAC pos must be open(<) at least this much(steps) in cranking
        IACcrankxt,             // no. seconds from end of cranking for IAC pos to blend into
        // coolant dependent pos.
    IACcoldtmp, IACcoldpos, IACcoldxt;
    int OverBoostKpa2;
    unsigned int fc_rpm_lower;
    unsigned char OverBoostOption;
    unsigned char hardware;     // determines what hardware connections in use MS3X etc.
    int OverBoostKpa;
    int OverBoostHyst;
    unsigned char overboostcutx;
    unsigned char overboostcuty;
    int TpsThresh, MapThresh;
    unsigned char Tpsacold,     // cold (-40F) accel amount in .1 ms units
        AccMult;                // cold (-40F) accel multiply factor (%)
    unsigned char TpsAsync,     // clock duration (in .1 sec tics) for accel enrichment
        TPSDQ;                  // deceleration fuel cut option (%)
    int TPSWOT,                 // TPS value at WOT (for flood clear) in %x10
        TPSOXLimit;             // Max tps value (%x10) where O2 closed loop active
    unsigned char Tps_acc_wght, // weight (0-100) to be given to tpsdot for accel enrichment.
        //  100 - Tps_acc_wght will then be given to mapdot.
        BaroOption,             // 0=no baro, 1=baro is 1st reading of map (before cranking), (baroCorr)
        // 2=independent barometer (=> EgoOption not 2 or 4)
        EgoOption,              // 0 = no ego;1= nb o2;2=2 nb o2;3=single wbo2;4=dual wbo2. NOTE:
        //  BaroOption MUST be < 2 if EgoOption = 2 or 4.
        EgoCountCmp,            // Ign Pulse counts between when EGO corrections are made
        EgoStep,                // % step change for EGO corrections
        oldEgoLimit,            // Upper/Lower rail limit (egocorr inside 100 +/- limit)
        EGOVtarget,              // NBO2 AFR determining rich/ lean
        Temp_Units,             // 0= coolant & mat in deg F; 1= deg C
        egonum;
    signed char rtc_trim;           
    int FastIdle,               // fast idle Off temperature (idle_ctl = 1 only)
        EgoTemp,                // min clt temp where ego active
        RPMOXLimit;             // Min rpm where O2 closed loop is active
    unsigned int ReqFuel;       // fuel pulsewidth (usec) at wide open throttle
    unsigned char Divider,      // divide factor for input tach pulses
        Alternate,              // option to alternate injector banks. bit 0 normal function, bit 1 means /2 during crank
        InjPWMTim,              // Time (.1 ms units) after Inj opening to start pwm
        InjPWMPd,               // Inj PWM period (us) - keep between 10-25 KHz (100-40 us)
        InjPWMDty,              // Inj PWM duty cycle (%)
        EngStroke,              // 0 = 4 stroke
        // 1 = 2 stroke
        InjType,                // 0 = port injection
        // 1 = throttle body
        NoInj;                  // no. of injectors (1-12)
    unsigned int OddFireang;    //  smaller angle bet firings (deg*10)
    unsigned char rpmLF, mapLF, tpsLF,  // Lag filter coefficient for Rpm,Map,Tps. Acts like
        egoLF,                  // averager: xnew = xold + xLF * (xmeas - xold), so xLF=0
        adcLF,               // means value will never change, xLF=100 means no filtering.
        // egoLF is coefficient for ego filter, adcLF is for clt,mat,batt,
        knk_pin_out,
        mafLF,
        xxdual_tble_optn,         // spare
        FuelAlgorithm,          
        IgnAlgorithm,          
        dummyafr,               // Not using this, make it a dummy
        dwelltime;              // time in 0.1ms
    unsigned int trigret_ang;   // angle between trigger and return
    unsigned char RevLimOption, // 0=none, 1=spark retard, 2=fuel cut
        RevLimMaxRtd;           // max amount of spark retard (degx10) (at Rpm2 below)
    unsigned char ego_startdelay;
    unsigned char can_poll2, opt142;
    unsigned char InjPWMTim2,   // Time (.1 ms units) after Inj opening to start pwm
        InjPWMPd2,              // Inj PWM period (us) - keep between 10-25 KHz (100-40 us)
        InjPWMDty2;             // Inj PWM duty cycle (%)
    unsigned char can_ego_id, can_ego_table;
    unsigned int can_ego_offset;
    unsigned int baro_upper, baro_lower, baro_default;
    int RevLimTPSbypassRPM;
    int RevLimNormal1;
    int RevLimNormal2;
    unsigned char hw_latency;
    unsigned char loadopts;     // bit 0: 1 mult for fuel, 0 add for fuel
    // bit 2: 1 multiply "load" into final equation, 2 don't
    // bit 3: Scale VE by "stoich" value
    unsigned long baud;         // baud rate
    int MAPOXLimit;             // Max map value (kPax10) where O2 closed loop active
    unsigned char can_poll_id_rtc,
        mycan_id;               // can_id (address) of this board (< MAX_CANBOARDS). Default 0 for ECU.

    unsigned char mapsample_percent;
    unsigned char can_poll_id_ports, can_poll_id;

    unsigned char TpsAsync2;    // accel tailoff duration (sec x 10)
    int TpsAccel2;              // end pulsewidth of accel enrichment (ms x 10)
    unsigned char egoAlgorithm,       // 0=simple prop error algorithm;
        // 1=same algorithm with variable transport delay;
        // 2=full PID with Smith Pred correction.
    egoKP, egoKI, egoKD;        // PID coefficients in %; egoKP also gain for
    // egoAlgorithm=0, with 100 = no gain; egoKD includes
    // 1/dt factor since fixed time step.
    unsigned int ac_idleup_vss_offpoint, ac_idleup_vss_hyst;
    unsigned char FlexFuel,     // Flex fuel option - modifies pw and spk adv based on % alcohol.
        fuelFreq[2],            // Table of fuel sensor freq(Hz) vs %fuel corr;
    fuelCorr[2], dwellmode;     // dwell mode
    unsigned int pwmidle_shift_lower_rpm;
    int ac_idleup_tps_offpoint, ac_idleup_tps_hyst;
    int fan_idleup_tps_offpoint, fan_idleup_tps_hyst;
    unsigned int fan_idleup_vss_offpoint;
    unsigned char knk_option,   // Bits 0-3: 0=no knock detection;1=operate at table value or 1
        // step below knock; 2=operate at table value or edge of knock.
        // Bits 4-7: 0/1 = knock signal < or > knk_thresh indicates knock occurred.
        knk_maxrtd,             // max total retard when knock occurs, (degx10),
    knk_step1, knk_step2,       // ign retard coarse/fine steps when 1st knock or after stopped,
        // (degx10); step1 large to quickly retard/ stop knock
    knk_trtd, knk_tadv,         // time between knock retard, adv corrections, (secx10);
        // allows short time step to quickly retard, longer to try advancing.
        knk_dtble_adv,          // change in table advance required to restart adv til knock or
        // reach table value (0 knock retard) process, deg x10.
        // This only applies with knk_option = 1.
        knk_ndet,               // number knock detects required for valid detection; pad byte.
        EAEOption;              // Bits 0-1: 1: on or off, 2: Use lag compensation, 3: X-tau for accel/decel
    // 4: X-tau for accel-decel including CLT,  
    unsigned char knk_port;
    unsigned int knk_maxmap,    // no knock retard above this map (kPax10).
    knk_lorpm, knk_hirpm,       // no knock retard below, above these rpms.
        No_Teeth;               // Nominal (include missing) teeth for wheel decoding
    unsigned char No_Miss_Teeth;        // Number of consecutive missing teeth.
    unsigned char pwmidle_shift_open_time;
    unsigned int Miss_ang;      // Position of missing tooth BTDC deg*10
    unsigned char ICISR_tmask,  // Time (msx10) after tach input capture during which further
        // interrupts are inhibited to mask coil ring or VR noise.
        ICISR_pmask;            // % of dtpred after tach input capture during which further
    // interrupts are inhibited to mask coil ring or VR noise.
    // This and prior mask not applicable with wheel decoding.
    unsigned int ae_lorpm, ae_hirpm;    // lorpm is rpm at which normal accel enrichment just starts
    // to scale down, and is reduced to 0 at ae_hirpm. To omit scaling, set _lorpm
    // = _hirpm= very large number.
    int ffSpkDel[2];            // Table of fuel sensor freq(Hz) vs spk corr (degx10);

    unsigned char spk_conf2,    // various see ini
        spk_config,             // config byte
        // bit0 sparkA 0=JS6 (MS2 default) 1= D14 (MS1/Extra default)
        // when num_spk_op > 3 always D14 as JS6 = sparkD
        // bit1 0=360deg 1=720
        // bit3,2 = 0 invalid, 1 = missing crank, 2 = 2nd trig, 3 = both
        // bit5,4 = 0 invalid, 1 rising, 2 falling, 3 rising&falling
        // bit6 = cam or crank speed
        // bit7 = rising&falling mode starts on falling or rising
        spk_mode,               // defines standard/EDIS/toothed wheel
        spk_mode3,            // num/type of coils
        rtbaroport,             // defines AD port used for RT baro 2nd sensor, MS2/GPIO
        //AD ports bottom three bits are AD0-7, upper bits
        // 0 = MS2, 1= GPIO1, 2=GPIO2
        OLDego2port,               // defines AD port used for 2nd EGO sensor MS2/GPIO
        mapport,                // AD port for MAP sensor. Allows on-board sensor to be used for baro and offboard for map.
    knkport_an, revlimcutx, revlimcuty, feature4_0;        // various settings
#define SPK_MODE3_SPKTRIM 0x01
    //bit 0 = simple/advanced false trigger protection when userlevel>191
    //bit 1 = enable middle LED as ignition trigger. Certain modes prevent this.
    unsigned char timing_flags, crank_dwell;
    unsigned char tsw_pin_f;      // fuel/spark hardware table switch pins
    int crank_timing, fixed_timing;
#define TIMING_FIXED 0x1
#define USE_1STDERIV_PREDICTION 0x2
    int tsf_rpm, tsf_kpa, tsf_tps, tss_rpm, tss_kpa, tss_tps;  // table switch
    unsigned char feature5_0;  // on/off feature settings
        //bitval 1 = overrun fuel cut
    unsigned char tsw_pin_s;      // fuel/spark hardware table switch pins
    unsigned char pwmidlecranktaper, knk_step_adv;    // knock advance step size
    unsigned int fc_rpm;
    int fc_kpa, fc_tps, fc_clt; // overrun fuel cut >rpm, <kpa, >tps, >clt
    unsigned char fc_delay;     // > time in 0.1s units
    unsigned char tacho_opt;    // tacho output option
    unsigned char fc_ego_delay;
    unsigned char EAElagsource; // 0 = tpsdot, 1 = MAPdot
    int EAElagthresh;
    unsigned int EAElagRPMmax;
    unsigned char feature3,     // feature settings (presently only ASM vs C ign_out ISR)
        // 0x2 Use MAP lag lookup based on TPSdot
        launch_opt;             //launch/flat shift options
    int launch_sft_lim, launch_sft_deg, launch_hrd_lim, launch_tps;
    unsigned char launchlimopt, // launch limiter options
    launchcutx, launchcuty, launch_opt2;
    int flats_arm,              // flat shift options
    flats_sft, flats_deg, flats_hrd;

    unsigned int staged_pri_size, staged_sec_size;
    unsigned char staged;       /* Bits 0-2: 00 = off, 01 = RPM, 2 = MAP, 3 = TPS, 4 = table, 5 = table 
                                 * Bits 3-5: second param: 00 = off, 01 = rpm, 10 = MAP, 11 = TPS
                                 * Bit 6: transition on
                                 * Bit 7: 2nd param: 0 = OR logic, 1 = AND logic
                                 */
    unsigned char staged_transition_events;
    int staged_param_1, staged_param_2;
    int staged_hyst_1, staged_hyst_2;

// Nitrous System
    unsigned char N2Oopt;       // bit 0-1: DT bank usage
    // bit 2: stage 1 enable
    // bit 3: stage 2 enable
    // bits 4-6: input pin
    unsigned char N2Oopt2;      // bits 0-1 output pins
    unsigned int N2ORpm, N2ORpmMax;
    signed int N2OTps, N2OClt, N2OAngle;
    int N2OPWLo, N2OPWHi;
    unsigned char N2Odel_launch, N2Odel_flat, N2Oholdon;
    unsigned char N2O2delay;
    unsigned int N2O2Rpm, N2O2RpmMax;
    signed int N2O2Angle;
    int N2O2PWLo, N2O2PWHi;

    unsigned char RotarySplitMode;
#define ROTARY_SPLIT_FD_MODE 0x1
#define ROTARY_SPLIT_ALLOW_NEG_SPLIT 0x2
#define ROTARY_SPLIT_RX8_MODE 0x4
    unsigned char dlyct;        // hardware timer delay and noise filtering
    unsigned char dwelltime_trl;        // base dwell time for trailing coils
    unsigned char N2Oopt3; // input pins
    signed int RevLimRtdAng, RevLimNormal3, RevLimNormal2_hyst;
    unsigned char pwmidleset;
    unsigned char trig_init;    //semi seq

    unsigned int pwmidle_ms;
    unsigned char pwmidle_close_delay;
    unsigned char pwmidle_open_duty,
        pwmidle_closed_duty, pwmidle_pid_wait_timer, pwmidle_min_duty;
    unsigned int pwmidle_engage_rpm_adder;
    int pwmidle_tps_thresh;
    unsigned char pwmidle_dp_adder;
    unsigned int pwmidle_rpmdot_threshold;
    unsigned int pwmidle_decelload_threshold;
    unsigned int pwmidle_Kp, pwmidle_Ki, pwmidle_Kd;
    unsigned char pwmidle_freq; // new meaning, see ini
    unsigned char boost_ctl_settings;   // bit 0-2: freq-divider
    // bit 3: on or off
    // bit 4: closed loop?
    // bit 5: invert? 
    // bit 6: use the initial value table?
#define BOOST_CTL_ON            0x8
#define BOOST_CTL_CLOSED_LOOP   0x10
#define BOOST_CTL_INVERT        0x20
#define BOOST_CTL_USE_INITIAL   0x40
    unsigned char boost_ctl_pins;
    unsigned char boost_ctl_Kp, boost_ctl_Ki, boost_ctl_Kd,
        boost_ctl_closeduty, boost_ctl_openduty;
    unsigned int boost_ctl_ms;
    unsigned char boost_ctl_pwm;
    unsigned char NoiseFilterOpts;
    unsigned int pwmidle_min_rpm, pwmidle_max_rpm;
    unsigned char pwmidle_targ_ramptime;
    unsigned char inj_time_maskxx;

    unsigned char tpsen_table[NO_TPS_DOTS],     // accel enrichment pw in 1% ReqFuel units vs tpsdot
        mapen_table[NO_MAP_DOTS];       // accel enrichment pw in 1% ReqFuel units vs mapdot
    signed int tpsdot_table[NO_TPS_DOTS],       // change in % x 10 per .1 sec
        mapdot_table[NO_MAP_DOTS];      // change in kPa x 10 per .1 sec

    unsigned char secondtrigopts;
#define NOISE_FILTER_ONLY 0x1
    unsigned int TC5_required_width;
    int EgoLimit;
    int stoich;
    int MAPOXMin;               // lower map value (kpax10) where 02 closed loop active -- KG 
    unsigned char IC2ISR_tmask;
    unsigned char IC2ISR_pmask;
    unsigned char extra_load_opts;      /* bit 0-2: afrload: 0 = use fuelload, 1 = MAP, 2 = % baro, 3 = TPS, 5 = MAFload */
    /* bit 4-6: eaeload: 0 = use fuelload, 1 = MAP, 2 = % baro, 3 = TPS, 5 = MAFload */
    unsigned char airden_scaling;
    unsigned int fan_idleup_vss_hyst;
    unsigned char log_style_led; // datalog LED output
    unsigned char primedelay;
#define PWMIDLE_CL_USE_INITVALUETABLE 0x1
#define PWMIDLE_CL_INITVAL_CLT 0x2
    unsigned char pwmidle_cl_opts;
    unsigned char pwmidle_dp_decay_factor;
    unsigned char idleveadv_to_pid;
#define IDLEVEADV_TO_PID_IDLEADV 0x1
#define IDLEVEADV_TO_PID_IDLEVE  0x2
    char ram4pad[8];
    unsigned int staged_secondary_enrichment;
    unsigned char staged_primary_delay;
    unsigned char idle_special_ops; 
#define IDLE_SPECIAL_OPS_IDLEADVANCE 0x1
#define IDLE_SPECIAL_OPS_IDLEVE 0x2
#define IDLE_SPECIAL_OPS_IDLEVE_VSSON 0xC
#define IDLE_SPECIAL_OPS_IDLEADV_VSSON 0x30
#define IDLE_SPECIAL_OPS_CLIDLE_TIMING_ASSIST 0x40
    int idleadvance_tps;
    int idleadvance_rpm;
    int idleadvance_load;
    int idleadvance_clt;
    unsigned char idleadvance_delay;
    int idleadvance_curve[4];
    int idleadvance_loads[4];
    unsigned char log_style2_but;
    unsigned char log_style;    // 0 = 64 byte block
    unsigned char log_style2;   // more settings
    unsigned char log_style3;   // even settings
    unsigned char log_length;   // minutes per file
    unsigned int log_int;       // 0.128ms
    unsigned int log_offset[64];        // on board datalogging. Offsets into outpc
    unsigned char log_size[64]; // on board datalogging. Size of value. 0=char, 1=int, 2=long
    unsigned char fire[16];     // firing order array
    unsigned char sequential;
#define SEQ_SEMI                0x1     // 2 channels
#define SEQ_FULL                0x2     // up to 8 channels
#define SEQ_END                 0x4     // angle specifies squirt stop
#define SEQ_MIDDLE              0x8     // angle specifies middle of squirt
#define SEQ_BEGIN               0x10    // angle specifies beginning of squirt
#define SEQ_TRIM                0x20    // enable per-injector trim
    unsigned char boost_launch_duty;
    unsigned int boost_launch_target;
    unsigned char boost_feats;
    unsigned char launch_3step;   // 3step setbits
    unsigned int launch_var_low, launch_var_up, launch_var_sof;
    int launch_sft_lim3, launch_sft_deg3, launch_hrd_lim3;
    int map_sample_duration;
    unsigned char opentime_opt[18];     // which table to use + any other options + space for the max
    unsigned char smallpw_opt[18];      // top bit of first byte used as master on/off
    unsigned char maxafr_opt1, maxafr_opt2; // max AFR options
    unsigned int maxafr_en_load, maxafr_en_rpm, maxafr_en_time,
        maxafr_spkcut_time;
    int maxafr_ret_tps;
    unsigned int maxafr_ret_map, maxafr_ret_rpm;
    signed int launch_addfuel;
    unsigned int wheeldia1, fdratio1, wheeldia2;
    unsigned char vss_pos, launch_var_on;
    unsigned char reluctorteeth1, reluctorteeth2, vss_opt, vss1_an,
        vss1_can_id, tsw_pin_ob;
    unsigned int vss1_can_offset, vss2_can_offset;
    unsigned char MapThreshXTD, MapThreshXTD2;
    unsigned char reluctorteeth3, reluctorteeth4, ss_opt, vss2_an,
        ss1_pwmseq, ss2_pwmseq;
    unsigned int gear_can_offset;
    unsigned char mapsample_opt, map2port;
    unsigned char n2o1n_pins, n2o1f_pins, n2o2n_pins, n2o2f_pins;
    unsigned int vss1_an_max, vss2_an_max;
    unsigned char tsw_pin_rf, tsw_pin_afr, tsw_pin_stoich, boost_vss;
    unsigned int ReqFuel_alt;
    int stoich_alt;
    unsigned char water_pins_pump, water_pins_valve, water_pins_in_shut;
    unsigned char water_freq;
    int boost_vss_tps, water_tps;
    unsigned int water_rpm, water_map, water_mat;
    int pwmidle_rpmdot_disablepid;
    int boost_ctl_lowerlimit;
    // This goes with the CAN polling for a generic external board
    unsigned char enable_poll;  // bit 0: enable ADC
    // bit 1: enable PWM
    // bit 2: enable digital I/O ports
    unsigned char poll_tables[3];  // Remote table numbers for RTC, PWM and ports data
    unsigned int poll_offset[4];   // Offset in the table (RTC, PWM, digi in, digi out)
    unsigned char egt_num;
    unsigned char accXport, accYport, accZport;
    unsigned int accXcal1, accXcal2, accYcal1, accYcal2, accZcal1, accZcal2;
    unsigned char accxyzLF;
    unsigned char egtport[16]; // all in here
    unsigned char egt_conf;
    int egt_temp0, egt_tempmax;
    int egt_warn, egt_max, egt_time;
    unsigned int vss1_can_scale, vss2_can_scale;
    unsigned char vss_pwmseq[2];
    unsigned char MAFOption; 
    unsigned int enginesize;
    unsigned char vssout_opt;
    unsigned int vssout_scale;
    unsigned char vss_can_size, canpwm_clk, canpwm_pre, canpwm_div, vss1_can_table, feature7;
    unsigned char vss1LF, vss2LF, ss1LF, ss2LF;
    unsigned int egt_addfuel;
    unsigned int launch_fcut_rpm;
    unsigned int gear_ratio[6];
    unsigned char gear_method, gear_port_an;
    unsigned int gearv[7];
    unsigned char gear_no, vssdotLF;
    unsigned char vssdot_int;
    unsigned char ac_idleup_settings;
    unsigned char ac_idleup_io;
    unsigned int ac_idleup_delay;
    unsigned char ac_idleup_adder;
    unsigned char fanctl_settings;
    unsigned int fanctl_idleup_delay;
    unsigned char fanctl_idleup_adder;
    int fanctl_ontemp;
    int fanctl_offtemp;
    unsigned char canadc_opt, fanctl_opt2;
    unsigned char canadc_id[6], canadc_tab[6];
    unsigned int canadc_off[6];
    int ac_idleup_cl_targetadder;
    int fan_idleup_cl_targetadder;
    int boost_ctl_clt_threshold;
    unsigned char fan_ctl_settings2;
    unsigned char ego_upper_bound;
    unsigned char ego_lower_bound;
    unsigned char pad1byte1; // for alignment
    unsigned int launch_maxvss;
    unsigned char maf_range;
    unsigned char ac_delay_since_last_on;
    unsigned char XXegomap[4]; // spare
    unsigned char boost_gear_switch;
    unsigned char staged_extended_opts;
#define STAGED_EXTENDED_USE_V3 0x1
#define STAGED_EXTENDED_SIMULT 0x2
#define STAGED_EXTENDED_STAGEPW1OFF 0x4
    unsigned char can_pwmout_id, can_pwmout_tab;
    unsigned int can_pwmout_offset;
    int idleve_tps;
    unsigned int idleve_rpm;
    int idleve_load;
    int idleve_clt;
    unsigned char idleve_delay;
    unsigned char ac_idleup_cl_lockout_mapadder;
} page4_data;

// page5 - variables mainly. This will be kept in non-paged RAM
typedef struct _page5_data {
    unsigned char pwm_testio, duty_testio;
    unsigned char testcoil,     // which coil to fire
        testdwell;              // dwell in 0.1ms
    unsigned int testint;       // interval in 0.128ms units
    unsigned int testpw, testinjcnt;
    unsigned char xtestmode;
    unsigned char testInjPWMTim,        // Time (.1 ms units) after Inj opening to start pwm
        testInjPWMPd,           // Inj PWM period (us) - keep between 10-25 KHz (100-40 us)
        testInjPWMDty;          // Inj PWM duty cycle (%)
    unsigned char testsel;      // which injector / coil to test
    unsigned char dbgtooth;     // debug this tooth
    unsigned int iacpostest;    // test word. Fix iac postion. 0 = off
    unsigned int iachometest;   // test home position
    unsigned char iactest;      // control byte
    unsigned char flashlock;
    unsigned char boost_ctl_settings2;   // bit 0-2: freq-divider
    // bit 3: on or off
    // bit 4: closed loop?
    // bit 5: invert? 
    unsigned char boost_ctl_pins2;
    unsigned char boost_ctl_Kp2, boost_ctl_Ki2, boost_ctl_Kd2,
        boost_ctl_closeduty2, boost_ctl_openduty2, boost_ctl_pwm2;
    int boost_ctl_lowerlimit2;
    unsigned char boost_ctl_sensor2;
    unsigned char pad33[7];
    unsigned char egoport[16];
    unsigned char egomap[16];

    unsigned char tc_opt, tc_slipthresh;       // TC
    unsigned int tc_minvss, tc_maxvss;
    int tc_mintps;
    unsigned char tc_enin;

    unsigned char knock_bpass, knock_conf, knock_int;

    unsigned char ff_tpw0, ff_tpw1;
    int ff_temp0, ff_temp1;
    unsigned char fueltemp1;
    unsigned char tc_knob;

    unsigned int maf_freq0, maf_freq1, map_freq0, map_freq1;

    unsigned char knock_gain[16], knock_sens[16];
    int s16_debug;
    unsigned char u08_debug134;

    unsigned char AE_options;
#define USE_NEW_AE 0x1
    int accel_blend_percent;
    unsigned int accel_tpsdot_threshold;
    unsigned int accel_mapdot_threshold;
    int accel_mapdots[8];
    int accel_tpsdots[8];
    int accel_mapdot_amts[8];
    int accel_tpsdot_amts[8];
    int accel_CLT_multiplier;
    int cl_idle_timing_target_deltas[8];
    int cl_idle_timing_advance_deltas[8];
    unsigned char ltt_opt, ltt_but_in, ltt_int, ltt_led_out;
    unsigned int ltt_thresh;
    int tc_minmap;
    unsigned char can_bcast1, can_bcast2;
    unsigned int can_bcast_int;

    unsigned char timedout1_in, timedout1_out;
    unsigned int timedout1_offdelay;
    unsigned char tstop_out, tstop_pad;
    unsigned int tstop_delay, tstop_hold;
    unsigned int oddfireangs[6];
    unsigned char cel_opt, cel_port;
    unsigned char afr_min, afr_max;
    unsigned int afr_var_upper, afr_var_lower;
    unsigned int map_minadc, map_maxadc, map_var_upper;
    unsigned int mat_minadc, mat_maxadc, mat_var_upper;
    unsigned int clt_minadc, clt_maxadc, clt_var_upper;
    unsigned int tps_minadc, tps_maxadc, tps_var_upper;
    unsigned char batt_minv, batt_maxv;
    unsigned int batt_var_upper;
    unsigned char cel_opt2, cel_action1;
    int cel_clt_cold, cel_clt_warm;
    unsigned int cel_warmtime;
    int cel_mat_default;
    unsigned int cel_revlim, cel_overboost;
    unsigned char cel_boost_duty, cel_boost_duty2, cel_synctol, cel_opt3;
    int cel_retard;
    unsigned int map_var_lower;
    int egt_minvalid, egt_maxvalid;
    unsigned int egt_var_upper, egt_var_lower;
    unsigned char cel_runtime, cel_action2;
    unsigned char blend_opt[7];
/*
 *
 */
    unsigned char pad661[303]; // (not quite so) big pad big gap
/*
 *
 */
    unsigned char fuelCorr_default, pad653;
    unsigned int fuelSpkDel_default;
    unsigned int map_phase_thresh;
    unsigned int fuel_pct[2];
    unsigned char sensor_source[16], sensor_trans[16];
    int sensor_val0[16], sensor_max[16];
    unsigned char sensorLF[16], sensor_temp, pad775;
    unsigned int injOpen[18];
    unsigned char shift_cut_on, shift_cut_in, shift_cut_out;
    int shift_cut_rpm, shift_cut_tps;
    unsigned char shift_cut_delay, shift_cut_time, shift_cut_add[5];
    unsigned char shift_cut_soldelay;
    int shift_cut_rpmauto[5];
    unsigned char shift_cut_reshift;
    unsigned char pwm_opt[6], pwm_opt2[6], pwm_onabove[6], pwm_offbelow[6];
    unsigned char dualfuel_sw, dualfuel_sw2;
    unsigned char opentime2_opt[18];     // which table to use + any other options + space for the max
    unsigned char smallpw2_opt[18];      // top bit of first byte used as master on/off
    unsigned char dualfuel_pin, dualfuel_opt;

    unsigned int inj2Open[18];
    unsigned char Inj2PWMTim1,   // Time (.1 ms units) after Inj opening to start pwm
        Inj2PWMPd1,              // Inj PWM period (us) - keep between 10-25 KHz (100-40 us)
        Inj2PWMDty1;             // Inj PWM duty cycle (%)
    unsigned char Inj2PWMTim2,   // Time (.1 ms units) after Inj opening to start pwm
        Inj2PWMPd2,              // Inj PWM period (us) - keep between 10-25 KHz (100-40 us)
        Inj2PWMDty2;             // Inj PWM duty cycle (%)

    unsigned char dualfuel_temp_sens, dualfuel_press_sens;
    int oldFuelAdj, oldSpkAdj, oldIdleAdj, oldSprAdj; // spares, not used
    unsigned char dualfuel_pri, dualfuel_sec; // neither actually used
    int ITB_load_mappoint;
    int ITB_load_idletpsthresh;
    int idle_voltage_comp_voltage[6];
    int idle_voltage_comp_delta[6];
    unsigned char pad954[40];
} page5_data;

// page8 - tables ONLY - this will be in banked ram
typedef struct _page8_data {
    int boost_ctl_load_targets[8][8];
    int boost_ctl_loadtarg_tps_bins[8];
    unsigned int boost_ctl_loadtarg_rpm_bins[8];
    unsigned char boost_ctl_pwm_targets[8][8];
    int boost_ctl_pwmtarg_tps_bins[8];
    unsigned int boost_ctl_pwmtarg_rpm_bins[8];
    int                         // this is primePWtable, CrankPctTable, asePctTable, aseCntTable
    CWPrime[NO_TEMPS], CrankPctTable[NO_TEMPS], CWAWEV[NO_TEMPS],
        CWAWC[NO_TEMPS];
    int MatTemps[NO_MAT_TEMPS]; // MAT temperatures for spark retard, degx10 F or C 
    unsigned char MatSpkRtd[NO_MAT_TEMPS];      // degx10 of spark retard vs mfld air temp 
    unsigned int EAEAWCRPMbins[NO_FRPMS], EAESOCRPMbins[NO_FRPMS], EAEAWCKPAbins[NO_FMAPS],     /* MAP bins for EAE */
        EAESOCKPAbins[NO_FMAPS];
    unsigned char EAEBAWC[12],  /* Added to the Wall coefficient */
        EAEBSOC[12],            /* sucking off the wall coefficient */
        EAEAWN[12],             /* added to the walls rpm scalar */
        EAESON[12],             /* pulled from walls rpm scalar */
        EAEAWW[12],             /* added to the walls CLT scalar */
        EAESOW[12];             /* pulled from the walls CLT scalar */
    int BaroVals[NO_BARS],      // Barometric pressures (kPa x 10) for baro correction table
        MatVals[NO_MATS];       // air temperatures (degx10) for air density correction table
    // to correct for varying mat sensor location
    int temp_table_p5[NO_TEMPS];

    int EAEAWWCLTbins[12], EAESOWCLTbins[12];
#define PWMIDLE_NUM_BINS 8
    unsigned int pwmidle_target_rpms[PWMIDLE_NUM_BINS];
    int pwmidle_clt_temps[PWMIDLE_NUM_BINS];
    int pad[10];                //
#define REVLIMRPMSIZE 8
    unsigned int RevLimLookup[REVLIMRPMSIZE], RevLimRpm1[REVLIMRPMSIZE];

    unsigned int knk_rpm[NO_KNKRPMS],   // tables of rpm vs knock threshhold(Vx100).
        knk_thresh[NO_KNKRPMS];         // THESE ARE NOT USED -> 24 bytes free
    int RotarySplitTable[8][8];
    unsigned int RotarySplitMAP[8];
    unsigned int RotarySplitRPM[8];
    unsigned int NoiseFilterRpm[4];
    unsigned int NoiseFilterLen[4];
    unsigned char VariableLagTPSBins[4];
    unsigned char VariableLagMapLags[4];
    unsigned int pwmidle_crank_dutyorsteps[4];
    int pwmidle_crank_clt_temps[4];
    unsigned int matclt_pct[6];
    unsigned int matclt_flow[6];
    /* user defined */
    unsigned int user_value1;
    unsigned int user_value2;
    unsigned char user_conf;
// some space for more variables, reduce this pad number as new vars added
// char = 1 byte, int = 2 bytes, long = 4 bytes
    unsigned char paduser[10];
/* end user defined */
    unsigned char pad1;
    int BaroCorDel[NO_BARS],    // Baro correction correction
         AirCorDel[NO_MATS];     // Air density corrections (+/-%) - added to eq. value
    unsigned char pad908[14];
} page8_data;

//page9 - tables ONLY - this will be kept in paged RAM
typedef struct _page9_data {
    signed char inj_trim[16][6][6];     // trim tables for 16 injector channels. 576 bytes
    unsigned int inj_trim_rpm[6];   // shared rpm bins. 12 bytes
    signed int inj_trim_load[6];    // shared load bins 12 bytes
    signed int inj_timing[12][12];
    unsigned int inj_timing_rpm[12];
    signed int inj_timing_load[12];
    unsigned int maxafr1_load[6];
    unsigned int maxafr1_rpm[6];
    unsigned char maxafr1_afr[6][6];
    unsigned char dwellrpm_dwell[6];
    unsigned int dwellrpm_rpm[6];
    unsigned char pad0[10];
} page9_data;

//page10 - tables ONLY - this will be kept in paged RAM
typedef struct _page10_data {
    int boost_ctl_load_targets2[8][8];  // 128
    int boost_ctl_loadtarg_tps_bins2[8];        // 16
    unsigned int boost_ctl_loadtarg_rpm_bins2[8];       // 16 
    unsigned char boost_ctl_pwm_targets2[8][8]; // 64 
    int boost_ctl_pwmtarg_tps_bins2[8]; // 16
    unsigned int boost_ctl_pwmtarg_rpm_bins2[8];        // 16 
    unsigned char boost_timed_pct[6];
    unsigned int boost_timed_time[6];

    unsigned char boostvss_duty[6];
    unsigned int boostvss_target[6];
    unsigned int boostvss_speed[6];

    unsigned int map_sample_rpms[8];
    int map_sample_timing[8];
    unsigned char afr_table[NO_INJ][NO_FMAPS][NO_FRPMS],        //  afr x 10
        warmen_table[NO_TEMPS]; // % enrichment vs temp
    unsigned char padxy[8];
    int iacstep_table[NO_TEMPS];        // iac steps vs temp
    unsigned int frpm_tablea[NO_INJ][NO_FRPMS]; // fuel, spk rpm tables 
    int fmap_tablea[NO_INJ][NO_FMAPS],  // kpa x 10,
        temp_table[NO_TEMPS];   // deg x 10 (C or F)
    unsigned char padxz[16];
    unsigned char deltV_table[NO_COILCHG_PTS], deltDur_table[NO_COILCHG_PTS];   // Vx10,%age/2
    int cold_adv_table[NO_TEMPS];
    int pwmidle_table[NO_TEMPS];        // 10 pwm

    signed int opentime[4][6];  // 4x 6 point curves of opening time
    signed int opentimev[6];    // 1x 6 voltage for opening time lookup
    signed int smallpw[4][6];   // 4x 6 point curves of opening time
    signed int smallpwpw[6];    // 1x 6 voltage for opening time lookup

    unsigned int waterinj_rpm[8], waterinj_map[4];
    unsigned char waterinj_duty[4][8];

    unsigned char pad852[2];
} page10_data;

//page11 - tables ONLY - this will be kept in paged RAM
typedef struct _page11_data {
    signed char spk_trim[16][6][6];     // trim tables for 10 injector channels. 576 bytes
    unsigned int spk_trim_rpm[6];   // shared rpm bins. 12 bytes
    signed int spk_trim_load[6];    // shared load bins 12 bytes
    unsigned int XAccTable[NO_XTRPMS],  // X factor (%x10) for amount of injected fuel that
        // clings to port wall as function of rpm (during accel).
        TauAccTable[NO_XTRPMS], // time constant (ms) for fuel puddle dissipation vs rpm.
        XDecTable[NO_XTRPMS],   // Same tables but to be used during decel.
        TauDecTable[NO_XTRPMS];
    unsigned int XTrpms[NO_XTRPMS];     // map, rpm values for above 
    //  tables (map in kPax10)
    unsigned int XClt[NO_TEMPS], TauClt[NO_TEMPS];      // same tau, but % scale as function of coolant temp.
    int XClt_temps[NO_TEMPS], TauClt_temps[NO_TEMPS];
    unsigned int n2o1_time[6], n2o1_rpm[6];
    unsigned char n2o1_duty[6];
    int n2o1_pw[6];
    unsigned int n2o1_retard[6];
    unsigned int n2o2_time[6], n2o2_rpm[6];
    unsigned char n2o2_duty[6];
    int n2o2_pw[6];
    unsigned int n2o2_retard[6];
    unsigned char staged_percents[8][8];
    unsigned int staged_rpms[8];
    int staged_loads[8];
    unsigned int MAFFlow[12];
    unsigned char MAFCor[12];
    unsigned int launch_time[6], launch_retard[6];
    unsigned char pad0[30];
} page11_data;

//page12 - tables ONLY - this will be kept in paged RAM
typedef struct _page12_data {
    unsigned int ve_table1[NO_EXFMAPS][NO_EXFRPMS];
    unsigned int ve_table2[NO_EXFMAPS][NO_EXFRPMS];
} page12_data;

//page13 - tables ONLY - this will be kept in paged RAM
typedef struct _page13_data {
    int adv_table1[NO_EXSMAPS][NO_EXSRPMS];     // spark table
    int adv_table2[NO_EXSMAPS][NO_EXSRPMS];     // spark table
} page13_data;

//page18 - tables ONLY - this will be kept in paged RAM
typedef struct _page18_data {
    unsigned int ve_table3[NO_EXFMAPS][NO_EXFRPMS];
    int adv_table3[NO_EXSMAPS][NO_EXSRPMS];
} page18_data;

//page19 - tables ONLY - this will be kept in paged RAM
typedef struct _page19_data {
    unsigned int frpm_tablev1[NO_EXFRPMS];      // fuel rpm axis
    unsigned int frpm_tablev2[NO_EXFRPMS];      // fuel rpm axis
    unsigned int frpm_tablev3[NO_EXFRPMS];      // fuel rpm axis
    int fmap_tablev1[NO_EXFMAPS];       // fuel load axis
    int fmap_tablev2[NO_EXFMAPS];       // fuel load axis
    int fmap_tablev3[NO_EXFMAPS];       // fuel load axis
    unsigned int srpm_table1[NO_EXSRPMS];       // spk rpm axis
    unsigned int srpm_table2[NO_EXSRPMS];       // spk rpm axis
    unsigned int srpm_table3[NO_EXSRPMS];       // spk rpm axis
    int smap_table1[NO_EXSMAPS];        // spk load axis
    int smap_table2[NO_EXSMAPS];        // spk load axis
    int smap_table3[NO_EXSMAPS];        // spk load axis
    unsigned int EAEAWCRPMbins2[NO_FRPMS], EAESOCRPMbins2[NO_FRPMS],
                 EAEAWCKPAbins2[NO_FMAPS],     /* MAP bins for EAE */
                 EAESOCKPAbins2[NO_FMAPS];
    unsigned char EAEBAWC2[12],  /* Added to the Wall coefficient */
        EAEBSOC2[12],            /* sucking off the wall coefficient */
        EAEAWN2[12],             /* added to the walls rpm scalar */
        EAESON2[12],             /* pulled from walls rpm scalar */
        EAEAWW2[12],             /* added to the walls CLT scalar */
        EAESOW2[12];
    int EAEAWWCLTbins2[12], EAESOWCLTbins2[12];
    int ITB_load_loadvals[12];
    int ITB_load_switchpoints[12];
    unsigned int ITB_load_rpms[12];
    unsigned int idleve_table1[4][4];
    unsigned int idleve_table2[4][4];
    int idleve_loads[2][4];
    unsigned int idleve_rpms[2][4];

    unsigned int tc_perfect_vss[10], tc_perfect_time[10], tc_react_x[4], tc_retard[4];
    unsigned char tc_spkcut[4];
    unsigned char tc_addfuel[4];
    unsigned char tc_nitrous[4], tc_boost[4];

    unsigned int knock_rpms[10], knock_thresholds[10];
    int knock_starts[10], knock_durations[10];

    unsigned int pwmidle_cl_initialvalue_rpms[5];
    int pwmidle_cl_initialvalue_matorclt[5];
    unsigned char pwmidle_cl_initialvalues[5][5];
    unsigned char align1;
    int knock_clts[4];
    unsigned int knock_upscale[4];
    unsigned int tc_perfect_rpm[10];
    char tc_boost_duty_delta[4];
    unsigned char pad0[18];
} page19_data;

//page21 - tables ONLY - this will be kept in paged RAM
typedef struct _page21_data {
    unsigned int pwm_rpms[6][6];
    int pwm_loadvals[6][6];
    unsigned char pwm_duties[6][6][6]; // 6 off 6x6 tables
    int                         // this is primePWtable2, CrankPctTable2, asePctTable2, aseCntTable2
    CWPrime2[NO_TEMPS], CrankPctTable2[NO_TEMPS], CWAWEV2[NO_TEMPS], CWAWC2[NO_TEMPS];
    unsigned char warmen_table2[NO_TEMPS]; // % enrichment vs temp
    int temp_table_p21[NO_TEMPS];
    unsigned int frpm_tablev4[NO_EXFRPMS];      // fuel rpm axis
    int fmap_tablev4[NO_EXFMAPS];       // fuel load axis
    unsigned int srpm_table4[NO_EXSRPMS];       // spk rpm axis
    int smap_table4[NO_EXSMAPS];        // spk load axis
    int dualfuel_temp[10], dualfuel_temp_adj[10], dualfuel_press[10], dualfuel_press_adj[10]; 
    int inj_timing_sec[12][12];
    int inj_timing_sec_load[12];
    unsigned int inj_timing_sec_rpm[12];
    unsigned char pad[10];
} page21_data;

//page22 - tables ONLY - this will be kept in paged RAM
typedef struct _page22_data {
    unsigned int ve_table4[NO_EXFMAPS][NO_EXFRPMS];
    int adv_table4[NO_EXSMAPS][NO_EXSRPMS];
} page22_data;

//page23 - tables ONLY - this will be kept in paged RAM
typedef struct _page23_data {
    // Generic spare port parameters: spr_port = 0(don't use)/1(use),where
    // spr_port[0-7] = PM2 - PM5; PTT6,7; PORTA0; they are set as outs to main
    //  pcb; out_offset,out_byte= byte offset from start of outpc structure and
    // size in bytes of 1st, 2nd variables to be tested for setting port;
    // condition='<', '>', '=';cond12 = '&','|',' ' connects the conditions for
    // the two variables with ' ' meaning only the first variable condition is
    // desired; thresh = value for the condition(e.g., var1 > thresh1); init_val,
    // port_val=value (0/1) to which the pin will be set at startup and when the
    // condition(s) is met; hyst is a hysteresis delta and works as ff: if a
    // setpoint condition is > and it is met, set port to val and leave until
    // variable is < thresh - hyst, then set pin back to 1 - val. Similarly if
    // condition is <, wait til var > thresh + hyst. For dual conditions, the
    // hysteresis conditions are evaluated the same way, but use the opposite
    // of cond12 to connect them (if cnd12 is &, use | and vice versa).
// NPORT == 48
    char spr_port[NPORT], condition1[NPORT], condition2[NPORT], cond12[NPORT],
         init_val[NPORT], port_val[NPORT],
         out_byte1[NPORT], out_byte2[NPORT]; // These are all a gross waste of memory
    int out_offset1[NPORT], out_offset2[NPORT],
        thresh1[NPORT], thresh2[NPORT], hyst1[NPORT], hyst2[NPORT];
    //----------- end spare ports
    unsigned char pad[64];
} page23_data;

//page24 - tables ONLY - this will be kept in paged RAM
typedef struct _page24_data {
    unsigned int narrowband_tgts[12][12];
    unsigned int narrowband_tgts_rpms[12];
    int narrowband_tgts_loads[12];
// ALS
    unsigned char als_opt, als_maxtime;
    int xxals_mintps, als_maxtps;
    unsigned char als_iac_dutysteps, als_in_pin, als_out_pin, als_pwm_duty;
    unsigned char als_pausetime, pad785;
    int als_minclt, als_maxclt;
    unsigned int als_rpms[6], als_tpss[6];
    int als_timing[6][6], als_addfuel[6][6];
    unsigned char als_fuelcut[6][6], als_sparkcut[6][6];
    int als_minrpm, als_maxrpm, als_maxmat;
// VVT
    int vvt_onoff_ang;
    unsigned char vvt_opt1, vvt_hold_duty, vvt_out[4];
    unsigned int vvt_ctl_ms;
    unsigned char vvt_ctl_Kp, vvt_ctl_Ki, vvt_ctl_Kd, vvt_test_duty;
    int vvt_min_ang[4], vvt_max_ang[4];
    unsigned char vvt_opt2, vvt_opt3;
    int vvt_timing_load[8];
    unsigned int vvt_timing_rpm[8];
    int vvt_timing[2][8][8];
    unsigned char vvt_tth[4];
    int vvt_coldpos[2];
//TC LU
    unsigned char tclu_opt, tclu_brakepin, tclu_enablepin, tclu_outpin, tclu_gearmin, tclu_delay;
    unsigned int tclu_vssmin;
    int tclu_tpsmin, tclu_tpsmax;
    unsigned int tclu_mapmin, tclu_mapmax;
    unsigned char vvt_opt4, vvt_opt5;
    unsigned char vvt_hold_duty_exh, vvt_ctl_Kp_exh, vvt_ctl_Ki_exh, vvt_ctl_Kd_exh;
    unsigned char vvt_cam1tth1, vvt_cam1tth2, vvt_cam2tth1, vvt_cam2tth2, vvt_cam3tth1, vvt_cam3tth2, vvt_cam4tth1, vvt_cam4tth2, vvt_opt6;
    unsigned char pad1;
    unsigned char als_rifuelcut[6][6];
    unsigned int als_rirpms[6], als_ritpss[6];
    unsigned char pad[6];
} page24_data;


//page25 - tables ONLY - this will be kept in paged RAM
typedef struct _page25_data {
    unsigned int mafv[64], mafflow[64];
    int alpha_map_table[NO_ATPSS][NO_ARPMS]; // Table of ave map values (kpa x 10) for 
                  //  [tps][rpm] pairs. Used only for alpha-N fallback mode.
    int amap_tps[NO_ATPSS];     // Tps values (% x 10) for alpha_map table
    unsigned int amap_rpm[NO_ARPMS];     // Rpm values for alpha_map table
    int tpswot_tps[6];     // Tps values (% x 10) for alpha_map table
    unsigned int tpswot_rpm[6];     // Rpm values for alpha_map table
    int MatVals[NO_MAFMATS];       // air temperatures (degx10) for air density correction table. Old version.
    int AirCorDel[NO_MAFMATS];     // Air density corrections (+/-%) - added to eq. value. Old version.
    unsigned char boost_ctl_cl_pwm_targs1[8][8];
    unsigned char boost_ctl_cl_pwm_targs2[8][8];
    unsigned int boost_ctl_cl_pwm_rpms1[8];
    unsigned int boost_ctl_cl_pwm_rpms2[8];
    int boost_ctl_cl_pwm_targboosts1[8];
    int boost_ctl_cl_pwm_targboosts2[8];
    int blendx[7][9];
    unsigned char blendy[7][9];
    unsigned int tcslipx[9];
    unsigned char tcslipy[9];
    unsigned char padding[216];
} page25_data;

typedef struct _ramonly_data { // 1k in size
    unsigned int rtvarsel[256];
    char ltt_table1[16][16];// learned trim table (bottom RH corner used for sequence info in flash)
    unsigned int ltt_rpms[16];     // rpm axis - copied from VE1, not written to flash
    int ltt_loads[16];             // load axis - same
    unsigned char ro_pad2[192];     // fill out struct
} ramonly_data;

typedef struct _vars1_data {
    int ck_map_sum, ck_mat_sum, ck_clt_sum, ck_tps_sum, ck_afr0_sum, ck_batt_sum, ck_cnt;
    int ck_map_last, ck_mat_last, ck_clt_last, ck_tps_last, ck_afr0_last, ck_batt_last, ck_sync_last;
    unsigned char ck_map_min_cnt, ck_mat_min_cnt, ck_clt_min_cnt, ck_tps_min_cnt, ck_afr0_min_cnt, ck_batt_min_cnt;
    unsigned char ck_map_max_cnt, ck_mat_max_cnt, ck_clt_max_cnt, ck_tps_max_cnt, ck_afr0_max_cnt, ck_batt_max_cnt;
    unsigned int ck_egt_sum[NUM_EGT_CHECK], ck_egt_last[NUM_EGT_CHECK], ck_egt_min_cnt[NUM_EGT_CHECK], ck_egt_max_cnt[NUM_EGT_CHECK];
    unsigned int rpmdot_data[RPMDOT_N][2];
    unsigned int vssdot_data[VSSDOT_N][2][2];
    int tpsdot_data[TPSDOT_N][2];
    int mapdot_data[MAPDOT_N][2];

} vars1_data;

typedef struct {
    union {
        page4_data pg4;
        page8_data pg8;
        page12_data pg12;
        page21_data pg21;
        vars1_data vars1;
    };
    union {
        page5_data pg5;
        page9_data pg9;
        page13_data pg13;
        page22_data pg22;
        page25_data pg25; // 2nd 1k block in f0
    };
    union {
        page10_data pg10;
        page18_data pg18;
        page23_data pg23;
        ramonly_data trimpage;  // 3rd 1k block in f0
    };
    union {
        page11_data pg11;
        page19_data pg19;
        page24_data pg24;
    };
} ramwindef;

typedef union _ign_time {
    long time_32_bits;
    unsigned int time_16_bits[2];
} ign_time;

typedef struct _ign_event {
    ign_time time;              // time after the tooth to fire
    char tooth;                 // the tooth to schedule an event from
    unsigned int coil;
    char ftooth;                // "force dwell" tooth
    unsigned char fs;           // "force spark" flag (only really need a byte)
} ign_event;

typedef struct _inj_event {
    unsigned int time;
    unsigned char volatile *volatile port;
    unsigned char pin;
    char tooth;
} inj_event;

typedef struct _map_event {
    unsigned int time;
    unsigned int map_window_set;
    char tooth;
    unsigned char evnum;
} map_event;

typedef struct _ign_queue {
    unsigned int sel;
    unsigned int time_us;
    unsigned int time_mms;
} ign_queue;

typedef struct _xg_queue {
    unsigned int sel;
    unsigned int cnt;
} xg_queue;

void set_coil(unsigned int *, unsigned int *);

typedef struct {
    unsigned int seconds, pw1, pw2, rpm;        // pw in usec
    int adv_deg;                // adv in deg x 10
    unsigned char squirt, engine, afrtgt1, afrtgt2;     // afrtgt in afr x 10
/*
; Squirt Event Scheduling Variables - bit fields for "squirt" variable above
inj1:    equ    0       ; 0 = no squirt; 1 = inj squirting
inj2:    equ    1

see below; 
Engine Operating/Status variables - bit fields for "engine" variable above
ready:  equ     0       ; 0 = engine not ready; 1 = ready to run
                                               (fuel pump on or ign plse)
crank:  equ     1       ; 0 = engine not cranking; 1 = engine cranking
startw: equ     2       ; 0 = not in startup warmup; 1 = in startw enrichment
warmup: equ     3       ; 0 = not in warmup; 1 = in warmup
tpsaen: equ     4       ; 0 = not in TPS acceleration mode; 1 = TPS acceleration mode
tpsden: equ     5       ; 0 = not in deacceleration mode; 1 = in deacceleration mode
mapaen: equ     6
mapden: equ     7
*/
#define ENGINE_READY  0x01
#define ENGINE_CRANK  0x02
#define ENGINE_ASE    0x04
#define ENGINE_WUE    0x08
#define ENGINE_TPSACC 0x10
#define ENGINE_TPSDEC 0x20
#define ENGINE_MAPACC 0x40
#define ENGINE_MAPDEC 0x80

    unsigned char wbo2_en1, wbo2_en2;   // from wbo2 - indicates whether wb afr valid
    int baro, map, mat, clt, tps, batt, ego1, ego2, knock,      // baro - kpa x 10
        // map - kpa x 10
        // mat, clt deg(C/F)x 10
        // tps - % x 10
        // batt - vlts x 10
        // ego1,2 - afr x 10
        // knock - volts x 100
    egocor1, egocor2, aircor, warmcor,  // all in %
    tpsaccel, tpsfuelcut, barocor, gammae,      // tpsaccel - acc enrich(.1 ms units)
        // tpsfuelcut - %
        // barocor,gammae - %
    vecurr1, vecurr2, iacstep, cold_adv_deg,    // vecurr - %
        // iacstep - steps
        // cold_adv_deg - deg x 10
    tpsdot, mapdot;             // tps, map rate of change - %x10/.1 sec,
    // kPax10 / .1 sec
    unsigned int coil_dur;      // msx10 coil chge set by ecu
    int mafload, fuelload,          // maf for future; kpa (=map or tps)
        fuelcor;                // fuel composition correction - %
    unsigned char port_status,  // Bits indicating spare port status.
        knk_rtd;                // amount of ign retard (degx10) subtracted from normal advance.
    unsigned int EAEfcor1;
    int egoV1, egoV2;           // ego sensor readbacks in Vx100
    unsigned char status1, status2, status3, status4;   // added ms2extra
    unsigned int looptime;
    unsigned int istatus5;
    unsigned int tpsadc;        //lets do a real calibration
    int fuelload2;
    int ignload;
    int ignload2;
    unsigned char synccnt;
    char timing_err;
    unsigned long wallfuel1;
    unsigned long wallfuel2;
    int sensors[16];             // generic sensors
    unsigned char canin1_8, canout1_8, canout9_16;  // CAN-extended remote digi ports
    unsigned char boostduty;
    int n2o_addfuel, n2o_retard;
    unsigned int pwseq[16];      // sequential pulsewidths
    unsigned char nitrous1_duty, nitrous2_duty;
    int egt[16];
    unsigned int maf;
    unsigned int gpiopwmin[4];
    unsigned int fuelflow, fuelcons;
    unsigned int EAEfcor2;
    unsigned char syncreason;
    unsigned char sd_status;
    int eaeload;
    int afrload;
    unsigned char gear, status6;
    int rpmdot, vss1dot, vss2dot, accelx, accely, accelz;
    unsigned char duty_pwm[6];
    unsigned char afr[16];
    unsigned int egov[16];
    int egocor[16];
    unsigned char stream_level;
    unsigned char water_duty;
    unsigned int dwell_trl;
    unsigned int vss1, vss2, ss1, ss2; // In 0.1 meters/sec for vss, shaft rpm for ss
    unsigned int nitrous_timer_out;
    unsigned int sd_filenum;
    unsigned char sd_error, sd_phase;   // phase only temporary
    unsigned char boostduty2;    // WAS spare value user0 for the 'user defined'
    unsigned char status7;
    int vvt_ang[4];
    int inj_timing_pri, inj_timing_sec;
    int vvt_target[4];
    unsigned char vvt_duty[4];
    unsigned int fuel_pct, fuel_temp;
    unsigned int cl_idle_targ_rpm;
    int tps_accel;
    int map_accel;
    int total_accel;
    unsigned char knock_cyl[16];
    unsigned int launch_timer;
    int launch_retard;
    unsigned int maf_volts;
    unsigned char porta, portb, porteh, portk, portmj, portp, portt;
    unsigned char xxsparexx;
    int boost_targ_1, boost_targ_2;
    int airtemp;
    unsigned int cel_status;
} variables;
// remember to add the above variables to realtime copy in serial.c

/* meanings of outpc.syncreason
e.g.
0 = no problem
1 = init error
2 = missing tooth at wrong time
3 = too many teeth before missing tooth (last)
4 = too few teeth before missing tooth (last)
5 = 1st tooth failed test
6 = nonsense input (last)
7 = nonsense input (mid)
8 = too many teeth before missing tooth (mid)
9 = too few teeth before missing tooth (mid)
10 = too many teeth before end of sequence
11 = too few teeth before second trigger
12 = too many sync errors
13 = dizzy wrong edge
14 = trigger return vane size
15 = EDIS
16 = EDIS
17 = second trigger not found when expected
18 = poll level wrong phase at re-check

space for more common reasons
plus other special reasons for the custom wheels
20 = subaru 6/7 tooth 6 error
21 = subaru 6/7 tooth 3 error
22 = Rover #2 missing tooth error
23 = 420A long tooth not found
24 = 420A cam phase wrong
25 = 420A 
26 = 420A
27 = 420A
28 = 36-1+1
29 = 36-2-2-2 semi sync failed
30 = 36-2-2-2 tooth 14 error
31 = Miata 99-00 - 2 cams not seen
32 = Miata 99-00 - 0 cams seen
33 = 6G72 - tooth 2 error
34 = 6G72 - tooth 4 error
35 = Weber-Marelli
36 = CAS 4/1
37 = 4G63
38 = 4G63
39 = 4G63
40 = Twin trigger
41 = Twin trigger
42 = Chrysler 2.2/2.5
43 = Renix
44 = Suzuki Swift
45 = Vitara
46 = Vitara
47 = Daihatsu 3
48 = Daihatsu 4
49 = VTR1000
50 = Rover #3
51 = GM 7X
52 = 36-2-2-2 tooth 30 error
53 = rc51 semi error
54 = rc51 re-sync error tooth 6
55 = rc51 re-sync error tooth 16
56 = rc51 re-sync error tooth 18
57 = fiat 1.8 tooth 0
58 = fiat 1.8 tooth 3
59 = fiat 1.8 tooth 6
60 = fiat 1.8 tooth 9
61 = 36-1+1 first
62 = 36-1+1 second
63 = QR25DE semi failed
64 = QR25DE lost running sync
65 = CAS360 running
66 = NGC8 semi failed
67 = LS1 semi failed
68 = LS1 resync failed
69 = YZF1000 resync failed
70 = 36-1+1 no cam
71 = Honda Acura resync failed
72 = Honda Acura resync tooth 7 failed
73 = Honda Acura resync tooth 29 failed
74 = VQ35DE semi 1
75 = VQ35DE semi 2
76 = VQ35DE resync
77 = Jeep 2000 resync fail
78 = Jeep 2002 semi 1
79 = Jeep 2002 semi 2
80 = Jeep 2002 resync fail
81 = GM7X cam resync
82 = Zetec VCT resync
83 = Zetec VCT cam phase on resync
84 = 2JZ VVTi resync
85 = 2JZ VVTi cam phase on resync
86 = TSX sync
87 = TSX crank re-sync
88 = TSX cam re-sync
89 = mazda6 2.3vvt sync
90 = mazda6 2.3vvt resync
91 = NGC4 cam failed resync
92 = LS1 failed cam re-sync
93 = Viper gen1 crank long/short error
94 = Viper gen1 double cam tooth error
96 = Viper gen1 single cam tooth error
*/

typedef struct {
    unsigned int adc[24];
    union {
        unsigned int pwmin16[4];
        unsigned long pwmin32[4];
    };
    union {
        unsigned int vss1_16;
        unsigned char vss1_8;
    };
    union {
        unsigned int vss2_16;
        unsigned char vss2_8;
    };
    unsigned char gear;
    unsigned char canpwmout[8];
    unsigned char rtc_sec, rtc_min, rtc_hour, rtc_day, rtc_date, rtc_month;
    unsigned int rtc_year;
    unsigned int ego[16];
    unsigned char spare; // USE THIS
    int FuelAdj, SpkAdj, IdleAdj, SprAdj; // allow other devices to change our behaviour. Published offsets of 512+118 = 630 et seq. AVOID MOVING
    unsigned char setrtc_sec, setrtc_min, setrtc_hour, setrtc_day, setrtc_date, setrtc_month;
    unsigned int setrtc_year;
    unsigned char setrtc_lock;
    unsigned int testmodelock; // must always be written as 12345 for valid command
    unsigned char testmodemode; // write a 0 to disable, 1 to enable, then 2 etc. for run modes
} datax;

// Prototypes - Note: ISRs prototyped above.
int main(void) FAR_TEXTfe_ATTR; // use fe to give 16k to play with
int median3pt(int*, int) FAR_TEXTf7_ATTR;
void sample_map_tps(char *) FAR_TEXTfa_ATTR;
void calc_baro_mat_load(void) FAR_TEXTfa_ATTR;
char crank_calcs(void) FAR_TEXTf3_ATTR;
void warmup_calcs(void) FAR_TEXTf9_ATTR;
void normal_accel(void) FAR_TEXTfe_ATTR;
void new_accel(long *, long *) FAR_TEXTf2_ATTR;
int new_accel_calc_percent(int, int *, int *, int, unsigned char,
                            unsigned char) FAR_TEXTf2_ATTR;
void main_fuel_calcs(long *, long *) FAR_TEXTfc_ATTR;
void long_term_trim() FAR_TEXTf3_ATTR;
void flex_fuel_calcs(long *, long *) FAR_TEXTf9_ATTR;
void do_overrun(void) FAR_TEXTf9_ATTR;
void n2o_launch_additional_fuel(void) FAR_TEXTfe_ATTR;
void do_knock(void) FAR_TEXTf3_ATTR;
void calc_advance(long *) FAR_TEXTfe_ATTR;
void do_launch(long *) FAR_TEXTf3_ATTR;
void do_everytooth_calcs(long *, long *, char *) FAR_TEXTfe_ATTR;
void bubble_sort(int *, char) FAR_TEXTf9_ATTR;
long long_abs(long) FAR_TEXTf9_ATTR;
int do_median_rpmdot(int) FAR_TEXTf9_ATTR;
void do_tach_mask(void) FAR_TEXTfa_ATTR;
void do_revlim_overboost_maxafr(void) FAR_TEXTf3_ATTR;
void injpwms(void) FAR_TEXTf4_ATTR;
void do_final_fuelcalcs(void) FAR_TEXTfe_ATTR;
void do_sequential_fuel(void) FAR_TEXTf9_ATTR;
void do_seqpw_calcs(unsigned long, int, int) FAR_TEXTf3_ATTR;
void handle_ovflo(void) FAR_TEXTf9_ATTR;
void handle_spareports(void) FAR_TEXTf9_ATTR;
void can_poll(void) FAR_TEXTfc_ATTR;
void do_egt(void) FAR_TEXTf2_ATTR;
void do_sensors(void) FAR_TEXTf9_ATTR;
void accelerometer(void) FAR_TEXTfe_ATTR;
void wheel_fill_event_array(ign_event *, ign_event *, int, int,
                            ign_time, unsigned int,
                            unsigned char) FAR_TEXTf9_ATTR;
void wheel_fill_map_event_array(map_event *, int, ign_time,
                                unsigned int, unsigned int) FAR_TEXTf9_ATTR;
#define INJ_FILL_ROTARY 0x1
#define INJ_FILL_STAGED 0x2
#define INJ_FILL_TESTMODE 0x4
void wheel_fill_inj_event_array(inj_event *, int, ign_time, unsigned int,
                                unsigned char,
                                unsigned char,
                                unsigned char) FAR_TEXTf3_ATTR;
void inj_event_array_portpin(unsigned char) FAR_TEXTf3_ATTR;
void setup_staging(void) FAR_TEXTf4_ATTR;
void calc_staged_pw(unsigned long) FAR_TEXTfa_ATTR;
unsigned char calc_duty(unsigned long) FAR_TEXTfa_ATTR;
void staged_on(unsigned long) FAR_TEXTfa_ATTR;
void set_prime_ASE(void) FAR_TEXTf4_ATTR;
void set_EAE_lagcomp(void) FAR_TEXTfa_ATTR;
void main_init(void) FAR_TEXTf8_ATTR;
void generic_IO_setup(volatile unsigned char * volatile *port, unsigned char *pin,
                      unsigned char *pin_match, unsigned char feature_conferr,
                      unsigned char pinsetting, unsigned char hwpwm,
                      unsigned char polarity, unsigned char direction, unsigned char) FAR_TEXTf5_ATTR;
void generic_adc_setup(volatile unsigned short **port, unsigned char feature_conferr,
                      unsigned char pinsetting, unsigned char) FAR_TEXTf5_ATTR;void vss_init() FAR_TEXTf9_ATTR;
void generic_hwpwm_setup(volatile unsigned char * volatile *port, unsigned char feature_conferr,
                      unsigned char pinsetting, unsigned char hwpwm,
                      unsigned char polarity, unsigned char) FAR_TEXTf5_ATTR;
void generic_digin_setup(volatile unsigned char * volatile *port, unsigned char *pin,
                      unsigned char *pin_match, unsigned char feature_conferr,
                      unsigned char pinsetting, unsigned char) FAR_TEXTf5_ATTR;
void generic_digout_setup(volatile unsigned char * volatile *port, unsigned char *pin,
                      unsigned char feature_conferr,
                      unsigned char pinsetting, unsigned char) FAR_TEXTf5_ATTR;
void generic_timer_setup(unsigned char, unsigned char, unsigned char, unsigned char) FAR_TEXTf6_ATTR;
void spr_port_init(void) FAR_TEXTf8_ATTR;
void var_init(void) FAR_TEXTf9_ATTR;
void pinport_init(void) FAR_TEXTfa_ATTR;
void egt_init(void) FAR_TEXTf2_ATTR;
void sensors_init(void) FAR_TEXTf2_ATTR;
void calc_reqfuel(unsigned int) FAR_TEXTfa_ATTR;
void calc_divider(void) FAR_TEXTf9_ATTR;
void cp_flash_ram(void);        // needs to be in non-banked RAM
void ign_reset(void) FAR_TEXTf9_ATTR;
void ign_kill(void) FAR_TEXTf9_ATTR;
void get_adc(char chan1, char chan2) FAR_TEXTfc_ATTR;
#define ego_get_sample() get_adc(5, 6)
int move_IACmotor(void);        // FAR_TEXTfc_ATTR;
int barocor_eq(int baro) FAR_TEXTfc_ATTR;
int aircor_eq(int mat) FAR_TEXTfc_ATTR;
void set_spr_port(char port, char val) FAR_TEXTfc_ATTR;
int coil_dur_table(int delta_volt) FAR_TEXTfc_ATTR;
int CW_table(int clt, int *table, int *temp_table,
             unsigned char page) FAR_TEXTfc_ATTR;
int intrp_2ditable(unsigned int x, int y, unsigned char nx,
                   unsigned char ny, unsigned int *x_table, int *y_table,
                   int *z_table, unsigned char page);
void CanInit(void) FAR_TEXTf9_ATTR;
void can_xsub01(void) FAR_TEXTfe_ATTR;
unsigned int can_sendburn(unsigned int, unsigned int) FAR_TEXTfe_ATTR;
unsigned int can_reqdata(unsigned int, unsigned int, unsigned int, unsigned int) FAR_TEXTfe_ATTR;
unsigned int can_snddata(unsigned int, unsigned int, unsigned int, unsigned int, unsigned int) FAR_TEXTfe_ATTR;
unsigned int can_t(unsigned int, unsigned int, unsigned int, unsigned int) FAR_TEXTfe_ATTR;
unsigned int can_crc32(unsigned int, unsigned int) FAR_TEXTf3_ATTR;
unsigned int can_inc_ring(unsigned int) FAR_TEXTf3_ATTR;
void Flash_Init(void) FAR_TEXTf9_ATTR;
void ign_wheel_init(void) FAR_TEXTfb_ATTR;
void calc_absangs(void) FAR_TEXTfa_ATTR;
void boost_ctl_init(void) FAR_TEXTfc_ATTR;
void boost_ctl(void) FAR_TEXTfc_ATTR;
void boost_ctl_ol(int, unsigned char, char *) FAR_TEXTfc_ATTR;
void boost_ctl_cl(int, int, int, int, int, unsigned char, unsigned char, char *) FAR_TEXTfc_ATTR;
void idle_ctl_init(void) FAR_TEXTfc_ATTR;
void idle_ctl(void) FAR_TEXTfc_ATTR;
void idle_test_mode(void) FAR_TEXTfc_ATTR;
void idle_on_off(void) FAR_TEXTfc_ATTR;
void idle_iac_warmup(void) FAR_TEXTfc_ATTR;
void idle_pwm_warmup(void) FAR_TEXTfc_ATTR;
void idle_closed_loop(void) FAR_TEXTfc_ATTR;
void idle_target_lookup(void) FAR_TEXTfc_ATTR;
void idle_closed_loop_throttlepressed(int rpm_thresh) FAR_TEXTf9_ATTR;
void idle_closed_loop_throttlelifted(int rpm_thresh, int rpm_targ) FAR_TEXTf5_ATTR;
void idle_closed_loop_pid(int targ_rpm, int rpm_thresh, char savelast) FAR_TEXTfa_ATTR;
int idle_closed_loop_newtarg(int targ_rpm) FAR_TEXTf9_ATTR;
void idle_closed_loop_calc_dp_decay(void) FAR_TEXTf9_ATTR;
long generic_pid_routine(int, int, int, int, int, int, int, int, long *,
                         long *, unsigned char) FAR_TEXTf9_ATTR;
void convert_unitless_percent(int, int, int, int, long *, long *) FAR_TEXTf9_ATTR;
void idle_voltage_compensation(void) FAR_TEXTf9_ATTR;
void idle_ac_idleup(void) FAR_TEXTf9_ATTR;
void fan_ctl_idleup(void) FAR_TEXTf9_ATTR;
void run_EAE_calcs(void) FAR_TEXTf9_ATTR;
void run_xtau_calcs(void) FAR_TEXTfc_ATTR;
void ego_init(void) FAR_TEXTf3_ATTR;
void ego_get_targs(void) FAR_TEXTfa_ATTR;
void ego_get_targs_gl(void) FAR_TEXTf9_ATTR;
void ego_closed_loop_simple(int *, unsigned int, int, int) FAR_TEXTf4_ATTR;
void ego_closed_loop_pid_dopid(long *, unsigned long, unsigned int, unsigned int, int) FAR_TEXTf4_ATTR;
void dribble_burn(void) FAR_TEXTf4_ATTR;
void ltt_burn(void) FAR_TEXTf4_ATTR;

void ego_calc(void) FAR_TEXTfe_ATTR;
void chk_crc(void) FAR_TEXTfe_ATTR;
signed int calc_opentime(unsigned char) FAR_TEXTfe_ATTR;
unsigned int smallpw(unsigned int, unsigned char) FAR_TEXTfe_ATTR;
unsigned char afrLF_calc(long t) FAR_TEXTfe_ATTR;
void user_defined(void) FAR_TEXTf2_ATTR;
void do_maxafr(void) FAR_TEXTfc_ATTR;
void speed_sensors(void) FAR_TEXTfe_ATTR;
void nitrous(void) FAR_TEXTf8_ATTR;
void traction(long *) FAR_TEXTf2_ATTR;
void water_inj(void) FAR_TEXTf4_ATTR;
void ck_log_clr(void) FAR_TEXTf4_ATTR;
void poll_i2c_rtc(void) FAR_TEXTf2_ATTR;
void antilag(void) FAR_TEXTf7_ATTR;
void vvt(void) FAR_TEXTf4_ATTR;
void vvt_ctl_pid_init(void) FAR_TEXTf4_ATTR;
void vvt_pid(unsigned char) FAR_TEXTf4_ATTR;
void tclu(void) FAR_TEXTf4_ATTR;

void calc_spk_trims(void) FAR_TEXTf9_ATTR;
void calc_fuel_trims(void) FAR_TEXTf9_ATTR;
void gearpos(void) FAR_TEXTf9_ATTR;
void calcvssdot(void) FAR_TEXTf9_ATTR;
void shifter(void) FAR_TEXTf2_ATTR;
void generic_pwm(void) FAR_TEXTfc_ATTR;
void generic_pwm_outs(void) FAR_TEXTfc_ATTR;
void knock_spi(void) FAR_TEXTf5_ATTR;
void serial(void) FAR_TEXTf3_ATTR;
void debug_init() FAR_TEXTf3_ATTR;
void debug_str(unsigned char*) FAR_TEXTf3_ATTR;
void debug_byte(unsigned char) FAR_TEXTf3_ATTR;
void debug_bytehex(unsigned char) FAR_TEXTf3_ATTR;
void debug_bytedec(unsigned char) FAR_TEXTf3_ATTR;
void debug_byte2dec(unsigned char) FAR_TEXTf3_ATTR;
void debug_inthex(unsigned int) FAR_TEXTf3_ATTR;
void debug_longhex(unsigned long) FAR_TEXTf3_ATTR;

void SPI_baudlow(void) FAR_TEXTf7_ATTR;
void SPI_baudhigh(void) FAR_TEXTf7_ATTR;
void do_sdcard(void)  FAR_TEXTf7_ATTR;
unsigned char asc2num(unsigned int) FAR_TEXTf7_ATTR;
void write_dirent(unsigned int, unsigned char) FAR_TEXTf3_ATTR;
void read_compress() FAR_TEXTf3_ATTR;
void do_spi2(void) FAR_TEXTf3_ATTR;
void ckstall(void) FAR_TEXTf2_ATTR;
void clear_all(void) FAR_TEXTfb_ATTR;
void syncfirst(void) TEXT_ATTR;
void check_sensors(void) FAR_TEXTfa_ATTR;
void check_sensors_init(void) FAR_TEXTfa_ATTR;
void check_sensors_reset(void) FAR_TEXTfa_ATTR;
void stack_watch_init(void) FAR_TEXTfa_ATTR;
void stack_watch(void) FAR_TEXTfa_ATTR;
int blend_xaxis(unsigned char) FAR_TEXTf9_ATTR;

// For ASM routines the memory allocation is set in the source file
// *** BUT *** imperative to set the same bank here as well or the C code jumps incorrectly (wrong page in 'call')

void erasefactor(void) FAR_TEXTfc_ATTR;
void do_complog_pri(unsigned long);
void configerror(void) FAR_TEXTf4_ATTR;
void config_xgate(void) FAR_TEXTfa_ATTR;
unsigned long muldiv(unsigned int, unsigned long) TEXT_ATTR;
unsigned long g_read32(unsigned int);
unsigned int g_read16(unsigned int);
unsigned char g_read8(unsigned int);
unsigned char g_read_copy(unsigned int, unsigned int, unsigned int);
void g_write32(unsigned long, unsigned int);
void g_write16(unsigned int, unsigned int);
void g_write8(unsigned char, unsigned int);
unsigned long g_crc32buf(unsigned long, unsigned int, unsigned int) FAR_TEXTfb_ATTR;

// These are asm snippets to support compress, mainly asm due to global addressing
int comp_readsd(unsigned int *, unsigned char *) FAR_TEXTfb_ATTR;
int comp_writesd(unsigned int, unsigned int) FAR_TEXTfb_ATTR;
unsigned long comp_readhash(unsigned int) FAR_TEXTfb_ATTR;
void comp_writehash(unsigned int, unsigned long) FAR_TEXTfb_ATTR;
void comp_clearhash(unsigned int) FAR_TEXTfb_ATTR;
unsigned long comp_readcode(unsigned int) FAR_TEXTfb_ATTR;
void comp_writecode(unsigned int, unsigned int) FAR_TEXTfb_ATTR;
void comp_memnmagic(unsigned int, unsigned int) FAR_TEXTfb_ATTR;
void comp_cp_remainder(unsigned int, unsigned int,
                       unsigned int) FAR_TEXTfb_ATTR;
void comp_output3(unsigned int, unsigned long) FAR_TEXTfb_ATTR;
unsigned long crc32buf(unsigned long, unsigned int, unsigned int) FAR_TEXTfb_ATTR;
void cp2serialbuf(unsigned int, unsigned int, unsigned int) FAR_TEXTf3_ATTR;
void do_maplog1(void);
void do_maplog2(void);
void do_maplog_rpm(void);
void do_maplog_tooth(void);
// end of ASM

typedef struct {
    /* CAN Xmt mssge ring buffer:
       can[0] is to hold Rx,TxISR messages,
       can[1] for main loop messages (so don't get clobbered by ISR)
       cxno = no msgs in queue waiting to be sent out
       cxno_in = index for inserting a msg in queue (incr after insert)
       cxno_out = index for sending out a msg (incr after load CAN buf)
       msg_type = CMD,REQ,RESP,XSUB (= set value, request value, respond
       to a request for value, execute a subroutine)
       varblk,varoffset,varbyte = blk no of data structure, byte offset
       from start of structure, no. bytes of data
       datbuf = the actual data (max of 8 bytes)
       dest = id no. of device to which msg being sent.
     */
    unsigned char cxno, cxno_in, cxno_out;
    unsigned char cx_msg_type[NO_CANMSG], cx_myvarblk[NO_CANMSG],
        cx_destvarblk[NO_CANMSG], cx_dest[NO_CANMSG], cx_varbyt[NO_CANMSG];
    unsigned short cx_myvaroff[NO_CANMSG], cx_destvaroff[NO_CANMSG];
    unsigned char cx_datbuf[NO_CANMSG][8];      // max msg data = 8 bytes
} canmsg;

// pointers for spare port pins
typedef struct {
    unsigned char a[8], b[8], e[8], h[8], j[8], k[8], m[8], p[8], t[8], ad0h[8], ad0l[8],
                    canin[8], canout1[8], canout2[8], canadc[24], canpwm[8],
                    pwmscla[2], pwmsclb[2];
    } portpins;

extern const tableDescriptor tables[NO_TBLES] TEXT_ATTR;

extern const unsigned char IACCoilA[8] TEXT_ATTR;
extern const unsigned char IACCoilB[8] TEXT_ATTR;

extern const unsigned char spr_port_hw[3][NPORT] TEXT_ATTR;
extern volatile unsigned char *const spr_port_addr[NPORT] TEXT_ATTR;
extern const unsigned char default_port_inits[40] TEXT_ATTR;
extern const unsigned int setcoil_array_norm[16][16] TEXT_ATTR;
extern const unsigned int setcoil_array_fcfd[4] TEXT_ATTR;
extern const unsigned int twopow[16] TEXT_ATTR;
extern const unsigned char pwmopts[16] TEXT_ATTR;
extern const char RevNum[] TEXT_ATTR;
extern const char Signature[] TEXT_ATTR;
#define SIZE_OF_REVNUM 20
#define SIZE_OF_SIGNATURE 60

/* Std CAN prototype and identifiers */
#define Ignition_Status 0x271 //Audi
#define SteerCSM_Status 0x2C1 //Audi
#define CentElec_Status 0x531 //Audi
#define CentConv_Status 0x591 //Audi
#define DrivDoor_Status 0x381 //Audi
#define PassDoor_Status 0x3B5 //Audi
#define RRDoor_Status   0x4BD //Audi
#define LRDoor_Status   0x4B9 //Audi
#define MultiFnc_Status 0x5C3 //Audi
#define FIS1_TEXT_line1 0x265 //Audi
#define FIS1_TEXT_line2 0x263 //Audi
#define AT_Line2    0x540 //Audi
#define VIN_IDENTIF    0x65F //Audi

#define ABS_Status      0x1A0 // 996 - ABS              416 - ESP/ABS/Brakes
#define Engine_RPM    0x280 // 996 - Motorsteuerger�t 640 - engine control unit RPM 1:1 ratio
#define Engine_Temp     0x289 // 996 - Motorsteuerger�t 649 - Engine Temp /y=46.14+1.67*x where x is degrees C
#define Vehicle_Speed    0x2A8 // 996 - Motorsteuerger�t 680 - engine control unit Speed (78270+(47907*kmh))/1000
#define AT_Line1    0x440 // 996 - Motorsteuerger�t 1088- Tiptronic Gear Position
#define TempoStat_ctl   0x510 // 996 - Cluster out     1296 - Instrument Cluster Tempostat control

void ECANSendMessage(unsigned int, unsigned char *, unsigned char, unsigned char);
void can_broadcast(void);

#include "ms3_vars.h"
#endif
