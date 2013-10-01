/* $Id: ms3_vars.h,v 1.110.2.7 2013/05/22 13:48:01 jsmcortina Exp $
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

/* This header should be included by ms3.h
 * It is parsed to auto-generated ms3_main_vars.h
 */

extern const int cltfactor_table[1024] LOOKUP_ATTR;
extern const int matfactor_table[1024] LOOKUP_ATTR;
extern const unsigned char egofactor_table[1024] LOOKUP_ATTR;
extern const unsigned int maffactor_table[1024] LOOKUP_ATTR;
extern const int gmfactor_table[1024] LOOKUP_ATTR; // hardcoded table - not yet changeable through serial
extern const unsigned char cpf2r[NO_TBLES] TEXT_ATTR;
 
#define NO_TBLES        32
#define NO_EXFMAPS      16
#define NO_EXFRPMS      16
#define NO_EXSMAPS      16
#define NO_EXSRPMS      16
#define NO_FMAPS        12
#define NO_SMAPS        12
#define NO_FRPMS        12
#define NO_SRPMS        12
#define NO_ARPMS         6
#define NO_ATPSS         6
#define NO_INJ           2
#define NO_TEMPS        10
#define NO_MAT_TEMPS     6
#define NO_TPS_DOTS      4
#define NO_MAP_DOTS      4
#define NO_KNKRPMS       6
#define NO_XTRPMS        5
#define NPORT           48
#define NO_COILCHG_PTS   6
#define EGODT           78      /* 78 .128 ms tics = 10 ms */
#define END_SYNCH        3
#define MAXNUMTEETH    136
#define NO_BARS          9
#define NO_MATS          9
#define NO_MAFMATS       6

#define MSG_CMD          0
#define MSG_REQ          1
#define MSG_RSP          2
#define MSG_XSUB         3
#define MSG_BURN         4
#define	OUTMSG_REQ       5
#define	OUTMSG_RSP       6
#define	MSG_XTND         7
// define xtended msgs from 8 on
#define MSG_FWD          8
#define MSG_CRC          9
#define MSG_STD 11 /* for sending STD CAN messages */
#define NO_CANMSG       16
#define MAX_CANBOARDS   16
// Error status words:
//    -bits 0-7 are current errors
//    -bits 8-15 are corresponding latched errors
#define XMT_ERR                 0x0101
#define CLR_XMT_ERR             0xFFFE
#define XMT_TOUT                0x0202
#define CLR_XMT_TOUT            0xFFFD
#define RCV_ERR                 0x0404
#define CLR_RCV_ERR             0xFFFB
#define SYS_ERR                 0x0808

#define TRIGLOGPAGE             0xF0    // also in ms3h.inc
#define TRIGLOGBASE             0x1c00  // also in ms3h.inc
#define SDLOGPAGE               0xF0    // also in ms3h.inc

#define PID_TYPE_STANDARD       0x1
#define PID_TYPE_A              0x2 
#define PID_TYPE_B              0x4 
#define PID_TYPE_C              0x8 
#define PID_INIT                0x10
#define PID_DIRECTION_INVERTED  0x20
#define PID_LOOPTIME_RTC        0x40

#define LOCALFLAGS_RUNFUEL      0x1

#define RPMDOT_N 20
#define VSSDOT_N 20
#define TPSDOT_N 10
#define MAPDOT_N 10

#define RPAGE_VARS1 0xfd
#define v1  ram_window.vars1

 #define MONVER (*(unsigned int *) 0xFEFE)

extern const page8_data flash8 CNFDATA_ATTR;
extern const page9_data flash9 CNFDATA_ATTR;
extern const page10_data flash10 CNFDATA_ATTR;
extern const page11_data flash11 CNFDATA_ATTR;
extern const page12_data flash12 CNFDATA_ATTR;
extern const page13_data flash13 CNFDATA_ATTR;
extern const page18_data flash18 CNFDATA_ATTR;
extern const page19_data flash19 CNFDATA_ATTR;
//no 20
extern const page21_data flash21 CNFDATA_ATTR;
extern const page22_data flash22 CNFDATA_ATTR;
extern const page23_data flash23 CNFDATA_ATTR;
extern const page24_data flash24 CNFDATA_ATTR;
extern const page4_data flash4 CNFDATA2_ATTR;
extern const page5_data flash5 CNFDATA2_ATTR;

extern ramwindef ram_window DATA1_ATTR;
extern page4_data ram4 DATA2_ATTR;
extern page5_data ram5 DATA2_ATTR;

#define time32 time.time_32_bits
/* NB these are indented to hide them from the ASM version */
 #define time16_high time.time_16_bits[0]
 #define time16_low time.time_16_bits[1]

#define MIN_IGN_ANGLE -900
#define NUM_TRIGS 16
extern ign_event dwell_events_a[NUM_TRIGS];
extern ign_event spark_events_a[NUM_TRIGS];
extern ign_event dwell_events_b[NUM_TRIGS];
extern ign_event spark_events_b[NUM_TRIGS];
extern inj_event inj_events_a[NUM_TRIGS];
extern inj_event inj_events_b[NUM_TRIGS];
extern unsigned char inj_cnt_xref[NUM_TRIGS];
extern unsigned char inj_status[NUM_TRIGS];
extern ign_event *dwell_events;
extern ign_event *spark_events;
extern inj_event *inj_events;
extern ign_event next_spark;
extern ign_event next_dwell;
extern ign_event next_dwl_trl;
extern ign_event next_spk_trl;
extern inj_event next_inj[NUM_TRIGS] XGATE_ALIGN_ATTR;
extern ign_time dwl_time_ovflo_trl;
extern ign_time spk_time_ovflo_trl;
extern ign_queue spkq[2], dwellq[2];
extern xg_queue xgspkq[2] XGATE_ALIGN_ATTR, xgdwellq[2] XGATE_ALIGN_ATTR;
extern map_event next_map_start_event, map_start_event[NUM_TRIGS];
extern unsigned char wheeldec_ovflo;
extern unsigned int map_start_countdown, map_window_countdown,
    map_window_set;
extern unsigned int map_temp, maf_temp, mafraw, map_temps[4];
extern unsigned int map_cnt, map_temp_cnt;
extern unsigned long map_sum, map_temp_sum;
extern unsigned char map_temps_cnt, map_deadman;
extern long MAFCoef;
extern unsigned int mafload_no_air;
//#define OVFLO_SPK 0x1
//#define OVFLO_DWL 0x2
#define OVFLO_ROT_SPK 0x4
#define OVFLO_ROT_DWL 0x8
extern unsigned char next_fuel;
extern unsigned char trigger_teeth[NUM_TRIGS];
extern int trig_angs[NUM_TRIGS];
extern int trig_ang;
extern ign_time inj1cntdown, inj2cntdown, inj_time_ovflo[NUM_TRIGS];
extern unsigned int seq_pw[NUM_TRIGS] XGATE_ALIGN_ATTR;

#define COILABIT 0x1
#define COILBBIT 0x2
#define COILCBIT 0x4
#define COILDBIT 0x8
#define COILEBIT 0x10
#define COILFBIT 0x20
#define COILGBIT 0x40
#define COILHBIT 0x80
#define COILIBIT 0x100
#define COILJBIT 0x200
#define COILKBIT 0x400
#define COILLBIT 0x800
#define COILMBIT 0x1000
#define COILNBIT 0x2000
#define COILOBIT 0x4000
#define COILPBIT 0x8000

#define OUTPUT_LATENCY 1        // known latency in code from outpc compare to bit flipping
#define SPK_TIME_MIN 110        // minimum time after tooth before stepping back

extern unsigned int coilsel XGATE_ALIGN_ATTR, dwellsel XGATE_ALIGN_ATTR, tmpcoilsel XGATE_ALIGN_ATTR, tmpdwellsel XGATE_ALIGN_ATTR, dwellsel_next XGATE_ALIGN_ATTR, tmp_coil, tmp_sel;
extern unsigned char rotaryspksel;
extern unsigned char rotarydwlsel;

extern variables outpc;
extern unsigned char canbuf[17];
#define SRLDATASIZE 2048

extern datax datax1;

#define STATUS1_NEEDBURN 1      // need burn
#define STATUS1_LOSTDATA  2     // lost data
#define STATUS1_CONFERR 4       // config error
#define STATUS1_SYNCOK      8
#define STATUS1_SYNCLATCH   0x10
#define STATUS1_FTBLSW   0x20
#define STATUS1_STBLSW   0x40
#define STATUS1_SYNCFULL 0x80

#define STATUS2_NITROUS1  1     // nitrous stage1
#define STATUS2_NITROUS2  2     // nitrous stage2
#define STATUS2_LAUNCHIN 4      // launch input on/off
#define STATUS2_LAUNCH  8       // launch available
#define STATUS2_FLATSHIFT  0x10 // flatshift active
#define STATUS2_SPKCUT 0x20     // doing spk cut
#define STATUS2_OVERBOOST_ACTIVE 0x40
#define STATUS2_CLIDLE  0x80    // idle is in closed-loop mode

#define STATUS3_CUT_FUEL 1
#define STATUS3_DONELOG 2       // set when trigger or tooth log is complete
#define STATUS3_3STEP 4
#define STATUS3_TESTMODE 8 // only for indication
#define STATUS3_3STEPIN 0x10
#define STATUS3_REVLIMSFT 0x20
#define STATUS3_BIKESHIFT 0x40
#define STATUS3_LAUNCHON 0x80
// outpc.status 4,5 are used for ease of getting data values into tuning software

#define STATUS6_EGTWARN 1
#define STATUS6_EGTSHUT 2
#define STATUS6_AFRWARN 4
#define STATUS6_AFRSHUT 8
#define STATUS6_IDLEVE 0x10
#define STATUS6_IDLEADV 0x20
#define STATUS6_FAN 0x40
#define STATUS6_MAPERROR 0x80

#define STATUS7_VVT1ERR 1
#define STATUS7_VVT2ERR 2
#define STATUS7_VVT3ERR 4
#define STATUS7_VVT4ERR 8
#define STATUS7_KNOCK 0x10
#define STATUS7_ACOUT 0x20
#define STATUS7_CEL 0x40
#define STATUS7_LIMP 0x80

// sensor variables
extern int last_tps, last_map, tpsdot_ltch, mapdot_ltch, old_map;
// fuel variables
extern unsigned int pwcalc1, pwcalc2, PrimeP, AWEV, AWC,
    pwcalc_eae1, pwcalc_eae2;
extern unsigned char pwm1_on, pwm2_on;
extern unsigned int RevLimRpm1, RevLimRpm2;
// ignition variables
extern unsigned char pulse_no;
extern long coil_dur_set;
extern unsigned int coil_dur;
extern unsigned long IgnTimerComp, dtpred, dtpred_last, dtpred_last2,
    dtpred_last3, NoiseFilterMin;
// IAC variables
extern unsigned char motor_time_ms;
extern int IACmotor_pos, IACmotor_pos_remainder, last_iacclt,
    tble_motor_pos;
extern unsigned char idle_wait_timer, pwmidle_shift_timer;
extern unsigned int ac_idleup_timer, fan_idleup_timer;
extern unsigned char pwmidle_reset;
extern unsigned int pwmidle_timer;
#define NUM_BOOST_CHANS 2
extern unsigned char boost_ctl_duty[NUM_BOOST_CHANS], boost_ctl_duty_pwm[NUM_BOOST_CHANS];
extern char IAC_moving, IACmotor_reset, IdleCtl, motor_step;
extern int IACmotor_last;
// General variables
extern unsigned int iacpwmctr, sec_timer;
extern unsigned long lmms XGATE_ALIGN_ATTR, t_enable_IC, t_enable_IC2, Rpm_Coeff, ltch_lmms XGATE_ALIGN_ATTR,
    ltch_lmms2, rcv_timeout;
extern unsigned int asecount, adc_lmms;
extern unsigned char flocker, tpsaclk, egocount, igncount, altcount,
    next_adc, first_adc, rxmode, txmode, tble_idx, burn_idx, synch, egopstat[8],
    knk_clk, knk_clk_test, knk_stat, knk_count, mms, millisec, srl_err_cnt;
extern unsigned int tpsaccel_end;
extern unsigned int boost_ctl_timer;
#define SYNC_SYNCED     0x1
#define SYNC_FIRST      0x2
#define SYNC_RPMCALC    0x4
#define SYNC_SEMI       0x8
#define SYNC_SEMI2      0x10
#define SYNC_WCOP       0x20    //  wasted cop for missing + 2nd trig startup
#define SYNC_WCOP2      0x40    //  forced wasted cop

extern long egoerrm1[8][2];
extern unsigned int FSens_Pd;
/* Clocks:
        - igncount: counts up each tach pulse, cleared when hit Divider pulses
               (and injection occurs).
        - asecount: counts up each time igncount = 0 (each Divider pulses).
        - egocount: counts up each tach pulse, cleared when hits EgoCountCmp
        - tpsaclk: counts every .1 sec
        - altcount: flips 0,1,0,1... on each injection, resulting in firing alternate
                injector banks if Alternate option.
*/
extern unsigned int txcnt, txgoal, rxoffset, rxnbytes, rxcnt, tble_word;
extern char bad_ego_flag, bad_ego_ltch, first_clt;
extern int knk_tble_adv, ffspkdel;
extern unsigned long WF1, WF2;
extern unsigned long AWA1, AWA2, SOA1, SOA2;
extern unsigned long tooth_diff_last_2, tooth_diff_last_1, tooth_diff_last,
    tooth_diff_this;
extern ign_time tooth_diff_rpm, tooth_diff_rpm_last;
extern unsigned char no_triggers;
extern unsigned char no_teeth;
extern unsigned char tooth_no, tooth_no_rpm;
extern unsigned char last_tooth;
extern unsigned char mid_last_tooth;

//had to add these when main_init broken off
extern unsigned int tcrank_done, tcold_pos;
extern int start_clt;

// CAN variables
extern unsigned long cansendclk, ltch_CAN;
extern unsigned int can_status;
extern canmsg can[2];
extern unsigned int canrxad, canrxgoal;
extern unsigned int cp_targ, cp_cnt, cp_offset, cp_time;
extern unsigned char cp_id, cp_table;

// pointers for spare port pins
extern portpins *init_portusage_addr;
extern volatile unsigned char *pPTJpin0, *pPTJpin1, *pPTJpin6, *pPTMpin3, *pPTMpin4, *pPTMpin5; // only for LEDs and stepper idle
extern unsigned char dummyReg XGATE_ALIGN_ATTR, lst_pval[NPORT];

// vars added for MS2/Extra
extern unsigned char page;      // which flash data page is presently in ram
extern unsigned char num_spk;   // calculated number of spark outputs
extern unsigned char num_inj, num_inj_pri;   // calculated number of injector outputs
extern unsigned char conf_err, conf_err_port, conf_err_pin, conf_err_feat;     // set if configuration error
extern unsigned int mltimestamp;        // time the mainloop
extern unsigned int EAEdivider;
extern unsigned char EAE_multiplier;
extern unsigned char mmsDiv, boost_ctl_clock;

// these were static in sci isr, asm needs them as global (same thing)
//extern int vfy_fail,;
extern unsigned char CANid, next_txmode, rd_wr, Tcntr;

extern unsigned char flagbyte0;
#define FLAGBYTE0_50MS    1     // use bits of this byte
#define FLAGBYTE0_TO      2     // tacho out divider
#define FLAGBYTE0_MAPLOG  4    // logging MAP
#define FLAGBYTE0_FOUNDFIRST 8
#define FLAGBYTE0_MAPLOGARM 0x10 // MAP logger armed
#define FLAGBYTE0_COMPLOG  0x20 // logging crank + cam teeth (composite loger
#define FLAGBYTE0_TTHLOG  0x40  // logging teeth
#define FLAGBYTE0_TRGLOG  0x80  // logging triggers

extern unsigned char flagbyte1;
#define flagbyte1_trig2active 1
#define flagbyte1_tstmode  2    // test mode enable - set to 1 turns off normal functions
#define flagbyte1_ovfclose 4    // nearly going to overflow
#define flagbyte1_polarity 8    // polarity check on (reduces tests in ISR)
#define flagbyte1_noisefilter 0x10      // noise filter on (reduces tests in ISR)
#define flagbyte1_igntrig 0x20  // ignition trigger LED feature on
#define FLAGBYTE1_EX 0x40       // Run code
#define flagbyte1_trig2statl 0x80       // 2nd trigger state in previous ISR (used by 6g72)

extern unsigned char flagbyte2;
#define flagbyte2_twintrig 1    // for quick checking
#define flagbyte2_twintrignow 2 // event happened just now
#define flagbyte2_crank_ok 4    // ok to go to crank mode (rock crawler / hei bypass)
#define FLAGBYTE2_SCI_CONFIRMED 8 // are we locked onto on serial port
#define flagbyte2_tc0stat 0x10  // Status of crank tach input when 2nd trigger (CAS4/2)
#define flagbyte2_runidle 0x20
#define flagbyte2_EAElag 0x40
#define flagbyte2_MV2 0x80      // return MS2 base style data to keep MV2 happy

extern unsigned char flagbyte3;
#define flagbyte3_firstIAC  1   // from MS2 2.87
#define flagbyte3_kill_srl  2   // from MS2 2.87
#define flagbyte3_can_reset 4   // from MS2 2.87
#define flagbyte3_getcandat 8   // from MS2 2.87
#define flagbyte3_sndcandat 0x10        // from MS2 2.87
#define flagbyte3_runboost  0x20
#define flagbyte3_toothinit 0x40        // set to 1 once we've reached the tooth
#define flagbyte3_samplemap 0x80

extern unsigned char flagbyte4;
#define flagbyte4_first_edis 1  // have we sent the multi-spark SAW word
#define flagbyte4_transition_done 2
#define flagbyte4_staging_on 4
#define flagbyte4_tach2 8       // entered ISR on 2nd tach signal (used locally in timer ISR)
#define flagbyte4_found_miss 0x10
#define flagbyte4_cantuning  0x20       // CAN is being used to read/write/burn tuning data. Avoid fights with serial due to paging mechanism.
#define flagbyte4_idlevereset  0x40
#define flagbyte4_idleadvreset 0x80

extern unsigned char flagbyte5;
#define FLAGBYTE5_CRK_DOUBLE 0x01       // crank - single or double edged
#define FLAGBYTE5_CRK_BOTH   0x02       // crank - triggering on both edges
#define FLAGBYTE5_CAM        0x04       // cam in use
#define FLAGBYTE5_CAM_DOUBLE 0x08       // cam - single or double edged
#define FLAGBYTE5_CAM_BOTH   0x10       // cam - triggering on both edges
#define FLAGBYTE5_CAM_NOISE  0x20       // cam - noise filter on
#define FLAGBYTE5_CAM_POLARITY 0x40     // cam - noise filter on
#define FLAGBYTE5_RUN_XTAU    0x80      // X-tau - run mainloop xtau calcs

extern unsigned char flagbyte6;
#define FLAGBYTE6_INJ_OVFCLOSE 0x01
#define FLAGBYTE6_SD_DEBOUNCE  0x02
#define FLAGBYTE6_SD_MANUAL    0x04
#define FLAGBYTE6_SD_GO        0x08
#define FLAGBYTE6_DONEINIT     0x10
#define FLAGBYTE6_STREAM_CONT  0x20
#define FLAGBYTE6_DELAYEDKILL  0x40
#define FLAGBYTE6_STREAM_HOLD  0x80

extern unsigned char flagbyte7;
#define FLAGBYTE7_VSS1ACT 0x01
#define FLAGBYTE7_VSS2ACT 0x02
#define FLAGBYTE7_SS1ACT 0x04
#define FLAGBYTE7_SS2ACT 0x08
#define FLAGBYTE7_SS_INIT 0x10
#define FLAGBYTE7_SENDFILE 0x20
#define FLAGBYTE7_SF_GO 0x40
#define FLAGBYTE7_CRC 0x80

extern unsigned char flagbyte8;
#define FLAGBYTE8_USE_MAF       0x01
#define FLAGBYTE8_USE_MAF_ONLY  0x02
#define FLAGBYTE8_MODE10        0x04
#define FLAGBYTE8_SAMPLE_VSS1   0x08
#define FLAGBYTE8_SAMPLE_VSS2   0x10
#define FLAGBYTE8_SAMPLE_SS1    0x20
#define FLAGBYTE8_SAMPLE_SS2    0x40
#define FLAGBYTE8_LOG_CLR    0x80 // tooth log buffer needs clearing

extern unsigned char flagbyte9;
#define FLAGBYTE9_CAS360_ON     0x01
//#define FLAGBYTE9_RPMDOT_SAMPLE 0x02
#define FLAGBYTE9_CRC_CAN       0x04
#define FLAGBYTE9_SPK4RISEFALL  0x08
#define FLAGBYTE9_CRKPOLCHK     0x10
#define FLAGBYTE9_GETRTC        0x20
#define FLAGBYTE9_EGTADD        0x40
#define FLAGBYTE9_EGTMELT       0x80

extern unsigned char flagbyte10;
#define FLAGBYTE10_LOCKEDFLASH  0x01
#define FLAGBYTE10_TBRAKE       0x02
#define FLAGBYTE10_SDERASEAUTO  0x04
#define FLAGBYTE10_FUELCUTTMP   0x08
#define FLAGBYTE10_SPKHILO      0x10 /* mimics ram4.ICIgnOption bit */
#define FLAGBYTE10_SPKCUTTMP    0x20
#define FLAGBYTE10_TC_N2O       0x40
#define FLAGBYTE10_CAMPOL       0x80

extern unsigned char flagbyte11;
#define FLAGBYTE11_DLI4         0x01 /* from MS2/Extra */
#define FLAGBYTE11_DLI6         0x02
#define FLAGBYTE11_CANRX        0x04
#define FLAGBYTE11_KNOCK        0x08
#define FLAGBYTE11_CAM1         0x10
#define FLAGBYTE11_CAM2         0x20
#define FLAGBYTE11_CAM3         0x40
#define FLAGBYTE11_CAM4         0x80

extern unsigned char flagbyte12;
#define FLAGBYTE12_CAM1ARM      0x01
#define FLAGBYTE12_CAM2ARM      0x02
#define FLAGBYTE12_CAM3ARM      0x04
#define FLAGBYTE12_CAM4ARM      0x08
#define FLAGBYTE12_MAF_FSLOW    0x10
#define FLAGBYTE12_MAP_FSLOW    0x20
#define FLAGBYTE12_WB1          0x40
#define FLAGBYTE12_WB2          0x80

extern unsigned char flagbyte13;
#define FLAGBYTE13_MAF_PT4      0x01
#define FLAGBYTE13_MAF_PT2      0x02
#define FLAGBYTE13_MAF_PT5      0x04
#define FLAGBYTE13_MAF_PT6      0x08
#define FLAGBYTE13_MAP_PT4      0x10
#define FLAGBYTE13_MAP_PT2      0x20
#define FLAGBYTE13_MAP_PT5      0x40
#define FLAGBYTE13_MAP_PT6      0x80

extern unsigned char flagbyte14;
#define FLAGBYTE14_SERIAL_TL      0x01
#define FLAGBYTE14_SERIAL_FWD     0x02
#define FLAGBYTE14_FUELFLOW_CALC  0x04
#define FLAGBYTE14_CP_ERR         0x08
#define FLAGBYTE14_SERIAL_PROCESS 0x10
#define FLAGBYTE14_SERIAL_SEND    0x20
#define FLAGBYTE14_SERIAL_BURN    0x40
#define FLAGBYTE14_SERIAL_OK      0x80

extern unsigned char flagbyte15;
#define FLAGBYTE15_MAP_AVG_RDY  0x01
#define FLAGBYTE15_MAP_AVG_TRIG 0x02
#define FLAGBYTE15_FIRSTRPM     0x04 // got the first rpm calc period
#define FLAGBYTE15_MAPWINDOW    0x08 // in MAP window
#define FLAGBYTE15_LTT64s       0x10 // LTT set every 64s
#define FLAGBYTE15_DPCNT        0x20 // Flag for dashpot decay timer ready
#define FLAGBYTE15_DB_WRAP      0x40 // debug buffer has wrapped
#define FLAGBYTE15_SDLOGRUN     0x80 // SDcard logging should keep running

extern unsigned char flagbyte16;
#define FLAGBYTE16_AC_ENABLE    0x01
#define FLAGBYTE16_AC_TPSHYST   0x02
#define FLAGBYTE16_AC_VSSHYST   0x04
#define FLAGBYTE16_FAN_ENABLE   0x08
#define FLAGBYTE16_FAN_TPSHYST  0x10
#define FLAGBYTE16_FAN_VSSHYST  0x20
#define FLAGBYTE16_CHKSENS      0x40
#define FLAGBYTE16_VVT_TIMED    0x80

extern unsigned char flagbyte17;
#define FLAGBYTE17_OVERRUNFC    0x01
#define FLAGBYTE17_REVLIMFC     0x02

extern unsigned long stall_timeout;     // save doing long div each 0.128ms
extern unsigned char last_fsensdat;
extern unsigned int FPdcounter, ff_pw;
extern unsigned char fc_counter, adc_ctr;
extern unsigned int lowres_ctr, tacho_targ;     // 0.128ms period counters (like MS1) for tacho
//spark cut
extern unsigned char spk_cutx, spk_cuty, spk_cuti, fuel_cutx, fuel_cuty, fuel_cuti;
extern unsigned char staged_num_events; /* number of events into staging trasition */
extern unsigned long pw_staged1, pw_staged2;    /* staged pulsewidths */
//extern unsigned int spk_mult; // save doing some long divs in ISR
//tooth / trigger logger
extern unsigned int log_offset;
//dwell exec software 0.128ms timers
extern unsigned char dwl[NUM_TRIGS], maxdwl, testcnt, rdwl[8];
extern unsigned int swtimer, swtimer_TC5_last, swtimer_inj, rtsci;
extern unsigned int deg_per_tooth[MAXNUMTEETH];    // deg*10 AHEAD of tooth-1 i.e. runs 0->, teeth are 1->
extern unsigned int tooth_absang[MAXNUMTEETH];
extern unsigned char pwmd1, pwmd2, trig2cnt, trig2cnt_last, bl_timer,
    n2o_act_timer, n2o2_act_timer;
extern unsigned long injtime, injtime_EAElagcomp, ultmp, ultmp2;
extern unsigned int mapsample_time, tpssample_time;     //.128 ms ticks
extern unsigned char running_seconds;
extern unsigned char fc_off_time;
extern unsigned long tmp_pw1, tmp_pw2, ticks_per_deg;
extern unsigned char tooth_init, tooth_counter, tooth_counter_main;
#define WHEEL_NUM_TEETH 20      // minimal to allow all teeth to be grabbed in mainloop exec time
extern ign_time act_tooth_time[WHEEL_NUM_TEETH];
extern unsigned char act_tooth_num[WHEEL_NUM_TEETH];

extern unsigned char fuel_cntr, firstsync_iter, set_count,
    EAElagcomp_squirts, EAElag_squirting;
extern unsigned char channel_squirted, squirtcount1, squirtcount2;
extern unsigned long dtpred_adder;
extern unsigned char syncerr;
extern unsigned int smallest_tooth_crk, smallest_tooth_cam, false_mask_crk,
    false_mask_cam, false_period_crk_tix, false_period_cam_tix;
extern int cycle_deg;
extern unsigned char injbits;
extern unsigned int TC0_last, TC_trig2_last, TC_trig2_last2, TC5_last,
    TIMTCNT_last;
extern unsigned long TC5_32bits, TC5_trig_firstedge;
extern unsigned char idle_advance_timer, idle_ve_timer;
extern unsigned long IC_last XGATE_ALIGN_ATTR;   // for triglog2
extern int gl_afrtgt1, gl_afrtgt2;
extern unsigned char num_cyl, divider;
extern unsigned int burnstat;
extern unsigned char tst_tmp;
extern unsigned char buf2[64];

// ignition definitions to swap timer interrupts TC2/TC5 if JS10 vs MS3X cam input
extern unsigned char TFLG_trig2;

//SD card related headers
extern unsigned long sd_int_sector;
extern unsigned char gu8SD_CID[16];

extern unsigned char vSD_CSD[16];
extern unsigned long u32MaxBlocks;
extern unsigned int sd_sectsize, sd_max_roots, sd_filenum, sd_filenum_low;
extern unsigned long sd_tot_sect, sd_thisfile_start, sd_filesize_bytes,
    sd_maxlogblocks;
extern unsigned long sd_part_start, sd_part_end, sd_fat_start,
    sd_dir_start, sd_file_start, sd_pos, sd_block, sd_saved_dir;
extern unsigned char sd_sect2clust, sd_nofats, sd_ledstat;
extern unsigned char sd_phase, sd_int_cmd, /*sd_error,*/ sd_int_error,
    sd_match, sd_match_mask, sd_rx, sd_seq, sd_retry;
extern volatile unsigned char sd_int_phase;
extern unsigned int sd_lmms_int, sd_subblock, sd_blsect, sd_saved_offset;
extern unsigned long sd_lmms_last, sd_lmms_last2, sd_fatsize,
    sd_thisfile_clust, sd_file_numclust, sd_clust_cnt, sd_ultmp;
extern unsigned int sd_int_addr, sd_int_cnt, sd_log_addr, sd_uitmp,
    sd_uitmp2, sd_ledclk;
extern unsigned int sd_stream_ad, sd_stream_avg, sd_rb_size, sd_rb_block;
extern unsigned char sd_stream_cnt, sd_magic;
extern unsigned long sd_crc;
extern unsigned int sd_timeout;

extern unsigned int cum_cycle, cum_tooth;
#define SD_RPAGE 0xf1
/* indented to hide from ASM */
 #define SS_ENABLE PTH &= ~0x08
 #define SS_DISABLE PTH |= 0x08

extern unsigned char num_inj_outs, num_inj_events, orig_divider, orig_alternate;
extern unsigned int ReqFuel, orig_ReqFuel;
extern unsigned long sync_loss_stamp, sync_loss_time;
extern unsigned int launch_timer, maxafr_timer, tb_timer;
extern unsigned char maxafr_stat;
extern unsigned int pit_16bits, vss1_last, vss1_time,  vss2_last, vss2_time,
 ss1_last, ss1_time, ss2_last, ss2_time,
 vss1_stall, vss2_stall, ss1_stall, ss2_stall, vss1_time_sum, vss2_time_sum, vss1_teeth, vss2_teeth;
extern unsigned long vss1_coeff, vss2_coeff, ss1_coeff, ss2_coeff;

#define VSS_STALL_TIMEOUT   10
#define HARDWARE_LEDSPK     0x01
#define HARDWARE_MS3XSPK    0x02
#define HARDWARE_TACHOSPK   0x04
#define HARDWARE_MS3XFUEL   0x10
#define HARDWARE_CAM   0x40 // changed from 0x08

//Port/pin definitions
extern volatile unsigned short *port_stream, *port_launch_var, *mapport, *port_map2,
    *port_egt[12], *mafport, *port_sensor[16], *port_knock_in, *port_tc_knob,
    *port_gearsel, *baroport, *egoport[8], *accXport, *accYport, *accZport;
extern volatile unsigned char *port_tacho, *port_idleonoff, *port_sdled,
    *port_sdbut, *port_launch, *port_maxafr, *port_n2oin, *port_n2o1n,
    *port_n2o1f, *port_n2o2n, *port_n2o2f, *port_tsw_rf, *port_tsw_afr,
    *port_tsw_stoich, *port_boost_tsw, *port_tsf, *port_tss, *port_3step,
    *port_idlepwm, *port_idlepwm3, *port_vss1, *port_vss2, *port_ss1,
    *port_ss2, *port_wipump, *port_wivalve, *port_wiin, *port_vssout,
    *port_ac_out, *port_ac_in, *port_fanctl_out, *port_shift_cut_in,
    *port_shift_cut_out, *port_pwm[6], *port_dualfuel, *port_tsw_ob,
    *port_knk, *port_alsin, *port_alsout, *boostport, *port_boost2,
    *port_tcluen, *port_tclubr, *port_tcluout, *port_tcenin, *port_vvt[4],
    *port_fp, *port_knock_out, *port_ltt_but, *port_ltt_led, *port_timed1_in,
    *port_timed1_out, *port_tstop, *port_sci, *port_testio, *port_cel,
    *port_spki XGATE_ALIGN_ATTR, *port_spkj XGATE_ALIGN_ATTR;
extern unsigned char pin_tacho, pin_idleonoff, pin_sdled, pin_sdbut,
    pin_launch, pin_maxafr, pin_n2oin,
    pin_match_n2oin, pin_n2o1n, pin_n2o1f, pin_n2o2n, pin_n2o2f,
    pin_tsw_rf, pin_tsw_afr, pin_tsw_stoich, pin_boost_tsw, pin_tsf,
    pin_tss, pin_3step, pin_idlepwm, pin_idlepwm3, pin_vss1, pin_vss2,
    pin_ss1, pin_ss2, pin_wipump, pin_wivalve, pin_wiin, pin_match_wiin, pin_vssout,
    pin_ac_out, pin_ac_in, pin_fanctl_out, pin_shift_cut_in,
    pin_shift_cut_match, pin_shift_cut_out, pin_pwm[6], pin_dualfuel, pin_tsw_ob,
    pin_knk, pin_knk_match, pin_match_launch, pin_match_sdbut, pin_match_ac_in,
    pin_alsin, pin_match_alsin, pin_alsout, boostpin, pin_boost2, pin_tcluen, pin_tclubr, pin_tcluout,
    pin_tcenin, pin_fp, pin_knock_out, pin_flex, pin_xgcam, pin_ltt_but, pin_ltt_led,
    pin_timed1_in, pin_timed1_out, pin_tstop, pin_testio, pin_cel,
    pin_spki, pin_spkj;

extern unsigned long XTX, XTm, XTf, Tau, sum_dltau[2];
extern long XTM[2];

extern unsigned char nitrous_pwm_clock, mmsDivn2o, water_pwm_clock,
    mmsDivwi;
extern unsigned int nitrous_timer, water_pw, water_pw_cnt;
extern unsigned int mode10_time, srl_timeout;

extern unsigned int dwell_us XGATE_ALIGN_ATTR, dwell_us2 XGATE_ALIGN_ATTR, dizzy_scaler[NUM_TRIGS]  XGATE_ALIGN_ATTR, trigret_scaler XGATE_ALIGN_ATTR;
extern unsigned long dwell_long;
extern unsigned int vssout_cnt, vssout_match, vss_time, accxyz_time, sens_time;
extern unsigned char spkmode;
extern unsigned char fuel_trim_cnt, spk_trim_cnt;
extern char fuel_trim[NUM_TRIGS], spk_trim[NUM_TRIGS];
extern unsigned int outvss1_last, outvss2_last, gear_scale;
extern unsigned char vss_cnt;
extern unsigned int cas_tooth  XGATE_ALIGN_ATTR, cas_tooth_div XGATE_ALIGN_ATTR;
extern unsigned int xg_debug XGATE_ALIGN_ATTR;
extern unsigned char xgate_deadman, xgswe_count;
extern unsigned int xgpc_copy;
extern unsigned char ls1_ls, ls1_sl, ls1_ls_last, ls1_sl_last, edge2_last;
//need these as real globals (were in ign_in)
extern unsigned long tooth_time1, tooth_time2;
extern unsigned int swtimer_last, swtimer_inj_last;
extern unsigned long NoiseFilter1;
extern unsigned int TC0this, TIMTCNT_this;
extern unsigned long temp1, TC0_32bits, TIMTCNT_32bits, TC_crank;
extern unsigned char last_edge, last_edge2;
#define MIN_VOLTS 222           // equiv to 6.5V

#define MAT_N 10
extern unsigned int mat_data[MAT_N];
#define CLT_N 10
extern unsigned int clt_data[CLT_N];
//#define EGO_N 10
//extern unsigned int ego1_data[EGO_N];
//extern unsigned int ego2_data[EGO_N];
#define TPS_N 3
extern unsigned int tps_data[TPS_N];
#define MAP_N 3
extern unsigned int map_data[MAP_N];
#define MAF_N 3
extern unsigned int maf_data[MAF_N];
#define BATT_N 10
extern unsigned int batt_data[BATT_N];
extern unsigned int tps_ring[8];
extern unsigned char tps_ring_cnt;
extern unsigned int inj_cnt[NUM_TRIGS] XGATE_ALIGN_ATTR;
extern unsigned char shift_cut_phase, shift_cut_timer, inj_chan[NUM_TRIGS], timer_usage, timerpool_flag[NUM_TRIGS];
extern unsigned char vvt_run;
extern unsigned char InjPWMTim1, InjPWMTim2, InjPWMPd1, InjPWMPd2, InjPWMDty1, InjPWMDty2;
extern int flash_adv_offset, flash_Miss_ang;
extern unsigned char do_dualouts;
extern unsigned int egt_timer, tc_addfuel, perfect_timer, sliptimer;
extern unsigned char tc_nitrous, tc_boost;
char tc_boost_duty_delta;
extern unsigned char i2cstate, i2cstate2, i2csubstate, i2csubbyte, i2caddr, i2cbyte;
extern int als_timing, als_addfuel, als_iacstep;
extern unsigned char als_timer, als_state;
extern unsigned int timerpool[NUM_TRIGS] XGATE_ALIGN_ATTR;
extern unsigned char cam_tooth[4], cam_tooth_sample[4];
extern unsigned long cam_time[4], cam_time_sample[4], crank_time_sample[4], cam_last_tooth, cam_last_tooth_1;
extern unsigned char vvt_intx;
extern int vvt_inj_timing_adj;
extern unsigned char tclu_state, tclu_timer;
extern int tps0_auto, tps0_orig;

/* 'user defined'                                                           *  
 * Here are some variables defined to help the new programmer get started   *
 * search for 'user defined' in ms3_main_decls.h for more notes             *
 *                                                                          */
extern unsigned long user_ulong;
extern unsigned int user_uint;
extern unsigned char user_uchar;
/* end user defined section                                                 */

extern unsigned int knocksample_time;     //.128 ms ticks
extern unsigned int knock_start_countdown, knock_window_countdown,
    knock_window_set, knock_temp;
extern map_event next_knock_start_event, knock_start_event[NUM_TRIGS];
extern unsigned int mafperiod_accum, mapperiod_accum; /* freq based MAF/MAP */
extern unsigned long maf_tim_l, map_tim_l;
extern unsigned char tim_mask, tim_mask_run;
extern unsigned char knock_chan_tmp, knock_chan_sample;
extern unsigned char knock_state, knock_chan, knock_gain, knock_retry;
extern unsigned int knock_res, knockin;
extern unsigned int rawpw[18];
extern unsigned long flowsum[2], flowsum_accum[2];
extern unsigned char injch[18];
extern unsigned char fly_div;
extern unsigned int xg_teeth XGATE_ALIGN_ATTR;
extern unsigned int xgcamTC XGATE_ALIGN_ATTR, xgcrkTC XGATE_ALIGN_ATTR, xgcam_last XGATE_ALIGN_ATTR;
extern unsigned char spi2_state, spiwb_byte1, spiwb_byte2, spiwb_errcnt1, spiwb_errcnt2;
extern unsigned int spiwb_timer1, spiwb_timer2;
extern unsigned char spk_crk_targ, dwl_crk_targ;
extern unsigned char ltt_but_state, ltt_fl_state, ltt_timer;
extern unsigned int ltt_but_debounce, ltt_fl_ad;
extern unsigned int db_ptr;
extern unsigned int can_bcast_last;
extern unsigned char sci_lock_timer;
extern unsigned char maplog_cnt, maplog_max;
extern unsigned char testmode_glob, iactest_glob, mmsDiv_testio, clk_testio;
extern unsigned int testmode_cnt;
extern unsigned char nextcyl_cnt;

extern unsigned char stat_map, stat_mat, stat_clt, stat_tps, stat_afr0, stat_batt, stat_sync, stat_flex, stat_maf, stat_knock;
extern unsigned int mapadc_thresh;
#define NUM_EGT_CHECK 16
extern unsigned char stat_egt[NUM_EGT_CHECK];

/* these were locally defined globals in ms3_idle.c */
extern long PV_last[2];
extern unsigned int pwmidle_targ_stepsize, pwmidle_targ_numsteps, pwmidle_targ_last, targ_rpm;
extern unsigned char pwmidle_shift_timer;
extern char valve_closed, last_direction, ac_idleup_adder, last_acbutton_state, ac_idleup_mapadder;
extern char ac_idleup_mapadder_last, fan_idleup_adder, last_fan_state;
extern int fan_idleup_cl_targetadder, idle_voltage_comp, ac_idleup_cl_targetadder, IACmotor_100;
extern int IACmotor_pos_tmp, pwmidle_stepsize, pwmidle_numsteps;
extern unsigned char ac_time_since_last_on;

/* these were locally defined globals in ms3_misc.c */
extern long boost_ctl_last_pv[NUM_BOOST_CHANS][2];
extern long boost_ctl_last_error[NUM_BOOST_CHANS];
extern char boost_PID_enabled[NUM_BOOST_CHANS];
extern int boost_ctl_duty_100[NUM_BOOST_CHANS];

extern long vvt_last_run[4], vvt_ctl_last_pv[4][2], vvt_ctl_last_error[4];
extern unsigned int vvt_timer;
extern unsigned char vvt_PID_enabled;
extern int vvt_duty_store[4];

/* these were locally defined globals in ms3_ign.c */
extern unsigned long dtpred_adder;

/* other vars follow */
extern unsigned int gp_clk[6], gp_max_on[6], gp_max_off[6];
extern unsigned char gp_stat[6];
