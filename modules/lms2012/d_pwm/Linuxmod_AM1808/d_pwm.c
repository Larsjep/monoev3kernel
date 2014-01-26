/*
 * LEGO® MINDSTORMS EV3
 *
 * Copyright (C) 2010-2013 The LEGO Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*! \page PwmModule PWM Module
 *
 */

#define   HW_ID_SUPPORT

#include  "../../lms2012/source/lms2012.h"
#include  "../../lms2012/source/am1808.h"

int       Hw         =  0;
int       HwInvBits  =  0;

#define   MODULE_NAME                   "pwm_module"
#define   DEVICE1_NAME                  PWM_DEVICE
#define   DEVICE2_NAME                  MOTOR_DEVICE


#define   SOFT_TIMER_MS                 4
#define   SOFT_TIMER_SETUP              (SOFT_TIMER_MS * 1000000)

/*
 *  NO_OF_TACHO_SAMPLES holds the number of recent tacho samples
 */
#define   NO_OF_TACHO_SAMPLES           128
#define   NO_OF_OUTPUT_PORTS            4

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   SPEED_PWMCNT_REL              (100) //(MAX_PWM_CNT/MAX_SPEED)


#define   COUNTS_PER_PULSE_LM           12800L
#define   COUNTS_PER_PULSE_MM           8100L


/*
 * Defines related to hardware PWM output
 */
// TBCTL (Time-Base Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
#define   TB_COUNT_UP                   0x0     // TBCNT MODE bits
#define   TB_COUNT_FREEZE               0x3     // TBCNT MODE bits
#define   TB_DISABLE                    0x0     // PHSEN bit
#define   TB_ENABLE                     0x4
#define   TB_SHADOW                     0x0     // PRDLD bit
#define   TB_IMMEDIATE                  0x8
#define   TB_SYNC_DISABLE               0x30    // SYNCOSEL bits
#define   TB_HDIV1                      0x0     // HSPCLKDIV bits
#define   TB_DIV1                       0x0     // CLKDIV bits
#define   TB_UP                         0x2000  // PHSDIR bit

// CMPCTL (Compare Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
#define   CC_CTR_A_ZERO                 0x0     // LOADAMODE bits
#define   CC_CTR_B_ZERO                 0x0
#define   CC_A_SHADOW                   0x00    // SHDWAMODE and SHDWBMODE bits
#define   CC_B_SHADOW                   0x00

#define   TBCTL                         0x0
#define   TBPHS                         0x3
#define   TBCNT                         0x4
#define   TBPRD                         0x5
#define   CMPCTL                        0x7
#define   CMPA                          0x9
#define   CMPB                          0xA
#define   AQCTLA                        0xB
#define   AQCTLB                        0xC


/* SYS config register configuration and bits*/
enum
{
  CFGCHIP1    =  0x60,
};

enum
{
  TBCLKSYNC   =  0x00001000,
};


/* TIMER64 register configuration */
enum
{
  REVID       = 0,
  EMUMGT      = 1,
  GPINTGPEN   = 2,
  GPDATGPDIR  = 3,
  TIM12       = 4,
  TIM34       = 5,
  PRD12       = 6,
  PRD34       = 7,
  TCR         = 8,
  TGCR        = 9,
  WDTCR       = 10,
  NOTUSED1    = 11,
  NOTUSED2    = 12,
  REL12       = 13,
  REL34       = 14,
  CAP12       = 15,
  CAP34       = 16,
  NOTUSED3    = 17,
  NOTUSED4    = 18,
  NOTUSED5    = 19,
  NOTUSED6    = 20,
  NOTUSED7    = 21,
  NOTUSED8    = 22,
  INTCTLSTAT  = 23,
  CMP0        = 24,
  CMP1        = 25,
  CMP2        = 26,
  CMP3        = 27,
  CMP4        = 28,
  CMP5        = 39,
  CMP6        = 30,
  CMP7        = 31,
};

/* eCAP Register configuration */
enum
{
  TSCTR       =  0,
  CTRPHS      =  2,
  CAP1        =  4,
  CAP2        =  6,
  CAP3        =  8,
  CAP4        = 10,
  ECCTL1      = 20,
  ECCTL2      = 21,
  ECEINT      = 22,
  ECFLG       = 23,
  ECCLR       = 24,
  ECFRC       = 25,
  ECAP_REVID  = 46,
};


#define  NON_INV   1
#define  INV      -1


enum
{
  FORWARD,
  BACKWARD,
  BRAKE,
  COAST,
};

enum
{
  UNLIMITED_UNREG,
  UNLIMITED_REG,
  LIMITED_REG_STEPUP,
  LIMITED_REG_STEPCONST,
  LIMITED_REG_STEPDOWN,
  LIMITED_UNREG_STEPUP,
  LIMITED_UNREG_STEPCONST,
  LIMITED_UNREG_STEPDOWN,
  LIMITED_REG_TIMEUP,
  LIMITED_REG_TIMECONST,
  LIMITED_REG_TIMEDOWN,
  LIMITED_UNREG_TIMEUP,
  LIMITED_UNREG_TIMECONST,
  LIMITED_UNREG_TIMEDOWN,
  LIMITED_STEP_SYNC,
  LIMITED_TURN_SYNC,
  LIMITED_DIFF_TURN_SYNC,
  SYNCED_SLAVE,
  RAMP_DOWN_SYNC,
  HOLD,
  BRAKED,
  STOP_MOTOR,
  IDLE,
};

enum
{
    ST_IDLE,
    ST_STALL,
    ST_HOLD,
    ST_START,
    ST_ACCEL,
    ST_MOVE,
    ST_DECEL
};


typedef struct
{
    SLONG baseCnt;
    SLONG curCnt;
    SLONG curVelocity;
    SLONG tachoCnt;
    SLONG state;
    SLONG time;
    SLONG time2;
    SLONG serial;
} MOTORSHARED;

typedef   struct
{
  MOTORSHARED *shared;
  SLONG   IrqTacho;
  SLONG   TachoCnt;
  SLONG   TimeCnt;
  SLONG   TimeInc;
  SLONG   OldTachoCnt;
  SLONG   Power;
  SLONG   moveP;
  SLONG   moveI;
  SLONG   moveD;
  SLONG   holdP;
  SLONG   holdI;
  SLONG   holdD;
  SLONG   tachoCnt;
  SLONG   baseTime;
  SLONG   curCnt;
  SLONG   curVelocity;
  SLONG   baseCnt;
  SLONG   basePower;
  SLONG   err1;
  SLONG   err2;
  SLONG   stallCnt;
  SLONG   stallLimit;
  SLONG   stallTime;
  SLONG   deadBand;
  SLONG   offset;

  SLONG   mT1;
  SLONG   mT2;
  SLONG   mT3;
  SLONG   mC1;
  SLONG   mC2;
  SLONG   mC3;
  SLONG   mV1;
  SLONG   mV2;
  SLONG   mA1;
  SLONG   mA3;

  UBYTE   no;
  UBYTE   moving;
  UBYTE   curHold;
  UBYTE   Type;
  UBYTE   State;
  UBYTE   Mutex;
}MOTOR;




static    int  ModuleInit(void);
static    void ModuleExit(void);

static    irqreturn_t IntA (int irq, void * dev);
static    irqreturn_t IntB (int irq, void * dev);
static    irqreturn_t IntC (int irq, void * dev);
static    irqreturn_t IntD (int irq, void * dev);

void      SetGpioRisingIrq(UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *));


void      SetDutyMA(ULONG Duty);
void      SetDutyMB(ULONG Duty);
void      SetDutyMC(ULONG Duty);
void      SetDutyMD(ULONG Duty);

#include  <linux/kernel.h>
#include  <linux/fs.h>
#include  <linux/signal.h>
#include  <linux/sched.h>

#ifndef   PCASM
#include  <linux/ioport.h>
#include  <asm/gpio.h>
#include  <asm/uaccess.h>
#include  <linux/module.h>
#include  <linux/miscdevice.h>

#include  <linux/mm.h>
#include  <linux/hrtimer.h>

#include  <linux/init.h>
#include  <asm/siginfo.h>     //siginfo
#include  <linux/rcupdate.h>  //rcu_read_lock
#include  <linux/uaccess.h>
#include  <linux/debugfs.h>

#include  <asm/io.h>
#include  <asm/uaccess.h>

#include  <linux/irq.h>
#include  <linux/interrupt.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The LEGO Group");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_SUPPORTED_DEVICE(DEVICE1_NAME);

module_init(ModuleInit);
module_exit(ModuleExit);

#include  <mach/mux.h>

#else
// Keep Eclipse happy
#endif


enum      OutputPortPins
{
  PWM,
  DIR0,
  DIR1,
  INT,
  DIR,
  SLEEP,
  FAULT,
  OUTPUT_PORT_PINS
};


#define   IRQA_PINNO                  ((pOutputPortPin[Hw])[(0 * OUTPUT_PORT_PINS) + INT].Pin)
#define   IRQB_PINNO                  ((pOutputPortPin[Hw])[(1 * OUTPUT_PORT_PINS) + INT].Pin)
#define   IRQC_PINNO                  ((pOutputPortPin[Hw])[(2 * OUTPUT_PORT_PINS) + INT].Pin)
#define   IRQD_PINNO                  ((pOutputPortPin[Hw])[(3 * OUTPUT_PORT_PINS) + INT].Pin)


/* EP2 hardware have sanyo motor driver */
INPIN     EP2_OutputPortPin[][OUTPUT_PORT_PINS] =
{
  { // Output port A
    { EPWM1B , NULL, 0 }, // PWM motor A (GPIO 2_14)
    { GP3_15 , NULL, 0 }, // DIR0 A
    { GP3_6  , NULL, 0 }, // DIR1 A
    { GP5_11 , NULL, 0 }, // INT  A
    { GP0_4  , NULL, 0 }, // DIR  A
    { -1     , NULL, 0 }, // Sleep AB
    { -1     , NULL, 0 }, // Fault AB
  },
  { // Output port B
    { EPWM1A , NULL, 0 }, // PWM motor B (GPIO 2_15)
    { GP2_1  , NULL, 0 }, // DIR0 B
    { GP0_3  , NULL, 0 }, // DIR1 B
    { GP5_8  , NULL, 0 }, // INT  B
    { GP2_9  , NULL, 0 }, // DIR  B
    { -1     , NULL, 0 }, // Sleep AB
    { -1     , NULL, 0 }, // Fault AB
  },
  { // Output port C
    { APWM0  , NULL, 0 }, // PWM motor C (GPIO 8_7)
    { GP6_8  , NULL, 0 }, // DIR0 C
    { GP5_9  , NULL, 0 }, // DIR1 C
    { GP5_13 , NULL, 0 }, // INT  C
    { GP3_14 , NULL, 0 }, // DIR  C
    { -1     , NULL, 0 }, // Sleep AB
    { -1     , NULL, 0 }, // Fault AB
  },
  { // Output port D
    { APWM1  , NULL, 0 }, // PWM MOTOR D (GPIO 0_0)
    { GP5_3  , NULL, 0 }, // DIR0 D
    { GP5_10 , NULL, 0 }, // DIR1 D
    { GP6_9  , NULL, 0 }, // INT  D
    { GP2_8  , NULL, 0 }, // DIR  D
    { -1     , NULL, 0 }, // Sleep AB
    { -1     , NULL, 0 }, // Fault AB
  },
};


/* FINALB have TI motor driver */
INPIN     FINALB_OutputPortPin[][OUTPUT_PORT_PINS] =
{
  { // Output port A
    { EPWM1A , NULL, 0 }, // PWM motor A (GPIO 2_15)
    { GP0_3  , NULL, 0 }, // DIR0 A
    { GP4_12 , NULL, 0 }, // DIR1 A
    { GP5_11 , NULL, 0 }, // INT  A
    { GP0_4  , NULL, 0 }, // DIR  A
    { GP3_10 , NULL, 0 }, // Sleep AB
    { GP2_0  , NULL, 0 }, // Fault AB
  },
  { // Output port B
    { EPWM1B , NULL, 0 }, // PWM motor B (GPIO 2_14)
    { GP3_15 , NULL, 0 }, // DIR0 B
    { GP3_6  , NULL, 0 }, // DIR1 B
    { GP5_8  , NULL, 0 }, // INT  B
    { GP2_9  , NULL, 0 }, // DIR  B
    { GP3_10 , NULL, 0 }, // Sleep AB - Same as above
    { GP2_0  , NULL, 0 }, // Fault AB - Same as above
  },
  { // Output port C
    { APWM1  , NULL, 0 }, // PWM motor C (GPIO 0_0)
    { GP5_10 , NULL, 0 }, // DIR0 C
    { GP5_3  , NULL, 0 }, // DIR1 C
    { GP5_13 , NULL, 0 }, // INT  C
    { GP3_14 , NULL, 0 }, // DIR  C
    { GP2_3  , NULL, 0 }, // Sleep CD
    { GP6_0  , NULL, 0 }, // Fault CD
  },
  { // Output port D
    { APWM0  , NULL, 0 }, // PWM MOTOR D (GPIO 8_7)
    { GP6_8  , NULL, 0 }, // DIR0 D
    { GP5_9  , NULL, 0 }, // DIR1 D
    { GP6_9  , NULL, 0 }, // INT  D
    { GP2_8  , NULL, 0 }, // DIR  D
    { GP2_3  , NULL, 0 }, // Sleep CD - Same as above
    { GP6_0  , NULL, 0 }, // Fault CD - Same as above
  },
};


INPIN     FINAL_OutputPortPin[][OUTPUT_PORT_PINS] =
{
  { // Output port A
    { EPWM1A , NULL, 0 }, // PWM motor A (GPIO 2_15)
    { GP0_3  , NULL, 0 }, // DIR0 A
    { GP4_12 , NULL, 0 }, // DIR1 A
    { GP5_11 , NULL, 0 }, // INT  A
    { GP0_4  , NULL, 0 }, // DIR  A
    { GP3_10 , NULL, 0 }, // Sleep AB
    { GP2_0  , NULL, 0 }, // Fault AB
  },
  { // Output port B
    { EPWM1B , NULL, 0 }, // PWM motor B (GPIO 2_14)
    { GP3_15 , NULL, 0 }, // DIR0 B
    { GP3_6  , NULL, 0 }, // DIR1 B
    { GP5_8  , NULL, 0 }, // INT  B
    { GP2_9  , NULL, 0 }, // DIR  B
    { GP3_10 , NULL, 0 }, // Sleep AB - Same as above
    { GP2_0  , NULL, 0 }, // Fault AB - Same as above
  },
  { // Output port C
    { APWM1  , NULL, 0 }, // PWM motor C (GPIO 0_0)
    { GP5_10 , NULL, 0 }, // DIR0 C
    { GP5_3  , NULL, 0 }, // DIR1 C
    { GP5_13 , NULL, 0 }, // INT  C
    { GP3_14 , NULL, 0 }, // DIR  C
    { GP2_3  , NULL, 0 }, // Sleep CD
    { GP6_0  , NULL, 0 }, // Fault CD
  },
  { // Output port D
    { APWM0  , NULL, 0 }, // PWM MOTOR D (GPIO 8_7)
    { GP6_8  , NULL, 0 }, // DIR0 D
    { GP5_9  , NULL, 0 }, // DIR1 D
    { GP6_9  , NULL, 0 }, // INT  D
    { GP2_8  , NULL, 0 }, // DIR  D
    { GP2_3  , NULL, 0 }, // Sleep CD - Same as above
    { GP6_0  , NULL, 0 }, // Fault CD - Same as above
  },
};


INPIN     *pOutputPortPin[] =
{
  [FINAL]     =   (INPIN*)&FINAL_OutputPortPin[0],    //  FINAL   platform
  [FINALB]    =   (INPIN*)&FINALB_OutputPortPin[0],   //  FINALB  platform
  [EP2]       =   (INPIN*)&EP2_OutputPortPin[0],      //  EP2     platform
};


#define   OutputReadDir(port,pin)       ((HwInvBits ^ (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).in_data)  &  pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].Mask)
#define   READDirA                      OutputReadDir(0,DIR)
#define   READDirB                      OutputReadDir(1,DIR)
#define   READDirC                      OutputReadDir(2,DIR)
#define   READDirD                      OutputReadDir(3,DIR)

/*
 * Variables
 */
static    ULONG   *GPIO;
static    ULONG   *SYSCFG0;
static    ULONG   *SYSCFG1;
static    ULONG   *PLLC1;
static    ULONG   *PSC1;
static    UWORD   *eHRPWM1;
static    UWORD   *eCAP0;
static    UWORD   *eCAP1;
static    ULONG   *TIMER64P3;

static    MOTOR   Motor[NO_OF_OUTPUT_PORTS];

static    void    (*SetDuty[NO_OF_OUTPUT_PORTS])(ULONG)  = {SetDutyMA,SetDutyMB,SetDutyMC,SetDutyMD};

static    UBYTE           ReadyStatus = 0;
static    UBYTE           TestStatus  = 0;

static    MOTORSHARED     MotorData[NO_OF_OUTPUT_PORTS];
static    MOTORSHARED     *pMotor = MotorData;



static    struct hrtimer  Device1Timer;
static    ktime_t         Device1Time;


/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 */
/*! \brief    Number of average samples for each motor
 *
 *  - Medium motor = 1, 2,  4,  8
 *  - Large motor  = 2, 8, 16, 32
 *
 *  Medium motor has not the jitter on the tacho when as large motor because
 *  it has one gear wheel less that the large motor.
 *
 *  Medium motor reaction time is much faster than large motor due to smaller motor
 *  and smaller gearbox
 */



/*
 * Macros
 */
#define   FLOATFaultPins                {\
                                          if (((pOutputPortPin[Hw])[(0 * OUTPUT_PORT_PINS) + FAULT].Pin) != -1)\
                                          {\
                                            (*pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + FAULT].pGpio).dir |=  pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + FAULT].Mask;\
                                          }\
                                          if (((pOutputPortPin[Hw])[(2 * OUTPUT_PORT_PINS) + FAULT].Pin) != -1)\
                                          {\
                                            (*pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + FAULT].pGpio).dir |=  pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + FAULT].Mask;\
                                          }\
                                        }

#define   SETSleepPins                  {\
                                          if (((pOutputPortPin[Hw])[(0 * OUTPUT_PORT_PINS) + SLEEP].Pin) != -1)\
                                          {\
                                            (*pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + SLEEP].pGpio).set_data  =  pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + SLEEP].Mask;\
                                            (*pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + SLEEP].pGpio).dir      &= ~pOutputPortPin[Hw][(0 * OUTPUT_PORT_PINS) + SLEEP].Mask;\
                                          }\
                                          if (((pOutputPortPin[Hw])[(2 * OUTPUT_PORT_PINS) + SLEEP].Pin) != -1)\
                                          {\
                                            (*pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + SLEEP].pGpio).set_data  =  pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + SLEEP].Mask;\
                                            (*pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + SLEEP].pGpio).dir      &= ~pOutputPortPin[Hw][(2 * OUTPUT_PORT_PINS) + SLEEP].Mask;\
                                          }\
                                        }

#define   SETMotorType(Port, NewType)   {\
                                          Motor[Port].Type = NewType;\
                                      }


#define   FREERunning24bittimer         ((ULONG)((((ULONG*)(TIMER64P3))[TIM34])>>8))


#define   OutputFloat(port,pin)         {\
                                          (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).dir |=  pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].Mask;\
                                        }

#define   OutputRead(port,pin)          ((*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).in_data & pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].Mask)

#define   OutputHigh(port,pin)          {\
                                          (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).set_data  =  pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].Mask;\
                                          (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).dir      &= ~pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].Mask;\
                                        }

#define   OutputLow(port,pin)           {\
                                          (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).clr_data  =  pOutputPortPin[Hw][(OUTPUT_PORT_PINS * port) + pin].Mask;\
                                          (*pOutputPortPin[Hw][(port * OUTPUT_PORT_PINS) + pin].pGpio).dir      &= ~pOutputPortPin[Hw][(OUTPUT_PORT_PINS * port) + pin].Mask;\
                                        }


#define   EHRPWMClkDis                  {\
                                          eHRPWM1[TBCTL]  = (TB_UP | TB_DISABLE | TB_SHADOW | TB_SYNC_DISABLE | TB_HDIV1 | TB_DIV1 | TB_COUNT_FREEZE);\
                                        }

#define   EHRPWMClkEna                  {\
                                          eHRPWM1[TBCTL]  = (TB_UP | TB_DISABLE | TB_SHADOW | TB_SYNC_DISABLE | TB_HDIV1 | TB_DIV1 | TB_COUNT_UP);\
                                          REGUnlock;\
                                          iowrite32((ioread32(&SYSCFG0[CFGCHIP1]) | TBCLKSYNC),&SYSCFG0[CFGCHIP1]); /*This is the clock to all eHRPWM's*/\
                                          REGLock;\
                                        }

#define   STOPPwm                       {\
                                          iowrite16(0x00, &eCAP0[0x15]);\
                                          iowrite16(0x00, &eCAP1[0x15]);\
                                          TIMER64P3[TGCR]  = 0x00000000;\
                                          iowrite16(0x00, &eHRPWM1[TBCTL]);\
                                          iowrite16(0x00, &eHRPWM1[CMPCTL]);\
                                          EHRPWMClkDis;\
                                        }

#define   SETPwmFreqKHz(KHz)            {\
                                          eHRPWM1[TBPRD] = KHz; /* For Motor A and Motor B */\
                                          eCAP0[CAP1]    = KHz; /* For Motor C             */\
                                          eCAP1[CAP1]    = KHz; /* For Motor D             */\
                                        }

#define   SETUPPwmModules               { \
                                          \
                                          /* eHRPWM Module */\
                                          EHRPWMClkDis;\
                                          eHRPWM1[TBPHS]  = 0;\
                                          eHRPWM1[TBCNT]  = 0;\
                                          eHRPWM1[CMPCTL] = (CC_A_SHADOW | CC_B_SHADOW | CC_CTR_A_ZERO | CC_CTR_B_ZERO);\
                                          eHRPWM1[AQCTLA] = 0x00000021;\
                                          eHRPWM1[AQCTLB] = 0x00000201;\
                                          EHRPWMClkEna;\
                                          \
                                          /* eCAP modules - APWM */\
                                          (eCAP0)[TSCTR]    = 0;\
                                          (eCAP1)[TSCTR]    = 0;\
                                          (eCAP0)[CTRPHS]   = 0;\
                                          (eCAP1)[CTRPHS]   = 0;\
                                          eCAP0[ECCTL2]     = 0x0690;\
                                          eCAP1[ECCTL2]     = 0x0690;\
                                          TIMER64P3[TGCR]   = 0x00003304;\
                                          TIMER64P3[TGCR]  |= 0x00000002;\
                                          TIMER64P3[PRD34]  = 0xFFFFFFFF;\
                                          TIMER64P3[TCR]    = 0x00800000;\
                                          \
                                          /* Setup PWM */\
                                          SetDutyMA(0);\
                                          SetDutyMB(0);\
                                          SetDutyMC(0);\
                                          SetDutyMD(0);\
                                          SETPwmFreqKHz(MAX_PWM_CNT-1);\
                                          FLOATFaultPins;\
                                          SETSleepPins;\
                                        }

#define   READIntA                      OutputRead(0,INT)
#define   READIntB                      OutputRead(1,INT)
#define   READIntC                      OutputRead(2,INT)
#define   READIntD                      OutputRead(3,INT)

#define FIX_SCALE 256

static SLONG intToFix(SLONG i)
{
    return i*FIX_SCALE;
}


static SLONG FixMult(SLONG a, SLONG b)
{
    return (a*b)/FIX_SCALE;
}

static SLONG FixDiv(SLONG a, SLONG b)
{
    return (a*FIX_SCALE)/b;
}

static SLONG FixRound(SLONG a)
{
    return (a >= 0 ?(a+(FIX_SCALE/2))/FIX_SCALE : (a-(FIX_SCALE/2))/FIX_SCALE);
}

static SLONG FixAbs(SLONG i)
{
	return (i > 0 ? i : -i);
}
#define INT_TO_FIX(i) (i*FIX_SCALE)
static SLONG STOP_LIMIT = INT_TO_FIX(2);

static SLONG F_SMOOTH1 = INT_TO_FIX(375)/1000;
static SLONG F_SMOOTH2 = INT_TO_FIX(625)/1000;
static SLONG S_SMOOTH1 = INT_TO_FIX(750)/1000;
static SLONG S_SMOOTH2 = INT_TO_FIX(250)/1000;
/*
static SLONG F_SMOOTH1 = INT_TO_FIX(300)/1000;
static SLONG F_SMOOTH2 = INT_TO_FIX(700)/1000;
static SLONG S_SMOOTH1 = INT_TO_FIX(750)/1000;
static SLONG S_SMOOTH2 = INT_TO_FIX(250)/1000;
*/
static SLONG MAX_POWER = INT_TO_FIX(100);
#define NO_LIMIT 0x7fffffff

/*
 * Functions
 */
void      CheckSpeedPowerLimits(SBYTE *pCheckVal)
{
  if (MAX_SPEED < *pCheckVal)
  {
    *pCheckVal = MAX_SPEED;
  }
  if (-MAX_SPEED > *pCheckVal)
  {
    *pCheckVal = -MAX_SPEED;
  }
}




void      SetDirRwd(UBYTE Port)
{
  OutputFloat(Port,DIR0);
  OutputHigh(Port,DIR1);
}

void      SetDirFwd(UBYTE Port)
{
  OutputHigh(Port,DIR0);
  OutputFloat(Port,DIR1);
}

void      SetBrake(UBYTE Port)
{
  OutputHigh(Port,DIR0);
  OutputHigh(Port,DIR1);
}

void      SetCoast(UBYTE Port)
{
  OutputLow(Port,DIR0);
  OutputLow(Port,DIR1);
}


UWORD     GetTachoDir(UBYTE Port)
{
  return(OutputRead(Port,DIR));
}


UWORD     GetTachoInt(UBYTE Port)
{
  return(OutputRead(Port,INT));
}


void      SetPower(UBYTE Port, SLONG Power, SLONG offset)
{
  if (MAX_PWM_CNT < Power)
  {
    Power             = MAX_PWM_CNT;
    Motor[Port].Power = Power;
  }
  if (-MAX_PWM_CNT > Power)
  {
    Power             = -MAX_PWM_CNT;
    Motor[Port].Power = Power;
  }
  if (0 != Power)
  {
    if (0 < Power)
    {
	  SetDirFwd(Port);
    }
    else
    {
	  SetDirRwd(Port);
	  Power = 0 - Power;
    }
    Power = ((Power * (10000 - offset))/10000) + offset;
  }
  SetDuty[Port](Power);
}




void      SetDutyMA(ULONG Duty)
{
  eHRPWM1[CMPA] = (UWORD)Duty;
}
void      SetDutyMB(ULONG Duty)
{
  eHRPWM1[CMPB] = (UWORD)Duty;
}
void      SetDutyMC(ULONG Duty)
{
  eCAP1[CAP2]   = Duty;
}
void      SetDutyMD(ULONG Duty)
{
  eCAP0[CAP2]   = Duty;
}

void      StopAndBrakeMotor(UBYTE MotorNo)
{
  ReadyStatus             &=  ~(0x01 << MotorNo);
  TestStatus              &=  ~(0x01 << MotorNo);
  Motor[MotorNo].Power     =  0;
  Motor[MotorNo].State     =  BRAKED;
  Motor[MotorNo].moving = FALSE;
  SetPower(MotorNo, Motor[MotorNo].Power, 0);
  SetBrake(MotorNo);
}


void      StopAndFloatMotor(UBYTE MotorNo)
{
  ReadyStatus             &=  ~(0x01 << MotorNo);
  TestStatus              &=  ~(0x01 << MotorNo);
  Motor[MotorNo].Power     =  0;
  Motor[MotorNo].State     =  IDLE;
  Motor[MotorNo].moving = FALSE;
  SetPower(MotorNo, Motor[MotorNo].Power, 0);
  SetCoast(MotorNo);
}

static void reset(MOTOR *pm)
{
	pm->baseCnt = pm->TachoCnt;
	pm->shared->baseCnt = pm->baseCnt;
	pm->tachoCnt = pm->curCnt = 0;
	pm->curVelocity = 0;
	pm->baseTime = pm->TimeCnt;
	pm->moving = FALSE;
	//printk("Reset\n");
}


/**
 * The move has completed either by the motor stopping or by it stalling
 * @param stalled
 */
static void endMove(MOTOR *pm, UBYTE stalled)
{
	//printk("end %d pos %d base %d calc %d actual %d\n", stalled, pm->TachoCnt, pm->baseCnt, FixRound(pm->curCnt), pm->curCnt);
	pm->moving = FALSE;
	pm->curVelocity = 0;
    if (pm->shared->state == ST_START)
        printk("set END from %d\n", pm->shared->state);
	if (stalled)
	{
		// stalled try and maintain current position
		reset(pm);
		pm->stallCnt = 0;
		pm->shared->state = ST_STALL;
	}
	else
	    pm->shared->state = (pm->curHold ? ST_HOLD : ST_IDLE);
}

/**
 * helper method for velocity regulation.
 * calculates power from error using double smoothing and PID like
 * control
 * @param error
 */
static void calcPower(MOTOR *pm, SLONG error, SLONG P, SLONG I, SLONG D, SLONG offset)
{
	// use smoothing to reduce the noise in frequent tacho count readings
	// New values
    SLONG newPower, power;
	pm->err1 = FixMult(pm->err1, F_SMOOTH1) + FixMult(error, F_SMOOTH2);  // fast smoothing
	pm->err2 = FixMult(pm->err2, S_SMOOTH1) + FixMult(error, S_SMOOTH2); // slow smoothing
	// Original values
	//err1 = 0.5f * err1 + 0.5f * error;  // fast smoothing
	//err2 = 0.8f * err2 + 0.2f * error; // slow smoothing
    newPower = pm->basePower + FixMult(P, pm->err1) + FixMult(D, (pm->err1 - pm->err2));
	pm->basePower = pm->basePower + FixMult(I, (newPower - pm->basePower));
	/*
    if (pm->err1 > 0)
    	pm->basePower += I;
    else if (pm->err1 < 0)
    	pm->basePower -= I;
    else
    	pm->basePower /= 2;
    */
	if (pm->basePower > MAX_POWER)
		pm->basePower = MAX_POWER;
	else if (pm->basePower < -MAX_POWER)
		pm->basePower = -MAX_POWER;
	//newPower = (float) (power*0.75 + newPower*0.25);
	power = (newPower > MAX_POWER ? MAX_POWER : newPower < -MAX_POWER ? -MAX_POWER : newPower);
	power = FixRound(power*(SLONG)SPEED_PWMCNT_REL);
	SetPower(pm->no, power, offset);
    //pm->Power = ((power * (10000 - FixAbs(offset)))/10000) + offset;
	//printk("e %d e1 %d p %d\n", error, pm->err1, pm->Power);
	//mode = (power == 0 ? TachoMotorPort.STOP : TachoMotorPort.FORWARD);
	//mode = TachoMotorPort.FORWARD;
}


void startMove(MOTOR *pm, int t1, int t2, int t3, int c1, int c2, int c3, int v1, int v2, int a1, int a3, int sl, int st, int startTime, UBYTE hold)
{
    int t = pm->TimeCnt;
	//printk("Start move\n");
    pm->mT1 = t1;
    pm->mT2 = t2;
    pm->mT3 = t3;
    pm->mV1 = v1;
    pm->mV2 = v2;
    pm->mA1 = a1;
    pm->mA3 = a3;
    pm->stallLimit = intToFix(sl);
    pm->stallTime = st/SOFT_TIMER_MS;
    pm->curHold = hold;
	pm->baseTime = t;
	// are we adjusting a current move?
	if (startTime != 0)
	    pm->baseTime -= (t - startTime);
	else
	{
	    // no so we can reset things to ditch any accumulated error
	    int adjust = pm->curCnt;
	    pm->baseCnt += FixRound(adjust);
	    pm->shared->baseCnt = pm->baseCnt;
	    pm->curCnt = 0;
	    c1 -= adjust;
	    c2 -= adjust;
	    c3 -= adjust;
	}
    pm->mC1 = c1;
    pm->mC2 = c2;
    pm->mC3 = c3;
    pm->moving = ((pm->mV1 != 0) || (pm->mV2 != 0));
    if (pm->moving)
        pm->shared->state = ST_START;
    else
        pm->shared->state = (hold ? ST_HOLD : ST_IDLE);
	//printk("Start move %d t1 %d t2 %d t3 %d pos %d\n", pm->moving, pm->mT1, pm->mT2, pm->mT3, FixRound(pm->curCnt));
	//printk("C2 %d C3 %d V1 %d V2 %d hold %d\n", FixRound(pm->mC2), FixRound(pm->mC3), FixRound(pm->mV1), FixRound(pm->mV2), pm->curHold);
	pm->State = UNLIMITED_REG;
	pm->shared->serial++;
}
/**
 * Monitors time and tachoCount to regulate velocity and stop motor rotation at limit angle
 */
void regulateMotor2(MOTOR *pm)
{
	int error;
	long long elapsed = pm->TimeCnt - pm->baseTime;
	//printk("rm 1");
	pm->tachoCnt = intToFix(pm->TachoCnt - pm->baseCnt);
	if (pm->moving)
	{
		if (elapsed <= pm->mT1)
		{
			//printk("T1\n");
			//printk("elap %d acc %d\n", (int) elapsed, pm->accTime);
			// We are still accelerating, calculate new position
			pm->curVelocity = (pm->mV1 + (SLONG)((long long)pm->mA1 * elapsed / (1024)));
			pm->curCnt = pm->mC1 + ((SLONG)((long long)(pm->mV1 + pm->curVelocity) * elapsed / (2 * 1024)));
			error = pm->curCnt - pm->tachoCnt;
			if ((pm->shared->state != ST_START) && (pm->shared->state != ST_ACCEL))
			    printk("set ACCEL from %d\n", pm->shared->state);
			pm->shared->state = ST_ACCEL;
			//error = intToFix(FixRound(pm->curCnt) - pm->TachoCnt);
			//printk("e %d tc %d\n", error, pm->TachoCnt);
			//printk("rm 2");
		}
		else if (elapsed <= pm->mT2)
		{
			//pm->State = IDLE;
			// no longer accelerating, calculate new position
			pm->curVelocity = pm->mV2;
            //pm->curCnt = (pm->mC2 + (SLONG)((long long)pm->curVelocity * (elapsed - (long long)pm->mT1) / 1024));
            pm->curCnt = (pm->mC2 + (SLONG)((long long)pm->curVelocity * (elapsed - (long long)pm->mT1) / 1024));
            error = pm->curCnt - pm->tachoCnt;
            pm->shared->state = ST_MOVE;
			//error = intToFix(FixRound(pm->curCnt) - pm->TachoCnt);
			// Check to see if the move is complete
			//printk("rm 3");
			//printk("rm 4");
		}
		else if (elapsed <= pm->mT3)
		{
			//printk("CV %d CC %d\n", FixRound(pm->curVelocity), FixRound(pm->curCnt));
			pm->curVelocity = pm->mV2 + (SLONG)((long long)pm->mA3 * (elapsed-pm->mT2) / 1024);
			pm->curCnt = pm->mC3 + ((SLONG)((long long)(pm->mV2 + pm->curVelocity) * (elapsed - pm->mT2) / (2 * 1024)));
			error = pm->curCnt - pm->tachoCnt;
            pm->shared->state = ST_DECEL;
		}
		else
		{
			error = pm->curCnt - pm->tachoCnt;
			if (((FixAbs(error) < STOP_LIMIT && elapsed > pm->mT3 + 100) || elapsed > pm->mT3 + 500))
			{
				endMove(pm, FALSE);
			}

		}
		// check for stall
		if (FixAbs(error) > pm->stallLimit)
		{
			//printk("elapsed %d\n", (int)elapsed);
			pm->baseTime += pm->TimeInc;
			if (pm->stallCnt++ > pm->stallTime)
				endMove(pm, TRUE);
		}
		else
		{
			pm->stallCnt /= 2;
		}
		//printk("rm 5");
		calcPower(pm, error, pm->moveP, pm->moveI, pm->moveD, pm->offset);

	}
	else if (pm->curHold)
	{
		// not moving, hold position
		error = pm->curCnt - pm->tachoCnt;
		// implement deadband during hold
		if (error > pm->deadBand)
		    error -= pm->deadBand;
		else if (error < -pm->deadBand)
		    error += pm->deadBand;
		else
		    error = 0;
		//printk("hold err %d\n", error);
		//printk("rm 9");
		calcPower(pm, error, pm->holdP, pm->holdI, pm->holdD, pm->offset);
		//printk("rm 10");
	}
	else
	{
		// Allow the motor to move freely
		pm->curCnt = pm->tachoCnt;
		pm->Power = 0;
		StopAndFloatMotor(pm->no);
	}
}

int locked[4];
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    Device1TimerInterrupt1
 *
 *  Motor timer interrupt function
 *
 *  Handles all motor regulation and timing related functionality
 *
 */
static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
  UBYTE No;

  static SLONG volatile TmpTacho;
  static SLONG volatile Tmp;

  hrtimer_forward_now(pTimer,Device1Time);
  for (No = 0; No < NO_OF_OUTPUT_PORTS; No++)
  {
    TmpTacho = Motor[No].IrqTacho;
    Tmp      = (TmpTacho - Motor[No].OldTachoCnt);

    Motor[No].TachoCnt      +=  Tmp;
    Motor[No].OldTachoCnt    =  TmpTacho;
    //Motor[No].TimeCnt       +=  Motor[No].TimeInc;  // Add or sub so that TimerCnt is 1 mS resolution

    /* Update shared memory */
    /*
    pMotor[No].curCnt   =  Motor[No].curCnt;
    pMotor[No].curVelocity   = Motor[No].curVelocity;
    pMotor[No].tachoCnt   =  Motor[No].TachoCnt;
    pMotor[No].time = Motor[No].TimeCnt;
*/

    if (FALSE == Motor[No].Mutex)
    {
        Motor[No].TimeCnt       +=  Motor[No].TimeInc;  // Add or sub so that TimerCnt is 1 mS resolution
      switch(Motor[No].State)
      {
        case UNLIMITED_UNREG:
        break;

        case UNLIMITED_REG:
        	regulateMotor2(&Motor[No]);
        	break;
        case LIMITED_REG_STEPUP:
        case LIMITED_REG_STEPCONST:
        case LIMITED_REG_STEPDOWN:
        case LIMITED_UNREG_STEPUP:
        case LIMITED_UNREG_STEPDOWN:
        case LIMITED_STEP_SYNC:
        case SYNCED_SLAVE:
        case LIMITED_DIFF_TURN_SYNC:
        case RAMP_DOWN_SYNC:
        case STOP_MOTOR:
        	break;
        case BRAKED:
        {
          //BrakeMotor(No, Motor[No].TachoCnt);
        }
        break;

        case IDLE:
        { /* Intentionally left empty */
        }
        break;
        default:
        { /* Intentionally left empty */
        }
        break;
      }
      /* Update shared memory */
      pMotor[No].time = Motor[No].TimeCnt;
      pMotor[No].curCnt   =  Motor[No].curCnt;
      pMotor[No].curVelocity   = Motor[No].curVelocity;
      pMotor[No].tachoCnt   =  Motor[No].TachoCnt;
      pMotor[No].time2 = Motor[No].TimeCnt;
      //if (pMotor[No].time == Motor[No].TimeCnt)
          //printk("t eq %d\n", pMotor[No].time);
    }
    else
        locked[No]++;
  }
  return (HRTIMER_RESTART);
}


static SLONG getVal(SBYTE *buf, SLONG offset)
{
	return (SLONG) ((SLONG)(buf[offset]) & 0xff) +
			(((SLONG)(buf[offset+1]) & 0xff) << 8) +
			(((SLONG)(buf[offset+2]) & 0xff) << 16) +
			(((SLONG)(buf[offset+3]) & 0xff) << 24);
}

/*! \page PWMModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 */
/*! \brief    Device1Write
 *
 *  VALID COMMANDS:
 *
 *  opOUTPUT_SET_TYPE:
 *  opOUTPUT_GET_TYPE
 *  opOUTPUT_SET_TYPE
 *  opOUTPUT_RESET:       Resets the output tacho counters
 *  opOUTPUT_STOP:        Stops the motor - either Braked or coasted
 *  opOUTPUT_POWER:       Sets the power - Duty -                        Do not start the motor
 *  opOUTPUT_SPEED:       Sets the Speed - setpoint for regulation -     Do not start the motor
 *  opOUTPUT_START:       Starts the motor if not started
 *  opOUTPUT_POLARITY:    Sets the polarity of the motor -               Do not start the motor
 *  opOUTPUT_READ:
 *  opOUTPUT_TEST:
 *  opOUTPUT_READY:
 *  opOUTPUT_POSITION:    Runs the motor to the absolute tacho positon - Starts the motor
 *  opOUTPUT_STEP_POWER:  Runs the motor un-regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_POWER:  Runs the motor un-regulated with ramp up const and down according to time
 *  opOUTPUT_STEP_SPEED:  Runs the motor regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_SPEED:  Runs the motor regulated with ramp up const and down according to the time
 *  opOUTPUT_STEP_SYNC:   Runs two motors regulated and syncronized, duration as specified by tacho cnts
 *  opOUTPUT_TIME_SYNC:   Runs two motors regulated and syncronized, duration as specified by time
 *  opOUTPUT_CLR_COUNT:   Resets the tacho count related to when motor is used as a sensor
 *
 *
 *  Default state:        TBD
 */
static ssize_t Device1Write(struct file *File,const char *Buffer,size_t Count,loff_t *Data)
{

  SBYTE   Buf[64];
  int     Lng = 0;

  copy_from_user(Buf,Buffer,Count);

  switch((UBYTE)(Buf[0]))
  {

    case opPROGRAM_STOP:
    {
      UBYTE Tmp;

      for (Tmp = 0; Tmp < OUTPUTS; Tmp++)
      {

        Motor[Tmp].Mutex   =  TRUE;
        ReadyStatus       &=  ~(0x01 << Tmp);   // Clear Ready flag
        TestStatus        &=  ~(0x01 << Tmp);   // Clear Test flag

        Motor[Tmp].Power       = 0;

          Motor[Tmp].State        = IDLE;
          Motor[Tmp].moving = FALSE;
          SetCoast(Tmp);

        Motor[Tmp].Mutex = FALSE;
      }
    }
    break;

    case opPROGRAM_START:
    break;

    case opOUTPUT_SET_TYPE:
    {
      UBYTE Tmp = Buf[1];
      Motor[Tmp].Mutex  =  TRUE;
      SETMotorType(Tmp, Buf[2]);             //  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
      SetCoast(Tmp);
      // All counts are reset when motor type changes
      Motor[Tmp].TachoCnt = 0;
      Motor[Tmp].curCnt      =  0;
      Motor[Tmp].curVelocity = 0;
      pMotor[Tmp].tachoCnt  =  0;
      pMotor[Tmp].curVelocity = 0;
      pMotor[Tmp].curCnt = 0;
      pMotor[Tmp].state = ST_IDLE;
      Motor[Tmp].TimeCnt       =  0;
      Motor[Tmp].moveP = getVal(Buf, 3);
      Motor[Tmp].moveI = getVal(Buf, 7);
      Motor[Tmp].moveD = getVal(Buf, 11);
      Motor[Tmp].holdP = getVal(Buf, 15);
      Motor[Tmp].holdI = getVal(Buf, 19);
      Motor[Tmp].holdD = getVal(Buf, 23);
      Motor[Tmp].offset = getVal(Buf, 27);
      Motor[Tmp].deadBand = getVal(Buf, 31);
      Motor[Tmp].stallCnt = 0;
      Motor[Tmp].stallLimit = intToFix(50);
      Motor[Tmp].stallTime = 1000;
      Motor[Tmp].TimeInc = SOFT_TIMER_MS;
      Motor[Tmp].basePower = 0;
      Motor[Tmp].Power = 0;
      Motor[Tmp].err1 = 0;
      Motor[Tmp].err2 = 0;
      reset(&Motor[Tmp]);
      Motor[Tmp].State  =  IDLE;
      Motor[Tmp].Mutex         =  FALSE;
    }
    break;

    case opOUTPUT_RESET:
    {
      UBYTE Tmp = Buf[1];


          Motor[Tmp].Mutex         =  TRUE;
          Motor[Tmp].TachoCnt      =  0;
          pMotor[Tmp].tachoCnt     =  0;
          Motor[Tmp].TimeCnt       =  0;
          reset(&Motor[Tmp]);
          Motor[Tmp].Mutex         =  FALSE;

    }
    break;

    case opOUTPUT_CLR_COUNT:
    {
      UBYTE Tmp = Buf[1];
          Motor[Tmp].Mutex         =  TRUE;
          pMotor[Tmp].tachoCnt     =  0;
          reset(&Motor[Tmp]);
          Motor[Tmp].Mutex         =  FALSE;
    }
    break;

    case opOUTPUT_STOP:
    {
      UBYTE Tmp = Buf[1];

          Motor[Tmp].Mutex     = TRUE;

          if (Buf[2])
          {
            StopAndBrakeMotor(Tmp);
          }
          else
          {
            StopAndFloatMotor(Tmp);
          }
          Motor[Tmp].State =  BRAKE;
          Motor[Tmp].Mutex = FALSE;
    }
    break;

    case opOUTPUT_POWER:
    {
      UBYTE Tmp = Buf[1];
      SLONG power;


      CheckSpeedPowerLimits(&(Buf[2]));
      Motor[Tmp].Mutex       = TRUE;
      power = (SLONG)(Buf[2]) * (SLONG)SPEED_PWMCNT_REL;

      if (power != Motor[Tmp].Power)
      {
        Motor[Tmp].Power  = power;
        SetPower(Tmp, power, 0);
      }
      Motor[Tmp].Mutex = FALSE;
    }
    break;

    case opOUTPUT_SPEED:
    break;

    case opOUTPUT_START:
    {
      UBYTE Tmp = Buf[1];


          Motor[Tmp].Mutex         =  TRUE;
          //startSubMove(&Motor[Tmp], getVal(Buf, 2), getVal(Buf, 6), getVal(Buf, 10), Buf[15]);
          startMove(&Motor[Tmp], getVal(Buf, 2), getVal(Buf, 6), getVal(Buf, 10), getVal(Buf, 14), getVal(Buf, 18),
                    getVal(Buf, 22), getVal(Buf, 26), getVal(Buf, 30), getVal(Buf, 34), getVal(Buf, 38), getVal(Buf, 42),
                    getVal(Buf, 46), getVal(Buf, 50), Buf[54]);
          Motor[Tmp].Mutex         =  FALSE;

    }
    break;

    case opOUTPUT_POLARITY:
    break;

    case opOUTPUT_POSITION:
    {
    }
    break;

    case opOUTPUT_STEP_POWER:
    break;

    case opOUTPUT_TIME_POWER:
    break;

    case opOUTPUT_STEP_SPEED:
    break;

    case opOUTPUT_TIME_SPEED:
    break;

    case opOUTPUT_STEP_SYNC:

    break;

    case opOUTPUT_TIME_SYNC:

    break;

    default:
    {
    }
    break;
  }

  return (Lng);
}


static ssize_t Device1Read(struct file *File,char *Buffer,size_t Count,loff_t *Offset)
{
  int     Lng     = 0;

  Lng    =  snprintf(&Buffer[0],Count,"%01u ",ReadyStatus);
  Lng   +=  snprintf(&Buffer[Lng],Count - Lng,"%01u ",TestStatus);

  return (Lng);
}


static    const struct file_operations Device1Entries =
{
  .owner        = THIS_MODULE,
  .read         = Device1Read,
  .write        = Device1Write
};


static    struct miscdevice Device1 =
{
  MISC_DYNAMIC_MINOR,
  DEVICE1_NAME,
  &Device1Entries
};


/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    GetPeriphealBasePtr
 *
 *  Helper function for getting the peripheal HW base address
 *
 */
void    GetPeriphealBasePtr(ULONG Address, ULONG Size, ULONG **Ptr)
{
  /* eCAP0 pointer */
  if (request_mem_region(Address,Size,MODULE_NAME) >= 0)
  {

    *Ptr  =  (ULONG*)ioremap(Address,Size);

    if (*Ptr != NULL)
    {
#ifdef DEBUG
      printk("%s memory Remapped from 0x%08lX\n",DEVICE1_NAME,(unsigned long)*Ptr);
#endif
    }
    else
    {
      printk("Memory remap ERROR");
    }
  }
  else
  {
    printk("Region request error");
  }
}


/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    Device1Init
 *
 */
static int Device1Init(void)
{
  int     Result = -1;
  UBYTE   Tmp;

  GetPeriphealBasePtr(0x01C14000, 0x190, (ULONG**)&SYSCFG0);  /* SYSCFG0 pointer    */
  GetPeriphealBasePtr(0x01E2C000, 0x1C,  (ULONG**)&SYSCFG1);  /* SYSCFG1 pointer    */
  GetPeriphealBasePtr(0x01F02000, 0x2854,(ULONG**)&eHRPWM1);  /* eHRPWM Pointer     */
  GetPeriphealBasePtr(0x01F06000, 0x60,  (ULONG**)&eCAP0);    /* eCAP0 pointer      */
  GetPeriphealBasePtr(0x01F07000, 0x60,  (ULONG**)&eCAP1);    /* eCAP1 pointer      */
  GetPeriphealBasePtr(0x01F0D000, 0x80,  (ULONG**)&TIMER64P3);/* TIMER64P3 pointer  */
  GetPeriphealBasePtr(0x01E26000, 0xD4,  (ULONG**)&GPIO);     /* GPIO pointer       */
  GetPeriphealBasePtr(0x01E1A000, 0x1F8, (ULONG**)&PLLC1);    /* PLLC1 pointer      */
  GetPeriphealBasePtr(0x01E27000, 0xA80, (ULONG**)&PSC1);     /* PSC1 pointer       */

  Result  =  misc_register(&Device1);
  if (Result)
  {
    printk("  %s device register failed\n",DEVICE1_NAME);
  }
  else
  {
#ifdef DEBUG
    printk("  %s device register succes\n",DEVICE1_NAME);
#endif

    iowrite32(0x00000003, &PSC1[0x291]); /* Setup ePWM module power on  */
    iowrite32(0x00000003, &PSC1[0x48]);  /* Eval the NEXT field         */

    iowrite32((ioread32(&PSC1[0x294]) | 0x00000003), &PSC1[0x294]);/* Turn PSC on for the eCAP module */
    iowrite32((ioread32(&PSC1[0x48])  | 0x00000003), &PSC1[0x48]); /* Execute the next step           */

    for(Tmp = 0; Tmp < NO_OF_OUTPUT_PORTS; Tmp++)
    {
      memset(&Motor[Tmp], 0, sizeof(MOTOR));

      Motor[Tmp].Type         =  TYPE_NONE;
      Motor[Tmp].State        =  IDLE;
      Motor[Tmp].Mutex        =  FALSE;
      Motor[Tmp].no           =  Tmp;

      SETMotorType(Tmp, TYPE_NONE);                  //  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
    }

    /* Float the tacho inputs */
    for(Tmp = 0; Tmp < NO_OF_OUTPUT_PORTS; Tmp++)
    {
      OutputFloat(Tmp,INT);
      OutputFloat(Tmp,DIR);
      SetCoast(Tmp);
    }

    /* Setup the PWM peripheals */
    SETUPPwmModules;

    /* Setup interrupt for the tacho int pins */
    SetGpioRisingIrq(IRQA_PINNO, IntA);
    SetGpioRisingIrq(IRQB_PINNO, IntB);
    SetGpioRisingIrq(IRQC_PINNO, IntC);
    SetGpioRisingIrq(IRQD_PINNO, IntD);
  }
  return (Result);
}


static void Device1Exit(void)
{
  hrtimer_cancel(&Device1Timer);
  misc_deregister(&Device1);
#ifdef DEBUG
  printk("  %s device unregistered\n",DEVICE1_NAME);
#endif
  iounmap(SYSCFG0);
  iounmap(SYSCFG1);
  iounmap(GPIO);
  iounmap(eCAP0);
  iounmap(eCAP1);
  iounmap(TIMER64P3);
  iounmap(eHRPWM1);
  iounmap(PLLC1);
  iounmap(PSC1);
#ifdef DEBUG
  printk("  %s memory unmapped\n",DEVICE1_NAME);
#endif
}


static    void  __iomem *GpioBase;

void      SetGpio(int Pin)
{
  int     Tmp = 0;
  void    __iomem *Reg;

  if (Pin >= 0)
  {
    while ((MuxRegMap[Tmp].Pin != -1) && (MuxRegMap[Tmp].Pin != Pin))
    {
      Tmp++;
    }
    if (MuxRegMap[Tmp].Pin == Pin)
    {
      Reg   =  da8xx_syscfg0_base + 0x120 + (MuxRegMap[Tmp].MuxReg << 2);

      *(u32*)Reg &=  MuxRegMap[Tmp].Mask;
      *(u32*)Reg |=  MuxRegMap[Tmp].Mode;

      if (Pin < NO_OF_GPIOS)
      {
#ifdef DEBUG
        printk("    GP%d_%-2d   0x%08X and 0x%08X or 0x%08X\n",(Pin >> 4),(Pin & 0x0F),(u32)Reg, MuxRegMap[Tmp].Mask, MuxRegMap[Tmp].Mode);
#endif
      }
      else
      {
#ifdef DEBUG
        printk("   OUTPUT FUNCTION 0x%08X and 0x%08X or 0x%08X\n",(u32)Reg, MuxRegMap[Tmp].Mask, MuxRegMap[Tmp].Mode);
#endif
      }
    }
    else
    {
      printk("    GP%d_%-2d Not found (Const no. %d, Tmp = %d)\n",(Pin >> 4),(Pin & 0x0F), Pin, Tmp);
    }
  }
}


void      InitGpio(void)
{
  int     Port;
  int     Pin;

  // unlock
  REGUnlock;

  for (Port = 0;Port < NO_OF_OUTPUT_PORTS;Port++)
  {
#ifdef DEBUG
    printk("  Output port %d\n",Port + 1);
#endif
    for (Pin = 0;Pin < OUTPUT_PORT_PINS;Pin++)
    {
      if ((pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin) >= 0)
      {
        pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].pGpio  =  (struct gpio_controller *__iomem)(GpioBase + ((pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin >> 5) * 0x28) + 0x10);
        pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Mask   =  (1 << (pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin & 0x1F));

        SetGpio(pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin);
      }
    }
  }

  // lock
  REGLock;
}


void    SetGpioRisingIrq(UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *))
{
  UWORD Status;

  Status = request_irq(gpio_to_irq(PinNo), IntFuncPtr, 0, "PWM_DEVICE", NULL);
  if(Status < 0)
  {
    printk("error %d requesting GPIO IRQ %d\n", Status, PinNo);
  }
  set_irq_type(gpio_to_irq(PinNo), IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING);
#ifdef DEBUG
  printk(".. done\n");
#endif
}


/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    IntA
 *
 *  Tacho A interrupt function
 *
 *  Tacho count is incremented or decremented on both positive
 *  and negative edges of the INT signal.
 *
 *  For each positive and negative edge of the INT tacho signal
 *  a timer is sampled. this is used to calculate the speed later on.
 *
 *  DirChgPtr is implemented for ensuring that there is enough
 *  samples in the same direction to calculate a speed.
 *
 */
static    irqreturn_t IntA (int irq, void * dev)
{
  ULONG   IntAState;
  ULONG   DirAState;
  ULONG   Timer;

  // Sample all necessary items as fast as possible
  IntAState  =  READIntA;
  DirAState  =  READDirA;
  Timer      =  FREERunning24bittimer;

    if (IntAState)
    {
      if(DirAState)
      {
        (Motor[0].IrqTacho)++;
      }
      else
      {
        (Motor[0].IrqTacho)--;
      }
    }
    else
    {
      if(DirAState)
      {
        (Motor[0].IrqTacho)--;
      }
      else
      {
        (Motor[0].IrqTacho)++;
      }
    }

  return IRQ_HANDLED;
}


static    irqreturn_t IntB (int irq, void * dev)
{
  ULONG   volatile IntBState;
  ULONG   volatile DirBState;
  ULONG   volatile Timer;

  // Sample all necessary items as fast as possible
  IntBState  =  READIntB;
  DirBState  =  READDirB;
  Timer      =  FREERunning24bittimer;

    if (IntBState)
    {
      if(DirBState)
      {
        (Motor[1].IrqTacho)++;
      }
      else
      {
        (Motor[1].IrqTacho)--;
      }
    }
    else
    {
      if(DirBState)
      {
        (Motor[1].IrqTacho)--;
      }
      else
      {
        (Motor[1].IrqTacho)++;
      }
    }

  return IRQ_HANDLED;
}


static    irqreturn_t IntC (int irq, void * dev)
{
  ULONG   IntCState;
  ULONG   DirCState;
  ULONG   Timer;

  // Sample all necessary items as fast as possible
  IntCState  =  READIntC;
  DirCState  =  READDirC;
  Timer      =  FREERunning24bittimer;

    if (IntCState)
    {
      if(DirCState)
      {
        (Motor[2].IrqTacho)++;
      }
      else
      {
        (Motor[2].IrqTacho)--;
      }
    }
    else
    {
      if(DirCState)
      {
        (Motor[2].IrqTacho)--;
      }
      else
      {
        (Motor[2].IrqTacho)++;
      }
    }

  return IRQ_HANDLED;
}


static    irqreturn_t IntD (int irq, void * dev)
{
  ULONG   IntDState;
  ULONG   DirDState;
  ULONG   Timer;

  // Sample all necessary items as fast as possible
  IntDState  =  READIntD;
  DirDState  =  READDirD;
  Timer      =  FREERunning24bittimer;

    if (IntDState)
    {
      if(DirDState)
      {
        (Motor[3].IrqTacho)++;
      }
      else
      {
        (Motor[3].IrqTacho)--;
      }
    }
    else
    {
      if(DirDState)
      {
        (Motor[3].IrqTacho)--;
      }
      else
      {
        (Motor[3].IrqTacho)++;
      }
    }

  return IRQ_HANDLED;
}




/*! \page PWMModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 */
 /*! \brief    Device2Write
 *
 *  Only used for daisy chaining to ensure Busy flags being set from when message has been
 *  received until being executed by the VM
 *
 */
static ssize_t Device2Write(struct file *File,const char *Buffer,size_t Count,loff_t *Data)
{
  int     Lng     = 0;
  SBYTE   Buf[20];

  copy_from_user(Buf, Buffer, Count);

  ReadyStatus |= Buf[0];   // Set Ready flag
  TestStatus  |= Buf[0];   // Set Test flag

  return (Lng);
}


/*! \page PWMModule
 *
 *  <hr size="1"/>
 *  <b>     read </b>
 *
 *
 */
/*! \brief    Device2Read
 *
 */
static ssize_t Device2Read(struct file *File,char *Buffer,size_t Count,loff_t *Offset)
{
  int     Lng     = 0;
  return (Lng);
}

#define     SHM_LENGTH    (sizeof(MotorData))
#define     NPAGES        ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)
static void *kmalloc_ptr;


static int Device2Mmap(struct file *filp, struct vm_area_struct *vma)
{
   int ret;

   ret = remap_pfn_range(vma,vma->vm_start,virt_to_phys((void*)((unsigned long)pMotor)) >> PAGE_SHIFT,vma->vm_end-vma->vm_start,PAGE_SHARED);

   if (ret != 0)
   {
     ret  =  -EAGAIN;
   }

   return (ret);
}


static    const struct file_operations Device2Entries =
{
  .owner        = THIS_MODULE,
  .read         = Device2Read,
  .write        = Device2Write,
  .mmap         = Device2Mmap,
};


static    struct miscdevice Device2 =
{
  MISC_DYNAMIC_MINOR,
  DEVICE2_NAME,
  &Device2Entries
};


static int Device2Init(void)
{
  int       Result = -1;
  int       i;
  MOTORSHARED *pTmp;

  Result  =  misc_register(&Device2);
  if (Result)
  {
    printk("  %s device register failed\n",DEVICE2_NAME);
  }
  else
  { // allocate kernel shared memory for tacho counts and speed

    if ((kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL)) != NULL)
    {
      pTmp = (MOTORSHARED*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
      for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE)
      {
        SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
      }
      pMotor =  pTmp;
      memset(pMotor,0,sizeof(MotorData));
      for(i = 0; i < NO_OF_OUTPUT_PORTS; i++)
          Motor[i].shared = &(pMotor[i]);

#ifdef DEBUG
      printk("  %s device register succes\n",DEVICE2_NAME);
#endif
    }
    else
    {
      printk("  %s kmalloc failed !!\n",DEVICE2_NAME);
    }
  }
  return (Result);
}


static void Device2Exit(void)
{
  MOTORSHARED   *pTmp;
  int         i;

  pTmp    =  pMotor;
  pMotor  =  MotorData;
  // free shared memory
  for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE)
  {
    ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
#ifdef DEBUG
    printk("  %s memory page %d unmapped\n",DEVICE1_NAME,i);
#endif
  }
  kfree(kmalloc_ptr);

  misc_deregister(&Device2);
#ifdef DEBUG
  printk("  %s device unregistered\n",DEVICE2_NAME);
#endif
for(i=0; i < 4; i++)
    printk("lock %d cnt %d\n", i, locked[i]);

}

#ifndef PCASM
module_param (HwId, charp, 0);
#endif

static int ModuleInit(void)
{
	long long ll = Motor[1].TimeCnt;
	Hw  =  HWID;

  if (Hw < PLATFORM_START)
  {
    Hw  =  PLATFORM_START;
  }
  if (Hw > PLATFORM_END)
  {
    Hw  =  PLATFORM_END;
  }
printk("long %d\n", sizeof(long));
printk("long long %d\n", sizeof(long long));
printk("sm size %d %d\n", sizeof(MOTORSHARED), ((char *)&pMotor[1]) - ((char *)&pMotor[0]));
//ll = ll / (long long)1000;
printk("div %d\n", (int)ll);

#ifdef DEBUG
  printk("%s init started\n",MODULE_NAME);
#endif

  if (request_mem_region(DA8XX_GPIO_BASE,0xD8,MODULE_NAME) >= 0)
  {
    GpioBase  =  (void*)ioremap(DA8XX_GPIO_BASE,0xD8);
    if (GpioBase != NULL)
    {
#ifdef DEBUG
      printk("%s gpio address mapped\n",MODULE_NAME);
#endif

      switch(Hw)
      {
        case EP2:
        {
          /* This is to comply with changing of inverters on the tacho direction pins */
          /* only Final hardware does not need to be inverted                         */
          HwInvBits = 0xFFFFFFFF;

          /* Motor PWM outputs has been switched in EP2 MA<->MB and MC<->MD */
          SetDuty[0]  = SetDutyMB;
          SetDuty[1]  = SetDutyMA;
          SetDuty[2]  = SetDutyMD;
          SetDuty[3]  = SetDutyMC;
        }
        break;
        case FINALB:
        {
          /* This is to comply with changing of inverters on the tacho direction pins */
          /* only Final hardware does not need to be inverted                         */
          HwInvBits = 0xFFFFFFFF;

          /* Motor PWM outputs has been switched in EP2 MA<->MB and MC<->MD */
          SetDuty[0]  = SetDutyMA;
          SetDuty[1]  = SetDutyMB;
          SetDuty[2]  = SetDutyMC;
          SetDuty[3]  = SetDutyMD;
        }
        break;
        case FINAL:
        {
          /* This is to comply with changing of inverters on the tacho direction pins */
          /* only Final hardware does not need to be inverted                         */
          HwInvBits = 0;

          /* Motor PWM outputs has been switched in EP2 MA<->MB and MC<->MD */
          SetDuty[0]  = SetDutyMA;
          SetDuty[1]  = SetDutyMB;
          SetDuty[2]  = SetDutyMC;
          SetDuty[3]  = SetDutyMD;
        }
        break;
      }

      InitGpio();

      Device1Init();
      Device2Init();

      /* Setup timer irq*/
      Device1Time  =  ktime_set(0,SOFT_TIMER_SETUP);
      hrtimer_init(&Device1Timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
      Device1Timer.function  =  Device1TimerInterrupt1;
      hrtimer_start(&Device1Timer,Device1Time,HRTIMER_MODE_REL);
    }
  }
  return (0);
}


static void ModuleExit(void)
{
#ifdef DEBUG
  printk("%s exit started\n",MODULE_NAME);
#endif
  STOPPwm;
  free_irq(gpio_to_irq(IRQA_PINNO), NULL);
  free_irq(gpio_to_irq(IRQB_PINNO), NULL);
  free_irq(gpio_to_irq(IRQC_PINNO), NULL);
  free_irq(gpio_to_irq(IRQD_PINNO), NULL);

  Device1Exit();
  Device2Exit();

  iounmap(GpioBase);

}




