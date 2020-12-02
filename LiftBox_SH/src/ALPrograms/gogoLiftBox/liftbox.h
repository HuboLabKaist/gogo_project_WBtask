/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */
#ifndef LIFTBOX_H
#define LIFTBOX_H
//#include <QCoreApplication>

#include "../../../share/Headers/commandlist.h"
#include "joint.h"
#include "taskmotion.h"
#include "ManualCAN.h"
#include "UserSharedMemory.h"
#include "BasicFiles/BasicTrajectory.h"
#include <iostream>
#include <libpcan.h>
#include <iostream>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <alchemy/task.h>

#define PODO_AL_NAME       "APPROACHBOX_AL"



using namespace std;

inline void pushData(doubles &tar, double var){
    tar.push_back(var);
    tar.pop_front();
}

/***************************** 1. Structs ************************************/
/* Shared memory */
pRBCORE_SHM         sharedData;
pUSER_SHM           userData;

/* RT task handler for control */
TaskMotion      *WBmotion;
RT_TASK         rtTaskCon;
RT_TASK         rtFlagCon;

enum{
    NOTHING = 0, SITDOWN, WAIT, HOLDBOX, RELEASEBOX, HANDUP, STANDUP, PUTONTABLE, PUSHIN, HANDSDOWN, HANDSBACK
};
enum{
    MODE_NOTHING = 0, MODE_BACK, MODE_MOVEBOX, MODE_MOVEBOX_FRONT, MODE_TEST, MODE_UB, MODE_MOVEBOX_LIFTBOX
};
enum{
    HOLD_YET = 0, HOLD_SUCCESS, HOLD_FAIL
};
enum{
    BACK_NOTHING = 0, BACK_BACK, BACK_SIT, BACK_HOLD, BACK_LIFT, BACK_WAIT,
    BACK_STAND, BACK_FRONT, BACK_OPENARMS, BACK_PUSHIN, BACK_GETHANDPOS,
    BACK_HANDSDOWN, BACK_RELEASE, BACK_HANDSBACK, BACK_TURNTOFRONT,BACK_HANDSBACK2,
    BACK_PUSHIN2,BACK_PREPUSHIN2,
    BACK_COM_FORWARD, BACK_COM_BACKWARD,
};
enum
{
    UB_NOTHING = 0, UB_APPROACH, UB_COM_FOLLOW, UB_FOLLOW_DONE,
    UB_HOLDCART, UB_RELEASECART, UB_WAIT, UB_GRIPPER_CLOSEHALF, UB_GRIPPER_OPEN,
    UB_WALKREADYISH,
};

enum{
    STOP_STOP = 0, STOP_RESUME,
};
////GRIPPER
//enum{
//    HAND_BOTH = 0, HAND_R, HAND_L
//};
enum{
    HAND_NO_ACT = 0, HAND_STOP, HAND_OPEN, HAND_GRASP, HAND_DONE
};

enum{
    TEST_NOTHING = 0, TEST_ADDMASS, TEST_SUBMASS
};

//Function back_motion
enum{
    NO_USE = 1000, HAND_GLOBAL, HAND_LOCAL,
};

//motion mode Flag
enum{
    PARTS = 0, LIFT, PUT, LIFTPUT,
};

//Move Box motion mode flag
enum{
    MOVEMODE_PARTS = 0, MOVEMODE_LIFT, MOVEMODE_PUT,
};

//Cart motion mode flag
enum Mode_Cart_mode{
    CART_PARTS = 0, CART_HOLD, CART_RELEASE,
};


/***************************** 2. Flags **************************************/
int     isTerminated;
int     __IS_WORKING;
int     __IS_GAZEBO;
/* Command */
int     Command_LIFTBOX = NOTHING;
int     Mode_LIFTBOX    = MODE_NOTHING;
int     Mode_BACKMOTION = BACK_NOTHING;
int     Mode_UBMOTION   = UB_NOTHING;
int     Mode_MOVEBOX    = MOVE_NOTHING;
int     FLAG_HAND       = HAND_NO_ACT;
int     Hold_Box_Flag   = false;
int     FLAG_motionMode = false;
int     FLAG_RESUME     = false;
int     FLAG_MB_mode    = false;
int     FLAG_CART_mode  = false;

int     saved_USER_COMMAND      = NOTHING;
int     saved_Command_LIFTBOX   = NOTHING;
int     saved_Mode_LIFTBOX      = MODE_NOTHING;

/***************************** 3. Variables **********************************/
/* Basic */
int PODO_NO;
int PODO_NO_DAEMON = 0;
int PODO_NO_WALKREADY;
char __AL_NAME[30];

/* WBIK */
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;

/* Motion */
typedef struct{
    double SitHandX = 0.6;
    double SitHandY = 0.35;
    double SitHandZ = 0.45;
    double SitPelZ = 0.8;
    double ElbAngle = 15.0;

    double StandHandX = 0.5;
    double StandHandZ = 1.0;
    double StandPelZ = 0.78;
    double FinalHandX = 0.3;
    double FinalHandY = 0.245;
    double FinalHandZ = 1.0;

    double HandPitch = -50.;
    double HandYaw = 10.;
    double PelPitch = 60.;


    double HandUpZ = 0.15;
    double HoldY = 0.0003;
    double ReleaseY = 0.0004;

    double TimeSit = 3.0;
    double TimeMaxHold = 3.5;
    double TimeRelease = 2.5;
    double TimeHandUp = 1.5;
    double TimeStand = 4.0;

    double FTlimit = 75;


    //Back motions

    ///COMshift
    //BACK_BACK
    double back_backCOMshift    = -0.042;
    //BACK_SIT
    double back_sitCOMshift     =  0.0;
    //BACK_STAND
    double back_standCOMshift   =  0.042;
    //BACK_LIFT
    double back_liftCOMshift    =  0.01;

    //BACK_OPENARMS
    double back_openArmsHandx   = -0.35;
    double back_openArmsHandy   = 0.45;
    double back_openArmsHandz   = 0.8;
    double TimeBackOpenArms     = 2.0;

    //BACK_SIT
    double back_sitPelz         = -0.38;
    double back_sitPelPitch     =  0.0;

    double back_sitHandx        = -0.45;//-0.40;
    double back_sitHandy        =  0.40;
    double back_sitHandz        =  0.28;

    //BACK_HOLD
    double back_holdHandy       = 0.08;
    double TimeBackHold         = 4.0;

    //BACK_LIFT
    double back_liftHandx       = -0.35;
    double back_liftHandz       =  0.52;
    double TimeBackLift         =  2.0;   //3.0;

    //BACK_STAND
    double back_standHandx      = 0.45;
    double back_standHandz      = 0.52;

    //BACK_STAND2
    double back_stand2Handx     = 0.62;//0.45+0.17;
    double back_stand2Handz     = 0.64;//0.52+0.12;

    //BACK_FRONT
    double back_frontComOffsetx = -0.025;

    //BACK_PUSHIN
    double back_pushInHandx     = -0.72 -0.03;
    double back_pushInHandy     =  0.32;
    double back_pushInHandz     =  1.40;
    double TimeBackPushIn       =  3.0;

    //BACK_HANDSDOWN
    double back_handsDownHandx  = back_pushInHandx;
    double back_handsDownHandy  = back_pushInHandy;
    double back_handsDownHandz  = back_pushInHandz - 0.08;

    //BACK_RELEASE
    double back_releaseHandx    = back_handsDownHandx;
    double back_releaseHandy    = back_handsDownHandy + 0.08;
    double back_releaseHandz    = back_handsDownHandz;

    //BACK_HANDSBACK
    double back_HandsBackHandx  = back_releaseHandx + 0.32 + 0.03 + 0.05;
    double back_HandsBackHandy  = back_releaseHandy;
    double back_HandsBackHandz  = back_releaseHandz - 0.18;
    double TimeBackHandsBack    = 3.0;

    //BACK_PUSHIN2
    double back_pushIn2Handx    = back_HandsBackHandx - 0.10;
    double back_pushIn2Handy    = back_HandsBackHandy - 0.15;
    double back_pushIn2Handz    = back_HandsBackHandz;
    double TimeBackPushIn2      = 3.0;

    //BACK_HANDSBACK2
    double back_HandsBack2Handx = back_HandsBackHandx + 0.20 -0.03 -0.05;
    double back_HandsBack2Handy = back_HandsBackHandy - 0.15;
    double back_HandsBack2Handz = back_HandsBackHandz - 0.10;
    double TimeBackHandsBack2   = 3.0;

    //BACK_TURNTOFRONT
    double TimeTurnFront        = 2.0;
    double back_turnToFrontCOMx = 0.015;


    //Put motions
    double pushInHandx = 0.0;
    double pushInHandy = 0.0;
    double pushInHandz = 0.0;
    double TimePushIn  = 4.0;

    double handsdownHandz = 0.06; // 6cm
    double TimeHandsDown  = 2.0;

    double handsBackHandx = 0.35;
    double handsBackHandz = 0.20;
    double TimeHandsBack  = 5.0;


}input;
input in;

/*MoveBox Box Lifting Motion*/
/* Motion */
typedef struct{

    //MOVE_PRETURN
    double move_preTurnHandx   =-0.30;
    double move_preTurnHandy   = 0.25;
    double move_preTurnHandz   = 1.05;
    double TimeMovePreTurn     = 0.5;

    //MOVE_RIGHT
    double TimeMoveRight       = 2.0;
    //MOVE_RIGHT_WBOX
    double TimeMoveRightWBox   = 3.8;

    //MOVE_OPENARMS
    double move_openArmsHandx   =-0.30 - 0.05;
    double move_openArmsHandy   = 0.40;
    double move_openArmsHandz   = 1.0;
    double TimeMoveOpenArms     = 2.0;

    //MOVE_APPROACH
    double move_approachHandx   =-0.65 -0.18;
    double move_approachHandy   = 0.40;
    double move_approachHandz   = 1.0 + 0.02;
    double TimeMoveApproach     = 2.5;

    //MOVE_HOLD
    double move_HoldHandy       = 0.08;
    double TimeMoveHold         = 3.0;

    //MOVE_LIFT
    double move_liftHandx       =-0.83;
    double move_liftHandy       = 0.32;
    double move_liftHandz       = 1.15;
    double TimeMoveLift         = 1.0;

    //MOVE_LIFT2
    double move_lift2Handx       =-0.45;
    double move_lift2Handy       = 0.32;
    double move_lift2Handz       = 1.25;
    double TimeMoveLift2         = 2.0;

    //MOVE_LEFT
    double TimeMoveLeft          = 2.0;

    //MOVE_PUTIN
    double move_putInHandx      =-0.8;
    double move_putInHandy      = 0.32;
    double move_putInHandz      = 1.15;
    double TimeMovePutIn        = 2.0;

    //MOVE_PUTIN2
    double move_putIn2Handx      =-0.65 -0.15;
    double move_putIn2Handy      = 0.32;
    double move_putIn2Handz      = 1.0 + 0.02;
    double TimeMovePutIn2        = 2.0;

    //MOVE_RELEASE
    double move_releaseHandx    =-0.65 -0.15;
    double move_releaseHandy    = 0.40;
    double move_releaseHandz    = 1.0 + 0.02;
    double TimeMoveRelease      = 2.0;

    //MOVE_HANDSBACK
    double move_handsBackHandx  =-0.40;
    double move_handsBackHandy  = 0.40;
    double move_handsBackHandz  = 1.0;
    double TimeMoveHandsBack    = 2.5;

    //MOVE_WALKREADYISH
    double move_walkReadyishHandx   =-0.30;
    double move_walkReadyishHandy   = 0.25;
    double move_walkReadyishHandz   = 1.00;
    double TimeMoveWalkReadyish     = 2.0;

    //MOVE_LEFTTOFRONT
    double TimeMoveLeftToFront = 2.0;

}moveBox_motion;
moveBox_motion move_in;

double PELpos[3];
double RHpos[3];
double LHpos[3];
doubles RHori(4);
doubles LHori(4);
doubles PELori(4);

double COMx_shift = 0.;

double WaitTime  = 0.;

int WaitMode     = NOTHING;
int WaitCount    = 0;
int HoldCount    = 0;
int ReleaseCount = 0;



int FLAG_Gripper = false;
int MODE_RGripper = 0;
int MODE_LGripper = 0;
int MODE_Gripper = 0;
int SIDE_Gripper = 0;
double LIMIT_Gripper = 0.;
double LIMIT_Gripper_half = -20.;
double DESIRED_Gripper = -40; //-40.;
int MAX_GRIPPER_CNT = 1000;

int mode_prev = 0;


//Box mass
double boxmass = 5.06;

//pelvis
double side_pelx = 0.0;
double side_pely = 0.0;
double side_pelz = 0.0;


/***************************** 4. Functions **********************************/
/* Basic */
int  HasAnyOwnership();
int  CheckMotionOwned();
int CheckMotionOwned_arm();
void RBTaskThread(void *);
void RBFlagThread(void *);
void CatchSignals(int _signal);
void ShowWBInfos();

/* Initialization */
int  RBInitialize(void);
void ShutDownAllFlag();
void SaveWalkReadyPos();
void StartWBIKmotion(int _mode);

/* Motion */
void LiftBox_Supervisor();
void GripperTH();
void Cart_WalkReady();

//part motions
void SitDown(int _mode);
void putOnTable(int _mode);
void pushIn();
void handsdown();
void handsback();
int  HoldBox();
void AbsHoldBox();
int  ReleaseBox();
void HandUp();
void StandUp(int _mode);

int  GainOverrideSY();
int arm_GainOverride(int gain, short _msec);
void Set_RHand_Global2Local(vec3 _pos, quat _ori);
void Set_LHand_Global2Local(vec3 _pos, quat _ori);
void SetOriPitch(doubles &target, double _pitch);
void SetOriYaw(doubles &target, double _yaw);
void SetOriHand(doubles &target, double _pitch, double _yaw);
void back_move(double COMx, double COMy, double Pelz, double PelPitch,
               int HandMode, double Handx, double Handy, double Handz,
               double HandOriPitch, double HandOriYaw, double Time);
//int  CheckFTsensor();
void SetWaitTime(int mode, double time);
void SetWaitTime_back(int mode, double time);
void SetWaitTime_move(int mode, double time);
void SetWaitTime_cart(int mode, double time);

/*zmp*/
vec3 ZMP_calc_HB(vec3 _pRF, quat _qRF, vec3 _F_RF, vec3 _M_RF, vec3 _pLF, quat _qLF, vec3 _F_LF,vec3 _M_LF);
vec3 ZMP_calc_SH();
double zmp_calc[3] = {0., };

//Command saving
int command_num  = 0;
int command_FLAG = 0;

#endif // LIFTBOX_H
