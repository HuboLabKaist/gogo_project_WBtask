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
pRBCORE_SHM_SENSOR  sharedSEN;

/* RT task handler for control */
TaskMotion      *WBmotion;
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

enum{
    NOTHING = 0, SITDOWN, WAIT, HOLDBOX, RELEASEBOX, HANDUP, STANDUP, PUTONTABLE, PUSHIN, HANDSDOWN, HANDSBACK
};
enum{
    MODE_NOTHING = 0, MODE_LIFTBOX, MODE_PUTBOX, MODE_BACK,
    MODE_BACK_LIFTBOX, MODE_BACK_PUTBOX, MODE_LIFTBOX_PARTS,
    MODE_BACK_LIFTPUT,
};
enum{
    HOLD_YET = 0, HOLD_SUCCESS, HOLD_FAIL
};
enum{
    BACK_NOTHING = 0, BACK_BACK, BACK_SIT, BACK_HOLD, BACK_LIFT, BACK_WAIT,
    BACK_STAND, BACK_FRONT, BACK_OPENARMS, BACK_PUSHIN, BACK_GETHANDPOS,
    BACK_HANDSDOWN, BACK_RELEASE, BACK_HANDSBACK, BACK_TURNTOFRONT,BACK_HANDSBACK2,
    BACK_STAND2, BACK_WAIT_STAND,
};

enum{
    TEST_NOTHING = 0, TEST_ADDMASS, TEST_SUBMASS
};

//Function back_motion
enum{
    NO_USE = 1000, HAND_GLOBAL, HAND_LOCAL,
};


/***************************** 2. Flags **************************************/
int     isTerminated;
int     __IS_WORKING;
int     __IS_GAZEBO;
/* Command */
int     Command_LIFTBOX = NOTHING;
int     Mode_LIFTBOX    = MODE_NOTHING;
int     Mode_BACKMOTION = BACK_NOTHING;
int     FLAG_HAND       = GRIPPER_BREAK;
int     Hold_Box_Flag   = false;

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

    //BACK_OPEN ARMS
    double back_openArmsHandx   = -0.35;
    double back_openArmsHandy   = 0.45;
    double back_openArmsHandz   = 0.8;
    double TimeBackOpenArms     = 2.0;

    //BACK_SIT
    double back_sitPelz     = -0.38;
    double back_sitPelPitch =  0.0;

    double back_sitHandx = -0.40;
    double back_sitHandy =  0.40;
    double back_sitHandz =  0.28;

    //BACK_HOLD
    double back_holdHandy = 0.08;
    double TimeBackHold   = 4.0;

    //BACK_LIFT
    double back_liftHandx = -0.35;
    double back_liftHandz =  0.52;
    double TimeBackLift   =  2.0;   //3.0;

    //BACK_STAND
    double back_standHandx = 0.45;
    double back_standHandz = 0.52;

    //BACK_STAND2
    double back_stand2Handx = 0.62;//0.45+0.17;
    double back_standH2andz = 0.64;//0.52+0.12;

    //BACK_FRONT
    double back_frontComOffsetx = 0.0;

    //BACK_PUSHIN
    double back_pushInHandx = -0.72 -0.03;
    double back_pushInHandy =  0.32;
    double back_pushInHandz =  1.40;
    double TimeBackPushIn   =  3.0;

    //BACK_HANDSDOWN
    double back_handsDownHandx  = back_pushInHandx;
    double back_handsDownHandy  = back_pushInHandy;
    double back_handsDownHandz  = back_pushInHandz - 0.08;

    //BACK_RELEASE
    double back_releaseHandx    = back_handsDownHandx;          //back_pushInHandx;
    double back_releaseHandy    = back_handsDownHandy + 0.08;   //back_pushInHandy + 0.08;
    double back_releaseHandz    = back_handsDownHandz;          //back_pushInHandz - 0.08;

    //BACK_HANDSBACK
    double back_HandsBackHandx  = back_releaseHandx + 0.32;      //back_pushInHandx + 0.32;
    double back_HandsBackHandy  = back_releaseHandy;            //back_pushInHandy + 0.08;
    double back_HandsBackHandz  = back_releaseHandz - 0.18;      //back_pushInHandz - 0.26;
    double TimeBackHandsBack  = 3.0;

    //BACK_HANDSBACK2
    double back_HandsBack2Handx = back_HandsBackHandx + 0.20;//back_pushInHandx + 0.48;
    double back_HandsBack2Handy = back_HandsBackHandy - 0.15;//back_pushInHandy - 0.07;
    double back_HandsBack2Handz = back_HandsBackHandz - 0.10;//back_pushInHandz - 0.36;
    double TimeBackHandsBack2  = 3.0;

    //BACK_TURNTOFRONT
    double TimeTurnFront = 3.0;


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

double PELpos[3];
double RHpos[3];
double LHpos[3];
doubles RHori(4);
doubles LHori(4);
doubles PELori(4);


double WaitTime = 0.;

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
double DESIRED_Gripper = 0.;
int MAX_GRIPPER_CNT = 300;

//Box mass
double boxmass = 5.06;

/***************************** 4. Functions **********************************/
/* Basic */
int  HasAnyOwnership();
int  CheckMotionOwned();
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
void Cart_Supervisor();
void GripperTH();

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
void Set_RHand_Global2Local(vec3 _pos, quat _ori);
void Set_LHand_Global2Local(vec3 _pos, quat _ori);
void SetOriPitch(doubles &target, double _pitch);
void SetOriYaw(doubles &target, double _yaw);
void SetOriHand(doubles &target, double _pitch, double _yaw);
void back_move(double COMx, double COMy, double Pelz, double PelPitch,
               int HandMode, double Handx, double Handy, double Handz,
               double HandOriPitch, double HandOriYaw, double Time);
int  CheckFTsensor();
void SetWaitTime(int mode, double time);
void SetWaitTime_back(int mode, double time);


void zRotation_neg90(double* x, double* y);
void zRotation_90(double* x, double* y);
#endif // LIFTBOX_H
