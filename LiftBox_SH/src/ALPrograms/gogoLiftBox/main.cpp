#include "liftbox.h"
// --------------------------------------------------------------------------------------------- //

#define SAVEN       300
double  SAVE[SAVEN][30000];
bool    FlagSave = false;
bool    FlagZMP  = false;
bool    FlagCOM  = false;
int     FlagSit        = false;
int     FlagSitNreset  = false;
int     Flag_backFront = false;
int     FlagSide       = false;
int     FlagSideNreset = false;
int     SaveIdx  = 0;
void    SaveOneStep(int cnt);
void    SaveAll();

using namespace std;


JointControlClass *joint;
void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    sprintf(__AL_NAME, "gogoLiftBox");
    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == -1 )
        isTerminated = -1;
    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedData, joint);

    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);
        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
            printf("LIFTBOX_COMMAND: %d",sharedData->COMMAND[PODO_NO].USER_COMMAND );

            case LIFTBOX_BACK_MOTION:
            {
                saved_USER_COMMAND = LIFTBOX_BACK_MOTION;

                WB_FLAG = true;
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                FLAG_motionMode = PARTS;
                Mode_LIFTBOX    = MODE_BACK;

                joint->RefreshToCurrentReference();

                mode_prev = MODE_BACK;

                if(FLAG_RESUME)
                {
                    sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] = saved_Command_LIFTBOX;
                    Mode_LIFTBOX = saved_Mode_LIFTBOX;
                    Mode_BACKMOTION = saved_Command_LIFTBOX;
                    FLAG_RESUME = false;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//back
                {
                    ShutDownAllFlag();

                    if(FLAG_RESUME)
                    {
                        FLAG_RESUME = false;
                    }
                    else
                    {
                        StartWBIKmotion(0);
                    }

                    Mode_LIFTBOX = MODE_BACK;
                    Mode_BACKMOTION = BACK_BACK;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//sit
                {
                     Mode_BACKMOTION = BACK_SIT;
                     FlagSit = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//hold
                {
                    Mode_BACKMOTION = BACK_HOLD;
                    Mode_LIFTBOX = MODE_BACK;
                    FlagSit = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//stand
                {
                    Mode_BACKMOTION = BACK_STAND;
                    FlagSit = false;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//front
                {
                     Mode_BACKMOTION = BACK_FRONT;
                     FlagSide = true;
                     Flag_backFront = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 5)//OpenArms
                {
                     Mode_BACKMOTION = BACK_OPENARMS;
                     FlagSide = true;
                     Flag_backFront = false;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 6)//back_pushin
                {
                     Mode_BACKMOTION = BACK_PUSHIN;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 7)//back_handsdown
                {
                     Mode_BACKMOTION = BACK_HANDSDOWN;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 8)//back_release
                {
                     Mode_BACKMOTION = BACK_RELEASE;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 9)//back_handsback
                {
                     Mode_BACKMOTION = BACK_HANDSBACK;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 10)//back_turnToFront
                {
                     Mode_BACKMOTION = BACK_TURNTOFRONT;
                     FlagSide = false;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 11)//BACK_PUSHIN2
                {
                    Mode_BACKMOTION = BACK_PUSHIN2;
                    FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 12)//BACK_HANDSBACK2
                {
                     Mode_BACKMOTION = BACK_HANDSBACK2;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 13)//BACK_PREPUSHIN2
                {
                     Mode_BACKMOTION = BACK_PREPUSHIN2;
                     FlagSide = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 14)//BACK_COM_FORWARD +
                {
                    ShutDownAllFlag();
                    StartWBIKmotion(0);

                    Mode_BACKMOTION = BACK_COM_FORWARD;
                    COMx_shift = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];

                    Mode_LIFTBOX = MODE_BACK;

                    if(FlagSit)
                        FlagSitNreset = true;
                    else if(FlagSide)
                        FlagSideNreset = true;
                    else
                    {
                        FlagSitNreset = false;
                        FlagSideNreset = false;
                    }

                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 15)//BACK_COM_BACKWARD -
                {
                    ShutDownAllFlag();
                    StartWBIKmotion(0);

                    Mode_BACKMOTION = BACK_COM_BACKWARD;
                    COMx_shift = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];

                    Mode_LIFTBOX = MODE_BACK;

                    if(FlagSit)
                        FlagSitNreset = true;
                    else if(FlagSide)
                        FlagSideNreset = true;
                    else
                    {
                        FlagSitNreset = false;
                        FlagSideNreset = false;
                    }

                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 99)//BACK_GETHANDPOS
                {
                    WB_FLAG = false;
                    Mode_BACKMOTION = BACK_GETHANDPOS;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_LIFTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_LIFTBOX received..\n";

                saved_USER_COMMAND = LIFTBOX_BACK_LIFTBOX;

                FLAG_motionMode = LIFT;

                ShutDownAllFlag();

                if(FLAG_RESUME)
                {
                    FILE_LOG(logINFO) << "RESUME LIFTBOX_BACK_LIFTBOX";

                    FLAG_RESUME = false;

                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();

                    Mode_LIFTBOX    = MODE_BACK;
                }else
                {
                    StartWBIKmotion(0);
                    Mode_LIFTBOX    = MODE_BACK;
                    Mode_BACKMOTION = BACK_BACK;
                }


                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_PUTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_PUTBOX received..\n";

                saved_USER_COMMAND = LIFTBOX_BACK_PUTBOX;

                FLAG_motionMode = PUT;

                if(FLAG_RESUME)
                {
                    FILE_LOG(logINFO) << "RESUME LIFTBOX_BACK_LIFTBOX";

                    FLAG_RESUME = false;

                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();

                    Mode_LIFTBOX    = MODE_BACK;
                }
                else
                {
                    Mode_LIFTBOX    = MODE_BACK;
                    Mode_BACKMOTION = BACK_FRONT;
                }


                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_LIFTPUT:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_LIFTPUT received..\n";

                saved_USER_COMMAND = LIFTBOX_BACK_LIFTPUT;
                FLAG_motionMode = LIFTPUT;

                ShutDownAllFlag();

                if(FLAG_RESUME)
                {
                    FILE_LOG(logINFO) << "RESUME LIFT&PUT";

                    FLAG_RESUME = false;

                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();

                    Mode_LIFTBOX    = MODE_BACK;
                }else
                {
                    StartWBIKmotion(0);
                    Mode_LIFTBOX    = MODE_BACK;
                    Mode_BACKMOTION = BACK_BACK;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_TEST:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_TEST received..\n";

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//Add mass
                {
                    WBmotion->kine_drc.m_RightHand += boxmass;
                    WBmotion->kine_drc.m_LeftHand  += boxmass;

                    std::cout << "ReleaseBox.m_RightHand = " << WBmotion->kine_drc.m_RightHand << std::endl;
                    std::cout << "ReleaseBox.m_LeftHand = "  << WBmotion->kine_drc.m_LeftHand  << std::endl;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//Sub mass
                {
                    WBmotion->kine_drc.m_RightHand -= boxmass;
                    WBmotion->kine_drc.m_LeftHand  -= boxmass;

                    std::cout << "ReleaseBox.m_RightHand = " << WBmotion->kine_drc.m_RightHand << std::endl;
                    std::cout << "ReleaseBox.m_LeftHand = "  << WBmotion->kine_drc.m_LeftHand  << std::endl;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//UB_test
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_test received..\n";
                    ShutDownAllFlag();

                    ///StartWBIKmotion(-1); Code inside below
                    joint->RefreshToCurrentReference();                    
                    WBmotion->ResetGlobalCoord(0); // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->StopAll();
                    WBmotion->RefreshToCurrentReference();

                    for(int i = 12; i <= 23; i++)
                        joint->SetMotionOwner(i);
                    for(int i = 25; i < 43; i++)
                        joint->SetMotionOwner(i);

                    WB_FLAG = true;

                    Mode_LIFTBOX    = MODE_TEST;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//get hand Pose
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_test received..\n";

                    joint->RefreshToCurrentReference();
                    WBmotion->ResetGlobalCoord(-1); // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->StopAll();
                    WBmotion->RefreshToCurrentReference();

                    FILE_LOG(logINFO) << "pLH x = " << WBmotion->pLH_3x1[0];
                    FILE_LOG(logINFO) << "pLH y = " << WBmotion->pLH_3x1[1];
                    FILE_LOG(logINFO) << "pLH z = " << WBmotion->pLH_3x1[2];

                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 5)//Get COM
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: Get COM..\n";

                    cout << endl << "in getCOM before refresh"<< endl;
                    cout << "RF x: " << WBmotion->pRF_3x1[0] << endl;
                    cout << "RF y: " << WBmotion->pRF_3x1[1] << endl;
                    cout << "RF z: " << WBmotion->pRF_3x1[2] << endl << endl;

                    joint->RefreshToCurrentReference();
                    WBmotion->ResetGlobalCoord(0); // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->StopAll();
                    WBmotion->RefreshToCurrentReference();

                    FILE_LOG(logINFO) << "COM x = " << WBmotion->pCOM_2x1[0];
                    FILE_LOG(logINFO) << "COM y = " << WBmotion->pCOM_2x1[1];

                    FlagCOM = true;

                    if(FlagSit)
                        FlagSitNreset = true;
                    else if(FlagSide)
                        FlagSideNreset = true;
                    else
                    {
                        FlagSitNreset = false;
                        FlagSideNreset = false;
                    }

                    Mode_LIFTBOX = mode_prev;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 6)//Get ZMP
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: Get ZMP..\n";

                    joint->RefreshToCurrentReference();
                    WBmotion->ResetGlobalCoord(0); // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->StopAll();
                    WBmotion->RefreshToCurrentReference();

                    if(FlagSit)
                        FlagSitNreset = true;
                    else if(FlagSide)
                        FlagSideNreset = true;
                    else
                    {
                        FlagSitNreset = false;
                        FlagSideNreset = false;
                    }

                    Mode_LIFTBOX = mode_prev;

                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 7)//RESET
                {
                    FILE_LOG(logSUCCESS) << "COM shift RESET";
                    sharedData->Flag_COMreset = true;
                    Mode_LIFTBOX = mode_prev;

                    //Turn off all the flags
                    FlagSit         = false;
                    FlagSitNreset   = false;
                    FlagSide        = false;
                    FlagSideNreset  = false;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_UB:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_UB received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                WBmotion->ResetGlobalCoord(-1); // RF_or_LF: 1=LF, -1=RF, 0=PC
                WBmotion->StopAll();
                WBmotion->RefreshToCurrentReference();

                for(int i = 12; i <= 23; i++) joint->SetMotionOwner(i);
                for(int i = 25; i <  43; i++) joint->SetMotionOwner(i);

                WB_FLAG = true;
                FLAG_CART_mode  = false;
                Mode_LIFTBOX    = MODE_UB;


                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//UB_approach
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_approach received..\n";
                    Mode_UBMOTION = UB_APPROACH;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//UB_COM_FOLLOW
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_COM_FOLLOW received..\n";
                    Mode_UBMOTION = UB_COM_FOLLOW;

                    arm_GainOverride(30,10);
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//UB_FOLLOW_DONE
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_FOLLOW_DONE received..\n";
                    Mode_UBMOTION = UB_FOLLOW_DONE;

                    arm_GainOverride(0,1000);
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//UB_HOLDCART
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_HOLDCART received..\n";
                    Mode_UBMOTION = UB_HOLDCART;

                    arm_GainOverride(0,1000);
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//UB_RELEASECART
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_RELEASECART received..\n";
                    Mode_UBMOTION = UB_RELEASECART;

                    arm_GainOverride(0,1000);
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 5)//UB_RELEASECART
                {
                    FILE_LOG(logSUCCESS) << "USER PARAMETER :: UB_RELEASECART received..\n";
                    Mode_UBMOTION = UB_WALKREADYISH;
                }
                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_MOVEBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_MOVEBOX received..\n";

                FLAG_MB_mode = false;

                int temp_Mode_BACKMOTION = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

                if( temp_Mode_BACKMOTION == MOVE_RIGHT
                        || temp_Mode_BACKMOTION == MOVE_RIGHT_WBOX
                        || temp_Mode_BACKMOTION == MOVE_MOVESTART
                        || temp_Mode_BACKMOTION == MOVE_LIFTBOX
                        || temp_Mode_BACKMOTION == MOVE_PUTBOX)
                {
                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();
                    StartWBIKmotion(0);
                }

                Mode_LIFTBOX    = MODE_MOVEBOX;
                Mode_BACKMOTION = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
                std::cout << "MODE_BACKMOTION: " <<  Mode_BACKMOTION << std::endl;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_MOVEBOX_FRONT:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_MOVEBOX_FRONT received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_MOVEBOX_FRONT;
                Mode_MOVEBOX = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_STOP:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_STOP received..\n";

                ShutDownAllFlag();

                Mode_LIFTBOX = MODE_NOTHING;

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//STOP_STOP
                {
                     Mode_BACKMOTION = STOP_STOP;
                     sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//STOP_RESUME
                {
                     Mode_BACKMOTION = STOP_RESUME;

                     FLAG_RESUME = true;

                     //RESUME THE PREVIOUS MODE:
                     sharedData->COMMAND[PODO_NO].USER_COMMAND = saved_USER_COMMAND;
                     Mode_LIFTBOX = saved_Mode_LIFTBOX;
                     Mode_BACKMOTION = saved_Command_LIFTBOX;
                }

                break;
            }
            case LIFTBOX_GRIPPER:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_GRIPPER received..\n";

                FLAG_Gripper  = true;

                joint->RefreshToCurrentReference();
                WBmotion->ResetGlobalCoord(-1);
                joint->SetAllMotionOwner();

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//open
                {
                    MODE_RGripper = GRIPPER_OPEN;
                    MODE_LGripper = GRIPPER_OPEN;
                }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//close_half
                {
                    MODE_RGripper = GRIPPER_CLOSE_HALF;
                    MODE_LGripper = GRIPPER_CLOSE_HALF;
                }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//close
                {
                    MODE_RGripper = GRIPPER_CLOSE;
                    MODE_LGripper = GRIPPER_CLOSE;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_SAVE:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_SAVE received..\n";


                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
                {
                    FlagSave = true;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)
                {
                    FlagSave = false;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            default:
            {
                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            }
    }
    cout << ">>> Process LiftBox is terminated..!!" << endl;
    return 0;
}


//THREAD RBTaskThread
void RBTaskThread(void *)
{
     while(isTerminated == 0)
    {
        LiftBox_Supervisor();
        GripperTH();

        if(WB_FLAG == true)
        {

            // Global whole body model
            WBmotion->updateAll();
            if(Mode_LIFTBOX == MODE_BACK || Mode_LIFTBOX == MODE_MOVEBOX || Mode_LIFTBOX == MODE_TEST)
            {
                if(Mode_BACKMOTION==MOVE_RESET_PARAM){
                    WBmotion->WBIK();
                }
                else{
                    WBmotion->WBIK_BACK();
                }
            }
            else{
                WBmotion->WBIK();
            }

            if(Mode_LIFTBOX == MODE_UB) // MODE_TEST)
            {
                if(!CheckMotionOwned_arm())
                    WB_FLAG = false;
            }
            else if(!CheckMotionOwned())
                WB_FLAG = false;

            for(int i=RHY; i<=LAR; i++)
            {
                joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);
            }

            //joint->SetJointRefAngle
            {
                joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

                joint->SetJointRefAngle(RSP,  WBmotion->Q_filt_34x1[idRSP]*R2D);
                joint->SetJointRefAngle(RSR,  WBmotion->Q_filt_34x1[idRSR]*R2D-OFFSET_RSR);
                joint->SetJointRefAngle(RSY,  WBmotion->Q_filt_34x1[idRSY]*R2D);
                joint->SetJointRefAngle(REB,  WBmotion->Q_filt_34x1[idREB]*R2D-OFFSET_ELB);
                joint->SetJointRefAngle(RWY,  WBmotion->Q_filt_34x1[idRWY]*R2D);
                joint->SetJointRefAngle(RWP,  WBmotion->Q_filt_34x1[idRWP]*R2D);
                joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

                joint->SetJointRefAngle(LSP,  WBmotion->Q_filt_34x1[idLSP]*R2D);
                joint->SetJointRefAngle(LSR,  WBmotion->Q_filt_34x1[idLSR]*R2D-OFFSET_LSR);
                joint->SetJointRefAngle(LSY,  WBmotion->Q_filt_34x1[idLSY]*R2D);
                joint->SetJointRefAngle(LEB,  WBmotion->Q_filt_34x1[idLEB]*R2D-OFFSET_ELB);
                joint->SetJointRefAngle(LWY,  WBmotion->Q_filt_34x1[idLWY]*R2D);
                joint->SetJointRefAngle(LWP,  WBmotion->Q_filt_34x1[idLWP]*R2D);
                joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);
            }

            for(int i=RHY; i<=LAR; i++)
                sharedData->gogo_Reference[i] = WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D;

            //save in SHD MEM
            {
                sharedData->gogo_Reference[WST]  = WBmotion->Q_filt_34x1[idWST] *R2D;

                sharedData->gogo_Reference[RSP]  = WBmotion->Q_filt_34x1[idRSP] *R2D;
                sharedData->gogo_Reference[RSR]  = WBmotion->Q_filt_34x1[idRSR] *R2D-OFFSET_RSR;
                sharedData->gogo_Reference[RSY]  = WBmotion->Q_filt_34x1[idRSY] *R2D;
                sharedData->gogo_Reference[REB]  = WBmotion->Q_filt_34x1[idREB] *R2D-OFFSET_ELB;
                sharedData->gogo_Reference[RWY]  = WBmotion->Q_filt_34x1[idRWY] *R2D;
                sharedData->gogo_Reference[RWP]  = WBmotion->Q_filt_34x1[idRWP] *R2D;
                sharedData->gogo_Reference[RWY2] = WBmotion->Q_filt_34x1[idRWY2]*R2D;

                sharedData->gogo_Reference[LSP]  = WBmotion->Q_filt_34x1[idLSP] *R2D;
                sharedData->gogo_Reference[LSR]  = WBmotion->Q_filt_34x1[idLSR] *R2D-OFFSET_LSR;
                sharedData->gogo_Reference[LSY]  = WBmotion->Q_filt_34x1[idLSY] *R2D;
                sharedData->gogo_Reference[LEB]  = WBmotion->Q_filt_34x1[idLEB] *R2D-OFFSET_ELB;
                sharedData->gogo_Reference[LWY]  = WBmotion->Q_filt_34x1[idLWY] *R2D;
                sharedData->gogo_Reference[LWP]  = WBmotion->Q_filt_34x1[idLWP] *R2D;
                sharedData->gogo_Reference[LWY2] = WBmotion->Q_filt_34x1[idLWY2]*R2D;
            }

        }

        if(FlagZMP)
        {
        }

        if(FlagCOM)
        {
            sharedData->COM_SH_cal[0] = WBmotion->pCOM_2x1[0] + WBmotion->Q0_x;
            sharedData->COM_SH_cal[1] = WBmotion->pCOM_2x1[1] + WBmotion->Q0_y;
        }

        //Saving Button
        if(FlagSave) {

            //FILE WRITING_COM
            FILE *testFile = NULL;
            testFile = fopen("/home/rainbow/Desktop/com.txt","a");
            if((testFile == NULL))
                std::cout << "Failed to open test.txt" << std::endl;
            else{
                for(int i = 0; i < 3; i++){
                    fprintf(testFile,"%f, ",sharedData->COM_SH[i]);
               }
                fprintf(testFile,"\n");
            }
            fclose(testFile);

            //FILE WRITING_COM
            FILE *com_ref = NULL;
            com_ref = fopen("/home/rainbow/Desktop/com_ref.txt","a");
            if((com_ref == NULL))
                std::cout << "Failed to open com_ref.txt" << std::endl;
            else{
                for(int i = 0; i < 3; i++){
                    fprintf(com_ref,"%f, ",sharedData->COM_SH_ref[i]);
               }
                fprintf(com_ref,"\n");
            }
            fclose(com_ref);


            //FILE WRITING_COM_LiftBox
            FILE *lb_comFile = NULL;
            lb_comFile = fopen("/home/rainbow/Desktop/lb_com.txt","a");
            if((lb_comFile == NULL))
                std::cout << "Failed to open lb_com.txt" << std::endl;
            else{
                for(int i = 0; i < 2; i++){
                    fprintf(lb_comFile,"%f, ",WBmotion->pCOM_2x1[i]);
               }
                fprintf(lb_comFile,"%f, ",WBmotion->pPelZ);
                fprintf(lb_comFile,"\n");
            }
            fclose(lb_comFile);

            //FILE WRITING_COM_local
            FILE *local_comFile = NULL;
            local_comFile = fopen("/home/rainbow/Desktop/com_local.txt","a");
            if((local_comFile == NULL))
                std::cout << "Failed to open com_local.txt" << std::endl;
            else{
                for(int i = 0; i < 3; i++){
                    fprintf(local_comFile,"%f, ",sharedData->COM_m_local[i]);
               }
                fprintf(local_comFile,"\n");
            }
            fclose(local_comFile);

            //FILE WRITING_daemon reference
            FILE *daemonFile = NULL;
            daemonFile = fopen("/home/rainbow/Desktop/daemon_ref.txt","a");
            if((daemonFile == NULL))
                std::cout << "Failed to open daemon_ref.txt" << std::endl;
            else{
                for(int joint_num = RHY; joint_num <= LHAND; joint_num++){
                    fprintf(daemonFile,"%f, ",sharedData->ENCODER[MC_GetID(joint_num)][MC_GetCH(joint_num)].CurrentPosition);
                }
                fprintf(daemonFile,"\n");
            }
            fclose(daemonFile);

            //FILE WRITING_gogoReference
            FILE *gogoFile = NULL;
            gogoFile = fopen("/home/rainbow/Desktop/gogo_ref.txt","a");
            if((gogoFile == NULL))
                std::cout << "Failed to open gogo_ref.txt" << std::endl;
            else{
                for(int i=RHY; i<=LWP; i++)
                {
                    fprintf(gogoFile,"%lf, ", sharedData->gogo_Reference[i]);
                }
                fprintf(gogoFile,"\n");
            }
            fclose(gogoFile);

            //FILE WRITING_HBReference
            FILE *HBFile = NULL;
            HBFile = fopen("/home/rainbow/Desktop/HB_ref.txt","a");
            if((HBFile == NULL))
                std::cout << "Failed to open HB_ref.txt" << std::endl;
            else{
                for(int i=RHY; i<=LAR; i++)
                {
                    fprintf(HBFile,"%lf, ", sharedData->HBWK_Reference[i]);
                }
                fprintf(HBFile,"\n");
            }
            fclose(HBFile);

            //FILE WRITING_FTSensor
            FILE *FTFile = NULL;
            FTFile = fopen("/home/rainbow/Desktop/FT_sensor.txt","a");
            if((FTFile == NULL))
                std::cout << "Failed to open FT_sensor.txt" << std::endl;
            else{
                for(int i=0; i<=3; i++)
                {
                    //0:RAFT, 1:LAFT, ...
                    fprintf(FTFile,"%lf, %lf, %lf, %lf, %lf, %lf, ", sharedData->FT[i].Fx, sharedData->FT[i].Fy, sharedData->FT[i].Fz,sharedData->FT[i].Mx,sharedData->FT[i].My,sharedData->FT[i].Mz);
                }
                fprintf(FTFile,"\n");
            }
            fclose(FTFile);


            //FILE WRITING_ZMP
            FILE *zmpFile = NULL;
            ZMP_calc_SH();

            zmpFile = fopen("/home/rainbow/Desktop/zmp.txt","a");
            if((zmpFile == NULL))
                std::cout << "Failed to open zmp.txt" << std::endl;
            else{
                for(int i=0; i<3; i++)
                {
                    fprintf(zmpFile, "%lf,", sharedData->zmp_HBWalk_com[i]);
                }
                fprintf(zmpFile,"\n");
            }
            fclose(zmpFile);


            //FILE WRITING_ZMP_REF
            FILE *zmpRefFile = NULL;
            zmpRefFile = fopen("/home/rainbow/Desktop/zmp_ref.txt","a");
            if((zmpRefFile == NULL))
                std::cout << "Failed to open zmp_ref.txt" << std::endl;
            else{
                for(int i=0; i<3; i++)
                {
                    fprintf(zmpRefFile, "%lf,", sharedData->zmp_HBWalk_com_ref[i]);
                }
                fprintf(zmpRefFile,"\n");
            }
            fclose(zmpRefFile);

            //FILE WRITING_IMU
            FILE *imuFile = NULL;
            imuFile = fopen("/home/rainbow/Desktop/imu.txt","a");
            if((imuFile == NULL))
                std::cout << "Failed to open imu.txt" << std::endl;
            else{
                fprintf(imuFile, "%lf,", sharedData->IMU[0].Roll);
                fprintf(imuFile, "%lf,", sharedData->IMU[0].Pitch);
                fprintf(imuFile, "%lf,", sharedData->IMU[0].RollVel);
                fprintf(imuFile, "%lf,", sharedData->IMU[0].PitchVel);
                fprintf(imuFile, "%lf,", sharedData->IMU[0].Roll_Acc);
                fprintf(imuFile, "%lf,", sharedData->IMU[0].Pitch_Acc);
                fprintf(imuFile,"\n");
            }
            fclose(imuFile);


            //FILE CommandInput
            FILE *CommandFile = NULL;
            CommandFile = fopen("/home/rainbow/Desktop/commandIn.txt","a");
            if((CommandFile == NULL))
                std::cout << "Failed to open commandIn.txt" << std::endl;
            else{
                fprintf(CommandFile,"%d, %d\n", command_FLAG, command_num);
                command_FLAG = false;
            }
            fclose(CommandFile);

            //FILE EndEffector
            FILE *EEFile = NULL;
            EEFile = fopen("/home/rainbow/Desktop/endEffector.txt","a");
            if((EEFile == NULL))
                std::cout << "Failed to open endEffector.txt" << std::endl;
            else{
                for(int i = 0; i < 3; i++)
                    fprintf(EEFile,"%lf, ", WBmotion->pRH_3x1[i]);
                for(int i = 0; i < 3; i++)
                    fprintf(EEFile,"%lf, ", WBmotion->pLH_3x1[i]);
                fprintf(EEFile,"\n");
            }
            fclose(EEFile);


        }

        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 500*1000);

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);
        if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
            joint->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }

    }
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!= PODO_NO)	return 0;
    }
    return 1;
}
int CheckMotionOwned_arm()
{
    for(int i=RSP;i<=LWP;i++)
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!= PODO_NO)	return 0;
    for(int i=RWY2;i<NO_OF_JOINTS;i++)
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!= PODO_NO)	return 0;
    return 1;

}
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int RBInitialize(void)
{
    // Block program termination
    isTerminated = 0;

    char task_thread_name[30];
    char flag_thread_name[30];
    sprintf(task_thread_name, "%s_TASK", __AL_NAME);
    sprintf(flag_thread_name, "%s_FLAG", __AL_NAME);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            return false;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================


    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return false;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return false;
            }
        }
        memset(userData, 0, sizeof(USER_SHM));
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedData, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, flag_thread_name, 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(3, &aCPU);
        if(rt_task_set_affinity(&rtFlagCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Flag real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtFlagCon, &RBFlagThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Flag real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Flag real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }

    if(rt_task_create(&rtTaskCon, task_thread_name, 0, 90, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Task real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBTaskThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Task real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Task real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}

void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    // RF_or_LF: 1=LF, -1=RF, 0=PC
    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

void ShutDownAllFlag()
{
    Command_LIFTBOX = NOTHING;
    Mode_LIFTBOX = MODE_NOTHING;
    WB_FLAG = false;
}

void ShowWBInfos()
{
    printf("======================= WBInfos =======================\n");
    printf("LHPos  = (%f, %f, %f)\n",    WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2]);
    printf("LHOri  = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0], WBmotion->qLH_4x1[1], WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    printf("RHPos  = (%f, %f, %f)\n",    WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
    printf("RHOri  = (%f, %f, %f, %f)\n",WBmotion->qRH_4x1[0], WBmotion->qRH_4x1[1], WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    printf("PelPos = (%f, %f, %f)\n",    WBmotion->pCOM_2x1[0],WBmotion->pCOM_2x1[1],WBmotion->pPelZ);
    printf("PelOri = (%f, %f, %f, %f)\n",WBmotion->qPEL_4x1[0],WBmotion->qPEL_4x1[1],WBmotion->qPEL_4x1[2],WBmotion->qPEL_4x1[3]);
    printf("=======================================================\n\n");
}

/* ************LIFTBOX_SUPERVISOR*************** */
/* ************LIFTBOX_SUPERVISOR*************** */
/* ************LIFTBOX_SUPERVISOR*************** */
void LiftBox_Supervisor()
{
    switch(Mode_LIFTBOX)
    {
        case MODE_BACK:
        {
            saved_Mode_LIFTBOX = MODE_BACK;

            switch(Mode_BACKMOTION)
            {
                case BACK_WAIT:
                {
                    WaitCount++;

                    if(WaitCount > WaitTime*200)
                    {
                        switch(WaitMode)
                        {
                            case BACK_BACK:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_BACK OK";
                                Mode_BACKMOTION = BACK_OPENARMS;
                                break;
                            }
                            case BACK_OPENARMS:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_OPENARMS OK";
                                Mode_BACKMOTION = BACK_SIT;
                                break;
                            }
                            case BACK_SIT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_SIT OK";
                                Mode_BACKMOTION = BACK_HOLD;
                                break;
                            }
                            case BACK_HOLD:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HOLD OK";
                                Mode_BACKMOTION = BACK_LIFT;
                                break;
                            }
                            case BACK_LIFT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_LIFT OK";
                                Mode_BACKMOTION = BACK_STAND;
                                break;
                            }
                            case BACK_STAND:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_STAND OK";
                                Mode_BACKMOTION = BACK_FRONT;
                                break;
                            }
                            case BACK_FRONT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_FRONT OK";
                                Mode_BACKMOTION = BACK_PUSHIN;
                                break;
                            }
                            case BACK_PUSHIN:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_PUSHIN OK";
                                Mode_BACKMOTION = BACK_HANDSDOWN;
                                break;
                            }
                            case BACK_HANDSDOWN:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HANDSDOWN OK";
                                Mode_BACKMOTION = BACK_RELEASE;
                                break;
                            }
                            case BACK_RELEASE:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_RELEASE OK";
                                Mode_BACKMOTION = BACK_HANDSBACK;
                                break;
                            }
                            case BACK_HANDSBACK:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HANDSBACK OK";
                                Mode_BACKMOTION = BACK_HANDSBACK2;
                                break;
                            }
                            case BACK_PREPUSHIN2:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_PREPUSHIN2 OK";
                                Mode_BACKMOTION = BACK_PUSHIN2;
                                break;
                            }
                            case BACK_PUSHIN2:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_PUSHIN2 OK";
                                Mode_BACKMOTION = BACK_HANDSBACK2;
                                break;
                            }
                            case BACK_HANDSBACK2:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HANDSBACK2 OK";
                                Mode_BACKMOTION = BACK_TURNTOFRONT;
                                break;
                            }
                        }
                        WaitCount = 0;
                    }
                    break;
                }
                case BACK_BACK:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_BACK";

                    command_num  = BACK_BACK;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_BACK;
                    WBmotion->addWSTPosInfo(-180.0, 4.0);


                    double COMx = WBmotion->pCOM_2x1[0] + in.back_backCOMshift;//-0.01;
                    double COMy = WBmotion->pCOM_2x1[1];

                    PELpos[0] = COMx;
                    PELpos[1] = COMy;
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 4.0);

                    //Turn off all the flags
                    FlagSit         = false;
                    FlagSitNreset   = false;
                    FlagSide        = false;
                    FlagSideNreset  = false;

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_BACK, 4.0);
                    break;
                }
                case BACK_OPENARMS:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_OPENARMS";

                    command_num  = BACK_OPENARMS;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_OPENARMS;

                    double COMx         = WBmotion->pCOM_2x1[0];// + 0.01;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = WBmotion->pPelZ;;
                    double PelPitch     = in.back_sitPelPitch;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_openArmsHandx;
                    double Handy        = in.back_openArmsHandy;
                    double Handz        = in.back_openArmsHandz;
                    double HandOriPitch = 0.;
                    double HandOriYaw   = 0.;
                    double Time         = in.TimeBackOpenArms;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_OPENARMS,in.TimeBackOpenArms);
                    break;
                }
                case BACK_SIT:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_SIT";

                    command_num  = BACK_SIT;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_SIT;

                    double COMx         = WBmotion->pCOM_2x1[0] + in.back_sitCOMshift; //-0.005;//+0.01;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = in.back_sitPelz;
                    double PelPitch     = in.back_sitPelPitch;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_sitHandx;
                    double Handy        = in.back_sitHandy;
                    double Handz        = in.back_sitHandz;
                    double HandOriPitch = 0.;
                    double HandOriYaw   = 0.;
                    double Time         = in.TimeSit;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_SIT, in.TimeSit);
                    break;
                }
                case BACK_HOLD:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_HOLD";

                    command_num  = BACK_HOLD;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_HOLD;

                    //Current hand pose
                    RHpos[0] = RHpos[0];    RHpos[1] = RHpos[1];    RHpos[2] = RHpos[2];
                    LHpos[0] = LHpos[0];    LHpos[1] = LHpos[1];    LHpos[2] = LHpos[2];

                    RHpos[1] += in.back_holdHandy;
                    LHpos[1] -= in.back_holdHandy;

                    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeBackHold);
                    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeBackHold);

                    WBmotion->kine_drc.m_RightHand += boxmass;
                    WBmotion->kine_drc.m_LeftHand  += boxmass;

                    //SET NEXT MODE
                    if(FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_HOLD, in.TimeBackHold);

                    break;
                }
                case BACK_LIFT:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_LIFT";

                    command_num  = BACK_LIFT;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_LIFT;

                    double COMx         = WBmotion->pCOM_2x1[0] + in.back_liftCOMshift;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_liftHandx;
                    double Handy        = LHpos[1];
                    double Handz        = in.back_liftHandz;
                    double HandOriPitch = 0.0;
                    double HandOriYaw   = 0.0;
                    double Time         = in.TimeBackLift;

                    if(FlagSitNreset)
                        Handz = in.back_liftHandz + in.back_sitPelz;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);


                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_LIFT, in.TimeBackLift);
                    break;
                }
                case BACK_STAND:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_STAND";

                    command_num  = BACK_STAND;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_STAND;

                    double COMx         = WBmotion->pCOM_2x1[0] + in.back_standCOMshift;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = 0.0;
                    double PelPitch     = 0.;
                    int    HandMode     = HAND_GLOBAL;
                    double Handx        = in.back_stand2Handx;
                    double Handy        = RHpos[1];
                    double Handz        = in.back_stand2Handz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   = 0.0;
                    double Time         = in.TimeStand;

                    if(FlagSitNreset)
                        Pelz = -in.back_sitPelz;
                    FlagSitNreset = false;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT || FLAG_motionMode == PUT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_STAND, in.TimeStand);
                    break;
                }
                case BACK_FRONT:    //90deg Turn this time
                {
                    FILE_LOG(logSUCCESS) << "case BACK_FRONT";


                    command_num  = BACK_FRONT;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_FRONT;

                    WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeStand);
                    WBmotion->addWSTPosInfo(-90.0, 5.0);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_FRONT, 5.0);
                    break;
                }
                case BACK_PUSHIN:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_PUSHIN";

                    saved_Command_LIFTBOX = BACK_PUSHIN;

                    command_num  = BACK_PUSHIN;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_pushInHandx;
                    double Handy        = in.back_pushInHandy;
                    double Handz        = in.back_pushInHandz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   =   0.0;
                    double Time         = in.TimeBackPushIn;

                    //Debug
                    std::cout << std::endl << "in BACK_PUSHIN" << std::endl;
                    std::cout << "RF x: " << WBmotion->pRF_3x1[0] << std::endl;
                    std::cout << "RF y: " << WBmotion->pRF_3x1[1] << std::endl;
                    std::cout << "RF z: " << WBmotion->pRF_3x1[2] << std::endl;

                    if(FlagSideNreset)
                    {
                        std::cout << "FlagSideNreset" << std::endl;
                        Handx = Handx ;//+ side_pely;
                        Handy = Handy ;//+ side_pelx;
                    }

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_PUSHIN, in.TimeBackPushIn);
                    break;
                }
                case BACK_HANDSDOWN:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_HANDSDOWN";

                    command_num  = BACK_HANDSDOWN;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_HANDSDOWN;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_handsDownHandx;
                    double Handy        = in.back_handsDownHandy;
                    double Handz        = in.back_handsDownHandz;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = in.TimeHandsDown;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_HANDSDOWN, in.TimeHandsDown);
                    break;
                }
                case BACK_RELEASE:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_RELEASE";

                    command_num  = BACK_RELEASE;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_RELEASE;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_releaseHandx;
                    double Handy        = in.back_releaseHandy;
                    double Handz        = in.back_releaseHandz;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = in.TimeHandsDown;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    WBmotion->kine_drc.m_RightHand -= boxmass;
                    WBmotion->kine_drc.m_LeftHand  -= boxmass;

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_RELEASE, in.TimeHandsDown);
                    break;
                }
                case BACK_HANDSBACK:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_HANDSBACK";

                    command_num  = BACK_HANDSBACK;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_HANDSBACK;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_HandsBackHandx;
                    double Handy        = in.back_HandsBackHandy;
                    double Handz        = in.back_HandsBackHandz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   =   0.0;
                    double Time         = in.TimeBackHandsBack;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_HANDSBACK, in.TimeBackHandsBack);
                    break;
                }
                case BACK_PREPUSHIN2:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_PREPUSHIN2";

                    command_num  = BACK_PREPUSHIN2;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_PREPUSHIN2;

                    FILE_LOG(logSUCCESS) << "prePushIn";
                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_HandsBackHandx;
                    double Handy        = in.back_HandsBackHandy - 0.15;
                    double Handz        = in.back_HandsBackHandz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   =   0.0;
                    double Time         = in.TimeBackPushIn2;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_PREPUSHIN2, in.TimeBackPushIn2);
                    break;
                }
                case BACK_PUSHIN2:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_PUSHIN2";


                    command_num  = BACK_PUSHIN2;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_PUSHIN2;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_pushIn2Handx;
                    double Handy        = in.back_pushIn2Handy;
                    double Handz        = in.back_pushIn2Handz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   =   0.0;
                    double Time         = in.TimeBackPushIn2;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_PUSHIN2, in.TimeBackPushIn2);
                    break;
                }
                case BACK_HANDSBACK2:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_HANDSBACK2";


                    command_num  = BACK_HANDSBACK2;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_HANDSBACK2;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_HandsBack2Handx;
                    double Handy        = in.back_HandsBack2Handy;
                    double Handz        = in.back_HandsBack2Handz;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   =   0.0;
                    double Time         = in.TimeBackHandsBack2;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    //SET NEXT MODE
                    if(FLAG_motionMode == PARTS || FLAG_motionMode == LIFT)
                        Mode_BACKMOTION = BACK_NOTHING;
                    else
                        SetWaitTime_back(BACK_HANDSBACK2, in.TimeBackHandsBack2);
                    break;
                }
                case BACK_TURNTOFRONT:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_TURNTOFRONT";


                    command_num  = BACK_TURNTOFRONT;
                    command_FLAG = true;

                    saved_Command_LIFTBOX = BACK_TURNTOFRONT;

                    WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_turnToFrontCOMx, WBmotion->pCOM_2x1[1], in.TimeTurnFront);
                    WBmotion->addWSTPosInfo(0.0, 5.0);

                    Mode_BACKMOTION = BACK_NOTHING;
                    break;
                }
                case BACK_COM_FORWARD: // +
                {
                    FILE_LOG(logSUCCESS) << "case MOVE_COM_FORWARD";

                    command_num  = BACK_COM_FORWARD;
                    command_FLAG = true;

                    if(COMx_shift <= 0.03)
                    {
                        PELpos[0] = WBmotion->pCOM_2x1[0] + COMx_shift;
                        PELpos[1] = WBmotion->pCOM_2x1[1];
                        std::cout << "TEST PELpos[0] = " << PELpos[0] << std::endl;
                        WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);
                    }
                    else
                    {
                        FILE_LOG(logERROR) << "Out of Range. Try smaller shift than 0.03";
                    }

                    COMx_shift = 0.0;

                    Mode_BACKMOTION = BACK_NOTHING;
                    break;
                }
                case BACK_COM_BACKWARD: // -
                {
                    FILE_LOG(logSUCCESS) << "case BACK_COM_BACKWARD";

                    if(COMx_shift <= 0.015)
                    {
//                        std::cout << "COMx shift = -" << COMx_shift << std::endl;
                        PELpos[0] = WBmotion->pCOM_2x1[0] - COMx_shift;
                        PELpos[1] = WBmotion->pCOM_2x1[1];

                        std::cout << "TEST PELpos[0] = " << PELpos[0] << std::endl;
                        WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);
                    }
                    else
                    {
                        FILE_LOG(logERROR) << "Out of Range. Try smaller shift than 0.015";
                    }

                    COMx_shift = 0.0;

                    Mode_BACKMOTION = BACK_NOTHING;
                    break;
                }
                case BACK_GETHANDPOS:
                {
                    FILE_LOG(logSUCCESS) << "case BACK_GETHANDPOS";

                    std::cout << "GET HAND POSITION>>>" << std::endl;

                    std::cout << "COM x = " << WBmotion->pCOM_2x1[0] << std::endl;
                    std::cout << "COM y = " << WBmotion->pCOM_2x1[1] << std::endl;
                    std::cout << "COM z = " << WBmotion->pPelZ       << std::endl;

                    std::cout << "RH(x,y,z) = (" <<  WBmotion->pRH_3x1[0] <<", "<<   WBmotion->pRH_3x1[1] <<", "<<   WBmotion->pRH_3x1[2] << ")"<< std::endl;
                    std::cout << "LH(x,y,z) = (" <<  WBmotion->pLH_3x1[0] <<", "<<   WBmotion->pLH_3x1[1] <<", "<<   WBmotion->pLH_3x1[2] << ")"<< std::endl;

                    Mode_BACKMOTION = BACK_NOTHING;
                    break;
                }
            }
            break;
        }
        case MODE_MOVEBOX:
        {
            if(Mode_BACKMOTION != MOVE_NOTHING)
            {

            }

            switch(Mode_BACKMOTION)
            {
                case MOVE_WAIT:
                {
                    WaitCount++;

                    if(WaitCount > WaitTime*200)
                    {
                        switch(WaitMode)
                        {
                            case MOVE_LEFT_COMBACK:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_LEFT_COMBACK OK";
                                Mode_BACKMOTION = MOVE_LEFT_COMBACK2;
                                break;
                            }
                            case MOVE_LEFT_COMBACK2:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_LEFT_COMBACK2 OK";
                                Mode_BACKMOTION = MOVE_LEFT_COMBACK3;
                                break;
                            }
                            case MOVE_COM_5BACKWARD:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_COM_5BACKWARD OK";
                                Mode_BACKMOTION = MOVE_RIGHT;
                                break;
                            }
                            case MOVE_RIGHT:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_RIGHT OK";
                                Mode_BACKMOTION = MOVE_OPENARMS;
                                break;
                            }
                            case MOVE_OPENARMS:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_OPENARMS OK";
                                Mode_BACKMOTION = MOVE_APPROACH;
                                break;
                            }
                            case MOVE_APPROACH:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_APPROACH OK";
                                Mode_BACKMOTION = MOVE_HOLD;
                                break;
                            }
                            case MOVE_HOLD:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_HOLD OK";
                                Mode_BACKMOTION = MOVE_LIFT;
                                break;
                            }
                            case MOVE_LIFT:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_LIFT OK";
                                Mode_BACKMOTION = MOVE_LIFT2;
                                break;
                            }
                            case MOVE_LIFT2:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_LIFT2 OK";
                                Mode_BACKMOTION = MOVE_LEFT;
                                break;
                            }
                            case MOVE_COM_5BACKWARD_ABS:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_COM_5BACKWARD_ABS OK";
                                Mode_BACKMOTION = MOVE_RIGHT_WBOX;
                                break;
                            }
                            case MOVE_RIGHT_WBOX:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_RIGHT_WBOX OK";
                                Mode_BACKMOTION = MOVE_PUTIN;
                                break;
                            }
                            case MOVE_PUTIN:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_PUTIN OK";
                                Mode_BACKMOTION = MOVE_PUTIN2;
                                break;
                            }
                            case MOVE_PUTIN2:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_PUTIN2 OK";
                                Mode_BACKMOTION = MOVE_RELEASE;
                                break;
                            }
                            case MOVE_RELEASE:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_RELEASE OK";
                                Mode_BACKMOTION = MOVE_HANDSBACK;
                                break;
                            }
                            case MOVE_HANDSBACK:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_HANDSBACK OK";
                                Mode_BACKMOTION = MOVE_WALKREADYISH;
                                break;
                            }
                            case MOVE_WALKREADYISH:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_WALKREADYISH OK";
                                Mode_BACKMOTION = MOVE_LEFTTOFRONT;
                                break;
                            }
                            case MOVE_LEFTTOFRONT:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_LEFTTOFRONT OK";
                                Mode_BACKMOTION = MOVE_NOTHING;
                                FLAG_MB_mode = false;

                                break;
                            }
                            case MOVE_PRETURN:
                            {
                                FILE_LOG(logSUCCESS) << "MOVE_PRETURN OK";
                                if(FLAG_MB_mode == MOVEMODE_LIFT)
                                    Mode_BACKMOTION = MOVE_COM_5BACKWARD;
                                else if(FLAG_MB_mode == MOVEMODE_PUT)
                                    Mode_BACKMOTION = MOVE_RIGHT_WBOX;
                                break;
                            }
                        }
                        WaitCount = 0;
                    }
                    break;
                }
                case MOVE_RESET_PARAM:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_RESET_PARAM\n";

                    double COMx         = WBmotion->pCOM_2x1[0];
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = WBmotion->pPelZ;

                    WBmotion->addCOMInfo(COMx, COMy, 1.0);
                    WBmotion->addPELPosInfo(Pelz, 1.0);

                    Mode_BACKMOTION = MOVE_NOTHING;

                    break;
                }
                case MOVE_PRETURN:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_PRETURN\n";

                    command_num  = MOVE_PRETURN;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_preTurnHandx;
                    double Handy        = move_in.move_preTurnHandy;
                    double Handz        = move_in.move_preTurnHandz;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMovePreTurn;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_PRETURN,move_in.TimeMovePreTurn);

                    break;

                }
                case MOVE_RIGHT:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_RIGHT\n";

                    command_num  = MOVE_RIGHT;
                    command_FLAG = true;


                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();
                    StartWBIKmotion(0);


                    WBmotion->addWSTPosInfo(-90.0, move_in.TimeMoveRight);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT){
                        SetWaitTime_move(MOVE_RIGHT,move_in.TimeMoveRight);
                    }

                    break;
                }
                case MOVE_RIGHT_WBOX:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_RIGHT_WBOX\n";

                    command_num  = MOVE_RIGHT_WBOX;
                    command_FLAG = true;

                    WBmotion->addWSTPosInfo(90.0, move_in.TimeMoveRightWBox);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_RIGHT_WBOX,move_in.TimeMoveRightWBox);
                    break;

                }
                case MOVE_OPENARMS:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_OPENARMS\n";

                    command_num  = MOVE_OPENARMS;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_openArmsHandx; //-0.30;
                    double Handy        = move_in.move_openArmsHandy; //0.40;
                    double Handz        = move_in.move_openArmsHandz; //1.0;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveOpenArms;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_OPENARMS,move_in.TimeMoveOpenArms);

                    break;
                }
                case MOVE_APPROACH:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_APPROACH\n";

                    command_num  = MOVE_APPROACH;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_approachHandx;//-0.45;
                    double Handy        = move_in.move_approachHandy;// 0.40;
                    double Handz        = move_in.move_approachHandz;// 1.0;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveApproach;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_APPROACH,move_in.TimeMoveApproach);

                    break;
                }
                case MOVE_HOLD:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_HOLD\n";

                    command_num  = MOVE_HOLD;
                    command_FLAG = true;

                    //Current hand pose
                    RHpos[0] = RHpos[0];    RHpos[1] = RHpos[1];    RHpos[2] = RHpos[2];
                    LHpos[0] = LHpos[0];    LHpos[1] = LHpos[1];    LHpos[2] = LHpos[2];

                    RHpos[1] += move_in.move_HoldHandy;//in.back_holdHandy;
                    LHpos[1] -= move_in.move_HoldHandy;//in.back_holdHandy;

                    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], move_in.TimeMoveHold);//in.TimeBackHold);
                    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], move_in.TimeMoveHold);//in.TimeBackHold);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_HOLD,move_in.TimeMoveHold);

                    break;
                }
                case MOVE_LIFT:
                {

                    command_num  = MOVE_LIFT;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_liftHandx;
                    double Handy        = move_in.move_liftHandy;
                    double Handz        = move_in.move_liftHandz;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveLift;//in.TimeBackLift;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    SetWaitTime_move(MOVE_LIFT, move_in.TimeMoveLift);
                    break;
                }
                case MOVE_LIFT2:
                {

                    command_num  = MOVE_LIFT2;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_lift2Handx; //-0.40;
                    double Handy        = move_in.move_lift2Handy; // 0.32;
                    double Handz        = move_in.move_lift2Handz; // 1.2;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveLift2;//in.TimeBackLift;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_LIFT2,move_in.TimeMoveLift2);
                    break;
                }
                case MOVE_LEFT:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_LEFT\n";

                    command_num  = MOVE_LEFT;
                    command_FLAG = true;

                    WBmotion->addWSTPosInfo(0.0, move_in.TimeMoveLeft);
                    Mode_BACKMOTION = MOVE_NOTHING;
                    break;
                }
                case MOVE_LEFT_COMBACK:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_LEFT_COMBACK\n";

                    command_num  = MOVE_LEFT_COMBACK;
                    command_FLAG = true;

                    PELpos[0] = WBmotion->pCOM_2x1[0];// - 0.05;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 3.0);

                    WBmotion->addWSTPosInfo(0.0, 3.0);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    break;
                }
                case MOVE_LEFT_COMBACK2:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_LEFT_COMBACK2\n";

                    command_num  = MOVE_LEFT_COMBACK2;
                    command_FLAG = true;

                    WB_FLAG = true;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();
                    StartWBIKmotion(0);

                    SetWaitTime_move(MOVE_LEFT_COMBACK2, 0.1);
                    break;
                }
                case MOVE_LEFT_COMBACK3:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_LEFT_COMBACK3\n";

                    command_num  = MOVE_LEFT_COMBACK3;
                    command_FLAG = true;

                    PELpos[0] = WBmotion->pCOM_2x1[0] - 0.03;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    break;
                }
                case MOVE_PUTIN:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_PUTIN\n";

                    command_num  = MOVE_PUTIN;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_putInHandx;; //-0.8;
                    double Handy        = move_in.move_putInHandy;  // 0.32;
                    double Handz        = move_in.move_putInHandz;; // 1.15;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMovePutIn;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    SetWaitTime_move(MOVE_PUTIN,move_in.TimeMovePutIn);

                    break;
                }
                case MOVE_PUTIN2:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_PUTIN2\n";

                    command_num  = MOVE_PUTIN2;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_putIn2Handx; //-0.8;
                    double Handy        = move_in.move_putIn2Handy; // 0.32;
                    double Handz        = move_in.move_putIn2Handz; // 1.02;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMovePutIn2;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_PUTIN2,move_in.TimeMovePutIn2);

                    break;
                }
                case MOVE_RELEASE:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_RELEASE\n";

                    command_num  = MOVE_RELEASE;
                    command_FLAG = true;


                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_releaseHandx; //-0.45;
                    double Handy        = move_in.move_releaseHandy; // 0.40;
                    double Handz        = move_in.move_releaseHandz; // 1.0;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveRelease;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_RELEASE,move_in.TimeMoveRelease);

                    break;
                }
                case MOVE_HANDSBACK:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_HANDSBACK\n";

                    command_num  = MOVE_HANDSBACK;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_handsBackHandx;//-0.30;
                    double Handy        = move_in.move_handsBackHandy;// 0.40;
                    double Handz        = move_in.move_handsBackHandz;// 1.0;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveHandsBack;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_HANDSBACK,move_in.TimeMoveHandsBack);

                    break;
                }
                case MOVE_WALKREADYISH:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_WALKREADYISH\n";

                    command_num  = MOVE_WALKREADYISH;
                    command_FLAG = true;

                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = 0.0;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = move_in.move_walkReadyishHandx;//-0.30;
                    double Handy        = move_in.move_walkReadyishHandy;// 0.25;
                    double Handz        = move_in.move_walkReadyishHandz;// 1.05;
                    double HandOriPitch = NO_USE;
                    double HandOriYaw   = NO_USE;
                    double Time         = move_in.TimeMoveWalkReadyish;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_WALKREADYISH,move_in.TimeMoveWalkReadyish);

                    break;
                }                
                case MOVE_LEFTTOFRONT:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_LEFTTOFRONT\n";

                    command_num  = MOVE_LEFTTOFRONT;
                    command_FLAG = true;

                    WBmotion->addWSTPosInfo(0.0, move_in.TimeMoveLeftToFront);


                    PELpos[0] = WBmotion->pCOM_2x1[0] + 0.03;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_LEFTTOFRONT,move_in.TimeMoveLeftToFront);

                    break;
                }
                case MOVE_GET_POSE:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_GET_POSE\n";

                    command_num  = MOVE_GET_POSE;
                    command_FLAG = true;

                    cout << "COM  x = " << WBmotion->pCOM_2x1[0]<< endl;
                    cout << "COM  y = " << WBmotion->pCOM_2x1[1]<< endl;

                    cout << "Pel  y = " << WBmotion->pPelY      << endl;
                    cout << "Pel  z = " << WBmotion->pPelZ      << endl;

                    cout << "Hand y = " << WBmotion->pLH_3x1[1] << endl;

                    Mode_BACKMOTION = MOVE_NOTHING;
                    break;
                    }
                case MOVE_COM_5FORWARD:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_COM_5FORWARD\n";

                    PELpos[0] = WBmotion->pCOM_2x1[0] + 0.03;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    break;
                }
                case MOVE_COM_5BACKWARD:
                {
                    FILE_LOG(logSUCCESS) << "--> MOVE_COM_5BACKWARD\n";

                    PELpos[0] = WBmotion->pCOM_2x1[0] - 0.03;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 1.5);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_LIFT)
                        SetWaitTime_move(MOVE_COM_5BACKWARD,1.5);
                    break;
                }                
                case MOVE_COM_5BACKWARD_ABS:
                {

                    command_num  = MOVE_COM_5BACKWARD_ABS;
                    command_FLAG = true;


                    FILE_LOG(logSUCCESS) << "--> MOVE_COM_5BACKWARD_ABS\n";

                    command_num  = MOVE_GET_POSE;
                    command_FLAG = true;


                    double COMx = 0.06;
                    double COMy = 0.0;
                    WBmotion->addCOMInfo(COMx, COMy, 1.0);

                    Mode_BACKMOTION = MOVE_NOTHING;
                    if(FLAG_MB_mode == MOVEMODE_PUT)
                        SetWaitTime_move(MOVE_COM_5BACKWARD_ABS,1.0);
                    break;
                }
                case MOVE_LIFTBOX:
                {

                    command_num  = MOVE_LIFTBOX;
                    command_FLAG = true;

                    double COMx  = WBmotion->pCOM_2x1[0];
                    double COMy  = WBmotion->pCOM_2x1[1];
                    double Handx = WBmotion->pLH_3x1[0];
                    double Handy = WBmotion->pLH_3x1[1];
                    double Handz = WBmotion->pLH_3x1[2];


                    cout << "COM(x,y) = (" << COMx << ", " << COMy << ")" << endl;
                    cout << "Hand(x,y,z) = (" << Handx << ", " << Handy <<", " << Handz << ")" << endl;

                    FLAG_MB_mode    = MOVEMODE_LIFT;
                    Mode_BACKMOTION = MOVE_PRETURN;//MOVE_COM_5BACKWARD;
                    break;
                }
                case MOVE_PUTBOX:
                {
                    command_num  = MOVE_PUTBOX;
                    command_FLAG = true;

                    FLAG_MB_mode    = MOVEMODE_PUT;
                    Mode_BACKMOTION = MOVE_COM_5BACKWARD_ABS;
                    break;
                }
            }
            break;
        }
        case MODE_MOVEBOX_FRONT:
        {
            switch(Mode_MOVEBOX)
            {
                case MOVE_HOLDPOSE:
                {
                    double Time = 3.0;

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.45;
                    RHpos[1] = LHpos[1] = 0.40;
                    RHpos[2] = LHpos[2] = 1.25;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }
                case MOVE_FRONT_HOLD:
                {
                    double Time = 2.0;

                    PELpos[0] = WBmotion->pCOM_2x1[0] -0.05;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], Time);

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.45;
                    RHpos[1] = LHpos[1] = 0.32;
                    RHpos[2] = LHpos[2] = 1.25;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }
                case MOVE_FRONT_PULL:
                {
                    double Time = 2.0;

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.40;
                    RHpos[1] = LHpos[1] = 0.32;
                    RHpos[2] = LHpos[2] = 1.25;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }
                case MOVE_FRONT_PUSH:
                {
                    double Time = 2.0;

                    cout << "MOVE_FRONT_PUSH..." << endl;

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.45;
                    RHpos[1] = LHpos[1] = 0.32;
                    RHpos[2] = LHpos[2] = 1.25;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }
                case MOVE_FRONT_RELEASE:
                {
                    double Time = 2.0;

                    PELpos[0] = WBmotion->pCOM_2x1[0]+0.05;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], Time);

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.45;
                    RHpos[1] = LHpos[1] = 0.40;
                    RHpos[2] = LHpos[2] = 1.25;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }
                case MOVE_FRONT_COM_BACK:
                {
                    double Time = 2.0;

                    PELpos[0] = WBmotion->pCOM_2x1[0] - 0.05;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], Time);

                    Mode_MOVEBOX = MOVE_NOTHING;
                    break;
                }

            }


            break;
        }
        case MODE_MOVEBOX_LIFTBOX:
        {
            switch(Mode_BACKMOTION)
            {
                case BACK_WAIT:
                {
                    WaitCount++;

                    if(WaitCount > WaitTime*200)
                    {
                        switch(WaitMode)
                        {
                            case BACK_BACK:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_BACK OK";
                                Mode_BACKMOTION = BACK_OPENARMS;
                                break;
                            }
                            case BACK_OPENARMS:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_OPENARMS OK";
                                Mode_BACKMOTION = BACK_SIT;
                                break;
                            }
                            case BACK_SIT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_SIT OK";
                                Mode_BACKMOTION = BACK_HOLD;
                                break;
                            }
                            case BACK_HOLD:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HOLD OK";
                                Mode_BACKMOTION = BACK_LIFT;
                                break;
                            }
                            case BACK_LIFT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_LIFT OK";
                                Mode_BACKMOTION = BACK_STAND;
                                break;
                            }
                            case BACK_STAND:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_STAND OK";
                                Mode_BACKMOTION = BACK_FRONT;
                                break;
                            }
                            case BACK_FRONT:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_FRONT OK";
                                Mode_BACKMOTION = BACK_PUSHIN;
                                break;
                            }
                            case BACK_PUSHIN:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_PUSHIN OK";
                                Mode_BACKMOTION = BACK_HANDSDOWN;
                                break;
                            }
                            case BACK_HANDSDOWN:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HANDSDOWN OK";
                                Mode_BACKMOTION = BACK_RELEASE;
                                break;
                            }
                            case BACK_RELEASE:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_RELEASE OK";
                                Mode_BACKMOTION = BACK_HANDSBACK;
                                break;
                            }
                            case BACK_HANDSBACK2:
                            {
                                FILE_LOG(logSUCCESS) << "BACK_HANDSBACK OK";
                                Mode_BACKMOTION = BACK_TURNTOFRONT;
                                break;
                            }
                        }
                        WaitCount = 0;
                    }
                    break;
                }
                case BACK_BACK:
                {
                    printf("init2rinit = %f, %f, %f\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
                    WBmotion->addWSTPosInfo(-180.0, 4.0);

                    SetWaitTime_back(BACK_BACK, 4.0);
                    break;
                }
                case BACK_OPENARMS:
                {
                    double COMx         = WBmotion->pCOM_2x1[0];// + 0.01;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = WBmotion->pPelZ;;
                    double PelPitch     = in.back_sitPelPitch;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_openArmsHandx;
                    double Handy        = in.back_openArmsHandy;
                    double Handz        = in.back_openArmsHandz;
                    double HandOriPitch = 0.;
                    double HandOriYaw   = 0.;
                    double Time         = in.TimeBackOpenArms;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    SetWaitTime_back(BACK_OPENARMS,in.TimeBackOpenArms);
                    break;
                }
                case BACK_SIT:
                {
                    double COMx         = WBmotion->pCOM_2x1[0] -0.005;// + 0.01;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = in.back_sitPelz;
                    double PelPitch     = in.back_sitPelPitch;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_sitHandx;
                    double Handy        = in.back_sitHandy;
                    double Handz        = in.back_sitHandz;
                    double HandOriPitch = 0.;
                    double HandOriYaw   = 0.;
                    double Time         = in.TimeSit;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    SetWaitTime_back(BACK_SIT, in.TimeSit);
                    break;

                }
                case BACK_HOLD:
                {
                    //Current hand pose
                    RHpos[0] = RHpos[0];    RHpos[1] = RHpos[1];    RHpos[2] = RHpos[2];
                    LHpos[0] = LHpos[0];    LHpos[1] = LHpos[1];    LHpos[2] = LHpos[2];

                    RHpos[1] += in.back_holdHandy;
                    LHpos[1] -= in.back_holdHandy;

                    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeBackHold);
                    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeBackHold);

                    WBmotion->kine_drc.m_RightHand += boxmass;
                    WBmotion->kine_drc.m_LeftHand  += boxmass;


                    SetWaitTime_back(BACK_HOLD, in.TimeBackHold);

                    break;
                }
                case BACK_LIFT:
                {
                    double COMx         = NO_USE;
                    double COMy         = NO_USE;
                    double Pelz         = NO_USE;
                    double PelPitch     = NO_USE;
                    int    HandMode     = HAND_LOCAL;
                    double Handx        = in.back_liftHandx;
                    double Handy        = LHpos[1];
                    double Handz        = in.back_liftHandz;
                    double HandOriPitch = 0.0;
                    double HandOriYaw   = 0.0;
                    double Time         = in.TimeBackLift;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                    SetWaitTime_back(BACK_LIFT, in.TimeBackLift);

                    break;
                }
                case BACK_STAND:
                {
                    double COMx         = WBmotion->pCOM_2x1[0] + 0.01;
                    double COMy         = WBmotion->pCOM_2x1[1];
                    double Pelz         = 0.0;
                    double PelPitch     = 0.;
                    int    HandMode     = HAND_GLOBAL;
                    double Handx        = 0.45;
                    double Handy        = RHpos[1];
                    double Handz        = 0.25;
                    double HandOriPitch = -90.0;
                    double HandOriYaw   = 0.0;
                    double Time         = in.TimeStand;

                    back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);


                    SetWaitTime_back(BACK_STAND, in.TimeStand);
                    break;
                }
                case BACK_FRONT:    //90deg Turn this time
                {
                    WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeStand);
                    WBmotion->addWSTPosInfo(0.0, 5.0);

                    Mode_BACKMOTION = BACK_NOTHING;
                }
            }
            break;
        }
        case MODE_TEST:
        {
            FILE_LOG(logSUCCESS) << "case MODE_TEST";
            double Time = 3.0;
            SetOriPitch(PELori, 0.0);
            WBmotion->addPELOriInfo(PELori, Time);

            RHpos[0] = LHpos[0] = 0.45;
            RHpos[1] = LHpos[1] = 0.32;
            RHpos[2] = LHpos[2] = 1.05; //0.25+0.8;

            RHpos[1] -= sharedData->COM_SH[1]*2;//*10;
            LHpos[1] += sharedData->COM_SH[1]*2;//*10;

            FILE_LOG(logINFO) << "RHpos[1] = " << RHpos[1] ;

            WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
            WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

            SetOriHand(RHori, -90.0, 0.0);
            SetOriHand(LHori, -90.0, 0.0);

            WBmotion->addRHOriInfo(RHori, Time);
            WBmotion->addLHOriInfo(LHori, Time);

            break;
        }
        case MODE_UB:
        {
            switch(Mode_UBMOTION)
            {
                case UB_WAIT:
                {
                    WaitCount++;

                    if(WaitCount > WaitTime*200)
                    {
                        switch(WaitMode)
                        {
                            case UB_APPROACH:
                            {
                                FILE_LOG(logSUCCESS) << "UB_APPROACH OK";
                                Mode_UBMOTION = UB_GRIPPER_CLOSEHALF;
                                break;
                            }
                            case UB_GRIPPER_CLOSEHALF:
                            {
                                FILE_LOG(logSUCCESS) << "UB_GRIPPER_CLOSEHALF OK";
                                Mode_UBMOTION = UB_COM_FOLLOW;
                                break;
                            }
                            case UB_GRIPPER_OPEN:
                            {
                                FILE_LOG(logSUCCESS) << "UB_GRIPPER_OPEN OK";
                                Mode_UBMOTION = UB_WALKREADYISH;
                                break;
                            }
                            case UB_WALKREADYISH:
                            {
                                FILE_LOG(logSUCCESS) << "UB_WALKREADYISH OK";
                                Mode_UBMOTION = UB_NOTHING;

                                //GIRPPER_CLOSE
                                FLAG_Gripper  = true;
                                MODE_RGripper = GRIPPER_CLOSE;
                                MODE_LGripper = GRIPPER_CLOSE;

                                FLAG_CART_mode = false;

                                break;
                            }

                        }
                        WaitCount = 0;
                    }
                    break;

                }
                case UB_APPROACH:
                {
                    FILE_LOG(logSUCCESS) << "case UB_APPROACH";
                    double Time = 3.0;

                    SetOriPitch(PELori, 0.0);
                    WBmotion->addPELOriInfo(PELori, Time);

                    RHpos[0] = LHpos[0] = 0.505;
                    RHpos[1] = LHpos[1] = 0.18;
                    RHpos[2] = LHpos[2] = 0.916339;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    WBmotion->addRElbPosInfo(-30.0, Time);
                    WBmotion->addLElbPosInfo( 30.0, Time);

                    FLAG_Gripper  = true;
                    MODE_RGripper = GRIPPER_OPEN;
                    MODE_LGripper = GRIPPER_OPEN;

                    Mode_UBMOTION = UB_NOTHING;
                    if(FLAG_CART_mode == CART_HOLD)
                        SetWaitTime_cart(UB_APPROACH,Time+3.0);
                    break;
                }
                case UB_GRIPPER_CLOSEHALF:
                {
                    FILE_LOG(logSUCCESS) << "case UB_GRIPPER_CLOSEHALF";
                    double Time = 3.0;

                    //GIRPPER_CLOSE_HALF
                    FLAG_Gripper  = true;
                    MODE_RGripper = GRIPPER_CLOSE_HALF;
                    MODE_LGripper = GRIPPER_CLOSE_HALF;

                    Mode_UBMOTION = UB_NOTHING;
                    if(FLAG_CART_mode == CART_HOLD)
                        SetWaitTime_cart(UB_GRIPPER_CLOSEHALF,Time);

                    break;
                }
                case UB_GRIPPER_OPEN:
                {
                    FILE_LOG(logSUCCESS) << "case UB_GRIPPER_OPEN";

                    //GIRPPER_OPEN
                    FLAG_Gripper  = true;
                    MODE_RGripper = GRIPPER_OPEN;
                    MODE_LGripper = GRIPPER_OPEN;

                    Mode_UBMOTION = UB_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        SetWaitTime_cart(UB_GRIPPER_OPEN,3.0);

                    break;
                }
                case UB_COM_FOLLOW:
                {
//                    FILE_LOG(logSUCCESS) << "case UB_COM_FOLLOW";

                    double Time = 0.002;

                    RHpos[0] = LHpos[0] = 0.533803;
                    RHpos[1] = LHpos[1] = 0.179943;
                    RHpos[2] = LHpos[2] = 0.916339;

                    if(FLAG_CART_mode == CART_HOLD)
                    {
                        RHpos[0] = LHpos[0] = 0.505;
                        RHpos[1] = LHpos[1] = 0.18;
                        RHpos[2] = LHpos[2] = 0.916339;
                    }

                    RHpos[1] += (sharedData->COM_m_local[1]);
                    LHpos[1] -= (sharedData->COM_m_local[1]);


                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    SetOriHand(RHori, -90.0, 0.0);
                    SetOriHand(LHori, -90.0, 0.0);

                    WBmotion->addRHOriInfo(RHori, Time);
                    WBmotion->addLHOriInfo(LHori, Time);

                    break;
                }
                case UB_FOLLOW_DONE:
                {
                    FILE_LOG(logSUCCESS) << "case UB_FOLLOW_DONE";

                    Mode_UBMOTION = UB_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        Mode_UBMOTION = UB_GRIPPER_OPEN;
                    break;
                }
                case UB_HOLDCART:
                {
                    FILE_LOG(logSUCCESS) << "case UB_HOLDCART";

                    Mode_UBMOTION = UB_APPROACH;
                    FLAG_CART_mode = CART_HOLD;

                    break;
                }
                case UB_RELEASECART:
                {
                    FILE_LOG(logSUCCESS) << "case UB_RELEASECART";

                    Mode_UBMOTION = UB_FOLLOW_DONE;
                    FLAG_CART_mode = CART_RELEASE;

                    break;
                }                
                case UB_WALKREADYISH:
                {
                    FILE_LOG(logSUCCESS) << "case UB_WALKREADYISH";

                    double Time = 3.0;

                    RHpos[0] = LHpos[0] = 0.293229;
                    RHpos[1] = LHpos[1] = 0.246403;
                    RHpos[2] = LHpos[2] = 1.02771;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    WBmotion->addRElbPosInfo(-5.0, Time);
                    WBmotion->addLElbPosInfo( 5.0, Time);

                    Mode_UBMOTION = UB_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        SetWaitTime_cart(UB_WALKREADYISH,3.0);

                    FLAG_CART_mode = false;

                    break;
                }
                case UB_NOTHING:
                {
                    Mode_LIFTBOX = MODE_NOTHING;
                    cout <<"UB_NOTHING" << endl;
                    break;
                }
                default:
                    break;
            }
        }
        case MODE_NOTHING:
        {
            break;
        }
    }
}

void SetOriPitch(doubles &target, double _pitch)
{
    quat pelori = quat(vec3(0,1,0), _pitch*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

void SetOriYaw(doubles &target, double _yaw)
{
    quat pelori = quat(vec3(0,0,1), _yaw*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

void SetOriHand(doubles &target, double _pitch, double _yaw)
{
    quat pelori = quat(vec3(0,0,1), _yaw*D2R)*quat(vec3(0,1,0), _pitch*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

int CheckFTsensor()
{
    if(sharedData->FT[2].Fy < -in.FTlimit && sharedData->FT[3].Fy > in.FTlimit)
    {
        return true;
    }else
    {
        return false;
    }

}

void SetWaitTime(int mode, double time)
{
    Command_LIFTBOX = WAIT;
    WaitCount = 0;
    WaitMode = mode;
    WaitTime = time;
}

void SetWaitTime_back(int mode, double time)
{
    Mode_BACKMOTION = BACK_WAIT;
    WaitCount = 0;
    WaitMode = mode;
    WaitTime = time;
}

void SetWaitTime_move(int mode, double time)
{
    Mode_BACKMOTION = MOVE_WAIT;
    WaitCount = 0;
    WaitMode = mode;
    WaitTime = time;
}

void SetWaitTime_cart(int mode, double time)
{
    Mode_UBMOTION = UB_WAIT;
    WaitCount = 0;
    WaitMode = mode;
    WaitTime = time;
}

int GainOverrideSY(){

    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 30, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 30, 10);
    usleep(5000);

    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}

int arm_GainOverride(int gain, short _msec){

//    cout<<"arm_GainOverrides!"<<endl;

    //Left ARM joints: LSP, LSR, LSY, LEB, LWY, LWP
    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);

//    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, gain, _msec); //gain: Gain percentage 0:fixed 100:totally lose
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, gain, _msec);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, gain, _msec);
//    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, gain, _msec);
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, gain, _msec);
    usleep(5000);

    //Right ARM joints: RSP, RSR, RSY, REB, RWY, RWP
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);

//    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, gain, _msec);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, gain, _msec);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, gain, _msec);
//    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, gain, _msec);
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, gain, _msec);
    usleep(5000);

    return 0;
}

void Set_RHand_Global2Local(vec3 _pos, quat _ori)
{
    vec3 p_init2global;

    /*
    cout << "*************************"<< endl;
    cout << "in Set_RHand_Global2Local"<< endl;
    cout << "RF x: " << WBmotion->pRF_3x1[0] << endl;
    cout << "RF y: " << WBmotion->pRF_3x1[1] << endl;
    cout << "RF z: " << WBmotion->pRF_3x1[2] << endl;
    cout << "*************************"<< endl;
    */

    p_init2global[0] = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    p_init2global[1] = (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2.;
    p_init2global[2] = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2.;


    mat4 T_init2global  = mat4(vec3(p_init2global),vec3(0,0,1), 0.0);

    mat4 T_global2hand  = mat4(_pos,_ori);

    mat4 T_init2local   = mat4(vec3(PELpos[0],PELpos[1],PELpos[2]), quat(PELori[0],PELori[1],PELori[2],PELori[3]));

    mat4 T_local2init   = T_init2local.inverse();

    mat4 T_local2global = T_local2init*T_init2global;

    mat4 T_local2hand   = T_local2global*T_global2hand;

    RHpos[0] = -T_local2hand.m03;
    RHpos[1] = T_local2hand.m13;
    RHpos[2] = T_local2hand.m23;
}

void Set_LHand_Global2Local(vec3 _pos, quat _ori)
{
    vec3 p_init2global;
    p_init2global[0] = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    p_init2global[1] = (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2.;
    p_init2global[2] = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2.;


    mat4 T_init2global = mat4(vec3(p_init2global),vec3(0,0,1), 0.0);

    mat4 T_global2hand = mat4(_pos,_ori);

    mat4 T_init2local = mat4(vec3(PELpos[0],PELpos[1],PELpos[2]), quat(PELori[0],PELori[1],PELori[2],PELori[3]));

    mat4 T_local2init = T_init2local.inverse();

    mat4 T_local2global = T_local2init*T_init2global;

    mat4 T_local2hand = T_local2global*T_global2hand;

    LHpos[0] = -T_local2hand.m03;
    LHpos[1] = T_local2hand.m13;
    LHpos[2] = T_local2hand.m23;
}

void back_move(double COMx, double COMy, double Pelz, double PelPitch, int HandMode, double Handx, double Handy, double Handz, double HandOriPitch, double HandOriYaw, double Time)
{
    if(COMx != NO_USE)
    {
        PELpos[0] = COMx;//WBmotion->pCOM_2x1[0]+0.01;
        PELpos[1] = COMy;//WBmotion->pCOM_2x1[1];
        WBmotion->addCOMInfo(PELpos[0], PELpos[1], Time);
    }

    if(Pelz != NO_USE)
    {
        PELpos[2] = Pelz;
        WBmotion->addPELPosInfo(PELpos[2], Time);
    }

    if(PelPitch != NO_USE)
    {
        SetOriPitch(PELori, PelPitch);
        WBmotion->addPELOriInfo(PELori, Time);
    }

    if(HandMode == HAND_GLOBAL)
    {
        RHpos[0] = LHpos[0] = Handx;
        RHpos[1] = LHpos[1] = Handy;
        RHpos[2] = LHpos[2] = Handz;

        WBmotion->addRHPosInfo(RHpos[0],  RHpos[1], RHpos[2], Time);
        WBmotion->addLHPosInfo(LHpos[0], -LHpos[1], LHpos[2], Time);
    }
    else if(HandMode == HAND_LOCAL)
    {
        if((Handx != NO_USE) || (Handy != NO_USE) || (Handz != NO_USE))
        {
            vec3 des_rpos = vec3(Handx, -Handy, Handz);
            vec3 des_lpos = vec3(Handx,  Handy, Handz);
            quat des_ori  = quat(vec3(0,1,0), 0);

            Set_RHand_Global2Local(des_rpos, des_ori);
            Set_LHand_Global2Local(des_lpos, des_ori);

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], Time);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], Time);
        }
    }

    if((HandOriPitch != NO_USE) || (HandOriYaw != NO_USE))
    {
        SetOriHand(RHori, HandOriPitch, HandOriYaw);
        SetOriHand(LHori, HandOriPitch,-HandOriYaw);

        WBmotion->addRHOriInfo(RHori, Time);
        WBmotion->addLHOriInfo(LHori, Time);
    }
}

void GripperTH()
{
    if(FLAG_Gripper == true)
    {
        //get position of each Gripper//
        float EncoderRHAND = sharedData->ENCODER[MC_ID_CH_Pairs[RHAND].id][MC_ID_CH_Pairs[RHAND].ch].CurrentPosition;
        float EncoderLHAND = sharedData->ENCODER[MC_ID_CH_Pairs[LHAND].id][MC_ID_CH_Pairs[LHAND].ch].CurrentPosition;
        //set velocity of each Gripper//
        int velocityRGripper = 130;
        int velocityLGripper = 130;
        if(sharedData->EXF_L_Enabled)
            velocityLGripper = 30;
        if(sharedData->EXF_R_Enabled)
            velocityRGripper = 30;

        static int DoneR, DoneL = true;

        switch(MODE_RGripper)
        {
            case GRIPPER_STOP:
            {
                joint->SetJointRefAngle(RHAND, 0);
                DoneR = true;
                break;
            }
            case GRIPPER_OPEN:
            {
                static int gripper_cnt = 0;
                DoneR = false;
                if(gripper_cnt > MAX_GRIPPER_CNT)
                {
                    FILE_LOG(logINFO)    << "Right Gripper open stop (time over)";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderRHAND < DESIRED_Gripper)
                {//open done
                    FILE_LOG(logSUCCESS) << "Right Gripper open done";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                joint->SetJointRefAngle(RHAND, -velocityRGripper);

                gripper_cnt++;
                break;
            }
            case GRIPPER_CLOSE_HALF:
            {
//                FILE_LOG(logSUCCESS) << "IN GRIPPER_CLOSE_HALF";
                static int gripper_cnt = 0;
                DoneR = false;

                if(gripper_cnt > MAX_GRIPPER_CNT)
                {
                    FILE_LOG(logINFO) << "Right Gripper close stop (time over)";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderRHAND > LIMIT_Gripper_half)
                {//grasp done
                    FILE_LOG(logSUCCESS) << "Right Gripper grasp done";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                std::cout << "velocity Setting: " << velocityRGripper << std::endl;
                joint->SetJointRefAngle(RHAND, velocityRGripper);
                gripper_cnt++;
                break;
            }
            case GRIPPER_CLOSE:
            {
                static int gripper_cnt = 0;
                DoneR = false;

                if(gripper_cnt > MAX_GRIPPER_CNT)
                {
                    FILE_LOG(logINFO) << "Right Gripper close stop (time over)";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderRHAND > LIMIT_Gripper)
                {//grasp done
                    FILE_LOG(logSUCCESS) << "Right Gripper grasp done";
                    MODE_RGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                joint->SetJointRefAngle(RHAND, velocityRGripper);
                gripper_cnt++;
                break;
            }
        }

        switch(MODE_LGripper)
        {
            case GRIPPER_STOP:
            {
                joint->SetJointRefAngle(LHAND, 0);
                DoneL = true;
                break;
            }
            case GRIPPER_OPEN:
            {
                static int gripper_cnt = 0;
                DoneL = false;
                if(gripper_cnt > MAX_GRIPPER_CNT)
                {
                    FILE_LOG(logINFO) << "Left Gripper open stop (time over)";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderLHAND < DESIRED_Gripper)
                {//open done
                    FILE_LOG(logSUCCESS) << "Left Gripper open done";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                joint->SetJointRefAngle(LHAND, -velocityLGripper);
                gripper_cnt++;
                break;
            }
            case GRIPPER_CLOSE_HALF:
            {
                static int gripper_cnt = 0;
                DoneL = false;
                if(gripper_cnt > MAX_GRIPPER_CNT) //> 120) //> MAX_GRIPPER_CNT) //
                {
                    FILE_LOG(logINFO) << "Left Gripper close stop (time over)";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderLHAND > LIMIT_Gripper_half)
                {//grasp done
                    FILE_LOG(logSUCCESS) << "Left Gripper grasp done";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                joint->SetJointRefAngle(LHAND, velocityLGripper);
                gripper_cnt++;
                break;
            }
            case GRIPPER_CLOSE:
            {
                static int gripper_cnt = 0;
                DoneL = false;
                if(gripper_cnt > MAX_GRIPPER_CNT) //> 120) //> MAX_GRIPPER_CNT) //
                {
                    FILE_LOG(logINFO) << "Left Gripper close stop (time over)";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                if(EncoderLHAND > LIMIT_Gripper)
                {//grasp done
                    FILE_LOG(logSUCCESS) << "Left Gripper grasp done";
                    MODE_LGripper = GRIPPER_STOP;
                    gripper_cnt = 0;
                    break;
                }
                joint->SetJointRefAngle(LHAND, velocityLGripper);
                gripper_cnt++;
                break;
            }
        }

        if(DoneR && DoneL)
        {
            FILE_LOG(logSUCCESS) << "Both Gripper move done!!";
            FLAG_Gripper = false;
        }
    }
}

void SaveOneStep(int cnt)
{
    for(int saveNum = RHY; saveNum < LHAND; saveNum++)
    {
        SAVE[saveNum][cnt] = sharedData->ENCODER[MC_GetID(saveNum)][MC_GetCH(saveNum)].CurrentReference;
    }
    for(int saveNum = RHY; saveNum < LHAND; saveNum++)
    {
        SAVE[saveNum + LHAND + 1][cnt] = 0.0;//sharedData->gogo_Reference[saveNum - LHAND -1];
    }
}
void SaveAll()
{

}

vec3 ZMP_calc_HB(vec3 _pRF, quat _qRF, vec3 _F_RF, vec3 _M_RF, vec3 _pLF, quat _qLF, vec3 _F_LF,vec3 _M_LF)
{
    //<Input> : global position of foot, local Force/Torque, orientation of foot
    // <Ouput> : global zmp

    if(_F_RF.z <= 0) _F_RF.z = 0;
    if(_F_LF.z <= 0) _F_LF.z = 0;

    if(_F_RF.z < 2.0 && _F_LF.z<2.0)
    {
        vec3 ZMP_global;
        ZMP_global = _pRF*0.5+_pLF*0.5;
        return ZMP_global;//not good...
    }

    vec3 pRF_real = _pRF + vec3(0,0,0.045);
    vec3 pLF_real = _pLF + vec3(0,0,0.045);

    // transform local force and moment to global frame
    mat3 RF_TFmat = mat3(_qRF);
    vec3 F_RF_global = RF_TFmat*_F_RF;
    vec3 M_RF_global = RF_TFmat*_M_RF;

    mat3 LF_TFmat = mat3(_qLF);
    vec3 F_LF_global = LF_TFmat*_F_LF;
    vec3 M_LF_global = LF_TFmat*_M_LF;

    vec3 rxF_LF_global = cross(pLF_real,F_LF_global);
    vec3 rxF_RF_global = cross(pRF_real,F_RF_global);

    vec3 ZMP_global_SH;
    ZMP_global_SH.x = (-rxF_LF_global.y - rxF_RF_global.y - M_LF_global.y - M_RF_global.y)/(F_LF_global.z + F_RF_global.z);
    ZMP_global_SH.y = ( rxF_LF_global.x + rxF_RF_global.x + M_LF_global.x + M_RF_global.x)/(F_LF_global.z + F_RF_global.z);
    ZMP_global_SH.z = (_pRF.z*F_RF_global.z + _pLF.z*F_LF_global.z)/(F_RF_global.z + F_LF_global.z);

    return ZMP_global_SH;
}


//get ZMP
vec3 ZMP_calc_SH()
{
    double M_LF_Global[3],M_RF_Global[3],F_LF_Global[3],F_RF_Global[3];
    double pCenter[3],qCenter[4], qCenter_bar[4];

    vec3 ZMP_temp;
    double _pRF[3];
    _pRF[0] = WBmotion->pRF_3x1[0];
    _pRF[1] = WBmotion->pRF_3x1[1];
    _pRF[2] = WBmotion->pRF_3x1[2];

    double _qRF[4];
    _qRF[0] = WBmotion->qRF_4x1[0];
    _qRF[1] = WBmotion->qRF_4x1[1];
    _qRF[2] = WBmotion->qRF_4x1[2];
    _qRF[3] = WBmotion->qRF_4x1[3];

    double _pLF[3];
    _pLF[0] = WBmotion->pLF_3x1[0];
    _pLF[1] = WBmotion->pLF_3x1[1];
    _pLF[2] = WBmotion->pLF_3x1[2];

    double _qLF[4];
    _qLF[0]  = WBmotion->qLF_4x1[0];
    _qLF[1]  = WBmotion->qLF_4x1[1];
    _qLF[2]  = WBmotion->qLF_4x1[2];
    _qLF[3]  = WBmotion->qLF_4x1[3];

    double _F_RF[3];
    _F_RF[0] = sharedData->FT[0].Fx;
    _F_RF[1] = sharedData->FT[0].Fy;
    _F_RF[2] = sharedData->FT[0].Fz;

    double _M_RF[3];
    _M_RF[0] = sharedData->FT[0].Mx;
    _M_RF[1] = sharedData->FT[0].My;
    _M_RF[2] = sharedData->FT[0].Mz;

    double _F_LF[3];
    _F_LF[0] = sharedData->FT[1].Fx;
    _F_LF[1] = sharedData->FT[1].Fy;
    _F_LF[2] = sharedData->FT[1].Fz;

    double _M_LF[3];
    _M_LF[0] = sharedData->FT[1].Mx;
    _M_LF[1] = sharedData->FT[1].My;
    _M_LF[2] = sharedData->FT[1].Mz;

    _pRF[0] = 0.0;
    _pRF[1] = -0.105;
    _pRF[2] = 0.0;
    _pLF[1] = 0.105;
    _qLF[0] = 1;
    _qRF[0] = 1;



    // Foot Center in Global Coord.
    pCenter[0] = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    pCenter[1] = (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2.;
    pCenter[2] = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2.;


    if(sharedData->FT[0].Fz + sharedData->FT[1].Fz > 50.)
    {
        _M_LF[0] = sharedData->FT[1].Mx;
        _M_LF[1] = sharedData->FT[1].My;
        _M_LF[2] = sharedData->FT[1].Mz;

        QTtransform(_qLF,_M_LF,M_LF_Global);

        _M_RF[0] = sharedData->FT[0].Mx;
        _M_RF[2] = sharedData->FT[0].Mz;
        _M_RF[1] = sharedData->FT[0].My;

        QTtransform(_qRF,_M_RF,M_RF_Global);

        _F_LF[0] = sharedData->FT[1].Fx;
        _F_LF[1] = sharedData->FT[1].Fy;
        _F_LF[2] = sharedData->FT[1].Fz;

        QTtransform(_qLF,_F_LF,F_LF_Global);

        _F_RF[0] = sharedData->FT[0].Fx;
        _F_RF[1] = sharedData->FT[0].Fy;
        _F_RF[2] = sharedData->FT[0].Fz;

        QTtransform(_qRF,_F_RF,F_RF_Global);

        double temp1[3],temp2[3],temp3[3],temp4[3];


        diff_vv(WBmotion->pRF_3x1,3,pCenter,temp1); // (despRF - pCenter)
        diff_vv(WBmotion->pLF_3x1,3,pCenter,temp2); // (despLF - pCenter)

        cross(1,temp1,F_RF_Global,temp3);  // (despRF - pCenter)x(F_RF_Global)
        cross(1,temp2,F_LF_Global,temp4);  // (despLF - pCenter)x(F_LF_Global)

        sum_vv(temp3,3,temp4,temp3);       // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
        sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
        sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global


        zmp_calc[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
        zmp_calc[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
        zmp_calc[2] = 0.;

        diff_vv(zmp_calc,3,pCenter,temp1); // zmp - pCenter

        qCenter_bar[0] =  qCenter[0];
        qCenter_bar[1] = -qCenter[1];
        qCenter_bar[2] = -qCenter[2];
        qCenter_bar[3] = -qCenter[3];
    }


}
