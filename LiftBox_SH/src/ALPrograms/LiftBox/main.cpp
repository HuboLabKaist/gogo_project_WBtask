#include "liftbox.h"
// --------------------------------------------------------------------------------------------- //

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
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    sprintf(__AL_NAME, "LiftBox");
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
            case LIFTBOX_LIFTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_LIFTBOX received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_LIFTBOX;
                Command_LIFTBOX = SITDOWN;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_LIFTBOX_PARTS:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_LIFTBOX_Parts received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_LIFTBOX_PARTS;

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//sitdown
                {
                    Command_LIFTBOX = SITDOWN;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//PutonTable
                {
                    Command_LIFTBOX = PUTONTABLE;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//RELEASEBOX
                {
//                    Command_LIFTBOX = HANDSDOWN;
                    Command_LIFTBOX = RELEASEBOX;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//STANDUP
                {
                    Command_LIFTBOX = STANDUP;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//PUSHIN
                {
                    Command_LIFTBOX = PUSHIN;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 5)//HANDSDOWN
                {
                    Command_LIFTBOX = HANDSDOWN;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 6)//HANDSBACK
                {
                    Command_LIFTBOX = HANDSBACK;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_PUTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_PUTBOX received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_PUTBOX;
                Command_LIFTBOX = SITDOWN;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_MOTION:
            {
                FILE_LOG(logSUCCESS) << "New CMD :: LIFTBOX_BACK_MOTION received..\n";

                Mode_LIFTBOX = MODE_BACK;
                joint->RefreshToCurrentReference();

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//back
                {
                    ShutDownAllFlag();
                    joint->RefreshToCurrentReference();
                    StartWBIKmotion(0);
                    joint->SetAllMotionOwner();
                    Mode_LIFTBOX = MODE_BACK;
                    Mode_BACKMOTION = BACK_BACK;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//sit
                {
                     Mode_BACKMOTION = BACK_SIT;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//hold
                {
                     Mode_BACKMOTION = BACK_HOLD;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//stand
                {
                     Mode_BACKMOTION = BACK_STAND;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//front
                {
                     Mode_BACKMOTION = BACK_FRONT;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 5)//OpenArms
                {
                     Mode_BACKMOTION = BACK_OPENARMS;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 6)//back_pushin
                {
                     Mode_BACKMOTION = BACK_PUSHIN;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 7)//back_handsdown
                {
                     Mode_BACKMOTION = BACK_HANDSDOWN;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 8)//back_handsback
                {
                     Mode_BACKMOTION = BACK_RELEASE;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 9)//back_handsback
                {
                     Mode_BACKMOTION = BACK_HANDSBACK;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 10)//back_turnToFront
                {
                     Mode_BACKMOTION = BACK_TURNTOFRONT;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 11)//back_handsback2
                {
                     Mode_BACKMOTION = BACK_STAND2;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 99)//BACK_GETHANDPOS
                {
                     Mode_BACKMOTION = BACK_GETHANDPOS;
                }

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_LIFTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_LIFTBOX received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(0);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX    = MODE_BACK_LIFTBOX;
                Mode_BACKMOTION = BACK_BACK;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_PUTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_PUTBOX received..\n";

//                ShutDownAllFlag();
//                joint->RefreshToCurrentReference();
//                StartWBIKmotion(0);
//                joint->SetAllMotionOwner();
//                WBmotion->RefreshToCurrentReference();

                Mode_LIFTBOX    = MODE_BACK_PUTBOX;
                Mode_BACKMOTION = BACK_FRONT;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_TEST:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_TEST received..\n";

                //TEST_ADDMASS
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

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_LIFTPUT:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_BACK_LIFTPUT received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(0);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX    = MODE_BACK_LIFTPUT;
                Mode_BACKMOTION = BACK_BACK;

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
            if(Mode_LIFTBOX == MODE_BACK || Mode_LIFTBOX == MODE_BACK_LIFTBOX || Mode_LIFTBOX == MODE_BACK_PUTBOX || Mode_LIFTBOX == MODE_BACK_LIFTPUT)
            {
//                FILE_LOG(logSUCCESS) << "WBIK_BACK()";
                WBmotion->WBIK_BACK();
            }
            else
                WBmotion->WBIK();

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

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }


        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

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
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
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
    printf("LHPos  = (%f, %f, %f)\n",WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
    printf("LHOri  = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    printf("RHPos  = (%f, %f, %f)\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
    printf("RHOri  = (%f, %f, %f, %f)\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    printf("PelPos = (%f, %f, %f)\n",WBmotion->pCOM_2x1[0],WBmotion->pCOM_2x1[1],WBmotion->pPelZ);
    printf("PelOri = (%f, %f, %f, %f)\n",WBmotion->qPEL_4x1[0],WBmotion->qPEL_4x1[1],WBmotion->qPEL_4x1[2],WBmotion->qPEL_4x1[3]);
    printf("=======================================================\n\n");
}

/* ************LIFTBOX_SUPERVISOR*************** */
void LiftBox_Supervisor()
{
    switch(Mode_LIFTBOX)
    {
    case MODE_LIFTBOX:
    {
        switch(Command_LIFTBOX)
        {
        case SITDOWN:
        {
            GainOverrideSY();
            SitDown(MODE_LIFTBOX);
            SetWaitTime(SITDOWN, in.TimeSit);
            break;
        }
        case WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == SITDOWN)
                {
                    FILE_LOG(logSUCCESS) << "SITDOWN OK";
                    Command_LIFTBOX = HOLDBOX;
                } else if(WaitMode == HANDUP)
                {
                    FILE_LOG(logSUCCESS) << "HANDUP OK";
                    Command_LIFTBOX = STANDUP;
                } else if(WaitMode == STANDUP)
                {
                    FILE_LOG(logSUCCESS) << "STANDUP OK";
                    Command_LIFTBOX = NOTHING;
                }
                WaitCount = 0;
            }
            break;
        }
        case HOLDBOX:
        {
            int StateHoldBox = HoldBox();

            if(StateHoldBox == HOLD_SUCCESS)
            {
                FILE_LOG(logSUCCESS) << "Hold Box SUCCESS";
                Command_LIFTBOX = HANDUP;
            } else if(StateHoldBox == HOLD_FAIL)
            {
                FILE_LOG(logERROR) << "Hold Box FAIL";
                Command_LIFTBOX = HANDUP;
            }
            break;
        }
        case HANDUP:
        {
            HandUp();
            SetWaitTime(HANDUP, in.TimeHandUp);
            break;
        }
        case STANDUP:
        {
            printf("standup\n");
            StandUp(MODE_LIFTBOX);
            SetWaitTime(STANDUP, in.TimeStand);
            break;
        }
        case NOTHING:
        {
            break;
        }
        default:
        {
            FILE_LOG(logERROR) << "LIFT BOX Command Error !!";
            printf("%d\n",Command_LIFTBOX);
            ShutDownAllFlag();
            break;
        }
        }
        break;
    }
    case MODE_LIFTBOX_PARTS:{
        switch(Command_LIFTBOX)
        {
        case SITDOWN:
        {
            GainOverrideSY();
            SitDown(MODE_LIFTBOX);
            Command_LIFTBOX = NOTHING;
            break;
        }
        case PUTONTABLE:
        {
            putOnTable(MODE_PUTBOX);
            Command_LIFTBOX = NOTHING;
            break;
        }
        case PUSHIN:
        {
            pushIn();
            Command_LIFTBOX = NOTHING;
            break;
        }
        case HANDSDOWN:
        {
            std::cout << "CASE HANDSDOWN" << std::endl;
            handsdown();
            Command_LIFTBOX = NOTHING;
            break;
        }
        case HANDSBACK:
        {
            std::cout << "CASE HANDSBACK" << std::endl;
            handsback();
            Command_LIFTBOX = NOTHING;
            break;
        }
        case RELEASEBOX:
        {
//            GainOverrideSY();

            RHpos[0] = WBmotion->pRH_3x1[0]; RHpos[1] = WBmotion->pRH_3x1[1]; RHpos[2] = WBmotion->pRH_3x1[2];
            LHpos[0] = WBmotion->pLH_3x1[0]; LHpos[1] = WBmotion->pLH_3x1[1]; LHpos[2] = WBmotion->pLH_3x1[2];

            if(ReleaseBox() == true)
            {
                FILE_LOG(logSUCCESS) << "Release Box SUCCESS";
                Command_LIFTBOX = NOTHING;

                WBmotion->kine_drc.m_RightHand -= boxmass;
                WBmotion->kine_drc.m_LeftHand  -= boxmass;

                //FT SENSOR USE:
//                if(Hold_Box_Flag == true)
//                {
//                    WBmotion->kine_drc.m_RightHand -= boxmass;
//                    WBmotion->kine_drc.m_LeftHand  -= boxmass;
//                }
//                Hold_Box_Flag = false;

                std::cout << "ReleaseBox.m_RightHand = " << WBmotion->kine_drc.m_RightHand << std::endl;
                std::cout << "ReleaseBox.m_LeftHand = "  << WBmotion->kine_drc.m_LeftHand  << std::endl;
            }
            break;
        }
        case STANDUP:
        {
            StandUp(MODE_PUTBOX);
            FILE_LOG(logSUCCESS) << "STANDUP";
            Command_LIFTBOX = NOTHING;
        }
        case NOTHING:
            break;
        }
        break;
    }
    case MODE_PUTBOX:
    {
        switch(Command_LIFTBOX)
        {
        case SITDOWN:
        {
            SitDown(MODE_PUTBOX);
            SetWaitTime(SITDOWN, in.TimeSit);
            break;
        }
        case WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == SITDOWN)
                {
                    FILE_LOG(logSUCCESS) << "SITDOWN OK";
                    Command_LIFTBOX = RELEASEBOX;
                } else if(WaitMode == STANDUP)
                {
                    FILE_LOG(logSUCCESS) << "STANDUP OK";
                    Command_LIFTBOX = NOTHING;
                }
                WaitCount = 0;
            }
            break;
        }
        case RELEASEBOX:
        {
            if(ReleaseBox() == true)
            {
                FILE_LOG(logSUCCESS) << "Release Box SUCCESS";
                Command_LIFTBOX = STANDUP;
            }
            break;
        }
        case STANDUP:
        {
            StandUp(MODE_PUTBOX);
            SetWaitTime(STANDUP, in.TimeStand);
            break;
        }
        case NOTHING:
        {
            break;
        }
        default:
        {
            FILE_LOG(logERROR) << "PUT BOX Command Error !!";
            ShutDownAllFlag();
            break;
        }
        }
        break;
    }
    case MODE_BACK:
    {
        switch(Mode_BACKMOTION)
        {
        case BACK_BACK:
        {
            printf("init2rinit = %f, %f, %f\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
            WBmotion->addWSTPosInfo(-180.0, 4.0);
            Mode_BACKMOTION = BACK_NOTHING;
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


//            PELpos[0] = COMx;//WBmotion->pCOM_2x1[0]+0.01;
//            PELpos[1] = COMy;//WBmotion->pCOM_2x1[1];
//            WBmotion->addCOMInfo(PELpos[0], PELpos[1], Time);

//            PELpos[2] = Pelz;
//            WBmotion->addPELPosInfo(PELpos[2], Time);

//            SetOriPitch(PELori, PelPitch);
//            WBmotion->addPELOriInfo(PELori, Time);

//            vec3 des_rpos = vec3(Handx, -Handy, Handz);
//            vec3 des_lpos = vec3(Handx,  Handy, Handz);
//            quat des_ori = quat(vec3(0,0,0), 1);

//            Set_RHand_Global2Local(des_rpos, des_ori);
//            Set_LHand_Global2Local(des_lpos, des_ori);

//            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], Time);
//            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], Time);

//            SetOriHand(RHori, HandOriPitch, HandOriYaw);
//            SetOriHand(LHori, HandOriPitch, -HandOriYaw);

//            WBmotion->addRHOriInfo(RHori, Time);
//            WBmotion->addLHOriInfo(LHori, Time);

            Mode_BACKMOTION = BACK_NOTHING;
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

            Mode_BACKMOTION = BACK_NOTHING;
            break;

        }
        case BACK_HOLD:
        {
            AbsHoldBox();

            WBmotion->kine_drc.m_RightHand += boxmass;
            WBmotion->kine_drc.m_LeftHand  += boxmass;

            Mode_BACKMOTION = BACK_LIFT;

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
//            back_move(NO_USE, NO_USE, NO_USE, NO_USE, HAND_LOCAL, in.back_liftHandx, LHpos[1], in.back_liftHandz+0.02, 0.0, 0.0, in.TimeBackLift);

            Mode_BACKMOTION = BACK_WAIT;
            break;
        }
        case BACK_WAIT:
        {
            static int cnt = 0;
            if(cnt > in.TimeHandUp*200)
            {
                cnt = 0;
                Mode_BACKMOTION = BACK_NOTHING;
                break;
            }
            cnt++;
            break;
        }
        case BACK_STAND:
        {
            std::cout << "YOU'RE IN BACK_STAND NOW" << std::endl;

            double COMx         = WBmotion->pCOM_2x1[0] + 0.01;
            double COMy         = WBmotion->pCOM_2x1[1];
            double Pelz         = 0.0;
            double PelPitch     = 0.;
            int    HandMode     = HAND_GLOBAL;
//            double Handx        = in.back_standHandx;
//            double Handy        = RHpos[1];
//            double Handz        = in.back_standHandz;
//            double HandOriPitch = 0.;
//            double HandOriYaw   = -5.0;
//            double Time         = in.TimeStand;
            double Handx        = in.back_stand2Handx;
            double Handy        = RHpos[1];
            double Handz        = in.back_standH2andz;
            double HandOriPitch = -90.0;
            double HandOriYaw   = 0.0;
            double Time         = in.TimeStand;

            back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

//            Mode_BACKMOTION = BACK_STAND2;
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
//        case BACK_WAIT_STAND:
//        {
//            static int cnt = 0;
//            if(cnt > 10*200)
//            {
//                cnt = 0;
//                Mode_BACKMOTION = BACK_STAND2;
//                break;
//            }
//            cnt++;
//            break;
//        }
        case BACK_STAND2:
        {
            std::cout << "YOU'RE IN BACK_STAND2 NOW" << std::endl;

            double COMx         = NO_USE;
            double COMy         = NO_USE;
            double Pelz         = NO_USE;
            double PelPitch     = NO_USE;
            int    HandMode     = HAND_GLOBAL;
            double Handx        = in.back_stand2Handx;
            double Handy        = RHpos[1];
            double Handz        = in.back_standH2andz;
            double HandOriPitch = -90.0;
            double HandOriYaw   = 0.0;
            double Time         = in.TimeStand;

            back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_FRONT:    //90deg Turn this time
        {
            WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeStand);
            WBmotion->addWSTPosInfo(-90.0, 5.0);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_PUSHIN:
        {
            std::cout << "YOU'RE IN BACK_PUSHIN NOW" << std::endl;

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

            back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_HANDSDOWN:
        {
            std::cout << "YOU'RE IN BACK_HANDSDOWN NOW" << std::endl;

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

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_RELEASE:
        {
            std::cout << "YOU'RE IN BACK_RELEASE NOW" << std::endl;

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

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_HANDSBACK:
        {
            std::cout << "YOU'RE IN BACK_HANDSBACK NOW" << std::endl;

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

            Mode_BACKMOTION = BACK_HANDSBACK2;
            break;
        }
        case BACK_HANDSBACK2:
        {
            std::cout << "YOU'RE IN BACK_HANDSBACK NOW" << std::endl;

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

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_TURNTOFRONT:
        {
            WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeTurnFront);
            WBmotion->addWSTPosInfo(0.0, 5.0);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_GETHANDPOS:
        {
            std::cout << "GET HAND POSITION>>>" << std::endl;

            std::cout << "COM x = " << WBmotion->pCOM_2x1[0] << std::endl;
            std::cout << "COM y = " << WBmotion->pCOM_2x1[1] << std::endl;

            std::cout << "RH(x,y,z) = (" <<  WBmotion->pRH_3x1[0] <<", "<<   WBmotion->pRH_3x1[1] <<", "<<   WBmotion->pRH_3x1[2] << ")"<< std::endl;
            std::cout << "LH(x,y,z) = (" <<  WBmotion->pLH_3x1[0] <<", "<<   WBmotion->pLH_3x1[1] <<", "<<   WBmotion->pLH_3x1[2] << ")"<< std::endl;

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        }
        break;
    }
    case MODE_NOTHING:
    {
        break;
    }
    case MODE_BACK_LIFTBOX:
    {
        switch(Mode_BACKMOTION)
        {
        case BACK_WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == BACK_BACK)
                {
                    FILE_LOG(logSUCCESS) << "BACK_BACK OK";
                    Mode_BACKMOTION = BACK_OPENARMS;
                }else if(WaitMode == BACK_OPENARMS)
                {
                    FILE_LOG(logSUCCESS) << "BACK_OPENARMS OK";
                    Mode_BACKMOTION = BACK_SIT;
                }else if(WaitMode == BACK_SIT)
                {
                    FILE_LOG(logSUCCESS) << "BACK_SIT OK";
                    Mode_BACKMOTION = BACK_HOLD;

//                    FILE_LOG(logSUCCESS) << "BEFORE BACK_HOLD";
//                    printf("kine_drc.m_RightHand = %f\n",WBmotion->kine_drc.m_RightHand);
                }else if(WaitMode == BACK_HOLD)
                {
                    FILE_LOG(logSUCCESS) << "BACK_HOLD OK";
                    Mode_BACKMOTION = BACK_LIFT;
                }else if(WaitMode == BACK_LIFT)
                {
                    FILE_LOG(logSUCCESS) << "BACK_LIFT OK";
                    Mode_BACKMOTION = BACK_STAND;
//                    Mode_BACKMOTION = BACK_NOTHING;

//                    FILE_LOG(logSUCCESS) << "After BACK_LIFT";
//                    printf("kine_drc.m_RightHand = %f\n",WBmotion->kine_drc.m_RightHand);
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

//            Mode_BACKMOTION = BACK_NOTHING;
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
            double Handx        = in.back_stand2Handx;
            double Handy        = RHpos[1];
            double Handz        = in.back_standH2andz;
            double HandOriPitch = -90.0;
            double HandOriYaw   = 0.0;
            double Time         = in.TimeStand;

            back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        }
        break;
    }
    case MODE_BACK_PUTBOX:
    {
        switch(Mode_BACKMOTION)
        {
        case BACK_WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == BACK_FRONT)
                {
                    FILE_LOG(logSUCCESS) << "BACK_FRONT OK";
                    Mode_BACKMOTION = BACK_PUSHIN;
                }else if(WaitMode == BACK_PUSHIN)
                {
                    FILE_LOG(logSUCCESS) << "BACK_PUSHIN OK";
                    Mode_BACKMOTION = BACK_HANDSDOWN;
                }else if(WaitMode == BACK_HANDSDOWN)
                {
                    FILE_LOG(logSUCCESS) << "BACK_HANDSDOWN OK";
                    Mode_BACKMOTION = BACK_RELEASE;
                }else if(WaitMode == BACK_RELEASE)
                {
                    FILE_LOG(logSUCCESS) << "BACK_RELEASE OK";
                    Mode_BACKMOTION = BACK_HANDSBACK;
                }else if(WaitMode == BACK_HANDSBACK2)
                {
                    FILE_LOG(logSUCCESS) << "BACK_HANDSBACK OK";
                    Mode_BACKMOTION = BACK_TURNTOFRONT;
                }
                WaitCount = 0;
            }
            break;
        }
        case BACK_FRONT:    //90deg Turn this time
        {
            WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeStand);
            WBmotion->addWSTPosInfo(-90.0, 5.0);
            SetWaitTime_back(BACK_FRONT, 5.0);
            break;
        }
        case BACK_PUSHIN:
        {
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

            back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

            SetWaitTime_back(BACK_PUSHIN, in.TimeBackPushIn);
            break;
        }
        case BACK_HANDSDOWN:
        {
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

            SetWaitTime_back(BACK_HANDSDOWN, in.TimeHandsDown);
            break;
        }
        case BACK_RELEASE:
        {
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

            SetWaitTime_back(BACK_RELEASE, in.TimeHandsDown);
            break;
        }
        case BACK_HANDSBACK:
        {
            std::cout << "YOU'RE IN BACK_HANDSBACK NOW" << std::endl;

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

            Mode_BACKMOTION = BACK_HANDSBACK2;
            break;
        }
        case BACK_HANDSBACK2:
        {
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

            SetWaitTime_back(BACK_HANDSBACK2, 5.0);
            break;
        }
        case BACK_TURNTOFRONT:
        {
            WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeTurnFront);
            WBmotion->addWSTPosInfo(0.0, 5.0);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        }

        break;
    }
    case MODE_BACK_LIFTPUT:
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
                double Handx        = in.back_stand2Handx;
                double Handy        = RHpos[1];
                double Handz        = in.back_standH2andz;
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
                WBmotion->addWSTPosInfo(-90.0, 5.0);
                SetWaitTime_back(BACK_FRONT, 5.0);
                break;
            }
            case BACK_PUSHIN:
            {
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

                back_move(COMx, COMy, Pelz, PelPitch, HandMode, Handx, Handy, Handz, HandOriPitch, HandOriYaw, Time);

                SetWaitTime_back(BACK_PUSHIN, in.TimeBackPushIn);
                break;
            }
            case BACK_HANDSDOWN:
            {
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

                SetWaitTime_back(BACK_HANDSDOWN, in.TimeHandsDown);
                break;
            }
            case BACK_RELEASE:
            {
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

                SetWaitTime_back(BACK_RELEASE, in.TimeHandsDown);
                break;
            }
            case BACK_HANDSBACK:
            {
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

                Mode_BACKMOTION = BACK_HANDSBACK2;
                break;
            }
            case BACK_HANDSBACK2:
            {
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

                SetWaitTime_back(BACK_HANDSBACK2, 5.0);
                break;
            }
            case BACK_TURNTOFRONT:
            {
                WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+ in.back_frontComOffsetx, WBmotion->pCOM_2x1[1], in.TimeTurnFront);
                WBmotion->addWSTPosInfo(0.0, 5.0);
                Mode_BACKMOTION = BACK_NOTHING;
                break;
            }
        }
        break;
    }
    }
}


void SitDown(int _mode)
{
    WBmotion->addPELPosInfo(in.SitPelZ, in.TimeSit);

    SetOriPitch(PELori, in.PelPitch);
    WBmotion->addPELOriInfo(PELori, in.TimeSit);

    //hand x
    RHpos[0] = LHpos[0] = in.SitHandX;

    //hand y
    if(_mode == MODE_LIFTBOX)
    {
        RHpos[1] = -in.SitHandY;
        LHpos[1] = in.SitHandY;
    }else if(_mode == MODE_PUTBOX)
    {
        RHpos[1] = RHpos[1];
        LHpos[1] = LHpos[1];
    }else
    {
        FILE_LOG(logERROR) << "SitDown mode error!!!";
        ShutDownAllFlag();
    }

    //hand z
    RHpos[2] = LHpos[2] = in.SitHandZ;

    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeSit);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeSit);
//    WBmotion->addRElbPosInfo(-in.ElbAngle, in.TimeSit);
//    WBmotion->addLElbPosInfo(in.ElbAngle, in.TimeSit);
    SetOriHand(RHori, in.HandPitch, in.HandYaw);
    SetOriHand(LHori, in.HandPitch, -in.HandYaw);
    WBmotion->addRHOriInfo(RHori, in.TimeSit);
    WBmotion->addLHOriInfo(LHori, in.TimeSit);
}

void putOnTable(int _mode)
{
    //hand x
    RHpos[0] = LHpos[0] = in.SitHandX;

    //hand y
    if(_mode == MODE_LIFTBOX)
    {
        RHpos[1] = -in.SitHandY;
        LHpos[1] = in.SitHandY;
    }else if(_mode == MODE_PUTBOX)
    {
        RHpos[1] = RHpos[1];
        LHpos[1] = LHpos[1];
    }else
    {
        FILE_LOG(logERROR) << "putOnTable error!!!";
        ShutDownAllFlag();
    }

    //hand z
    RHpos[2] = LHpos[2] = 1.35;//in.SitHandZ + 0.55; //in.SitHandZ = 0.45

    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeSit);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeSit);

    SetOriHand(RHori, in.HandPitch, in.HandYaw);
    SetOriHand(LHori, in.HandPitch, -in.HandYaw);

    WBmotion->addRHOriInfo(RHori, in.TimeSit);
    WBmotion->addLHOriInfo(LHori, in.TimeSit);
}

void pushIn()
{
    //Current hand position
    RHpos[0] = WBmotion->pRH_3x1[0]; RHpos[1] = WBmotion->pRH_3x1[1]; RHpos[2] = WBmotion->pRH_3x1[2];
    LHpos[0] = WBmotion->pLH_3x1[0]; LHpos[1] = WBmotion->pLH_3x1[1]; LHpos[2] = WBmotion->pLH_3x1[2];

    std::cout <<std::endl << "BEFORE >>>" << std::endl;
    std::cout << "RH (x,y,z) = (" << RHpos[0] <<", "<< RHpos[1] <<", "<< RHpos[2] << std::endl;
    std::cout << "LH (x,y,z) = (" << LHpos[0] <<", "<< LHpos[1] <<", "<< LHpos[2] << std::endl;

    zRotation_neg90(&in.pushInHandx,&in.pushInHandy);

    RHpos[0] += in.pushInHandx;
    LHpos[0] += in.pushInHandx;

    RHpos[1] += in.pushInHandy;
    LHpos[1] += in.pushInHandy;

    RHpos[2] += in.pushInHandz;
    LHpos[2] += in.pushInHandz;


    std::cout << "END >>>" << std::endl;
    std::cout << "RH (x,y,z) = (" << RHpos[0] <<", "<< RHpos[1] <<", "<< RHpos[2] << std::endl;
    std::cout << "LH (x,y,z) = (" << LHpos[0] <<", "<< LHpos[1] <<", "<< LHpos[2] << std::endl;


    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimePushIn);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimePushIn);
}

void handsdown()
{
    //Current hand position
    RHpos[0] = WBmotion->pRH_3x1[0]; RHpos[1] = WBmotion->pRH_3x1[1]; RHpos[2] = WBmotion->pRH_3x1[2];
    LHpos[0] = WBmotion->pLH_3x1[0]; LHpos[1] = WBmotion->pLH_3x1[1]; LHpos[2] = WBmotion->pLH_3x1[2];

    RHpos[2] -= in.handsdownHandz;
    LHpos[2] -= in.handsdownHandz;

    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeHandsDown);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeHandsDown);
}

void handsback()
{
    //Current hand position
    RHpos[0] = WBmotion->pRH_3x1[0]; RHpos[1] = WBmotion->pRH_3x1[1]; RHpos[2] = WBmotion->pRH_3x1[2];
    LHpos[0] = WBmotion->pLH_3x1[0]; LHpos[1] = WBmotion->pLH_3x1[1]; LHpos[2] = WBmotion->pLH_3x1[2];

    RHpos[0] -= in.handsBackHandx;
    LHpos[0] -= in.handsBackHandx;

    RHpos[2] -= in.handsBackHandz;
    LHpos[2] -= in.handsBackHandz;

    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeHandsDown);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeHandsDown);
}

void StandUp(int _mode)
{
    WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+0.01, WBmotion->pCOM_2x1[1], in.TimeStand);
    WBmotion->addPELPosInfo(in.StandPelZ, in.TimeStand);

    SetOriPitch(PELori, 0.);
    WBmotion->addPELOriInfo(PELori, in.TimeStand);

    if(_mode == MODE_LIFTBOX)
    {
        RHpos[0] = LHpos[0] = in.StandHandX;
        RHpos[2] = LHpos[2] = in.StandHandZ;

        SetOriHand(RHori, in.HandPitch, in.HandYaw);
        SetOriHand(LHori, in.HandPitch, -in.HandYaw);
    }else if(_mode == MODE_PUTBOX)
    {
        RHpos[0] = LHpos[0] = in.FinalHandX;
        RHpos[1] = -in.FinalHandY;
        LHpos[1] = in.FinalHandY;
        RHpos[2] = LHpos[2] = in.FinalHandZ;

        SetOriPitch(RHori, -90.);
        SetOriPitch(LHori, -90.);
    }else
    {
        FILE_LOG(logERROR) << "SitDown mode error!!!";
        ShutDownAllFlag();
    }
    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeStand);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeStand);

    WBmotion->addRHOriInfo(RHori, in.TimeStand);
    WBmotion->addLHOriInfo(LHori, in.TimeStand);
}

void HandUp()
{
    RHpos[2] = RHpos[2] + in.HandUpZ;
    LHpos[2] = LHpos[2] + in.HandUpZ;
    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeHandUp);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeHandUp);
}

void AbsHoldBox()
{
    //Current hand pose
    RHpos[0] = RHpos[0];    RHpos[1] = RHpos[1];    RHpos[2] = RHpos[2];
    LHpos[0] = LHpos[0];    LHpos[1] = LHpos[1];    LHpos[2] = LHpos[2];

    RHpos[1] += in.back_holdHandy;
    LHpos[1] -= in.back_holdHandy;

    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimePushIn);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimePushIn);
}

int HoldBox()
{
    if(CheckFTsensor() == true)
    {
        HoldCount = 0;
        return HOLD_SUCCESS;
    }else
    {
        if(HoldCount > in.TimeMaxHold*200)
        {
            HoldCount = 0;

            RHpos[0] = RHpos[0];
            RHpos[1] = RHpos[1];
            RHpos[2] = RHpos[2];
            LHpos[0] = LHpos[0];
            LHpos[1] = LHpos[1];
            LHpos[2] = LHpos[2];

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], 0.005);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], 0.005);

            return HOLD_FAIL;
        } else
        {
            HoldCount++;

            RHpos[1] += in.HoldY;
            LHpos[1] -= in.HoldY;
            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], 0.005);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], 0.005);

            return HOLD_YET;
        }
    }
}

int ReleaseBox()
{
    if(ReleaseCount > in.TimeRelease*200)
    {
        ReleaseCount = 0;
        return true;
    } else
    {
        //Current hand position
        RHpos[0] = WBmotion->pRH_3x1[0]; RHpos[1] = WBmotion->pRH_3x1[1]; RHpos[2] = WBmotion->pRH_3x1[2];
        LHpos[0] = WBmotion->pLH_3x1[0]; LHpos[1] = WBmotion->pLH_3x1[1]; LHpos[2] = WBmotion->pLH_3x1[2];

        ReleaseCount++;

        RHpos[1] -= in.ReleaseY; //double ReleaseY = 0.0004;
        LHpos[1] += in.ReleaseY;
        WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], 0.005);
        WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], 0.005);
        return false;
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

void Set_RHand_Global2Local(vec3 _pos, quat _ori)
{
    vec3 p_init2global;

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

        WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], Time);
        WBmotion->addLHPosInfo(LHpos[0], -LHpos[1], LHpos[2], Time);
    }
    else if(HandMode == HAND_LOCAL)
    {
        if((Handx != NO_USE) || (Handy != NO_USE) || (Handz != NO_USE))
        {
            vec3 des_rpos = vec3(Handx, -Handy, Handz);
            vec3 des_lpos = vec3(Handx,  Handy, Handz);
            quat des_ori = quat(vec3(0,1,0), 0);

            Set_RHand_Global2Local(des_rpos, des_ori);
            Set_LHand_Global2Local(des_lpos, des_ori);

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], Time);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], Time);
        }
    }

    if((HandOriPitch != NO_USE) || (HandOriYaw != NO_USE))
    {
        SetOriHand(RHori, HandOriPitch, HandOriYaw);
        SetOriHand(LHori, HandOriPitch, -HandOriYaw);

        WBmotion->addRHOriInfo(RHori, Time);
        WBmotion->addLHOriInfo(LHori, Time);
    }
}

void zRotation_neg90(double* x, double* y)
{
    double temp = *x;
    *x = *y;
    *y = -temp;
}
void zRotation_90(double* x, double* y)
{
    double temp = *x;
    *x = -*y;
    *y = temp;
}
void GripperTH()
{

    if(FLAG_Gripper == true)
    {

        FILE_LOG(logSUCCESS) << ">>> IN GRIPPERTH..\n";

        //get position of each Gripper//
        float EncoderRHAND = 0.;//sharedSEN->ENCODER[MC_ID_CH_Pairs[RHAND].id][MC_ID_CH_Pairs[RHAND].ch].CurrentPosition;
        float EncoderLHAND = 0.;//= sharedSEN->ENCODER[MC_ID_CH_Pairs[LHAND].id][MC_ID_CH_Pairs[LHAND].ch].CurrentPosition;


        FILE_LOG(logSUCCESS) << ">>> READ Encoder..\n";


        //set velocity of each Gripper//
        int velocityRGripper = 130;
        int velocityLGripper = 130;
//        if(sharedSEN->EXF_L_Enabled)
//            velocityLGripper = 30;
//        if(sharedSEN->EXF_R_Enabled)
//            velocityRGripper = 30;

        FILE_LOG(logSUCCESS) << ">>> SET Velocity..\n";


        static int DoneR, DoneL = true;

        printf("CMD = %d\n",MODE_RGripper);
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
                FILE_LOG(logINFO) << "Right Gripper open stop (time over)";
                MODE_RGripper = GRIPPER_STOP;
                gripper_cnt = 0;
                break;
            }
            if(EncoderRHAND < LIMIT_Gripper)
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
            if(EncoderLHAND < LIMIT_Gripper)
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
        case GRIPPER_CLOSE:
        {
            static int gripper_cnt = 0;
            DoneL = false;
            if(gripper_cnt > MAX_GRIPPER_CNT)
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
