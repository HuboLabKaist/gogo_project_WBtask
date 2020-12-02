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

    sprintf(__AL_NAME, "gogoCart");
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
                    // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->ResetGlobalCoord(0);
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
                    // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->ResetGlobalCoord(-1);
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
                    // RF_or_LF: 1=LF, -1=RF, 0=PC
                    WBmotion->ResetGlobalCoord(0);
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
                Mode_UBMOTION   = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

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

                FlagSave = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
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
//        LiftBox_Supervisor();
        Cart_Supervisor();
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

            usleep(10);

            //FILE WRITING_daemon reference
            FILE *daemonFile = NULL;
            daemonFile = fopen("/home/rainbow/Desktop/daemon_ref.txt","a");
            if((daemonFile == NULL))
                std::cout << "Failed to open daemon_ref.txt" << std::endl;
            else{
                for(int joint_num = RHY; joint_num <= LHAND; joint_num++){
    //                fprintf(daemonFile,"%f, ",sharedData->ENCODER[MC_GetID(joint_num)][MC_GetCH(joint_num)].CurrentReference);
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
            zmpFile = fopen("/home/rainbow/Desktop/zmp.txt","a");
            if((zmpFile == NULL))
                std::cout << "Failed to open zmp.txt" << std::endl;
            else{
                for(int i=0; i<3; i++)
                {
                    fprintf(zmpFile, "%lf,", sharedData->zmp_HBWalk[i]);
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

/* ************Cart_Supervisor*************** */
/* ************Cart_Supervisor*************** */
/* ************Cart_Supervisor*************** */
void Cart_Supervisor()
{
    switch(Mode_LIFTBOX)
    {
        case MODE_UB:
        {
            switch(Mode_UBMOTION)
            {
                case CART_WAIT:
                {
                    WaitCount++;

                    if(WaitCount > WaitTime*200)
                    {
                        switch(WaitMode)
                        {
                            case CART_APPROACH:
                            {
                                FILE_LOG(logSUCCESS) << "CART_APPROACH OK";
                                Mode_UBMOTION = CART_GRIPPER_CLOSEHALF;
                                break;
                            }
                            case CART_GRIPPER_CLOSEHALF:
                            {
                                FILE_LOG(logSUCCESS) << "CART_GRIPPER_CLOSEHALF OK";
                                arm_GainOverride(30,10);
                                Mode_UBMOTION = CART_COM_FOLLOW;
                                break;
                            }
                            case CART_GRIPPER_OPEN:
                            {
                                FILE_LOG(logSUCCESS) << "CART_GRIPPER_OPEN OK";
                                Mode_UBMOTION = CART_WALKREADYISH;
                                break;
                            }
                            case CART_WALKREADYISH:
                            {
                                FILE_LOG(logSUCCESS) << "CART_WALKREADYISH OK";
                                Mode_UBMOTION = CART_NOTHING;

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
                case CART_APPROACH:
                {
                    FILE_LOG(logSUCCESS) << "case CART_APPROACH!";

                    command_num  = CART_APPROACH;
                    command_FLAG = true;

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

                    Mode_UBMOTION = CART_NOTHING;
                    if(FLAG_CART_mode == CART_HOLD)
                        SetWaitTime_cart(CART_APPROACH,Time+3.0);
                    break;
                }
                case CART_GRIPPER_CLOSEHALF:
                {
                    FILE_LOG(logSUCCESS) << "case CART_GRIPPER_CLOSEHALF";

                    command_num  = CART_GRIPPER_CLOSEHALF;
                    command_FLAG = true;

                    double Time = 3.0;

                    //GIRPPER_CLOSE_HALF
                    FLAG_Gripper  = true;
                    MODE_RGripper = GRIPPER_CLOSE_HALF;
                    MODE_LGripper = GRIPPER_CLOSE_HALF;

                    Mode_UBMOTION = CART_NOTHING;
                    if(FLAG_CART_mode == CART_HOLD)
                        SetWaitTime_cart(CART_GRIPPER_CLOSEHALF,Time);

                    break;
                }
                case CART_GRIPPER_OPEN:
                {
                    FILE_LOG(logSUCCESS) << "case CART_GRIPPER_OPEN";

                    command_num  = CART_GRIPPER_OPEN;
                    command_FLAG = true;


                    //GIRPPER_OPEN
                    FLAG_Gripper  = true;
                    MODE_RGripper = GRIPPER_OPEN;
                    MODE_LGripper = GRIPPER_OPEN;

                    PELpos[0] = WBmotion->pCOM_2x1[0] + 0.02;
                    PELpos[1] = WBmotion->pCOM_2x1[1];
                    WBmotion->addCOMInfo(PELpos[0], PELpos[1], 2.0);

                    Mode_UBMOTION = CART_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        SetWaitTime_cart(CART_GRIPPER_OPEN,3.0);

                    break;
                }
                case CART_COM_FOLLOW:
                {

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
                case CART_FOLLOW_DONE:
                {
                    FILE_LOG(logSUCCESS) << "case CART_FOLLOW_DONE";

                    command_num  = CART_FOLLOW_DONE;
                    command_FLAG = true;

                    arm_GainOverride(0,1000);

                    Mode_UBMOTION = CART_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        Mode_UBMOTION = CART_GRIPPER_OPEN;
                    break;
                }
                case CART_HOLDCART:
                {
                    FILE_LOG(logSUCCESS) << "case CART_HOLDCART";

                    command_num  = CART_HOLDCART;
                    command_FLAG = true;


                    Mode_UBMOTION = CART_APPROACH;
                    FLAG_CART_mode = CART_HOLD;

                    break;
                }
                case CART_RELEASECART:
                {
                    FILE_LOG(logSUCCESS) << "case CART_RELEASECART";

                    command_num  = CART_RELEASECART;
                    command_FLAG = true;

                    Mode_UBMOTION = CART_FOLLOW_DONE;
                    FLAG_CART_mode = CART_RELEASE;

                    break;
                }
                case CART_WALKREADYISH:
                {
                    FILE_LOG(logSUCCESS) << "case CART_WALKREADYISH";

                    command_num  = CART_WALKREADYISH;
                    command_FLAG = true;

                    double Time = 3.0;

                    RHpos[0] = LHpos[0] = 0.35;//0.293229;
                    RHpos[1] = LHpos[1] = 0.246403;
                    RHpos[2] = LHpos[2] = 1.02771;

                    WBmotion->addRHPosInfo(RHpos[0], -RHpos[1], RHpos[2], Time);
                    WBmotion->addLHPosInfo(LHpos[0],  LHpos[1], LHpos[2], Time);

                    WBmotion->addRElbPosInfo(-5.0, Time);
                    WBmotion->addLElbPosInfo( 5.0, Time);

                    Mode_UBMOTION = CART_NOTHING;
                    if(FLAG_CART_mode == CART_RELEASE)
                        SetWaitTime_cart(CART_WALKREADYISH,3.0);

                    FLAG_CART_mode = false;

                    break;
                }
                case CART_NOTHING:
                {
                    Mode_LIFTBOX = MODE_NOTHING;
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

void SetWaitTime_cart(int mode, double time)
{
    Mode_UBMOTION = CART_WAIT;
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
