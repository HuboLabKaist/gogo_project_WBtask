#include "approachbox.h"
#include "manualwalking.cpp"
#include "controller.cpp"



/****************************** 1. main *************************************/
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

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "ApproachBox");
    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }


    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    rbjoy = new RBJoystick();
    rbjoy->ConnectJoy();

    // Initialize MPCWalking -------------------------------
    //************* Get F.K
    First_Initialize();

    // QP solver setting
    #if (NUMTESTS > 0)
      int i;
      double time;
      double time_per;
    #endif
      set_defaults();
      setup_indexing();
      /* Solve problem instance for the record. */
      settings.verbose = 1;

    // MPC Precomputing for Fast calculation
    PreComputeQP();
    GetGain(CoM_Height);
    while(__IS_WORKING){
        usleep(100*1000);
        if(rbjoy->JoyAxis[5]>32000 && rbjoy->JoyAxis[5] < 33000 && walk_flag == false)
        {
            // RT
            // Put dummy foot prints in the buffur
            // walking on place
            printf("joystic command : %d \n",rbjoy->JoyAxis[5]);
            printf("joystic command : %d \n",rbjoy->JoyAxis[5]);

            sharedData->COMMAND[PODO_NO].USER_COMMAND = APPROACHBOX_WALK_READY;
//            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            printf("Walk on place!!!!        \n");
            Upperbody_Gain_Override();

        }
        if(rbjoy->JoyButton[5] >0 && walk_flag == true)
        {
            // RB joy stick walking start. you can give a command
            printf("joystic command : %d \n",rbjoy->JoyButton[5]);
            printf("joystic command : %d \n",rbjoy->JoyButton[5]);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = APPROACHBOX_WALK_START;

            // RB
//            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
        }

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
        case APPROACHBOX_AL_TEST:
        {
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_WALK_START:
        {
            if(continouse_walking_flag == true && walk_flag == true)
            {
                FILE_LOG(logSUCCESS) << "Command LIFT_WALK_START..";

                STEP_LENGTH = userData->G2M.StepLength;
                STEP_ANGLE  = userData->G2M.StepAngle;
                STEP_OFFSET  = userData->G2M.StepOffset;
                STEP_STOP = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

                __JOY_COMMAND_MODE = true;
//                walk_start_flag = true;
                continouse_walking_flag = false;
                U0_Gain = 0.9;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_WALK_READY:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_READY..";

                Upperbody_Gain_Override();

                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;

                pv_Index = 0;
                Walking_initialize();

                /* Foot Print Generation Test
                 * (dir, step_num, step_length, step_angle, step_offset, LPEL2PEL) */
                FPG_TEST(APPROACHBOX_FORWARD_WALKING,10,0.001,0.,0.21,kine_drc_hubo4.L_PEL2PEL);
                usleep(200*1000);

                walk_flag = 1;

                continouse_walking_flag = true;

                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();

                printf("while right des x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
                printf("while left  des x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
                printf("while COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
                printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
                printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
                printf("while right global x: %f  y:%f   z:%f  \n",GLOBAL_X_RF,GLOBAL_Y_RF,GLOBAL_Z_RF);
                printf("while left  global x: %f  y:%f   z:%f  \n",GLOBAL_X_LF,GLOBAL_Y_LF,GLOBAL_Z_LF);

            }
            U0_Gain = 0.9;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_FORWARD_WALKING:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command Forward Walking received..";

                Upperbody_Gain_Override();
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
                pv_Index = 0;

                //WBIK_PARA_CHANGE();
                add_boxmass(2.5);
                /* walking initialize */
                Walking_initialize();

                /* Foot Print Generation Test */
                FPG_TEST(APPROACHBOX_FORWARD_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

                usleep(200*1000);
                walk_flag = 1;

                sharedData->LaserLength=3333;

                continouse_walking_flag = false;
                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();

                U0_Gain = 0.9;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_RIGHT_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command RIGHT Walking received..";
            Upperbody_Gain_Override();
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
            pv_Index = 0;

            WBIK_PARA_CHANGE();
            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_RIGHT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;

            U0_Gain = 0.9;
            break;
        }
        case APPROACHBOX_LEFT_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command LEFT Walking received..";
            Upperbody_Gain_Override();
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;

            pv_Index = 0;

            WBIK_PARA_CHANGE();
            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_LEFT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;

            U0_Gain = 0.9;
            break;
        }
        case APPROACHBOX_CW_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command CW Walking received..";
            //Upperbody_Gain_Override();
            GainOverrideSY();
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
            pv_Index = 0;
            Gyro_Ankle_FeedBack_ONOFF = 1.0;
            roll_torque_ONOFF =0.0;
            pitch_torque_ONOFF =0.0;
            add_boxmass(2.5);
            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_CW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;

            U0_Gain = 0.9;
            break;
        }
        case APPROACHBOX_CCW_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command CCW Walking received..";
            //Upperbody_Gain_Override();
            GainOverrideSY();
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;

            Gyro_Ankle_FeedBack_ONOFF = 1.0;
            roll_torque_ONOFF =0.0;
            pitch_torque_ONOFF =0.0;
            add_boxmass(2.5);
            pv_Index = 0;
            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_CCW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;

            U0_Gain = 0.9;
            continouse_walking_flag = 0;
            break;
        }
        case APPROACHBOX_WALK_STOP:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//MODE_NORMAL
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_NORMAL)";
                stop_flag = true;
            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//MODE_LIFTBOX
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_LIFTBOX)";
                walkstop_liftbox_flag = true;
            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//MODE_DOOR
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_DOOR)";
                walkstop_door_flag = true;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            U0_Gain = 0.9;
            break;
        }
        case APPROACHBOX_DATA_SAVE:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                FILE_LOG(logSUCCESS) << "Command Data Save received..";
                fp = fopen("data.txt","w");
                for(int i=0;i<ROW;i++)
                {
                    for(int j=0;j<COL;j++)fprintf(fp,"%g\t", Save_Data[j][i]);
                    fprintf(fp,"\n");
                }
                fclose(fp);
                FILE_LOG(logSUCCESS) << "Data Save Complete ~!..";
            }else
            {
                FILE_LOG(logSUCCESS) << "Command Reset Data received..";
                Save_Index = 0;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_COMPLIANCE_START:
        {
           FILE_LOG(logSUCCESS) << "Arm Compliance control start..\n";
           ShutDownAllFlag();
           OnOff_compliance = true;
           walk_flag = true;
           jCon->RefreshToCurrentReference();
           jCon->SetAllMotionOwner();

           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case APPROACHBOX_COMPLIANCE_STOP:
        {
           FILE_LOG(logSUCCESS) << "Arm Compliance control stop..\n";
           OnOff_compliance = false;
           ShutDownAllFlag();
           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case APPROACHBOX_PUSH_DOOR:
        {
           FILE_LOG(logSUCCESS) << "Push Door start..\n";
           FLAG_pushdoor = true;
           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case SINGLELOG_WALK:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Single Log Walking Start!!";


                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
                usleep(200*1000);
//                sharedData->
//                    cmd.COMMAND_TARGET = RBCORE_PODO_NO;


                Upperbody_Gain_Override();
                pv_Index = 0;
                WBIK_PARA_CHANGE();

                /* walking initialize */
                Walking_initialize();

                /* Foot Print Generation Test */
                FPG_SINGLELOG(userData->G2M.StepNum,para.SINGLELOG_LENGTH,kine_drc_hubo4.L_PEL2PEL);
                usleep(200*1000);
                walk_flag = 1;
                FLAG_SingleLog = true;

                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_REALWALK:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_WALK_START)
            {
                if(walk_flag == 0 && sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
                {
                    FILE_LOG(logSUCCESS) << "RealWalk Ready..";
                    Upperbody_Gain_Override();
                    sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                    sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
                    pv_Index = 0;

                    WBIK_PARA_CHANGE();
                    /* walking initialize */
                    Walking_initialize();

                    FPG_TEST(APPROACHBOX_FORWARD_WALKING,5,0.001,0.,0.21,kine_drc_hubo4.L_PEL2PEL);

                    walk_flag = true;
                    continouse_walking_flag = true;

                    jCon->RefreshToCurrentReference();
                    jCon->SetAllMotionOwner();
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
                {
                    FILE_LOG(logSUCCESS) << "RealWalk Start..!!";
                    RSTEP_LENGTH = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    continouse_walking_flag = false;
                    Command_RealWalk = REALWALK_WALK_START;
                }
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_SINGLELOG_START)
            {
                FILE_LOG(logSUCCESS) << "SingleLog Start..!!";
                RSTEP_NUM = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                Command_RealWalk = REALWALK_SINGLELOG_START;
                //FLAG_SingleLog = false;
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_STOP)
            {
                Command_RealWalk = REALWALK_STOP;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_ZEROGAIN:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                FILE_LOG(logSUCCESS) << "Lock both Arm !!";
                FLAG_CalEncoderFK = false;
                printf("flag is %f\n",FLAG_CalEncoderFK);
                printf("__IS_WORKING is %f\n",__IS_WORKING);
                Upperbody_Gain_Lock1();
            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)
            {
                FILE_LOG(logSUCCESS) << "Zero gain Right Arm !!";
                FLAG_CalEncoderFK = true;
                printf("flag is %f\n",FLAG_CalEncoderFK);
                printf("__IS_WORKING is %f\n",__IS_WORKING);
                ZeroGainRightArm1();
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)
            {
                FILE_LOG(logSUCCESS) << "Zero gain Left Arm !!";
                FLAG_CalEncoderFK = true;
                printf("flag is %f\n",FLAG_CalEncoderFK);
                ZeroGainLeftArm1();

            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)
            {
                FILE_LOG(logSUCCESS) << "Zero right arm test !!";
                FLAG_CalEncoderFK = true;
                printf("flag is %f\n",FLAG_CalEncoderFK);
                ZeroGainRightArm1Test();
            }

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case STEPPING_STONE_WALK:
        {
            if(walk_flag == 0)
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: STEPPING_STONE_WALK";

                //Upperbody_Gain_Override();
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
                pv_Index = 0;

                WBIK_PARA_CHANGE();
                /* walking initialize */
                Walking_initialize();
                SetFootPrint(5);
                FPG_MANUAL();

                usleep(200*1000);
                walk_flag = 1;

                continouse_walking_flag = false;
                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();

                U0_Gain = 0.9;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case YUJIN_TEST_INTERPOLATION:
        {
            FILE_LOG(logSUCCESS) << "Change Pos";
            FLAG_Interpolation_changed = true;
            in_pf = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            in_t = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_CHANGE_PARA:
        {

            add_boxmass(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = APPROACHBOX_NO_ACT;
            break;
        }
        default:
            break;
        }//end switch
    }//end while
    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}//end main

/****************************** 2. TaskThread *******************************/
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
//        YujinTest_Interpolation();
        if(walk_flag == true)
        {
            if(__JOY_COMMAND_MODE == true)
                FLAG_WalkingJoy = true;
            else
                FLAG_WalkingJoy = false;

            if(rbjoy->isConnected() && __JOY_COMMAND_MODE)
            {
                float velX = (float)(-rbjoy->JoyAxis[1]) / JOY_MAX * 0.25;
                float velTh = (float)(-rbjoy->JoyAxis[0]) / JOY_MAX * 15.0;

                if(fabs(velX) < 0.001){

                    velX = 0.001;
                }
                if(fabs(velTh) < 0.01)
                {
                    velTh = 0.0;
                }

                JOY_STICK_STEP_LENGTH = velX;
                JOY_STICK_STEP_ANGLE = velTh;
                JOY_STICK_STEP_OFFSET =  0.21 + 0.05 * fabs(velTh /15.0);

                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot)){
                    ;
                }

                int right_left = 0;
                int moving_leg;
                double rl = 0.;

                if(last_moving_leg == MOVING_RIGHT){
                    moving_leg = MOVING_LEFT;
                    right_left  = 1;
                }else
                {
                    right_left  = 0;
                    moving_leg = MOVING_RIGHT;
                }

                _footprint_info newfoot;

                memcpy(&last_target_foot, &target_foot[0], sizeof(_footprint_info));
                for(int i=0; i<5; i++)
                {
                    if(right_left == 1){
                        rl = 1;
                    }else{
                        rl = -1;
                    }
                    if(moving_leg == MOVING_RIGHT){
                        newfoot.footprint.rori[0] = last_target_foot.footprint.lori[0] + JOY_STICK_STEP_ANGLE;
                        newfoot.footprint.rori[1] = last_target_foot.footprint.rori[1];
                        newfoot.footprint.rori[2] = last_target_foot.footprint.rori[2];

                        newfoot.footprint.lori[0] = last_target_foot.footprint.lori[0];
                        newfoot.footprint.lori[1] = last_target_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_target_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_target_foot.footprint.lfoot[0] + JOY_STICK_STEP_LENGTH*cos(newfoot.footprint.rori[0]*D2R) - JOY_STICK_STEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                        newfoot.footprint.rfoot[1] = last_target_foot.footprint.lfoot[1] + JOY_STICK_STEP_LENGTH*sin(newfoot.footprint.rori[0]*D2R) + JOY_STICK_STEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
                        newfoot.footprint.rfoot[2] = last_target_foot.footprint.rfoot[2];

                        newfoot.footprint.lfoot[0] = last_target_foot.footprint.lfoot[0];
                        newfoot.footprint.lfoot[1] = last_target_foot.footprint.lfoot[1];
                        newfoot.footprint.lfoot[2] = last_target_foot.footprint.lfoot[2];

                        moving_leg = MOVING_LEFT;
                    }else
                    {
                        newfoot.footprint.rori[0] = last_target_foot.footprint.rori[0];
                        newfoot.footprint.rori[1] = last_target_foot.footprint.rori[1];
                        newfoot.footprint.rori[2] = last_target_foot.footprint.rori[2];

                        newfoot.footprint.lori[0] = last_target_foot.footprint.rori[0] + JOY_STICK_STEP_ANGLE;
                        newfoot.footprint.lori[1] = last_target_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_target_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_target_foot.footprint.rfoot[0];
                        newfoot.footprint.rfoot[1] = last_target_foot.footprint.rfoot[1];
                        newfoot.footprint.rfoot[2] = last_target_foot.footprint.rfoot[2];

                        newfoot.footprint.lfoot[0] = last_target_foot.footprint.rfoot[0] + JOY_STICK_STEP_LENGTH*cos(newfoot.footprint.lori[0]*D2R) - JOY_STICK_STEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[1] = last_target_foot.footprint.rfoot[1] + JOY_STICK_STEP_LENGTH*sin(newfoot.footprint.lori[0]*D2R) + JOY_STICK_STEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[2] = last_target_foot.footprint.lfoot[2];

                        moving_leg = MOVING_RIGHT;
                    }

                    newfoot.time.dsp_time = DSP_TIME;
                    newfoot.time.ssp_time = SSP_TIME;

                    if(rbjoy->JoyButton[4]>0)
                    {
                        FILE_LOG(logERROR) << "STOP JOY WALKING!!";
                        __JOY_COMMAND_MODE  = false;
                        while(pull_short_foot(dummyfoot)){
                            ;
                        }
                        push_short_foot(last_short_foot);
                        break;
                    }else
                    {
                        push_short_foot(newfoot);
                        memcpy(&last_target_foot, &newfoot, sizeof(_footprint_info));
                        right_left ^= 1;
                    }
                }
            }

            if(walkstop_liftbox_flag == true)
            {
                FILE_LOG(logERROR) << "APPROACH DONE!!";
                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot))
                {;}

                make_last_liftbox_footprint();

                printf("shut down approach\n");
                walkstop_liftbox_flag = false;
            }

            if(walkstop_door_flag == true)
            {
                FILE_LOG(logERROR) << "APPROACH DONE!!";
                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot))
                {;}

                make_last_door_footprint();

                printf("shut down approach\n");
                walkstop_door_flag = false;
            }

            if(OnOff_compliance == true)
            {
                //printf("Compliance on!\n");
                StartComplianceControl();
                //ZMP_FeedBack_ONOFF = true;
                ZMPControl();
            }

            /* Kalman, High Pass Filter */
            State_Estimator(sharedData->FOG.RollVel,sharedData->FOG.PitchVel, sharedData->FOG.YawVel, sharedData->IMU[0].AccX, sharedData->IMU[0].AccY, Estimated_Orientation);

            /* Update target_foot & make Trajectory ZMP,Foot reference */
            update_window();

            /* Calculate the ZMP using value of FT sensor */
            get_zmp2();

            /* Calculate the COM x,y(GLOBAL_X,Y) */
            WMG();

            /* Control on/off */
            Controller();

            /* Calculate desired value and IK */
            WBIK();

            /* save data */
            save();
            SendDatatoGUI();

        } else
        {




            static double cnt111 = 0.;
            cnt111 = cnt111+1;

            if(cnt111>0.5)
            {
                sharedData->LaserLength=6666;
            }

                if(LandingState == FINAL)
                {
                    Upperbody_Gain_Lock();
                    printf("******************* Walking Is Finishied!!!");
                    Walking_initialize();

                }
                LandingState = END;
        }


        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

/****************************** 3. FlagThread *******************************/
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}

/****************************** 4. MissionDoor ******************************/
void SendDatatoGUI()
{
    userData->M2G.Des_posX = PosX;
    userData->M2G.Des_velX = VelX;
    userData->M2G.LPF_Fz = LPF_RH_Fz;
    userData->M2G.LPF_Fx = LPF_RH_Fx;
    userData->M2G.LPF_Fy = LPF_RH_Fy;
    userData->M2G.curZMP[0] = Y_ZMP_Local;
    userData->M2G.curZMP[1] = zmp_Local[1]*1000.;
    //userData->M2G.curZMP[2] = Local_bar[1]*1000.;
}
void ShutDownAllFlag()
{
    //WB_FLAG = false;
    OnOff_compliance = false;

    /*Init Variables*/
    PosX = PosY = PosZ = VelX = VelY = VelZ = 0.;
    //ZMPControlX = ZMPControlY = Del_PC_X_DSP_XZMP_CON = Del_PC_Y_DSP_YZMP_CON = I_ZMP_CON_X = 0.;
}
void StartComplianceControl()
{
    /* Fz LowPassFilter */
    float LPF_Gain_X = 0.05;
    float LPF_Gain_Y = 0.05;
    float LPF_Gain_Z = 0.1;


    if(sharedData->FT[3].Fz > 5. || sharedData->FT[3].Fz < -5.)
    {
        LPF_RH_Fz = sharedData->FT[3].Fz;
    } else
    {
        LPF_RH_Fz = 0.;
    }
    if(sharedData->FT[3].Fx > 3. || sharedData->FT[3].Fx < -3.)
    {
        LPF_RH_Fx = sharedData->FT[3].Fx;
    } else
    {
        LPF_RH_Fx = 0.;
    }
    if(sharedData->FT[3].Fy > 0.5 || sharedData->FT[3].Fy < -0.5)
    {
        LPF_RH_Fy = sharedData->FT[3].Fy;

    } else
    {
        LPF_RH_Fy = 0.;
    }

    LPF_RH_Fz = sharedData->FT[3].Fz;
    LPF_RH_Fx = sharedData->FT[3].Fx;
    LPF_RH_Fy = sharedData->FT[3].Fy;

    LPF_RH_Fz = LPF_Gain_Z*LPF_RH_Fz + (1-LPF_Gain_Z)*Before_RH_Fz;
    LPF_RH_Fx = LPF_Gain_X*LPF_RH_Fx + (1-LPF_Gain_X)*Before_RH_Fx;
    LPF_RH_Fy = LPF_Gain_Y*LPF_RH_Fy + (1-LPF_Gain_Y)*Before_RH_Fy;

    Before_RH_Fz = LPF_RH_Fz;
    Before_RH_Fx = LPF_RH_Fx;
    Before_RH_Fy = LPF_RH_Fy;



    /* RHand Compliance control */
    float Compliance_Gain_X = 0.01;
    float Resilence_Gain_X = 5.5;

    float Compliance_Gain_Y = 0.01;
    float Resilence_Gain_Y = 5.5;

    float Compliance_Gain_Z = 0.005;
    float Resilence_Gain_Z = 2.5;

    Desired_Force = 0;
    Measured_Force_Z = LPF_RH_Fz;
    Measured_Force_Y = LPF_RH_Fy;
    Measured_Force_X = LPF_RH_Fx;

    VelX = Compliance_Gain_X*(Desired_Force - Measured_Force_Z) - Resilence_Gain_X*PosX;
    PosX += 0.005*VelX;

    VelY = Compliance_Gain_Y*(Desired_Force + Measured_Force_Y) - Resilence_Gain_Y*PosY;
    PosY += 0.005*VelY;

    VelZ = Compliance_Gain_Z*(Desired_Force + Measured_Force_X) - Resilence_Gain_Z*PosZ;
    PosZ += 0.005*VelZ;

    if(PosX > 0.3)
    {
        PosX = 0.3;
    }else if(PosX <-0.3)
    {
        PosX = -0.3;
    }

    if(PosY > 0.07)
    {
        PosY = 0.07;
    }else if(PosY <-0.07)
    {
        PosY = -0.07;
    }

    if(PosZ > 0.07)
    {
        PosZ = 0.07;
    }else if(PosZ <-0.07)
    {
        PosZ = -0.07;
    }
}

void ZMPControl()
{
    get_zmp2();
    Kirk_Control();
    zmp_Local[0] = -0.001*Del_PC_X_DSP_XZMP_CON;
    zmp_Local[1] = -0.001*Del_PC_Y_DSP_YZMP_CON;
    zmp_Local[2] = 0;

    //Local2Global(zmp_Local,Global);

    ZMPControlX = zmp_Local[0];
    ZMPControlY = zmp_Local[1];
}

/****************************** 5. Motion Generator *************************/
void WMG()
{
    int Preview_time = 15,cn = 20;
    double disturbance[2][3] = {{0,},},FOOT_REF[4] = {0.0,},temp_zmp[15]={0.0,},temp_state[3] = {0.0,},temp_vec[15] = {0.0,},FOOT_IND[2] = {1.,1.},optimal_U[15] = {0.,};
    static double pv_state[2][3] = {{0.0}}, pv_state_old[2][3] = {{0.0}};
    double UK[15][3]={{0.0,},},U0[15]={0.0,},MPC_pk[18] = {0.,},MPC_zmp[2]={0.0,};
    static double refx[3] = {0.,},refy[3] = {0.,},px0[3] = {0.,},py0[3] = {0.,},pxf[3] = {0.,},pyf[3] = {0.,},t1=0,t2=0,tf=0.1;

    if(pv_Index == 0)
    {// initialize the parameter (pv_Index : The num of times WMG() was executed.)

        pv_state_old[0][0] = 0.;
        pv_state_old[0][1] = 0.;
        pv_state_old[0][2] = 0.;
        pv_state_old[1][0] = 0.;
        pv_state_old[1][1] = 0.;
        pv_state_old[1][2] = 0.;

        printf(">>>>>>>> WMG  Initial CoM  <<<<<<<<< \n");
        printf("%f  %f  %f \n",pv_state_old[0][0],pv_state_old[0][1],pv_state_old[0][2]);
        printf("%f  %f  %f \n",pv_state_old[1][0],pv_state_old[1][1],pv_state_old[1][2]);

        for(int i = 0;i<Preview_time;i++)
        {// 15 times.
            temp_vec[i] = 0.;
            t1 = 0;
            t2 = 0;
        }
    }

    if(pv_Index == 700)
    {//3.5sec?
            disturbance[1][0] = 0.;
            disturbance[1][1] = 0.;
            disturbance[1][2] = 0.;

            disturbance[0][0] = 0.;
            disturbance[0][1] = 0.;
            disturbance[0][2] = 0.;
    }


    if(pv_Index%cn == 0)
    {//0.1sec

        /* Set U0, UK, FOOT_IND (But U0, UK not used)*/
        UKSEL(20,U0,UK,FOOT_IND);

        t1 = 0;
        t2 = 0;
        for(int i=0;i<2;i++)
        {
            if(i==0)
            {
                px0[0] = pv_state_old[i][0] + disturbance[i][0];
                px0[1] = pv_state_old[i][1] + disturbance[i][1];
                px0[2] = pv_state_old[i][2] + disturbance[i][2];
            }else
            {
                py0[0] = pv_state_old[i][0] + disturbance[i][0];
                py0[1] = pv_state_old[i][1] + disturbance[i][1];
                py0[2] = pv_state_old[i][2] + disturbance[i][2];
            }

            temp_state[0] = pv_state_old[i][0] + disturbance[i][0];
            temp_state[1] = pv_state_old[i][1] + disturbance[i][1];
            temp_state[2] = pv_state_old[i][2] + disturbance[i][2];

            /* MPC_pk */
            mat15by3x3by1(MPC_pc,temp_state,temp_vec);
            for(int asdf = 0;asdf<15;asdf++)
            {
                    if(i >0)
                    {
                        temp_zmp[asdf] = window[(asdf+1)*cn].zmp.y;
                    }else
                    {
                        temp_zmp[asdf] = window[(asdf+1)*cn].zmp.x;
                    }
            }
            temp_debug[7] = temp_zmp[0];
            for(int asdf = 0;asdf<15;asdf++)
            {
                    MPC_pk[asdf] = temp_vec[asdf] - MPC_beta*temp_zmp[asdf];
            }

            /* ------Cix>=ci */
            for(int sk = 0;sk<Preview_time*2;sk++)
            {
                if(sk<15)
                {
                    MPC_ci[sk] = MPC_sp[i] + temp_zmp[sk];
                }else
                {
                    MPC_ci[sk] = MPC_sp[i] - temp_zmp[sk - Preview_time];
                }
            }

            for(int s=0;s<15;s++)
            {
                for(int t=0;t<15;t++)
                {
                    params.Q[s + t*15 ] = MPC_Q[s][t];
                }
            }

            for(int s=0;s<15;s++)
            {
                params.c[s] = MPC_pk[s];
            }

            for(int s=0;s<30;s++)
            {
                for(int t=0;t<15;t++)
                {//zmp can go out of the foot
                    params.A[s + t*30] = 0.;//MPC_Ci[s][t];
                }
            }

            for(int s=0;s<30;s++)
            {
                params.b[s] = 0.;//MPC_ci[s];
            }

            /* solving optimization problem */
            solve();

            mat15by3x3by1(Pzs,temp_state,temp_vec);

           for(int si=0;si<15;si++)
           {
               temp_vec[si] = vars.x[si] - temp_vec[si];
           }

           mat15by15x15by1(Pzu_inv,temp_vec,optimal_U);

            pv_state[i][0] = (MPC_A[0][0]*pv_state_old[i][0] + MPC_A[0][1]*pv_state_old[i][1] + MPC_A[0][2]*pv_state_old[i][2]) + (MPC_B[0])*optimal_U[0];
            pv_state[i][1] = (MPC_A[1][0]*pv_state_old[i][0] + MPC_A[1][1]*pv_state_old[i][1] + MPC_A[1][2]*pv_state_old[i][2]) + (MPC_B[1])*optimal_U[0];
            pv_state[i][2] = (MPC_A[2][0]*pv_state_old[i][0] + MPC_A[2][1]*pv_state_old[i][1] + MPC_A[2][2]*pv_state_old[i][2]) + (MPC_B[2])*optimal_U[0];

            /* ZMP Output from LIPM */
            MPC_zmp[i] = MPC_C[0]*pv_state[i][0] + MPC_C[1]*pv_state[i][1]+ MPC_C[2]*pv_state[i][2];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];

            if(i == 0)
            {
                temp_debug[0] = temp_zmp[0];
                temp_debug[1] = MPC_zmp[i];
                temp_debug[2] = pv_state[i][0];
                temp_debug[3] = pv_state[i][1];
            }else
            {
                temp_debug[5] = temp_zmp[0];
                temp_debug[6] = MPC_zmp[i];
                temp_debug[7] = pv_state[i][0];
                temp_debug[8] = pv_state[i][1];
            }
        }

        temp_debug[10] = FOOT_REF[0];
        temp_debug[11] = FOOT_IND[0];

        pxf[0] = pv_state_old[0][0];
        pxf[1] = pv_state_old[0][1];
        pxf[2] = pv_state_old[0][2];
        pyf[0] = pv_state_old[1][0];
        pyf[1] = pv_state_old[1][1];
        pyf[2] = pv_state_old[1][2];

    }

    t1 = t1 + 0.005;
    t2 = t2 + 0.005;

    Fifth(t1,tf,px0,pxf,refx);
    Fifth(t2,tf,py0,pyf,refy);

    _temp_debug_data[0] = py0[0];
    _temp_debug_data[1] = py0[1];
    _temp_debug_data[2] = py0[2];

    _temp_debug_data[3] = refx[0];
    _temp_debug_data[4] = refx[1];
    _temp_debug_data[5] = refx[2];

    _temp_debug_data[6] = refy[0];
    _temp_debug_data[7] = refy[1];
    _temp_debug_data[8] = refy[2];

    GLOBAL_Y_LIPM = refy[0];
    GLOBAL_X_LIPM = refx[0];
    GLOBAL_Z_LIPM = userData->WalkReadyCOM[Zdir];

    GLOBAL_Y_LIPM_d = refy[1];
    GLOBAL_X_LIPM_d = refx[1];

    GLOBAL_X_RF = window[0].right_foot_ref.x;
    GLOBAL_Y_RF = window[0].right_foot_ref.y;

    GLOBAL_X_LF = window[0].left_foot_ref.x;
    GLOBAL_Y_LF = window[0].left_foot_ref.y;

    GLOBAL_ZMP_REF_X = window[0].zmp.x;
    GLOBAL_ZMP_REF_Y = window[0].zmp.y;

    double Global[3],Local[3];
    Global[0] = GLOBAL_X_LIPM;
    Global[1] = GLOBAL_Y_LIPM;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LIPM_n =  Local[0];
    GLOBAL_Y_LIPM_n =  Local[1];

    Global[0] = GLOBAL_X_LIPM_d;
    Global[1] = GLOBAL_Y_LIPM_d;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LIPM_d_n =  Local[0];
    GLOBAL_Y_LIPM_d_n =  Local[1];

    Global[0] = GLOBAL_X_RF;
    Global[1] = GLOBAL_Y_RF;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_RF_n = Local[0];
    GLOBAL_Y_RF_n = Local[1];

    Global[0] = GLOBAL_X_LF;
    Global[1] = GLOBAL_Y_LF;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LF_n = Local[0];
    GLOBAL_Y_LF_n = Local[1];

    if(pv_Index < 1)
    {
        printf(">>>>>>>>>>>>>>>>>>>  WMG \n");
        printf("zmp[0].x: %f   zmp[0].y: %f  \n",window[0].zmp.x,window[0].zmp.y);
        printf("com x: %f com y: %f  \n",GLOBAL_X_LIPM,GLOBAL_Y_LIPM);
        printf("GLOBAL_X_RF : %f   GLOBAL_Y_RF y: %f  \n",GLOBAL_X_RF,GLOBAL_Y_RF);
        printf("GLOBAL_X_LF : %f   GLOBAL_Y_LF y: %f  \n",GLOBAL_X_LF,GLOBAL_Y_LF);
    }
    pv_Index++ ;
}
void WBIK()
{
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;

    double Global[3],Local[3];


    // Task Space Command

    if(pv_Index ==1)
    {
        // ankle torque control;

        RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDPitch =0.;
        LDPitch =0.;
        RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDRoll =0.;
        LDRoll =0.;

        RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDPitch2 =0.;
        LDPitch2 =0.;

        RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDRoll2 =0.;
        LDRoll2 =0.;

        Zctrl = FootForceControl(0,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,1,0.0);
        Zctrl = 0.;
        Zctrl2 = FootForceControl2(0,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,1,0.0);
        Zctrl2 = 0.;

        HUBO2ZMPInitLegLength(0.,0.,0);
        Add_FootTask[RIGHT][Zdir] = 0.;
        Add_FootTask[LEFT][Zdir] = 0.;

        RecoverRightLegLength(0.,0.,0);
        RecoverLeftLegLength(0.,0.,0);
        Add_Leg_Recovery[RIGHT][Zdir] = 0.;
        Add_Leg_Recovery[LEFT][Zdir] = 0.;

        // zmp control
        kirkZMPCon_XP2(0,0,0);
        kirkZMPCon_YP2(0,0,0);
        Del_PC_X_DSP_XZMP_CON = 0;
        Del_PC_Y_DSP_YZMP_CON = 0;

        I_ZMP_CON_X = 0.0f;
        I_ZMP_CON_Y = 0.0f;


        y_i_1 = 0;
        y_i_11= 0;
        u_i_1 = 0;
        u_i_11 = 0;
        NotchFilter_GyroRollControlInput(0,0);
        NotchFilter_GyroPitchControlInput(0,0);
        NotchFilter_GyroRollVel(0,0);
        NotchFilter_GyroPitchVel(0,0);
        GLOBAL_Xori_RF_last = 0;
        GLOBAL_Xori_LF_last = 0;
        GLOBAL_Yori_RF_last = 0;
        GLOBAL_Yori_LF_last = 0;

        GLOBAL_Xori_RF2_last = 0;
        GLOBAL_Xori_LF2_last = 0;
        GLOBAL_Yori_RF2_last = 0;

        GLOBAL_Yori_LF2_last = 0;

        U_Gain = 0.;
        GLOBAL_Xori_RF = 0.;
        GLOBAL_Xori_LF = 0.;
        GLOBAL_Yori_RF = 0.;
        GLOBAL_Yori_LF = 0.;

        printf("WBIK >>>>>>>>>>>>>>>>> \n");
        printf("GLOBAL_X_LIPM_n : %f   GLOBAL_Y_LIPM_n:%f \n",GLOBAL_X_LIPM_n,GLOBAL_Y_LIPM_n);
        printf("GLOBAL_Y_LF_n : %f   GLOBAL_Y_RF_n:%f \n",GLOBAL_Y_LF_n,GLOBAL_Y_RF_n);
        printf("GLOBAL_X_RF : %f   GLOBAL_Y_RF:%f \n",GLOBAL_X_RF,GLOBAL_Y_RF);
        printf("GLOBAL_X_LF : %f   GLOBAL_Y_LF:%f \n",GLOBAL_X_LF,GLOBAL_Y_LF);
    }

    // pelvis
    qtRX(1.0*0*D2R, temp2des_qPEL_4x1);
    qtRY(1.0*0*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);



    //------------- CoM
    Local[0] = GLOBAL_X_LIPM_n + G_DSP_X*(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X + I_ZMP_CON_X*0.)*ZMP_FeedBack_ONOFF;// + ZMPControlX*OnOff_compliance;
    if(Inv_ONOFF == 0){
        U_Gain = 0;
    }
//    if(FLAG_SingleLog == true)
//    {
//        if(U0_Gain < 0.9)
//        {
//            U0_Gain += 0.005;
//        }
//    } else
//    {
//        if(U0_Gain > 0.8)
//        {
//            U0_Gain -= 0.005;
//        }
//    }

    CONT_Y = Local[1] =  (GLOBAL_Y_LIPM_n)*(U0_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y + I_ZMP_CON_Y*0.0)*ZMP_FeedBack_ONOFF;// + ZMPControlY*OnOff_compliance;

    Local[2]=0;
    Local2Global(Local,Global);

    des_pCOM_3x1_hat[Xdir] = Global[0];
    des_pCOM_3x1_hat[Ydir] = Global[1];
    des_pCOM_3x1_hat[Zdir] = userData->WalkReadyCOM[Zdir];// + GLOBAL_Z_LIPM;// - (fsm->AddRightFootInfos[0][2] + fsm->AddLeftFootInfos[0][2])*0.7;

    double RotX[9],RotY[9],RotZ[9],RotYX[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = 0.*D2R;
    TorsoOri[1] = 0.*D2R;
    TorsoOri[2] = 0.;

    RZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);

    RX(TorsoOri_n[0],RotX);
    RY(TorsoOri_n[1],RotY);

    mult_mm(RotY,3,3,RotX,3,RotYX);

    mult_mv(RotYX,3,3,des_pCOM_3x1_hat,des_pCOM_3x1);
    //---------------

    // ------------ RF
    des_pRF_3x1_hat[Xdir] = GLOBAL_X_RF;
    des_pRF_3x1_hat[Ydir] = GLOBAL_Y_RF;
    des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Zctrl*0.5*impONOFF;//+ Zctrl2*0.5*impONOFF2+ Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[RIGHT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,des_pRF_3x1_hat,des_pRF_3x1);

    double RY,RP,RR;
    double temp_QT[4]={1,0,0,0};
    QTcross(temp_QT,temp2des_qPEL_4x1,temp1des_qPEL_4x1);
    QTcross(temp1des_qPEL_4x1,temp4des_qPEL_4x1,temp3des_qPEL_4x1);
    QT2YPR(temp3des_qPEL_4x1,RY,RP,RR);


    // ------------ LF
    des_pLF_3x1_hat[Xdir] = GLOBAL_X_LF;
    des_pLF_3x1_hat[Ydir] = GLOBAL_Y_LF;
    des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF - Zctrl*0.5*impONOFF;//- Zctrl2*0.5*impONOFF2+ Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[LEFT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,des_pLF_3x1_hat,des_pLF_3x1);




    if(pv_Index <=1)
    {
        printf("%d WBIK*********** right x: %f  y:%f   z:%f  \n",pv_Index, des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
        printf("%d WBIK*********** left  x: %f  y:%f   z:%f  \n",pv_Index,des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
        printf("%d WBIK COM x: %f  y: %f   z:%f \n",pv_Index,des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
        printf("%d  Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
        printf("%d  Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
        printf("%d  WBIK Pelvis :  %f   %f   %f \n",pv_Index,WBIK_Q[0],WBIK_Q[1],WBIK_Q[2]);
    }
    // 1 . PELVIS Orientation
    Pel_Yaw = (window[0].right_foot_ref.yaw + window[0].left_foot_ref.yaw)*D2R/2.;
    qtRZ(Pel_Yaw, temp1des_qPEL_4x1);
    qtRX(1.0*0*D2R, temp2des_qPEL_4x1);
    qtRY(0.*D2R, temp4des_qPEL_4x1);
//    qtRY(30*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);

    // 2. Right Foot Orientation
    RightYaw = window[0].right_foot_ref.yaw*D2R;
    RightPitch = window[0].right_foot_ref.pitch*D2R - RDPitch_atc*pitch_torque_ONOFF*D2R;
    RightRoll  = window[0].right_foot_ref.roll*D2R - RDRoll_atc*roll_torque_ONOFF*D2R;//RA_Notch_Filter(RDRoll_atc*D2R,1)*roll_torque_ONOFF;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp4des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp2des_qRF_4x1,temp5des_qRF_4x1);

    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];

    // 6 . LF Orietation
    LeftYaw = window[0].left_foot_ref.yaw*D2R ;
    LeftPitch = window[0].left_foot_ref.pitch*D2R - LDPitch_atc*pitch_torque_ONOFF*D2R;
//    LeftRoll = window[0].left_foot_ref.roll*D2R - LDRoll_atc*roll_torque_ONOFF*D2R;//LA_Notch_Filter(LDRoll_atc*D2R,1)*roll_torque_ONOFF;
    LeftRoll = window[0].left_foot_ref.roll*D2R - LA_Notch_Filter(LDRoll_atc*D2R,1)*roll_torque_ONOFF;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];

    memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));


    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    // Ankle Joint control using gyro

    GLOBAL_Xori_RF_n = GLOBAL_Xori_RF*cos(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(WBIK_Q[idRHY]);
    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(WBIK_Q[idRHY]);

    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(WBIK_Q[idLHY]);
    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(WBIK_Q[idLHY]);


    for(int i=0; i<=LAR; i++)
    {
        FWRefAngleCurrent[i] = WBIK_Q[i+7]*R2D;

        FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D + GLOBAL_Xori_RF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_RAR*Sagging_Comp_ONOFF - RDRoll*ssp_torque_ONOFF;
        FWRefAngleCurrent[RAP] = WBIK_Q[RAP+7]*R2D + GLOBAL_Yori_RF*Gyro_Ankle_FeedBack_ONOFF - RDPitch*ssp_torque_ONOFF;

        FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D + GLOBAL_Xori_LF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_LAR*Sagging_Comp_ONOFF - LDRoll*ssp_torque_ONOFF;
        FWRefAngleCurrent[LAP] = WBIK_Q[LAP+7]*R2D + GLOBAL_Yori_LF*Gyro_Ankle_FeedBack_ONOFF - LDPitch*ssp_torque_ONOFF;


        FWRefAngleCurrent[LHR] = WBIK_Q[LHR+7]*R2D + GLOBAL_LHR_Comp*Sagging_Comp_ONOFF;//+ DES_GLOBAL_VIB_Y_LF*R2D*Vibration_Control_ONOFF;
        FWRefAngleCurrent[RHR] = WBIK_Q[RHR+7]*R2D + GLOBAL_RHR_Comp*Sagging_Comp_ONOFF;//+ DES_GLOBAL_VIB_Y_RF*R2D*Vibration_Control_ONOFF;


        jCon->Joints[i]->RefAngleCurrent = FWRefAngleCurrent[i];
    }

    if(OnOff_compliance == true)
    {

    } else if(window[0].state == STATE_EMPTY)
    {
        FINAL_TIMER  = FINAL_TIMER + 0.005;
//        walk_flag = 0;
        if(FINAL_TIMER > STOP_TIME)
        {
            printf("Walking is finished and Walkflag is set to 0 \n");
            walk_flag = 0;
            continouse_walking_flag = false;
            walk_start_flag = false;
            walkstop_liftbox_flag = false;
            stop_flag = false;
            __JOY_COMMAND_MODE = false;
            FLAG_SingleLog = false;
            FLAG_SingleLogStart = false;
            LandingState = FINAL;
        }
    }
}
void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2])
{
    /*
     * Initialize every control period(0.1sec)
     */
    int flag1 = 0,flag2 = 0 ,Rflag1 = 0,Rflag2 = 0,Lflag1 = 0,Lflag2 = 0;
    static int before_ind = 1;

    for(int i = 0;i<3;i++)
    {
        for(int j = 0;j<15;j++)
        {
            U[j][i] = 0.;
            V[j] = 0;
        }
    }

    if(window[0].state == DSP_INIT_RF)
    {
        // Left foot = 1, right foot  = -1
        IND[0] = 1.;
        for(int i=0;i<=14;i++)
        {
                if((window[i*sampling_tic].state == DSP_INIT_RF) && flag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                } else if((window[i*sampling_tic].state == SSP_RF) && flag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                } else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 )
                {
                    U[i][1] = 1;
                    V[i] = 0;
                    flag2 = 1;
                } else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag2 == 1 )
                {
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }
    } else if(window[0].state == DSP_INIT_LF)
    {
        // Left foot = 1, right foot  = -1
        IND[0] = -1.;
        for(int i=0;i<=14;i++)
        {
                if((window[i*sampling_tic].state == DSP_INIT_LF) && flag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                } else if((window[i*sampling_tic].state == SSP_LF ) && flag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                } else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag1 == 1 )
                {
                    U[i][1] = 1;
                    V[i] = 0;
                } else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 )
                {
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }

    } else if(window[0].state == SSP_RF || window[0].state == DSP_RF)
    {
        IND[0] = 1;
        before_ind = 1;

        IND[0] = IND[0] + 1;

        if(window[0 + sampling_tic].state == DSP_LF)
        {
            IND[1] = 0;
        }

        Rflag1 = 0; Rflag2=0;
        for(int i=0;i<=14;i++)
        {
                //L SSP 0
                if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                    Lflag1 = 0;
                    Lflag2 = 0;
                // R SSP 1
                } else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    Lflag1 = 1;
                // L SSP 2
                } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 1)
                {
                    U[i][1] = 1;
                    V[i] = 0;
                    Lflag2 = 1;
                // R SSP 3
                } else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 1)
                {
                    U[i][2] = 1;
                    V[i] = 0;
                    Lflag1 = 0;
                } else if((window[i*sampling_tic].state == DSP_FINAL ||window[i*sampling_tic].state == STATE_EMPTY))
                {
                    U[i][0] = 0;
                    U[i][1] = 0;
                    U[i][2] = 0;
                    V[i] = 1;
                }
        }
    // R SSP
    } else if(window[0].state == SSP_LF || window[0].state == DSP_LF)
    {
        // left swing phase
        IND[0] = -1.;
        before_ind = -1;
        IND[1] = IND[1] + 1;

        if(window[0 + sampling_tic].state == DSP_RF)
        {
                IND[1] = 0;
        }

        for(int i=0;i<=14;i++)
        {
            Lflag1 = 0; Lflag2=0;
            //R SSP 0
            if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 0)
            {
                U[i][0] = 0;
                V[i] = 1;
                Rflag1 = 0;
                Rflag2 = 0;
            // R SSP 1
            } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Rflag2 == 0)
            {
                U[i][0] = 1;
                V[i] = 0;
                Rflag1 = 1;
            // L SSP 2
            } else if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 1)
            {
                U[i][1] = 1;
                V[i] = 0;
                Rflag2 = 1;
            // R SSP 3
            } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF ) && Rflag2 == 1)
            {
                U[i][2] = 1;
                V[i] = 0;
                Rflag1 = 0;
            } else if((window[i*sampling_tic].state == DSP_FINAL || window[i*sampling_tic].state == STATE_EMPTY ))
            {
                U[i][0] = 0;
                U[i][1] = 0;
                U[i][2] = 0;
                V[i] = 1;
            }
        }
    }
}

void approach_last()
{
    make_last_liftbox_footprint();
}

void make_last_liftbox_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    STEP_LENGTH = 0.1;
    STEP_ANGLE = 0.0;
    STEP_OFFSET = 0.21 + 0.05*fabs(STEP_ANGLE/10.0);

    for(int i=0;i<3;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] - STEP_LENGTH - STEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + STEP_LENGTH + STEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }
        if(i==2)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        } else
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1] - STEP_LENGTH;
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1] + STEP_LENGTH;
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }

        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
    }
}

void make_last_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    for(int i=0;i<3;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1]- RSTEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1]+ RSTEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }
        if(i==2)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        } else
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }

        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
    }
}

void make_last_door_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    STEP_LENGTH = 0.2;
    STEP_ANGLE = 0.0;
    STEP_OFFSET = 0.21 + 0.05*fabs(STEP_ANGLE/10.0);

    for(int i=0;i<2;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + STEP_LENGTH;
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] - STEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + STEP_LENGTH;
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + STEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        } else if(i==1)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        }
        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach door foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
    }
}

void SetFootPrint(int num)
{
    MnFootPrint.StepNum = num;
    double Mn_DSP_time = 0.5;
    double Mn_SSP_time = 1.2;
    int rl = 0;

    double SteppingStoneX[8] = {0.3, 0.6, 0.9, 1.2, 1.5, 1.6, 1.9, 2.1};
    double SteppingStoneY[8] = {-0.15, 0.2, -0.05, 0.15, -0.08, 0.2, -0.07, 0.15};
    for(int i=0;i<MnFootPrint.StepNum+2;i++)
    {
        if(i==0)
        {
            MnFootPrint.FootPrint[i].rfoot[0] = SteppingStoneX[i];//0.3;
            MnFootPrint.FootPrint[i].rfoot[1] = SteppingStoneY[i];//- 0.11 + 0.3*0.015*(rand()%20 - 10);
            MnFootPrint.FootPrint[i].rfoot[2] = FK_pRFoot_3x1[2];
            MnFootPrint.FootPrint[i].rori[0] = FK_RFoot_yaw*R2D;
            MnFootPrint.FootPrint[i].rori[1] = FK_RFoot_roll*R2D;
            MnFootPrint.FootPrint[i].rori[2] = FK_RFoot_pitch*R2D;

            MnFootPrint.FootPrint[i].lfoot[0] = FK_pLFoot_3x1[0];
            MnFootPrint.FootPrint[i].lfoot[1] = FK_pLFoot_3x1[1];//0.3*0.01*(rand()%10.);
            MnFootPrint.FootPrint[i].lfoot[2] = FK_pLFoot_3x1[2];
            MnFootPrint.FootPrint[i].lori[0] = FK_LFoot_yaw*R2D;
            MnFootPrint.FootPrint[i].lori[1] = FK_LFoot_roll*R2D;
            MnFootPrint.FootPrint[i].lori[2] = FK_LFoot_pitch*R2D;
            rl = 1;

            MnFootPrint.DSPtime[i] = Mn_DSP_time *2.;
            MnFootPrint.SSPtime[i] = Mn_SSP_time;
        }
        else if(i==MnFootPrint.StepNum-1)
        {
            if(rl == 0)
            {//move right foot
                MnFootPrint.FootPrint[i].rfoot[0] = SteppingStoneX[i];//MnFootPrint.FootPrint[i-1].lfoot[0] + 0.3;
                MnFootPrint.FootPrint[i].rfoot[1] = - 0.105;
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0] = 0.;
                MnFootPrint.FootPrint[i].rori[1] = 0.;
                MnFootPrint.FootPrint[i].rori[2] = 0.;

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = MnFootPrint.FootPrint[i-1].lfoot[1];
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] =  MnFootPrint.FootPrint[i-1].lori[0];
                MnFootPrint.FootPrint[i].lori[1] =  MnFootPrint.FootPrint[i-1].lori[1];
                MnFootPrint.FootPrint[i].lori[2] =  MnFootPrint.FootPrint[i-1].lori[2];
                rl = 1;
            } else
            {//move left foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = MnFootPrint.FootPrint[i-1].rfoot[1];
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0] =  MnFootPrint.FootPrint[i-1].rori[0];
                MnFootPrint.FootPrint[i].rori[1] =  MnFootPrint.FootPrint[i-1].rori[1];
                MnFootPrint.FootPrint[i].rori[2] =  MnFootPrint.FootPrint[i-1].rori[2];

                MnFootPrint.FootPrint[i].lfoot[0] = SteppingStoneX[i];//MnFootPrint.FootPrint[i-1].rfoot[0] + 0.3;
                MnFootPrint.FootPrint[i].lfoot[1] = 0.105;
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] = 0.;
                MnFootPrint.FootPrint[i].lori[1] = 0.;
                MnFootPrint.FootPrint[i].lori[2] = 0.;
                rl = 0;
            }
            MnFootPrint.DSPtime[i] = Mn_DSP_time;
            MnFootPrint.SSPtime[i] = Mn_SSP_time;
        }
        else if(i==MnFootPrint.StepNum)
        {
            if(rl == 0)
            {//move right foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = - 0.105;
                MnFootPrint.FootPrint[i].rfoot[2] = 0.;
                MnFootPrint.FootPrint[i].rori[0] = 0.;
                MnFootPrint.FootPrint[i].rori[1] = 0.;
                MnFootPrint.FootPrint[i].rori[2] = 0.;

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = MnFootPrint.FootPrint[i-1].lfoot[1];
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] =  MnFootPrint.FootPrint[i-1].lori[0];
                MnFootPrint.FootPrint[i].lori[1] =  MnFootPrint.FootPrint[i-1].lori[1];
                MnFootPrint.FootPrint[i].lori[2] =  MnFootPrint.FootPrint[i-1].lori[2];
                rl = 1;
            } else
            {//move left foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = MnFootPrint.FootPrint[i-1].rfoot[1];
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0] =  MnFootPrint.FootPrint[i-1].rori[0];
                MnFootPrint.FootPrint[i].rori[1] =  MnFootPrint.FootPrint[i-1].rori[1];
                MnFootPrint.FootPrint[i].rori[2] =  MnFootPrint.FootPrint[i-1].rori[2];

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = 0.105;
                MnFootPrint.FootPrint[i].lfoot[2] = 0.;
                MnFootPrint.FootPrint[i].lori[0] = 0.;
                MnFootPrint.FootPrint[i].lori[1] = 0.;
                MnFootPrint.FootPrint[i].lori[2] = 0.;
                rl = 0;
            }
            MnFootPrint.DSPtime[i] = Mn_DSP_time;
            MnFootPrint.SSPtime[i] = Mn_SSP_time;
        }
        else if(i==MnFootPrint.StepNum+1)
        {
            if(rl == 0)
            {//move right foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = MnFootPrint.FootPrint[i-1].rfoot[1];
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0]  = MnFootPrint.FootPrint[i-1].rori[0];
                MnFootPrint.FootPrint[i].rori[1]  = MnFootPrint.FootPrint[i-1].rori[1];
                MnFootPrint.FootPrint[i].rori[2]  = MnFootPrint.FootPrint[i-1].rori[2];

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = MnFootPrint.FootPrint[i-1].lfoot[1];
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] =  MnFootPrint.FootPrint[i-1].lori[0];
                MnFootPrint.FootPrint[i].lori[1] =  MnFootPrint.FootPrint[i-1].lori[1];
                MnFootPrint.FootPrint[i].lori[2] =  MnFootPrint.FootPrint[i-1].lori[2];
                rl = 1;
            } else
            {//move left foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = MnFootPrint.FootPrint[i-1].rfoot[1];
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0] =  MnFootPrint.FootPrint[i-1].rori[0];
                MnFootPrint.FootPrint[i].rori[1] =  MnFootPrint.FootPrint[i-1].rori[1];
                MnFootPrint.FootPrint[i].rori[2] =  MnFootPrint.FootPrint[i-1].rori[2];

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = MnFootPrint.FootPrint[i-1].lfoot[1];
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] =  MnFootPrint.FootPrint[i-1].lori[0];
                MnFootPrint.FootPrint[i].lori[1] =  MnFootPrint.FootPrint[i-1].lori[1];
                MnFootPrint.FootPrint[i].lori[2] =  MnFootPrint.FootPrint[i-1].lori[2];
                rl = 0;
            }
            MnFootPrint.DSPtime[i] = Mn_DSP_time*2.;
            MnFootPrint.SSPtime[i] = 0.0;
        }
        else
        {
            if(rl == 0)
            {//move right foot
                MnFootPrint.FootPrint[i].rfoot[0] = SteppingStoneX[i];//MnFootPrint.FootPrint[i-1].lfoot[0] + 0.3;
                MnFootPrint.FootPrint[i].rfoot[1] = SteppingStoneY[i];//- 0.11 + 0.3*0.015*(rand()%20 - 10);
                MnFootPrint.FootPrint[i].rfoot[2] = 0.;
                MnFootPrint.FootPrint[i].rori[0] = 0.;
                MnFootPrint.FootPrint[i].rori[1] = 0.;
                MnFootPrint.FootPrint[i].rori[2] = 0.;

                MnFootPrint.FootPrint[i].lfoot[0] = MnFootPrint.FootPrint[i-1].lfoot[0];
                MnFootPrint.FootPrint[i].lfoot[1] = MnFootPrint.FootPrint[i-1].lfoot[1];
                MnFootPrint.FootPrint[i].lfoot[2] = MnFootPrint.FootPrint[i-1].lfoot[2];
                MnFootPrint.FootPrint[i].lori[0] =  MnFootPrint.FootPrint[i-1].lori[0];
                MnFootPrint.FootPrint[i].lori[1] =  MnFootPrint.FootPrint[i-1].lori[1];
                MnFootPrint.FootPrint[i].lori[2] =  MnFootPrint.FootPrint[i-1].lori[2];
                rl = 1;
            } else
            {//move left foot
                MnFootPrint.FootPrint[i].rfoot[0] = MnFootPrint.FootPrint[i-1].rfoot[0];
                MnFootPrint.FootPrint[i].rfoot[1] = MnFootPrint.FootPrint[i-1].rfoot[1];
                MnFootPrint.FootPrint[i].rfoot[2] = MnFootPrint.FootPrint[i-1].rfoot[2];
                MnFootPrint.FootPrint[i].rori[0] =  MnFootPrint.FootPrint[i-1].rori[0];
                MnFootPrint.FootPrint[i].rori[1] =  MnFootPrint.FootPrint[i-1].rori[1];
                MnFootPrint.FootPrint[i].rori[2] =  MnFootPrint.FootPrint[i-1].rori[2];

                MnFootPrint.FootPrint[i].lfoot[0] = SteppingStoneX[i];//MnFootPrint.FootPrint[i-1].rfoot[0] + 0.3;
                MnFootPrint.FootPrint[i].lfoot[1] = SteppingStoneY[i];//0.11 + 0.3*0.015*(rand()%20 - 10);
                MnFootPrint.FootPrint[i].lfoot[2] = 0.;
                MnFootPrint.FootPrint[i].lori[0] = 0.;
                MnFootPrint.FootPrint[i].lori[1] = 0.;
                MnFootPrint.FootPrint[i].lori[2] = 0.;
                rl = 0;
            }
            MnFootPrint.DSPtime[i] = Mn_DSP_time;
            MnFootPrint.SSPtime[i] = Mn_SSP_time;
        }


    }

    printf("MakeFootPrint like this!!!\n");
    for(int i=0;i<MnFootPrint.StepNum+2;i++)
    {
        printf("%d th FootPrint ]]]]\n",i);
        printf("R:[%f, %f] L:[%f, %f]\n",
               MnFootPrint.FootPrint[i].rfoot[0],MnFootPrint.FootPrint[i].rfoot[1],MnFootPrint.FootPrint[i].lfoot[0],MnFootPrint.FootPrint[i].lfoot[1]);
    }

}

/****************************** 6. Controller *******************************/
void Controller(){

    double init_start_time = 0.6,init_final_time = 1.5;
    double lpf_ratio = 0.9,lpf_ratio2 = 0.7;

    LF_LPF_Fz = LF_LPF_Fz*(1.0 - lpf_ratio) + sharedData->FT[LAFT].Fz*(lpf_ratio);
    RF_LPF_Fz = RF_LPF_Fz*(1.0 - lpf_ratio) + sharedData->FT[RAFT].Fz*(lpf_ratio);

    LF_LPF_Fx = LF_LPF_Fx*(1.0 - lpf_ratio) + sharedData->FT[LAFT].Fx *(lpf_ratio);
    RF_LPF_Fx = RF_LPF_Fx*(1.0 - lpf_ratio) + sharedData->FT[RAFT].Fx *(lpf_ratio);

    LF_LPF_Fy = LF_LPF_Fy*(1.0 - lpf_ratio) + sharedData->FT[LAFT].Fy *(lpf_ratio);
    RF_LPF_Fy = RF_LPF_Fy*(1.0 - lpf_ratio) + sharedData->FT[RAFT].Fy *(lpf_ratio);


    LF_LPF_My = LF_LPF_My*(1.0 - lpf_ratio) + sharedData->FT[LAFT].My *(lpf_ratio);
    RF_LPF_My = RF_LPF_My*(1.0 - lpf_ratio) + sharedData->FT[RAFT].My *(lpf_ratio);


    LF_LPF_Mx = LF_LPF_Mx*(1.0 - lpf_ratio) + sharedData->FT[LAFT].Mx *(lpf_ratio);
    RF_LPF_Mx = RF_LPF_Mx*(1.0 - lpf_ratio) + sharedData->FT[RAFT].Mx *(lpf_ratio);

    switch(window[0].state)
    {
    case DSP_INIT_RF:
    {// For continous walking
        LandingState = DSP;

        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }
        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }
        Leg_Length_Control();
        break;
    }
    case DSP_INIT_LF:
    {// For continous walking
        LandingState = DSP;

        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }
        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }
        Leg_Length_Control();
        break;
    }
    case DSP_FINAL:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = FINAL;
//        Upperbody_Gain_Lock();

        Leg_Length_Control();
        RecoverLegLength();
        break;
    }
    case DSP_RF:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = DSP;
        break;
    }
    case DSP_LF:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = DSP;
        break;
    }
    case SSP_RF:
    {
        LandingState = RSSP;
        break;
    }
    case SSP_LF:
    {
        LandingState = LSSP;

    }
    case STATE_EMPTY:
    {

//                Upperbody_Gain_Lock();
        Leg_Length_Control();
        RecoverLegLength();
        break;
    }
    default:
        break;
    }

    ReactiveControl(1,1,1);
//    Pelvis_Ori_Control();
    Compensator_deflection(window[0].state);
    Kirk_Control();
    LandingControl(window[0].timer.current,window[0].state,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz);
    Gyro_Feedback();
}

void State_Estimator(double p,double q, double r, double ax, double ay, double orientation[3])
{
    double eye[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    double dA[4][4] = {{0.,-p*0.5*0.005,-q*0.5*0.005,-r*0.5*0.005},{p*0.5*0.005,0.,r*0.5*0.005,-q*0.5*0.005},{q*0.5*0.005,-r*0.5*0.005,0.,p*0.5*0.005},{r*0.5*0.005,q*0.5*0.005,-p*0.5*0.005,0.}};
    double A[4][4] = {{0.,},},z[4] = {0.,};
    double ang[3] = {0.,},state_global_pelvis[6] = {0.,};

    /* Kalman filter for orientation estimation */
    mat4by4plus4by4(eye,dA,A);

    ang[0] = ax*D2R;
    ang[1] = ay*D2R;
    ang[2] = 0.;

    EulerToQt(ang,z);

    Kalman(A,z,orientation);

    /* High pass filter */
    double a = 0.95;
    HPF_Estimated_Orientation[0] = a*(Old_HPF_Estimated_Orientation[0] + orientation[0] - Old_Estimated_Orientation[0]);
    HPF_Estimated_Orientation[1] = a*(Old_HPF_Estimated_Orientation[1] + orientation[1] - Old_Estimated_Orientation[1]);
    HPF_Estimated_Orientation[2] = a*(Old_HPF_Estimated_Orientation[2] + orientation[2] - Old_Estimated_Orientation[2]);

    Old_Estimated_Orientation[0] = orientation[0];
    Old_Estimated_Orientation[1] = orientation[1];
    Old_Estimated_Orientation[2] = orientation[2];

    Old_HPF_Estimated_Orientation[0] = HPF_Estimated_Orientation[0];
    Old_HPF_Estimated_Orientation[1] = HPF_Estimated_Orientation[1];
    Old_HPF_Estimated_Orientation[2] = HPF_Estimated_Orientation[2];

    /* Set state global pelvis, comp orientation */
    double comp_a = 0.6;
    Comp_Orientation[0] = ang[0]*(1. - comp_a) + comp_a*p*R2D;
    Comp_Orientation[1] = ang[1]*(1. - comp_a) + comp_a*q*R2D;
    Comp_Orientation[2] = ang[2]*(1. - comp_a) + comp_a*r*R2D;

    state_global_pelvis[0] = WBIK_Q[0];
    state_global_pelvis[1] = WBIK_Q[1];
    state_global_pelvis[2] = WBIK_Q[2];

    state_global_pelvis[3] = 0.;
    state_global_pelvis[4] = 0.;
    state_global_pelvis[5] = 0.;
}

void ZMP_intergral_control()
{

    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}

double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-69.2573,-0.9504}};
//    const double B[2] = {0,97.6827};
//    const double C[2] = {5.6935,0.0667};
//    const double D = -6.8597;
//    const double Kg[2] = {-0.4991,0.0824};
//    const double Og[2] = {5.0752,32.2715};


    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-109.662271123215, -1.748903000593}};


    const double B[2] = {0.000000000000, 146.363311122469};

    const double C[2] = {8.530951944626, 0.122815420704};

    const double D = -10.278255354971;

    const double Kg[2] = {-0.576731084285, 0.056374079926};

const double Og[2] = {3.124192093049, 13.017619274973};



    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 40.0) y = 40.0;
    else if(y < -40.0) y = -40.0;

    return y ;
}
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-137.9279,-1.4689}};
//    const double B[2] = {0,180.4183};
//    const double C[2] = {10.5159,0.1032};
//    const double D = -12.6697;

//    const double Kg[2] = {-0.6509,	0.0417};
//    const double Og[2] = {2.7954,10.9990};


    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-135.385519905204, -1.748903000593}};

    const double B[2] = {0.000000000000, 177.355177124865};

    const double C[2] = {10.337348079785, 0.122815420704};


    const double D = -12.454636240705;


    const double Kg[2] = {-0.620988468962, 0.046523011807};


   const double Og[2] = {2.634028183252, 8.323309982623};



    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
    y = Kg[0]*(x_new[0]) + Kg[1]*(x_new[1]);

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}

void Gyro_Feedback()
{
    if(pv_Index == 1)
    {
        y_i_1 = 0;
        y_i_11= 0;
        u_i_1 = 0;
        u_i_11 = 0;
        NotchFilter_GyroRollControlInput(0,0);
        NotchFilter_GyroPitchControlInput(0,0);
        NotchFilter_GyroRollVel(0,0);
        NotchFilter_GyroPitchVel(0,0);
        GLOBAL_Xori_RF_last = 0;
        GLOBAL_Xori_LF_last = 0;
        GLOBAL_Yori_RF_last = 0;
        GLOBAL_Yori_LF_last = 0;

        GLOBAL_Xori_RF2_last = 0;
        GLOBAL_Xori_LF2_last = 0;
        GLOBAL_Yori_RF2_last = 0;

        GLOBAL_Yori_LF2_last = 0;

//        U_Gain = 0.;
        GLOBAL_Xori_RF = 0.;
        GLOBAL_Xori_LF = 0.;
        GLOBAL_Yori_RF = 0.;
        GLOBAL_Yori_LF = 0.;

        GLOBAL_Xori_RF_last2=0;
        GLOBAL_Yori_RF_last2=0;
        GLOBAL_Xori_LF_last2=0;
        GLOBAL_Yori_LF_last2=0;
    }

    static int CNT_AnkleControl1=0,CNT_AnkleControl2=0,CNT_AnkleControl3=0,CNT_AnkleControl4=0;

    FOGRollVel_NF2 = NotchFilter_GyroRollVel(sharedData->FOG.RollVel,1);
    FOGPitchVel_NF2 = NotchFilter_GyroPitchVel(sharedData->FOG.PitchVel,1);

    FOGRollVel_LPF  = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGRollVel_LPF  + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGRollVel_NF2;
    FOGPitchVel_LPF = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGPitchVel_LPF + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGPitchVel_NF2;


    AnkleControl1 = (sharedData->FOG.Roll*1.0f + sharedData->FOG.RollVel*D2R*2.);
    AnkleControl2 = (sharedData->FOG.Pitch*1.0f + sharedData->FOG.PitchVel*D2R*2.0);


    den_a1 = 1;
    den_a2 = -0.801151070558751;//-0.854080685463467;//-0.909929988177738;
    num_b1 = 0.099424464720624;//0.072959657268267;//0.045035005911131;
    num_b2 = 0.099424464720624;//0.072959657268267;//0.045035005911131;

    u_i = AnkleControl1;
    y_i = -den_a2*y_i_1 + num_b1*u_i + num_b2*u_i_1; //1st order
    y_i_1 = y_i;
    u_i_1 = u_i;

    u_i1 = AnkleControl2;
    y_i1 = -den_a2*y_i_11 + num_b1*u_i1 + num_b2*u_i_11; //1st order
    y_i_11 = y_i1;
    u_i_11 = u_i1;

    if(y_i>20) y_i = 20;
    if(y_i<-20) y_i = -20;
    if(y_i1>20) y_i1 = 20;
    if(y_i1<-20) y_i1 = -20;

    FOGRollVel_NF = NotchFilter_GyroRollControlInput(y_i,1);
    FOGPitchVel_NF = NotchFilter_GyroPitchControlInput(y_i1,1);

    if(window[0].state == SSP_RF)
    {
        Foot_gainLF = 0.5*(1 - cos(PI*CNT_AnkleControl2/20));
        if(CNT_AnkleControl2<20)CNT_AnkleControl2++;

        if(EarlyLandingFlag[RIGHT] == 0)// If the right foot is not landed earlyer,
        {
            //supporting foot
            GLOBAL_Xori_LF = Foot_gainLF*(FOGRollVel_NF) + GLOBAL_Xori_LF_last*(1-Foot_gainLF);
            GLOBAL_Yori_LF = Foot_gainLF*(FOGPitchVel_NF) + GLOBAL_Yori_LF_last*(1-Foot_gainLF);
            GLOBAL_Xori_LF_last2 = GLOBAL_Xori_LF;
            GLOBAL_Yori_LF_last2 = GLOBAL_Yori_LF;
        }

        //swing foot
        GLOBAL_Xori_RF = GLOBAL_Xori_RF_last*(1-Foot_gainLF);
        GLOBAL_Yori_RF = GLOBAL_Yori_RF_last*(1-Foot_gainLF);

        GLOBAL_Xori_RF2 = Foot_gainLF*(0);
        GLOBAL_Yori_RF2 = Foot_gainLF*(0);

        GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
        GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;

        CNT_AnkleControl4 = 0;

    }else if(window[0].state == SSP_LF)
    {
        Foot_gainRF = 0.5*(1 - cos(PI*CNT_AnkleControl1/20));
        if(CNT_AnkleControl1<20)CNT_AnkleControl1++;

        if(EarlyLandingFlag[LEFT] == 0)// If the left foot is not landed earlyer,
        {
            //supporting foot
            GLOBAL_Xori_RF = Foot_gainRF*(FOGRollVel_NF) + GLOBAL_Xori_RF_last*(1-Foot_gainRF);
            GLOBAL_Yori_RF = Foot_gainRF*(FOGPitchVel_NF) + GLOBAL_Yori_RF_last*(1-Foot_gainRF);
            GLOBAL_Xori_RF_last2 = GLOBAL_Xori_RF;
            GLOBAL_Yori_RF_last2 = GLOBAL_Yori_RF;
        }

        //swing foot
        GLOBAL_Xori_LF = GLOBAL_Xori_LF_last*(1-Foot_gainRF);
        GLOBAL_Yori_LF = GLOBAL_Yori_LF_last*(1-Foot_gainRF);

        GLOBAL_Xori_LF2 = Foot_gainRF*(0);
        GLOBAL_Yori_LF2 = Foot_gainRF*(0);

        GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
        GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;
        CNT_AnkleControl4 = 0;

    }else if(window[0].state == DSP_LF || window[0].state == DSP_RF)
    {
        Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl4/50));
        if(CNT_AnkleControl4<50)CNT_AnkleControl4++;

        if(Pre_LandingState == RSSP)
        {
            GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);// when dsp, set Xori to zero
            GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);

            GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
            GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
        }
        if(Pre_LandingState == LSSP)
        {
            GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);// when dsp, set Xori to zero
            GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);

            GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
            GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
        }

        CNT_AnkleControl1 = CNT_AnkleControl2 = CNT_AnkleControl3 = 0;

        GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
        GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

        GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
        GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;

    }else if(window[0].state == DSP_FINAL)
    {
        Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl3/50));
        if(CNT_AnkleControl3<50)CNT_AnkleControl3++;

        if(Pre_LandingState == RSSP)
        {
            GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);
            GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);
            GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
            GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
        }

        if(Pre_LandingState == LSSP)
        {
            GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);
            GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);
            GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
            GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
        }


        CNT_AnkleControl1 = CNT_AnkleControl2 = 0;

        GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
        GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

        GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
        GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;

        GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
        GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;

        GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
        GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;
    }


}

double HUBO2ZMPInitLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.000011;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(sume > 0.08/KI) sume = .08/KI;
    else if(sume < -0.08/KI) sume = -.08/KI;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double RecoverRightLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double RecoverLeftLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double NotchFilter_GyroRollControlInput(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchControlInput(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroRollVel(double _input, int _reset)
{
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchVel(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}

void Kirk_Control()
{
    //printf("ZMP LOCAL %f \   Del_PC_X_DSP_XZMP_CON: %f \n",X_ZMP_Local,Del_PC_X_DSP_XZMP_CON);

    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;
    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;
}

double RMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
    if(state ==1)
    {
        mTorque = RDF*(HPF_Estimated_Orientation[1]*R2D*0.3 + sharedData->FOG.PitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state == 0)
    {
        mTorque = 0;
    }

    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 2500;//1125;
        T = 1.0;
    }else if(state == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }


        return CL*R2D;



}
double LMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired

        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
        mTorque = LDF*(HPF_Estimated_Orientation[1]*R2D*0.3 + sharedData->FOG.PitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state2 == 0)
    {
        mTorque = 0;
    }


    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
        d = 3000;//1125;
        T = 1.0;
    }else if(state2 == 0)
    {
        d = 3000;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }




        return CL*R2D;



}
double RMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state ==1)
    {
        mTorque = RDF*(HPF_Estimated_Orientation[0]*R2D*0.3 + sharedData->FOG.RollVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state == 0)
    {
        mTorque = 0;
    }

    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 2500;//1125;
        T = 1.0;
    }else if(state == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }


        return CL*R2D;



}
double LMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
        mTorque = LDF*(HPF_Estimated_Orientation[0]*R2D + sharedData->FOG.RollVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state2 == 0)
    {
        mTorque = 0;
    }


    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
        d = 2500;//3000;
        T = 1.0;
    }else if(state2 == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }




        return CL*R2D;



}

void get_zmp2()
{
        // Foot Center in Global Coord.
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

        qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);


        if(sharedData->FT[RAFT].Fz + sharedData->FT[LAFT].Fz > 50.)
        {
//            M_LF[0] =  sharedData->FT[LAFT].Mx;
//            M_LF[1] =  sharedData->FT[LAFT].My;
//            M_LF[2] =  sharedData->FT[LAFT].Mz;

//            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

//            M_RF[0] =  sharedData->FT[RAFT].Mx;
//            M_RF[1] =  sharedData->FT[RAFT].My;
//            M_RF[2] =  sharedData->FT[RAFT].Mz;

//            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

//            F_LF[0] = sharedData->FT[LAFT].Fx;
//            F_LF[1] = sharedData->FT[LAFT].Fy;
//            F_LF[2] = sharedData->FT[LAFT].Fz;

//            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

//            F_RF[0] = sharedData->FT[RAFT].Fx;
//            F_RF[1] = sharedData->FT[RAFT].Fy;
//            F_RF[2] = sharedData->FT[RAFT].Fz;

            M_LF[0] =  sharedData->FT[LAFT].Mx;
            M_LF[1] =  sharedData->FT[LAFT].My;
            M_LF[2] =  sharedData->FT[LAFT].Mz;

            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedData->FT[RAFT].Mx;
            M_RF[1] =  sharedData->FT[RAFT].My;
            M_RF[2] =  sharedData->FT[RAFT].Mz;

            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = 0.;
            F_LF[1] = 0.;
            F_LF[2] = sharedData->FT[LAFT].Fz;

            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = 0.;
            F_RF[1] = 0.;
            F_RF[2] = sharedData->FT[RAFT].Fz;

            QTtransform(des_qRF_4x1,F_RF,F_RF_Global);

            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

            cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
            cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)

            sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
            sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
            sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

            zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
            zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
            zmp[2] = 0.;

            diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter
            qCenter_bar[0] = qCenter[0];
            qCenter_bar[1] = -qCenter[1];
            qCenter_bar[2] = -qCenter[2];
            qCenter_bar[3] = -qCenter[3];

            QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)
            temp2[0] = GLOBAL_ZMP_REF_X;
            temp2[1] = GLOBAL_ZMP_REF_Y;
            temp2[2] = 0;
            diff_vv(temp2,3,pCenter,temp1);
            QTtransform(qCenter_bar, temp1, zmp_ref_local);

            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];

            X_ZMP_REF_Global = GLOBAL_ZMP_REF_X;
            Y_ZMP_REF_Global = GLOBAL_ZMP_REF_Y;


        }
}

void LandingControl(int cur_time,int state,double rForce,double lForce)
{
    static double temp_Z_RF[3]={0.,},temp_Z_LF[3]={0.};
    double Force_Threshold = 100.,target_time = 0.4;

    if(state == SSP_RF)
    {
        // Recovery left supporting leg
        if(window[0].timer.current < 0.4){
            GLOBAL_Z_LF_goal[0] = window[0].left_foot_ref.z;    GLOBAL_Z_LF_goal[1] = 0.;   GLOBAL_Z_LF_goal[2] = 0.;
            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_LF_last ,GLOBAL_Z_LF_goal  , temp_Z_LF);
        }else{
            GLOBAL_Z_LF_last[0] = GLOBAL_Z_LF;
            temp_Z_LF[0]= window[0].left_foot_ref.z;
        }

        GLOBAL_Z_LF = temp_Z_LF[0];
        // ----------------------------

        // Detect early landing of Right foot
        if(window[0].timer.current <= 0.4)
        {
            EarlyLandingFlag[RIGHT] = 0;
            GLOBAL_Z_RF = window[0].right_foot_ref.z;
        }else
        {
            if(rForce > Force_Threshold && window[0].timer.current>= 0.65){
                if(EarlyLandingFlag[RIGHT] == 0)
                {
                    GLOBAL_Z_RF_last_earlylanding = GLOBAL_Z_RF;
                }

                EarlyLandingFlag[RIGHT] = 1;

            }

            if(EarlyLandingFlag[RIGHT] == 0){ // normal walking
                GLOBAL_Z_RF = window[0].right_foot_ref.z;
            }else{
                GLOBAL_Z_RF = GLOBAL_Z_RF_last_earlylanding;

            }
            GLOBAL_Z_RF_last[0] = GLOBAL_Z_RF;

        }

    }else if(state == SSP_LF){

        // Recovery right supporting leg up to left off phase
        if(window[0].timer.current < 0.4){

            GLOBAL_Z_RF_goal[0] = window[0].right_foot_ref.z;    GLOBAL_Z_RF_goal[1] = 0.;   GLOBAL_Z_RF_goal[2] = 0.;

            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_RF_last , GLOBAL_Z_RF_goal , temp_Z_RF);

        }else{
            GLOBAL_Z_RF_last[0] = GLOBAL_Z_RF;
            temp_Z_RF[0]= window[0].right_foot_ref.z;
        }

        GLOBAL_Z_RF = temp_Z_RF[0];
        // ----------------------------

        // Detect early landing of Left foot
        if(window[0].timer.current <= 0.4)
        {
            EarlyLandingFlag[LEFT] = 0;
            GLOBAL_Z_LF = window[0].left_foot_ref.z;

        }else
        {
            if(lForce > Force_Threshold && window[0].timer.current>= 0.65){

                if(EarlyLandingFlag[LEFT] == 0){

                    GLOBAL_Z_LF_last_earlylanding = GLOBAL_Z_LF;
                }
                EarlyLandingFlag[LEFT] = 1;

            }

            if(EarlyLandingFlag[LEFT] == 0){ // normal walking
                GLOBAL_Z_LF = window[0].left_foot_ref.z;
            }else{
                GLOBAL_Z_LF = GLOBAL_Z_LF_last_earlylanding;
            }

            GLOBAL_Z_LF_last[0] = GLOBAL_Z_LF;
        }



    }else if( state == DSP_FINAL)
    {

        if(window[0].timer.current < 0.4)
        {

        GLOBAL_Z_RF_goal[0] = window[0].right_foot_ref.z;    GLOBAL_Z_RF_goal[1] = 0.;   GLOBAL_Z_RF_goal[2] = 0.;

        Fifth( window[0].timer.current , 0.4, GLOBAL_Z_RF_last , GLOBAL_Z_RF_goal , temp_Z_RF);


        GLOBAL_Z_LF_goal[0] = window[0].left_foot_ref.z;    GLOBAL_Z_LF_goal[1] = 0.;   GLOBAL_Z_LF_goal[2] = 0.;

        Fifth( window[0].timer.current , 0.4, GLOBAL_Z_LF_last ,GLOBAL_Z_LF_goal  , temp_Z_LF);


            GLOBAL_Z_LF = temp_Z_LF[0];
            GLOBAL_Z_RF = temp_Z_RF[0];
        }

    }

    if(EarlyLanding_ONOFF == 0)
    {
        GLOBAL_Z_LF = window[0].left_foot_ref.z;
        GLOBAL_Z_RF = window[0].right_foot_ref.z;
    }





}
void Leg_Length_Control()
{
    // Integral control for nutral state
    // Leg_length Control
    double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

    if(window[0].state == DSP_INIT_RF || window[0].state == DSP_INIT_LF)
    {
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }
    else if(window[0].state == DSP_FINAL)
    {

        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }else if(window[0].state == STATE_EMPTY || window[0].timer.current < 2.0)
    {

    }

    if(Add_FootTask[RIGHT][Zdir] > 0.05)Add_FootTask[RIGHT][Zdir] =  0.05;
    if(Add_FootTask[RIGHT][Zdir] <-0.05)Add_FootTask[RIGHT][Zdir] = -0.05;

    if(Add_FootTask[LEFT][Zdir] > 0.05)Add_FootTask[LEFT][Zdir] =  0.05;
    if(Add_FootTask[LEFT][Zdir] <-0.05)Add_FootTask[LEFT][Zdir] = -0.05;
}
void RecoverLegLength()
{
    // Integral control for nutral state
    // Leg_length Control
    double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

    if(window[0].state == DSP_INIT_RF || window[0].state == DSP_INIT_LF)
    {


    }
    else if(window[0].state == DSP_FINAL)
    {

            Add_Leg_Recovery[RIGHT][Zdir] = RecoverRightLegLength(0.,(GLOBAL_Z_RF + Add_Leg_Recovery[RIGHT][Zdir])- Init_Right_Leg , 1);
            Add_Leg_Recovery[LEFT][Zdir]  = RecoverLeftLegLength(0.,(GLOBAL_Z_LF + Add_Leg_Recovery[LEFT][Zdir])- Init_Left_Leg , 1);

            // Recovery Leg initial position


    }else if(window[0].state == STATE_EMPTY && window[0].timer.current < 2.0)
    {
        Add_Leg_Recovery[RIGHT][Zdir] = RecoverRightLegLength(0.,(GLOBAL_Z_RF + Add_Leg_Recovery[RIGHT][Zdir])- Init_Right_Leg , 1);
        Add_Leg_Recovery[LEFT][Zdir]  = RecoverLeftLegLength(0.,(GLOBAL_Z_LF + Add_Leg_Recovery[LEFT][Zdir])- Init_Left_Leg , 1);
    }

    if(Add_Leg_Recovery[RIGHT][Zdir] > 0.05)Add_Leg_Recovery[RIGHT][Zdir] =  0.05;
    if(Add_Leg_Recovery[RIGHT][Zdir] <-0.05)Add_Leg_Recovery[RIGHT][Zdir] = -0.05;

    if(Add_Leg_Recovery[LEFT][Zdir] > 0.05)Add_Leg_Recovery[LEFT][Zdir] =  0.05;
    if(Add_Leg_Recovery[LEFT][Zdir] <-0.05)Add_Leg_Recovery[LEFT][Zdir] = -0.05;
}


double RFootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{
    static double CL=0.0;
    static double d1 = 10000,d2 = 15000, m = 2.5,dt = 0.005,T = 0.2;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};


    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(pv_Index >=2)
    {
        Q[0] = (dRforce  - mRforce )/d1 - (1.0/T)*Q[1];

        CL = Q[2] = Q[1] + Q[0]*dt;

        Q[1] = Q[2];

        if(CL > 0.05)
        {
            CL = 0.05;
        }else if( CL < -0.05)
        {
            CL = -0.05;
        }
    }


    return CL;
}
double LFootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{

    static double CL=0.0;
    static double d1 = 10000,d2 = 15000, m = 2.5,dt = 0.005,T = 0.2;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};

    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(pv_Index >=2)
    {
        Q[0] = (dLforce  - mLforce )/d1 - (1.0/T)*Q[1];

        CL = Q[2] = Q[1] + Q[0]*dt;

        Q[1] = Q[2];

        if(CL > 0.05)
        {
            CL = 0.05;
        }else if( CL < -0.05)
        {
            CL = -0.05;
        }
    }


    return CL;
}
double FootForceControl2(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0)
{
    static double CL=0.0;
    static double d1 = 150,d2 = 28000, m = 2.5,dt = 0.005,T = 0.1;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};



    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    {

        if(pv_Index >=2)
        {
            double gain = 1.;//+ sqrt(sharedData->FOG.Pitch*sharedData->FOG.Pitch)*0.01;

            Q[0] = ( -HPF_Estimated_Orientation[0]*D2R*1.5 - sharedData->FOG.RollVel*1.28 )/d1 - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.05)
            {
                CL = 0.05;
            }else if( CL < -0.05)
            {
                CL = -0.05;
            }

        }

    }

    return CL;
}

double RMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (-alpha*M*g);
    LDF = (-(1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;


    if(state == 1)
    {
        d = 50.;//1125;
        T = 0.4;
    }else if(state == 0)
    {
        d = 50.;
        T = 0.05;
    }else if(state == 3)
    {
        d = 2500;
        T = 0.5;
    }


    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( 0 - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }


        return CL*R2D;
}
double LMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (-alpha*M*g);
    LDF = (-(1.0 - alpha)*M*g);

    //desired
dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;



    if(state2 == 1)
    {

        d = 50.0;
        T = 0.4;
    }else if(state2 == 0)
    {
        d = 50;
        T = 0.05;
    }else if(state2 == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( 0 - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }
        return CL*R2D;
}
double RMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    {
        mTorque = FTmx;
    }

    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 50.;
        T = 0.4;

    }else if(state == 0)
    {
        d = 50.;
        T = 0.05;
    }else if(state == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }


        return CL*R2D;



}
double LMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired

        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    {
        mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);
    }

    static double CL=0.0;
    static double d = 50, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
//        d = 10;
//        T = 0.2;
        d = 50.0;
        T = 0.4;
    }else if(state2 == 0)
    {
        d = 50;
        T = 0.05;
    }else if(state2 == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }

        return CL*R2D;



}



/****************************** 8. Initialize *******************************/
void First_Initialize()
{

    Controller_initialize();

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = -5.*D2R;
    WBIK_Q0[idLSP] = -5.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;

    WBIK_PARA_CHANGE();

    cout << "IK Version: " << kine_drc_hubo4.get_version() << endl;

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.83;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo4.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo4.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    get_WBIK_Q_from_RefAngleCurrent();

    printf("First Init Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
    printf("First Init Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

    printf("COM : %f  %f  %f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
    printf("qPel : %f  %f  %f  %f \n",des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);
    printf("pRF : %f  %f  %f \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
    printf("qRF : %f  %f  %f  %f \n",des_qRF_4x1[0],des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3]);
    printf("pLF : %f  %f  %f \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
    printf("qLF : %f  %f  %f  %f \n",des_qLF_4x1[0],des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3]);

    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    printf("First Init Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
    printf("First Init Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);


    get_WBIK_Q_from_RefAngleCurrent();

    //----------------------------------- Reset COM to 0
    WBIK_Q[idQ0] = 1;
    WBIK_Q[idQ1] = 0;
    WBIK_Q[idQ2] = 0;
    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("First Initialize FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    // F.K

    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

    printf("222Fisrt Iniitalize  FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    printf("FK right pos  %f, %f, %f \n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2]);
    printf("FK right ori  %f, %f, %f \n",FK_qRFoot_4x1[0],FK_qRFoot_4x1[1],FK_qRFoot_4x1[2],FK_qRFoot_4x1[3]);

    printf("FK left  pos  %f, %f, %f \n",FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);
    printf("FK left  ori  %f, %f, %f \n",FK_qLFoot_4x1[0],FK_qLFoot_4x1[1],FK_qLFoot_4x1[2],FK_qLFoot_4x1[3]);

}

void WBIK_PARA_CHANGE(){
    // Tuned value with the head
    kine_drc_hubo4.C_Torso[0] = 0.000941-0.045;
    // weight of Torso is increased ( Torso + Head)
    kine_drc_hubo4.m_Torso = 24.98723 + 5.0; // kg
    kine_drc_hubo4.m_RightWrist = 4.5;
    kine_drc_hubo4.m_LeftWrist = 4.5;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.L_FOOT = 0.113;
    kine_drc_hubo4.iter_limit = 100;
    kine_drc_hubo4.converge_criterium = 1e-6;

}

void add_boxmass(double _mass){
    // Tuned value with the head
    kine_drc_hubo4.C_Torso[0] = 0.000941-0.045;
    // weight of Torso is increased ( Torso + Head)
    kine_drc_hubo4.m_Torso = 24.98723 + 5.0; // kg
    kine_drc_hubo4.m_RightWrist = 4.5 + _mass;
    kine_drc_hubo4.m_LeftWrist = 4.5 + _mass;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.L_FOOT = 0.113;
    kine_drc_hubo4.iter_limit = 100;
    kine_drc_hubo4.converge_criterium = 1e-6;

}
void Walking_initialize()
{

    printf("##################### Walking Initialize!!!!!!!!!!!!!!!!!!! \n");
    FINAL_TIMER = 0.;
    Controller_initialize();

    get_WBIK_Q_from_RefAngleCurrent();

    double PelYaw,PelRoll,PelPitch,temp1_qPel_4x1[4],temp2_qPel_4x1[4],temp3_qPel_4x1[4],temp4_qPel_4x1[4],temp5_qPel_4x1[4];
    PelYaw = 0*D2R;
    PelRoll = 0*D2R;
    PelPitch = 0*D2R;

    qtRZ(PelYaw, temp1_qPel_4x1);
    qtRX(PelRoll, temp2_qPel_4x1);
    qtRY(PelPitch, temp3_qPel_4x1);

    QTcross(temp1_qPel_4x1,temp2_qPel_4x1,temp4_qPel_4x1);
    QTcross(temp4_qPel_4x1,temp3_qPel_4x1,temp5_qPel_4x1);

    WBIK_Q[idQ0] = temp5_qPel_4x1[0];//1;
    WBIK_Q[idQ1] = temp5_qPel_4x1[1];//0;
    WBIK_Q[idQ2] = temp5_qPel_4x1[2];//0;
    WBIK_Q[idQ3] = temp5_qPel_4x1[3];//0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    printf("FK3 com = %f,%f,%f,PEL = %f,%f,%f,RF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    printf("init_WBIK_pCOM : (%f,%f,%f),init_WBIK_Q : (%f,%f,%f)\n",init_WBIK_pCOM[0],init_WBIK_pCOM[1],init_WBIK_pCOM[2],init_WBIK_Q[0],init_WBIK_Q[1],init_WBIK_Q[2]);


    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];


    printf("========================\n");
    printf("PEL : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],WBIK_Q[idQ0],WBIK_Q[idQ1],WBIK_Q[idQ2],WBIK_Q[idQ3]);
    printf("RLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idRHY],WBIK_Q[idRHR],WBIK_Q[idRHP],WBIK_Q[idRKN],WBIK_Q[idRAP],WBIK_Q[idRAR]);
    printf("LLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idLHY],WBIK_Q[idLHR],WBIK_Q[idLHP],WBIK_Q[idLKN],WBIK_Q[idLAP],WBIK_Q[idLAR]);

    printf("RARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idRSP],WBIK_Q[idRSR],WBIK_Q[idRSY],WBIK_Q[idREB],WBIK_Q[idRWP],WBIK_Q[idRWY],WBIK_Q[idRWY2]);
    printf("LARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idLSP],WBIK_Q[idLSR],WBIK_Q[idLSY],WBIK_Q[idLEB],WBIK_Q[idLWP],WBIK_Q[idLWY],WBIK_Q[idLWY2]);
    printf("========================\n");


    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,PEL = %f,%f,%f,\nRF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);


    //Set Foot print Initial value


    QT2YPR(FK_qRFoot_4x1,FK_RFoot_yaw,FK_RFoot_pitch,FK_RFoot_roll);

    QT2YPR(FK_qLFoot_4x1,FK_LFoot_yaw,FK_LFoot_pitch,FK_LFoot_roll);


    Init_Right_Leg = GLOBAL_Z_RF = GLOBAL_Z_RF_last[0] = FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    Init_Left_Leg  = GLOBAL_Z_LF = GLOBAL_Z_LF_last[0] = FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];



        des_pRF_3x1[0] =FK_pRFoot_3x1[0];
        des_pRF_3x1[1] =FK_pRFoot_3x1[1];
        des_pRF_3x1[2] =FK_pRFoot_3x1[2];

        des_pLF_3x1[0] =FK_pLFoot_3x1[0];
        des_pLF_3x1[1] =FK_pLFoot_3x1[1];
        des_pLF_3x1[2] =FK_pLFoot_3x1[2];

        des_pCOM_3x1[0] = FK_pCOM_3x1[0];
        des_pCOM_3x1[1] = FK_pCOM_3x1[1];
        des_pCOM_3x1[2] = FK_pCOM_3x1[2];

    //controller reset
    kirkZMPCon_XP2(0,0,0);
    kirkZMPCon_YP2(0,0,0);
    Del_PC_X_DSP_XZMP_CON = 0;
    Del_PC_Y_DSP_YZMP_CON = 0;

    Del_PC_X_SSP_XZMP_CON = 0;
    Del_PC_Y_SSP_YZMP_CON = 0;

    I_ZMP_CON_X = 0.0f;
    I_ZMP_CON_Y = 0.0f;

}

void Controller_initialize()
{
    des_pRF_3x1[0] =0;
    des_pRF_3x1[1] =0;
    des_pRF_3x1[2] =0;

    des_pLF_3x1[0] =0;
    des_pLF_3x1[1] =0;
    des_pLF_3x1[2] =0;


    X_ZMP = 0;
    Y_ZMP = 0;
    X_ZMP_Local = 0;
    Y_ZMP_Local = 0;
    X_ZMP_Global = 0;
    Y_ZMP_Global = 0;
    X_ZMP_n = 0;
    Y_ZMP_n = 0;
    X_ZMP_LPF = 0;
    Y_ZMP_LPF = 0;
    CNT_final_gain_DSP_ZMP_CON = 0;
    CNT_final_gain_SSP_ZMP_CON = 0;


    // Gyro feedback;
    y_i_1 = 0;
    y_i_11= 0;
    u_i_1 = 0;
    u_i_11 = 0;
    NotchFilter_GyroRollControlInput(0,0);
    NotchFilter_GyroPitchControlInput(0,0);
    NotchFilter_GyroRollVel(0,0);
    NotchFilter_GyroPitchVel(0,0);
    GLOBAL_Xori_RF_last = 0;
    GLOBAL_Xori_LF_last = 0;
    GLOBAL_Yori_RF_last = 0;
    GLOBAL_Yori_LF_last = 0;

    GLOBAL_Xori_RF2_last = 0;
    GLOBAL_Xori_LF2_last = 0;
    GLOBAL_Yori_RF2_last = 0;

    GLOBAL_Yori_LF2_last = 0;

    U_Gain = 0.;
    GLOBAL_Xori_RF = 0.;
    GLOBAL_Xori_LF = 0.;
    GLOBAL_Yori_RF = 0.;
    GLOBAL_Yori_LF = 0.;






    // ankle torque control;
    RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDPitch =0.;
    LDPitch =0.;
    RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDRoll =0.;
    LDRoll =0.;

    RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDPitch2 =0.;
    LDPitch2 =0.;


    RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDRoll2 =0.;
    LDRoll2 =0.;


}

void Walking_initialize_1st()
{

    get_WBIK_Q_from_RefAngleCurrent();

    WBIK_Q[idQ0] = 1;
    WBIK_Q[idQ1] = 0;
    WBIK_Q[idQ2] = 0;
    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    init_WBIK_pCOM[0] = FK_pCOM_3x1[0];
    init_WBIK_pCOM[1] = FK_pCOM_3x1[1];
    init_WBIK_pCOM[2] = FK_pCOM_3x1[2];

    init_WBIK_Q[0] = WBIK_Q[idX];
    init_WBIK_Q[1] = WBIK_Q[idY];
    init_WBIK_Q[2] = WBIK_Q[idZ];

    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);


    printf("FK_pRFoot_3x1[0] = %f, FK_pRFoot_3x1[1] = %f, FK_pRFoot_3x1[2] = %f,FK_pLFoot_3x1[0] = %f, FK_pLFoot_3x1[1] = %f, FK_pLFoot_3x1[2] = %f\n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    GLOBAL_Z_RF = 0;//FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    GLOBAL_Z_LF = 0;//FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];
}


/****************************** 9. Functions ********************************/
/* GainOverride */
void Upperbody_Gain_Lock()
{

    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);

            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,500); //--LSP
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,500); //--LSR
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,500); //--LSY
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,500); //--LEB

            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,500); //--RSP
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,500); //--RSR
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,500); //--RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,500); //--REB


}
void Upperbody_Gain_Override()
{
    ZeroGainLeftArm();
    ZeroGainRightArm();
}
int ZeroGainLeftArm(){

    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 40, 10);
    usleep(5000);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}
int ZeroGainRightArm(){

    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 40, 10);
    usleep(5000);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}

int ZeroGainLeftArm1(){
    //-- lsp
    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1500, 130, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 100, 10);
    usleep(5000);

    //-- lsr
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 100, 10);
    usleep(5000);

    //--- lsy, leb
    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 800,80, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 100, 10);
    usleep(5000);

    //--- lwy, lwp
    MCsetFrictionParameter(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 1500, 230, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 1500, 200, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 100, 10);
    usleep(5000);

    //---- lwy2
    MCsetFrictionParameter(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 1100, 180, 0);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 100, 10);
    usleep(5000);


    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);
    usleep(5000);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}

int ZeroGainRightArm1(){
    //-- Rsp
    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1500, 100, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 100, 10);
    //usleep(5000);

    //-- Rsr
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 2000, 150, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 100, 10);
    //usleep(5000);

    //--- Rsy, Reb
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 2000, 150, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 100, 10);
    //usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 800, 70, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 100, 10);
    //usleep(5000);

    //--- Rwy, Rwp
    MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 1500, 180, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 100, 10);
    //usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 1500,230, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 100, 10);
    //usleep(5000);

    //---- Rwy2
    MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 1250, 170, 0);
    //usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, SW_MODE_NON_COMPLEMENTARY);
    //usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);
    //usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 100, 10);
    //usleep(5000);

    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
    //usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
    //usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
    //usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
    //usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);
    //usleep(5000);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
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



int ZeroGainRightArm1Test(){

    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 1500,230, 0);

    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);

    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 100, 10);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}

/* FeedForward Motion Control */
void Compensator_deflection(int state)
{
    static double p0[3] = {0.,0.,0.},pf[3] = {0.0,0.,0.},ref[3]={0.,};
    double comp_angle = 1.0; // deg
    double sagging_time = 0.2;
    if(state == SSP_RF)
    {

        if(window[0].timer.current<sagging_time)
        {
            p0[0] = 0.;
            pf[0] = comp_angle;
            Fifth(window[0].timer.current,sagging_time,p0,pf,ref);
            deflection_comp_LAR = ref[0];

        }else if(window[0].timer.current > sagging_time + 0.4 && window[0].timer.current < sagging_time + 0.6)
        {
            p0[0] = comp_angle;
            pf[0] = 0.;
            Fifth(window[0].timer.current-sagging_time - 0.4,sagging_time,p0,pf,ref);
            deflection_comp_LAR = ref[0];
        }

    }else if(state == SSP_LF)
    {

        if(window[0].timer.current<sagging_time)
        {
            p0[0] = 0.;
            pf[0] = -comp_angle;
            Fifth(window[0].timer.current,sagging_time,p0,pf,ref);
            deflection_comp_RAR = ref[0];

        }else if(window[0].timer.current > sagging_time + 0.4 && window[0].timer.current < sagging_time + 0.6)
        {
            p0[0] = -comp_angle;
            pf[0] = 0.;
            Fifth(window[0].timer.current - sagging_time - 0.4,sagging_time,p0,pf,ref);
            deflection_comp_RAR = ref[0];
        }
    }
}

void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d)
{
    if(pv_Index == 1){
        JW_InvPattern_U[0]=Pattern1;
        JW_InvPattern_U[1]=Pattern2;

        JW_InvPattern_U_I[0] = 0.;
        JW_InvPattern_U_I[1] = 0.;

        JW_InvPattern_Y_old[0] = Pattern1;
        JW_InvPattern_Y_old[1] = Pattern1_d;

        JW_InvPattern_X_old[0] = Pattern2;
        JW_InvPattern_X_old[1] = Pattern2_d;
        printf("Invmodel reset!!!\n");
    }
        JW_InvPattern_Klqr[0]=30.6386;//20.6386f;//2.3166;// 30.6386;//315.2293f;
        JW_InvPattern_Klqr[1]=0.7508;//0.7508f;//0.1868;//0.7508;//2.4780f;

        JW_InvPattern_U_I[0] +=0.1*(Pattern1-JW_InvPattern_Y_old[0]);
        JW_InvPattern_U[0]   = JW_InvPattern_Klqr[0]*(Pattern1-JW_InvPattern_Y_old[0])+JW_InvPattern_Klqr[1]*(Pattern1_d-JW_InvPattern_Y_old[1])+JW_InvPattern_U_I[0];

        JW_InvPattern_U_I[1] +=0.1*(Pattern2-JW_InvPattern_X_old[0]);
        JW_InvPattern_U[1]   = JW_InvPattern_Klqr[0]*(Pattern2-JW_InvPattern_X_old[0])+JW_InvPattern_Klqr[1]*(Pattern2_d-JW_InvPattern_X_old[1])+JW_InvPattern_U_I[1];


        JW_InvPattern_A[0][0] =0.f;
        JW_InvPattern_A[0][1] =1.0f;
        JW_InvPattern_A[1][0] =-JW_InvPattern_k/JW_InvPattern_m;
        JW_InvPattern_A[1][1] =-JW_InvPattern_c/JW_InvPattern_m;

        JW_InvPattern_A_X[0][0] =0.f;
        JW_InvPattern_A_X[0][1] =1.0f;
        JW_InvPattern_A_X[1][0] =-JW_InvPattern_k_X/JW_InvPattern_m;
        JW_InvPattern_A_X[1][1] =-JW_InvPattern_c_X/JW_InvPattern_m;

        JW_InvPattern_l = sqrt((userData->WalkReadyCOM[2])*(userData->WalkReadyCOM[2]) + Pattern1*Pattern1);

        if(pv_Index == 1){

            Y_inv = Pattern1;
            Y_inv_d = Pattern1_d;
            theta = atan2(Pattern1,(userData->WalkReadyCOM[2]));
            theta_d = 0;
            U_I[0] = 0.0f;
            Y_inv_old = Pattern1;
        }
        U_I[0] +=.1*(Pattern1-Y_inv);
        U[0]   = JW_InvPattern_Klqr[0]*(Pattern1 - Y_inv) + JW_InvPattern_Klqr[1]*(Pattern1_d - Y_inv_d) + U_I[0];

        theta_ref = atan2(U[0],(userData->WalkReadyCOM[2]));
        if(pv_Index == 1){
            theta_ref =-(((9.81/JW_InvPattern_l*sin(theta))/(1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l))-JW_InvPattern_c*(theta_d))/JW_InvPattern_k - theta) ;//*(*(theta-theta_ref)+);

        }
        theta_dd = 9.81/JW_InvPattern_l*sin(theta)-1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l)*(JW_InvPattern_k*(theta-theta_ref)+JW_InvPattern_c*(theta_d));
        theta_d = theta_d + theta_dd*DEL_T;
        theta   = theta + theta_d*DEL_T;

        Y_inv_old = Y_inv;
        Y_inv=(userData->WalkReadyCOM[2])*tan(theta);
        Y_inv_d = (Y_inv - Y_inv_old)/DEL_T;
        JW_InvPattern_B[0] = 0.f;
        JW_InvPattern_B[1] = JW_InvPattern_k/JW_InvPattern_m;

        JW_InvPattern_B_X[0] = 0.f;
        JW_InvPattern_B_X[1] = JW_InvPattern_k_X/JW_InvPattern_m;

        JW_InvPattern_Y_d[0] = JW_InvPattern_A[0][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[0][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[0]*JW_InvPattern_U[0];
        JW_InvPattern_Y_d[1] = JW_InvPattern_A[1][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[1][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[1]*JW_InvPattern_U[0];

        JW_InvPattern_X_d[0] = JW_InvPattern_A_X[0][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[0][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[0]*JW_InvPattern_U[1];
        JW_InvPattern_X_d[1] = JW_InvPattern_A_X[1][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[1][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[1]*JW_InvPattern_U[1];

        JW_InvPattern_Y[0] = JW_InvPattern_Y_old[0] + JW_InvPattern_Y_d[0]*DEL_T;
        JW_InvPattern_Y[1] = JW_InvPattern_Y_old[1] + JW_InvPattern_Y_d[1]*DEL_T;

        JW_InvPattern_X[0] = JW_InvPattern_X_old[0] + JW_InvPattern_X_d[0]*DEL_T;
        JW_InvPattern_X[1] = JW_InvPattern_X_old[1] + JW_InvPattern_X_d[1]*DEL_T;

        JW_InvPattern_Y_old[0] = JW_InvPattern_Y[0];
        JW_InvPattern_Y_old[1] = JW_InvPattern_Y[1];

        JW_InvPattern_X_old[0] = JW_InvPattern_X[0];
        JW_InvPattern_X_old[1] = JW_InvPattern_X[1];

        double Global[3],Local[3];
        Global[0] = JW_InvPattern_U[1];
        Global[1] = 0;
        Global[2] = 0;
        Global2Local2(Global,Local);
        JW_InvPattern_U_n[1] = Local[0];
}

void get_WBIK_Q_from_RefAngleCurrent()
{
    jCon->RefreshToCurrentReference();
    for(int i=RHY; i<=LAR; i++) {
       WBIK_Q[i+7] = jCon->Joints[i]->RefAngleCurrent*D2R;

    }
    WBIK_Q[idWST] = jCon->GetJointRefAngle(WST)*D2R;
    WBIK_Q[idRSP] = jCon->GetJointRefAngle(RSP)*D2R;
    WBIK_Q[idRSR] = (jCon->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    WBIK_Q[idRSY] = jCon->GetJointRefAngle(RSY)*D2R;
    WBIK_Q[idREB] = (jCon->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    WBIK_Q[idRWY] = jCon->GetJointRefAngle(RWY)*D2R;
    WBIK_Q[idRWP] = jCon->GetJointRefAngle(RWP)*D2R;
    WBIK_Q[idRWY2] = jCon->GetJointRefAngle(RWY2)*D2R;

    WBIK_Q[idLSP] = jCon->GetJointRefAngle(LSP)*D2R;
    WBIK_Q[idLSR] = (jCon->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    WBIK_Q[idLSY] = jCon->GetJointRefAngle(LSY)*D2R;
    WBIK_Q[idLEB] = (jCon->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    WBIK_Q[idLWY] = jCon->GetJointRefAngle(LWY)*D2R;
    WBIK_Q[idLWP] = jCon->GetJointRefAngle(LWP)*D2R;
    WBIK_Q[idLWY2] = jCon->GetJointRefAngle(LWY2)*D2R;

    Qub[idRSR] = WBIK_Q[idRSR];
    Qub[idRSP] = WBIK_Q[idRSP];
    Qub[idRSY] = WBIK_Q[idRSY];
    Qub[idREB] = WBIK_Q[idREB];
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWP] = WBIK_Q[idRWP];
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWY2] = WBIK_Q[idRWY2];

    Qub[idLSR] = WBIK_Q[idLSR];
    Qub[idLSP] = WBIK_Q[idLSP];
    Qub[idLSY] = WBIK_Q[idLSY];
    Qub[idLEB] = WBIK_Q[idLEB];
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWP] = WBIK_Q[idLWP];
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWY2] = WBIK_Q[idLWY2];

    Qub[idWST] = WBIK_Q[idWST];

}

double BasicTrajectory(double count, int mode)
{
    if(count > 1.) count = 1.;
    else if(count < 0.) count = 0.;

    if(mode == MODE_ONEtoZERO) return 1./2.*(cos(count*PI) + 1.);
    else return 1./2.*(1. - cos(count*PI));
}


//double pCenter[3],qCenter[4],qCenter_bar[4];
double temp1[3];
void Global2Local(double _Global[],double _Local[])
{


    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

    qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] =  qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Global2Local2(double _Global[],double _Local[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] = qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Local2Global(double _Local[],double _Global[])
{
    double pCenter[3],qCenter[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

    qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    QTtransform(qCenter, _Local, temp1);

    sum_vv(temp1,3,pCenter,_Global); // zmp - pCenter
}

/* QP Formulation */
void PreComputeQP()
{
    // MPC Precomputing for Fast calculation

      MPC_A[0][0] = 1.0;      MPC_A[0][1] = MPC_T;    MPC_A[0][2] = MPC_T*MPC_T/2.0;
      MPC_A[1][0] = 0.0;      MPC_A[1][1] = 1.0;      MPC_A[1][2] = MPC_T;
      MPC_A[2][0] = 0.0;      MPC_A[2][1] = 0.0;      MPC_A[2][2] = 1.0;

      MPC_B[0] = MPC_T*MPC_T*MPC_T/6.0;  MPC_B[1] = MPC_T*MPC_T/2.0;  MPC_B[2] = MPC_T;

      MPC_C[0] = 1.0;  MPC_C[1] = 0.;  MPC_C[2] = -MPC_h/MPC_g;

      // Precomputing
      for(int i=0;i<MPC_time;i++)
      {
          Pps[i][0] = 1.0;    Pps[i][1] = MPC_T*((double)(i+1));    Pps[i][2] = MPC_T*MPC_T*((double)(i+1))*((double)(i+1))/2.0;
          Pvs[i][0] = 0.0;    Pvs[i][1] = 1.0;    Pvs[i][2] = MPC_T*((double)(i+1));
          Pzs[i][0] = 1.0;    Pzs[i][1] = MPC_T*((double)(i+1));    Pzs[i][2] = MPC_T*MPC_T*((double)(i+1))*((double)(i+1))/2.0-MPC_h/MPC_g;

          for(int j = 0;j<i+1;j++)
          {
              Ppu[i][j] = (1.0 + 3.0*(((double)(i+1))-((double)(j+1))) + 3.0*(((double)(i+1))-((double)(j+1)))*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T*MPC_T/6.0;


              Pvu[i][j] = (1.0 + 2.0*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T/2.0;

              Pzu[i][j] = (1.0 + 3.0*(((double)(i+1))-((double)(j+1))) + 3.0*(((double)(i+1))-((double)(j+1)))*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T*MPC_T/6.0 - MPC_T*MPC_h/MPC_g;

          }
      }

      printf("==================================================================\n");
      printf("--Pps : %f    %f  %f \n",Pps[0][0],Pps[0][1],Pps[0][2]);
      printf("--Pps : %f    %f  %f \n",Pps[1][0],Pps[1][1],Pps[1][2]);
      printf("--Pps : %f    %f  %f \n",Pps[14][0],Pps[14][1],Pps[14][2]);
      printf("==================================================================\n");

      printf("--Pvs : %f    %f  %f \n",Pvs[0][0],Pvs[0][1],Pvs[0][2]);
      printf("--Pvs : %f    %f  %f \n",Pvs[1][0],Pvs[1][1],Pvs[1][2]);
      printf("--Pvs : %f    %f  %f \n",Pvs[14][0],Pvs[14][1],Pvs[14][2]);
      printf("==================================================================\n");
      printf("--Pzs : %f    %f  %f \n",Pzs[0][0],Pzs[0][1],Pzs[0][2]);
      printf("--Pzs : %f    %f  %f \n",Pzs[1][0],Pzs[1][1],Pzs[1][2]);
      printf("--Pzs : %f    %f  %f \n",Pzs[14][0],Pzs[14][1],Pzs[14][2]);

      InvMatrix(MPC_time,(double*)Pzu,(double*)Pzu_inv);

      for(int i=0; i<MPC_time;i++){
          for(int j=0;j<MPC_time;j++){
              Pzu_inv_trans[j][i] = Pzu_inv[i][j];
              Pvu_trans[j][i] = Pvu[i][j];
          }
      }


      mat15by15x15by15(Pzu_inv_trans,Pzu_inv,Pzu_invxtrans);

      mat15by15x15by15(Pzu_inv_trans,Pvu_trans,temp_mat);
      mat15by15x15by15(temp_mat,Pvu,temp_mat2);
      mat15by15x15by15(temp_mat2,Pzu_inv,temp_mat);

      for(int i = 0;i<MPC_time;i++){
          for(int j = 0;j<MPC_time;j++){
              if(i == j){
                  temp_mat2[i][j] = MPC_beta;
              }
          }
      }

      for(int i=0;i<MPC_time;i++){
          for(int j=0;j<MPC_time;j++){
              MPC_Q[i][j] = MPC_gamma*Pzu_invxtrans[i][j] + MPC_alpha*temp_mat[i][j] + temp_mat2[i][j];
          }
      }


      for(int i=MPC_time;i<MPC_time + MPC_m;i++){
          for(int j=MPC_time;j<MPC_time+ MPC_m;j++){
              if(i == j){
                  MPC_Q[i][j] = MPC_mu*1.0;
              }
          }
      }



      // MPC_pk

      mat15by15x15by15(Pvu,Pzu_inv,temp_mat);
      mat15by15x15by3(temp_mat,Pzs,t_mat);
      mat15by3minus15by3(Pvs,t_mat,t_mat2);

      mat15by15x15by15(Pzu_inv_trans,Pvu_trans,temp_mat2);
      mat15by15x15by3(temp_mat2,t_mat2,t_mat3);



      mat15by15x15by15(Pzu_inv_trans,Pzu_inv,temp_mat);
      mat15by15x15by3(temp_mat,Pzs,t_mat);



      for(int i=0;i<MPC_time;i++)
      {
          for(int j=0;j<3;j++)
          {
              t_mat3[i][j] = t_mat3[i][j]*MPC_alpha;
              t_mat[i][j] = t_mat[i][j]*MPC_gamma;
          }
      }

      mat15by3minus15by3(t_mat3,t_mat,MPC_pc);


      printf("==================================================================\n");
      printf("--pc : %f       %f      %f \n",MPC_pc[0][0],MPC_pc[0][1],MPC_pc[0][2]);
      printf("--Pc : %f       %f      %f \n",MPC_pc[1][0],MPC_pc[1][1],MPC_pc[1][2]);
      printf("--Pc : %f       %f      %f \n",MPC_pc[14][0],MPC_pc[14][1],MPC_pc[14][2]);



      printf("==================================================================\n");
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[0][0],MPC_Q[0][1],MPC_Q[0][2],MPC_Q[0][3],MPC_Q[0][4],MPC_Q[0][5],MPC_Q[0][6],MPC_Q[0][7],MPC_Q[0][8],MPC_Q[0][9],MPC_Q[0][10],MPC_Q[0][11]);
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[1][0],MPC_Q[1][1],MPC_Q[1][2],MPC_Q[1][3],MPC_Q[1][4],MPC_Q[1][5],MPC_Q[1][6],MPC_Q[1][7],MPC_Q[1][8],MPC_Q[1][9],MPC_Q[1][10],MPC_Q[1][11]);
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[14][0],MPC_Q[14][1],MPC_Q[14][2],MPC_Q[14][3],MPC_Q[14][4],MPC_Q[14][5],MPC_Q[14][6],MPC_Q[14][7],MPC_Q[14][8],MPC_Q[14][9],MPC_Q[14][10],MPC_Q[14][11]);


      //--Ci

      // MPC_C1
      for(int i = 0; i<MPC_time;i++){
          for(int j = 0; j<MPC_time;j++){
                  if(i == j){
                      MPC_Ci[i][j] = 1.;
                  }else{
                      MPC_Ci[i][j] = 0.;
                  }
          }
      }

      for(int i = MPC_time; i<MPC_time*2;i++){
          for(int j = 0; j<MPC_time;j++){
                  if(i-MPC_time == j){
                      MPC_Ci[i][j] = -1.;
                  }else{
                      MPC_Ci[i][j] = 0.;
                  }
          }
      }


      for(int oo=0;oo<30;oo++)
      {
          printf("%d--MPC_Ci : %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",oo,MPC_Ci[oo][0],MPC_Ci[oo][1],MPC_Ci[oo][2],MPC_Ci[oo][3],MPC_Ci[oo][4],MPC_Ci[oo][5],MPC_Ci[oo][6],MPC_Ci[oo][7],MPC_Ci[oo][8],MPC_Ci[oo][9],MPC_Ci[oo][10],MPC_Ci[oo][11],MPC_Ci[oo][12],MPC_Ci[oo][13],MPC_Ci[oo][14]);
      }

}

void save()
{
    if(Save_Index < ROW)
    {
            Save_Data[0][Save_Index] = window[0].zmp.x;
            Save_Data[1][Save_Index] = window[1].zmp.x;
            Save_Data[2][Save_Index] = window[299].zmp.x;
            Save_Data[3][Save_Index] = window[0].zmp.y;
            Save_Data[4][Save_Index] = window[1].zmp.y;
            Save_Data[5][Save_Index] = window[299].zmp.y;
            Save_Data[6][Save_Index] = window[0].state;
            Save_Data[7][Save_Index] = window[0].right_foot_ref.x;
            Save_Data[8][Save_Index] = window[0].right_foot_ref.y;
            Save_Data[9][Save_Index] = window[0].right_foot_ref.z;
            Save_Data[10][Save_Index] = window[0].left_foot_ref.x;
            Save_Data[11][Save_Index] = window[0].left_foot_ref.y;
            Save_Data[12][Save_Index] = window[0].left_foot_ref.z;

            Save_Data[13][Save_Index] = window[0].left_foot_ref.yaw;
            Save_Data[14][Save_Index] = window[0].right_foot_ref.yaw;

            Save_Data[15][Save_Index] = Pel_Yaw;

            Save_Data[16][Save_Index] = GLOBAL_X_LIPM;
            Save_Data[17][Save_Index] = GLOBAL_Y_LIPM;
            Save_Data[18][Save_Index] = des_pCOM_3x1[0];
            Save_Data[19][Save_Index] = des_pCOM_3x1[1];
            Save_Data[20][Save_Index] = ALPHA;
            Save_Data[21][Save_Index] = window[0].timer.current;
            Save_Data[22][Save_Index] = window[0].timer.total;

            Save_Data[23][Save_Index] = deflection_comp_LAR;
            Save_Data[24][Save_Index] = deflection_comp_RAR;

            Save_Data[25][Save_Index] = sharedData->FT[RAFT].Fz;
            Save_Data[26][Save_Index] = sharedData->FT[LAFT].Fz;

            Save_Data[27][Save_Index] = sharedData->FOG.Roll;
            Save_Data[28][Save_Index] = sharedData->FOG.RollVel;

            Save_Data[29][Save_Index] = sharedData->FOG.Pitch;
            Save_Data[30][Save_Index] = sharedData->FOG.PitchVel;

            Save_Data[31][Save_Index] = GLOBAL_Z_RF;
            Save_Data[32][Save_Index] = GLOBAL_Z_LF;

            Save_Data[32][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[33][Save_Index] = Del_PC_Y_DSP_YZMP_CON;

            Save_Data[34][Save_Index] = I_ZMP_CON_X;
            Save_Data[35][Save_Index] = I_ZMP_CON_Y;

            Save_Data[36][Save_Index] = GLOBAL_Xori_RF;
            Save_Data[37][Save_Index] = GLOBAL_Xori_LF;

            Save_Data[38][Save_Index] = GLOBAL_Yori_RF;
            Save_Data[39][Save_Index] = GLOBAL_Yori_LF;

            Save_Data[40][Save_Index] = U_Gain;
            Save_Data[41][Save_Index] = U_Gain_DSP;

            Save_Data[42][Save_Index] = des_pRF_3x1[2];
            Save_Data[43][Save_Index] = des_pLF_3x1[2];

            Save_Data[44][Save_Index] = GLOBAL_Z_RF;
            Save_Data[45][Save_Index] = GLOBAL_Z_LF;//Zctrl2;

            Save_Data[46][Save_Index] = Zctrl;
            Save_Data[47][Save_Index] = Zctrl2;

            Save_Data[48][Save_Index] = EarlyLandingFlag[RIGHT];
            Save_Data[49][Save_Index] = EarlyLandingFlag[LEFT];

            Save_Data[50][Save_Index] = X_ZMP_REF_Global;
            Save_Data[51][Save_Index] = Y_ZMP_REF_Global;
            Save_Data[52][Save_Index] = X_ZMP_Global;
            Save_Data[53][Save_Index] = Y_ZMP_Global;

            Save_Data[54][Save_Index] = GLOBAL_Xori_RF_n;
            Save_Data[55][Save_Index] = GLOBAL_Xori_LF_n;

            Save_Data[56][Save_Index] = AnkleControl1;
            Save_Data[57][Save_Index] = AnkleControl2;

            Save_Data[58][Save_Index] = Add_FootTask[RIGHT][Zdir];
            Save_Data[59][Save_Index] = AnkleControl2;

            Save_Data[60][Save_Index] = X_ZMP_REF_Local;
            Save_Data[61][Save_Index] = Y_ZMP_REF_Local;

            Save_Data[62][Save_Index] = X_ZMP_Local;
            Save_Data[63][Save_Index] = Y_ZMP_Local;


            Save_Data[64][Save_Index] = Estimated_Orientation[0]*R2D;
            Save_Data[65][Save_Index] = Estimated_Orientation[1]*R2D;//Y_ZMP_Local;


            Save_Data[66][Save_Index] = sharedData->IMU[0].AccX;
            Save_Data[67][Save_Index] = sharedData->IMU[0].AccY;//Estmated_Orientation[1]*D2R;//Y_ZMP_Local;

            Save_Data[68][Save_Index] = HPF_Estimated_Orientation[0]*R2D;
            Save_Data[69][Save_Index] = HPF_Estimated_Orientation[1]*R2D;

            Save_Data[70][Save_Index] = Comp_Orientation[0];
            Save_Data[71][Save_Index] = Comp_Orientation[1];


            Save_Data[72][Save_Index] = sharedData->IMU[CIMU].Pitch_Comp;//U[0];
            Save_Data[73][Save_Index] = sharedData->IMU[CIMU].PitchVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[74][Save_Index] = sharedData->IMU[CIMU].Roll_Comp;//U[0];
            Save_Data[75][Save_Index] = sharedData->IMU[CIMU].RollVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];

            Save_Data[76][Save_Index] = Add_Leg_Recovery[RIGHT][Zdir];
            Save_Data[77][Save_Index] = Add_Leg_Recovery[LEFT][Zdir];//Comp_Orientation[1];

            Save_Data[78][Save_Index] = temp_debug[0];
            Save_Data[79][Save_Index] = temp_debug[1];


            Save_Data[80][Save_Index] = temp_debug[5];
            Save_Data[81][Save_Index] = temp_debug[6];


            Save_Data[82][Save_Index] = GLOBAL_X_LIPM_d;
            Save_Data[83][Save_Index] = GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[84][Save_Index] = CONT_X;
            Save_Data[85][Save_Index] = CONT_Y;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[86][Save_Index] = U[0];
            Save_Data[87][Save_Index] = U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];

            Save_Data[88][Save_Index] = target_foot[0].footprint.lfoot[0];
            Save_Data[89][Save_Index] = target_foot[0].footprint.lfoot[1];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];
            Save_Data[90][Save_Index] = target_foot[0].footprint.lori[0];//U[0];

            Save_Data[91][Save_Index] = target_foot[0].footprint.rfoot[0];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];
            Save_Data[92][Save_Index] = target_foot[0].footprint.rfoot[1];//U[0];
            Save_Data[93][Save_Index] = target_foot[0].footprint.rori[0];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[94][Save_Index] = sharedData->IMU[CIMU].Pitch;//U[0];
            Save_Data[95][Save_Index] = sharedData->IMU[CIMU].PitchVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[96][Save_Index] = sharedData->IMU[CIMU].Roll;//U[0];
            Save_Data[97][Save_Index] = sharedData->IMU[CIMU].RollVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[98][Save_Index] = des_pRF_3x1[0];
            Save_Data[99][Save_Index] = des_pRF_3x1[1];

            Save_Data[100][Save_Index] = des_pLF_3x1[0];
            Save_Data[101][Save_Index] = des_pLF_3x1[1];

            Save_Data[102][Save_Index] = RDPitch_atc;
            Save_Data[103][Save_Index] = LDPitch_atc;

            Save_Data[104][Save_Index] = WBIK_Q[idRHP];

            Save_Data[105][Save_Index] = X_ZMP_Local;
            Save_Data[106][Save_Index] = Y_ZMP_Local;
            Save_Data[107][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[108][Save_Index] = Del_PC_Y_DSP_YZMP_CON;
            Save_Data[109][Save_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.);
            Save_Data[110][Save_Index] = (- 0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.);
            Save_Data[111][Save_Index] = Save_Index;

            Save_Data[112][Save_Index] = F_RF_Global[1];
            Save_Data[113][Save_Index] = F_LF_Global[1];
            Save_Data[114][Save_Index] = M_RF_Global[1];
            Save_Data[115][Save_Index] = M_LF_Global[1];
            Save_Data[116][Save_Index] = pCenter[0];
            Save_Data[117][Save_Index] = pCenter[1];


            Save_Data[118][Save_Index] = RDRoll_atc;
            Save_Data[119][Save_Index] = LDRoll_atc;

            Save_Data[120][Save_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.);
            Save_Data[121][Save_Index] = (- 0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.);

            Save_Data[122][Save_Index] = GLOBAL_X_LIPM_n;

            Save_Data[123][Save_Index] = G_DSP_X;

            Save_Data[124][Save_Index] = OnOff_compliance;
            Save_Data[125][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[126][Save_Index] = I_ZMP_CON_X;
            Save_Data[127][Save_Index] = GLOBAL_Y_LIPM_n;
            Save_Data[128][Save_Index] = U0_Gain;
            Save_Data[129][Save_Index] = Del_PC_Y_DSP_YZMP_CON;
            Save_Data[130][Save_Index] = G_DSP_Y;

            Save_Data[131][Save_Index] = sharedData->FT[0].Fz;
            Save_Data[132][Save_Index] = sharedData->FT[1].Fz;

            Save_Data[133][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentReference;
            Save_Data[134][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentReference;
            Save_Data[135][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentReference;
            Save_Data[136][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentReference;

            Save_Data[137][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
            Save_Data[138][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentPosition;
            Save_Data[139][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition;
            Save_Data[140][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition;

            Save_Data[141][Save_Index] = ZMP_FeedBack_ONOFF;

            Save_Data[142][Save_Index] = GLOBAL_X_RF;
            Save_Data[143][Save_Index] = GLOBAL_Y_RF;

            Save_Data[144][Save_Index] = pCenter[0];
            Save_Data[145][Save_Index] = pCenter[1];
            Save_Data[146][Save_Index] = pCenter[2];

            Save_Data[147][Save_Index] = qCenter[0];
            Save_Data[148][Save_Index] = qCenter[1];
            Save_Data[149][Save_Index] = qCenter[2];
            Save_Data[150][Save_Index] = qCenter[3];

            Save_Data[151][Save_Index] = temp1[0];
            Save_Data[152][Save_Index] = temp1[1];
            Save_Data[153][Save_Index] = temp1[2];


            Save_Data[154][Save_Index] = FWRefAngleCurrent[RAP];
            Save_Data[155][Save_Index] = FWRefAngleCurrent[RAR];
            Save_Data[156][Save_Index] = FWRefAngleCurrent[LAP];
            Save_Data[157][Save_Index] = FWRefAngleCurrent[LAR];

            Save_Data[158][Save_Index] = WBIK_Q[RAP+7];
            Save_Data[159][Save_Index] = WBIK_Q[RAR+7];
            Save_Data[160][Save_Index] = WBIK_Q[LAP+7];
            Save_Data[161][Save_Index] = WBIK_Q[LAR+7];

            Save_Data[162][Save_Index] = GLOBAL_Xori_RF;
            Save_Data[163][Save_Index] = GLOBAL_Yori_RF;

            Save_Data[164][Save_Index] = deflection_comp_RAR;
            Save_Data[165][Save_Index] = GLOBAL_CPC_ZMP_X_REF;
            Save_Data[166][Save_Index] = GLOBAL_CPC_ZMP_Y_REF;

            Save_Data[167][Save_Index] = GLOBAL_Xori_LF;
            Save_Data[168][Save_Index] = GLOBAL_Yori_LF;

            Save_Data[169][Save_Index] = deflection_comp_LAR;
            Save_Data[170][Save_Index] = LDRoll;
            Save_Data[171][Save_Index] = LDPitch;

            Save_Data[172][Save_Index] = des_pRF_3x1_hat[Xdir];
            Save_Data[173][Save_Index] = des_pRF_3x1_hat[Ydir];
            Save_Data[174][Save_Index] = des_pRF_3x1_hat[Zdir];

            Save_Data[175][Save_Index] = des_pLF_3x1_hat[Xdir];
            Save_Data[176][Save_Index] = des_pLF_3x1_hat[Ydir];
            Save_Data[177][Save_Index] = des_pLF_3x1_hat[Zdir];


            Save_Data[178][Save_Index] = sharedData->FT[RAFT].Mx;
            Save_Data[179][Save_Index] = sharedData->FT[RAFT].My;//des_pLF_3x1_hat[Ydir];
            Save_Data[180][Save_Index] = sharedData->FT[RAFT].Mz;


            Save_Data[181][Save_Index] = sharedData->FT[LAFT].Mx;
            Save_Data[182][Save_Index] = sharedData->FT[LAFT].My;
            Save_Data[183][Save_Index] = sharedData->FT[LAFT].Mz;

            Save_Data[184][Save_Index] = ratio;
            Save_Data[185][Save_Index] = des_left_roll_torq;//sharedData->FT[LAFT].My;
            Save_Data[186][Save_Index] = des_right_roll_torq;//sharedData->FT[LAFT].Mz;
            Save_Data[187][Save_Index] = des_left_pitch_torq;//sharedData->FT[LAFT].My;
            Save_Data[188][Save_Index] = des_right_pitch_torq;//sharedData->FT[LAFT].Mz;
            Save_Data[189][Save_Index] = GLOBAL_Pitch;//sharedData->FT[LAFT].My;
            Save_Data[190][Save_Index] = GLOBAL_Pitch_vel;//sharedData->FT[LAFT].Mz;
            Save_Data[191][Save_Index] = GLOBAL_Roll;//sharedData->FT[LAFT].My;
            Save_Data[192][Save_Index] = GLOBAL_Roll_vel;//sharedData->FT[LAFT].Mz;

            Save_Data[193][Save_Index] = GLOBAL_CPC_ZMP_X_REF;//sharedData->FT[LAFT].My;
            Save_Data[194][Save_Index] = GLOBAL_CPC_ZMP_Y_REF;//sharedData->FT[LAFT].Mz;

            Save_Data[195][Save_Index] = RF_LPF_Fz;//sharedData->FT[LAFT].My;
            Save_Data[196][Save_Index] = LF_LPF_Fz;//sharedData->FT[LAFT].Mz;

            Save_Data[197][Save_Index] = RF_LPF_Mx;//sharedData->FT[LAFT].My;
            Save_Data[198][Save_Index] = LF_LPF_Mx;//sharedData->FT[LAFT].Mz;

            Save_Data[199][Save_Index] = RF_LPF_My;//sharedData->FT[LAFT].My;
            Save_Data[200][Save_Index] = LF_LPF_My;//sharedData->FT[LAFT].Mz;

            Save_Data[201][Save_Index] = pnow;
            Save_Data[202][Save_Index] = vnow;
            Save_Data[203][Save_Index] = anow;

            Save_Data[204][Save_Index] = pi;
            Save_Data[205][Save_Index] = pf;

            Save_Data[206][Save_Index] = FLAG_TIME;


            Save_Data[208][Save_Index] = target_foot[1].footprint.rfoot[0];
            Save_Data[209][Save_Index] = target_foot[1].footprint.rfoot[1];
            Save_Data[210][Save_Index] = target_foot[1].footprint.lfoot[0];
            Save_Data[211][Save_Index] = target_foot[1].footprint.lfoot[1];
            Save_Data[212][Save_Index] = target_foot[2].footprint.rfoot[0];
            Save_Data[213][Save_Index] = target_foot[2].footprint.rfoot[1];
            Save_Data[214][Save_Index] = target_foot[2].footprint.lfoot[0];
            Save_Data[215][Save_Index] = target_foot[2].footprint.lfoot[1];


            Save_Data[216][Save_Index] = short_foot[0].footprint.rfoot[0];
            Save_Data[217][Save_Index] = short_foot[0].footprint.lfoot[0];
            Save_Data[218][Save_Index] = short_foot[1].footprint.rfoot[0];
            Save_Data[219][Save_Index] = short_foot[1].footprint.lfoot[0];
            Save_Data[220][Save_Index] = short_foot[2].footprint.rfoot[0];
            Save_Data[221][Save_Index] = short_foot[2].footprint.lfoot[0];
            Save_Data[222][Save_Index] = short_foot[3].footprint.rfoot[0];
            Save_Data[223][Save_Index] = short_foot[3].footprint.lfoot[0];

            Save_Data[224][Save_Index] = target_foot[0].movingleg;
            Save_Data[225][Save_Index] = target_foot[1].movingleg;
            Save_Data[226][Save_Index] = target_foot[2].movingleg;


            Save_Data[227][Save_Index] = CPX_DOT;
            Save_Data[228][Save_Index] = CPY_DOT;

            Save_Data[229][Save_Index] = Hip_torque;
            Save_Data[230][Save_Index] = Hip_torque_ref;



            Save_Index++;

            if(Save_Index >= ROW) Save_Index = 0;


    }
}
void Upperbody_Gain_Lock1()
{
    jCon->RefreshToCurrentEncoder();
    jCon->SetAllMotionOwner();

    usleep(2000);

    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);

    //--LSR
//        RBenableFrictionCompensation(3,18,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,18, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);

    //--LSY, LEB
//        RBenableFrictionCompensation(3,19,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);


    //---LWY, LWP
//        RBenableFrictionCompensation(3,20,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch,JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch,JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);

    //---LWY2
//        RBenableFrictionCompensation(3,37,DISABLE,DISABLE);
    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);


    //---GainOverride recover
//        RBJointGainOverride(3,17,50,50,2000); //--LSP
//        RBJointGainOverride(3,18,50,50,2000); //--LSR
//        RBJointGainOverride(3,19,50,50,2000); //--LSY, LEB
//        RBJointGainOverride(3,20,50,50,2000); //---LWY, LWP
//        RBJointGainOverride(3,37,50,50,2000); // LWY2
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 65,1000); //--LSP
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 65,1000); //--LSR
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 65,1000); //--LSY
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 50,1000); //--LEB
    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 65,1000); //---LWY
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 65,1000); //--LWP
    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 65,1000); //--LF1

    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);

    usleep(1000*1000);
//        RBJointGainOverride(3,17,1000,1000,1000); //--LSP
//        RBJointGainOverride(3,18,1000,1000,1000); //--LSR
//        RBJointGainOverride(3,19,1000,1000,1000); //--LSY, LEB
//        RBJointGainOverride(3,20,1000,1000,1000); //---LWY, LWP
//        RBJointGainOverride(3,37,1000,1000,1000); //---LWY2
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,1000); //--LSP
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,1000); //--LSR
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,1000); //--LSY
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,1000); //--LEB
    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0,1000); //---LWY
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0,1000); //--LWP
    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,1000); //--LF1
    //--RSP
//        RBenableFrictionCompensation(2,13,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,13, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);

    //--RSR
//        RBenableFrictionCompensation(2,14,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,14, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);

    //--RSY, REB
//        RBenableFrictionCompensation(2,15,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,15, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);

    //---RWY, RWP
//        RBenableFrictionCompensation(2,16,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,16, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch,JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch,JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);

    //---RWY2
//        RBenableFrictionCompensation(3,36,DISABLE,DISABLE);
    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);

    //---GainOverride recover
//        RBJointGainOverride(2,13,50,50,2000); //--RSP
//        RBJointGainOverride(2,14,50,50,2000); //--RSR
//        RBJointGainOverride(2,15,50,50,2000); //--RSY, REB
//        RBJointGainOverride(2,16,50,50,2000); //---RWY, RWP
//        RBJointGainOverride(2,36,50,50,2000); // RWY2
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 65,1000); //--RSP
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 65,1000); //--RSR
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 65,1000); //--RSY
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 65,1000); //--REB
    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 65,1000); //---RWY
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 65,1000); //--RWP
    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 65,1000); //--RF1

    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);
    usleep(1000*1000);
//        RBJointGainOverride(2,13,1000,1000,1000); //--RSP
//        RBJointGainOverride(2,14,1000,1000,1000); //--RSR
//        RBJointGainOverride(2,15,1000,1000,1000); //--RSY, REB
//        RBJointGainOverride(2,16,1000,1000,1000); //---RWY, RWP
//        RBJointGainOverride(2,36,1000,1000,1000); //---RWY2
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,1000); //--RSP
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,1000); //--RSR
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,1000); //--RSY
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,1000); //--REB
    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0,1000); //---RWY
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0,1000); //--RWP
    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,1000); //--RF1

}


double RX_ATC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    double alpha,RDF2,LDF2,torqueX=0,dTorque=0,mTorque=0;
    double old_torqueX=0,old_mTorque=0;
    double CL=0.0, dt = DEL_T,Lgain = 0.00;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};
    double M = weight,g=-9.81;
    double var_gain = 1.0;

    if(pv_Index >=3 && RF_LPF_Fz > 20 && window[0].state == SSP_RF)
    {
        var_gain = 750.0/(RF_LPF_Fz+30.);
        if(var_gain > 50.0)
        {
            var_gain = 50.0;

        }else if(var_gain < 1.0)
        {
            var_gain = 1.0;
        }
    }else
    {
        var_gain = 1.0;
    }
        var_gain = 1.0;
//GLOBAL_VAR_GAIN_RR = var_gain;

        double d = 580,T = 100.,new_gain = 100.0;//(80.0-var_gain);// + 120.0;// + 100;


    alpha = fabs(ref_zmp_y + GLOBAL_CPC_ZMP_Y_REF*1.0 - LFoot_y)/fabs(LFoot_y - RFoot_y);

//    torque_ratio = alpha;

//    if(window[0].state == SSP_RF && RF_LPF_Fz < 40){
//        alpha = 0;
//    }

//    if(window[0].state == SSP_LF && LF_LPF_Fz < 40){
//        alpha = 1;
//    }
    if(window[0].state == SSP_RF){
        alpha = 0;
    }

    if(window[0].state == SSP_LF){
        alpha = 1;
    }

    if(alpha<=0){
        alpha = 0.;
    }else if(alpha>=1){
        alpha = 1.;
    }

    GLOBAL_RDF2 = RDF2 = (-alpha*M*g);
    GLOBAL_LDF2 = LDF2 = (-(1.0 - alpha)*M*g);

//    des_right_roll_torq = torqueX = (GLOBAL_CPC_ZMP_Y_REF  + OLD_ZMP_ERR_Y)*RDF2;//(-(RFoot_x - zmp_x)*RDF2 - (LFoot_x-zmp_x)*LDF2);


   des_right_roll_torq = torqueX = GLOBAL_CPC_ZMP_Y_REF*RDF2;//(GLOBAL_CPC_ZMP_Y_REF )*RDF2;//(-(RFoot_x - zmp_x)*RDF2 - (LFoot_x-zmp_x)*LDF2);


//    printf("GLOBAL_CPC_ZMP_Y_REF: %f     RDF2: %f     des_right_roll_torq: %f \n",GLOBAL_CPC_ZMP_Y_REF,RDF2,des_right_roll_torq);
////    RmTorqueX = mTorque = alpha*torqueX;

    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);

    if(torqueX >40. ){ torqueX = 40.;}

    if(torqueX <-40. ){ torqueX = -40.;}


    des_right_roll_torq = torqueX;

    if(mTorque >40.){ mTorque = 40.;}

    if(mTorque <-40. ){ mTorque = -40.;}

    if(window[0].state == SSP_RF && RF_LPF_Fz < 40)
    {
            T = 0.15;
    }else{
//                           d = 80,T = 100.5;
                                   d = new_gain,T = 100.5;
    }


    if((window[0].state == DSP_FINAL || window[0].state == STATE_EMPTY))// && RF_LPF_Fz < 50 ) )
    {
//                   d = 80,T = 100.5;
                                   d = new_gain*2.0,T = 100.5;
    }

    if((window[0].state == DSP_INIT_RF) || window[0].state == DSP_INIT_LF)// && RF_LPF_Fz < 50 ) )
    {

//                d = 180,T = 100.5;
                                   d = new_gain,T = 100.5;
    }

//0.000007
    if(pv_Index >=3)
    {
        Q[0] = ( torqueX - mTorque)/d   - (1.0/T)*Q[1];

        old_torqueX = torqueX;
        old_mTorque = mTorque;

        CL = Q[2] = Q[1] + Q[0]*dt + ( torqueX - mTorque)*0.00000;

        if(CL > 15*D2R){ CL = 15.0*D2R;}
        else if(CL < -15.0*D2R){ CL = -15.0*D2R;}

        Q[1] = CL;//Q[2];
    }else
    {
        old_torqueX = torqueX;
        old_mTorque = mTorque;
    }

    return CL*R2D;


}

double LX_ATC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{

    double alpha,RDF2,LDF2,torqueX=0,dTorque=0,mTorque=0;
    double old_torqueX=0,old_mTorque=0;
    double CL=0.0, dt = DEL_T,Lgain = 0.00;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};
    double M = weight,g=-9.81;
    double var_gain = 0.;



    if( pv_Index >=3 && LF_LPF_Fz > 20 && window[0].state == SSP_LF)
    {
        var_gain = 750.0/(LF_LPF_Fz+30.);
        if(var_gain > 50.0)
        {
            var_gain = 50.0;
        }else if(var_gain < 1.)
        {
            var_gain = 1.;
        }
    }else
    {
        var_gain = 1.0;
    }

            var_gain = 1.0;
//GLOBAL_VAR_GAIN_LR = var_gain;


        double d = 580,T = 100.,new_gain =100.0;//(80.-var_gain);// 120.0;// + 100.;

    d = new_gain,T = 100.5;

//    alpha = fabs(ref_zmp_y - LFoot_y  + GLOBAL_CPC_ZMP_Y_REF*0.3)/fabs(LFoot_y - RFoot_y);
    ratio = alpha = fabs(ref_zmp_y - LFoot_y  + GLOBAL_CPC_ZMP_Y_REF*1.0)/fabs(LFoot_y - RFoot_y);




//    if(window[0].state == SSP_RF && RF_LPF_Fz < 30 ){
//        alpha = 0;
//    }

//    if(window[0].state == SSP_LF && LF_LPF_Fz < 30 ){
//        alpha = 1;
//    }


    if(window[0].state == SSP_RF){
        alpha = 0;
    }

    if(window[0].state == SSP_LF){
        alpha = 1;
    }

    if(alpha<=0){ alpha = 0.;
    }else if(alpha>=1){ alpha = 1.;}

    RDF2 = (-alpha*M*g);
    LDF2 = (-(1.0 - alpha)*M*g);

    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;

    des_left_roll_torq = torqueX = (GLOBAL_CPC_ZMP_Y_REF)*LDF2;//(-(RFoot_x - zmp_x)*RDF2 - (LFoot_x-zmp_x)*LDF2);



    if(torqueX >40 ){torqueX = 40.;}

    if(torqueX <-40 ){torqueX = -40.;}

    des_left_roll_torq = torqueX;


    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);

    if(mTorque >40 ){mTorque = 40.;}

    if(mTorque <-40 ){mTorque = -40.;}

    if((window[0].state == SSP_LF && LF_LPF_Fz < 40 ) ){
        T = .15;

    }else{
//                           d = 80,T = 100.5;
                                   d = new_gain,T = 100.5;
    }


    if((window[0].state == DSP_FINAL || window[0].state == STATE_EMPTY))// && RF_LPF_Fz < 50 ) )
    {
                                           d = new_gain*2.0,T = 100.5;
    }


    if((window[0].state == DSP_INIT_RF) || window[0].state == DSP_INIT_LF)// && RF_LPF_Fz < 50 ) )
    {
                                   d = new_gain,T = 100.5;
    }

    //0.000007

    if(pv_Index >=3)
    {
//            Q[0] = ( torqueX - mTorque + Lgain*(old_torqueX-mTorque) )/d - (1.0/T)*Q[1] ;
        Q[0] = ( torqueX - mTorque )/d  - (1.0/T)*Q[1] ;

//                        Q[0] = ( torqueX - mTorque )/d + ( (torqueX - old_torqueX) - (mTorque - old_mTorque) )/1000 - (1.0/T)*Q[1];
        old_torqueX = torqueX;
        old_mTorque = mTorque;

//        CL = Q[2] = Q[1] + Q[0]*dt + ( torqueX - mTorque)*0.000002;
        CL = Q[2] = Q[1] + Q[0]*dt + ( old_torqueX - mTorque)*0.00000;

        if(CL > 15*D2R){ CL = 15.0*D2R;}
        else if(CL < -15.0*D2R){ CL = -15.0*D2R;}

        Q[1] = CL;//Q[2];
    }else
    {
        old_torqueX = torqueX;
        old_mTorque = mTorque;
    }

    return CL*R2D;
}

double RY_ATC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    double alpha,RDF2,LDF2,torqueX=0,dTorque=0,mTorque=0;
    static double old_torqueX=0,old_mTorque=0;

    double M = weight,g=-9.81;

    alpha = fabs((ref_zmp_y) - LFoot_y )/fabs(LFoot_y - RFoot_y);

    if(window[0].state == SSP_RF){
        alpha = 0;
    }

    if(window[0].state == SSP_LF){
        alpha = 1;
    }

    if(alpha<=0)
    {
        alpha = 0.;
    }else if(alpha>=1)
    {
        alpha = 1.;
    }


    RDF2 = (-alpha*M*g);
    LDF2 = (-(1.0 - alpha)*M*g);

    dTorque = 0.;



    des_right_pitch_torq = torqueX = -(GLOBAL_CPC_ZMP_X_REF )*RDF2;//(-(RFoot_x - zmp_x)*RDF2 - (LFoot_x-zmp_x)*LDF2);

//    printf("RDF: %f    dRpitch: %f \n",RDF2,des_right_pitch_torq);

    if(torqueX >40 )
    {
        torqueX = 40.;
    }

    if(torqueX <-40. )
    {
        torqueX = -40.;
    }


    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);
    if(mTorque >40. )
    {
        mTorque = 40.;
    }

    if(mTorque <-40. )
    {
        mTorque = -40.;
    }

    des_right_pitch_torq = torqueX;


    static double CL=0.0;
    double  m = 1.5,dt = DEL_T,Lgain = 0.002;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};
    static int RMXC_cnt = 0;
    double var_gain = 0.;



    if(pv_Index >=3 && RF_LPF_Fz > 40 && window[0].state == SSP_RF )
    {
        var_gain = 750.0/(RF_LPF_Fz+40);
        if(var_gain > 50.0)
        {
            var_gain = 50.0;

        }else if(var_gain < 1.0)
        {
            var_gain = 1.0;
        }
    }else
    {
        var_gain = 1.0;
    }

            var_gain = 1.0;
//GLOBAL_VAR_GAIN_RP = var_gain;
//GLOBAL_VAR_GAIN = var_gain;

double d = 430,T = 100.5,new_gain = 150.0;//90.-var_gain*1.2;//((150.0-80.0 + 10.0)/var_gain)*1.0;

    d = new_gain,T = 100.5;

    if((window[0].state == SSP_RF && RF_LPF_Fz < 20 ) )
    {
        T = 0.2;
    }else
    {
//                           d = 80,T = 100.5;
       d = new_gain,T = 100.5;
    }

    if((window[0].state == DSP_INIT_RF) || window[0].state ==DSP_INIT_LF)// && RF_LPF_Fz < 50 ) )
    {
//        d = 100,T = 100.5;
       d = new_gain,T = 100.5;
    }

    if((window[0].state == STATE_EMPTY) || window[0].state == DSP_FINAL)// && RF_LPF_Fz < 50 ) )
    {
//        d = 100,T = 100.5;
       d = new_gain*2.0,T = 100.5;
    }

    if(pv_Index >=3)
    {
        Q[0] = ( torqueX - mTorque )/d + ((torqueX-old_torqueX)/DEL_T -(mTorque-old_mTorque)/DEL_T)*0.000002   - (1.0/T)*Q[1] ;//+ ( torqueX - mTorque)*0.000000;

        old_torqueX = torqueX;
        old_mTorque = mTorque;

        CL = Q[2] = Q[1] + Q[0]*dt  ;

        if(CL > 25*D2R){ CL = 25.0*D2R;}
        else if(CL < -25.0*D2R){ CL = -25.0*D2R;}

        Q[1] = CL;//Q[2];
    }else
    {
        old_torqueX= torqueX;
        old_mTorque= mTorque;
    }

        return CL*R2D;

}

double LY_ATC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    double alpha,RDF2,LDF2,torqueX=0,dTorque=0,mTorque=0;
    double old_torqueX=0,old_mTorque=0;
double M = weight,g=-9.81;
//    double M = 174.5,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);

    if(alpha<=0){
        alpha = 0.;
    }else if(alpha>=1){
        alpha = 1.;
    }

    if(window[0].state == SSP_RF){
        alpha = 0;
    }

    if(window[0].state == SSP_LF){
        alpha = 1;
    }

    RDF2 = (-alpha*M*g);
    LDF2 = (-(1.0 - alpha)*M*g);

    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;

    des_left_pitch_torq = torqueX = - (GLOBAL_CPC_ZMP_X_REF )*LDF2;

    GLOBAL_DES_LDF = LDF2;


    if(torqueX >40. ){
        torqueX = 40.;
    }

    if(torqueX <-40. ){
        torqueX = -40.;
    }

    des_left_pitch_torq = torqueX ;

    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);

    if(mTorque >40. ){
        mTorque = 40.;
    }

    if(mTorque <-40. ){
        mTorque = -40.;
    }

    des_left_pitch_torq = torqueX;


    static double CL=0.0;
    double  m = 1.5,dt = DEL_T;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};
    static int RMXC_cnt = 0;
    double var_gain = 0.;

    if(pv_Index >=3 && LF_LPF_Fz > 20 && window[0].state == SSP_LF )
    {
        var_gain = 750.0/(LF_LPF_Fz +40);
        if(var_gain > 50.0)
        {
            var_gain = 50.0;
        }else if(var_gain < 1.0)
        {
            var_gain = 1.0;
        }
    }else
    {
        var_gain = 1.0;
    }

//        var_gain = 1.0;
        GLOBAL_VAR_GAIN_LP = var_gain;

    double d = 430,T = 100.5,new_gain = 150.0;//90-var_gain*1.2;//((150.0-80.0 + 10.0 )/var_gain)*1.0;

    d = new_gain,T = 100.5;

    if((window[0].state == SSP_LF && LF_LPF_Fz < 40 ) )
    {
        T = 0.2;
    }else
    {
//                           d = 80,T = 100.5;
                                               d = new_gain,T = 100.5;
    }

    if((window[0].state == DSP_INIT_RF) || window[0].state == DSP_INIT_LF)// && RF_LPF_Fz < 50 ) )
    {
//        d = 100,T = 100.5;
                                               d = new_gain,T = 100.5;
    }

    if((window[0].state == STATE_EMPTY)|| window[0].state == DSP_FINAL)// && RF_LPF_Fz < 50 ) )
    {
//                   d = 100.,T = 100.5;
                                               d = new_gain*2.0,T = 100.5;
    }

    if(pv_Index >=3)
    {

//            Q[0] = ( torqueX - mTorque )/d - (1.0/T)*Q[1];

//            Q[0] = ( torqueX - mTorque )/d + ( (torqueX - old_torqueX) - (mTorque - old_mTorque) )/700 - (1.0/T)*Q[1];

                    Q[0] = ( torqueX - mTorque )/d + ((torqueX-old_torqueX)/DEL_T -(mTorque-old_mTorque)/DEL_T)*0.000002  - (1.0/T)*Q[1];

        old_torqueX = torqueX;
        old_mTorque = mTorque;

//    printf("GLOBAL_CPC_ZMP_Y_REF: %f   LX dtorque: %f  mtorque : %f\n",GLOBAL_CPC_ZMP_Y_REF,torqueX,mTorque);
//            CL = Q[2] = Q[1] + Q[0]*dt + ( torqueX - mTorque)*0.000003;

        CL = Q[2] = Q[1] + Q[0]*dt;// + ( torqueX - mTorque)*0.000000;

        if(CL > 25*D2R){ CL = 25.0*D2R;}
        else if(CL < -25.0*D2R){ CL = -25.0*D2R;}

        Q[1] = CL;// Q[2];

    }else
    {
        old_torqueX= torqueX;
        old_mTorque= mTorque;
    }

    return CL*R2D;
}


double RightDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF2,LDF2;
    // no weight
//    double M = 174.5,g=-9.81;

    // with people.
      double M = weight,g=-9.81;

//    alpha = (fabs(ref_zmp_y + GLOBAL_CPC_ZMP_Y_REF*0.8 - LFoot_y)/fabs(LFoot_y - RFoot_y));


    alpha = (fabs((ref_zmp_y + GLOBAL_CPC_ZMP_Y_REF)*1.0 - LFoot_y)/fabs(LFoot_y - RFoot_y));




    if(window[0].state == SSP_RF)
    {
        alpha = 0;
    }


    if(window[0].state == SSP_LF)
    {
        alpha = 1;
    }



    if(alpha<=0)
    {
        alpha = 0.;
    }else if(alpha>=1)
    {
        alpha = 1.;
    }

     alpha;

//    printf("GLOBAL_CPC_ZMP_Y_REF: %f \n",GLOBAL_CPC_ZMP_Y_REF);
////    printf("ratio : %f , ref_zmp_y: %f , LFoot_y; %f, RFoot_y:%f  \n",ratio,ref_zmp_y,LFoot_y,RFoot_y);


    RDF2 = -alpha*(M*g);
    LDF2 = -(1.0 - alpha)*M*g;

//    printf("alhpa : %f    RDF : %f \n",alpha,RDF2);

    return RDF2;
}

double LeftDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF2,LDF2;
   // with people
//    double M = 174.5,g=-9.81;
     double M = weight,g=-9.81;

    double min = -0.1,max = 0.1;
//    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
//    alpha = (fabs(GLOBAL_CPC_ZMP_Y_REF - max)/fabs(max - min));

//    alpha = (fabs(ref_zmp_y + GLOBAL_CPC_ZMP_Y_REF*0.8 - LFoot_y)/fabs(LFoot_y - RFoot_y));

    alpha = (fabs((ref_zmp_y + GLOBAL_CPC_ZMP_Y_REF)*1.0 - LFoot_y)/fabs(LFoot_y - RFoot_y));


    if(window[0].state == SSP_RF)
    {
        alpha = 0;
    }


    if(window[0].state == SSP_LF)
    {
        alpha = 1;
    }

    if(alpha<=0)
    {
        alpha = 0.;
    }else if(alpha>=1)
    {
        alpha = 1.;
    }

    RDF2 = -alpha*M*g;
    LDF2 = -(1.0 - alpha)*M*g;


    return LDF2;
}

double FootForceControl(double state ,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{
     double CL=0.0;
     double d1 = 35000.,d2 = 28000,dt = 0.005,T = 100.0;
    static double Q[3]={0.0,},old_zctrl = 0.;
    static int ssp_CNT = 0;

//    static double

//    ratio = (fabs(window[0].zmp.y + GLOBAL_CPC_ZMP_Y_REF*1.0 - 0.125)/fabs(0.125 - (-0.125)));

//    if(ratio<=0)
//    {
//        ratio = 0.;
//    }else if(ratio>=1)
//    {
//        ratio = 1.;
//    }


    if(window[0].state == DSP_INIT_LF && window[0].state == DSP_INIT_RF)
    {
         d1 = 45000;
         T = 100;
    }

    if(window[0].state != DSP_INIT_LF && window[0].state != DSP_INIT_RF)
    {

//        if((window[0].state == SSP_LF && LF_LPF_Fz < 30 ) || (window[0].state == SSP_RF && RF_LPF_Fz < 30))

        if((window[0].state == SSP_LF  ) || (window[0].state == SSP_RF ))
        {
            ssp_CNT = ssp_CNT +1;

//            d1 = 25000.;
//            T = 0.12;



            d1 = 35000.;
            T = 0.15;

        }else
        {
            ssp_CNT = 0;
        }

//        if((window[0].state == DSP_LF || window[0].state == DSP_RF) && (LF_LPF_Fz > 30 && RF_LPF_Fz > 30))
        if((window[0].state == DSP_LF || window[0].state == DSP_RF));// && (LF_LPF_Fz > 30 && RF_LPF_Fz > 30))
        {
//            d1 =25000;
//            T = 100.0;

            d1 =8000.;
            T = 100.0;

        }
    }

    if(window[0].state == STATE_EMPTY || window[0].state == DSP_FINAL)
    {
//        d1 = 65000.;
//        d1 = 85000.;

//        d1 = 35000.;

//        d1 = 35000.;

        d1 = 25000.;


        T = 100.;

    }

    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    double RFXn = RF_LPF_Fx;//sharedSEN->FT[RAFT].Fx;
    double RFYn = RF_LPF_Fy;//sharedSEN->FT[RAFT].Fy;
    double LFXn = LF_LPF_Fx;//sharedSEN->FT[LAFT].Fx;
    double LFYn = LF_LPF_Fy;//sharedSEN->FT[LAFT].Fy;

    double temp_mLforce = LF_LPF_Fz;//+ sqrtp(LFXn*LFXn) + sqrtp(LFYn*LFYn);
    double temp_mRforce = RF_LPF_Fz;//+ sqrtp(RFXn*RFXn) + sqrtp(RFYn*RFYn);


    {

        if(pv_Index >=3)
        {
//            if(reset == 1)
//            {
//                //DSP
//                d = 5000;
//            }else
//            {
//                d = 25000;
//            }

//            Q[0] = ( (dLforce - dRforce) - ((mLforce) - mRforce) )/d - (1.0/T)*Q[1];


//            Q[0] = ( -sharedData->FOGRoll -sharedData->FOGRollVel*0.05 )/d - (1.0/T)*Q[1];

//            Q[0] = ((dLforce - dRforce) - ((mLforce) - mRforce) )/d1 - (1.0/T)*Q[1];

            Q[0] = ((dLforce - dRforce) - (temp_mLforce - temp_mRforce) )/d1 - (1.0/T)*Q[1];

//            Q[0] = ( (1.0 - ratio)*(temp_mLforce + temp_mRforce) - ratio*(temp_mLforce + temp_mRforce) - (temp_mLforce - temp_mRforce) )/d1 - (1.0/T)*Q[1];

//            printf("ratio: %f  RDF : %f  LDF:%f  mRforce: %f   mLforec:%f  \n",ratio,(1.0 - ratio)*(mLforce + mRforce),ratio*(mLforce + mRforce),mLforce,mRforce);

            CL = Q[2] = Q[1] + Q[0]*0.005;

            Q[1] = Q[2];

            if(CL > 0.07)
            {
                CL = 0.07;
            }else if( CL < -0.07)
            {
                CL = -0.07;
            }


//    printf("Q[0]:%f  d1:%f   T:%f   Q[1]:%f  RDF : %f  LDF:%f  temp_mLforce: %f   temp_mRforce:%f  \n",Q[0],d1,T,Q[1],dLforce,dRforce,temp_mLforce,temp_mRforce);
//printf("Q[0]:%f  d1:%f   T:%f   Q[1]:%f  RDF : %f  LDF:%f  temp_mLforce: %f   temp_mRforce:%f  \n",Q[0],d1,T,Q[1],dLforce,dRforce,mLforce,mRforce);
        }
    }

//    old_zctrl = old_zctrl*0.8+ CL*0.2;

    return CL;
}

void ReactiveControl(int state,int state2,int state3)
{


    double cpx2 = 0.,cpy2 = 0.,cpx3 = 0.,cpy3 = 0.;
    static double old_cpx3 = 0.,old_cpy3 = 0.,old_cpx_dot = 0.,old_cpy_dot = 0.,old_pitch=0.,old_roll=0.,old_pitch_vel=0.,old_roll_vel=0.,roll_vel2 = 0,roll_vel3 = 0.,pitch_vel3 = 0.,lpf_roll2 = 0,old_lpf_roll2 = 0, old_lpf_pitch2 = 0.,lpf_pitch2 = 0.,pitch_vel2 = 0.,old_lpf_pitch_vel2 = 0.,old_lpf_roll_vel2 = 0.;
    double F_gain = 1.0,F_gain2 = 0.07;
    static double initial_roll = 0.,initial_pitch = 0.;

    if(pv_Index <= 1)
    {
        old_roll = initial_roll = sharedData->FOG.Roll;
        old_pitch = initial_pitch = sharedData->FOG.Pitch;
    }

    //******************88
//            lpf_roll2 = -(sharedData->IMU[CIMU].Roll + (GLOBAL_LHR_Comp + GLOBAL_RHR_Comp))*D2R*0.83*F_gain + old_lpf_roll2*(1.0  - F_gain);
    GLOBAL_Roll = lpf_roll2 = -((sharedData->IMU[CIMU].Roll*1.0 - initial_roll*0.0 + 0.2))*D2R*0.8*F_gain + old_lpf_roll2*(1.0  - F_gain);

    GLOBAL_Roll_vel = roll_vel3 = -sharedData->IMU[CIMU].RollVel*D2R*0.8*F_gain2 + old_lpf_roll_vel2*(1.0  - F_gain2);//(roll_vel2*0.6 + 0.4*(lpf_roll2-old_lpf_roll2)/DEL_T)*0.77;//((lpf_roll2-old_lpf_roll2)/DEL_T)*1.0;// + roll_vel2*0.5;

    old_lpf_roll_vel2 = roll_vel3;

    GLOBAL_Pitch = lpf_pitch2 = ((sharedData->IMU[CIMU].Pitch*1.0 - 0.5 - initial_pitch*0.0) )*D2R*0.8*F_gain + old_lpf_pitch2*(1.0  - F_gain);


    GLOBAL_Pitch_vel =  pitch_vel3 =  ((sharedData->IMU[CIMU].PitchVel*D2R))*0.8*F_gain2 + old_lpf_pitch_vel2*(1.0  - F_gain2);//(pitch_vel2*0.6 + 0.4*(lpf_pitch2-old_lpf_pitch2)/DEL_T)*0.77;

    old_lpf_pitch_vel2 = pitch_vel3;

    double cpx_lpf=0.5,cpy_lpf=0.5;

    cpx3 = cpx3*(1.-cpx_lpf) + cpx_lpf*(lpf_pitch2*3.5 + 1.8*pitch_vel3/sqrt(9.81/0.80));//*0.8 + old_cpx*0.2;

    cpy3 = cpy3*(1.-cpy_lpf) + cpy_lpf*(lpf_roll2*2.0 + 1.8*roll_vel3/sqrt(9.81/0.80));//(lpf_roll2 + roll_vel2/sqrt(9.81/0.77));//*0.8 + old_cpy*0.2;

    CPX_DOT = (cpx3 - old_cpx3)/TICK_TIME;
    CPY_DOT = (cpy3 - old_cpy3)/TICK_TIME;


    CPX_DOT = CPX_DOT*0.07 + old_cpx_dot*0.93;
    CPY_DOT = CPY_DOT*0.07 + old_cpy_dot*0.93;

    old_cpy3 = cpy3;
    old_cpx3 = cpx3;

    old_cpx_dot = CPX_DOT;

    old_cpy_dot = CPY_DOT;

    {
        GLOBAL_CPC_ZMP_Y_REF = cpy3*1.0*2.0;// + cpy3_GLOBAL_CPC_DOT_Y*0.02;
        GLOBAL_CPC_ZMP_X_REF = cpx3*1.0*2.0;// + cpx3_GLOBAL_CPC_DOT_X*0.02;

        GLOBAL_CPC_ZMP_Y_REF2 = cpy2*2.5;//*0.25;//*5*1.4;
        GLOBAL_CPC_ZMP_X_REF2 = cpx2*2.5;//*0.25;//*5;

    }


    RDF = RightDesForce(state, X_ZMP_REF_Local,Y_ZMP_REF_Local,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n);

    LDF = LeftDesForce(state, X_ZMP_REF_Local,Y_ZMP_REF_Local,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n);


    if(sharedData->FT[RAFT].Fz > 20 || sharedData->FT[LAFT].Fz> 20)
    {

        RDRoll_atc = RX_ATC(state,state2,state3,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_Local/1000.0,Y_ZMP_Local/1000.0,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n,RF_LPF_Mx,0);//,sharedData->FT[RAFT].Mx,fsm->RightInfos[0][4]*D2R);
        LDRoll_atc = LX_ATC(state,state2,state3,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_Local/1000.0,Y_ZMP_Local/1000.0,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n,LF_LPF_Mx,0);

        RDPitch_atc = RY_ATC(state,state2,state3,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_Local/1000.0,Y_ZMP_Local/1000.0,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n,RF_LPF_My,0);
        LDPitch_atc = LY_ATC(state,state2,state3,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_Local/1000.0,Y_ZMP_Local/1000.0,GLOBAL_X_LF_n,GLOBAL_X_RF_n,GLOBAL_Y_LF_n,GLOBAL_Y_RF_n,LF_LPF_My,0);

        Zctrl = FootForceControl(1,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,0,0.0);



    }

}

double Pelvis_Ori_Control()
{
    double CL=0.0;
    double d1 = 15000.,d2 = 28000,dt = 0.005,T = 1000.0;
    static double Q[3]={0.0,};

    double RF_Roll_Ref = -1.0,LF_Roll_Ref = 1.0;

    static double LF_phase_cnt = 0,LFRF_phase_cnt = 0,RF_phase_cnt = 0;
    static int before_state = 0,ori_state_cnt = 0;
    static double LF_pelvis_cnt = 0.,RF_pelvis_cnt = 0.;

/// ************** 8.18

    // compenstate the hip roll

    static double Rflag = 0,Lflag = 0;

    if(window[0].state == DSP_LF || window[0].state == SSP_LF)
    {

        RF_pelvis_cnt = 0.;
        LF_phase_cnt = 0.0;
        LFRF_phase_cnt = 0;

        LF_pelvis_cnt = LF_pelvis_cnt +1;

        if(LF_pelvis_cnt*0.005  > DSP_TIME*0.8)
        {

            RF_phase_cnt = RF_phase_cnt + 0.01;

            if(RF_phase_cnt >1.0)
            {
                RF_phase_cnt = 1.0;

            }
        }

        GLOBAL_RHR_Comp = RF_Roll_Ref*ZeroToOneCosine(RF_phase_cnt);

        GLOBAL_LHR_Comp = LF_Roll_Ref*OneToZeroCosine(RF_phase_cnt);

        before_state = DSP_LF;
    }

    if(window[0].state == DSP_INIT_RF)
    {

        LF_phase_cnt = LF_phase_cnt + 0.004;

        if(LF_phase_cnt >1.0)
        {
            LF_phase_cnt = 1.0;
            RF_phase_cnt = 0.0;
            LFRF_phase_cnt = 0;
        }

        GLOBAL_LHR_Comp = LF_Roll_Ref*ZeroToOneCosine(LF_phase_cnt);

        Rflag = 0.0;

    }


    if(window[0].state == DSP_RF || window[0].state == SSP_RF)
    {
        LF_pelvis_cnt = 0.;
        RF_phase_cnt = 0.0;
        LFRF_phase_cnt = 0;

        RF_pelvis_cnt = RF_pelvis_cnt +1;

        // 1.1 dsp 20%
        if(RF_pelvis_cnt*0.005  > DSP_TIME*0.8)
        {
            LF_phase_cnt = LF_phase_cnt + 0.01;

            if(LF_phase_cnt >1.0)
            {
                LF_phase_cnt = 1.0;
            }
        }

        GLOBAL_LHR_Comp = LF_Roll_Ref*ZeroToOneCosine(LF_phase_cnt);

        GLOBAL_RHR_Comp = RF_Roll_Ref*OneToZeroCosine(LF_phase_cnt);

        before_state = DSP_RF;
    }

    if(window[0].state == DSP_FINAL || window[0].state == STATE_EMPTY)
    {
        LFRF_phase_cnt  = LFRF_phase_cnt + 0.025;
        RF_pelvis_cnt = 0.;
        LF_pelvis_cnt= 0.;

        if(LFRF_phase_cnt  >1.0)
        {
            LFRF_phase_cnt  = 1.0;
            RF_phase_cnt = 0.0;
                        LF_phase_cnt = 0.0;
        }

        if(before_state == DSP_RF)
        {

            GLOBAL_LHR_Comp = LF_Roll_Ref*OneToZeroCosine(LFRF_phase_cnt);
        }


        if(before_state == DSP_LF)
        {
            GLOBAL_RHR_Comp = RF_Roll_Ref*OneToZeroCosine(LFRF_phase_cnt);
        }

    }
}

double OneToZeroCosine(double _time)
{
    if(_time > 1.)
    {
        _time = 1.;
    }

    if(_time < 0.)
    {
        _time = 0.;
    }

    return 1./2.*(cos(_time*PI) + 1.);
}

double ZeroToOneCosine(double _time)
{
    if(_time > 1.)
    {
        _time = 1.;
    }

    if(_time < 0.)
    {
        _time = 0.;
    }

    return 1./2.*(1. - cos(_time*PI));
}

double LA_Notch_Filter(double u, int zero)
{

    static double x1new,x2new, x1=0., x2=0.;

    static double filt;
//** 2.8~3.8hz
//    const double adm[4] = {0.963902092858671, 0.100718376039535, -0.100718376039535,0.994834675664877};
//    const double bdm[2] = {0.0437452781237664, -0.00224346895298136};
//    const double cdm[2] = {-0.694344243723409, -0.0356093233438260};
//    const double ddm = 0.984533708596897;


    //** 2.5~ 3.5
        const double adm[4] = {0.964818957103952, 0.0913644073095150, -0.0913644073095150,0.995751539910159};
        const double bdm[2] = {0.0437452781237664, -0.00203416268655044};
        const double cdm[2] = {-0.694668404186043,-0.0323021959838239};
        const double ddm = 0.984533708596897;


    x1new=adm[0]*x1 + adm[1]*x2 + bdm[0]*u;
    x2new=adm[2]*x1 + adm[3]*x2 + bdm[1]*u;

    if(zero==0) { x1new=0.; x2new=0.;}

    filt=cdm[0]*x1new+cdm[1]*x2new + ddm*u;


    x1=x1new;	x2=x2new;

    return filt;

}

double RA_Notch_Filter(double u, int zero)
{

    static double x1new,x2new, x1=0., x2=0.;

    static double filt;

//    const double adm[4] = {0.963902092858671, 0.100718376039535, -0.100718376039535,0.994834675664877};
//    const double bdm[2] = {0.0437452781237664, -0.00224346895298136};
//    const double cdm[2] = {-0.694344243723409, -0.0356093233438260};
//    const double ddm = 0.984533708596897;
    const double adm[4] = {0.964818957103952, 0.0913644073095150, -0.0913644073095150,0.995751539910159};
    const double bdm[2] = {0.0437452781237664, -0.00203416268655044};
    const double cdm[2] = {-0.694668404186043,-0.0323021959838239};
    const double ddm = 0.984533708596897;


    x1new=adm[0]*x1 + adm[1]*x2 + bdm[0]*u;
    x2new=adm[2]*x1 + adm[3]*x2 + bdm[1]*u;

    if(zero==0) { x1new=0.; x2new=0.;}

    filt=cdm[0]*x1new+cdm[1]*x2new + ddm*u;


    x1=x1new;	x2=x2new;

    return filt;

}

void Preview()
{
    static double pv_state[2][3] = {{0.0}}, pv_state_old[2][3] = {{0.0}};
    static double pv_ZMP[2] = {0.f,};
    static double pv_Err[2] = {0.f,};
    static double pv_U[2] = {0.f,};
    double temp_sum[2]={0.,0,};
    int pv_time_Index = 0;

//    if(fsm->StateInfos[0][0] != STATE_FINISHED)
    {
//        printf("%d =========== \n ",pv_Index);
        if(pv_Index == 0)
        {
//            temp_X = GLOBAL_X_LIPM_n - 0.001*Del_PC_X_DSP_XZMP_CON - I_ZMP_CON_X;
//            temp_Y = U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0- 0.001*Del_PC_Y_DSP_YZMP_CON - I_ZMP_CON_Y;
//            pv_state_old[0][0] = temp_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) - temp_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            pv_state_old[1][0] = temp_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + temp_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            Del_PC_X_DSP_XZMP_CON = 0;
//            Del_PC_Y_DSP_YZMP_CON = 0;
//            kirkZMPCon_XP2(0,0,0);
//            kirkZMPCon_YP2(0,0,0);
//            I_ZMP_CON_X =0;
//            I_ZMP_CON_Y =0;
//            printf("reset =========== \n ");

//                    WBIK_flag=1;
         }


        for(int i=0;i<=1;i++)
        {
            //=========================

            if(i == 0)
            {
                sys_out_zmp_x = pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];
                pv_Err[i] = pv_Err[i] + pv_ZMP[i] - window[0].zmp.x;

                temp_sum[i] = 0.0f;

                for(pv_time_Index=0;pv_time_Index<WIN_NUM;pv_time_Index++)
                {
                    temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(window[pv_time_Index].zmp.x);
                }
            }else
            {
                sys_out_zmp_y = pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];
                pv_Err[i] = pv_Err[i] + pv_ZMP[i] - window[0].zmp.y;

                temp_sum[i] = 0.0f;

                for(pv_time_Index=0;pv_time_Index<WIN_NUM;pv_time_Index++)
                {
                    temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(window[pv_time_Index].zmp.y);
                }
            }

            pv_U[i] = -pv_Gi[1]*pv_Err[i] - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

            //temp_sum[i] =0;
            //pv_Err[i] =0;

            pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
            pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
            pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];
        }

        GLOBAL_Y_LIPM = pv_state[1][0];
        GLOBAL_X_LIPM = pv_state[0][0];

        GLOBAL_Y_LIPM_d = pv_state[1][1];
        GLOBAL_X_LIPM_d = pv_state[0][1];

        GLOBAL_Y_LIPM_dd = pv_state[1][2];
        GLOBAL_X_LIPM_dd = pv_state[0][2];

////        GLOBAL_X_RF = window[0].right_foot_ref.x;
//        GLOBAL_Y_RF = window[0].right_foot_ref.y;
////        GLOBAL_Z_RF = window[0].right_foot_ref.z;

////        GLOBAL_X_LF = window[0].left_foot_ref.x;
//        GLOBAL_Y_LF = window[0].left_foot_ref.y;
////        GLOBAL_Z_LF = window[0].left_foot_ref.z;




        GLOBAL_Z_RF = window[0].right_foot_ref.z;// + ryref[0];//fsm->RightInfos[0][1] + ryref[0];
        GLOBAL_Z_LF = window[0].left_foot_ref.z;// + lyref[0];//fsm->LeftInfos[0][1] + lyref[0];


        GLOBAL_X_RF = window[0].right_foot_ref.x;//+ rxref[0];
        GLOBAL_X_LF = window[0].left_foot_ref.x;//+ lxref[0];//fsm->LeftInfos[0][0] + lxref[0];

        GLOBAL_Y_RF = window[0].right_foot_ref.y;// + ryref[0];//fsm->RightInfos[0][1] + ryref[0];
        GLOBAL_Y_LF = window[0].left_foot_ref.y ;//+ lyref[0];//fsm->LeftInfos[0][1] + lyref[0];

        double Global[3],Local[3];

        Global[0] = GLOBAL_X_RF;
        Global[1] = GLOBAL_Y_RF;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_RF_n = Local[0];// GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_RF_n = Local[1];//-GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        Global[0] = GLOBAL_X_LF;
        Global[1] = GLOBAL_Y_LF;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LF_n = Local[0];// GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LF_n = Local[1];//-GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_ZMP_REF_X = window[0].zmp.x;
        GLOBAL_ZMP_REF_Y = window[0].zmp.y;


//        double Global[3],Local[3];
        Global[0] = GLOBAL_X_LIPM;
        Global[1] = GLOBAL_Y_LIPM;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LIPM_n =  Local[0];
        GLOBAL_Y_LIPM_n =  Local[1];

        Global[0] = GLOBAL_X_LIPM_d;
        Global[1] = GLOBAL_Y_LIPM_d;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LIPM_d_n =  Local[0];
        GLOBAL_Y_LIPM_d_n =  Local[1];

        Global[0] = GLOBAL_X_LIPM_dd;
        Global[1] = GLOBAL_Y_LIPM_dd;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LIPM_dd_n =  Local[0];
        GLOBAL_Y_LIPM_dd_n =  Local[1];

        pv_Index++ ;
    }
}


void GetGain(double H)
{
    int nCount=0;
    double temp_Gd_gain,temp_Gx_gain,temp_Gi_gain;

    pv_A[0][0] = 1.0f;
    pv_A[0][1] = TICK_TIME;
    pv_A[0][2] = TICK_TIME*TICK_TIME/2.0;

    pv_A[1][0] = 0.0f;
    pv_A[1][1] = 1.0f;
    pv_A[1][2] = TICK_TIME;

    pv_A[2][0] = 0.0f;
    pv_A[2][1] = 0.0f;
    pv_A[2][2] = 1.0f;

    pv_B[0][0] = TICK_TIME*TICK_TIME*TICK_TIME/6.0f;
    pv_B[1][0] = TICK_TIME*TICK_TIME/2.0f;
    pv_B[2][0] = TICK_TIME;


    pv_C[0][0] = 1.0f;
    pv_C[0][1] = 0.0f;
    pv_C[0][2] = -H/9.81f;

    pv_Index= 0;


    printf("get gain start \n");
    if(H >= 0.49 && H < 0.51)
        {   //printf("H =0.50 \n");
            fp2 = fopen("../share/Gain/Gd50.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx50.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi50.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.51 && H < 0.53)
    {   //printf("H =0.52 \n");
        fp2 = fopen("../share/Gain/Gd52.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx52.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi52.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.53 && H < 0.55)
    {   //printf("H =0.54 \n");
        fp2 = fopen("../share/Gain/500HzGd54.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx54.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi54.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.55 && H < 0.56)
    {   printf("H =0.55 \n");
        fp2 = fopen("../share/Gain/500HzGd55.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain; printf("gd %d \n",nCount);}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx55.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi55.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain; printf("xxx %d \n",nCount);}
        fclose(fp4);
    }
    else if(H >= 0.56 && H < 0.57)
    {   //printf("H =0.56 \n");
        fp2 = fopen("../share/Gain/500HzGd56.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx56.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi56.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.57 && H < 0.58)
    {   //printf("H =0.57 \n");
        fp2 = fopen("../share/Gain/500HzGd57.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx57.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi57.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.58 && H < 0.59)
    {   //printf("H =0.58 \n");
        fp2 = fopen("../share/Gain/500HzGd58.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx58.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi58.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.59 && H < 0.60)
    {   //printf("H =0.59 \n");
        fp2 = fopen("../share/Gain/500HzGd59.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/500HzGx59.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/500HzGi59.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.60 && H < 0.61)
    {   //printf("H =0.60 \n");
        fp2 = fopen("../share/Gain/Gd60.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx60.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi60.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.61 && H < 0.62)
    {   //printf("H =0.61 \n");
        fp2 = fopen("../share/Gain/Gd61.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx61.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi61.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.62 && H < 0.63)
    {   //printf("H =0.62 \n");
        fp2 = fopen("../share/Gain/Gd62.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx62.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi62.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.63 && H < 0.65)
        {   //printf("H =0.63 \n");
            fp2 = fopen("../share/Gain/Gd63.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx63.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi63.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.65 && H < 0.66)
    {   //printf("H =0.65 \n");
        fp2 = fopen("../share/Gain/Gd65.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx65.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi65.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.66 && H < 0.67)
    {   //printf("H =0.66 \n");
        fp2 = fopen("../share/Gain/Gd66.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx66.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi66.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.67 && H < 0.68)
    {   //printf("H =0.67 \n");
        fp2 = fopen("../share/Gain/Gd67.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx67.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi67.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.68 && H < 0.69)
    {//printf("H =0.68 \n");
        fp2 = fopen("../share/Gain/Gd68.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx68.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi68.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.69 && H < 0.70)
    {//printf("H =0.69 \n");
        fp2 = fopen("../share/Gain/Gd69.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx69.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi69.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.70 && H < 0.71)
    {//printf("H =0.70 \n");
        fp2 = fopen("../share/Gain/Gd70.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx70.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi70.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.71 && H < 0.72)
    {//printf("H =0.71 \n");
        fp2 = fopen("../share/Gain/Gd71.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx71.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi71.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.72 && H < 0.73)
    {//printf("H =0.72 \n");
        fp2 = fopen("../share/Gain/Gd72.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx72.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi72.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.73 && H < 0.74)
    {//printf("H =0.73 \n");
        fp2 = fopen("../share/Gain/Gd73.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx73.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi73.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.74 && H < 0.75)
    {//printf("H =0.74 \n");
        fp2 = fopen("../share/Gain/Gd74.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx74.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi74.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.75 && H < 0.76)
    {//printf("H =0.75 \n");
        fp2 = fopen("../share/Gain/Gd75.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx75.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi75.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.76 && H < 0.77)
    {//printf("H =0.76 \n");
        fp2 = fopen("../share/Gain/MPCGd76.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/MPCGx76.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/MPCGi76.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.77 && H < 0.78)
    {//printf("H =0.77 \n");
        fp2 = fopen("../share/Gain/MPCGd77.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/MPCGx77.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/MPCGi77.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.78 && H < 0.79)
    {//printf("H =0.78 \n");
        fp2 = fopen("../share/Gain/Gd78.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx78.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi78.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.79 && H < 0.80)
    {//printf("H =0.79 \n");
        fp2 = fopen("../share/Gain/Gd79.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx79.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi79.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.80 && H < 0.81)
    {//printf("H =0.80 \n");
        fp2 = fopen("../share/Gain/MPCGd80.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/MPCGx80.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/MPCGi80.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.81 && H < 0.82)
    {//printf("H =0.81 \n");
        fp2 = fopen("../share/Gain/Gd81.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx81.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi81.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.82 && H < 0.83)
    {//printf("H =0.82 \n");
        fp2 = fopen("../share/Gain/Gd82.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx82.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi82.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.83 && H < 0.84)
    {//printf("H =0.83 \n");
        fp2 = fopen("../share/Gain/Gd83.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx83.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi83.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.84 && H < 0.85)
    {//printf("H =0.84 \n");
        fp2 = fopen("../share/Gain/Gd84.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx84.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi84.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    //else if(H >= 0.85 && H < 0.86)
    else if(H >= 0.85)
    {//printf("H =0.85 \n");
        fp2 = fopen("../share/Gain/Gd85.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx85.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi85.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }

}


void YujinTest_Interpolation()
{
    static double time = 0.0;
    static double t = 0.0;

    if(time > 20.0)
        return;

    if(FLAG_Interpolation_changed == true)
    {
        pi = pnow;
        pf = in_pf;
        vi = vnow;
        vf = 0.;
        ai = anow;
        af = 0.;

        t = 0.;
        T = in_t;
        FLAG_Interpolation_changed = false;
    }

    if(t > T)
        return;

    a0 = pi;
    a1 = vi;
    a2 = ai/2.;
    a3 = (20*pf - 20*pi - (8*vf + 12*vi)*T - (3*ai - af)*T*T)/(2*T*T*T);
    a4 = (30*pi - 30*pf + (14*vf + 16*vi)*T + (3*ai - 2*af)*T*T)/(2*T*T*T*T);
    a5 = (12*pf - 12*pi - (6*vf + 6*vi)*T - (ai - af)*T*T)/(2*T*T*T*T*T);

    pnow = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
    vnow = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
    anow = 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;


    t = t+0.005;
    time = time+0.005;

    printf("[Time = %f] pf = %f | pnow = %f, vnow = %f, anow = %f\n",time,pf,pnow,vnow,anow);

}

