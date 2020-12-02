#include "testdialog.h"
#include "ui_testdialog.h"

#include "CommonHeader.h"
#include "BasicFiles/PODOALDialog.h"
#include "../../share/Headers/commandlist.h"

#include "../../../share/Headers/ik_math4.h"



TestDialog::TestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TestDialog)
{
    ui->setupUi(this);
    AlnumWalkReady      = PODOALDialog::GetALNumFromFileName("WalkReady");
    ALNum_ApproachBox   = PODOALDialog::GetALNumFromFileName("ApproachBox");
    ALNum_LiftBox       = PODOALDialog::GetALNumFromFileName("LiftBox");
    ALNum_WBWalk        = PODOALDialog::GetALNumFromFileName("WBWalk");
    ALNUM_gogoLiftBox   = PODOALDialog::GetALNumFromFileName("gogoLiftBox");
    AlnumHBWalking      = PODOALDialog::GetALNumFromFileName("HBWalking");
    AlnumHBWalking_COM  = PODOALDialog::GetALNumFromFileName("HBWalking_COM");
    AlnumManualMove     = PODOALDialog::GetALNumFromALName("ManualMove");


    ui->JOY_TABLE_INFO_RIGHT->setColumnWidth(0, 60);

    // Joy Stick variables
    joystick = new RBJoystick();
    joystick->ConnectJoy();
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50


    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSensors()));

}

TestDialog::~TestDialog()
{
    delete ui;
}

void TestDialog::on_PB_WKRD_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;

    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Forward_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum    = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle  = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_FORWARD_WALKING;

    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_LiftBox_clicked()
{
    printf("LiftBox_Sit_Down\n");
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_PutBox_clicked()
{
    printf("LiftBox_Hold_BOX\n");
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_PUTBOX;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_Back_LiftBox_clicked()
{
    printf("LIFTBOX_BACK_LIFTBOX\n");
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_LIFTBOX;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Back_PutBox_clicked()
{
    printf("BACK_PUTBOX\n");
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_PUTBOX;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Back_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Sitdown_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_Front_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Standup_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_HoldBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 4;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_pushButton_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX_PARTS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_PutOnTable_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX_PARTS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_RB_ReleaseBox_clicked()
{
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 8;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_StandUP2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX_PARTS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Open_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
//    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX_PARTS;
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
//    pLAN->SendCommand(cmd);

    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_GRASP_CART;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_ApproachCart_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_GO_CART;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Close_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_RELEASE_CART;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_PushIn_clicked()
{
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 6;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_HandsDown_clicked()
{
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 7;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_pushButton_clicked()
{
    USER_COMMAND cmd;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 9;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_AddMass_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_SubMass_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_ArmsOpen_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 5;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_getPHand_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 99;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}


void TestDialog::on_RB_TrunToFront_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 10;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_pushButton_3_clicked()
{

}

void TestDialog::on_PB_ALLinOne_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_LIFTPUT;
//    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}


void TestDialog::on_PB_Move_LiftBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_HoldPose_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_LB_WalkReady_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 14;

    pLAN->SendCommand(cmd);
}


void TestDialog::on_PB_PositionWalking_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND        = 118;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0]    = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();

    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_STOP;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Resume_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_STOP;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_G_open_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRIPPER;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_G_close_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRIPPER;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_G_close_all_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRIPPER;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}


void TestDialog::on_PB_PushIn2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 11;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_HandsBack2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 12;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_prePushIn2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 13;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_UBtest_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 4;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Approach_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMfollow_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMfollowDone_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_SaveStart_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; // Save Start
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_SaveDone_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // Save Done
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_LB_WalkReady_2_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 14;

    pLAN->SendCommand(cmd);

}

void TestDialog::on_JOY_BTN_START_clicked()
{
    // Joy Stick Start
    JoyModeFlag=true;
    ui->JOY_BTN_START->setDisabled(true);
    ui->JOY_BTN_STOP->setDisabled(false);
    ui->MANUAL_BTN_JOY_WALKING_ON->setDisabled(false);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_JOYSTICK_MODE;//ManualMove_AL_JOYSTICK_MODE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // On
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_JOY_BTN_STOP_clicked()
{
    // Joy Stick Stop
    JoyModeFlag=false;
    ui->JOY_BTN_START->setDisabled(false);
    ui->JOY_BTN_STOP->setDisabled(true);
    ui->MANUAL_BTN_JOY_WALKING_ON->setDisabled(true);
    ui->MANUAL_BTN_JOY_WALKING_OFF->setDisabled(true);
//    ui->JOY_TAB_WHEELSTART->setDisabled(true);
//    ui->JOY_TAB_WHEELSTOP->setDisabled(true);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_JOYSTICK_MODE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Off
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_MANUAL_BTN_JOY_WALKING_ON_clicked()
{
    JoysticWalkingFlag = true;
    printf("Joy Stick Walking Mode On !!\n");

    ui->MANUAL_BTN_JOY_WALKING_ON->setDisabled(true);
    ui->MANUAL_BTN_JOY_WALKING_OFF->setDisabled(false);

    ui->LE_JOYSTICK_WALKING_STATUS->setStyleSheet("background-color: green");
}

void TestDialog::on_MANUAL_BTN_JOY_WALKING_OFF_clicked()
{
    JoysticWalkingFlag = false;
    printf("Joy Stick Walking Mode Off !!\n");

    ui->MANUAL_BTN_JOY_WALKING_ON->setDisabled(false);
    ui->MANUAL_BTN_JOY_WALKING_OFF->setDisabled(true);

    ui->LE_JOYSTICK_WALKING_STATUS->setStyleSheet("background-color: red");

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[15]   = 0; // Joystick walking Flag off
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = des_t_step;
    cmd.COMMAND_DATA.USER_COMMAND         = HBWalking_JoyStick_Walk_Stop;
    cmd.COMMAND_TARGET                    = AlnumHBWalking;
    pLAN->SendCommand(cmd);


    USER_COMMAND cmd2;
    cmd2.COMMAND_DATA.USER_PARA_CHAR[15]   = 0; // Joystick walking Flag off
    cmd2.COMMAND_DATA.USER_PARA_DOUBLE[15] = des_t_step;
    cmd2.COMMAND_DATA.USER_COMMAND         = HBWalking_JoyStick_Walk_Stop;
    cmd2.COMMAND_TARGET                    = AlnumHBWalking_COM;
    pLAN->SendCommand(cmd2);

    HBWK_com_flag = false;

}

//JOYSTICK FUNCTIONS
void TestDialog::DisplayUpdate()
{
    // JoyStick Input
    if(JoyModeFlag==true)
        GetJoystickData();
    else
        InitJoystickData();

    //// ==================================Joy Stick Walking ======================

    if(JoysticWalkingFlag == true){
        if(JOY_A == 1) JOY_A_counter++;
        else{
            JOY_A_counter = 0;
            JOY_A_flag = false;
        }

        if(JOY_B == 1) JOY_B_counter++;
        else{
            JOY_B_counter = 0;
            JOY_B_flag = false;
        }

        if(JOY_X == 1) JOY_X_counter++;
        else{
            JOY_X_counter = 0;
            JOY_X_flag = false;
        }

        if(JOY_AROW_UD == -32767) JOY_ARROW_DN_counter++;
        else{
            JOY_ARROW_DN_counter = 0;
            JOY_ARROW_DN_flag = false;
        }

        if(JOY_AROW_UD == 32767) JOY_ARROW_UP_counter++;
        else{
            JOY_ARROW_UP_counter = 0;
            JOY_ARROW_UP_flag = false;
        }

        if(JOY_Y == 1) JOY_Y_counter++;
        else{
            JOY_Y_counter = 0;
            JOY_Y_flag = false;
        }

        if(JOY_START == 1) JOY_START_counter++;
        else{
            JOY_START_counter = 0;
            JOY_START_flag = false;
        }

        if(JOY_A_counter > 10 && JOY_A_flag == false){
            // Walking Start
            JOY_A_flag = true;
            des_t_step = 0.9;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_COMMAND = HBWalking_PrevWalk;
            cmd.COMMAND_DATA.USER_PARA_INT[0]    = 10;
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0.8; //step time??? //2.5;//new change
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = 0.0;
            cmd.COMMAND_DATA.USER_PARA_CHAR[15]  = 10;  // Joystick walking Flag on
            cmd.COMMAND_TARGET = AlnumHBWalking;
            pLAN->SendCommand(cmd);
        }
        if(JOY_B_counter > 10 && JOY_B_flag == false){ // Walking terminate
            // Walking stop
            JOY_B_flag = true;
            on_MANUAL_BTN_JOY_WALKING_OFF_clicked();
        }
        if(JOY_X_counter > 10 && JOY_X_flag == false){  // WalkReady button
//            // WalkReady pos
//            JOY_X_flag = true;
//            USER_COMMAND cmd;
//            cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
//            cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
//            cmd.COMMAND_TARGET = AlnumWalkReady;
//            pLAN->SendCommand(cmd);

            // Walking Start
            HBWK_com_flag = true;
            JOY_X_flag = true;
            des_t_step = 0.9;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_COMMAND = HBWalking_PrevWalk;
            cmd.COMMAND_DATA.USER_PARA_INT[0]    = 10;
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0.8; //step time??? //2.5;//new change
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = 0.0;
            cmd.COMMAND_DATA.USER_PARA_CHAR[15]  = 10;  // Joystick walking Flag on
            cmd.COMMAND_TARGET = AlnumHBWalking_COM;
            pLAN->SendCommand(cmd);
        }

        if(JOY_Y_counter > 10 && JOY_Y_flag == false){  // IMU nulling

            JOY_Y_flag = true;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            cmd.COMMAND_TARGET = RBCORE_PODO_NO;
            pLAN->SendCommand(cmd);
            printf("IMU null joystick");

            usleep(5000*1000);
        }

        if(JOY_ARROW_DN_counter > 5 && JOY_ARROW_DN_flag == false){
            JOY_ARROW_DN_flag = true;
            des_t_step = des_t_step - 0.02;
            if(des_t_step <= 0.6) des_t_step = 0.6;
            if(des_t_step >= 0.9) des_t_step = 0.9;
        }

        if(JOY_ARROW_UP_counter > 3 && JOY_ARROW_UP_flag == false){
            JOY_ARROW_UP_flag = true;
            des_t_step = des_t_step + 0.02;
            if(des_t_step <= 0.6) des_t_step = 0.6;
            if(des_t_step >= 0.9) des_t_step = 0.9;
        }


        if(JOY_START_counter > 5 && JOY_START_flag == false){
            JOY_START_flag = true;
            // reset to acc angle
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            cmd.COMMAND_TARGET = RBCORE_PODO_NO;
            pLAN->SendCommand(cmd);

            printf("IMU reset joystick");

            usleep(500*1000);
        }

        //ORIGINAL
//        if(JOY_A == 0 && JOY_B == 0){
//            USER_COMMAND cmd;
//            cmd.COMMAND_DATA.USER_PARA_CHAR[0]  =   (char)JOY_X;
//            cmd.COMMAND_DATA.USER_PARA_CHAR[1]  =   (char)JOY_Y;
//            cmd.COMMAND_DATA.USER_PARA_CHAR[2]  =   (char)JOY_A;
//            cmd.COMMAND_DATA.USER_PARA_CHAR[3]  =   (char)JOY_B;
//            cmd.COMMAND_DATA.USER_PARA_INT[0]   =   (int)JOY_RJOG_RL;
//            cmd.COMMAND_DATA.USER_PARA_INT[1]   =   (int)JOY_RJOG_UD;
//            cmd.COMMAND_DATA.USER_PARA_INT[2]   =   (int)JOY_LJOG_RL;
//            cmd.COMMAND_DATA.USER_PARA_INT[3]   =   (int)JOY_LJOG_UD;
//            cmd.COMMAND_DATA.USER_PARA_INT[4]   =   (int)JOY_RB;
//            cmd.COMMAND_DATA.USER_PARA_INT[5]   =   (int)JOY_LB;

//            cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = (double)des_t_step;

//            cmd.COMMAND_DATA.USER_PARA_CHAR[15] = 0; // Joystick walking Flag off

//            cmd.COMMAND_DATA.USER_COMMAND = HBWalking_NO_ACT;
//            cmd.COMMAND_TARGET = AlnumHBWalking;
//            pLAN->SendCommand(cmd);
//        }

        //NEW
        if(JOY_A == 0 && JOY_B == 0 && JOY_X == 0){

            if(!HBWK_com_flag)
            {
                USER_COMMAND cmd;
                cmd.COMMAND_DATA.USER_PARA_CHAR[0]  =   (char)JOY_X;
                cmd.COMMAND_DATA.USER_PARA_CHAR[1]  =   (char)JOY_Y;
                cmd.COMMAND_DATA.USER_PARA_CHAR[2]  =   (char)JOY_A;
                cmd.COMMAND_DATA.USER_PARA_CHAR[3]  =   (char)JOY_B;
                cmd.COMMAND_DATA.USER_PARA_INT[0]   =   (int)JOY_RJOG_RL;
                cmd.COMMAND_DATA.USER_PARA_INT[1]   =   (int)JOY_RJOG_UD;
                cmd.COMMAND_DATA.USER_PARA_INT[2]   =   (int)JOY_LJOG_RL;
                cmd.COMMAND_DATA.USER_PARA_INT[3]   =   (int)JOY_LJOG_UD;
                cmd.COMMAND_DATA.USER_PARA_INT[4]   =   (int)JOY_RB;
                cmd.COMMAND_DATA.USER_PARA_INT[5]   =   (int)JOY_LB;

                cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = (double)des_t_step;

                cmd.COMMAND_DATA.USER_PARA_CHAR[15] = 0; // Joystick walking Flag off

                cmd.COMMAND_DATA.USER_COMMAND = HBWalking_NO_ACT;
                cmd.COMMAND_TARGET = AlnumHBWalking;
                pLAN->SendCommand(cmd);
            }
            else if(HBWK_com_flag)
            {
                USER_COMMAND cmd2;
                cmd2.COMMAND_DATA.USER_PARA_CHAR[0]  =   (char)JOY_X;
                cmd2.COMMAND_DATA.USER_PARA_CHAR[1]  =   (char)JOY_Y;
                cmd2.COMMAND_DATA.USER_PARA_CHAR[2]  =   (char)JOY_A;
                cmd2.COMMAND_DATA.USER_PARA_CHAR[3]  =   (char)JOY_B;
                cmd2.COMMAND_DATA.USER_PARA_INT[0]   =   (int)JOY_RJOG_RL;
                cmd2.COMMAND_DATA.USER_PARA_INT[1]   =   (int)JOY_RJOG_UD;
                cmd2.COMMAND_DATA.USER_PARA_INT[2]   =   (int)JOY_LJOG_RL;
                cmd2.COMMAND_DATA.USER_PARA_INT[3]   =   (int)JOY_LJOG_UD;
                cmd2.COMMAND_DATA.USER_PARA_INT[4]   =   (int)JOY_RB;
                cmd2.COMMAND_DATA.USER_PARA_INT[5]   =   (int)JOY_LB;

                cmd2.COMMAND_DATA.USER_PARA_DOUBLE[15] = (double)des_t_step;

                cmd2.COMMAND_DATA.USER_PARA_CHAR[15] = 0; // Joystick walking Flag off

                cmd2.COMMAND_DATA.USER_COMMAND = HBWalking_NO_ACT;
                cmd2.COMMAND_TARGET = AlnumHBWalking_COM;
                pLAN->SendCommand(cmd2);

            }
        }

    }

    // If Manual Hand Mode
    if(ManualModeFlag == true){
        if(ManualModeCount<10){
            ManualModeCount++;
        }else{
        }
        if(ManualModeCount>1000000)
            ManualModeCount=0;
    }
    else{
        ;
    }
    // Data Show
    QString str;
    QTableWidgetItem *tempItem;
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_A));
    ui->JOY_TABLE_INFO_RIGHT->setItem(0,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_B));
    ui->JOY_TABLE_INFO_RIGHT->setItem(1,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LJOG_UD));
    ui->JOY_TABLE_INFO_RIGHT->setItem(2,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RJOG_RL));
    ui->JOY_TABLE_INFO_RIGHT->setItem(3,0,tempItem);
}
void TestDialog::GetJoystickData(void)
{
    if(joystick->isConnected() == false){
        printf("JoyStick connection failure...!!!\n");
        return;
    }

    // Button Data
    JOY_X = joystick->JoyButton[2];
    JOY_A = joystick->JoyButton[0];
    JOY_B = joystick->JoyButton[1];
    JOY_Y = joystick->JoyButton[3];
    JOY_LB = joystick->JoyButton[4];
    JOY_RB = joystick->JoyButton[5];
    JOY_LT = joystick->JoyAxis[2];
    JOY_RT = joystick->JoyAxis[5];


    if((int)(JOY_LT) == -1){
        JOY_LT = 1;
    }
    else{
        JOY_LT = 0;
    }
    if((int)(JOY_RT) == -1){
        JOY_RT = 1;
    }
    else{
        JOY_RT = 0;
    }

    JOY_BACK = joystick->JoyButton[6];
    JOY_START = joystick->JoyButton[7];
    JOY_LJOG_PUSH = joystick->JoyButton[9];
    JOY_RJOG_PUSH = joystick->JoyButton[10];


    // AXIS Data
    JOY_LJOG_RL =  joystick->JoyAxis[0];
    JOY_LJOG_UD = -joystick->JoyAxis[1];
    JOY_RJOG_RL =  joystick->JoyAxis[3];
    JOY_RJOG_UD = -joystick->JoyAxis[4];


    // Hyo in
    double th_hyo = 3000;
    if((JOY_LJOG_RL < th_hyo) && (JOY_LJOG_RL > -th_hyo)) JOY_LJOG_RL = 0;
    if((JOY_LJOG_UD < th_hyo) && (JOY_LJOG_UD > -th_hyo)) JOY_LJOG_UD = 0;
    if((JOY_RJOG_RL < th_hyo) && (JOY_RJOG_RL > -th_hyo)) JOY_RJOG_RL = 0;
    if((JOY_RJOG_UD < th_hyo) && (JOY_RJOG_UD > -th_hyo)) JOY_RJOG_UD = 0;

    JOY_AROW_RL = joystick->JoyAxis[6];
    JOY_AROW_UD = -joystick->JoyAxis[7];

}
void TestDialog::InitJoystickData(void)
{
    JOY_LT=JOY_LB=0;
    JOY_LJOG_RL=JOY_LJOG_UD=0;
    JOY_AROW_RL=JOY_AROW_UD=0;
    JOY_LJOG_PUSH=0;

    JOY_RT=JOY_RB=0;
    JOY_RJOG_RL=JOY_RJOG_UD=0;
    JOY_A=JOY_B=JOY_X=JOY_Y=0;
    JOY_RJOG_PUSH=0;

    JOY_BACK=JOY_START=0;
}
//END JOYSTIC FUNCTIONS


//JOYSTICK FUNCTIONS_original CODE
/*
void TestDialog::DisplayUpdate()
{
    // JoyStick Input
    if(JoyModeFlag==true)
        GetJoystickData();
    else
        InitJoystickData();

    //// ==================================Joy Stick Walking ======================

    if(JoysticWalkingFlag == true){
        if(JOY_A == 1) JOY_A_counter++;
        else{
            JOY_A_counter = 0;
            JOY_A_flag = false;
        }

        if(JOY_B == 1) JOY_B_counter++;
        else{
            JOY_B_counter = 0;
            JOY_B_flag = false;
        }

        if(JOY_X == 1) JOY_X_counter++;
        else{
            JOY_X_counter = 0;
            JOY_X_flag = false;
        }

        if(JOY_AROW_UD == -32767) JOY_ARROW_DN_counter++;
        else{
            JOY_ARROW_DN_counter = 0;
            JOY_ARROW_DN_flag = false;
        }

        if(JOY_AROW_UD == 32767) JOY_ARROW_UP_counter++;
        else{
            JOY_ARROW_UP_counter = 0;
            JOY_ARROW_UP_flag = false;
        }

        if(JOY_Y == 1) JOY_Y_counter++;
        else{
            JOY_Y_counter = 0;
            JOY_Y_flag = false;
        }

        if(JOY_START == 1) JOY_START_counter++;
        else{
            JOY_START_counter = 0;
            JOY_START_flag = false;
        }

        if(JOY_A_counter > 10 && JOY_A_flag == false){
            // Walking Start
            JOY_A_flag = true;
            des_t_step = 0.9;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_COMMAND = HBWalking_PrevWalk;
            cmd.COMMAND_DATA.USER_PARA_INT[0]    = 10;
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0.8; //step time??? //2.5;//new change
            cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = 0.0;
            cmd.COMMAND_DATA.USER_PARA_CHAR[15]  = 10;  // Joystick walking Flag on
            cmd.COMMAND_TARGET = AlnumHBWalking;
            pLAN->SendCommand(cmd);
        }
        if(JOY_B_counter > 10 && JOY_B_flag == false){ // Walking terminate
            // Walking stop
            JOY_B_flag = true;
            on_MANUAL_BTN_JOY_WALKING_OFF_clicked();
        }
        if(JOY_X_counter > 3 && JOY_X_flag == false){  // WalkReady button
            // WalkReady pos
            JOY_X_flag = true;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
            cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
            cmd.COMMAND_TARGET = AlnumWalkReady;
            pLAN->SendCommand(cmd);
        }

        if(JOY_ARROW_DN_counter > 5 && JOY_ARROW_DN_flag == false){
            JOY_ARROW_DN_flag = true;
            des_t_step = des_t_step - 0.02;
            if(des_t_step <= 0.6) des_t_step = 0.6;
            if(des_t_step >= 0.9) des_t_step = 0.9;
        }

        if(JOY_ARROW_UP_counter > 3 && JOY_ARROW_UP_flag == false){
            JOY_ARROW_UP_flag = true;
            des_t_step = des_t_step + 0.02;
            if(des_t_step <= 0.6) des_t_step = 0.6;
            if(des_t_step >= 0.9) des_t_step = 0.9;
        }

        if(JOY_Y_counter > 5 && JOY_Y_flag == false){  // IMU nulling
            JOY_Y_flag = true;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            cmd.COMMAND_TARGET = RBCORE_PODO_NO;
            pLAN->SendCommand(cmd);
            printf("IMU null joystick");

            usleep(5000*1000);
        }

        if(JOY_START_counter > 5 && JOY_START_flag == false){
            JOY_START_flag = true;
            // reset to acc angle
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
            cmd.COMMAND_TARGET = RBCORE_PODO_NO;
            pLAN->SendCommand(cmd);

            printf("IMU reset joystick");

            usleep(500*1000);
        }

        if(JOY_A == 0 && JOY_B == 0){
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0]  =   (char)JOY_X;
            cmd.COMMAND_DATA.USER_PARA_CHAR[1]  =   (char)JOY_Y;
            cmd.COMMAND_DATA.USER_PARA_CHAR[2]  =   (char)JOY_A;
            cmd.COMMAND_DATA.USER_PARA_CHAR[3]  =   (char)JOY_B;
            cmd.COMMAND_DATA.USER_PARA_INT[0]   =   (int)JOY_RJOG_RL;
            cmd.COMMAND_DATA.USER_PARA_INT[1]   =   (int)JOY_RJOG_UD;
            cmd.COMMAND_DATA.USER_PARA_INT[2]   =   (int)JOY_LJOG_RL;
            cmd.COMMAND_DATA.USER_PARA_INT[3]   =   (int)JOY_LJOG_UD;
            cmd.COMMAND_DATA.USER_PARA_INT[4]   =   (int)JOY_RB;
            cmd.COMMAND_DATA.USER_PARA_INT[5]   =   (int)JOY_LB;

            cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = (double)des_t_step;

            cmd.COMMAND_DATA.USER_PARA_CHAR[15] = 0; // Joystick walking Flag off

            cmd.COMMAND_DATA.USER_COMMAND = HBWalking_NO_ACT;
            cmd.COMMAND_TARGET = AlnumHBWalking;
            pLAN->SendCommand(cmd);
        }

    }

    // If Manual Hand Mode
    if(ManualModeFlag == true){
        if(ManualModeCount<10){
            ManualModeCount++;
        }else{
        }
        if(ManualModeCount>1000000)
            ManualModeCount=0;
    }
    else{
        ;
    }
    // Data Show
    QString str;
    QTableWidgetItem *tempItem;
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_A));
    ui->JOY_TABLE_INFO_RIGHT->setItem(0,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_B));
    ui->JOY_TABLE_INFO_RIGHT->setItem(1,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LJOG_UD));
    ui->JOY_TABLE_INFO_RIGHT->setItem(2,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RJOG_RL));
    ui->JOY_TABLE_INFO_RIGHT->setItem(3,0,tempItem);
}
void TestDialog::GetJoystickData(void)
{
    if(joystick->isConnected() == false){
        printf("JoyStick connection failure...!!!\n");
        return;
    }

    // Button Data
    JOY_X = joystick->JoyButton[2];
    JOY_A = joystick->JoyButton[0];
    JOY_B = joystick->JoyButton[1];
    JOY_Y = joystick->JoyButton[3];
    JOY_LB = joystick->JoyButton[4];
    JOY_RB = joystick->JoyButton[5];
    JOY_LT = joystick->JoyAxis[2];
    JOY_RT = joystick->JoyAxis[5];


    if((int)(JOY_LT) == -1){
        JOY_LT = 1;
    }
    else{
        JOY_LT = 0;
    }
    if((int)(JOY_RT) == -1){
        JOY_RT = 1;
    }
    else{
        JOY_RT = 0;
    }

    JOY_BACK = joystick->JoyButton[6];
    JOY_START = joystick->JoyButton[7];
    JOY_LJOG_PUSH = joystick->JoyButton[9];
    JOY_RJOG_PUSH = joystick->JoyButton[10];


    // AXIS Data
    JOY_LJOG_RL = joystick->JoyAxis[0];
    JOY_LJOG_UD = -joystick->JoyAxis[1];
    JOY_RJOG_RL = joystick->JoyAxis[3];
    JOY_RJOG_UD = -joystick->JoyAxis[4];


    // Hyo in
    double th_hyo = 3000;
    if((JOY_LJOG_RL < th_hyo) && (JOY_LJOG_RL > -th_hyo)) JOY_LJOG_RL = 0;
    if((JOY_LJOG_UD < th_hyo) && (JOY_LJOG_UD > -th_hyo)) JOY_LJOG_UD = 0;
    if((JOY_RJOG_RL < th_hyo) && (JOY_RJOG_RL > -th_hyo)) JOY_RJOG_RL = 0;
    if((JOY_RJOG_UD < th_hyo) && (JOY_RJOG_UD > -th_hyo)) JOY_RJOG_UD = 0;

    JOY_AROW_RL = joystick->JoyAxis[6];
    JOY_AROW_UD = -joystick->JoyAxis[7];

}
void TestDialog::InitJoystickData(void)
{
    JOY_LT=JOY_LB=0;
    JOY_LJOG_RL=JOY_LJOG_UD=0;
    JOY_AROW_RL=JOY_AROW_UD=0;
    JOY_LJOG_PUSH=0;

    JOY_RT=JOY_RB=0;
    JOY_RJOG_RL=JOY_RJOG_UD=0;
    JOY_A=JOY_B=JOY_X=JOY_Y=0;
    JOY_RJOG_PUSH=0;

    JOY_BACK=JOY_START=0;
}
*/
//END JOYSTIC FUNCTIONS


void TestDialog::on_PB_SaveStart_HB_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    cmd.COMMAND_DATA.USER_COMMAND = 101;    //HBWalking_DATA_SAVE
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; // Save Start
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_SaveDone_HB_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    cmd.COMMAND_DATA.USER_COMMAND = 101;    //HBWalking_DATA_SAVE
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // Save Done
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_turnRight_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_RIGHT;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_OpenArms_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_OPENARMS;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_Approach_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_APPROACH;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_HoldBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_HOLD;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_MB_LiftBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_LIFT;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_TurnLeft_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_LEFT;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_PutBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_PUTIN;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_Release_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_RELEASE;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_HandsBack_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_HANDSBACK;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_WKRDish_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_WALKREADYISH;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_F_HoldReady_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_HOLDPOSE;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_F_Hold_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_FRONT_HOLD;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_F_Release_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_FRONT_RELEASE;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Pull_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_FRONT_PULL;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Push_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_FRONT_PUSH;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMBack_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX_FRONT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_FRONT_COM_BACK;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMBackWalkReady_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND        = 118;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0]    = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1]    = 0;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();

    cmd.COMMAND_TARGET = AlnumHBWalking_COM;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMBackWalking_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND        = 118;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0]    = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1]    = 1;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();

    cmd.COMMAND_TARGET = AlnumHBWalking_COM;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_TurnLeft_COM_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_LEFT_COMBACK;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_turnRight_WBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_RIGHT_WBOX;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_resetParam_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_RESET_PARAM;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_getPos_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_GET_POSE;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_COMx5forward_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_COM_5FORWARD;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);


}

void TestDialog::on_PB_COMx5Backward_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_COM_5BACKWARD;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::on_PB_MoveStart_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_MOVESTART;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);


}

void TestDialog::on_PB_COMxBackWalking_clicked()
{

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND        = 118;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0]    = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1]    = 1;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();

    cmd.COMMAND_TARGET = AlnumHBWalking_COM;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_WalkReady_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;

    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COMBack5_abs_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_COM_5BACKWARD_ABS;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_liftBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_LIFTBOX;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_putBox_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_PUTBOX;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_WKRD_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;

    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_MB_preTurn_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_MOVEBOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = MOVE_PRETURN;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_holdCart_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_ReleaseCart_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 4;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_Cart_walkReadyish_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_UB;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 5;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COM_Forward_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 14;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_COM_m->text().toDouble();
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;

    total_COM_shift += ui->LE_COM_m->text().toDouble();

    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COM_Backward_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 15;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_COM_m->text().toDouble();;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;

    total_COM_shift -= ui->LE_COM_m->text().toDouble();

    pLAN->SendCommand(cmd);
}

//SENSOR DISPLAY
void TestDialog::UpdateSensors(){
    QString str;

    // FT ======
    ui->LE_RAFT_Mx->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].Mx));
    ui->LE_RAFT_My->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].My));
    ui->LE_RAFT_Mz->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].Mz));
    ui->LE_RAFT_Fx->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].Fx));
    ui->LE_RAFT_Fy->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].Fy));
    ui->LE_RAFT_Fz->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[0].Fz));

    ui->LE_LAFT_Mx->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].Mx));
    ui->LE_LAFT_My->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].My));
    ui->LE_LAFT_Mz->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].Mz));
    ui->LE_LAFT_Fx->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].Fx));
    ui->LE_LAFT_Fy->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].Fy));
    ui->LE_LAFT_Fz->setText(str.sprintf("%.2f", PODO_DATA.CoreData.FT[1].Fz));

    get_zmp();

    ui->LE_ZMPx->setText(str.sprintf("%.4f", zmp_calc[0]));
    ui->LE_ZMPy->setText(str.sprintf("%.4f", zmp_calc[1]));
    ui->LE_ZMPz->setText(str.sprintf("%.4f", zmp_calc[2]));

    ui->LE_COMx->setText(str.sprintf("%.4f", PODO_DATA.CoreData.COM_SH_cal[0]));
    ui->LE_COMy->setText(str.sprintf("%.4f", PODO_DATA.CoreData.COM_SH_cal[1]));

    ui->LE_COM_shift_total->setText(str.sprintf("%.3f", total_COM_shift));

}

void TestDialog::on_PB_getCOM_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 5;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_getZMP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 6;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);

}

void TestDialog::get_zmp()
{
    // ZMP 읽기 //
    double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
    double pCenter[3],qCenter[4],qCenter_bar[4];

    PODO_DATA.UserM2G.pRF[0] = 0.0;
    PODO_DATA.UserM2G.pRF[1] = -0.105;
    PODO_DATA.UserM2G.pRF[2] = 0.0;
    PODO_DATA.UserM2G.pLF[1] = 0.105;
    PODO_DATA.UserM2G.qLF[0] = 1;
    PODO_DATA.UserM2G.qRF[0] = 1;

    // Foot Center in Global Coord.
    pCenter[0] = (PODO_DATA.UserM2G.pRF[0] + PODO_DATA.UserM2G.pLF[0])/2.;
    pCenter[1] = (PODO_DATA.UserM2G.pRF[1] + PODO_DATA.UserM2G.pLF[1])/2.;
    pCenter[2] = (PODO_DATA.UserM2G.pRF[2] + PODO_DATA.UserM2G.pLF[2])/2.;

    if(PODO_DATA.CoreData.FT[0].Fz + PODO_DATA.CoreData.FT[1].Fz > 50.)
    {
        M_LF[0] =  PODO_DATA.CoreData.FT[1].Mx;
        M_LF[1] =  PODO_DATA.CoreData.FT[1].My;
        M_LF[2] =  PODO_DATA.CoreData.FT[1].Mz;

        QTtransform(PODO_DATA.UserM2G.qLF,M_LF,M_LF_Global);

        M_RF[0] =  PODO_DATA.CoreData.FT[0].Mx;
        M_RF[1] =  PODO_DATA.CoreData.FT[0].My;
        M_RF[2] =  PODO_DATA.CoreData.FT[0].Mz;

        QTtransform(PODO_DATA.UserM2G.qRF,M_RF,M_RF_Global);

        F_LF[0] = PODO_DATA.CoreData.FT[1].Fx;
        F_LF[1] = PODO_DATA.CoreData.FT[1].Fy;
        F_LF[2] = PODO_DATA.CoreData.FT[1].Fz;

        QTtransform(PODO_DATA.UserM2G.qLF,F_LF,F_LF_Global);

        F_RF[0] = PODO_DATA.CoreData.FT[0].Fx;
        F_RF[1] = PODO_DATA.CoreData.FT[0].Fy;
        F_RF[2] = PODO_DATA.CoreData.FT[0].Fz;

        QTtransform(PODO_DATA.UserM2G.qRF,F_RF,F_RF_Global);

        double temp1[3],temp2[3],temp3[3],temp4[3];

        diff_vv(PODO_DATA.UserM2G.pRF,3,pCenter,temp1); // (despRF - pCenter)
        diff_vv(PODO_DATA.UserM2G.pLF,3,pCenter,temp2); // (despLF - pCenter)

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

        QTtransform(qCenter_bar, temp1, zmp_local_calc); // qCenter_bar*(zmp-pCenter)

        //ZMP on UserSharedMemory
        PODO_DATA.UserM2G.ZMP_calc[0] = zmp_calc[0];
        PODO_DATA.UserM2G.ZMP_calc[1] = zmp_calc[1];
        PODO_DATA.UserM2G.ZMP_calc[2] = zmp_calc[2];


    }
}

void TestDialog::on_PB_HomePos_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_HOMEPOS;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_COM_RESET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 7;
    cmd.COMMAND_TARGET = ALNUM_gogoLiftBox;
    pLAN->SendCommand(cmd);
    total_COM_shift = 0.0;
}
