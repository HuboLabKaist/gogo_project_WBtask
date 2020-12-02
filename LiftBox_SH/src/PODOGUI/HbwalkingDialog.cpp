#include "HbwalkingDialog.h"
#include "ui_HbwalkingDialog.h"
#include "BasicFiles/PODOALDialog.h"
#include "CommonHeader.h"
//#include "../../share/Headers/commandlist.h"
#include <iostream>

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY,
    WALKREADY_HIT_READY,
    WALKREADY_HIT_HIT,
    WALKREADY_HIT_RETURN,
    WALKREADY_HIT_INIT_POS,
};

enum HBWalking_COMMAND
{
    HBWalking_NO_ACT = 100,
    HBWalking_DATA_SAVE,
    HBWalking_TORQUE_TEST,
    HBWalking_ANKLE_TEST,
    HBWalking_STOP,
    HBWalking_WALK_TEST,
    HBWalking_SYSID_START,
    HBWalking_SYSID_STEP_INPUT_START,
    HBWalking_SYSID_STOP_SAVE,
    HBWalking_CONTROL_ON,
    HBWalking_OL_TORQUE_TUNING,
    HBWalking_ZERO_GAIN,
    HBWalking_INV_DYN_CONTROL,
    HBWalking_DYN_STANDING,
    HBWalking_POS_STANDING,
    HBWalking_DYN_WALKING,
    HBWalking_DYN_WALKING2,
    HBWalking_JUMP,
    HBWalking_PrevWalk,
    HBWalking_GetComHeight,
    HBWalking_Test,
    HBWalking_JoyStick_Walk_Stop,
    HBWalking_Ready_To_Walk,
    HBWalking_SE_Start_STOP,
    HBWalking_Clear_Controller,
};

HBWalkingDialog::HBWalkingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HBWalkingDialog)
{
    ui->setupUi(this);

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50

    AlnumHBWalking = PODOALDialog::GetALNumFromFileName("HBWalking");
    AlnumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
}

HBWalkingDialog::~HBWalkingDialog()
{
    delete ui;
}

void HBWalkingDialog::on_BT_WALK_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 11;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);

    usleep(500*1000);

    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_Clear_Controller;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_HOME_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_HOMEPOS;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_WALK_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_PrevWalk;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_TEST_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_STOP;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_Test;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_DATA_SAVE;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_READY_TO_WALK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_Ready_To_Walk;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_ONE_LEG_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 12; // right leg stance
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_ONE_LEG_READY_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 13; // left leg stance
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}
