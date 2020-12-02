#ifndef TESTDIALOG_H
#define TESTDIALOG_H

#include <QDialog>
#include "CommonHeader.h"
#include "JoyStick/joystickclass.h"


namespace Ui {
class TestDialog;
}

class TestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TestDialog(QWidget *parent = 0);
    ~TestDialog();

private slots:
    void on_PB_Forward_clicked();

    void on_PB_WKRD_clicked();

    void on_PB_LiftBox_clicked();

    void on_PB_PutBox_clicked();

    void on_PB_Back_LiftBox_clicked();

    void on_PB_Back_PutBox_clicked();

    void on_PB_Back_clicked();

    void on_PB_Sitdown_clicked();

    void on_PB_Front_clicked();

    void on_PB_Standup_clicked();

    void on_PB_HoldBox_clicked();

    void on_pushButton_2_clicked();

    void on_PB_PutOnTable_clicked();

    void on_RB_ReleaseBox_clicked();

    void on_PB_StandUP2_clicked();

    void on_PB_Open_clicked();

    void on_PB_ApproachCart_clicked();

    void on_PB_Close_clicked();

    void on_PB_PushIn_clicked();

    void on_PB_HandsDown_clicked();

    void on_pushButton_clicked();

    void on_PB_AddMass_clicked();

    void on_PB_SubMass_clicked();

    void on_PB_ArmsOpen_clicked();

    void on_PB_getPHand_clicked();


    void on_RB_TrunToFront_clicked();

    void on_pushButton_3_clicked();

    void on_PB_ALLinOne_clicked();

    void on_PB_Move_LiftBox_clicked();

    void on_PB_HoldPose_clicked();

    void on_PB_LB_WalkReady_clicked();


    void on_PB_PositionWalking_clicked();

    void on_PB_STOP_clicked();

    void on_PB_Resume_clicked();

    void on_PB_G_open_clicked();

    void on_PB_G_close_clicked();

    void on_PB_PushIn2_clicked();

    void on_PB_HandsBack2_clicked();

    void on_PB_prePushIn2_clicked();

    void on_PB_UBtest_clicked();

    void on_PB_Approach_clicked();

    void on_PB_COMfollow_clicked();

    void on_PB_COMfollowDone_clicked();

    void on_PB_G_close_all_clicked();

    void on_PB_SaveStart_clicked();

    void on_PB_SaveDone_clicked();

    void on_PB_LB_WalkReady_2_clicked();

    void on_JOY_BTN_START_clicked();

    void on_JOY_BTN_STOP_clicked();

    void on_MANUAL_BTN_JOY_WALKING_ON_clicked();

    void on_MANUAL_BTN_JOY_WALKING_OFF_clicked();

    void DisplayUpdate();

    void GetJoystickData(void);

    void InitJoystickData(void);

    void on_PB_SaveStart_HB_clicked();

    void on_PB_SaveDone_HB_clicked();

    void on_PB_MB_turnRight_clicked();

    void on_PB_MB_OpenArms_clicked();

    void on_PB_MB_Approach_clicked();

    void on_PB_MB_HoldBox_clicked();

    void on_PB_MB_LiftBox_clicked();

    void on_PB_MB_TurnLeft_clicked();

    void on_PB_MB_PutBox_clicked();

    void on_PB_MB_Release_clicked();

    void on_PB_MB_HandsBack_clicked();

    void on_PB_WKRDish_clicked();

    void on_PB_MB_F_Hold_clicked();

    void on_PB_MB_F_Release_clicked();

    void on_PB_MB_F_HoldReady_clicked();

    void on_PB_Pull_clicked();

    void on_PB_Push_clicked();

    void on_PB_COMBack_clicked();

    void on_PB_COMBackWalkReady_clicked();

    void on_PB_COMBackWalking_clicked();

    void on_PB_MB_TurnLeft_COM_clicked();

    void on_PB_MB_turnRight_WBox_clicked();

    void on_PB_resetParam_clicked();

    void on_PB_getPos_clicked();

    void on_PB_COMx5forward_clicked();

    void on_PB_COMx5Backward_clicked();

    void on_PB_MoveStart_clicked();

    void on_PB_COMxBackWalking_clicked();

    void on_PB_MB_WalkReady_clicked();

    void on_PB_COMBack5_abs_clicked();

    void on_PB_MB_liftBox_clicked();

    void on_PB_MB_putBox_clicked();

    void on_PB_MB_WKRD_clicked();

    void on_PB_MB_preTurn_clicked();

    void on_PB_holdCart_clicked();

    void on_PB_ReleaseCart_clicked();

    void on_PB_Cart_walkReadyish_clicked();

    void on_PB_COM_Forward_clicked();

    void on_PB_COM_Backward_clicked();

    void UpdateSensors();

    void on_PB_getCOM_clicked();

    void on_PB_getZMP_clicked();

    void get_zmp();

    void on_PB_HomePos_clicked();

    void on_PB_COM_RESET_clicked();

private:
    Ui::TestDialog *ui;
    LANDialog      *lanData; //Communication with Daemon

    RBJoystick     *joystick;
    QTimer		   *displayTimer;

    pUSER_SHM      userData;

    int    AlnumWalkReady;
    int    ALNum_ApproachBox;
    int    ALNum_LiftBox;
    int    ALNum_WBWalk;
    int    ALNUM_gogoLiftBox;
    int    AlnumHBWalking;
    int    AlnumHBWalking_COM;
    int    AlnumManualMove;

    char          JoyModeFlag=false;
    bool          ManualModeFlag = false;
    unsigned long ManualModeCount=0;

    char JoysticWalkingFlag=false;
    unsigned int JOY_A_counter = 0;
    unsigned int JOY_B_counter = 0;
    unsigned int JOY_X_counter = 0;
    //unsigned int JOY_BACK_counter = 0;
    unsigned int JOY_START_counter = 0;
    unsigned int JOY_ARROW_UP_counter = 0;
    unsigned int JOY_ARROW_DN_counter = 0;
    unsigned int JOY_Y_counter = 0;
    //unsigned int JOY_LB_counter = 0;
    //unsigned int JOY_RB_counter = 0;
    bool JOY_A_flag = false;
    bool JOY_B_flag = false;
    bool JOY_X_flag = false;
    bool JOY_Y_flag = false;
    //bool JOY_BACK_flag = false;
    bool JOY_START_flag = false;
    bool JOY_ARROW_UP_flag = false;
    bool JOY_ARROW_DN_flag = false;

    int  HBWK_com_flag = false;

    char   JOY_LT, JOY_LB;
    int    JOY_LJOG_RL, JOY_LJOG_UD;
    int    JOY_AROW_RL, JOY_AROW_UD;
    char   JOY_LJOG_PUSH;

    char   JOY_RT, JOY_RB;
    int    JOY_RJOG_RL, JOY_RJOG_UD;
    char   JOY_A, JOY_B, JOY_X, JOY_Y;
    char   JOY_RJOG_PUSH;

    char   JOY_BACK, JOY_START;
    double des_t_step;

    double zmp_calc[3] = {0., };
    double zmp_local_calc[3] = {0., };

    double total_COM_shift = 0.0;
};

#endif // TESTDIALOG_H
