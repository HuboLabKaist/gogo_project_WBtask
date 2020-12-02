#ifndef WBTASK_H
#define WBTASK_H

#include <QDialog>
#include "CommonHeader.h"
#include "JoyStick/joystickclass.h"
//AL CHECK
#include "BasicFiles/PODOALDialog.h"

namespace Ui {
class WBTask;
}

class WBTask : public QDialog
{
    Q_OBJECT

public:
    explicit WBTask(QWidget *parent = 0);
    ~WBTask();

private slots:

    //Joystick Functions
    void DisplayUpdate();
    void GetJoystickData(void);
    void InitJoystickData(void);
    //END Joystick Functions

    //Sensor
    void UpdateSensors();
    //END Sensor

    void on_PB_WKRD_clicked();

    void on_PB_PositionWalking_clicked();

    void on_JOY_BTN_START_clicked();

    void on_JOY_BTN_STOP_clicked();

    void on_MANUAL_BTN_JOY_WALKING_ON_clicked();

    void on_MANUAL_BTN_JOY_WALKING_OFF_clicked();

    void on_PB_Back_LiftBox_clicked();

    void on_PB_Back_PutBox_clicked();

    void on_PB_ALLinOne_clicked();

    void on_PB_STOP_clicked();

    void on_PB_MB_LB_WalkReady_clicked();

    void on_PB_MB_liftBox_clicked();

    void on_PB_MB_putBox_clicked();

    void on_PB_MB_WKRD_clicked();

    void on_PB_COMxBackWalking_clicked();

    void on_PB_Cart_LB_WKRD_clicked();

    void on_PB_holdCart_clicked();

    void on_PB_ReleaseCart_clicked();

    void on_PB_G_open_clicked();

    void on_PB_G_close_half_clicked();

    void on_PB_G_close_clicked();

    void on_PB_SaveStart_clicked();

    void on_PB_SaveDone_clicked();

    void on_PB_SaveStart_HB_clicked();

    void on_PB_SaveDone_HB_clicked();

    void on_PB_CART_Walking_clicked();

    void on_PB_MB_STOP_clicked();

    void on_PB_CART_STOP_clicked();

    void on_PB_SaveStart_Cart_clicked();

    void on_PB_SaveDone_Cart_clicked();

private:
    Ui::WBTask     *ui;
    LANDialog      *lanData; //Communication with Daemon

    RBJoystick     *joystick;
    QTimer		   *displayTimer;

    //AL Numbers
    int    AlnumWalkReady;
    int    ALNum_ApproachBox;
    int    ALNum_LiftBox;
    int    ALNum_WBWalk;
    int    ALNUM_gogoLiftBox;
    int    AlnumHBWalking;
    int    AlnumHBWalking_COM;
    int    AlnumManualMove; //Joystick
    int    Alnum_gogoCart;

    //Joystick Variables
    char          JoyModeFlag           = false;
    bool          ManualModeFlag        = false;
    unsigned long ManualModeCount       = 0;
    char          JoysticWalkingFlag    = false;
    unsigned int  JOY_A_counter         = 0;
    unsigned int  JOY_B_counter         = 0;
    unsigned int  JOY_X_counter         = 0;
    //unsigned int  JOY_BACK_counter      = 0;
    unsigned int  JOY_START_counter     = 0;
    unsigned int  JOY_ARROW_UP_counter  = 0;
    unsigned int  JOY_ARROW_DN_counter  = 0;
    unsigned int  JOY_Y_counter         = 0;
    //unsigned int  JOY_LB_counter        = 0;
    //unsigned int  JOY_RB_counter        = 0;
    bool   JOY_A_flag           = false;
    bool   JOY_B_flag           = false;
    bool   JOY_X_flag           = false;
    bool   JOY_Y_flag           = false;
    //bool   JOY_BACK_flag        = false;
    bool   JOY_START_flag       = false;
    bool   JOY_ARROW_UP_flag    = false;
    bool   JOY_ARROW_DN_flag    = false;
    int    HBWK_com_flag        = false;    //HBWalking or HBWalking_COM
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
    //END Joystick Variables

    int PB_COMWalk_flag = false;
};

#endif // WBTASK_H
