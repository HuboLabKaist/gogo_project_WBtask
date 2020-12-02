#ifndef LIFTBOXDIALOG_H
#define LIFTBOXDIALOG_H

#include <QDialog>
#include <QVector>
#include <QTableWidget>
#include "CommonHeader.h"
#include "../../share/Headers/commandlist.h"
#include "../../share/Headers/LANData/VisionLANData.h"

namespace Ui {
class LiftBoxDialog;
}
enum{
    NO_HAND = 0, HAND_RIGHT, HAND_LEFT
};

typedef struct _HAND_INFO_
{
    double  HANDpos[3];
    double  HANDori[4];
    double  HANDelb;
}info_hand;

typedef QVector<info_hand> infos;

typedef struct _TABLE_HAND_
{
    /* final pos and ori */
    info_hand info;

    /* table index */
    int     Rindex;
    int     Lindex;
    int     Saveindex;
    int     Selectedindex;

    /* Flag */
    int     FLAG_HAND = NO_HAND;

}TABLEHAND;

class LiftBoxDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LiftBoxDialog(QWidget *parent = 0);
    ~LiftBoxDialog();
    void InsertRow(QTableWidget *table);
    void InitTable(QTableWidget *table);
    void SetTableRow(QTableWidget *table, int index);

private slots:
    void on_pushButton_clicked();
    void DisplayUpdate();

    void on_PB_READY_clicked();

    void on_PB_START_clicked();

    void on_PB_STOP_clicked();

    void on_pushButton_2_clicked();

    void on_BTN_FORWARD_clicked();

    void on_BTN_LEFT_clicked();

    void on_BTN_RIGHT_clicked();

    void on_BTN_CCW_clicked();

    void on_BTN_CW_clicked();

    void on_BTN_Walk_Ready_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_DATA_SAVE_clicked();

    void on_PB_SIT_DOWN_clicked();

    void on_PB_STAND_UP_clicked();

    void on_PB_STAND_UP_2_clicked();

    void on_PB_STAND_UP_3_clicked();

    void on_PB_BOX__clicked();

    void on_PB_SINGLELOG_WALK_clicked();

    void on_PB_RHAND_GO_clicked();

    void on_PB_LHAND_GO_clicked();

    void on_PB_HAND_GRASP_clicked();

    void on_PB_HAND_STOP_clicked();

    void on_PB_HAND_OPEN_clicked();

    void on_PB_GOR_clicked();

    void on_PB_GOL_clicked();

    void on_PB_LOCK_clicked();

    void on_PB_RCV_Reset_clicked();

    void on_PB_RCV_Push_clicked();

    void on_PB_LCV_Reset_clicked();

    void on_PB_LCV_Push_clicked();

    void on_tableWidget_itemSelectionChanged();

    void on_tableWidgetL_itemSelectionChanged();

    void on_LE_HANDPOS_Y_textChanged(const QString &arg1);

    void on_PB_SAVE_Reset_clicked();

    void on_PB_SAVE_Push_clicked();

    void on_PB_SAVE_save_clicked();

    void on_tableWidgetSave_itemSelectionChanged();

    void on_PB_GOL_Test_clicked();

    void on_PB_CHANGE_INTERPOLATION_clicked();

    void on_PB_STEPPING_STONE_clicked();

    void on_PB_LIFTBOX_BACK_clicked();

    void on_PB_LIFTBOX_SIT_clicked();

    void on_PB_LIFTBOX_HOLD_clicked();

    void on_PB_LIFTBOX_STAND_clicked();

    void on_PB_LIFTBOX_FRONT_clicked();

private:
    Ui::LiftBoxDialog   *ui;
    QTimer              *displayTimer;
    int ALNum_LiftBox;
    int ALNum_ApproachBox;
    int ALNum_WalkReady;

    TABLEHAND           tv;
    infos Rinfos;
    infos Linfos;
    infos Saveinfos;
    info_hand FKinfo;
    int     item_height = 20;
    int     item_width = 40;
    int     col_0_width = 120;
    int     col_1_width = 160;
    int     col_2_width = 40;
};

#endif // LIFTBOXDIALOG_H
