#ifndef HBWALKINGDIALOG_H
#define HBWALKINGDIALOG_H

#include <QDialog>
#include "CommonHeader.h"

namespace Ui {
class HBWalkingDialog;
}

class HBWalkingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit HBWalkingDialog(QWidget *parent = 0);
    ~HBWalkingDialog();

private slots:
    void on_BT_WALK_READY_clicked();

    void on_BT_HOME_POS_clicked();

    void on_BT_WALK_TEST_clicked();

    void on_BT_TEST_STOP_clicked();

    void on_BT_TEST_clicked();

    void on_BT_DATA_SAVE_clicked();

    void on_BT_READY_TO_WALK_clicked();

    void on_BT_ONE_LEG_READY_clicked();

    void on_BT_ONE_LEG_READY_2_clicked();

private:
    Ui::HBWalkingDialog *ui;

    LANDialog			*lanData;
    QTimer				*displayTimer;

    int                 AlnumHBWalking;
    int                 AlnumWalkReady;
};

#endif // HBWALKINGDIALOG_H
