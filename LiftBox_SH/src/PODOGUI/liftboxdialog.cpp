#include "liftboxdialog.h"
#include "ui_liftboxdialog.h"
#include "BasicFiles/PODOALDialog.h"

LiftBoxDialog::LiftBoxDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LiftBoxDialog)
{
    ui->setupUi(this);
    ALNum_LiftBox       = PODOALDialog::GetALNumFromFileName("LiftBox");
    ALNum_ApproachBox   = PODOALDialog::GetALNumFromFileName("ApproachBox");
    ALNum_WalkReady     = PODOALDialog::GetALNumFromFileName("WalkReady");
    displayTimer        = new QTimer(this);

    /* Init ListView */

    tv.Rindex = 0;
    tv.Lindex = 0;
    tv.Saveindex = 0;



    InitTable(ui->tableWidget);
    InitTable(ui->tableWidgetL);
    InitTable(ui->tableWidgetSave);

    tv.Rindex++;
    tv.Lindex++;

    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);
}

LiftBoxDialog::~LiftBoxDialog()
{
    delete ui;
}

void LiftBoxDialog::InsertRow(QTableWidget *table)
{
    QFont tableFont;
    tableFont.setPointSize(8);
    int index;
    if(table == ui->tableWidget)
    {
        index = tv.Rindex;
        Rinfos.push_back(tv.info);
    }
    else if(table == ui->tableWidgetL)
    {
        index = tv.Lindex;
        Linfos.push_back(tv.info);
    } else
    {
        index = tv.Saveindex;
        Saveinfos.push_back(tv.info);
    }

    table->insertRow(index);
    table->setRowHeight(index,20);
    table->setVerticalHeaderItem(index, new QTableWidgetItem());
    table->verticalHeaderItem(index)->setText(QString().sprintf("%d", index));
    table->verticalHeaderItem(index)->setSizeHint(QSize(item_width, item_height));
    table->verticalHeaderItem(index)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    table->verticalHeaderItem(index)->setFont(tableFont);

    for(int j=0; j<3; j++){
        table->setItem(index, j, new QTableWidgetItem());
        table->item(index,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->item(index,j)->setFlags(table->item(index,j)->flags() & ~Qt::ItemIsEditable);
       // table->item(index,j)->setFont(tableFont);
    }


    SetTableRow(table, index);
}

void LiftBoxDialog::InitTable(QTableWidget *table)
{
    double WalkReadyRHpos[3] = {0.293229, -0.246403, 1.019864};
    double WalkReadyLHpos[3] = {0.293229, 0.246403, 1.019864};
    double WalkReadyRHquat[4] = {0.706434, -0.003802, -0.706434, 0.043453};
    double WalkReadyLHquat[4] = {0.706434, 0.003802, -0.706434, -0.043453};
    double WalkReadyElb = 0.0;

    QFont tableFont;
    tableFont.setPointSize(8);

    int index;
    int SkipFLAG;
    if(table == ui->tableWidget)
    {
        index = tv.Rindex;

        memcpy(tv.info.HANDpos,WalkReadyRHpos,sizeof(double)*4);
        memcpy(tv.info.HANDori,WalkReadyRHquat,sizeof(double)*4);
        tv.info.HANDelb = WalkReadyElb;
        Rinfos.push_back(tv.info);
        SkipFLAG = false;
    }
    else if(table == ui->tableWidgetL)
    {
        index = tv.Lindex;
        memcpy(tv.info.HANDpos,WalkReadyLHpos,sizeof(double)*4);
        memcpy(tv.info.HANDori,WalkReadyLHquat,sizeof(double)*4);
        tv.info.HANDelb = WalkReadyElb;
        Linfos.push_back(tv.info);
        SkipFLAG = false;
    } else
    {
        SkipFLAG = true;
    }

    for(int i=0;i<3;i++)
    {
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, 25));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, 25));
    table->horizontalHeaderItem(2)->setSizeHint(QSize(col_2_width, 25));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->setColumnWidth(2, col_2_width);
    table->horizontalHeaderItem(0)->setText("Pos");
    table->horizontalHeaderItem(1)->setText("Quat");
    table->horizontalHeaderItem(2)->setText("Elb");

    // Vertical - Row
    if(SkipFLAG != true)
    {
        table->insertRow(index);
        table->setRowHeight(index,20);
        table->setVerticalHeaderItem(index, new QTableWidgetItem());
        table->verticalHeaderItem(index)->setText("WR");
        table->verticalHeaderItem(index)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(index)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(index)->setFont(tableFont);

        for(int j=0; j<3; j++){
            table->setItem(index, j, new QTableWidgetItem());
            table->item(index,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(index,j)->setFlags(table->item(index,j)->flags() & ~Qt::ItemIsEditable);
            table->item(index,j)->setFont(tableFont);
        }

        SetTableRow(table, index);
    }



    table->setMinimumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);

}

void LiftBoxDialog::SetTableRow(QTableWidget *table, int index)
{

    table->item(index, 0)->setText(QString().sprintf("[%.2f, %.2f, %.2f]",tv.info.HANDpos[0],tv.info.HANDpos[1],tv.info.HANDpos[2]));

    table->item(index, 1)->setText(QString().sprintf("[%.2f, %.2f, %.2f, %.2f]",tv.info.HANDori[0],tv.info.HANDori[1],tv.info.HANDori[2],tv.info.HANDori[3]));

    table->item(index, 2)->setText(QString().sprintf("[%.2f]",tv.info.HANDelb));

}

void LiftBoxDialog::on_pushButton_clicked()
{
    FILE *fp;
    info_hand temp;
    int cnt = 0;
    fp = fopen("ARM.txt","w");
    while(!Saveinfos.empty())
    {
        temp = Saveinfos.first();

        fprintf(fp,"%g\t", temp.HANDpos[0]);
        fprintf(fp,"%g\t", temp.HANDpos[1]);
        fprintf(fp,"%g\t", temp.HANDpos[2]);
        fprintf(fp,"%g\t", temp.HANDori[0]);
        fprintf(fp,"%g\t", temp.HANDori[1]);
        fprintf(fp,"%g\t", temp.HANDori[2]);
        fprintf(fp,"%g\t", temp.HANDori[3]);
        fprintf(fp,"%g\t", temp.HANDelb);
        fprintf(fp, "\n");
        Saveinfos.pop_front();
    }
    fclose(fp);
    printf("Data Save Complete~\n");
}

void LiftBoxDialog::DisplayUpdate()
{
    QString str;

    if(tv.FLAG_HAND == HAND_RIGHT)
    {
       // ui->TV_RHAND->set
    }
//    ui->lineEdit->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[0]));
//    ui->lineEdit_2->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[1]));
//    ui->lineEdit_3->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[2]));
//    ui->lineEdit_4->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[0]));
//    ui->lineEdit_5->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[1]));
//    ui->lineEdit_6->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[2]));
//    ui->lineEdit_7->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[3]));
    ui->LE_FKPOS_X->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.OutputX));
    ui->LE_FKPOS_Y->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.OutputY));
    ui->LE_FKPOS_Z->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.OutputZ));

    ui->LE_FKORI_w->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.Outputw));
    ui->LE_FKORI_x->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.Outputx));
    ui->LE_FKORI_y->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.Outputy));
    ui->LE_FKORI_z->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.Outputz));
    ui->LE_FKELB->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.Outputelb));

}

void LiftBoxDialog::on_PB_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_READY;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_START;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_STOP;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_pushButton_2_clicked()
{
}

void LiftBoxDialog::on_BTN_FORWARD_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_FORWARD_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_LEFT_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_LEFT_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_RIGHT_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_RIGHT_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_CCW_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_CCW_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_CW_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_CW_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_Walk_Ready_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_SIT_DOWN_clicked()
{
    printf("LiftBox_Sit_Down\n");
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFTBOX;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_clicked()
{
    printf("LiftBox_Stand_Up\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_STAND_UP;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_2_clicked()
{
    printf("LiftBox_Hold_BOX\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_HOLD_BOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_3_clicked()
{
    printf("LiftBox_Lift_Box\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFT_BOX;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_BOX__clicked()
{
    printf("LiftBox_Hold_BOX\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_PUTBOX;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_SINGLELOG_WALK_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = SINGLELOG_WALK;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM_2->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH_2->text().toDouble();

    printf("StepNum = %d, StepLength = %f\n",pLAN->G2MData->StepNum, pLAN->G2MData->StepLength);
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_RHAND_GO_clicked()
{
    USER_COMMAND cmd;
    double pos[3], ori[3],quat[4], Elb_deg;

    pos[0] = ui->LE_HANDPOS_X->text().toDouble();
    pos[1] = ui->LE_HANDPOS_Y->text().toDouble();
    pos[2] = ui->LE_HANDPOS_Z->text().toDouble();
    Elb_deg = ui->LE_HAND_ELB->text().toDouble();

    quat[0] = ui->LE_HANDORI_w->text().toDouble();
    quat[1] = ui->LE_HANDORI_x->text().toDouble();
    quat[2] = ui->LE_HANDORI_y->text().toDouble();
    quat[3] = ui->LE_HANDORI_z->text().toDouble();

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_HAND_GO;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;//right

    for(int i=0;i<3;i++)
    {
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i] = pos[i];
    }

    for(int i=0;i<4;i++)
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3+i] = quat[i];

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = Elb_deg;


    printf("Move RHAND pos = [%f,%f,%f] ori = [%f,%f,%f] elb = [%f]\n",pos[0],pos[1],pos[1],ori[0],ori[1],ori[2], Elb_deg);

    for(int i=0;i<3;i++)
        tv.info.HANDpos[i] = pos[i];

    for(int i=0;i<4;i++)
        tv.info.HANDori[i] = quat[i];

    tv.info.HANDelb = Elb_deg;

    tv.FLAG_HAND = RIGHT;

    InsertRow(ui->tableWidget);
    tv.Rindex++;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LHAND_GO_clicked()
{
    USER_COMMAND cmd;
    double pos[3], ori[3], quat[4], Elb_deg;

    pos[0] = ui->LE_HANDPOS_X->text().toDouble();
    pos[1] = ui->LE_HANDPOS_Y->text().toDouble();
    pos[2] = ui->LE_HANDPOS_Z->text().toDouble();
    Elb_deg = ui->LE_HAND_ELB->text().toDouble();

    quat[0] = ui->LE_HANDORI_w->text().toDouble();
    quat[1] = ui->LE_HANDORI_x->text().toDouble();
    quat[2] = ui->LE_HANDORI_y->text().toDouble();
    quat[3] = ui->LE_HANDORI_z->text().toDouble();

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_HAND_GO;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;//left

    for(int i=0;i<3;i++)
    {
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i] = pos[i];
    }
    for(int i=0;i<4;i++)
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3+i] = quat[i];

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = Elb_deg;

    printf("Move LHAND pos = [%f,%f,%f] ori = [%f,%f,%f]\n",pos[0],pos[1],pos[1],ori[0],ori[1],ori[2]);

    for(int i=0;i<3;i++)
        tv.info.HANDpos[i] = pos[i];

    for(int i=0;i<4;i++)
        tv.info.HANDori[i] = quat[i];

    tv.info.HANDelb = Elb_deg;
    InsertRow(ui->tableWidgetL);
    tv.Lindex++;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_HAND_GRASP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_HAND_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_HAND_OPEN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)       cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)       cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_GOR_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_ZEROGAIN;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_GOL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_ZEROGAIN;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LOCK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_ZEROGAIN;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_RCV_Reset_clicked()
{
    while(tv.Rindex > 1)
    {
        ui->tableWidget->removeRow(tv.Rindex);
        tv.Rindex--;
    }
    ui->tableWidget->removeRow(tv.Rindex);
   // Rinfos.pop_front();
}

void LiftBoxDialog::on_PB_RCV_Push_clicked()
{
    ui->LE_HANDPOS_X->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDpos[0]));
    ui->LE_HANDPOS_Y->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDpos[1]));
    ui->LE_HANDPOS_Z->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDpos[2]));

    ui->LE_HANDORI_w->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDori[0]));
    ui->LE_HANDORI_x->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDori[1]));
    ui->LE_HANDORI_y->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDori[2]));
    ui->LE_HANDORI_z->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDori[3]));

    ui->LE_HAND_ELB->setText(QString().sprintf("%f",Rinfos[tv.Selectedindex].HANDelb));
}

void LiftBoxDialog::on_PB_LCV_Reset_clicked()
{
    while(tv.Lindex > 1)
    {
        ui->tableWidgetL->removeRow(tv.Lindex);
        tv.Lindex--;
        Linfos.removeLast();
    }
    ui->tableWidgetL->removeRow(tv.Lindex);
    //Linfos.removeLast();
}

void LiftBoxDialog::on_PB_LCV_Push_clicked()
{
    ui->LE_HANDPOS_X->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDpos[0]));
    ui->LE_HANDPOS_Y->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDpos[1]));
    ui->LE_HANDPOS_Z->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDpos[2]));

    ui->LE_HANDORI_w->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDori[0]));
    ui->LE_HANDORI_x->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDori[1]));
    ui->LE_HANDORI_y->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDori[2]));
    ui->LE_HANDORI_z->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDori[3]));

    ui->LE_HAND_ELB->setText(QString().sprintf("%f",Linfos[tv.Selectedindex].HANDelb));
}

void LiftBoxDialog::on_tableWidget_itemSelectionChanged()
{
    tv.Selectedindex = ui->tableWidget->currentRow();
}

void LiftBoxDialog::on_tableWidgetL_itemSelectionChanged()
{
    tv.Selectedindex = ui->tableWidgetL->currentRow();
}

void LiftBoxDialog::on_LE_HANDPOS_Y_textChanged(const QString &arg1)
{
    ui->PB_LHAND_GO->setEnabled(false);
    ui->PB_RHAND_GO->setEnabled(false);

    if(ui->LE_HANDPOS_Y->text().toDouble() < 0.2)
    {
        ui->PB_RHAND_GO->setEnabled(true);
    }
    if(ui->LE_HANDPOS_Y->text().toDouble() > -0.2)
    {
        ui->PB_LHAND_GO->setEnabled(true);
    }
}

void LiftBoxDialog::on_PB_SAVE_Reset_clicked()
{
    while(tv.Saveindex > 1)
    {
        ui->tableWidgetSave->removeRow(tv.Saveindex);
        tv.Saveindex--;
        Saveinfos.removeLast();
    }
    ui->tableWidgetSave->removeRow(tv.Saveindex);
}

void LiftBoxDialog::on_PB_SAVE_Push_clicked()
{
    ui->LE_HANDPOS_X->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDpos[0]));
    ui->LE_HANDPOS_Y->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDpos[1]));
    ui->LE_HANDPOS_Z->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDpos[2]));

    ui->LE_HANDORI_w->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDori[0]));
    ui->LE_HANDORI_x->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDori[1]));
    ui->LE_HANDORI_y->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDori[2]));
    ui->LE_HANDORI_z->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDori[3]));

    ui->LE_HAND_ELB->setText(QString().sprintf("%f",Saveinfos[tv.Selectedindex].HANDelb));
}

void LiftBoxDialog::on_PB_SAVE_save_clicked()
{
    tv.info.HANDpos[0] = ui->LE_FKPOS_X->text().toDouble();
    tv.info.HANDpos[1] = ui->LE_FKPOS_Y->text().toDouble();
    tv.info.HANDpos[2] = ui->LE_FKPOS_Z->text().toDouble();
    tv.info.HANDelb = ui->LE_FKELB->text().toDouble();
    tv.info.HANDori[0] = ui->LE_FKORI_w->text().toDouble();
    tv.info.HANDori[1] = ui->LE_FKORI_x->text().toDouble();
    tv.info.HANDori[2] = ui->LE_FKORI_y->text().toDouble();
    tv.info.HANDori[3] = ui->LE_FKORI_z->text().toDouble();

    InsertRow(ui->tableWidgetSave);
    tv.Saveindex++;
}

void LiftBoxDialog::on_tableWidgetSave_itemSelectionChanged()
{

    tv.Selectedindex = ui->tableWidgetSave->currentRow();
}

void LiftBoxDialog::on_PB_GOL_Test_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_ZEROGAIN;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_CHANGE_INTERPOLATION_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = YUJIN_TEST_INTERPOLATION;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_INTERPOLATION_DEG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_INTERPOLATION_T->text().toDouble();
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STEPPING_STONE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = STEPPING_STONE_WALK;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LIFTBOX_BACK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LIFTBOX_SIT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LIFTBOX_HOLD_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LIFTBOX_STAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_LIFTBOX_FRONT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_BACK_MOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 4;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    pLAN->SendCommand(cmd);
}
