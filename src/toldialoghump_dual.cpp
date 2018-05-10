#include "../include/motion_manager/toldialoghump_dual.hpp"


namespace motion_manager {

using namespace Qt;

TolDialogHUMPDual::TolDialogHUMPDual(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TolDialogHUMPDual)
{
    ui->setupUi(this);

     QObject::connect(ui->checkBox_approach, SIGNAL(stateChanged(int)), this, SLOT(checkApproach(int)));
     QObject::connect(ui->checkBox_retreat, SIGNAL(stateChanged(int)), this, SLOT(checkRetreat(int)));
     QObject::connect(ui->checkBox_sel_final_posture_right, SIGNAL(stateChanged(int)), this, SLOT(checkFinalPostureRight(int)));
     QObject::connect(ui->checkBox_sel_final_posture_left, SIGNAL(stateChanged(int)), this, SLOT(checkFinalPostureLeft(int)));
     QObject::connect(ui->checkBox_add_plane, SIGNAL(stateChanged(int)), this, SLOT(checkAddPlane(int)));
     //QObject::connect(ui->checkBox_hand_cond, SIGNAL(stateChanged(int)), this, SLOT(checkHandCond(int)));
     //QObject::connect(ui->checkBox_hand_cond_approach, SIGNAL(stateChanged(int)), this, SLOT(checkHandCondApproach(int)));

    if(ui->checkBox_approach->isChecked()){
        ui->groupBox_pre_grasp_right->setEnabled(false);
        ui->groupBox_pre_place_right->setEnabled(false);
        //ui->label_pick_right->setEnabled(false);

        ui->groupBox_pre_grasp_left->setEnabled(false);
        ui->groupBox_pre_place_left->setEnabled(false);
        //ui->label_pick_left->setEnabled(false);
    }
    if(ui->checkBox_retreat->isChecked()){
        ui->groupBox_post_grasp_right->setEnabled(false);
        ui->groupBox_post_place_right->setEnabled(false);
        //ui->label_pick->setEnabled(false);

        ui->groupBox_post_grasp_left->setEnabled(false);
        ui->groupBox_post_place_left->setEnabled(false);
    }
}

TolDialogHUMPDual::~TolDialogHUMPDual()
{
    delete ui;
}





void TolDialogHUMPDual::getTolsArmRight(vector<double> &tols)
{
    tols.clear();
    tols.push_back(ui->lineEdit_shoulder_r_right->text().toDouble());
    tols.push_back(ui->lineEdit_elbow_r_right->text().toDouble());
    tols.push_back(ui->lineEdit_wrist_r_right->text().toDouble());
    tols.push_back(ui->lineEdit_hand_r_right->text().toDouble());
}

void TolDialogHUMPDual::getTolsArmLeft(vector<double> &tols)
{
    tols.clear();
    tols.push_back(ui->lineEdit_shoulder_r_left->text().toDouble());
    tols.push_back(ui->lineEdit_elbow_r_left->text().toDouble());
    tols.push_back(ui->lineEdit_wrist_r_left->text().toDouble());
    tols.push_back(ui->lineEdit_hand_r_left->text().toDouble());
}


void TolDialogHUMPDual::getTolsHandRight(MatrixXd &tols)
{

   tols = MatrixXd::Constant(4,3,1);

    tols(0,0) = ui->lineEdit_hand_1_1_right->text().toDouble(); tols(0,1) = ui->lineEdit_hand_1_2_right->text().toDouble(); tols(0,2) = ui->lineEdit_hand_1_3_right->text().toDouble();
    tols(1,0) = ui->lineEdit_hand_2_1_right->text().toDouble(); tols(1,1) = ui->lineEdit_hand_2_2_right->text().toDouble(); tols(1,2) = ui->lineEdit_hand_2_3_right->text().toDouble();
    tols(2,0) = ui->lineEdit_hand_3_1_right->text().toDouble(); tols(2,1) = ui->lineEdit_hand_3_2_right->text().toDouble(); tols(2,2) = ui->lineEdit_hand_3_3_right->text().toDouble();
    tols(3,0) = ui->lineEdit_hand_tip_1_right->text().toDouble(); tols(3,1) = ui->lineEdit_hand_tip_2_right->text().toDouble(); tols(3,2) = ui->lineEdit_hand_tip_3_right->text().toDouble();
}

void TolDialogHUMPDual::getTolsHandLeft(MatrixXd &tols)
{

   tols = MatrixXd::Constant(4,3,1);

    tols(0,0) = ui->lineEdit_hand_1_1_left->text().toDouble(); tols(0,1) = ui->lineEdit_hand_1_2_left->text().toDouble(); tols(0,2) = ui->lineEdit_hand_1_3_left->text().toDouble();
    tols(1,0) = ui->lineEdit_hand_2_1_left->text().toDouble(); tols(1,1) = ui->lineEdit_hand_2_2_left->text().toDouble(); tols(1,2) = ui->lineEdit_hand_2_3_left->text().toDouble();
    tols(2,0) = ui->lineEdit_hand_3_1_left->text().toDouble(); tols(2,1) = ui->lineEdit_hand_3_2_left->text().toDouble(); tols(2,2) = ui->lineEdit_hand_3_3_left->text().toDouble();
    tols(3,0) = ui->lineEdit_hand_tip_1_left->text().toDouble(); tols(3,1) = ui->lineEdit_hand_tip_2_left->text().toDouble(); tols(3,2) = ui->lineEdit_hand_tip_3_left->text().toDouble();
}



void TolDialogHUMPDual::getLambdaRight(std::vector<double> &lambda)
{

   lambda.clear();
   lambda.push_back(ui->lineEdit_lambda_1_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_2_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_3_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_4_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_5_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_6_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_7_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_8_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_9_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_10_right->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_11_right->text().toDouble());

}

void TolDialogHUMPDual::getLambdaLeft(std::vector<double> &lambda)
{

   lambda.clear();
   lambda.push_back(ui->lineEdit_lambda_1_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_2_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_3_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_4_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_5_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_6_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_7_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_8_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_9_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_10_left->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_11_left->text().toDouble());

}


void TolDialogHUMPDual::getTolsObstaclesRight(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_obs_xx_1_right->text().toDouble(); tols(0,1)=ui->lineEdit_obs_yy_1_right->text().toDouble(); tols(0,2)=ui->lineEdit_obs_zz_1_right->text().toDouble(); tols(0,3)=ui->lineEdit_obs_xy_1_right->text().toDouble(); tols(0,4)=ui->lineEdit_obs_xz_1_right->text().toDouble(); tols(0,5)=ui->lineEdit_obs_yz_1_right->text().toDouble();
    tols(1,0)=ui->lineEdit_obs_xx_2_right->text().toDouble(); tols(1,1)=ui->lineEdit_obs_yy_2_right->text().toDouble(); tols(1,2)=ui->lineEdit_obs_zz_2_right->text().toDouble(); tols(1,3)=ui->lineEdit_obs_xy_2_right->text().toDouble(); tols(1,4)=ui->lineEdit_obs_xz_2_right->text().toDouble(); tols(1,5)=ui->lineEdit_obs_yz_2_right->text().toDouble();
    tols(2,0)=ui->lineEdit_obs_xx_3_right->text().toDouble(); tols(2,1)=ui->lineEdit_obs_yy_3_right->text().toDouble(); tols(2,2)=ui->lineEdit_obs_zz_3_right->text().toDouble(); tols(2,3)=ui->lineEdit_obs_xy_3_right->text().toDouble(); tols(2,4)=ui->lineEdit_obs_xz_3_right->text().toDouble(); tols(2,5)=ui->lineEdit_obs_yz_3_right->text().toDouble();

}

void TolDialogHUMPDual::getTolsObstaclesLeft(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_obs_xx_1_left->text().toDouble(); tols(0,1)=ui->lineEdit_obs_yy_1_left->text().toDouble(); tols(0,2)=ui->lineEdit_obs_zz_1_left->text().toDouble(); tols(0,3)=ui->lineEdit_obs_xy_1_left->text().toDouble(); tols(0,4)=ui->lineEdit_obs_xz_1_left->text().toDouble(); tols(0,5)=ui->lineEdit_obs_yz_1_left->text().toDouble();
    tols(1,0)=ui->lineEdit_obs_xx_2_left->text().toDouble(); tols(1,1)=ui->lineEdit_obs_yy_2_left->text().toDouble(); tols(1,2)=ui->lineEdit_obs_zz_2_left->text().toDouble(); tols(1,3)=ui->lineEdit_obs_xy_2_left->text().toDouble(); tols(1,4)=ui->lineEdit_obs_xz_2_left->text().toDouble(); tols(1,5)=ui->lineEdit_obs_yz_2_left->text().toDouble();
    tols(2,0)=ui->lineEdit_obs_xx_3_left->text().toDouble(); tols(2,1)=ui->lineEdit_obs_yy_3_left->text().toDouble(); tols(2,2)=ui->lineEdit_obs_zz_3_left->text().toDouble(); tols(2,3)=ui->lineEdit_obs_xy_3_left->text().toDouble(); tols(2,4)=ui->lineEdit_obs_xz_3_left->text().toDouble(); tols(2,5)=ui->lineEdit_obs_yz_3_left->text().toDouble();

}


void TolDialogHUMPDual::getTolsTargetRight(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_tar_xx_1_right->text().toDouble(); tols(0,1)=ui->lineEdit_tar_yy_1_right->text().toDouble(); tols(0,2)=ui->lineEdit_tar_zz_1_right->text().toDouble(); tols(0,3)=ui->lineEdit_tar_xy_1_right->text().toDouble(); tols(0,4)=ui->lineEdit_tar_xz_1_right->text().toDouble(); tols(0,5)=ui->lineEdit_tar_yz_1_right->text().toDouble();
    tols(1,0)=ui->lineEdit_tar_xx_2_right->text().toDouble(); tols(1,1)=ui->lineEdit_tar_yy_2_right->text().toDouble(); tols(1,2)=ui->lineEdit_tar_zz_2_right->text().toDouble(); tols(1,3)=ui->lineEdit_tar_xy_2_right->text().toDouble(); tols(1,4)=ui->lineEdit_tar_xz_2_right->text().toDouble(); tols(1,5)=ui->lineEdit_tar_yz_2_right->text().toDouble();
    tols(2,0)=ui->lineEdit_tar_xx_3_right->text().toDouble(); tols(2,1)=ui->lineEdit_tar_yy_3_right->text().toDouble(); tols(2,2)=ui->lineEdit_tar_zz_3_right->text().toDouble(); tols(2,3)=ui->lineEdit_tar_xy_3_right->text().toDouble(); tols(2,4)=ui->lineEdit_tar_xz_3_right->text().toDouble(); tols(2,5)=ui->lineEdit_tar_yz_3_right->text().toDouble();

}

void TolDialogHUMPDual::getTolsTargetLeft(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_tar_xx_1_left->text().toDouble(); tols(0,1)=ui->lineEdit_tar_yy_1_left->text().toDouble(); tols(0,2)=ui->lineEdit_tar_zz_1_left->text().toDouble(); tols(0,3)=ui->lineEdit_tar_xy_1_left->text().toDouble(); tols(0,4)=ui->lineEdit_tar_xz_1_left->text().toDouble(); tols(0,5)=ui->lineEdit_tar_yz_1_left->text().toDouble();
    tols(1,0)=ui->lineEdit_tar_xx_2_left->text().toDouble(); tols(1,1)=ui->lineEdit_tar_yy_2_left->text().toDouble(); tols(1,2)=ui->lineEdit_tar_zz_2_left->text().toDouble(); tols(1,3)=ui->lineEdit_tar_xy_2_left->text().toDouble(); tols(1,4)=ui->lineEdit_tar_xz_2_left->text().toDouble(); tols(1,5)=ui->lineEdit_tar_yz_2_left->text().toDouble();
    tols(2,0)=ui->lineEdit_tar_xx_3_left->text().toDouble(); tols(2,1)=ui->lineEdit_tar_yy_3_left->text().toDouble(); tols(2,2)=ui->lineEdit_tar_zz_3_left->text().toDouble(); tols(2,3)=ui->lineEdit_tar_xy_3_left->text().toDouble(); tols(2,4)=ui->lineEdit_tar_xz_3_left->text().toDouble(); tols(2,5)=ui->lineEdit_tar_yz_3_left->text().toDouble();


}



double TolDialogHUMPDual::getWMax()
{

    return ui->lineEdit_w_max->text().toDouble();
}

double TolDialogHUMPDual::getAlphaMax()
{

    return ui->lineEdit_alpha_max->text().toDouble();
}

void TolDialogHUMPDual::setWMax(double w)
{

    ui->lineEdit_w_max->setText(QString::number(w));
}




double TolDialogHUMPDual::getTolTarPosRight()
{

    return ui->lineEdit_tar_pos_right->text().toDouble();
}

double TolDialogHUMPDual::getTolTarPosLeft()
{

    return ui->lineEdit_tar_pos_left->text().toDouble();
}


double TolDialogHUMPDual::getTolTarOrRight()
{

    return ui->lineEdit_tar_or_right->text().toDouble();
}

double TolDialogHUMPDual::getTolTarOrLeft()
{

    return ui->lineEdit_tar_or_left->text().toDouble();
}

void TolDialogHUMPDual::setInfo(string info)
{

    this->infoLine = info;
}

bool TolDialogHUMPDual::getTargetAvoidance()
{

   return !ui->checkBox_tar_av->isChecked();
}

bool TolDialogHUMPDual::getObstacleAvoidance()
{

    return !ui->checkBox_ob_av->isChecked();
}

bool TolDialogHUMPDual::getApproach()
{
    return !ui->checkBox_approach->isChecked();
}

bool TolDialogHUMPDual::getRetreat()
{
    return !ui->checkBox_retreat->isChecked();
}

void TolDialogHUMPDual::getInitVelRight(std::vector<double> &init_vel)
{
    init_vel.clear();
    init_vel.push_back(ui->lineEdit_init_vel_1_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_2_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_3_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_4_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_5_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_6_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_7_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_8_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_9_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_10_right->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_11_right->text().toDouble());
}

void TolDialogHUMPDual::getInitVelLeft(std::vector<double> &init_vel)
{
    init_vel.clear();
    init_vel.push_back(ui->lineEdit_init_vel_1_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_2_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_3_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_4_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_5_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_6_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_7_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_8_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_9_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_10_left->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_11_left->text().toDouble());
}

void TolDialogHUMPDual::getFinalVelRight(std::vector<double> &final_vel)
{
    final_vel.clear();
    final_vel.push_back(ui->lineEdit_final_vel_1_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_2_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_3_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_4_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_5_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_6_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_7_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_8_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_9_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_10_right->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_11_right->text().toDouble());
}

void TolDialogHUMPDual::getFinalVelLeft(std::vector<double> &final_vel)
{
    final_vel.clear();
    final_vel.push_back(ui->lineEdit_final_vel_1_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_2_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_3_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_4_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_5_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_6_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_7_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_8_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_9_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_10_left->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_11_left->text().toDouble());
}

void TolDialogHUMPDual::getInitAccRight(std::vector<double> &init_acc)
{
    init_acc.clear();
    init_acc.push_back(ui->lineEdit_init_acc_1_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_2_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_3_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_4_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_5_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_6_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_7_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_8_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_9_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_10_right->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_11_right->text().toDouble());
}

void TolDialogHUMPDual::getInitAccLeft(std::vector<double> &init_acc)
{
    init_acc.clear();
    init_acc.push_back(ui->lineEdit_init_acc_1_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_2_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_3_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_4_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_5_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_6_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_7_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_8_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_9_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_10_left->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_11_left->text().toDouble());
}

void TolDialogHUMPDual::getFinalAccRight(std::vector<double> &final_acc)
{
    final_acc.clear();
    final_acc.push_back(ui->lineEdit_final_acc_1_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_2_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_3_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_4_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_5_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_6_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_7_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_8_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_9_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_10_right->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_11_right->text().toDouble());
}

void TolDialogHUMPDual::getFinalAccLeft(std::vector<double> &final_acc)
{
    final_acc.clear();
    final_acc.push_back(ui->lineEdit_final_acc_1_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_2_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_3_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_4_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_5_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_6_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_7_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_8_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_9_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_10_left->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_11_left->text().toDouble());
}

/*

void TolDialogHUMPDual::getVelApproach(std::vector<double> &vel_approach)
{
    vel_approach.clear();
    vel_approach.push_back(ui->lineEdit_vel_approach_1->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_2->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_3->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_4->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_5->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_6->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_7->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_8->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_9->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_10->text().toDouble());
    vel_approach.push_back(ui->lineEdit_vel_approach_11->text().toDouble());
}

void TolDialogHUMPDual::getAccApproach(std::vector<double> &acc_approach)
{
    acc_approach.clear();
    acc_approach.push_back(ui->lineEdit_acc_approach_1->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_2->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_3->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_4->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_5->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_6->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_7->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_8->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_9->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_10->text().toDouble());
    acc_approach.push_back(ui->lineEdit_acc_approach_11->text().toDouble());
}

*/
void TolDialogHUMPDual::getPreGraspApproachRight(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x_right->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y_right->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z_right->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist_right->text().toDouble());
}

void TolDialogHUMPDual::getPreGraspApproachLeft(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x_left->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y_left->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z_left->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist_left->text().toDouble());
}

void TolDialogHUMPDual::getPostGraspRetreatRight(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x_right->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y_right->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z_right->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist_right->text().toDouble());
}

void TolDialogHUMPDual::getPostGraspRetreatLeft(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x_left->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y_left->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z_left->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist_left->text().toDouble());
}

void TolDialogHUMPDual::getPrePlaceApproachRight(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x_right->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y_right->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z_right->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist_right->text().toDouble());

}

void TolDialogHUMPDual::getPrePlaceApproachLeft(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x_left->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y_left->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z_left->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist_left->text().toDouble());

}

void TolDialogHUMPDual::getPostPlaceRetreatRight(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x_right->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y_right->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z_right->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist_right->text().toDouble());
}

void TolDialogHUMPDual::getPostPlaceRetreatLeft(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x_left->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y_left->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z_left->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist_left->text().toDouble());
}

double TolDialogHUMPDual::getW_red_app_right()
{
    return ui->lineEdit_w_red_app_right->text().toDouble();
}

double TolDialogHUMPDual::getW_red_app_left()
{
    return ui->lineEdit_w_red_app_left->text().toDouble();
}

double TolDialogHUMPDual::getW_red_ret_right()
{
    return ui->lineEdit_w_red_ret_right->text().toDouble();
}

double TolDialogHUMPDual::getW_red_ret_left()
{
    return ui->lineEdit_w_red_ret_left->text().toDouble();
}

void TolDialogHUMPDual::getTargetMoveRight(std::vector<double> &target)
{
    target.clear();
    target.push_back(ui->lineEdit_target_x_right->text().toDouble());
    target.push_back(ui->lineEdit_target_y_right->text().toDouble());
    target.push_back(ui->lineEdit_target_z_right->text().toDouble());
    target.push_back(ui->lineEdit_target_roll_right->text().toDouble());
    target.push_back(ui->lineEdit_target_pitch_right->text().toDouble());
    target.push_back(ui->lineEdit_target_yaw_right->text().toDouble());
}

void TolDialogHUMPDual::getTargetMoveLeft(std::vector<double> &target)
{
    target.clear();
    target.push_back(ui->lineEdit_target_x_left->text().toDouble());
    target.push_back(ui->lineEdit_target_y_left->text().toDouble());
    target.push_back(ui->lineEdit_target_z_left->text().toDouble());
    target.push_back(ui->lineEdit_target_roll_left->text().toDouble());
    target.push_back(ui->lineEdit_target_pitch_left->text().toDouble());
    target.push_back(ui->lineEdit_target_yaw_left->text().toDouble());
}

void TolDialogHUMPDual::setTargetMoveRight(std::vector<double> &target)
{
    ui->lineEdit_target_x_right->setText(QString::number(target.at(0)));
    ui->lineEdit_target_y_right->setText(QString::number(target.at(1)));
    ui->lineEdit_target_z_right->setText(QString::number(target.at(2)));
    ui->lineEdit_target_roll_right->setText(QString::number(target.at(3)));
    ui->lineEdit_target_pitch_right->setText(QString::number(target.at(4)));
    ui->lineEdit_target_yaw_right->setText(QString::number(target.at(5)));
}

void TolDialogHUMPDual::setTargetMoveLeft(std::vector<double> &target)
{
    ui->lineEdit_target_x_left->setText(QString::number(target.at(0)));
    ui->lineEdit_target_y_left->setText(QString::number(target.at(1)));
    ui->lineEdit_target_z_left->setText(QString::number(target.at(2)));
    ui->lineEdit_target_roll_left->setText(QString::number(target.at(3)));
    ui->lineEdit_target_pitch_left->setText(QString::number(target.at(4)));
    ui->lineEdit_target_yaw_left->setText(QString::number(target.at(5)));
}

void TolDialogHUMPDual::getFinalArmRight(std::vector<double> &finalArm)
{
    finalArm.clear();
    finalArm.push_back(ui->lineEdit_final_arm_1_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_2_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_3_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_4_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_5_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_6_right->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_7_right->text().toDouble());
}

void TolDialogHUMPDual::getFinalArmLeft(std::vector<double> &finalArm)
{
    finalArm.clear();
    finalArm.push_back(ui->lineEdit_final_arm_1_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_2_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_3_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_4_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_5_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_6_left->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_7_left->text().toDouble());
}

void TolDialogHUMPDual::getFinalHandRight(std::vector<double> &finalHand)
{
    finalHand.clear();
    finalHand.push_back(ui->lineEdit_final_hand_1_right->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_2_right->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_3_right->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_4_right->text().toDouble());
}

void TolDialogHUMPDual::getFinalHandLeft(std::vector<double> &finalHand)
{
    finalHand.clear();
    finalHand.push_back(ui->lineEdit_final_hand_1_left->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_2_left->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_3_left->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_4_left->text().toDouble());
}

void TolDialogHUMPDual::setPlaneParameters(std::vector<double> &point1,std::vector<double> &point2,std::vector<double> &point3)
{
    if(!point1.empty() && !point2.empty() && !point3.empty()){
        ui->lineEdit_point_1_x->setText(QString::number(point1.at(0)));
        ui->lineEdit_point_1_y->setText(QString::number(point1.at(1)));
        ui->lineEdit_point_1_z->setText(QString::number(point1.at(2)));
        ui->lineEdit_point_2_x->setText(QString::number(point2.at(0)));
        ui->lineEdit_point_2_y->setText(QString::number(point2.at(1)));
        ui->lineEdit_point_2_z->setText(QString::number(point2.at(2)));
        ui->lineEdit_point_3_x->setText(QString::number(point3.at(0)));
        ui->lineEdit_point_3_y->setText(QString::number(point3.at(1)));
        ui->lineEdit_point_3_z->setText(QString::number(point3.at(2)));
    }
}

void TolDialogHUMPDual::setInitJointsVelRight(std::vector<double>& init_vel)
{
    if(!init_vel.empty())
    {
        ui->lineEdit_init_vel_1_right->setText(QString::number(init_vel.at(0)));
        ui->lineEdit_init_vel_2_right->setText(QString::number(init_vel.at(1)));
        ui->lineEdit_init_vel_3_right->setText(QString::number(init_vel.at(2)));
        ui->lineEdit_init_vel_4_right->setText(QString::number(init_vel.at(3)));
        ui->lineEdit_init_vel_5_right->setText(QString::number(init_vel.at(4)));
        ui->lineEdit_init_vel_6_right->setText(QString::number(init_vel.at(5)));
        ui->lineEdit_init_vel_7_right->setText(QString::number(init_vel.at(6)));
        ui->lineEdit_init_vel_8_right->setText(QString::number(init_vel.at(7)));
        ui->lineEdit_init_vel_9_right->setText(QString::number(init_vel.at(8)));
        ui->lineEdit_init_vel_10_right->setText(QString::number(init_vel.at(9)));
        ui->lineEdit_init_vel_11_right->setText(QString::number(init_vel.at(10)));
    }
}

void TolDialogHUMPDual::setInitJointsVelLeft(std::vector<double>& init_vel)
{
    if(!init_vel.empty())
    {
        ui->lineEdit_init_vel_1_left->setText(QString::number(init_vel.at(0)));
        ui->lineEdit_init_vel_2_left->setText(QString::number(init_vel.at(1)));
        ui->lineEdit_init_vel_3_left->setText(QString::number(init_vel.at(2)));
        ui->lineEdit_init_vel_4_left->setText(QString::number(init_vel.at(3)));
        ui->lineEdit_init_vel_5_left->setText(QString::number(init_vel.at(4)));
        ui->lineEdit_init_vel_6_left->setText(QString::number(init_vel.at(5)));
        ui->lineEdit_init_vel_7_left->setText(QString::number(init_vel.at(6)));
        ui->lineEdit_init_vel_8_left->setText(QString::number(init_vel.at(7)));
        ui->lineEdit_init_vel_9_left->setText(QString::number(init_vel.at(8)));
        ui->lineEdit_init_vel_10_left->setText(QString::number(init_vel.at(9)));
        ui->lineEdit_init_vel_11_left->setText(QString::number(init_vel.at(10)));
    }
}

void TolDialogHUMPDual::setInitJointsAccRight(std::vector<double>& init_acc)
{
    if(!init_acc.empty())
    {
        ui->lineEdit_init_acc_1_right->setText(QString::number(init_acc.at(0)));
        ui->lineEdit_init_acc_2_right->setText(QString::number(init_acc.at(1)));
        ui->lineEdit_init_acc_3_right->setText(QString::number(init_acc.at(2)));
        ui->lineEdit_init_acc_4_right->setText(QString::number(init_acc.at(3)));
        ui->lineEdit_init_acc_5_right->setText(QString::number(init_acc.at(4)));
        ui->lineEdit_init_acc_6_right->setText(QString::number(init_acc.at(5)));
        ui->lineEdit_init_acc_7_right->setText(QString::number(init_acc.at(6)));
        ui->lineEdit_init_acc_8_right->setText(QString::number(init_acc.at(7)));
        ui->lineEdit_init_acc_9_right->setText(QString::number(init_acc.at(8)));
        ui->lineEdit_init_acc_10_right->setText(QString::number(init_acc.at(9)));
        ui->lineEdit_init_acc_11_right->setText(QString::number(init_acc.at(10)));
    }
}

void TolDialogHUMPDual::setInitJointsAccLeft(std::vector<double>& init_acc)
{
    if(!init_acc.empty())
    {
        ui->lineEdit_init_acc_1_left->setText(QString::number(init_acc.at(0)));
        ui->lineEdit_init_acc_2_left->setText(QString::number(init_acc.at(1)));
        ui->lineEdit_init_acc_3_left->setText(QString::number(init_acc.at(2)));
        ui->lineEdit_init_acc_4_left->setText(QString::number(init_acc.at(3)));
        ui->lineEdit_init_acc_5_left->setText(QString::number(init_acc.at(4)));
        ui->lineEdit_init_acc_6_left->setText(QString::number(init_acc.at(5)));
        ui->lineEdit_init_acc_7_left->setText(QString::number(init_acc.at(6)));
        ui->lineEdit_init_acc_8_left->setText(QString::number(init_acc.at(7)));
        ui->lineEdit_init_acc_9_left->setText(QString::number(init_acc.at(8)));
        ui->lineEdit_init_acc_10_left->setText(QString::number(init_acc.at(9)));
        ui->lineEdit_init_acc_11_left->setText(QString::number(init_acc.at(10)));
    }
}

void TolDialogHUMPDual::getPlaneParameters(std::vector<double> &params)
{
    params.clear(); double a,b,c,d;
    std::vector<double> point1;
    std::vector<double> point2;
    std::vector<double> point3;

    point1.push_back(ui->lineEdit_point_1_x->text().toDouble());
    point1.push_back(ui->lineEdit_point_1_y->text().toDouble());
    point1.push_back(ui->lineEdit_point_1_z->text().toDouble());

    point2.push_back(ui->lineEdit_point_2_x->text().toDouble());
    point2.push_back(ui->lineEdit_point_2_y->text().toDouble());
    point2.push_back(ui->lineEdit_point_2_z->text().toDouble());

    point3.push_back(ui->lineEdit_point_3_x->text().toDouble());
    point3.push_back(ui->lineEdit_point_3_y->text().toDouble());
    point3.push_back(ui->lineEdit_point_3_z->text().toDouble());


    Matrix3d D; double det;
    D << point1.at(0),point1.at(1),point1.at(2),
        point2.at(0),point2.at(1),point2.at(2),
        point3.at(0),point3.at(1),point3.at(2);

    det = D.determinant();

    if(det!=0){
        d=1000;

        Matrix3d A;
        A << 1,point1.at(1),point1.at(2),
            1,point2.at(1),point2.at(2),
            1,point3.at(1),point3.at(2);
        a = (-d/det)*A.determinant();

        Matrix3d B;
        B << point1.at(0),1,point1.at(2),
            point2.at(0),1,point2.at(2),
            point3.at(0),1,point3.at(2);
        b = (-d/det)*B.determinant();

        Matrix3d C;
        C << point1.at(0),point1.at(1),1,
            point2.at(0),point2.at(1),1,
            point3.at(0),point3.at(1),1;
        c = (-d/det)*C.determinant();

        params.push_back(a);
        params.push_back(b);
        params.push_back(c);
        params.push_back(d);
    }



}

// Q_SLOTS

void TolDialogHUMPDual::on_pushButton_save_clicked()
{

   QString filename = QFileDialog::getSaveFileName(this,
                                                   tr("Save the file of tolerances"),
                                                   QString(MAIN_PATH)+"/Tols",
                                                   "All Files (*.*);;Tol Files (*.tol)");
   QFile f( filename );
   if(f.open( QIODevice::WriteOnly )){

       QTextStream stream( &f );
       stream << "### Parameters of the Human-like Upper-limbs Motion Library ###" << endl;
       stream << "# "<< this->infoLine.c_str() << endl;

       stream << "# Geometric Dimensions of the arms and of the fingers (right)" << endl;
       stream << "Shoulder_radius_right=" << ui->lineEdit_shoulder_r_right->text().toStdString().c_str() << endl;
       stream << "Elbow_radius_right=" << ui->lineEdit_elbow_r_right->text().toStdString().c_str() << endl;
       stream << "Wrist_radius_right=" << ui->lineEdit_wrist_r_right->text().toStdString().c_str() << endl;
       stream << "Hand_radius_right=" << ui->lineEdit_hand_r_right->text().toStdString().c_str() << endl;
       stream << "Hand_1_1_right=" << ui->lineEdit_hand_1_1_right->text().toStdString().c_str() << endl;
       stream << "Hand_1_2_right=" << ui->lineEdit_hand_1_2_right->text().toStdString().c_str() << endl;
       stream << "Hand_1_3_right=" << ui->lineEdit_hand_1_3_right->text().toStdString().c_str() << endl;
       stream << "Hand_2_1_right=" << ui->lineEdit_hand_2_1_right->text().toStdString().c_str() << endl;
       stream << "Hand_2_2_right=" << ui->lineEdit_hand_2_2_right->text().toStdString().c_str() << endl;
       stream << "Hand_2_3_right=" << ui->lineEdit_hand_2_3_right->text().toStdString().c_str() << endl;
       stream << "Hand_3_1_right=" << ui->lineEdit_hand_3_1_right->text().toStdString().c_str() << endl;
       stream << "Hand_3_2_right=" << ui->lineEdit_hand_3_2_right->text().toStdString().c_str() << endl;
       stream << "Hand_3_3_right=" << ui->lineEdit_hand_3_3_right->text().toStdString().c_str() << endl;
       stream << "Hand_tip_1_right=" << ui->lineEdit_hand_tip_1_right->text().toStdString().c_str() << endl;
       stream << "Hand_tip_2_right=" << ui->lineEdit_hand_tip_2_right->text().toStdString().c_str() << endl;
       stream << "Hand_tip_3_right=" << ui->lineEdit_hand_tip_3_right->text().toStdString().c_str() << endl;

       stream << "# Geometric Dimensions of the arms and of the fingers (left)" << endl;
       stream << "Shoulder_radius_left=" << ui->lineEdit_shoulder_r_left->text().toStdString().c_str() << endl;
       stream << "Elbow_radius_left=" << ui->lineEdit_elbow_r_left->text().toStdString().c_str() << endl;
       stream << "Wrist_radius_left=" << ui->lineEdit_wrist_r_left->text().toStdString().c_str() << endl;
       stream << "Hand_radius_left=" << ui->lineEdit_hand_r_left->text().toStdString().c_str() << endl;
       stream << "Hand_1_1_left=" << ui->lineEdit_hand_1_1_left->text().toStdString().c_str() << endl;
       stream << "Hand_1_2_left=" << ui->lineEdit_hand_1_2_left->text().toStdString().c_str() << endl;
       stream << "Hand_1_3_left=" << ui->lineEdit_hand_1_3_left->text().toStdString().c_str() << endl;
       stream << "Hand_2_1_left=" << ui->lineEdit_hand_2_1_left->text().toStdString().c_str() << endl;
       stream << "Hand_2_2_left=" << ui->lineEdit_hand_2_2_left->text().toStdString().c_str() << endl;
       stream << "Hand_2_3_left=" << ui->lineEdit_hand_2_3_left->text().toStdString().c_str() << endl;
       stream << "Hand_3_1_left=" << ui->lineEdit_hand_3_1_left->text().toStdString().c_str() << endl;
       stream << "Hand_3_2_left=" << ui->lineEdit_hand_3_2_left->text().toStdString().c_str() << endl;
       stream << "Hand_3_3_left=" << ui->lineEdit_hand_3_3_left->text().toStdString().c_str() << endl;
       stream << "Hand_tip_1_left=" << ui->lineEdit_hand_tip_1_left->text().toStdString().c_str() << endl;
       stream << "Hand_tip_2_left=" << ui->lineEdit_hand_tip_2_left->text().toStdString().c_str() << endl;
       stream << "Hand_tip_3_left=" << ui->lineEdit_hand_tip_3_left->text().toStdString().c_str() << endl;

       stream << "# Joint Expanse factors (right)" << endl;
       stream << "lambda_1_right=" << ui->lineEdit_lambda_1_right->text().toStdString().c_str() << endl;
       stream << "lambda_2_right=" << ui->lineEdit_lambda_2_right->text().toStdString().c_str() << endl;
       stream << "lambda_3_right=" << ui->lineEdit_lambda_3_right->text().toStdString().c_str() << endl;
       stream << "lambda_4_right=" << ui->lineEdit_lambda_4_right->text().toStdString().c_str() << endl;
       stream << "lambda_5_right=" << ui->lineEdit_lambda_5_right->text().toStdString().c_str() << endl;
       stream << "lambda_6_right=" << ui->lineEdit_lambda_6_right->text().toStdString().c_str() << endl;
       stream << "lambda_7_right=" << ui->lineEdit_lambda_7_right->text().toStdString().c_str() << endl;
       stream << "lambda_8_right=" << ui->lineEdit_lambda_8_right->text().toStdString().c_str() << endl;
       stream << "lambda_9_right=" << ui->lineEdit_lambda_9_right->text().toStdString().c_str() << endl;
       stream << "lambda_10_right=" << ui->lineEdit_lambda_10_right->text().toStdString().c_str() << endl;
       stream << "lambda_11_right=" << ui->lineEdit_lambda_11_right->text().toStdString().c_str() << endl;

       stream << "# Joint Expanse factors (left)" << endl;
       stream << "lambda_1_left=" << ui->lineEdit_lambda_1_left->text().toStdString().c_str() << endl;
       stream << "lambda_2_left=" << ui->lineEdit_lambda_2_left->text().toStdString().c_str() << endl;
       stream << "lambda_3_left=" << ui->lineEdit_lambda_3_left->text().toStdString().c_str() << endl;
       stream << "lambda_4_left=" << ui->lineEdit_lambda_4_left->text().toStdString().c_str() << endl;
       stream << "lambda_5_left=" << ui->lineEdit_lambda_5_left->text().toStdString().c_str() << endl;
       stream << "lambda_6_left=" << ui->lineEdit_lambda_6_left->text().toStdString().c_str() << endl;
       stream << "lambda_7_left=" << ui->lineEdit_lambda_7_left->text().toStdString().c_str() << endl;
       stream << "lambda_8_left=" << ui->lineEdit_lambda_8_left->text().toStdString().c_str() << endl;
       stream << "lambda_9_left=" << ui->lineEdit_lambda_9_left->text().toStdString().c_str() << endl;
       stream << "lambda_10_left=" << ui->lineEdit_lambda_10_left->text().toStdString().c_str() << endl;
       stream << "lambda_11_left=" << ui->lineEdit_lambda_11_left->text().toStdString().c_str() << endl;

       stream << "# Initial Velocity (right)" << endl;
       stream << "init_vel_1_right=" << ui->lineEdit_init_vel_1_right->text().toStdString().c_str() << endl;
       stream << "init_vel_2_right=" << ui->lineEdit_init_vel_2_right->text().toStdString().c_str() << endl;
       stream << "init_vel_3_right=" << ui->lineEdit_init_vel_3_right->text().toStdString().c_str() << endl;
       stream << "init_vel_4_right=" << ui->lineEdit_init_vel_4_right->text().toStdString().c_str() << endl;
       stream << "init_vel_5_right=" << ui->lineEdit_init_vel_5_right->text().toStdString().c_str() << endl;
       stream << "init_vel_6_right=" << ui->lineEdit_init_vel_6_right->text().toStdString().c_str() << endl;
       stream << "init_vel_7_right=" << ui->lineEdit_init_vel_7_right->text().toStdString().c_str() << endl;
       stream << "init_vel_8_right=" << ui->lineEdit_init_vel_8_right->text().toStdString().c_str() << endl;
       stream << "init_vel_9_right=" << ui->lineEdit_init_vel_9_right->text().toStdString().c_str() << endl;
       stream << "init_vel_10_right=" << ui->lineEdit_init_vel_10_right->text().toStdString().c_str() << endl;
       stream << "init_vel_11_right=" << ui->lineEdit_init_vel_11_right->text().toStdString().c_str() << endl;

       stream << "# Initial Velocity (left)" << endl;
       stream << "init_vel_1_left=" << ui->lineEdit_init_vel_1_left->text().toStdString().c_str() << endl;
       stream << "init_vel_2_left=" << ui->lineEdit_init_vel_2_left->text().toStdString().c_str() << endl;
       stream << "init_vel_3_left=" << ui->lineEdit_init_vel_3_left->text().toStdString().c_str() << endl;
       stream << "init_vel_4_left=" << ui->lineEdit_init_vel_4_left->text().toStdString().c_str() << endl;
       stream << "init_vel_5_left=" << ui->lineEdit_init_vel_5_left->text().toStdString().c_str() << endl;
       stream << "init_vel_6_left=" << ui->lineEdit_init_vel_6_left->text().toStdString().c_str() << endl;
       stream << "init_vel_7_left=" << ui->lineEdit_init_vel_7_left->text().toStdString().c_str() << endl;
       stream << "init_vel_8_left=" << ui->lineEdit_init_vel_8_left->text().toStdString().c_str() << endl;
       stream << "init_vel_9_left=" << ui->lineEdit_init_vel_9_left->text().toStdString().c_str() << endl;
       stream << "init_vel_10_left=" << ui->lineEdit_init_vel_10_left->text().toStdString().c_str() << endl;
       stream << "init_vel_11_left=" << ui->lineEdit_init_vel_11_left->text().toStdString().c_str() << endl;

       stream << "# Final Velocity (right)" << endl;
       stream << "final_vel_1_right=" << ui->lineEdit_final_vel_1_right->text().toStdString().c_str() << endl;
       stream << "final_vel_2_right=" << ui->lineEdit_final_vel_2_right->text().toStdString().c_str() << endl;
       stream << "final_vel_3_right=" << ui->lineEdit_final_vel_3_right->text().toStdString().c_str() << endl;
       stream << "final_vel_4_right=" << ui->lineEdit_final_vel_4_right->text().toStdString().c_str() << endl;
       stream << "final_vel_5_right=" << ui->lineEdit_final_vel_5_right->text().toStdString().c_str() << endl;
       stream << "final_vel_6_right=" << ui->lineEdit_final_vel_6_right->text().toStdString().c_str() << endl;
       stream << "final_vel_7_right=" << ui->lineEdit_final_vel_7_right->text().toStdString().c_str() << endl;
       stream << "final_vel_8_right=" << ui->lineEdit_final_vel_8_right->text().toStdString().c_str() << endl;
       stream << "final_vel_9_right=" << ui->lineEdit_final_vel_9_right->text().toStdString().c_str() << endl;
       stream << "final_vel_10_right=" << ui->lineEdit_final_vel_10_right->text().toStdString().c_str() << endl;
       stream << "final_vel_11_right=" << ui->lineEdit_final_vel_11_right->text().toStdString().c_str() << endl;

       stream << "# Final Velocity (left)" << endl;
       stream << "final_vel_1_left=" << ui->lineEdit_final_vel_1_left->text().toStdString().c_str() << endl;
       stream << "final_vel_2_left=" << ui->lineEdit_final_vel_2_left->text().toStdString().c_str() << endl;
       stream << "final_vel_3_left=" << ui->lineEdit_final_vel_3_left->text().toStdString().c_str() << endl;
       stream << "final_vel_4_left=" << ui->lineEdit_final_vel_4_left->text().toStdString().c_str() << endl;
       stream << "final_vel_5_left=" << ui->lineEdit_final_vel_5_left->text().toStdString().c_str() << endl;
       stream << "final_vel_6_left=" << ui->lineEdit_final_vel_6_left->text().toStdString().c_str() << endl;
       stream << "final_vel_7_left=" << ui->lineEdit_final_vel_7_left->text().toStdString().c_str() << endl;
       stream << "final_vel_8_left=" << ui->lineEdit_final_vel_8_left->text().toStdString().c_str() << endl;
       stream << "final_vel_9_left=" << ui->lineEdit_final_vel_9_left->text().toStdString().c_str() << endl;
       stream << "final_vel_10_left=" << ui->lineEdit_final_vel_10_left->text().toStdString().c_str() << endl;
       stream << "final_vel_11_left=" << ui->lineEdit_final_vel_11_left->text().toStdString().c_str() << endl;

       stream << "# Initial Acceleration (right)" << endl;
       stream << "init_acc_1_right=" << ui->lineEdit_init_acc_1_right->text().toStdString().c_str() << endl;
       stream << "init_acc_2_right=" << ui->lineEdit_init_acc_2_right->text().toStdString().c_str() << endl;
       stream << "init_acc_3_right=" << ui->lineEdit_init_acc_3_right->text().toStdString().c_str() << endl;
       stream << "init_acc_4_right=" << ui->lineEdit_init_acc_4_right->text().toStdString().c_str() << endl;
       stream << "init_acc_5_right=" << ui->lineEdit_init_acc_5_right->text().toStdString().c_str() << endl;
       stream << "init_acc_6_right=" << ui->lineEdit_init_acc_6_right->text().toStdString().c_str() << endl;
       stream << "init_acc_7_right=" << ui->lineEdit_init_acc_7_right->text().toStdString().c_str() << endl;
       stream << "init_acc_8_right=" << ui->lineEdit_init_acc_8_right->text().toStdString().c_str() << endl;
       stream << "init_acc_9_right=" << ui->lineEdit_init_acc_9_right->text().toStdString().c_str() << endl;
       stream << "init_acc_10_right=" << ui->lineEdit_init_acc_10_right->text().toStdString().c_str() << endl;
       stream << "init_acc_11_right=" << ui->lineEdit_init_acc_11_right->text().toStdString().c_str() << endl;

       stream << "# Initial Acceleration (left)" << endl;
       stream << "init_acc_1_left=" << ui->lineEdit_init_acc_1_left->text().toStdString().c_str() << endl;
       stream << "init_acc_2_left=" << ui->lineEdit_init_acc_2_left->text().toStdString().c_str() << endl;
       stream << "init_acc_3_left=" << ui->lineEdit_init_acc_3_left->text().toStdString().c_str() << endl;
       stream << "init_acc_4_left=" << ui->lineEdit_init_acc_4_left->text().toStdString().c_str() << endl;
       stream << "init_acc_5_left=" << ui->lineEdit_init_acc_5_left->text().toStdString().c_str() << endl;
       stream << "init_acc_6_left=" << ui->lineEdit_init_acc_6_left->text().toStdString().c_str() << endl;
       stream << "init_acc_7_left=" << ui->lineEdit_init_acc_7_left->text().toStdString().c_str() << endl;
       stream << "init_acc_8_left=" << ui->lineEdit_init_acc_8_left->text().toStdString().c_str() << endl;
       stream << "init_acc_9_left=" << ui->lineEdit_init_acc_9_left->text().toStdString().c_str() << endl;
       stream << "init_acc_10_left=" << ui->lineEdit_init_acc_10_left->text().toStdString().c_str() << endl;
       stream << "init_acc_11_left=" << ui->lineEdit_init_acc_11_left->text().toStdString().c_str() << endl;

       stream << "# Final Acceleration (right)" << endl;
       stream << "final_acc_1_right=" << ui->lineEdit_final_acc_1_right->text().toStdString().c_str() << endl;
       stream << "final_acc_2_right=" << ui->lineEdit_final_acc_2_right->text().toStdString().c_str() << endl;
       stream << "final_acc_3_right=" << ui->lineEdit_final_acc_3_right->text().toStdString().c_str() << endl;
       stream << "final_acc_4_right=" << ui->lineEdit_final_acc_4_right->text().toStdString().c_str() << endl;
       stream << "final_acc_5_right=" << ui->lineEdit_final_acc_5_right->text().toStdString().c_str() << endl;
       stream << "final_acc_6_right=" << ui->lineEdit_final_acc_6_right->text().toStdString().c_str() << endl;
       stream << "final_acc_7_right=" << ui->lineEdit_final_acc_7_right->text().toStdString().c_str() << endl;
       stream << "final_acc_8_right=" << ui->lineEdit_final_acc_8_right->text().toStdString().c_str() << endl;
       stream << "final_acc_9_right=" << ui->lineEdit_final_acc_9_right->text().toStdString().c_str() << endl;
       stream << "final_acc_10_right=" << ui->lineEdit_final_acc_10_right->text().toStdString().c_str() << endl;
       stream << "final_acc_11_right=" << ui->lineEdit_final_acc_11_right->text().toStdString().c_str() << endl;

       stream << "# Final Acceleration (left)" << endl;
       stream << "final_acc_1_left=" << ui->lineEdit_final_acc_1_left->text().toStdString().c_str() << endl;
       stream << "final_acc_2_left=" << ui->lineEdit_final_acc_2_left->text().toStdString().c_str() << endl;
       stream << "final_acc_3_left=" << ui->lineEdit_final_acc_3_left->text().toStdString().c_str() << endl;
       stream << "final_acc_4_left=" << ui->lineEdit_final_acc_4_left->text().toStdString().c_str() << endl;
       stream << "final_acc_5_left=" << ui->lineEdit_final_acc_5_left->text().toStdString().c_str() << endl;
       stream << "final_acc_6_left=" << ui->lineEdit_final_acc_6_left->text().toStdString().c_str() << endl;
       stream << "final_acc_7_left=" << ui->lineEdit_final_acc_7_left->text().toStdString().c_str() << endl;
       stream << "final_acc_8_left=" << ui->lineEdit_final_acc_8_left->text().toStdString().c_str() << endl;
       stream << "final_acc_9_left=" << ui->lineEdit_final_acc_9_left->text().toStdString().c_str() << endl;
       stream << "final_acc_10_left=" << ui->lineEdit_final_acc_10_left->text().toStdString().c_str() << endl;
       stream << "final_acc_11_left=" << ui->lineEdit_final_acc_11_left->text().toStdString().c_str() << endl;

       stream << "# Tolerances with the target [mm] (right)" << endl;
       stream << "tar_xx_1_right=" << ui->lineEdit_tar_xx_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_xx_2_right=" << ui->lineEdit_tar_xx_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_xx_3_right=" << ui->lineEdit_tar_xx_3_right->text().toStdString().c_str()<< endl;
       stream << "tar_yy_1_right=" << ui->lineEdit_tar_yy_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_yy_2_right=" << ui->lineEdit_tar_yy_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_yy_3_right=" << ui->lineEdit_tar_yy_3_right->text().toStdString().c_str()<< endl;
       stream << "tar_zz_1_right=" << ui->lineEdit_tar_zz_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_zz_2_right=" << ui->lineEdit_tar_zz_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_zz_3_right=" << ui->lineEdit_tar_zz_3_right->text().toStdString().c_str()<< endl;
       stream << "tar_xy_1_right=" << ui->lineEdit_tar_xy_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_xy_2_right=" << ui->lineEdit_tar_xy_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_xy_3_right=" << ui->lineEdit_tar_xy_3_right->text().toStdString().c_str()<< endl;
       stream << "tar_xz_1_right=" << ui->lineEdit_tar_xz_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_xz_2_right=" << ui->lineEdit_tar_xz_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_xz_3_right=" << ui->lineEdit_tar_xz_3_right->text().toStdString().c_str()<< endl;
       stream << "tar_yz_1_right=" << ui->lineEdit_tar_yz_1_right->text().toStdString().c_str()<< endl;
       stream << "tar_yz_2_right=" << ui->lineEdit_tar_yz_2_right->text().toStdString().c_str()<< endl;
       stream << "tar_yz_3_right=" << ui->lineEdit_tar_yz_3_right->text().toStdString().c_str()<< endl;

       stream << "# Tolerances with the target [mm] (left)" << endl;
       stream << "tar_xx_1_left=" << ui->lineEdit_tar_xx_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_xx_2_left=" << ui->lineEdit_tar_xx_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_xx_3_left=" << ui->lineEdit_tar_xx_3_left->text().toStdString().c_str()<< endl;
       stream << "tar_yy_1_left=" << ui->lineEdit_tar_yy_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_yy_2_left=" << ui->lineEdit_tar_yy_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_yy_3_left=" << ui->lineEdit_tar_yy_3_left->text().toStdString().c_str()<< endl;
       stream << "tar_zz_1_left=" << ui->lineEdit_tar_zz_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_zz_2_left=" << ui->lineEdit_tar_zz_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_zz_3_left=" << ui->lineEdit_tar_zz_3_left->text().toStdString().c_str()<< endl;
       stream << "tar_xy_1_left=" << ui->lineEdit_tar_xy_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_xy_2_left=" << ui->lineEdit_tar_xy_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_xy_3_left=" << ui->lineEdit_tar_xy_3_left->text().toStdString().c_str()<< endl;
       stream << "tar_xz_1_left=" << ui->lineEdit_tar_xz_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_xz_2_left=" << ui->lineEdit_tar_xz_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_xz_3_left=" << ui->lineEdit_tar_xz_3_left->text().toStdString().c_str()<< endl;
       stream << "tar_yz_1_left=" << ui->lineEdit_tar_yz_1_left->text().toStdString().c_str()<< endl;
       stream << "tar_yz_2_left=" << ui->lineEdit_tar_yz_2_left->text().toStdString().c_str()<< endl;
       stream << "tar_yz_3_left=" << ui->lineEdit_tar_yz_3_left->text().toStdString().c_str()<< endl;


       stream << "# Tolerances with the obstacles [mm] (right)" << endl;
       stream << "obs_xx_1_right=" << ui->lineEdit_obs_xx_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_xx_2_right=" << ui->lineEdit_obs_xx_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_xx_3_right=" << ui->lineEdit_obs_xx_3_right->text().toStdString().c_str()<< endl;
       stream << "obs_yy_1_right=" << ui->lineEdit_obs_yy_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_yy_2_right=" << ui->lineEdit_obs_yy_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_yy_3_right=" << ui->lineEdit_obs_yy_3_right->text().toStdString().c_str()<< endl;
       stream << "obs_zz_1_right=" << ui->lineEdit_obs_zz_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_zz_2_right=" << ui->lineEdit_obs_zz_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_zz_3_right=" << ui->lineEdit_obs_zz_3_right->text().toStdString().c_str()<< endl;
       stream << "obs_xy_1_right=" << ui->lineEdit_obs_xy_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_xy_2_right=" << ui->lineEdit_obs_xy_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_xy_3_right=" << ui->lineEdit_obs_xy_3_right->text().toStdString().c_str()<< endl;
       stream << "obs_xz_1_right=" << ui->lineEdit_obs_xz_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_xz_2_right=" << ui->lineEdit_obs_xz_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_xz_3_right=" << ui->lineEdit_obs_xz_3_right->text().toStdString().c_str()<< endl;
       stream << "obs_yz_1_right=" << ui->lineEdit_obs_yz_1_right->text().toStdString().c_str()<< endl;
       stream << "obs_yz_2_right=" << ui->lineEdit_obs_yz_2_right->text().toStdString().c_str()<< endl;
       stream << "obs_yz_3_right=" << ui->lineEdit_obs_yz_3_right->text().toStdString().c_str()<< endl;

       stream << "# Tolerances with the obstacles [mm] (left)" << endl;
       stream << "obs_xx_1_left=" << ui->lineEdit_obs_xx_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_xx_2_left=" << ui->lineEdit_obs_xx_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_xx_3_left=" << ui->lineEdit_obs_xx_3_left->text().toStdString().c_str()<< endl;
       stream << "obs_yy_1_left=" << ui->lineEdit_obs_yy_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_yy_2_left=" << ui->lineEdit_obs_yy_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_yy_3_left=" << ui->lineEdit_obs_yy_3_left->text().toStdString().c_str()<< endl;
       stream << "obs_zz_1_left=" << ui->lineEdit_obs_zz_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_zz_2_left=" << ui->lineEdit_obs_zz_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_zz_3_left=" << ui->lineEdit_obs_zz_3_left->text().toStdString().c_str()<< endl;
       stream << "obs_xy_1_left=" << ui->lineEdit_obs_xy_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_xy_2_left=" << ui->lineEdit_obs_xy_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_xy_3_left=" << ui->lineEdit_obs_xy_3_left->text().toStdString().c_str()<< endl;
       stream << "obs_xz_1_left=" << ui->lineEdit_obs_xz_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_xz_2_left=" << ui->lineEdit_obs_xz_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_xz_3_left=" << ui->lineEdit_obs_xz_3_left->text().toStdString().c_str()<< endl;
       stream << "obs_yz_1_left=" << ui->lineEdit_obs_yz_1_left->text().toStdString().c_str()<< endl;
       stream << "obs_yz_2_left=" << ui->lineEdit_obs_yz_2_left->text().toStdString().c_str()<< endl;
       stream << "obs_yz_3_left=" << ui->lineEdit_obs_yz_3_left->text().toStdString().c_str()<< endl;

       stream << "# Tolerances for the final posture (right)" << endl;
       stream << "tar_pos_right=" << ui->lineEdit_tar_pos_right->text().toStdString().c_str()<< endl;
       stream << "tar_or_right="<< ui->lineEdit_tar_or_right->text().toStdString().c_str() << endl;

       stream << "# Tolerances for the final posture (left)" << endl;
       stream << "tar_pos_left=" << ui->lineEdit_tar_pos_left->text().toStdString().c_str()<< endl;
       stream << "tar_or_left="<< ui->lineEdit_tar_or_left->text().toStdString().c_str() << endl;

       stream << "# Pick settings (right)" << endl;
       stream << "pre_grasp_x_right="<< ui->lineEdit_pre_grasp_x_right->text().toStdString().c_str() << endl;
       stream << "pre_grasp_y_right="<< ui->lineEdit_pre_grasp_y_right->text().toStdString().c_str() << endl;
       stream << "pre_grasp_z_right="<< ui->lineEdit_pre_grasp_z_right->text().toStdString().c_str() << endl;
       stream << "pre_grasp_dist_right="<< ui->lineEdit_pre_grasp_dist_right->text().toStdString().c_str() << endl;
       stream << "post_grasp_x_right="<< ui->lineEdit_post_grasp_x_right->text().toStdString().c_str() << endl;
       stream << "post_grasp_y_right="<< ui->lineEdit_post_grasp_y_right->text().toStdString().c_str() << endl;
       stream << "post_grasp_z_right="<< ui->lineEdit_post_grasp_z_right->text().toStdString().c_str() << endl;
       stream << "post_grasp_dist_right="<< ui->lineEdit_post_grasp_dist_right->text().toStdString().c_str() << endl;

       stream << "# Pick settings (left)" << endl;
       stream << "pre_grasp_x_left="<< ui->lineEdit_pre_grasp_x_left->text().toStdString().c_str() << endl;
       stream << "pre_grasp_y_left="<< ui->lineEdit_pre_grasp_y_left->text().toStdString().c_str() << endl;
       stream << "pre_grasp_z_left="<< ui->lineEdit_pre_grasp_z_left->text().toStdString().c_str() << endl;
       stream << "pre_grasp_dist_left="<< ui->lineEdit_pre_grasp_dist_left->text().toStdString().c_str() << endl;
       stream << "post_grasp_x_left="<< ui->lineEdit_post_grasp_x_left->text().toStdString().c_str() << endl;
       stream << "post_grasp_y_left="<< ui->lineEdit_post_grasp_y_left->text().toStdString().c_str() << endl;
       stream << "post_grasp_z_left="<< ui->lineEdit_post_grasp_z_left->text().toStdString().c_str() << endl;
       stream << "post_grasp_dist_left="<< ui->lineEdit_post_grasp_dist_left->text().toStdString().c_str() << endl;

       stream << "# Place settings (right)" << endl;
       stream << "pre_place_x_right="<< ui->lineEdit_pre_place_x_right->text().toStdString().c_str() << endl;
       stream << "pre_place_y_right="<< ui->lineEdit_pre_place_y_right->text().toStdString().c_str() << endl;
       stream << "pre_place_z_right="<< ui->lineEdit_pre_place_z_right->text().toStdString().c_str() << endl;
       stream << "pre_place_dist_right="<< ui->lineEdit_pre_place_dist_right->text().toStdString().c_str() << endl;
       stream << "post_place_x_right="<< ui->lineEdit_post_place_x_right->text().toStdString().c_str() << endl;
       stream << "post_place_y_right="<< ui->lineEdit_post_place_y_right->text().toStdString().c_str() << endl;
       stream << "post_place_z_right="<< ui->lineEdit_post_place_z_right->text().toStdString().c_str() << endl;
       stream << "post_place_dist_right="<< ui->lineEdit_post_place_dist_right->text().toStdString().c_str() << endl;
       stream << "w_red_app_right=" << ui->lineEdit_w_red_app_right->text().toStdString().c_str() << endl;
       stream << "w_red_ret_right=" << ui->lineEdit_w_red_ret_right->text().toStdString().c_str() << endl;

       stream << "# Place settings (left)" << endl;
       stream << "pre_place_x_left="<< ui->lineEdit_pre_place_x_left->text().toStdString().c_str() << endl;
       stream << "pre_place_y_left="<< ui->lineEdit_pre_place_y_left->text().toStdString().c_str() << endl;
       stream << "pre_place_z_left="<< ui->lineEdit_pre_place_z_left->text().toStdString().c_str() << endl;
       stream << "pre_place_dist_left="<< ui->lineEdit_pre_place_dist_left->text().toStdString().c_str() << endl;
       stream << "post_place_x_left="<< ui->lineEdit_post_place_x_left->text().toStdString().c_str() << endl;
       stream << "post_place_y_left="<< ui->lineEdit_post_place_y_left->text().toStdString().c_str() << endl;
       stream << "post_place_z_left="<< ui->lineEdit_post_place_z_left->text().toStdString().c_str() << endl;
       stream << "post_place_dist_left="<< ui->lineEdit_post_place_dist_left->text().toStdString().c_str() << endl;
       stream << "w_red_app_left=" << ui->lineEdit_w_red_app_left->text().toStdString().c_str() << endl;
       stream << "w_red_ret_left=" << ui->lineEdit_w_red_ret_left->text().toStdString().c_str() << endl;

       stream << "# Move settings (right)" << endl;
       stream << "target_x_right=" << ui->lineEdit_target_x_right->text().toStdString().c_str() << endl;
       stream << "target_y_right=" << ui->lineEdit_target_y_right->text().toStdString().c_str() << endl;
       stream << "target_z_right=" << ui->lineEdit_target_z_right->text().toStdString().c_str() << endl;
       stream << "target_roll_right=" << ui->lineEdit_target_roll_right->text().toStdString().c_str() << endl;
       stream << "target_pitch_right=" << ui->lineEdit_target_pitch_right->text().toStdString().c_str() << endl;
       stream << "target_yaw_right=" << ui->lineEdit_target_yaw_right->text().toStdString().c_str() << endl;
       stream << "final_arm_1_right=" << ui->lineEdit_final_arm_1_right->text().toStdString().c_str() << endl;
       stream << "final_arm_2_right=" << ui->lineEdit_final_arm_2_right->text().toStdString().c_str() << endl;
       stream << "final_arm_3_right=" << ui->lineEdit_final_arm_3_right->text().toStdString().c_str() << endl;
       stream << "final_arm_4_right=" << ui->lineEdit_final_arm_4_right->text().toStdString().c_str() << endl;
       stream << "final_arm_5_right=" << ui->lineEdit_final_arm_5_right->text().toStdString().c_str() << endl;
       stream << "final_arm_6_right=" << ui->lineEdit_final_arm_6_right->text().toStdString().c_str() << endl;
       stream << "final_arm_7_right=" << ui->lineEdit_final_arm_7_right->text().toStdString().c_str() << endl;
       stream << "final_hand_1_right=" << ui->lineEdit_final_hand_1_right->text().toStdString().c_str() << endl;
       stream << "final_hand_2_right=" << ui->lineEdit_final_hand_2_right->text().toStdString().c_str() << endl;
       stream << "final_hand_3_right=" << ui->lineEdit_final_hand_3_right->text().toStdString().c_str() << endl;
       stream << "final_hand_4_right=" << ui->lineEdit_final_hand_4_right->text().toStdString().c_str() << endl;
       if (ui->checkBox_sel_final_posture_right->isChecked()){ stream << "sel_final_right=true"<< endl;}else{stream << "sel_final_right=false"<< endl;}

       stream << "# Move settings (left)" << endl;
       stream << "target_x_left=" << ui->lineEdit_target_x_left->text().toStdString().c_str() << endl;
       stream << "target_y_left=" << ui->lineEdit_target_y_left->text().toStdString().c_str() << endl;
       stream << "target_z_left=" << ui->lineEdit_target_z_left->text().toStdString().c_str() << endl;
       stream << "target_roll_left=" << ui->lineEdit_target_roll_left->text().toStdString().c_str() << endl;
       stream << "target_pitch_left=" << ui->lineEdit_target_pitch_left->text().toStdString().c_str() << endl;
       stream << "target_yaw_left=" << ui->lineEdit_target_yaw_left->text().toStdString().c_str() << endl;
       stream << "final_arm_1_left=" << ui->lineEdit_final_arm_1_left->text().toStdString().c_str() << endl;
       stream << "final_arm_2_left=" << ui->lineEdit_final_arm_2_left->text().toStdString().c_str() << endl;
       stream << "final_arm_3_left=" << ui->lineEdit_final_arm_3_left->text().toStdString().c_str() << endl;
       stream << "final_arm_4_left=" << ui->lineEdit_final_arm_4_left->text().toStdString().c_str() << endl;
       stream << "final_arm_5_left=" << ui->lineEdit_final_arm_5_left->text().toStdString().c_str() << endl;
       stream << "final_arm_6_left=" << ui->lineEdit_final_arm_6_left->text().toStdString().c_str() << endl;
       stream << "final_arm_7_left=" << ui->lineEdit_final_arm_7_left->text().toStdString().c_str() << endl;
       stream << "final_hand_1_left=" << ui->lineEdit_final_hand_1_left->text().toStdString().c_str() << endl;
       stream << "final_hand_2_left=" << ui->lineEdit_final_hand_2_left->text().toStdString().c_str() << endl;
       stream << "final_hand_3_left=" << ui->lineEdit_final_hand_3_left->text().toStdString().c_str() << endl;
       stream << "final_hand_4_left=" << ui->lineEdit_final_hand_4_left->text().toStdString().c_str() << endl;
       if (ui->checkBox_sel_final_posture_left->isChecked()){ stream << "sel_final_left=true"<< endl;}else{stream << "sel_final_left=false"<< endl;}


       stream << "plane_point1_x=" << ui->lineEdit_point_1_x->text().toStdString().c_str() << endl;
       stream << "plane_point1_y=" << ui->lineEdit_point_1_y->text().toStdString().c_str() << endl;
       stream << "plane_point1_z=" << ui->lineEdit_point_1_z->text().toStdString().c_str() << endl;
       stream << "plane_point2_x=" << ui->lineEdit_point_2_x->text().toStdString().c_str() << endl;
       stream << "plane_point2_y=" << ui->lineEdit_point_2_y->text().toStdString().c_str() << endl;
       stream << "plane_point2_z=" << ui->lineEdit_point_2_z->text().toStdString().c_str() << endl;
       stream << "plane_point3_x=" << ui->lineEdit_point_3_x->text().toStdString().c_str() << endl;
       stream << "plane_point3_y=" << ui->lineEdit_point_3_y->text().toStdString().c_str() << endl;
       stream << "plane_point3_z=" << ui->lineEdit_point_3_z->text().toStdString().c_str() << endl;
       if (ui->checkBox_add_plane->isChecked()){ stream << "add_plane=true"<< endl;}else{stream << "add_plane=false"<< endl;}       
       stream << "# Others" << endl;
       stream << "max_velocity="<< ui->lineEdit_w_max->text().toStdString().c_str() <<endl;
       stream << "max_acceleration="<< ui->lineEdit_alpha_max->text().toStdString().c_str() <<endl;
       if (ui->checkBox_tar_av->isChecked()){ stream << "tar_av=false"<< endl;}else{stream << "tar_av=true"<< endl;}
       if (ui->checkBox_ob_av->isChecked()){stream << "ob_av=false"<< endl;}else{stream << "ob_av=true"<< endl;}
       if(ui->checkBox_approach->isChecked()){stream << "approach=false"<<endl;}else{stream << "approach=true"<<endl;}
       if(ui->checkBox_retreat->isChecked()){stream << "retreat=false"<<endl;}else{stream << "retreat=true"<<endl;}
       if(ui->checkBox_rand_init->isChecked()){stream << "rand_init=true"<<endl;}else{stream << "rand_init=false"<<endl;}
       if(ui->checkBox_coll->isChecked()){stream << "coll=false"<<endl;}else{stream << "coll=true"<<endl;}

       if(ui->checkBox_straight_line_right->isChecked()){stream << "straight_line_right=true"<<endl;}else{stream << "straight_line_right=false"<<endl;}
       if(ui->checkBox_straight_line_left->isChecked()){stream << "straight_line_left=true"<<endl;}else{stream << "straight_line_left=false"<<endl;}
       //stream << "# END" << endl;


       f.close();
   }




}

void TolDialogHUMPDual::on_pushButton_load_clicked()
{

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Tols",
                                                    "All Files (*.*);; Tol Files (*.tol)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;

        while(!stream.atEnd()){

            line = f.readLine();
            if(line.at(0)!=QChar('#')){

                 //std::cout << line.toStdString().c_str();
                QStringList fields = line.split("=");

                //std::cout << fields.at(0).toStdString().c_str() << "\n";
                //std::cout << fields.at(1).toStdString().c_str() << "\n";

                if (QString::compare(fields.at(0),QString("Shoulder_radius_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_shoulder_r_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Elbow_radius_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_elbow_r_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Wrist_radius_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_wrist_r_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_radius_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_r_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_3_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("Shoulder_radius_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_shoulder_r_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Elbow_radius_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_elbow_r_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Wrist_radius_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_wrist_r_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_radius_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_r_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_3_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("lambda_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_7_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_8_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_8_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_9_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_9_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_10_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_10_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_11_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_11_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("lambda_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_7_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_8_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_8_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_9_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_9_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_10_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_10_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_11_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_11_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("init_vel_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_7_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_8_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_8_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_9_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_9_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_10_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_10_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_11_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_11_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("init_vel_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_7_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_8_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_8_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_9_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_9_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_10_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_10_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_11_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_11_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_vel_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_7_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_8_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_8_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_9_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_9_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_10_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_10_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_11_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_11_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_vel_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_7_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_8_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_8_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_9_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_9_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_10_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_10_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_11_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_11_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("init_acc_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_7_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_8_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_8_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_9_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_9_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_10_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_10_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_11_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_11_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("init_acc_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_7_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_8_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_8_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_9_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_9_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_10_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_10_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_11_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_11_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_acc_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_7_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_8_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_8_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_9_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_9_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_10_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_10_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_11_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_11_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_acc_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_7_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_8_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_8_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_9_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_9_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_10_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_10_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_11_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_11_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("tar_xx_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_3_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("tar_xx_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_3_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("obs_xx_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_3_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("obs_xx_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_3_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("tar_pos_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_pos_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_or_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_or_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("tar_pos_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_pos_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_or_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_or_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("pre_grasp_x_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_x_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_y_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_y_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_z_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_z_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_dist_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_dist_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("pre_grasp_x_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_x_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_y_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_y_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_z_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_z_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_dist_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_dist_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("post_grasp_x_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_x_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_y_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_y_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_z_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_z_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_dist_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_dist_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("post_grasp_x_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_x_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_y_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_y_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_z_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_z_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_dist_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_dist_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("pre_place_x_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_x_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_y_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_y_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_z_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_z_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_dist_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_dist_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("pre_place_x_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_x_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_y_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_y_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_z_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_z_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_dist_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_dist_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("post_place_x_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_x_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_y_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_y_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_z_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_z_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_dist_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_dist_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("post_place_x_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_x_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_y_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_y_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_z_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_z_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_dist_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_dist_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("w_red_app_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_app_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("w_red_ret_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_ret_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("w_red_app_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_app_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("w_red_ret_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_ret_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("target_x_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_x_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_y_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_y_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_z_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_z_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_roll_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_roll_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_pitch_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_pitch_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_yaw_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_yaw_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("target_x_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_x_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_y_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_y_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_z_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_z_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_roll_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_roll_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_pitch_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_pitch_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_yaw_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_yaw_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_arm_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_4_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_5_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_5_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_6_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_6_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_7_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_7_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_arm_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_4_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_5_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_5_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_6_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_6_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_7_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_7_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_hand_1_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_1_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_2_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_2_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_3_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_3_right->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_4_right"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_4_right->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("final_hand_1_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_1_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_2_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_2_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_3_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_3_left->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_4_left"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_4_left->setText(fields.at(1));

                }else if(QString::compare(fields.at(0),QString("sel_final_right"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_sel_final_posture_right->setChecked(false);
                    }else{
                        ui->checkBox_sel_final_posture_right->setChecked(true);
                    }
                }else if(QString::compare(fields.at(0),QString("sel_final_left"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_sel_final_posture_left->setChecked(false);
                    }else{
                        ui->checkBox_sel_final_posture_left->setChecked(true);
                    }

                }else if(QString::compare(fields.at(0),QString("plane_point1_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point1_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point1_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("add_plane"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_add_plane->setChecked(false);
                    }else{
                        ui->checkBox_add_plane->setChecked(true);
                    }
                }else if(QString::compare(fields.at(0),QString("max_velocity"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_max->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("max_acceleration"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_alpha_max->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_tar_av->setChecked(true);
                    }else{
                        ui->checkBox_tar_av->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("ob_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_ob_av->setChecked(true);
                    }else{
                        ui->checkBox_ob_av->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("approach"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_approach->setChecked(true);
                    }else{
                        ui->checkBox_approach->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("retreat"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_retreat->setChecked(true);
                    }else{
                        ui->checkBox_retreat->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("rand_init"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_rand_init->setChecked(false);
                    }else{
                        ui->checkBox_rand_init->setChecked(true);
                    }
                }else if(QString::compare(fields.at(0),QString("coll"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_coll->setChecked(true);
                    }else{
                        ui->checkBox_coll->setChecked(false);
                    }

                }else if(QString::compare(fields.at(0),QString("straight_line_right"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_straight_line_right->setChecked(false);
                    }else{
                        ui->checkBox_straight_line_right->setChecked(true);
                    }

                }else if(QString::compare(fields.at(0),QString("straight_line_left"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_straight_line_left->setChecked(false);
                    }else{
                        ui->checkBox_straight_line_left->setChecked(true);
                    }

                }

            }

        }

        f.close();

    }

}

void TolDialogHUMPDual::checkApproach(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_pre_grasp_right->setEnabled(true);
        ui->groupBox_pre_place_right->setEnabled(true);
        //ui->label_pick->setEnabled(true);

        ui->groupBox_pre_grasp_left->setEnabled(true);
        ui->groupBox_pre_place_left->setEnabled(true);

    }else{
        //checked
        ui->groupBox_pre_grasp_right->setEnabled(false);
        ui->groupBox_pre_place_right->setEnabled(false);
        //ui->label_pick->setEnabled(false);

        ui->groupBox_pre_grasp_left->setEnabled(false);
        ui->groupBox_pre_place_left->setEnabled(false);
    }
}

void TolDialogHUMPDual::checkRetreat(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_post_grasp_right->setEnabled(true);
        ui->groupBox_post_place_right->setEnabled(true);
        //ui->label_pick->setEnabled(true);

        ui->groupBox_post_grasp_left->setEnabled(true);
        ui->groupBox_post_place_left->setEnabled(true);

    }else{
        //checked
        ui->groupBox_post_grasp_right->setEnabled(false);
        ui->groupBox_post_place_right->setEnabled(false);
        //ui->label_pick->setEnabled(false);

        ui->groupBox_post_grasp_left->setEnabled(false);
        ui->groupBox_post_place_left->setEnabled(false);
    }
}

void TolDialogHUMPDual::checkFinalPostureRight(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_target_right->setEnabled(true);
        ui->groupBox_final_arm_right->setEnabled(false);
    }else{
        //checked
        ui->groupBox_target_right->setEnabled(false);
        ui->groupBox_final_arm_right->setEnabled(true);
    }
}

void TolDialogHUMPDual::checkFinalPostureLeft(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_target_left->setEnabled(true);
        ui->groupBox_final_arm_left->setEnabled(false);
    }else{
        //checked
        ui->groupBox_target_left->setEnabled(false);
        ui->groupBox_final_arm_left->setEnabled(true);
    }
}

void TolDialogHUMPDual::checkAddPlane(int state)
{
    if(state==0){
        //unchecked
        ui->groupBox_plane->setEnabled(false);
    }else{
        //checked
        ui->groupBox_plane->setEnabled(true);
    }
}

    /*
void TolDialogHUMPDual::checkSetHandCond(int state)
{

    if(state==0){
        // -- unchecked -- //
        ui->groupBox_init_hand_vel->setEnabled(false);
        ui->groupBox_final_hand_vel->setEnabled(false);
        //ui->groupBox_init_hand_acc->setEnabled(false);
        //ui->groupBox_final_hand_acc->setEnabled(false);
    }else{
        // -- checked -- //
        ui->groupBox_init_hand_vel->setEnabled(true);
        ui->groupBox_final_hand_vel->setEnabled(true);
        //ui->groupBox_init_hand_acc->setEnabled(true);
        //ui->groupBox_final_hand_acc->setEnabled(true);

        // init vel
        ui->lineEdit_init_vel_1->setEnabled(false);
        ui->lineEdit_init_vel_2->setEnabled(false);
        ui->lineEdit_init_vel_3->setEnabled(false);
        ui->lineEdit_init_vel_4->setEnabled(false);
        ui->lineEdit_init_vel_5->setEnabled(false);
        ui->lineEdit_init_vel_6->setEnabled(false);
        ui->lineEdit_init_vel_7->setEnabled(false);
        // final vel
        ui->lineEdit_final_vel_1->setEnabled(false);
        ui->lineEdit_final_vel_2->setEnabled(false);
        ui->lineEdit_final_vel_3->setEnabled(false);
        ui->lineEdit_final_vel_4->setEnabled(false);
        ui->lineEdit_final_vel_5->setEnabled(false);
        ui->lineEdit_final_vel_6->setEnabled(false);
        ui->lineEdit_final_vel_7->setEnabled(false);
        // init acc
        //ui->lineEdit_init_acc_1->setEnabled(false);
        //ui->lineEdit_init_acc_2->setEnabled(false);
        //ui->lineEdit_init_acc_3->setEnabled(false);
        //ui->lineEdit_init_acc_4->setEnabled(false);
        //ui->lineEdit_init_acc_5->setEnabled(false);
        //ui->lineEdit_init_acc_6->setEnabled(false);
        //ui->lineEdit_init_acc_7->setEnabled(false);
        // final acc
        //ui->lineEdit_final_acc_1->setEnabled(false);
        //ui->lineEdit_final_acc_2->setEnabled(false);
        //ui->lineEdit_final_acc_3->setEnabled(false);
        //ui->lineEdit_final_acc_4->setEnabled(false);
        //ui->lineEdit_final_acc_5->setEnabled(false);
        //ui->lineEdit_final_acc_6->setEnabled(false);
        //ui->lineEdit_final_acc_7->setEnabled(false);
    }

}
    */
/*
void TolDialogHUMPDual::checkSetHandCondApproach(int state)
{

}
*/

bool TolDialogHUMPDual::getRandInit()
{
    return ui->checkBox_rand_init->isChecked();
}

void TolDialogHUMPDual::setRandInit(bool rand)
{
    ui->checkBox_rand_init->setChecked(rand);
}

bool TolDialogHUMPDual::getColl()
{
    return !ui->checkBox_coll->isChecked();
}

void TolDialogHUMPDual::setColl(bool coll)
{
    ui->checkBox_coll->setChecked(!coll);
}

bool TolDialogHUMPDual::get_use_final_posture_right()
{
    return ui->checkBox_sel_final_posture_right->isChecked();
}

bool TolDialogHUMPDual::get_use_final_posture_left()
{
    return ui->checkBox_sel_final_posture_left->isChecked();
}

bool TolDialogHUMPDual::get_add_plane()
{
    return ui->checkBox_add_plane->isChecked();
}

bool TolDialogHUMPDual::get_straight_line_right()
{
    return ui->checkBox_straight_line_right->isChecked();
}

bool TolDialogHUMPDual::get_straight_line_left()
{
    return ui->checkBox_straight_line_left->isChecked();
}

void TolDialogHUMPDual::setStraightLineRight(bool straight)
{
    ui->checkBox_straight_line_right->setChecked(straight);
}

void TolDialogHUMPDual::setStraightLineLeft(bool straight)
{
    ui->checkBox_straight_line_left->setChecked(straight);
}

void TolDialogHUMPDual::set_add_plane(bool plane)
{
    ui->checkBox_add_plane->setChecked(plane);
}


} // namespace motion_manager
