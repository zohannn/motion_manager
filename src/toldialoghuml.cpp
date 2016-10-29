#include "../include/motion_manager/toldialoghuml.hpp"


namespace motion_manager {

using namespace Qt;

TolDialogHUML::TolDialogHUML(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TolDialogHUML)
{
    ui->setupUi(this);

     QObject::connect(ui->checkBox_approach, SIGNAL(stateChanged(int)), this, SLOT(checkApproach(int)));

    if(ui->checkBox_approach->isChecked()){
        ui->groupBox_pre_grasp->setEnabled(false);
        ui->groupBox_pre_place->setEnabled(false);
    }
    if(ui->checkBox_retreat->isChecked()){
        ui->groupBox_post_grasp->setEnabled(false);
        ui->groupBox_post_place->setEnabled(false);
    }
}

TolDialogHUML::~TolDialogHUML()
{
    delete ui;
}



double TolDialogHUML::getTolStop()
{

    return ui->lineEdit_tol_stop->text().toDouble();
}


void TolDialogHUML::getTolsArm(vector<double> &tols)
{
    tols.clear();
    tols.push_back(ui->lineEdit_shoulder_r->text().toDouble());
    tols.push_back(ui->lineEdit_elbow_r->text().toDouble());
    tols.push_back(ui->lineEdit_wrist_r->text().toDouble());
    tols.push_back(ui->lineEdit_hand_r->text().toDouble());
}


void TolDialogHUML::getTolsHand(MatrixXd &tols)
{

   tols = MatrixXd::Constant(4,3,1);

    tols(0,0) = ui->lineEdit_hand_1_1->text().toDouble(); tols(0,1) = ui->lineEdit_hand_1_2->text().toDouble(); tols(0,2) = ui->lineEdit_hand_1_3->text().toDouble();
    tols(1,0) = ui->lineEdit_hand_2_1->text().toDouble(); tols(1,1) = ui->lineEdit_hand_2_2->text().toDouble(); tols(1,2) = ui->lineEdit_hand_2_3->text().toDouble();
    tols(2,0) = ui->lineEdit_hand_3_1->text().toDouble(); tols(2,1) = ui->lineEdit_hand_3_2->text().toDouble(); tols(2,2) = ui->lineEdit_hand_3_3->text().toDouble();
    tols(3,0) = ui->lineEdit_hand_tip_1->text().toDouble(); tols(3,1) = ui->lineEdit_hand_tip_2->text().toDouble(); tols(3,2) = ui->lineEdit_hand_tip_3->text().toDouble();
}



void TolDialogHUML::getLambda(std::vector<double> &lambda)
{

   lambda.clear();
   lambda.push_back(ui->lineEdit_lambda_1->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_2->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_3->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_4->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_5->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_6->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_7->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_8->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_9->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_10->text().toDouble());
   lambda.push_back(ui->lineEdit_lambda_11->text().toDouble());

}


void TolDialogHUML::getTolsObstacles(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_obs_xx_1->text().toDouble(); tols(0,1)=ui->lineEdit_obs_yy_1->text().toDouble(); tols(0,2)=ui->lineEdit_obs_zz_1->text().toDouble(); tols(0,3)=ui->lineEdit_obs_xy_1->text().toDouble(); tols(0,4)=ui->lineEdit_obs_xz_1->text().toDouble(); tols(0,5)=ui->lineEdit_obs_yz_1->text().toDouble();
    tols(1,0)=ui->lineEdit_obs_xx_2->text().toDouble(); tols(1,1)=ui->lineEdit_obs_yy_2->text().toDouble(); tols(1,2)=ui->lineEdit_obs_zz_2->text().toDouble(); tols(1,3)=ui->lineEdit_obs_xy_2->text().toDouble(); tols(1,4)=ui->lineEdit_obs_xz_2->text().toDouble(); tols(1,5)=ui->lineEdit_obs_yz_2->text().toDouble();
    tols(2,0)=ui->lineEdit_obs_xx_3->text().toDouble(); tols(2,1)=ui->lineEdit_obs_yy_3->text().toDouble(); tols(2,2)=ui->lineEdit_obs_zz_3->text().toDouble(); tols(2,3)=ui->lineEdit_obs_xy_3->text().toDouble(); tols(2,4)=ui->lineEdit_obs_xz_3->text().toDouble(); tols(2,5)=ui->lineEdit_obs_yz_3->text().toDouble();

}


void TolDialogHUML::getTolsTarget(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_tar_xx_1->text().toDouble(); tols(0,1)=ui->lineEdit_tar_yy_1->text().toDouble(); tols(0,2)=ui->lineEdit_tar_zz_1->text().toDouble(); tols(0,3)=ui->lineEdit_tar_xy_1->text().toDouble(); tols(0,4)=ui->lineEdit_tar_xz_1->text().toDouble(); tols(0,5)=ui->lineEdit_tar_yz_1->text().toDouble();
    tols(1,0)=ui->lineEdit_tar_xx_2->text().toDouble(); tols(1,1)=ui->lineEdit_tar_yy_2->text().toDouble(); tols(1,2)=ui->lineEdit_tar_zz_2->text().toDouble(); tols(1,3)=ui->lineEdit_tar_xy_2->text().toDouble(); tols(1,4)=ui->lineEdit_tar_xz_2->text().toDouble(); tols(1,5)=ui->lineEdit_tar_yz_2->text().toDouble();
    tols(2,0)=ui->lineEdit_tar_xx_3->text().toDouble(); tols(2,1)=ui->lineEdit_tar_yy_3->text().toDouble(); tols(2,2)=ui->lineEdit_tar_zz_3->text().toDouble(); tols(2,3)=ui->lineEdit_tar_xy_3->text().toDouble(); tols(2,4)=ui->lineEdit_tar_xz_3->text().toDouble(); tols(2,5)=ui->lineEdit_tar_yz_3->text().toDouble();


}


int TolDialogHUML::getSteps()
{

    return ui->lineEdit_steps->text().toInt();
}


double TolDialogHUML::getWMax()
{

    return ui->lineEdit_w_max->text().toDouble();
}




double TolDialogHUML::getTolTarPos()
{

    return ui->lineEdit_tar_pos->text().toDouble();
}


double TolDialogHUML::getTolTarOr()
{

    return ui->lineEdit_tar_or->text().toDouble();
}

void TolDialogHUML::setInfo(string info)
{

    this->infoLine = info;
}

bool TolDialogHUML::getTargetAvoidance()
{

   return !ui->checkBox_tar_av->isChecked();
}

bool TolDialogHUML::getObstacleAvoidance()
{

    return !ui->checkBox_ob_av->isChecked();
}

bool TolDialogHUML::getApproach()
{
    return !ui->checkBox_approach->isChecked();
}

bool TolDialogHUML::getRetreat()
{
    return !ui->checkBox_retreat->isChecked();
}

void TolDialogHUML::getInitVel(std::vector<double> &init_vel)
{
    init_vel.clear();
    init_vel.push_back(ui->lineEdit_init_vel_1->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_2->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_3->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_4->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_5->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_6->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_7->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_8->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_9->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_10->text().toDouble());
    init_vel.push_back(ui->lineEdit_init_vel_11->text().toDouble());
}

void TolDialogHUML::getFinalVel(std::vector<double> &final_vel)
{
    final_vel.clear();
    final_vel.push_back(ui->lineEdit_final_vel_1->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_2->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_3->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_4->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_5->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_6->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_7->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_8->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_9->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_10->text().toDouble());
    final_vel.push_back(ui->lineEdit_final_vel_11->text().toDouble());
}

void TolDialogHUML::getInitAcc(std::vector<double> &init_acc)
{
    init_acc.clear();
    init_acc.push_back(ui->lineEdit_init_acc_1->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_2->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_3->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_4->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_5->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_6->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_7->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_8->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_9->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_10->text().toDouble());
    init_acc.push_back(ui->lineEdit_init_acc_11->text().toDouble());
}

void TolDialogHUML::getFinalAcc(std::vector<double> &final_acc)
{
    final_acc.clear();
    final_acc.push_back(ui->lineEdit_final_acc_1->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_2->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_3->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_4->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_5->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_6->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_7->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_8->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_9->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_10->text().toDouble());
    final_acc.push_back(ui->lineEdit_final_acc_11->text().toDouble());
}

void TolDialogHUML::getPreGraspApproach(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist->text().toDouble());
}

void TolDialogHUML::getPostGraspRetreat(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist->text().toDouble());
}

void TolDialogHUML::getPrePlaceApproach(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist->text().toDouble());

}

void TolDialogHUML::getPostPlaceRetreat(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist->text().toDouble());
}


// Q_SLOTS

void TolDialogHUML::on_pushButton_save_clicked()
{

   QString filename = QFileDialog::getSaveFileName(this,
                                                   tr("Save the file of tolerances"),
                                                   QString(MAIN_PATH)+"/Tols",
                                                   "All Files (*.*);;Tol Files (*.tol)");
   QFile f( filename );
   if(f.open( QIODevice::WriteOnly )){

       QTextStream stream( &f );
       stream << "### Tolerances for the Human-like Upper-limbs Motion Library ###" << endl;
       stream << "# "<< this->infoLine.c_str() << endl;
       stream << "# Geometric Dimensions of the arms and of the fingers" << endl;
       stream << "Shoulder_radius=" << ui->lineEdit_shoulder_r->text().toStdString().c_str() << endl;
       stream << "Elbow_radius=" << ui->lineEdit_elbow_r->text().toStdString().c_str() << endl;
       stream << "Wrist_radius=" << ui->lineEdit_wrist_r->text().toStdString().c_str() << endl;
       stream << "Hand_radius=" << ui->lineEdit_hand_r->text().toStdString().c_str() << endl;
       stream << "Hand_1_1=" << ui->lineEdit_hand_1_1->text().toStdString().c_str() << endl;
       stream << "Hand_1_2=" << ui->lineEdit_hand_1_2->text().toStdString().c_str() << endl;
       stream << "Hand_1_3=" << ui->lineEdit_hand_1_3->text().toStdString().c_str() << endl;
       stream << "Hand_2_1=" << ui->lineEdit_hand_2_1->text().toStdString().c_str() << endl;
       stream << "Hand_2_2=" << ui->lineEdit_hand_2_2->text().toStdString().c_str() << endl;
       stream << "Hand_2_3=" << ui->lineEdit_hand_2_3->text().toStdString().c_str() << endl;
       stream << "Hand_3_1=" << ui->lineEdit_hand_3_1->text().toStdString().c_str() << endl;
       stream << "Hand_3_2=" << ui->lineEdit_hand_3_2->text().toStdString().c_str() << endl;
       stream << "Hand_3_3=" << ui->lineEdit_hand_3_3->text().toStdString().c_str() << endl;
       stream << "Hand_tip_1=" << ui->lineEdit_hand_tip_1->text().toStdString().c_str() << endl;
       stream << "Hand_tip_2=" << ui->lineEdit_hand_tip_2->text().toStdString().c_str() << endl;
       stream << "Hand_tip_3=" << ui->lineEdit_hand_tip_3->text().toStdString().c_str() << endl;
       stream << "# Joint Expanse factors" << endl;
       stream << "lambda_1=" << ui->lineEdit_lambda_1->text().toStdString().c_str() << endl;
       stream << "lambda_2=" << ui->lineEdit_lambda_2->text().toStdString().c_str() << endl;
       stream << "lambda_3=" << ui->lineEdit_lambda_3->text().toStdString().c_str() << endl;
       stream << "lambda_4=" << ui->lineEdit_lambda_4->text().toStdString().c_str() << endl;
       stream << "lambda_5=" << ui->lineEdit_lambda_5->text().toStdString().c_str() << endl;
       stream << "lambda_6=" << ui->lineEdit_lambda_6->text().toStdString().c_str() << endl;
       stream << "lambda_7=" << ui->lineEdit_lambda_7->text().toStdString().c_str() << endl;
       stream << "lambda_8=" << ui->lineEdit_lambda_8->text().toStdString().c_str() << endl;
       stream << "lambda_9=" << ui->lineEdit_lambda_9->text().toStdString().c_str() << endl;
       stream << "lambda_10=" << ui->lineEdit_lambda_10->text().toStdString().c_str() << endl;
       stream << "lambda_11=" << ui->lineEdit_lambda_11->text().toStdString().c_str() << endl;
       stream << "# Initial Velocity " << endl;
       stream << "init_vel_1=" << ui->lineEdit_init_vel_1->text().toStdString().c_str() << endl;
       stream << "init_vel_2=" << ui->lineEdit_init_vel_2->text().toStdString().c_str() << endl;
       stream << "init_vel_3=" << ui->lineEdit_init_vel_3->text().toStdString().c_str() << endl;
       stream << "init_vel_4=" << ui->lineEdit_init_vel_4->text().toStdString().c_str() << endl;
       stream << "init_vel_5=" << ui->lineEdit_init_vel_5->text().toStdString().c_str() << endl;
       stream << "init_vel_6=" << ui->lineEdit_init_vel_6->text().toStdString().c_str() << endl;
       stream << "init_vel_7=" << ui->lineEdit_init_vel_7->text().toStdString().c_str() << endl;
       stream << "init_vel_8=" << ui->lineEdit_init_vel_8->text().toStdString().c_str() << endl;
       stream << "init_vel_9=" << ui->lineEdit_init_vel_9->text().toStdString().c_str() << endl;
       stream << "init_vel_10=" << ui->lineEdit_init_vel_10->text().toStdString().c_str() << endl;
       stream << "init_vel_11=" << ui->lineEdit_init_vel_11->text().toStdString().c_str() << endl;
       stream << "# Final Velocity " << endl;
       stream << "final_vel_1=" << ui->lineEdit_final_vel_1->text().toStdString().c_str() << endl;
       stream << "final_vel_2=" << ui->lineEdit_final_vel_2->text().toStdString().c_str() << endl;
       stream << "final_vel_3=" << ui->lineEdit_final_vel_3->text().toStdString().c_str() << endl;
       stream << "final_vel_4=" << ui->lineEdit_final_vel_4->text().toStdString().c_str() << endl;
       stream << "final_vel_5=" << ui->lineEdit_final_vel_5->text().toStdString().c_str() << endl;
       stream << "final_vel_6=" << ui->lineEdit_final_vel_6->text().toStdString().c_str() << endl;
       stream << "final_vel_7=" << ui->lineEdit_final_vel_7->text().toStdString().c_str() << endl;
       stream << "final_vel_8=" << ui->lineEdit_final_vel_8->text().toStdString().c_str() << endl;
       stream << "final_vel_9=" << ui->lineEdit_final_vel_9->text().toStdString().c_str() << endl;
       stream << "final_vel_10=" << ui->lineEdit_final_vel_10->text().toStdString().c_str() << endl;
       stream << "final_vel_11=" << ui->lineEdit_final_vel_11->text().toStdString().c_str() << endl;
       stream << "# Initial Acceleration " << endl;
       stream << "init_acc_1=" << ui->lineEdit_init_acc_1->text().toStdString().c_str() << endl;
       stream << "init_acc_2=" << ui->lineEdit_init_acc_2->text().toStdString().c_str() << endl;
       stream << "init_acc_3=" << ui->lineEdit_init_acc_3->text().toStdString().c_str() << endl;
       stream << "init_acc_4=" << ui->lineEdit_init_acc_4->text().toStdString().c_str() << endl;
       stream << "init_acc_5=" << ui->lineEdit_init_acc_5->text().toStdString().c_str() << endl;
       stream << "init_acc_6=" << ui->lineEdit_init_acc_6->text().toStdString().c_str() << endl;
       stream << "init_acc_7=" << ui->lineEdit_init_acc_7->text().toStdString().c_str() << endl;
       stream << "init_acc_8=" << ui->lineEdit_init_acc_8->text().toStdString().c_str() << endl;
       stream << "init_acc_9=" << ui->lineEdit_init_acc_9->text().toStdString().c_str() << endl;
       stream << "init_acc_10=" << ui->lineEdit_init_acc_10->text().toStdString().c_str() << endl;
       stream << "init_acc_11=" << ui->lineEdit_init_acc_11->text().toStdString().c_str() << endl;
       stream << "# Final Acceleration " << endl;
       stream << "final_acc_1=" << ui->lineEdit_final_acc_1->text().toStdString().c_str() << endl;
       stream << "final_acc_2=" << ui->lineEdit_final_acc_2->text().toStdString().c_str() << endl;
       stream << "final_acc_3=" << ui->lineEdit_final_acc_3->text().toStdString().c_str() << endl;
       stream << "final_acc_4=" << ui->lineEdit_final_acc_4->text().toStdString().c_str() << endl;
       stream << "final_acc_5=" << ui->lineEdit_final_acc_5->text().toStdString().c_str() << endl;
       stream << "final_acc_6=" << ui->lineEdit_final_acc_6->text().toStdString().c_str() << endl;
       stream << "final_acc_7=" << ui->lineEdit_final_acc_7->text().toStdString().c_str() << endl;
       stream << "final_acc_8=" << ui->lineEdit_final_acc_8->text().toStdString().c_str() << endl;
       stream << "final_acc_9=" << ui->lineEdit_final_acc_9->text().toStdString().c_str() << endl;
       stream << "final_acc_10=" << ui->lineEdit_final_acc_10->text().toStdString().c_str() << endl;
       stream << "final_acc_11=" << ui->lineEdit_final_acc_11->text().toStdString().c_str() << endl;
       stream << "# Tolerances with the target [mm]" << endl;
       stream << "tar_xx_1=" << ui->lineEdit_tar_xx_1->text().toStdString().c_str()<< endl;
       stream << "tar_xx_2=" << ui->lineEdit_tar_xx_2->text().toStdString().c_str()<< endl;
       stream << "tar_xx_3=" << ui->lineEdit_tar_xx_3->text().toStdString().c_str()<< endl;
       stream << "tar_yy_1=" << ui->lineEdit_tar_yy_1->text().toStdString().c_str()<< endl;
       stream << "tar_yy_2=" << ui->lineEdit_tar_yy_2->text().toStdString().c_str()<< endl;
       stream << "tar_yy_3=" << ui->lineEdit_tar_yy_3->text().toStdString().c_str()<< endl;
       stream << "tar_zz_1=" << ui->lineEdit_tar_zz_1->text().toStdString().c_str()<< endl;
       stream << "tar_zz_2=" << ui->lineEdit_tar_zz_2->text().toStdString().c_str()<< endl;
       stream << "tar_zz_3=" << ui->lineEdit_tar_zz_3->text().toStdString().c_str()<< endl;
       stream << "tar_xy_1=" << ui->lineEdit_tar_xy_1->text().toStdString().c_str()<< endl;
       stream << "tar_xy_2=" << ui->lineEdit_tar_xy_2->text().toStdString().c_str()<< endl;
       stream << "tar_xy_3=" << ui->lineEdit_tar_xy_3->text().toStdString().c_str()<< endl;
       stream << "tar_xz_1=" << ui->lineEdit_tar_xz_1->text().toStdString().c_str()<< endl;
       stream << "tar_xz_2=" << ui->lineEdit_tar_xz_2->text().toStdString().c_str()<< endl;
       stream << "tar_xz_3=" << ui->lineEdit_tar_xz_3->text().toStdString().c_str()<< endl;
       stream << "tar_yz_1=" << ui->lineEdit_tar_yz_1->text().toStdString().c_str()<< endl;
       stream << "tar_yz_2=" << ui->lineEdit_tar_yz_2->text().toStdString().c_str()<< endl;
       stream << "tar_yz_3=" << ui->lineEdit_tar_yz_3->text().toStdString().c_str()<< endl;
       stream << "# Tolerances with the obstacles [mm]" << endl;
       stream << "obs_xx_1=" << ui->lineEdit_obs_xx_1->text().toStdString().c_str()<< endl;
       stream << "obs_xx_2=" << ui->lineEdit_obs_xx_2->text().toStdString().c_str()<< endl;
       stream << "obs_xx_3=" << ui->lineEdit_obs_xx_3->text().toStdString().c_str()<< endl;
       stream << "obs_yy_1=" << ui->lineEdit_obs_yy_1->text().toStdString().c_str()<< endl;
       stream << "obs_yy_2=" << ui->lineEdit_obs_yy_2->text().toStdString().c_str()<< endl;
       stream << "obs_yy_3=" << ui->lineEdit_obs_yy_3->text().toStdString().c_str()<< endl;
       stream << "obs_zz_1=" << ui->lineEdit_obs_zz_1->text().toStdString().c_str()<< endl;
       stream << "obs_zz_2=" << ui->lineEdit_obs_zz_2->text().toStdString().c_str()<< endl;
       stream << "obs_zz_3=" << ui->lineEdit_obs_zz_3->text().toStdString().c_str()<< endl;
       stream << "obs_xy_1=" << ui->lineEdit_obs_xy_1->text().toStdString().c_str()<< endl;
       stream << "obs_xy_2=" << ui->lineEdit_obs_xy_2->text().toStdString().c_str()<< endl;
       stream << "obs_xy_3=" << ui->lineEdit_obs_xy_3->text().toStdString().c_str()<< endl;
       stream << "obs_xz_1=" << ui->lineEdit_obs_xz_1->text().toStdString().c_str()<< endl;
       stream << "obs_xz_2=" << ui->lineEdit_obs_xz_2->text().toStdString().c_str()<< endl;
       stream << "obs_xz_3=" << ui->lineEdit_obs_xz_3->text().toStdString().c_str()<< endl;
       stream << "obs_yz_1=" << ui->lineEdit_obs_yz_1->text().toStdString().c_str()<< endl;
       stream << "obs_yz_2=" << ui->lineEdit_obs_yz_2->text().toStdString().c_str()<< endl;
       stream << "obs_yz_3=" << ui->lineEdit_obs_yz_3->text().toStdString().c_str()<< endl;
       stream << "# Tolerances for the final posture" << endl;
       stream << "tar_pos=" << ui->lineEdit_tar_pos->text().toStdString().c_str()<< endl;
       stream << "tar_or="<< ui->lineEdit_tar_or->text().toStdString().c_str() << endl;
       stream << "# Pick settings" << endl;
       stream << "pre_grasp_x="<< ui->lineEdit_pre_grasp_x->text().toStdString().c_str() << endl;
       stream << "pre_grasp_y="<< ui->lineEdit_pre_grasp_y->text().toStdString().c_str() << endl;
       stream << "pre_grasp_z="<< ui->lineEdit_pre_grasp_z->text().toStdString().c_str() << endl;
       stream << "pre_grasp_dist="<< ui->lineEdit_pre_grasp_dist->text().toStdString().c_str() << endl;
       stream << "post_grasp_x="<< ui->lineEdit_post_grasp_x->text().toStdString().c_str() << endl;
       stream << "post_grasp_y="<< ui->lineEdit_post_grasp_y->text().toStdString().c_str() << endl;
       stream << "post_grasp_z="<< ui->lineEdit_post_grasp_z->text().toStdString().c_str() << endl;
       stream << "post_grasp_dist="<< ui->lineEdit_post_grasp_dist->text().toStdString().c_str() << endl;
       stream << "# Place settings" << endl;
       stream << "pre_place_x="<< ui->lineEdit_pre_place_x->text().toStdString().c_str() << endl;
       stream << "pre_place_y="<< ui->lineEdit_pre_place_y->text().toStdString().c_str() << endl;
       stream << "pre_place_z="<< ui->lineEdit_pre_place_z->text().toStdString().c_str() << endl;
       stream << "pre_place_dist="<< ui->lineEdit_pre_place_dist->text().toStdString().c_str() << endl;
       stream << "post_place_x="<< ui->lineEdit_post_place_x->text().toStdString().c_str() << endl;
       stream << "post_place_y="<< ui->lineEdit_post_place_y->text().toStdString().c_str() << endl;
       stream << "post_place_z="<< ui->lineEdit_post_place_z->text().toStdString().c_str() << endl;
       stream << "post_place_dist="<< ui->lineEdit_post_place_dist->text().toStdString().c_str() << endl;
       stream << "# Move settings" << endl;
       stream << "# Others" << endl;
       stream << "max_velocity="<< ui->lineEdit_w_max->text().toStdString().c_str() <<endl;
       stream << "steps=" << ui->lineEdit_steps->text().toStdString().c_str()<< endl;
       stream << "tol_stop=" << ui->lineEdit_tol_stop->text().toStdString().c_str()<< endl;
       if (ui->checkBox_tar_av->isChecked()){ stream << "tar_av=false"<< endl;}else{stream << "tar_av=true"<< endl;}
       if (ui->checkBox_ob_av->isChecked()){stream << "ob_av=false"<< endl;}else{stream << "ob_av=true"<< endl;}
       if(ui->checkBox_approach->isChecked()){stream << "approach=false"<<endl;}else{stream << "approach=true"<<endl;}
       if(ui->checkBox_retreat->isChecked()){stream << "retreat=false"<<endl;}else{stream << "retreat=true"<<endl;}
       //stream << "# END" << endl;


       f.close();
   }




}

void TolDialogHUML::on_pushButton_load_clicked()
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

                if (QString::compare(fields.at(0),QString("Shoulder_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_shoulder_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Elbow_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_elbow_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Wrist_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_wrist_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_vel_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_vel_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_vel_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_vel_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("init_acc_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_init_acc_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_acc_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_acc_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_pos"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_or"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("max_velocity"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_max->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("steps"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_steps->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tol_stop"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tol_stop->setText(fields.at(1));
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
                }

            }

        }

        f.close();

    }

}

void TolDialogHUML::checkApproach(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_pre_grasp->setEnabled(true);
        ui->groupBox_pre_place->setEnabled(true);
    }else{
        //checked
        ui->groupBox_pre_grasp->setEnabled(false);
        ui->groupBox_pre_place->setEnabled(false);
    }
}

void TolDialogHUML::checkRetreat(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_post_grasp->setEnabled(true);
        ui->groupBox_post_place->setEnabled(true);
    }else{
        //checked
        ui->groupBox_post_grasp->setEnabled(false);
        ui->groupBox_post_place->setEnabled(false);
    }
}


} // namespace motion_manager
