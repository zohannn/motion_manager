#include "../include/motion_manager/toldialoghump.hpp"


namespace motion_manager {

using namespace Qt;

TolDialogHUMP::TolDialogHUMP(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TolDialogHUMP)
{
    ui->setupUi(this);

     QObject::connect(ui->checkBox_approach, SIGNAL(stateChanged(int)), this, SLOT(checkApproach(int)));
     QObject::connect(ui->checkBox_retreat, SIGNAL(stateChanged(int)), this, SLOT(checkRetreat(int)));
     QObject::connect(ui->checkBox_sel_final_posture, SIGNAL(stateChanged(int)), this, SLOT(checkFinalPosture(int)));
     QObject::connect(ui->checkBox_add_plane, SIGNAL(stateChanged(int)), this, SLOT(checkAddPlane(int)));
     QObject::connect(ui->checkBox_warm_start, SIGNAL(stateChanged(int)), this, SLOT(checkWarmStart(int)));
     //QObject::connect(ui->checkBox_hand_cond, SIGNAL(stateChanged(int)), this, SLOT(checkHandCond(int)));
     //QObject::connect(ui->checkBox_hand_cond_approach, SIGNAL(stateChanged(int)), this, SLOT(checkHandCondApproach(int)));

    if(ui->checkBox_approach->isChecked()){
        ui->groupBox_pre_grasp->setEnabled(false);
        ui->groupBox_pre_place->setEnabled(false);
        ui->label_pick->setEnabled(false);
    }
    if(ui->checkBox_retreat->isChecked()){
        ui->groupBox_post_grasp->setEnabled(false);
        ui->groupBox_post_place->setEnabled(false);
        ui->label_pick->setEnabled(false);
    }

    if(ui->checkBox_warm_start->isChecked()){
        ui->tab_hump_warm->setEnabled(true);
    }else{
        ui->tab_hump_warm->setEnabled(false);
    }

    this->warm_start = false;
    this->warm_start_plan = false;
    this->warm_start_approach = false;
    this->warm_start_retreat = false;
    this->warm_start_bounce = false;
}

TolDialogHUMP::~TolDialogHUMP()
{
    delete ui;
}





void TolDialogHUMP::getTolsArm(vector<double> &tols)
{
    tols.clear();
    tols.push_back(ui->lineEdit_shoulder_r->text().toDouble());
    tols.push_back(ui->lineEdit_elbow_r->text().toDouble());
    tols.push_back(ui->lineEdit_wrist_r->text().toDouble());
    tols.push_back(ui->lineEdit_hand_r->text().toDouble());
}


void TolDialogHUMP::getTolsHand(MatrixXd &tols)
{

   tols = MatrixXd::Constant(4,3,1);

    tols(0,0) = ui->lineEdit_hand_1_1->text().toDouble(); tols(0,1) = ui->lineEdit_hand_1_2->text().toDouble(); tols(0,2) = ui->lineEdit_hand_1_3->text().toDouble();
    tols(1,0) = ui->lineEdit_hand_2_1->text().toDouble(); tols(1,1) = ui->lineEdit_hand_2_2->text().toDouble(); tols(1,2) = ui->lineEdit_hand_2_3->text().toDouble();
    tols(2,0) = ui->lineEdit_hand_3_1->text().toDouble(); tols(2,1) = ui->lineEdit_hand_3_2->text().toDouble(); tols(2,2) = ui->lineEdit_hand_3_3->text().toDouble();
    tols(3,0) = ui->lineEdit_hand_tip_1->text().toDouble(); tols(3,1) = ui->lineEdit_hand_tip_2->text().toDouble(); tols(3,2) = ui->lineEdit_hand_tip_3->text().toDouble();
}



void TolDialogHUMP::getLambda(std::vector<double> &lambda)
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


void TolDialogHUMP::getTolsObstacles(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_obs_xx_1->text().toDouble(); tols(0,1)=ui->lineEdit_obs_yy_1->text().toDouble(); tols(0,2)=ui->lineEdit_obs_zz_1->text().toDouble(); tols(0,3)=ui->lineEdit_obs_xy_1->text().toDouble(); tols(0,4)=ui->lineEdit_obs_xz_1->text().toDouble(); tols(0,5)=ui->lineEdit_obs_yz_1->text().toDouble();
    tols(1,0)=ui->lineEdit_obs_xx_2->text().toDouble(); tols(1,1)=ui->lineEdit_obs_yy_2->text().toDouble(); tols(1,2)=ui->lineEdit_obs_zz_2->text().toDouble(); tols(1,3)=ui->lineEdit_obs_xy_2->text().toDouble(); tols(1,4)=ui->lineEdit_obs_xz_2->text().toDouble(); tols(1,5)=ui->lineEdit_obs_yz_2->text().toDouble();
    tols(2,0)=ui->lineEdit_obs_xx_3->text().toDouble(); tols(2,1)=ui->lineEdit_obs_yy_3->text().toDouble(); tols(2,2)=ui->lineEdit_obs_zz_3->text().toDouble(); tols(2,3)=ui->lineEdit_obs_xy_3->text().toDouble(); tols(2,4)=ui->lineEdit_obs_xz_3->text().toDouble(); tols(2,5)=ui->lineEdit_obs_yz_3->text().toDouble();

}


void TolDialogHUMP::getTolsTarget(MatrixXd &tols)
{

    tols = MatrixXd::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_tar_xx_1->text().toDouble(); tols(0,1)=ui->lineEdit_tar_yy_1->text().toDouble(); tols(0,2)=ui->lineEdit_tar_zz_1->text().toDouble(); tols(0,3)=ui->lineEdit_tar_xy_1->text().toDouble(); tols(0,4)=ui->lineEdit_tar_xz_1->text().toDouble(); tols(0,5)=ui->lineEdit_tar_yz_1->text().toDouble();
    tols(1,0)=ui->lineEdit_tar_xx_2->text().toDouble(); tols(1,1)=ui->lineEdit_tar_yy_2->text().toDouble(); tols(1,2)=ui->lineEdit_tar_zz_2->text().toDouble(); tols(1,3)=ui->lineEdit_tar_xy_2->text().toDouble(); tols(1,4)=ui->lineEdit_tar_xz_2->text().toDouble(); tols(1,5)=ui->lineEdit_tar_yz_2->text().toDouble();
    tols(2,0)=ui->lineEdit_tar_xx_3->text().toDouble(); tols(2,1)=ui->lineEdit_tar_yy_3->text().toDouble(); tols(2,2)=ui->lineEdit_tar_zz_3->text().toDouble(); tols(2,3)=ui->lineEdit_tar_xy_3->text().toDouble(); tols(2,4)=ui->lineEdit_tar_xz_3->text().toDouble(); tols(2,5)=ui->lineEdit_tar_yz_3->text().toDouble();


}



double TolDialogHUMP::getWMax()
{

    return ui->lineEdit_w_max->text().toDouble();
}

double TolDialogHUMP::getAlphaMax()
{

    return ui->lineEdit_alpha_max->text().toDouble();
}

void TolDialogHUMP::setWMax(double w)
{

    ui->lineEdit_w_max->setText(QString::number(w));
}




double TolDialogHUMP::getTolTarPos()
{

    return ui->lineEdit_tar_pos->text().toDouble();
}


double TolDialogHUMP::getTolTarOr()
{

    return ui->lineEdit_tar_or->text().toDouble();
}

void TolDialogHUMP::setInfo(string info)
{

    this->infoLine = info;
}

bool TolDialogHUMP::getTargetAvoidance()
{

   return !ui->checkBox_tar_av->isChecked();
}

bool TolDialogHUMP::getObstacleAvoidance()
{

    return !ui->checkBox_ob_av->isChecked();
}

bool TolDialogHUMP::getApproach()
{
    return !ui->checkBox_approach->isChecked();
}

bool TolDialogHUMP::getRetreat()
{
    return !ui->checkBox_retreat->isChecked();
}

void TolDialogHUMP::getInitVel(std::vector<double> &init_vel)
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

void TolDialogHUMP::getFinalVel(std::vector<double> &final_vel)
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

void TolDialogHUMP::getInitAcc(std::vector<double> &init_acc)
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

void TolDialogHUMP::getFinalAcc(std::vector<double> &final_acc)
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

/*

void TolDialogHUMP::getVelApproach(std::vector<double> &vel_approach)
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

void TolDialogHUMP::getAccApproach(std::vector<double> &acc_approach)
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
void TolDialogHUMP::getPreGraspApproach(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist->text().toDouble());
}

void TolDialogHUMP::getPostGraspRetreat(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist->text().toDouble());
}

void TolDialogHUMP::getPrePlaceApproach(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist->text().toDouble());

}

void TolDialogHUMP::getPostPlaceRetreat(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist->text().toDouble());
}

double TolDialogHUMP::getW_red_app()
{
    return ui->lineEdit_w_red_app->text().toDouble();
}

double TolDialogHUMP::getW_red_ret()
{
    return ui->lineEdit_w_red_ret->text().toDouble();
}

void TolDialogHUMP::getTargetMove(std::vector<double> &target)
{
    target.clear();
    target.push_back(ui->lineEdit_target_x->text().toDouble());
    target.push_back(ui->lineEdit_target_y->text().toDouble());
    target.push_back(ui->lineEdit_target_z->text().toDouble());
    target.push_back(ui->lineEdit_target_roll->text().toDouble());
    target.push_back(ui->lineEdit_target_pitch->text().toDouble());
    target.push_back(ui->lineEdit_target_yaw->text().toDouble());
}

void TolDialogHUMP::setTargetMove(std::vector<double> &target)
{
    ui->lineEdit_target_x->setText(QString::number(target.at(0)));
    ui->lineEdit_target_y->setText(QString::number(target.at(1)));
    ui->lineEdit_target_z->setText(QString::number(target.at(2)));
    ui->lineEdit_target_roll->setText(QString::number(target.at(3)));
    ui->lineEdit_target_pitch->setText(QString::number(target.at(4)));
    ui->lineEdit_target_yaw->setText(QString::number(target.at(5)));
}

void TolDialogHUMP::getFinalArm(std::vector<double> &finalArm)
{
    finalArm.clear();
    finalArm.push_back(ui->lineEdit_final_arm_1->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_2->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_3->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_4->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_5->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_6->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_7->text().toDouble());
}

void TolDialogHUMP::getFinalHand(std::vector<double> &finalHand)
{
    finalHand.clear();
    finalHand.push_back(ui->lineEdit_final_hand_1->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_2->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_3->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_4->text().toDouble());
}



void TolDialogHUMP::setPlaneParameters(std::vector<double> &point1,std::vector<double> &point2,std::vector<double> &point3)
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

void TolDialogHUMP::setInitJointsVel(std::vector<double>& init_vel)
{
    if(!init_vel.empty())
    {
        ui->lineEdit_init_vel_1->setText(QString::number(init_vel.at(0)));
        ui->lineEdit_init_vel_2->setText(QString::number(init_vel.at(1)));
        ui->lineEdit_init_vel_3->setText(QString::number(init_vel.at(2)));
        ui->lineEdit_init_vel_4->setText(QString::number(init_vel.at(3)));
        ui->lineEdit_init_vel_5->setText(QString::number(init_vel.at(4)));
        ui->lineEdit_init_vel_6->setText(QString::number(init_vel.at(5)));
        ui->lineEdit_init_vel_7->setText(QString::number(init_vel.at(6)));
        ui->lineEdit_init_vel_8->setText(QString::number(init_vel.at(7)));
        ui->lineEdit_init_vel_9->setText(QString::number(init_vel.at(8)));
        ui->lineEdit_init_vel_10->setText(QString::number(init_vel.at(9)));
        ui->lineEdit_init_vel_11->setText(QString::number(init_vel.at(10)));
    }
}

void TolDialogHUMP::setInitJointsAcc(std::vector<double>& init_acc)
{
    if(!init_acc.empty())
    {
        ui->lineEdit_init_acc_1->setText(QString::number(init_acc.at(0)));
        ui->lineEdit_init_acc_2->setText(QString::number(init_acc.at(1)));
        ui->lineEdit_init_acc_3->setText(QString::number(init_acc.at(2)));
        ui->lineEdit_init_acc_4->setText(QString::number(init_acc.at(3)));
        ui->lineEdit_init_acc_5->setText(QString::number(init_acc.at(4)));
        ui->lineEdit_init_acc_6->setText(QString::number(init_acc.at(5)));
        ui->lineEdit_init_acc_7->setText(QString::number(init_acc.at(6)));
        ui->lineEdit_init_acc_8->setText(QString::number(init_acc.at(7)));
        ui->lineEdit_init_acc_9->setText(QString::number(init_acc.at(8)));
        ui->lineEdit_init_acc_10->setText(QString::number(init_acc.at(9)));
        ui->lineEdit_init_acc_11->setText(QString::number(init_acc.at(10)));
    }
}

void TolDialogHUMP::getPlaneParameters(std::vector<double> &params)
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

void TolDialogHUMP::on_pushButton_save_clicked()
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
       /*
       stream << "# Velocity Approach" << endl;
       stream << "vel_approach_1=" << ui->lineEdit_vel_approach_1->text().toStdString().c_str() << endl;
       stream << "vel_approach_2=" << ui->lineEdit_vel_approach_2->text().toStdString().c_str() << endl;
       stream << "vel_approach_3=" << ui->lineEdit_vel_approach_3->text().toStdString().c_str() << endl;
       stream << "vel_approach_4=" << ui->lineEdit_vel_approach_4->text().toStdString().c_str() << endl;
       stream << "vel_approach_5=" << ui->lineEdit_vel_approach_5->text().toStdString().c_str() << endl;
       stream << "vel_approach_6=" << ui->lineEdit_vel_approach_6->text().toStdString().c_str() << endl;
       stream << "vel_approach_7=" << ui->lineEdit_vel_approach_7->text().toStdString().c_str() << endl;
       stream << "vel_approach_8=" << ui->lineEdit_vel_approach_8->text().toStdString().c_str() << endl;
       stream << "vel_approach_9=" << ui->lineEdit_vel_approach_9->text().toStdString().c_str() << endl;
       stream << "vel_approach_10=" << ui->lineEdit_vel_approach_10->text().toStdString().c_str() << endl;
       stream << "vel_approach_11=" << ui->lineEdit_vel_approach_11->text().toStdString().c_str() << endl;
       stream << "# Acceleration Approach" << endl;
       stream << "acc_approach_1=" << ui->lineEdit_acc_approach_1->text().toStdString().c_str() << endl;
       stream << "acc_approach_2=" << ui->lineEdit_acc_approach_2->text().toStdString().c_str() << endl;
       stream << "acc_approach_3=" << ui->lineEdit_acc_approach_3->text().toStdString().c_str() << endl;
       stream << "acc_approach_4=" << ui->lineEdit_acc_approach_4->text().toStdString().c_str() << endl;
       stream << "acc_approach_5=" << ui->lineEdit_acc_approach_5->text().toStdString().c_str() << endl;
       stream << "acc_approach_6=" << ui->lineEdit_acc_approach_6->text().toStdString().c_str() << endl;
       stream << "acc_approach_7=" << ui->lineEdit_acc_approach_7->text().toStdString().c_str() << endl;
       stream << "acc_approach_8=" << ui->lineEdit_acc_approach_8->text().toStdString().c_str() << endl;
       stream << "acc_approach_9=" << ui->lineEdit_acc_approach_9->text().toStdString().c_str() << endl;
       stream << "acc_approach_10=" << ui->lineEdit_acc_approach_10->text().toStdString().c_str() << endl;
       stream << "acc_approach_11=" << ui->lineEdit_acc_approach_11->text().toStdString().c_str() << endl;
       */
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
       stream << "w_red_app=" << ui->lineEdit_w_red_app->text().toStdString().c_str() << endl;
       stream << "w_red_ret=" << ui->lineEdit_w_red_ret->text().toStdString().c_str() << endl;
       stream << "# Move settings" << endl;
       stream << "target_x=" << ui->lineEdit_target_x->text().toStdString().c_str() << endl;
       stream << "target_y=" << ui->lineEdit_target_y->text().toStdString().c_str() << endl;
       stream << "target_z=" << ui->lineEdit_target_z->text().toStdString().c_str() << endl;
       stream << "target_roll=" << ui->lineEdit_target_roll->text().toStdString().c_str() << endl;
       stream << "target_pitch=" << ui->lineEdit_target_pitch->text().toStdString().c_str() << endl;
       stream << "target_yaw=" << ui->lineEdit_target_yaw->text().toStdString().c_str() << endl;
       stream << "final_arm_1=" << ui->lineEdit_final_arm_1->text().toStdString().c_str() << endl;
       stream << "final_arm_2=" << ui->lineEdit_final_arm_2->text().toStdString().c_str() << endl;
       stream << "final_arm_3=" << ui->lineEdit_final_arm_3->text().toStdString().c_str() << endl;
       stream << "final_arm_4=" << ui->lineEdit_final_arm_4->text().toStdString().c_str() << endl;
       stream << "final_arm_5=" << ui->lineEdit_final_arm_5->text().toStdString().c_str() << endl;
       stream << "final_arm_6=" << ui->lineEdit_final_arm_6->text().toStdString().c_str() << endl;
       stream << "final_arm_7=" << ui->lineEdit_final_arm_7->text().toStdString().c_str() << endl;
       stream << "final_hand_1=" << ui->lineEdit_final_hand_1->text().toStdString().c_str() << endl;
       stream << "final_hand_2=" << ui->lineEdit_final_hand_2->text().toStdString().c_str() << endl;
       stream << "final_hand_3=" << ui->lineEdit_final_hand_3->text().toStdString().c_str() << endl;
       stream << "final_hand_4=" << ui->lineEdit_final_hand_4->text().toStdString().c_str() << endl;
       if (ui->checkBox_sel_final_posture->isChecked()){ stream << "sel_final=true"<< endl;}else{stream << "sel_final=false"<< endl;}
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
       if(ui->checkBox_coll_body->isChecked()){stream << "coll_body=false"<<endl;}else{stream << "coll_body=true"<<endl;}
       if(ui->checkBox_straight_line->isChecked()){stream << "straight_line=true"<<endl;}else{stream << "straight_line=false"<<endl;}
       if(ui->checkBox_warm_start->isChecked()){stream << "warm_start=true"<<endl;}else{stream << "warm_start=false"<<endl;}
       //stream << "# END" << endl;


       f.close();
   }

}

void TolDialogHUMP::on_pushButton_load_warm_start_settings_clicked()
{

    this->warm_start_plan = false;
    this->warm_start_approach = false;
    this->warm_start_retreat = false;
    this->warm_start_bounce = false;
    this->ui->tabWidget_warm_start->setTabEnabled(0,false); // plan
    x_plan.clear(); zL_plan.clear(); zU_plan.clear(); dual_plan.clear();
    this->ui->tabWidget_warm_start->setTabEnabled(1,false); // approach
    x_approach.clear(); zL_approach.clear(); zU_approach.clear(); dual_approach.clear();
    this->ui->tabWidget_warm_start->setTabEnabled(2,false); // retreat
    x_retreat.clear(); zL_retreat.clear(); zU_retreat.clear(); dual_retreat.clear();
    this->ui->tabWidget_warm_start->setTabEnabled(3,false); // bounce
    x_bounce.clear(); zL_bounce.clear(); zU_bounce.clear(); dual_bounce.clear();


    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of warm start settings"),
                                                    QString(MAIN_PATH)+"/Duals",
                                                    "All Files (*.*);; Tol Files (*.dual)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;
        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)!=QChar('#')){
                QStringList fields = line.split("=");
                if (QString::compare(fields.at(0),QString("X_plan"),Qt::CaseInsensitive)==0){
                    this->ui->tabWidget_warm_start->setTabEnabled(0,true);
                    this->warm_start_plan = true;
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        x_plan.push_back(data.at(i).toDouble());
                }else if (QString::compare(fields.at(0),QString("ZL_plan"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zL_plan.push_back(data.at(i).toDouble());
                }else if (QString::compare(fields.at(0),QString("ZU_plan"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zU_plan.push_back(data.at(i).toDouble());
                }else if (QString::compare(fields.at(0),QString("Dual_plan"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        dual_plan.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("X_approach"),Qt::CaseInsensitive)==0){
                    this->ui->tabWidget_warm_start->setTabEnabled(1,true);
                    this->warm_start_approach = true;
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        x_approach.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZL_approach"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zL_approach.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZU_approach"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zU_approach.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("Dual_approach"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        dual_approach.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("X_retreat"),Qt::CaseInsensitive)==0){
                    this->ui->tabWidget_warm_start->setTabEnabled(2,true);
                    this->warm_start_retreat = true;
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        x_retreat.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZL_retreat"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zL_retreat.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZU_retreat"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zU_retreat.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("Dual_retreat"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        dual_retreat.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("X_bounce"),Qt::CaseInsensitive)==0){
                    this->ui->tabWidget_warm_start->setTabEnabled(3,true);
                    this->warm_start_bounce = true;
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        x_bounce.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZL_bounce"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zL_bounce.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("ZU_bounce"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        zU_bounce.push_back(data.at(i).toDouble());
                }else if(QString::compare(fields.at(0),QString("Dual_bounce"),Qt::CaseInsensitive)==0){
                    QStringList data = fields.at(1).split("|");
                    for (size_t i=0; i<data.size();++i)
                        dual_bounce.push_back(data.at(i).toDouble());
                }
            }
        }

        f.close();
    }

    QStringList h_headers; h_headers << "Solution [rad]" << "Lower Bounds [rad]" << "Upper Bounds [rad]";
    QStringList v_headers;
    QStringList h_dual_headers;  h_dual_headers << "Lagrange multipliers";
    QStringList v_dual_headers;

    // ----------- display the plan data ----------------------- //
    this->ui->tableWidget_init_guess_plan->clear();
    this->ui->tableWidget_dual_vars_plan->clear();

    // initial guess and bounds multipliers
    v_headers.clear();
    this->ui->tableWidget_init_guess_plan->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_plan->setRowCount(x_plan.size());
    for(int i =0; i < x_plan.size(); ++i){
        double x_value = x_plan.at(i);
        double zL_value = zL_plan.at(i);
        double zU_value = zU_plan.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_plan->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_plan->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_plan->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    v_dual_headers.clear();
    this->ui->tableWidget_dual_vars_plan->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_plan->setRowCount(dual_plan.size());
    for(int i =0; i < dual_plan.size(); ++i){
        double d_value = dual_plan.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_plan->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_plan->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_plan->setVerticalHeaderLabels(v_dual_headers);


    // ----------- display the approach data ----------------------- //
    this->ui->tableWidget_init_guess_approach->clear();
    this->ui->tableWidget_dual_vars_approach->clear();

    // initial guess and bounds multipliers
    v_headers.clear();
    this->ui->tableWidget_init_guess_approach->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_approach->setRowCount(x_approach.size());
    for(int i =0; i < x_approach.size(); ++i){
        double x_value = x_approach.at(i);
        double zL_value = zL_approach.at(i);
        double zU_value = zU_approach.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_approach->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_approach->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_approach->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    v_dual_headers.clear();
    this->ui->tableWidget_dual_vars_approach->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_approach->setRowCount(dual_approach.size());
    for(int i =0; i < dual_approach.size(); ++i){
        double d_value = dual_approach.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_approach->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_approach->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_approach->setVerticalHeaderLabels(v_dual_headers);

    // ----------- display the retreat data ----------------------- //
    this->ui->tableWidget_init_guess_retreat->clear();
    this->ui->tableWidget_dual_vars_retreat->clear();

    // initial guess and bounds multipliers
    v_headers.clear();
    this->ui->tableWidget_init_guess_retreat->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_retreat->setRowCount(x_retreat.size());
    for(int i =0; i < x_retreat.size(); ++i){
        double x_value = x_retreat.at(i);
        double zL_value = zL_retreat.at(i);
        double zU_value = zU_retreat.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_retreat->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_retreat->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_retreat->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    v_dual_headers.clear();
    this->ui->tableWidget_dual_vars_retreat->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_retreat->setRowCount(dual_retreat.size());
    for(int i =0; i < dual_retreat.size(); ++i){
        double d_value = dual_retreat.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_retreat->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_retreat->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_retreat->setVerticalHeaderLabels(v_dual_headers);


    // ----------- display the bounce data ----------------------- //
    this->ui->tableWidget_init_guess_bounce->clear();
    this->ui->tableWidget_dual_vars_bounce->clear();

    // initial guess and bounds multipliers
    v_headers.clear();
    this->ui->tableWidget_init_guess_bounce->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_bounce->setRowCount(x_bounce.size());
    for(int i =0; i < x_bounce.size(); ++i){
        double x_value = x_bounce.at(i);
        double zL_value = zL_bounce.at(i);
        double zU_value = zU_bounce.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_bounce->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_bounce->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_bounce->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    v_dual_headers.clear();
    this->ui->tableWidget_dual_vars_bounce->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_bounce->setRowCount(dual_bounce.size());
    for(int i =0; i < dual_bounce.size(); ++i){
        double d_value = dual_bounce.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_bounce->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_bounce->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_bounce->setVerticalHeaderLabels(v_dual_headers);


}

void TolDialogHUMP::on_pushButton_load_clicked()
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
                    /*
                }else if(QString::compare(fields.at(0),QString("vel_approach_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_approach_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_vel_approach_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("acc_approach_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_acc_approach_11->setText(fields.at(1));
                    */
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
                }else if(QString::compare(fields.at(0),QString("w_red_app"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("w_red_ret"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_red_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_roll"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_roll->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_pitch"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_pitch->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_yaw"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_yaw->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("sel_final"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_sel_final_posture->setChecked(false);
                    }else{
                        ui->checkBox_sel_final_posture->setChecked(true);
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
                }else if(QString::compare(fields.at(0),QString("coll_body"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_coll_body->setChecked(true);
                    }else{
                        ui->checkBox_coll_body->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("straight_line"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_straight_line->setChecked(false);
                    }else{
                        ui->checkBox_straight_line->setChecked(true);
                    }

                }else if(QString::compare(fields.at(0),QString("warm_start"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_warm_start->setChecked(false);
                    }else{
                        ui->checkBox_warm_start->setChecked(true);
                    }

                }

            }

        }

        f.close();

    }

}

void TolDialogHUMP::checkApproach(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_pre_grasp->setEnabled(true);
        ui->groupBox_pre_place->setEnabled(true);
        ui->label_pick->setEnabled(true);
    }else{
        //checked
        ui->groupBox_pre_grasp->setEnabled(false);
        ui->groupBox_pre_place->setEnabled(false);
        ui->label_pick->setEnabled(false);
    }
}

void TolDialogHUMP::checkRetreat(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_post_grasp->setEnabled(true);
        ui->groupBox_post_place->setEnabled(true);
        ui->label_pick->setEnabled(true);
    }else{
        //checked
        ui->groupBox_post_grasp->setEnabled(false);
        ui->groupBox_post_place->setEnabled(false);
        ui->label_pick->setEnabled(false);
    }
}

void TolDialogHUMP::checkFinalPosture(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_target->setEnabled(true);
        ui->groupBox_final_arm->setEnabled(false);
    }else{
        //checked
        ui->groupBox_target->setEnabled(false);
        ui->groupBox_final_arm->setEnabled(true);
    }
}

void TolDialogHUMP::checkAddPlane(int state)
{
    if(state==0){
        //unchecked
        ui->groupBox_plane->setEnabled(false);
    }else{
        //checked
        ui->groupBox_plane->setEnabled(true);
    }
}

void TolDialogHUMP::checkWarmStart(int state)
{
    if(state==0){
        //unchecked
        warm_start = false;
        ui->tab_hump_warm->setEnabled(false);
        //ui->tabWidget->setTabEnabled(4,false);
    }else{
        //checked
        warm_start = true;
        ui->tab_hump_warm->setEnabled(true);
        //ui->tabWidget->setTabEnabled(4,true);
    }
}

    /*
void TolDialogHUMP::checkSetHandCond(int state)
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
void TolDialogHUMP::checkSetHandCondApproach(int state)
{

}
*/

bool TolDialogHUMP::getRandInit()
{
    return ui->checkBox_rand_init->isChecked();
}

void TolDialogHUMP::setRandInit(bool rand)
{
    ui->checkBox_rand_init->setChecked(rand);
}

bool TolDialogHUMP::getColl()
{
    return !ui->checkBox_coll->isChecked();
}

bool TolDialogHUMP::getCollBody()
{
    return !ui->checkBox_coll_body->isChecked();
}

void TolDialogHUMP::setColl(bool coll)
{
    ui->checkBox_coll->setChecked(!coll);
}

bool TolDialogHUMP::get_use_final_posture()
{
    return ui->checkBox_sel_final_posture->isChecked();
}

bool TolDialogHUMP::get_add_plane()
{
    return ui->checkBox_add_plane->isChecked();
}

bool TolDialogHUMP::get_straight_line()
{
    return ui->checkBox_straight_line->isChecked();
}

void TolDialogHUMP::setStraightLine(bool straight)
{
    ui->checkBox_straight_line->setChecked(straight);
}

void TolDialogHUMP::set_add_plane(bool plane)
{
    ui->checkBox_add_plane->setChecked(plane);
}

bool TolDialogHUMP::getWarmStartOption()
{
    return this->warm_start;
}

bool TolDialogHUMP::getWarmStartPlanOption()
{
    return this->warm_start_plan;
}

bool TolDialogHUMP::getWarmStartApproachOption()
{
    return this->warm_start_approach;
}

bool TolDialogHUMP::getWarmStartRetreatOption()
{
    return this->warm_start_retreat;
}

bool TolDialogHUMP::getWarmStartBounceOption()
{
    return this->warm_start_bounce;
}


void TolDialogHUMP::getPlanData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual)
{
    x = this->x_plan;
    zL = this->zL_plan;
    zU = this->zU_plan;
    dual = this->dual_plan;
}

void TolDialogHUMP::getApproachData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual)
{
    x = this->x_approach;
    zL = this->zL_approach;
    zU = this->zU_approach;
    dual = this->dual_approach;
}

void TolDialogHUMP::getRetreatData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual)
{
    x = this->x_retreat;
    zL = this->zL_retreat;
    zU = this->zU_retreat;
    dual = this->dual_retreat;
}

void TolDialogHUMP::getBounceData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual)
{
    x = this->x_bounce;
    zL = this->zL_bounce;
    zU = this->zU_bounce;
    dual = this->dual_bounce;
}



} // namespace motion_manager
