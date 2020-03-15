﻿/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/motion_manager/main_window.hpp"



/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace Qt;
using namespace std;

namespace motion_manager{

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // create Ros Communication dialog
    mrosCommdlg = new RosCommDialog(&qnode, this);
    mrosCommdlg->setModal(true);

    // create Vrep Communication dialog
    mvrepCommdlg = new VrepCommDialog(&qnode, this);
    mvrepCommdlg->setModal(true);
#if MOVEIT==1
    // create RViz Communication dialog
    mrvizCommdlg = new RVizCommDialog(&qnode, this);
    mrvizCommdlg->setModal(true);
#endif

    //create HUMP Tuning dialog
    mTolHumpdlg = new TolDialogHUMP(this);
    mTolHumpdlg->setModal(true);

    //create HUMP Tuning dialog for bimanual motion
    mTolHumpDualdlg = new TolDialogHUMPDual(this);
    mTolHumpDualdlg->setModal(true);

    //create RRT Tuning dialog
    mRRTdlg = new RRTDialog(this);
    mRRTdlg->setModal(true);

    //create RRT Connect Tuning dialog
    mRRTConnectdlg = new RRTConnectDialog(this);
    mRRTConnectdlg->setModal(true);

    //create RRT star Tuning dialog
    mRRTstardlg = new RRTstarDialog(this);
    mRRTstardlg->setModal(true);

    //create PRM Tuning dialog
    mPRMdlg = new PRMDialog(this);
    mPRMdlg->setModal(true);

    //create PRM star Tuning dialog
    mPRMstardlg = new PRMstarDialog(this);
    mPRMstardlg->setModal(true);

    //create the Results Joints dialog
    mResultsJointsdlg = new ResultsJointsDialog(this);
    mResultsJointsdlg->setModal(false);

    // create the Power Law dialog
    mPowerLawdlg = new PowerLawDialog(this);
    mPowerLawdlg->setModal(false);

    // create the Power Law 3D dialog
    mPowerLaw3Ddlg = new PowerLaw3DDialog(this);
    mPowerLaw3Ddlg->setModal(false);

    // create the velocity components dialog
    mCompVeldlg = new CompVelocityDialog(this);
    mCompVeldlg->setModal(false);

    // create the natural collision avoidance dialog
    mNatCollAvdlg = new NatCollAvDialog(this);
    mNatCollAvdlg->setModal(false);

    // create the warm start results dialog
    mWarmdlg = new WarmStartResultsDialog(this);
    mWarmdlg->setModal(false);

    //create the Results Control Joints dialog
    mResultsCtrlJointsdlg = new ResultsCtrlJointsDialog(this);
    mResultsCtrlJointsdlg->setModal(false);

    // create the control components dialog
    mCompCtrldlg = new CompControlDialog(this);
    mCompCtrldlg->setModal(false);

    // create the errors control dialog
    mErrCtrldlg = new ErrorsControlDialog(this);
    mErrCtrldlg->setModal(false);

    // create the tracking control dialog
    mCompTrackCtrldlg = new CompTrackingControlDialog(this);
    mCompTrackCtrldlg->setModal(false);

    // create the time mapping dialog
    mTimeMapdlg = new TimeMapDialog(this);
    mTimeMapdlg->setModal(false);

    //create the Results Control Predicted Swivel Angle dialog
    mResultsCtrlPredSwivelAngledlg = new ResultsCtrlPredSwivelAngleDialog(this);
    mResultsCtrlPredSwivelAngledlg->setModal(false);

    // check boxes
    QObject::connect(this->ui.checkBox_tar_x_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_x_pos_var(int)));
    QObject::connect(this->ui.checkBox_tar_y_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_y_pos_var(int)));
    QObject::connect(this->ui.checkBox_tar_z_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_z_pos_var(int)));
    QObject::connect(this->ui.checkBox_tar_roll_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_roll_var(int)));
    QObject::connect(this->ui.checkBox_tar_pitch_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_pitch_var(int)));
    QObject::connect(this->ui.checkBox_tar_yaw_var, SIGNAL(stateChanged(int)), this, SLOT(check_tar_yaw_var(int)));

    QObject::connect(this->ui.checkBox_obsts_x_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_x_pos_var(int)));
    QObject::connect(this->ui.checkBox_obsts_y_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_y_pos_var(int)));
    QObject::connect(this->ui.checkBox_obsts_z_pos_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_z_pos_var(int)));
    QObject::connect(this->ui.checkBox_obsts_roll_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_roll_var(int)));
    QObject::connect(this->ui.checkBox_obsts_pitch_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_pitch_var(int)));
    QObject::connect(this->ui.checkBox_obsts_yaw_var, SIGNAL(stateChanged(int)), this, SLOT(check_obsts_yaw_var(int)));

    QObject::connect(this->ui.checkBox_right_hand_status, SIGNAL(stateChanged(int)), this, SLOT(check_right_hand_status(int)));
    QObject::connect(this->ui.checkBox_use_vel_control, SIGNAL(stateChanged(int)), this, SLOT(check_use_vel_control(int)));

    QObject::connect(this->ui.checkBox_des_right_hand_pos_x, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_pos_x(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_pos_y, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_pos_y(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_pos_z, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_pos_z(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_q_x, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_q_x(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_q_y, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_q_y(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_q_z, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_q_z(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_q_w, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_q_w(int)));
    QObject::connect(this->ui.checkBox_use_plan_hand_pos, SIGNAL(stateChanged(int)), this, SLOT(check_use_plan_hand_pos(int)));

    QObject::connect(this->ui.checkBox_des_right_hand_vel_x, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_x(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_vel_y, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_y(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_vel_z, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_z(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_vel_wx, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_wx(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_vel_wy, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_wy(int)));
    QObject::connect(this->ui.checkBox_des_right_hand_vel_wz, SIGNAL(stateChanged(int)), this, SLOT(check_des_right_hand_vel_wz(int)));

    QObject::connect(this->ui.checkBox_joints_limits_av, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_joints_limits_av(int)));
    QObject::connect(this->ui.checkBox_sing_av, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_sing_av(int)));
    QObject::connect(this->ui.checkBox_obsts_av, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_obsts_av(int)));
    QObject::connect(this->ui.checkBox_obsts_filter_noise, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_obsts_filter_noise(int)));
    QObject::connect(this->ui.checkBox_hl_add, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_hl_add(int)));    
    QObject::connect(this->ui.checkBox_draw_ellipse, SIGNAL(stateChanged(int)), this, SLOT(check_draw_ellipse(int)));
    QObject::connect(this->ui.checkBox_tar_filter_noise, SIGNAL(stateChanged(int)), this, SLOT(check_ctrl_tar_filter_noise(int)));


    ReadSettings();
    setWindowIcon(QIcon(":/images/motion_managerIcon.png"));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    // ROS connected signal
    QObject::connect(mrosCommdlg, SIGNAL(rosConnected(bool)), this, SLOT(updateRosStatus(bool)));

    // V-REP connected signal
    QObject::connect(mvrepCommdlg, SIGNAL(vrepConnected(bool)), this, SLOT(updateVrepStatus(bool)));

#if MOVEIT==1
    // RViz connectedsignal
    QObject::connect(mrvizCommdlg, SIGNAL(rvizConnected(bool)), this, SLOT(updateRVizStatus(bool)));
#endif


    // new element in the scenario signal
    QObject::connect(&qnode, SIGNAL(newElement(string)),this,SLOT(addElement(string)));
    QObject::connect(&qnode, SIGNAL(newObject(string)),this,SLOT(addObject(string)));
    QObject::connect(&qnode, SIGNAL(newPose(string)),this,SLOT(addPose(string)));

    // update an element in the scenario signal
    QObject::connect(&qnode, SIGNAL(updateElement(int,string)),this,SLOT(updateElement(int,string)));

    // new joint in the home posture
    QObject::connect(&qnode, SIGNAL(newJoint(string)),this,SLOT(updateHomePosture(string)));

    //description of the selected scenario
    QObject::connect(ui.listWidget_scenario, SIGNAL(itemClicked(QListWidgetItem*)),this, SLOT(onListScenarioItemClicked(QListWidgetItem*)));


    /*********************
    ** Start settings
    **********************/    
    ui.tabWidget_main->setCurrentIndex(0);
    ui.tabWidget_sol->setCurrentIndex(0);
    ui.groupBox_ctrl_options->setEnabled(true);
    ui.pushButton_time_map->setEnabled(true);
    ui.groupBox_noise_filter->setEnabled(true);
    ui.radioButton_N_5->setChecked(true);
    ui.groupBox_jlim_params->setEnabled(false);
    ui.groupBox_sing_av_params->setEnabled(false);
    ui.groupBox_obsts_av_params->setEnabled(false);
    ui.groupBox_hl_add_params->setEnabled(false);
    ui.checkBox_joints_velocity_ctrl->setEnabled(true);
    this->i_ctrl=0;
    this->t_j_past=0.0;
    this->t_der_past=0.0;
    this->t_past=0.0;
    this->t_past_ctrl = sec::zero();
    this->t_der_past_ctrl = sec::zero();
    this->qnode.resetSimTime();
    this->samples_des_hand_pose=0;
    this->samples_des_hand_vel=0;
    this->samples_pos=0;
    this->samples_vel=0;
    this->samples_h_vel=0;
    this->samples_w_vel=0;
    this->samples_e_vel=0;
    this->samples_s_vel=0;
    this->N_filter_length=5;
    this->des_hand_pose_buff = boost::make_shared<CircularBuffers<double>>(7, this->N_filter_length);
    this->des_hand_vel_buff = boost::make_shared<CircularBuffers<double>>(7, this->N_filter_length);
    this->arm_pos_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_ARM, this->N_filter_length);
    this->hand_pos_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_HAND, this->N_filter_length);
    this->arm_vel_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_ARM, this->N_filter_length);
    this->hand_vel_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_HAND, this->N_filter_length);
    this->r_hand_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_wrist_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_elbow_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_shoulder_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    // logging
    init();
    logging::add_common_attributes();

#if MOVEIT==0
    ui.pushButton_execMov_moveit->setEnabled(false);
    ui.comboBox_planner->clear();
    ui.comboBox_planner->addItem(QString("HUMP"));
#endif

    // scenarios
    scenarios.clear();
#if HAND == 0
    scenarios.push_back(QString("Assembly scenario: Toy vehicle with Jarde"));

#elif HAND == 1

    scenarios.push_back(QString("Assembly scenario: Toy vehicle with ARoS and Bill"));
    scenarios.push_back(QString("Empty scenario: empty scenario with ARoS"));
    scenarios.push_back(QString("Empty scenario: empty scenario with ARoS and NO collisions"));
    scenarios.push_back(QString("Human assistance scenario: Serving a drink with ARoS"));
    scenarios.push_back(QString("Challenging scenario: picking a cup from a shelf with ARoS"));
    scenarios.push_back(QString("Assembly scenario: swap two columns of the toy vehicle"));
    scenarios.push_back(QString("Human assistance scenario: Moving a tray with ARoS"));
    scenarios.push_back(QString("Natural obstacle avoidance with ARoS"));
    scenarios.push_back(QString("Learning tasks: reaching with one obstacle"));
    scenarios.push_back(QString("Learning tasks: reaching with many obstacles"));
    scenarios.push_back(QString("Learning tasks: picking the blue column"));
    scenarios.push_back(QString("Controlling: scenario with no obstacles"));
    scenarios.push_back(QString("Controlling: scenario with no obstacles for showing the effects of singularities"));
    scenarios.push_back(QString("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane"));
    scenarios.push_back(QString("Controlling: pick a red column"));
    scenarios.push_back(QString("Controlling: follow a moving red column"));

#endif

    for (size_t i=0; i< scenarios.size();++i){
        ui.listWidget_scenario->addItem(scenarios.at(i));
    }

    // controlling: low pass filter of target's information
    this->lpf_tar_pos_x.reset(new LowPassFilter());
    this->lpf_tar_pos_y.reset(new LowPassFilter());
    this->lpf_tar_pos_z.reset(new LowPassFilter());
    this->lpf_tar_or_q_x.reset(new LowPassFilter());
    this->lpf_tar_or_q_y.reset(new LowPassFilter());
    this->lpf_tar_or_q_z.reset(new LowPassFilter());
    this->lpf_tar_or_q_w.reset(new LowPassFilter());

    // controlling: low pass filter of obstacles' information
    this->lpf_obsts_pos_x.reset(new LowPassFilter());
    this->lpf_obsts_pos_y.reset(new LowPassFilter());
    this->lpf_obsts_pos_z.reset(new LowPassFilter());
    this->lpf_obsts_or_q_x.reset(new LowPassFilter());
    this->lpf_obsts_or_q_y.reset(new LowPassFilter());
    this->lpf_obsts_or_q_z.reset(new LowPassFilter());
    this->lpf_obsts_or_q_w.reset(new LowPassFilter());

    // controlling: low pass filter of the position of the joints
    this->lpf_joint_pos_1.reset(new LowPassFilter());
    this->lpf_joint_pos_2.reset(new LowPassFilter());
    this->lpf_joint_pos_3.reset(new LowPassFilter());
    this->lpf_joint_pos_4.reset(new LowPassFilter());
    this->lpf_joint_pos_5.reset(new LowPassFilter());
    this->lpf_joint_pos_6.reset(new LowPassFilter());
    this->lpf_joint_pos_7.reset(new LowPassFilter());
    this->lpf_joint_pos_8.reset(new LowPassFilter());
    this->lpf_joint_pos_9.reset(new LowPassFilter());
    this->lpf_joint_pos_10.reset(new LowPassFilter());
    this->lpf_joint_pos_11.reset(new LowPassFilter());


    // ---------- threads --------------------------- //
    get_right_hand_status = true;
    display_r_hand_status_thrd = boost::thread(boost::bind(&MainWindow::display_r_hand_status, this));
    exec_control = true; pos_control = false; vel_control = false;
    execPosControl_thrd = boost::thread(boost::bind(&MainWindow::execPosControl, this));
    execVelControl_thrd = boost::thread(boost::bind(&MainWindow::execVelControl, this));

}

MainWindow::~MainWindow()
{
    get_right_hand_status = false;
    if (display_r_hand_status_thrd.joinable())
        display_r_hand_status_thrd.join();

    exec_control = false;
    if(execPosControl_thrd.joinable())
        execPosControl_thrd.join();
    if(execVelControl_thrd.joinable())
        execVelControl_thrd.join();
    if(execReplanning_thrd.joinable())
        execReplanning_thrd.join();

}

/*****************************************************************************
** Implementation [Threads]
*****************************************************************************/

void MainWindow::display_r_hand_status()
{
    while(get_right_hand_status)
    {
        if (this->ui.checkBox_right_hand_status->isChecked())
        {
            boost::unique_lock<boost::mutex> lck(hh_control_mtx);

            ros::spinOnce();

            humanoidPtr hh = this->curr_scene->getHumanoid();

            vector<double> r_posture; vector<double> r_velocities;
            hh->getRightArmPosture(r_posture);
            hh->getRightArmVelocities(r_velocities);

            vector<double> r_hand_pos_comp; vector<double> r_hand_orr_comp;
            hh->getHandPos(1,r_hand_pos_comp,r_posture);
            hh->getHandOr(1,r_hand_orr_comp,r_posture);
            vector<double> r_hand_vel_comp;
            hh->getHandVel(1,r_hand_vel_comp,r_posture,r_velocities);

            if(!r_hand_pos_comp.empty())
            {
                this->ui.label_comp_right_hand_pos_x_value->setText(QString::number(r_hand_pos_comp.at(0)));
                this->ui.label_comp_right_hand_pos_y_value->setText(QString::number(r_hand_pos_comp.at(1)));
                this->ui.label_comp_right_hand_pos_z_value->setText(QString::number(r_hand_pos_comp.at(2)));
                this->ui.label_comp_right_hand_or_roll_value->setText(QString::number(r_hand_orr_comp.at(0)));
                this->ui.label_comp_right_hand_or_pitch_value->setText(QString::number(r_hand_orr_comp.at(1)));
                this->ui.label_comp_right_hand_or_yaw_value->setText(QString::number(r_hand_orr_comp.at(2)));
            }
            if(!r_hand_vel_comp.empty())
            {
                this->ui.label_comp_right_hand_vel_x_value->setText(QString::number(r_hand_vel_comp.at(0)));
                this->ui.label_comp_right_hand_vel_y_value->setText(QString::number(r_hand_vel_comp.at(1)));
                this->ui.label_comp_right_hand_vel_z_value->setText(QString::number(r_hand_vel_comp.at(2)));
                this->ui.label_comp_right_hand_vel_wx_value->setText(QString::number(r_hand_vel_comp.at(3)));
                this->ui.label_comp_right_hand_vel_wy_value->setText(QString::number(r_hand_vel_comp.at(4)));
                this->ui.label_comp_right_hand_vel_wz_value->setText(QString::number(r_hand_vel_comp.at(5)));
            }
        }
    }
}

void MainWindow::execReplanning()
{
    boost::unique_lock<boost::mutex> lck(hh_control_mtx);

    // solve the new problem
    this->qnode.log(QNode::Info,string("Re-planning ..."));
    objectPtr obj_man = this->curr_scene->getObject(this->i_tar_ctrl);
    bool prec = this->ui.radioButton_prec->isChecked();
    int mov_type = this->curr_mov->getType();
    this->prob_ctrl.reset(new Problem(0,new Movement(mov_type,1,obj_man,prec),new Scenario(*(this->curr_scene.get()))));
    HUMotion::planning_result_ptr results_ptr;
    try{
        results_ptr = this->prob_ctrl->solve(this->tols_ctrl);
        if(results_ptr!=nullptr && results_ptr->status==0){
            string solv_time = boost::str(boost::format("%1%") % (this->prob_ctrl->getTime()));
            this->qnode.log(QNode::Info,string("Re-planning succeed! Problem solved in ")+solv_time+string(" ms. Using the new planned trajectory . . ."));
            // the planning succeed: update the results
            this->updatePlanningResults(this->prob_ctrl,results_ptr);
            this->exec_command_ctrl=true;
            this->replanning_succeed=true;
            this->t_past=this->curr_time; // reset time
            this->i_ctrl=0; // plan stage
        }else{
            this->replanning_succeed=false;
            this->qnode.log(QNode::Info,string("Re-planning failed!"));
        }
    }catch( ... ){
        this->replanning_succeed=false;
        this->qnode.log(QNode::Info,string("Re-planning raised an exception!"));
    }
}

void MainWindow::execPosControl()
{

    while(exec_control)
    {

        if(pos_control)
        {
            boost::unique_lock<boost::mutex> lck(hh_control_mtx);

            bool sim_robot = this->ui.radioButton_sim->isChecked(); // true to control the simulator, false to control the real robot

            // desired pose
            double des_hand_pos_x = 0.0; double des_hand_pos_y = 0.0; double des_hand_pos_z = 0.0;
            double des_hand_or_q_x = 0.0; double des_hand_or_q_y = 0.0; double des_hand_or_q_z = 0.0; double des_hand_or_q_w = 0.0;
            VectorXd des_hand_pose(7); vector<double> des_hand_pose_vec(7,0.0);
            VectorXd hand_pos_vec_x = VectorXd::Zero(7); vector<double> hand_pos_vec_xd(7,0.0);

            // desired velocity
            VectorXd hand_vel_vec_x = VectorXd::Zero(7);
//            VectorXd des_hand_vel(7); vector<double> des_hand_vel_vec(7,0.0);

            // desired acceleration
//            VectorXd des_hand_acc(7); vector<double> des_hand_acc_vec(7,0.0);

            // bounce pose
            double bounce_hand_pos_x = 0.0; double bounce_hand_pos_y = 0.0; double bounce_hand_pos_z = 0.0;
            double bounce_hand_q_x = 0.0; double bounce_hand_q_y = 0.0; double bounce_hand_q_z = 0.0; double bounce_hand_q_w = 0.0;

            // simulating noise of the vision system
            double obj_x_var = 100; // mm
            double obj_y_var = 100; // mm
            double obj_z_var = 100; // mm
            double obj_q_x_var = 0.1;
            double obj_q_y_var = 0.1;
            double obj_q_z_var = 0.1;
            double obj_q_w_var = 0.1;

            bool follow_tar = false;


            std::string stage_descr = "plan"; int stages = 1; int mov_type = 0;

            VectorXd jointsInitPosition_hand(JOINTS_HAND); vector<double> jointsInitPosition_hand_vec(JOINTS_HAND);
            VectorXd jointsFinalPosition_hand(JOINTS_HAND); vector<double> jointsFinalPosition_hand_vec(JOINTS_HAND);
            VectorXd jointsInitVelocity_hand(JOINTS_HAND); vector<double> jointsInitVelocity_hand_vec(JOINTS_HAND);
            VectorXd jointsFinalVelocity_hand(JOINTS_HAND); vector<double> jointsFinalVelocity_hand_vec(JOINTS_HAND);
            VectorXd jointsInitAcceleration_hand(JOINTS_HAND); vector<double> jointsInitAcceleration_hand_vec(JOINTS_HAND);
            VectorXd jointsFinalAcceleration_hand(JOINTS_HAND); vector<double> jointsFinalAcceleration_hand_vec(JOINTS_HAND);
            VectorXd jointsBouncePosition_hand(JOINTS_HAND); vector<double> jointsBouncePosition_hand_vec(JOINTS_HAND);
            Matrix3d I_3 = Matrix3d::Identity();
            vector<vector<double>> hand_h_positions; int n_steps; double period_T;
            vector<vector<double>> hand_h_orientations; vector<vector<double>> hand_h_orientations_q;
            vector<vector<double>> hand_h_lin_velocities; vector<vector<double>> hand_h_ang_velocities;
            vector<vector<double>> hand_h_lin_accelerations; vector<vector<double>> hand_h_ang_accelerations;
            Vector3d h_hand_ang_vel_q_e_init; double h_hand_ang_vel_q_w_init; Vector3d h_hand_ang_vel_q_e_end; double h_hand_ang_vel_q_w_end;
            Vector3d h_hand_ang_acc_q_e_init; double h_hand_ang_acc_q_w_init; Vector3d h_hand_ang_acc_q_e_end; double h_hand_ang_acc_q_w_end;


            Vector3d tar_pos; Quaterniond tar_q;
            if(this->ui.checkBox_use_plan_hand_pos->isChecked()){
                 follow_tar = this->ui.checkBox_follow_target->isChecked();
                 stages = this->des_handPosition.size();
                 stage_descr = this->h_results->trajectory_descriptions.at(this->i_ctrl);
                 mov_type = this->curr_mov->getType();
                 //vector<double> xt; vector<double> yt;
                 //vector<double> zt;
                 double dist_app = 0.0; Vector3d vv_app; double dist_ret = 0.0; Vector3d vv_ret;
                 if(mov_type==0){ // pick
                     targetPtr tar;
                     if(sim_robot){
                        tar = this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight();
                        tar_pos(0) = tar->getPos().Xpos;
                        tar_pos(1) = tar->getPos().Ypos;
                        tar_pos(2) = tar->getPos().Zpos;
                        tar_q = tar->getQuaternion();
                     }else{
                        tar = this->curr_scene->getHandTarget();
                        tar_pos(0) = tar->getPos().Xpos;
                        tar_pos(1) = tar->getPos().Ypos;
                        tar_pos(2) = tar->getPos().Zpos;
                        tar_q = tar->getQuaternion();
                        //tar_q = this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight()->getQuaternion(); tar->setQuaternion(tar_q);
                        if(stage_descr.compare("plan")==0){this->tar_rec = targetPtr(new Target(*tar.get()));} // record the latest target of the plan stage

                        /*
                        BOOST_LOG_SEV(lg, info) << "stage = " << stage_descr;
                        BOOST_LOG_SEV(lg, info) << "Target  X pos = " << tar->getPos().Xpos;
                        BOOST_LOG_SEV(lg, info) << "Target  Y pos = " << tar->getPos().Ypos;
                        BOOST_LOG_SEV(lg, info) << "Target  Z pos = " << tar->getPos().Zpos;
                        BOOST_LOG_SEV(lg, info) << "Target  Q X = " << tar->getQuaternion().x();
                        BOOST_LOG_SEV(lg, info) << "Target  Q Y = " << tar->getQuaternion().y();
                        BOOST_LOG_SEV(lg, info) << "Target  Q Z = " << tar->getQuaternion().z();
                        BOOST_LOG_SEV(lg, info) << "Target  Q W = " << tar->getQuaternion().w();

                        BOOST_LOG_SEV(lg, info) << "Target Recorded X pos = " << this->tar_rec->getPos().Xpos;
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Y pos = " << this->tar_rec->getPos().Ypos;
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Z pos = " << this->tar_rec->getPos().Zpos;
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Q X = " << this->tar_rec->getQuaternion().x();
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Q Y = " << this->tar_rec->getQuaternion().y();
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Q Z = " << this->tar_rec->getQuaternion().z();
                        BOOST_LOG_SEV(lg, info) << "Target Recorded Q W = " << this->tar_rec->getQuaternion().w();
                        */

                     }
                     /*
                     BOOST_LOG_SEV(lg, info) << "target position x = " << tar_pos(0);
                     BOOST_LOG_SEV(lg, info) << "target position y = " << tar_pos(1);
                     BOOST_LOG_SEV(lg, info) << "target position z = " << tar_pos(2);
                     BOOST_LOG_SEV(lg, info) << "target quaternion x = " << tar_q.x();
                     BOOST_LOG_SEV(lg, info) << "target quaternion y = " << tar_q.y();
                     BOOST_LOG_SEV(lg, info) << "target quaternion z = " << tar_q.z();
                     BOOST_LOG_SEV(lg, info) << "target quaternion w = " << tar_q.w();
                     */
                     dist_app = this->approach_ctrl.at(3);
                     vv_app << this->approach_ctrl.at(0),this->approach_ctrl.at(1),this->approach_ctrl.at(2);
                     dist_ret = this->retreat_ctrl.at(3);
                     vv_ret << this->retreat_ctrl.at(0),this->retreat_ctrl.at(1),this->retreat_ctrl.at(2);
                     //this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight()->getXt(xt);
                     //this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight()->getYt(yt);
                     //tar->getZt(zt);
                     if(this->ui.checkBox_tar_noise->isChecked() && sim_robot){
                         tar_pos(0) = tar_pos(0) - (obj_x_var/2) + obj_x_var*(rand() / double(RAND_MAX));
                         tar_pos(1) = tar_pos(1) - (obj_y_var/2) + obj_y_var*(rand() / double(RAND_MAX));
                         tar_pos(2) = tar_pos(2) - (obj_z_var/2) + obj_z_var*(rand() / double(RAND_MAX));
                         tar_q.x() = tar_q.x() - (obj_q_x_var/2) + obj_q_x_var*(rand() / double(RAND_MAX));
                         tar_q.y() = tar_q.y() - (obj_q_y_var/2) + obj_q_y_var*(rand() / double(RAND_MAX));
                         tar_q.z() = tar_q.z() - (obj_q_z_var/2) + obj_q_z_var*(rand() / double(RAND_MAX));
                         tar_q.w() = tar_q.w() - (obj_q_w_var/2) + obj_q_w_var*(rand() / double(RAND_MAX));
                     }
                     if(this->ui.checkBox_tar_filter_noise->isChecked()){
                         tar_pos(0) = this->lpf_tar_pos_x->update(tar_pos(0));
                         tar_pos(1) = this->lpf_tar_pos_y->update(tar_pos(1));
                         tar_pos(2) = this->lpf_tar_pos_z->update(tar_pos(2));
                         tar_q.x() = this->lpf_tar_or_q_x->update(tar_q.x());
                         tar_q.y() = this->lpf_tar_or_q_y->update(tar_q.y());
                         tar_q.z() = this->lpf_tar_or_q_z->update(tar_q.z());
                         tar_q.w() = this->lpf_tar_or_q_w->update(tar_q.w());
                     }
                 }else if(mov_type==2 || mov_type==3 || mov_type==4){ // place
                     posePtr tar;
                     tar = this->curr_scene->getPose(this->i_tar_ctrl);
                     tar_pos(0) = tar->getPos().Xpos;
                     tar_pos(1) = tar->getPos().Ypos;
                     tar_pos(2) = tar->getPos().Zpos;
                     tar_q = tar->getQuaternion();
                     /*
                     BOOST_LOG_SEV(lg, info) << "target position x = " << tar_pos(0);
                     BOOST_LOG_SEV(lg, info) << "target position y = " << tar_pos(1);
                     BOOST_LOG_SEV(lg, info) << "target position z = " << tar_pos(2);
                     BOOST_LOG_SEV(lg, info) << "target quaternion x = " << tar_q.x();
                     BOOST_LOG_SEV(lg, info) << "target quaternion y = " << tar_q.y();
                     BOOST_LOG_SEV(lg, info) << "target quaternion z = " << tar_q.z();
                     BOOST_LOG_SEV(lg, info) << "target quaternion w = " << tar_q.w();
                     */

                     dist_app = this->approach_ctrl.at(3);
                     vv_app << this->approach_ctrl.at(0),this->approach_ctrl.at(1),this->approach_ctrl.at(2);
                     dist_ret = this->retreat_ctrl.at(3);
                     vv_ret << this->retreat_ctrl.at(0),this->retreat_ctrl.at(1),this->retreat_ctrl.at(2);
                     //this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight()->getXt(xt);
                     //this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight()->getYt(yt);
                     //tar->getZt(zt);
                     if(this->ui.checkBox_tar_noise->isChecked() && sim_robot){
                         tar_pos(0) = tar_pos(0) - (obj_x_var/2) + obj_x_var*(rand() / double(RAND_MAX));
                         tar_pos(1) = tar_pos(1) - (obj_y_var/2) + obj_y_var*(rand() / double(RAND_MAX));
                         tar_pos(2) = tar_pos(2) - (obj_z_var/2) + obj_z_var*(rand() / double(RAND_MAX));
                         tar_q.x() = tar_q.x() - (obj_q_x_var/2) + obj_q_x_var*(rand() / double(RAND_MAX));
                         tar_q.y() = tar_q.y() - (obj_q_y_var/2) + obj_q_y_var*(rand() / double(RAND_MAX));
                         tar_q.z() = tar_q.z() - (obj_q_z_var/2) + obj_q_z_var*(rand() / double(RAND_MAX));
                         tar_q.w() = tar_q.w() - (obj_q_w_var/2) + obj_q_w_var*(rand() / double(RAND_MAX));
                     }
                     if(this->ui.checkBox_tar_filter_noise->isChecked()){
                         tar_pos(0) = this->lpf_tar_pos_x->update(tar_pos(0));
                         tar_pos(1) = this->lpf_tar_pos_y->update(tar_pos(1));
                         tar_pos(2) = this->lpf_tar_pos_z->update(tar_pos(2));
                         tar_q.x() = this->lpf_tar_or_q_x->update(tar_q.x());
                         tar_q.y() = this->lpf_tar_or_q_y->update(tar_q.y());
                         tar_q.z() = this->lpf_tar_or_q_z->update(tar_q.z());
                         tar_q.w() = this->lpf_tar_or_q_w->update(tar_q.w());
                     }
                 }
                 //Vector3d xt_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xt.data(), xt.size());
                 //Vector3d yt_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yt.data(), yt.size());
                 //Vector3d zt_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(zt.data(), zt.size());
                 Matrix3d Rot_tar = tar_q.toRotationMatrix();
                 Vector3d zt_vec = Rot_tar.col(2);
                 Vector3d vv_app_w = Rot_tar*vv_app; Vector3d vv_ret_w = Rot_tar*vv_ret;

                 hand_h_positions = this->handPosition_mov_stages.at(this->i_ctrl);
                 hand_h_orientations = this->handOrientation_mov_stages.at(this->i_ctrl);
                 hand_h_orientations_q = this->handOrientation_q_mov_stages.at(this->i_ctrl);
                 hand_h_lin_velocities = this->handLinearVelocity_mov_stages.at(this->i_ctrl);
                 hand_h_ang_velocities = this->handAngularVelocity_mov_stages.at(this->i_ctrl);
                 hand_h_lin_accelerations = this->handLinearAcceleration_mov_stages.at(this->i_ctrl);
                 hand_h_ang_accelerations = this->handAngularAcceleration_mov_stages.at(this->i_ctrl);
                 vector<double> timesteps = this->timesteps_mov.at(this->i_ctrl);
                 period_T = std::accumulate(timesteps.begin(), timesteps.end(), 0.0);
                 n_steps = hand_h_positions.size();

                 // initial pose of the human-like hand
                 this->h_hand_pos_init.resize(3); Vector3d hand_pos_tmp;
                 if(stage_descr.compare("plan")==0){
                     vector<double> h_vec = hand_h_positions.at(0);
                     hand_pos_tmp(0) = h_vec.at(0);
                     hand_pos_tmp(1) = h_vec.at(1);
                     hand_pos_tmp(2) = h_vec.at(2);
                     this->h_hand_or_q_init = hand_h_orientations_q.at(0); // initial human-like hand orientation (quaternion)
                 }else if(stage_descr.compare("approach")==0){
                    if(sim_robot){
                        hand_pos_tmp = tar_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                        this->h_hand_or_q_init.at(0) = tar_q.x();
                        this->h_hand_or_q_init.at(1) = tar_q.y();
                        this->h_hand_or_q_init.at(2) = tar_q.z();
                        this->h_hand_or_q_init.at(3) = tar_q.w();
                    }else{
                        if(mov_type==0){ // pick
                            Rot_tar = this->tar_rec->getQuaternion().toRotationMatrix();
                            zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
                            Vector3d tar_rec_pos;
                            tar_rec_pos(0) = this->tar_rec->getPos().Xpos;
                            tar_rec_pos(1) = this->tar_rec->getPos().Ypos;
                            tar_rec_pos(2) = this->tar_rec->getPos().Zpos;
                            hand_pos_tmp = tar_rec_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;

                            this->h_hand_or_q_init.at(0) = this->tar_rec->getQuaternion().x();
                            this->h_hand_or_q_init.at(1) = this->tar_rec->getQuaternion().y();
                            this->h_hand_or_q_init.at(2) = this->tar_rec->getQuaternion().z();
                            this->h_hand_or_q_init.at(3) = this->tar_rec->getQuaternion().w();
                        }else{
                            hand_pos_tmp = tar_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                            this->h_hand_or_q_init.at(0) = tar_q.x();
                            this->h_hand_or_q_init.at(1) = tar_q.y();
                            this->h_hand_or_q_init.at(2) = tar_q.z();
                            this->h_hand_or_q_init.at(3) = tar_q.w();
                        }

                    }
                 }else if(stage_descr.compare("retreat")==0){
                    if(sim_robot){
                        hand_pos_tmp = tar_pos - this->dHO_ctrl*zt_vec;
                        this->h_hand_or_q_init.at(0) = tar_q.x();
                        this->h_hand_or_q_init.at(1) = tar_q.y();
                        this->h_hand_or_q_init.at(2) = tar_q.z();
                        this->h_hand_or_q_init.at(3) = tar_q.w();
                    }else{
                        if(mov_type==0){ // pick
                            Rot_tar = this->tar_rec->getQuaternion().toRotationMatrix();
                            zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
                            Vector3d tar_rec_pos;
                            tar_rec_pos(0) = this->tar_rec->getPos().Xpos;
                            tar_rec_pos(1) = this->tar_rec->getPos().Ypos;
                            tar_rec_pos(2) = this->tar_rec->getPos().Zpos;
                            hand_pos_tmp = tar_rec_pos - this->dHO_ctrl*zt_vec;

                            this->h_hand_or_q_init.at(0) = this->tar_rec->getQuaternion().x();
                            this->h_hand_or_q_init.at(1) = this->tar_rec->getQuaternion().y();
                            this->h_hand_or_q_init.at(2) = this->tar_rec->getQuaternion().z();
                            this->h_hand_or_q_init.at(3) = this->tar_rec->getQuaternion().w();
                        }else{
                            hand_pos_tmp = tar_pos - this->dHO_ctrl*zt_vec;
                            this->h_hand_or_q_init.at(0) = tar_q.x();
                            this->h_hand_or_q_init.at(1) = tar_q.y();
                            this->h_hand_or_q_init.at(2) = tar_q.z();
                            this->h_hand_or_q_init.at(3) = tar_q.w();
                        }
                    }
                 }
                 this->h_hand_pos_init.at(0) = hand_pos_tmp(0);
                 this->h_hand_pos_init.at(1) = hand_pos_tmp(1);
                 this->h_hand_pos_init.at(2) = hand_pos_tmp(2);

                 // initial orientation of the human-like hand
                 Vector3d h_hand_or_q_e_init; h_hand_or_q_e_init << this->h_hand_or_q_init.at(0), this->h_hand_or_q_init.at(1), this->h_hand_or_q_init.at(2);
                 double h_hand_or_q_w_init = this->h_hand_or_q_init.at(3);

                 // end hand pose
                 // position
                 Vector3d hand_pos_vec;
                 if(stage_descr.compare("plan")==0){
                    if(stages>1){
                        string stage_succ = this->h_results->trajectory_descriptions.at(this->i_ctrl+1);
                        if(stage_succ.compare("approach")==0){
                            if(sim_robot){
                                hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                            }else{
                                if(mov_type==0){//pick
                                    Vector3d tar_rec_pos;
                                    tar_rec_pos(0) = this->tar_rec->getPos().Xpos;
                                    tar_rec_pos(1) = this->tar_rec->getPos().Ypos;
                                    tar_rec_pos(2) = this->tar_rec->getPos().Zpos;
                                    hand_pos_vec = tar_rec_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                                }else{
                                    hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                                }
                            }
                        }else{hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec;}
                    }else{
                        hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec;
                    }
                 }else if(stage_descr.compare("approach")==0){
                    if(follow_tar){
                        hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec + dist_app*vv_app_w;
                    }else{
                        if(sim_robot){
                            hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec;
                       }else{
                           if(mov_type==0){//pick
                               Rot_tar = this->tar_rec->getQuaternion().toRotationMatrix();
                               zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
                               Vector3d tar_rec_pos;
                               tar_rec_pos(0) = this->tar_rec->getPos().Xpos;
                               tar_rec_pos(1) = this->tar_rec->getPos().Ypos;
                               tar_rec_pos(2) = this->tar_rec->getPos().Zpos;
                               hand_pos_vec = tar_rec_pos - this->dHO_ctrl*zt_vec;
                           }else{
                              hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec;
                           }
                       }
                    }
                 }else if(stage_descr.compare("retreat")==0){
                     if(sim_robot){
                        hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
                     }else{
                         if(mov_type==0){//pick
                             Rot_tar = this->tar_rec->getQuaternion().toRotationMatrix();
                             zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
                             Vector3d tar_rec_pos;
                             tar_rec_pos(0) = this->tar_rec->getPos().Xpos;
                             tar_rec_pos(1) = this->tar_rec->getPos().Ypos;
                             tar_rec_pos(2) = this->tar_rec->getPos().Zpos;
                             hand_pos_vec = tar_rec_pos - this->dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
                         }else{
                            hand_pos_vec = tar_pos - this->dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
                         }
                     }
                 }
                 // position
                 this->h_hand_pos_end.resize(3);
                 this->h_hand_pos_end.at(0) = hand_pos_vec(0);
                 this->h_hand_pos_end.at(1) = hand_pos_vec(1);
                 this->h_hand_pos_end.at(2) = hand_pos_vec(2);
                 // orientation
                 Quaterniond h_hand_or_q_end_vec(Rot_tar);
                 this->h_hand_or_q_end.resize(4);
                 this->h_hand_or_q_end.at(0) = h_hand_or_q_end_vec.x();
                 this->h_hand_or_q_end.at(1) = h_hand_or_q_end_vec.y();
                 this->h_hand_or_q_end.at(2) = h_hand_or_q_end_vec.z();
                 this->h_hand_or_q_end.at(3) = h_hand_or_q_end_vec.w();
                 Vector3d h_hand_or_q_e_end; h_hand_or_q_e_end << this->h_hand_or_q_end.at(0), this->h_hand_or_q_end.at(1), this->h_hand_or_q_end.at(2);
                 double h_hand_or_q_w_end = this->h_hand_or_q_end.at(3);
                 hand_pos_vec_x(0) = this->h_hand_pos_end.at(0);
                 hand_pos_vec_x(1) = this->h_hand_pos_end.at(1);
                 hand_pos_vec_x(2) = this->h_hand_pos_end.at(2);
                 hand_pos_vec_x(3) = this->h_hand_or_q_end.at(0);
                 hand_pos_vec_x(4) = this->h_hand_or_q_end.at(1);
                 hand_pos_vec_x(5) = this->h_hand_or_q_end.at(2);
                 hand_pos_vec_x(6) = this->h_hand_or_q_end.at(3);

                 this->h_hand_lin_vel_init = hand_h_lin_velocities.at(0); // initial human-like hand linear velocities
                 this->h_hand_ang_vel_init = hand_h_ang_velocities.at(0); // initial human-like hand angular velocities
                 this->h_hand_lin_acc_init = hand_h_lin_accelerations.at(0); // initial human-like hand linear accelerations
                 this->h_hand_ang_acc_init = hand_h_ang_accelerations.at(0); // initial human-like hand angular accelerations

                 this->h_hand_lin_vel_end = hand_h_lin_velocities.at(n_steps-1); // end human-like hand linear velocities
                 this->h_hand_ang_vel_end = hand_h_ang_velocities.at(n_steps-1); // end human-like hand angular velocities
                 this->h_hand_lin_acc_end = hand_h_lin_accelerations.at(n_steps-1); // end human-like hand linear accelerations
                 this->h_hand_ang_acc_end = hand_h_ang_accelerations.at(n_steps-1); // end human-like hand angular accelerations

                Vector3d h_hand_ang_vel_init_vec; h_hand_ang_vel_init_vec << this->h_hand_ang_vel_init.at(0), this->h_hand_ang_vel_init.at(1), this->h_hand_ang_vel_init.at(2);
                Vector3d h_hand_ang_vel_end_vec; h_hand_ang_vel_end_vec << this->h_hand_ang_vel_end.at(0), this->h_hand_ang_vel_end.at(1), this->h_hand_ang_vel_end.at(2);
                Vector3d h_hand_ang_acc_init_vec; h_hand_ang_acc_init_vec << this->h_hand_ang_acc_init.at(0), this->h_hand_ang_acc_init.at(1), this->h_hand_ang_acc_init.at(2);
                Vector3d h_hand_ang_acc_end_vec; h_hand_ang_acc_end_vec << this->h_hand_ang_acc_end.at(0), this->h_hand_ang_acc_end.at(1), this->h_hand_ang_acc_end.at(2);

                 // initial angular velocity (quaternion)
                 h_hand_ang_vel_q_e_init = 0.5*(h_hand_or_q_w_init*I_3*h_hand_ang_vel_init_vec-h_hand_or_q_e_init.cross(h_hand_ang_vel_init_vec));
                 h_hand_ang_vel_q_w_init = -0.5*(h_hand_or_q_e_init.dot(h_hand_ang_vel_init_vec));

                 // end angular velocity (quaternion)
                 h_hand_ang_vel_q_e_end = 0.5*(h_hand_or_q_w_end*I_3*h_hand_ang_vel_end_vec-h_hand_or_q_e_end.cross(h_hand_ang_vel_end_vec));
                 h_hand_ang_vel_q_w_end = -0.5*(h_hand_or_q_e_end.dot(h_hand_ang_vel_end_vec));

                 // initial angular acceleration (quaternion)
                 h_hand_ang_acc_q_e_init = 0.5*I_3*(h_hand_ang_vel_q_w_init*h_hand_ang_vel_init_vec+h_hand_or_q_w_init*h_hand_ang_acc_init_vec)
                                          -0.5*(h_hand_ang_vel_q_e_init.cross(h_hand_ang_vel_init_vec)+h_hand_or_q_e_init.cross(h_hand_ang_acc_init_vec));
                 h_hand_ang_acc_q_w_init = -0.5*(h_hand_ang_vel_q_e_init.dot(h_hand_ang_vel_init_vec)+h_hand_or_q_e_init.dot(h_hand_ang_acc_init_vec));

                 // end angular acceleration (quaternion)
                 h_hand_ang_acc_q_e_end = 0.5*I_3*(h_hand_ang_vel_q_w_end*h_hand_ang_vel_end_vec+h_hand_or_q_w_end*h_hand_ang_acc_end_vec)
                                          -0.5*(h_hand_ang_vel_q_e_end.cross(h_hand_ang_vel_end_vec)+h_hand_or_q_e_end.cross(h_hand_ang_acc_end_vec));
                 h_hand_ang_acc_q_w_end = -0.5*(h_hand_ang_vel_q_e_end.dot(h_hand_ang_vel_end_vec)+h_hand_or_q_e_end.dot(h_hand_ang_acc_end_vec));


                // joint of the fingers
                MatrixXd jointsPosition = this->jointsPosition_mov_ctrl.at(this->i_ctrl);
                MatrixXd jointsVelocity = this->jointsVelocity_mov_ctrl.at(this->i_ctrl);
                MatrixXd jointsAcceleration = this->jointsAcceleration_mov_ctrl.at(this->i_ctrl);
                //position
                jointsInitPosition_hand = jointsPosition.row(0).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsInitPosition_hand_vec[0], jointsInitPosition_hand.size()) = jointsInitPosition_hand;
                jointsFinalPosition_hand = jointsPosition.row(jointsPosition.rows()-1).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsFinalPosition_hand_vec[0], jointsFinalPosition_hand.size()) = jointsFinalPosition_hand;
                //velocity
                jointsInitVelocity_hand = jointsVelocity.row(0).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsInitVelocity_hand_vec[0], jointsInitVelocity_hand.size()) = jointsInitVelocity_hand;
                jointsFinalVelocity_hand = jointsVelocity.row(jointsVelocity.rows()-1).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsFinalVelocity_hand_vec[0], jointsFinalVelocity_hand.size()) = jointsFinalVelocity_hand;
                //acceleration
                jointsInitAcceleration_hand = jointsAcceleration.row(0).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsInitAcceleration_hand_vec[0], jointsInitAcceleration_hand.size()) = jointsInitAcceleration_hand;
                jointsFinalAcceleration_hand = jointsAcceleration.row(jointsAcceleration.rows()-1).tail<JOINTS_HAND>();
                VectorXd::Map(&jointsFinalAcceleration_hand_vec[0], jointsFinalAcceleration_hand.size()) = jointsFinalAcceleration_hand;

                des_hand_pos_x = this->h_hand_pos_end.at(0);
                des_hand_pos_y = this->h_hand_pos_end.at(1);
                des_hand_pos_z = this->h_hand_pos_end.at(2);
                des_hand_or_q_x = this->h_hand_or_q_end.at(0);
                des_hand_or_q_y = this->h_hand_or_q_end.at(1);
                des_hand_or_q_z = this->h_hand_or_q_end.at(2);
                des_hand_or_q_w = this->h_hand_or_q_end.at(3);

            }else{
                des_hand_pos_x = this->ui.lineEdit_des_right_hand_pos_x->text().toDouble();
                des_hand_pos_y = this->ui.lineEdit_des_right_hand_pos_y->text().toDouble();
                des_hand_pos_z = this->ui.lineEdit_des_right_hand_pos_z->text().toDouble();
                des_hand_or_q_x = this->ui.lineEdit_des_right_hand_q_x->text().toDouble();
                des_hand_or_q_y = this->ui.lineEdit_des_right_hand_q_y->text().toDouble();
                des_hand_or_q_z = this->ui.lineEdit_des_right_hand_q_z->text().toDouble();
                des_hand_or_q_w = this->ui.lineEdit_des_right_hand_q_w->text().toDouble();
            }


            double vel_max = this->ui.lineEdit_vel_max->text().toDouble()*M_PI/180;
            bool joints_arm_vel_ctrl = this->ui.checkBox_joints_velocity_ctrl->isChecked();
            double swivel_angle_th = this->ui.lineEdit_swivel_angle_th->text().toDouble();
            double g_map_th_max_replan = this->ui.lineEdit_g_th_max_replan->text().toDouble();
            double g_map_th_min_replan = this->ui.lineEdit_g_th_min_replan->text().toDouble();
            double t_f_trap = this->ui.lineEdit_t_f_trap->text().toDouble();

            bool jlim_en = this->ui.checkBox_joints_limits_av->isChecked();
            double jlim_th = 0; double jlim_rate = 1;
            double jlim_coeff = 1; double jlim_damping = 0.001;
            if(jlim_en){
                jlim_th = this->ui.lineEdit_jlim_th->text().toDouble();
                jlim_rate = this->ui.lineEdit_jlim_rate->text().toDouble();
                jlim_coeff = this->ui.lineEdit_jlim_coeff->text().toDouble();
                jlim_damping = this->ui.lineEdit_jlim_damping->text().toDouble();
            }
            bool sing_en = this->ui.checkBox_sing_av->isChecked();
            double sing_coeff = 1; double sing_damping = 0.001;
            if(sing_en){
                sing_damping = this->ui.lineEdit_sing_damping->text().toDouble();
                sing_coeff = this->ui.lineEdit_sing_coeff->text().toDouble();
            }

            bool obsts_en = this->ui.checkBox_obsts_av->isChecked();
            double obst_coeff = 1; double obst_damping = 0.001;
            double obst_coeff_torso = 1; double obst_damping_torso = 0.001;
            // get the obstacles in the scenario
            vector<objectPtr> obsts; this->curr_scene->getObjects(obsts);
            if(this->ui.checkBox_use_plan_hand_pos->isChecked())
            {
                if((mov_type==0) || (mov_type==2) || (mov_type==3) || (mov_type==4)){
                    // pick or place
                    string obj_tar_name = this->curr_mov->getObject()->getName();
                    for(size_t i=0;i<obsts.size();++i)
                    {
                        if(obj_tar_name.compare(obsts.at(i)->getName())==0)
                        {
                            obsts.erase(obsts.begin()+i);
                        }
                    }
                }
            }
            vector<objectPtr>  obsts_n = obsts;
            if(obsts_en){
                obst_coeff = this->ui.lineEdit_obsts_coeff->text().toDouble();
                obst_damping = this->ui.lineEdit_obsts_damping->text().toDouble();
                obst_coeff_torso = this->ui.lineEdit_obsts_coeff_torso->text().toDouble();
                obst_damping_torso = this->ui.lineEdit_obsts_damping_torso->text().toDouble();
                if(this->ui.checkBox_obsts_noise->isChecked()){
                    objectPtr obs_new;
                    for(size_t i=0; i<obsts.size(); ++i){
                        objectPtr obs = obsts.at(i);
                        string obs_name = obs->getName();
                        obs_new.reset(new Object(obs_name));
                        motion_manager::pos obs_pos; Quaterniond obs_or_q;
                        obs_pos.Xpos = obs->getPos().Xpos - (obj_x_var/2) + obj_x_var*(rand() / double(RAND_MAX));
                        obs_pos.Ypos = obs->getPos().Ypos - (obj_y_var/2) + obj_y_var*(rand() / double(RAND_MAX));
                        obs_pos.Zpos = obs->getPos().Zpos - (obj_z_var/2) + obj_z_var*(rand() / double(RAND_MAX));
                        obs_or_q.x() = obs->getQuaternion().x() - (obj_q_x_var/2) + obj_q_x_var*(rand() / double(RAND_MAX));
                        obs_or_q.y() = obs->getQuaternion().y() - (obj_q_y_var/2) + obj_q_y_var*(rand() / double(RAND_MAX));
                        obs_or_q.z() = obs->getQuaternion().z() - (obj_q_z_var/2) + obj_q_z_var*(rand() / double(RAND_MAX));
                        obs_or_q.w() = obs->getQuaternion().w() - (obj_q_w_var/2) + obj_q_w_var*(rand() / double(RAND_MAX));
                        if(this->ui.checkBox_obsts_filter_noise->isChecked()){
                            obs_pos.Xpos = this->lpf_obsts_pos_x->update(obs_pos.Xpos);
                            obs_pos.Ypos = this->lpf_obsts_pos_y->update(obs_pos.Ypos);
                            obs_pos.Zpos = this->lpf_obsts_pos_z->update(obs_pos.Zpos);
                            obs_or_q.x() = this->lpf_obsts_or_q_x->update(obs_or_q.x());
                            obs_or_q.y() = this->lpf_obsts_or_q_y->update(obs_or_q.y());
                            obs_or_q.z() = this->lpf_obsts_or_q_z->update(obs_or_q.z());
                            obs_or_q.w() = this->lpf_obsts_or_q_w->update(obs_or_q.w());
                        }
                        obs_new->setPos(obs_pos,false);
                        obs_new->setOr(obs_or_q,false);
                        obs_new->setSize(obs->getSize());
                        obsts_n.at(i) = obs_new;
                    } // for loop obstacles
                }// noise on obstacles
            }// enable obstacles avoidance

            // human-likeness
            bool hl_en = this->ui.checkBox_hl_add->isChecked();
            double hl_p_pos_coeff_plan = 1; double hl_p_or_coeff_plan = 1;
            double hl_p_pos_coeff_app = 1; double hl_p_or_coeff_app = 1;
            double hl_p_pos_coeff_ret = 1; double hl_p_or_coeff_ret = 1;
            double hl_d_pos_coeff_plan = 1; double hl_d_or_coeff_plan = 1;
            double hl_d_pos_coeff_app = 1; double hl_d_or_coeff_app = 1;
            double hl_d_pos_coeff_ret = 1; double hl_d_or_coeff_ret = 1;
            double hl_p_pos_coeff = 1; double hl_p_or_coeff = 1;
            double hl_d_pos_coeff = 1; double hl_d_or_coeff = 1;
            double g_map_th_pa = 0.99; double g_map_th_rp = 0.99;
            double fing_p_coeff = 0.1; double fing_d_coeff = 0.1;
            double phi = 0.0; double tb = 0.0;
            if(hl_en){
                hl_p_pos_coeff_plan = this->ui.lineEdit_hl_p_pos_coeff_plan->text().toDouble();
                hl_p_or_coeff_plan = this->ui.lineEdit_hl_p_or_coeff_plan->text().toDouble();
                hl_p_pos_coeff_app = this->ui.lineEdit_hl_p_pos_coeff_app->text().toDouble();
                hl_p_or_coeff_app = this->ui.lineEdit_hl_p_or_coeff_app->text().toDouble();
                hl_p_pos_coeff_ret = this->ui.lineEdit_hl_p_pos_coeff_ret->text().toDouble();
                hl_p_or_coeff_ret = this->ui.lineEdit_hl_p_or_coeff_ret->text().toDouble();
                hl_d_pos_coeff_plan = this->ui.lineEdit_hl_d_pos_coeff_plan->text().toDouble();
                hl_d_or_coeff_plan = this->ui.lineEdit_hl_d_or_coeff_plan->text().toDouble();
                hl_d_pos_coeff_app = this->ui.lineEdit_hl_d_pos_coeff_app->text().toDouble();
                hl_d_or_coeff_app = this->ui.lineEdit_hl_d_or_coeff_app->text().toDouble();
                hl_d_pos_coeff_ret = this->ui.lineEdit_hl_d_pos_coeff_ret->text().toDouble();
                hl_d_or_coeff_ret = this->ui.lineEdit_hl_d_or_coeff_ret->text().toDouble();

                g_map_th_pa = this->ui.lineEdit_g_th_plan_app->text().toDouble();
                if(follow_tar){
                    g_map_th_rp = 2; // never go to retreat or to the subsequent movement
                }else{
                    g_map_th_rp = this->ui.lineEdit_g_th_ret_plan->text().toDouble();
                }
                fing_p_coeff = this->ui.lineEdit_fing_p_coeff->text().toDouble();
                fing_d_coeff = this->ui.lineEdit_fing_d_coeff->text().toDouble();
                phi = this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getHUMPlanner()->getPHI();
                tb = this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getHUMPlanner()->getTB();
                vector<double> bounce_hand_pos = this->bounce_handPosition;
                //vector<double> bounce_hand_orr = this->bounce_handOrientation;
                vector<double> bounce_hand_orr_q = this->bounce_handOrientation_q;
                bounce_hand_pos_x = bounce_hand_pos.at(0);
                bounce_hand_pos_y = bounce_hand_pos.at(1);
                bounce_hand_pos_z = bounce_hand_pos.at(2);
                bounce_hand_q_x = bounce_hand_orr_q.at(0);
                bounce_hand_q_y = bounce_hand_orr_q.at(1);
                bounce_hand_q_z = bounce_hand_orr_q.at(2);
                bounce_hand_q_w = bounce_hand_orr_q.at(3);
                vector<double> bounce_posture = this->h_results->bounce_warm_start_res.x;
                vector<double> bounce_hand_posture(bounce_posture.begin()+JOINTS_ARM,bounce_posture.end());
                jointsBouncePosition_hand_vec = bounce_hand_posture;
                if(mov_type==2 || mov_type==3 ||mov_type==4){ // place
                    jointsBouncePosition_hand = jointsInitPosition_hand;
                }else{
                    jointsBouncePosition_hand << 0.0,jointsBouncePosition_hand_vec.at(0),
                                             jointsBouncePosition_hand_vec.at(0),jointsBouncePosition_hand_vec.at(1);
                }
            }

            // desired position
            Vector3d des_hand_pos; VectorXd des_hand_or_q(4); Vector3d des_hand_or_q_e;
            des_hand_pos << des_hand_pos_x,des_hand_pos_y,des_hand_pos_z;
            des_hand_or_q_e << des_hand_or_q_x,des_hand_or_q_y,des_hand_or_q_z;
            des_hand_or_q << des_hand_or_q_x,des_hand_or_q_y,des_hand_or_q_z,des_hand_or_q_w;
            des_hand_pose << des_hand_pos_x,des_hand_pos_y,des_hand_pos_z,des_hand_or_q_x,des_hand_or_q_y,des_hand_or_q_z,des_hand_or_q_w;
            VectorXd::Map(&des_hand_pose_vec[0], des_hand_pose.size()) = des_hand_pose;

            double error_pos_th = this->ui.lineEdit_err_p_pos->text().toDouble();
            double error_or_th = this->ui.lineEdit_err_p_or->text().toDouble();
//            double error_lin_vel_th = this->ui.lineEdit_err_d_pos->text().toDouble();
//            double error_ang_vel_th = this->ui.lineEdit_err_d_or->text().toDouble();

            double coeff_p_pos = this->ui.lineEdit_coeff_p_pos->text().toDouble();
            double coeff_p_or = this->ui.lineEdit_coeff_p_or->text().toDouble();
            double coeff_d_pos = this->ui.lineEdit_coeff_d_pos->text().toDouble();
            double coeff_d_or = this->ui.lineEdit_coeff_d_or->text().toDouble();

            // -------------- simulation or real robot --------------------------------- //
            this->qnode.setSimRobot(sim_robot);
            double time_step; // time step of the controlling
            bool condition; // condition to process the control

            if(sim_robot){
                // ---------------- start the simulation --------------------------- //
                if(!this->qnode.isSimulationRunning() || this->qnode.isSimulationPaused())
                {

                    // enable set joints subscriber
                    this->qnode.enableSetJoints();

                    // start the simulation
                    this->qnode.startSim();
                }
                time_step = this->qnode.getSimTimeStep(); // sec
                condition = this->qnode.isSimulationRunning() && ((this->qnode.getSimTime()-this->t_j_past)>time_step);
            }else{
                time_step = 0.005; // time step of receiving the state of the joints from ARoS [sec]
                condition = (Clock::now() - this->t_j_past_ctrl) > boost::chrono::duration<double,boost::ratio<1>>(time_step);

            }
            ros::spinOnce(); // handle ROS messages


            if(condition)
            {
                // do it every time step

                // posture
                vector<double> r_arm_posture_mes(JOINTS_ARM,0.0); vector<double> r_hand_posture_mes(JOINTS_HAND,0.0);
                vector<double> r_arm_posture(JOINTS_ARM,0.0); vector<double> r_hand_posture(JOINTS_HAND,0.0); //vector<double> r_hand_posture_ctrl(JOINTS_HAND,0.0);
                // velocities
                VectorXd r_arm_null_velocities = VectorXd::Zero(JOINTS_ARM);
                vector<double> r_arm_velocities(JOINTS_ARM,0.0); vector<double> r_hand_velocities(JOINTS_HAND,0.0);
                vector<double> r_arm_velocities_read(JOINTS_ARM,0.0); vector<double> r_hand_velocities_read(JOINTS_HAND,0.0);
                // accelerations
                //vector<double> r_arm_accelerations(JOINTS_ARM,0.0);
                vector<double> r_arm_accelerations_read(JOINTS_ARM,0.0); vector<double> r_hand_accelerations_read(JOINTS_HAND,0.0);
                vector<double> r_hand_acc_read(6,0.0);
                vector<double> r_wrist_acc_read(6,0.0);
                vector<double> r_elbow_acc_read(6,0.0);
                vector<double> r_shoulder_acc_read(6,0.0);


                this->curr_scene->getHumanoid()->getRightArmPosture(r_arm_posture_mes);
                this->curr_scene->getHumanoid()->getRightHandPosture(r_hand_posture_mes);
                // filtering the joint positions
                r_arm_posture.at(0) = this->lpf_joint_pos_1->update(r_arm_posture_mes.at(0));
                r_arm_posture.at(1) = this->lpf_joint_pos_2->update(r_arm_posture_mes.at(1));
                r_arm_posture.at(2) = this->lpf_joint_pos_3->update(r_arm_posture_mes.at(2));
                r_arm_posture.at(3) = this->lpf_joint_pos_4->update(r_arm_posture_mes.at(3));
                r_arm_posture.at(4) = this->lpf_joint_pos_5->update(r_arm_posture_mes.at(4));
                r_arm_posture.at(5) = this->lpf_joint_pos_6->update(r_arm_posture_mes.at(5));
                r_arm_posture.at(6) = this->lpf_joint_pos_7->update(r_arm_posture_mes.at(6));
                r_hand_posture.at(0) = this->lpf_joint_pos_8->update(r_hand_posture_mes.at(0));
                r_hand_posture.at(1) = this->lpf_joint_pos_9->update(r_hand_posture_mes.at(1));
                r_hand_posture.at(2) = this->lpf_joint_pos_10->update(r_hand_posture_mes.at(2));
                r_hand_posture.at(3) = this->lpf_joint_pos_11->update(r_hand_posture_mes.at(3));

                vector<double> r_hand_pos; vector<double> r_wrist_pos; vector<double> r_elbow_pos; vector<double> r_shoulder_pos;
                this->curr_scene->getHumanoid()->getAllPos(1,r_hand_pos,r_wrist_pos,r_elbow_pos,r_shoulder_pos,r_arm_posture);
                vector<double> r_hand_pos_q; vector<double> r_wrist_pos_q; vector<double> r_elbow_pos_q; vector<double> r_shoulder_pos_q;
                this->curr_scene->getHumanoid()->getAllPos_q(1,r_hand_pos_q,r_wrist_pos_q,r_elbow_pos_q,r_shoulder_pos_q,r_arm_posture);

                vector<double> r_hand_lin_pos(r_hand_pos.begin(), r_hand_pos.begin()+3);
                vector<double> r_hand_ang_pos(r_hand_pos.begin()+3, r_hand_pos.begin()+6);
                vector<double> r_hand_q(r_hand_pos_q.begin()+3, r_hand_pos_q.begin()+7);
                vector<double> r_wrist_lin_pos(r_wrist_pos.begin(), r_wrist_pos.begin()+3);
                vector<double> r_wrist_ang_pos(r_wrist_pos.begin()+3, r_wrist_pos.begin()+6);
                vector<double> r_elbow_lin_pos(r_elbow_pos.begin(), r_elbow_pos.begin()+3);
                vector<double> r_elbow_ang_pos(r_elbow_pos.begin()+3, r_elbow_pos.begin()+6);
                vector<double> r_shoulder_lin_pos(r_shoulder_pos.begin(), r_shoulder_pos.begin()+3);
                vector<double> r_shoulder_ang_pos(r_shoulder_pos.begin()+3, r_shoulder_pos.begin()+6);

                // get desired hand velocity
                VectorXd::Map(&hand_pos_vec_xd[0], hand_pos_vec_x.size()) = hand_pos_vec_x;
                this->des_hand_pose_buff->push(hand_pos_vec_xd);
                if(this->samples_des_hand_pose==this->N_filter_length-1 && this->des_hand_pose_buff->full()){
                    for(size_t i=0; i< hand_pos_vec_xd.size();++i)
                    {
                        hand_vel_vec_x(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->des_hand_pose_buff->at(i));
                    }
                }else{this->samples_des_hand_pose++;}

                // predicted swivel angle
                double alpha_predicted = RADTODEG*this->getPredictedSwivelAngle(hand_pos_vec_x);
                //BOOST_LOG_SEV(lg, info) << "alpha_predicted = " << alpha_predicted;

                // derivative of the predicted swivel angle
                double der_alpha_predicted = RADTODEG*this->getDerivativePredictedSwivelAngle(hand_pos_vec_x,hand_vel_vec_x);
                //BOOST_LOG_SEV(lg, info) << "der_alpha_predicted = " << der_alpha_predicted;

//                double curr_alpha = this->curr_scene->getHumanoid()->getSwivelAngle(1,r_arm_posture);


//                // get desired hand acceleration
//                this->des_hand_vel_buff->push(des_hand_vel_vec);
//                if(this->samples_des_hand_vel==this->N_filter_length-1 && this->des_hand_vel_buff->full()){
//                    for(size_t i=0; i< des_hand_vel_vec.size();++i)
//                    {
//                        des_hand_acc_vec.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->des_hand_vel_buff->at(i));
//                    }
//                }else{this->samples_des_hand_vel++;}
//                des_hand_acc = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(des_hand_acc_vec.data(), des_hand_acc_vec.size());


//                vector<double> r_arm_velocities_mes;
//                vector<double> r_hand_velocities_mes(JOINTS_HAND,0.0);
//                if(this->ui.checkBox_use_velocity_based_ctrl->isChecked())
//                {
//                    this->curr_scene->getHumanoid()->getRightArmVelocities(r_arm_velocities_read);
//                    this->curr_scene->getHumanoid()->getRightHandVelocities(r_hand_velocities_read);
//                // filter noise from arm velocities
//                r_arm_velocities.at(0) = this->lpf_r_arm_vel_1->update(r_arm_velocities_mes.at(0));
//                r_arm_velocities.at(1) = this->lpf_r_arm_vel_2->update(r_arm_velocities_mes.at(1));
//                r_arm_velocities.at(2) = this->lpf_r_arm_vel_3->update(r_arm_velocities_mes.at(2));
//                r_arm_velocities.at(3) = this->lpf_r_arm_vel_4->update(r_arm_velocities_mes.at(3));
//                r_arm_velocities.at(4) = this->lpf_r_arm_vel_5->update(r_arm_velocities_mes.at(4));
//                r_arm_velocities.at(5) = this->lpf_r_arm_vel_6->update(r_arm_velocities_mes.at(5));
//                r_arm_velocities.at(6) = this->lpf_r_arm_vel_7->update(r_arm_velocities_mes.at(6));
//                }else{
                // get the joint velocities
                this->arm_pos_buff->push(r_arm_posture);
                this->hand_pos_buff->push(r_hand_posture);
                if(this->samples_pos==this->N_filter_length-1 && this->arm_pos_buff->full() && this->hand_pos_buff->full()){
                    for(size_t i=0; i< r_arm_posture.size();++i)
                    {
                        r_arm_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_pos_buff->at(i));
                    }
                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
                    {
                        r_hand_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_pos_buff->at(i));
                    }
                }else{this->samples_pos++;}

//                }
//                VectorXd r_arm_velocities_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_arm_velocities.data(), r_arm_velocities.size());

                vector<double> r_hand_vel; vector<double> r_wrist_vel; vector<double> r_elbow_vel; vector<double> r_shoulder_vel;
                this->curr_scene->getHumanoid()->getAllVel(1,r_hand_vel,r_wrist_vel,r_elbow_vel,r_shoulder_vel,r_arm_posture,r_arm_velocities_read);

                vector<double> r_hand_lin_vel(r_hand_vel.begin(), r_hand_vel.begin()+3);
                vector<double> r_hand_ang_vel(r_hand_vel.begin()+3, r_hand_vel.begin()+6);
                vector<double> r_wrist_lin_vel(r_wrist_vel.begin(), r_wrist_vel.begin()+3);
                vector<double> r_wrist_ang_vel(r_wrist_vel.begin()+3, r_wrist_vel.begin()+6);
                vector<double> r_elbow_lin_vel(r_elbow_vel.begin(), r_elbow_vel.begin()+3);
                vector<double> r_elbow_ang_vel(r_elbow_vel.begin()+3, r_elbow_vel.begin()+6);
                vector<double> r_shoulder_lin_vel(r_shoulder_vel.begin(), r_shoulder_vel.begin()+3);
                vector<double> r_shoulder_ang_vel(r_shoulder_vel.begin()+3, r_shoulder_vel.begin()+6);

                // get the joint accelerations
                this->arm_vel_buff->push(r_arm_velocities_read);
                this->hand_vel_buff->push(r_hand_velocities_read);
                if(this->samples_vel==this->N_filter_length-1 && this->arm_vel_buff->full() && this->hand_vel_buff->full()){
                    for(size_t i=0; i< r_arm_velocities_read.size();++i)
                    {
                        r_arm_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_vel_buff->at(i));
                    }
                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
                    {
                        r_hand_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_vel_buff->at(i));
                    }
                }else{this->samples_vel++;}


                // get the hand acceleration
                this->r_hand_vel_buff->push(r_hand_vel);
                if(this->samples_h_vel==this->N_filter_length-1 && this->r_hand_vel_buff->full()){
                    for(size_t i=0; i< r_hand_vel.size();++i)
                    {
                        r_hand_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_hand_vel_buff->at(i));
                    }
                }else{this->samples_h_vel++;}


                // get the wrist acceleration
                this->r_wrist_vel_buff->push(r_wrist_vel);
                if(this->samples_w_vel==this->N_filter_length-1 && this->r_wrist_vel_buff->full()){
                    for(size_t i=0; i< r_wrist_vel.size();++i)
                    {
                        r_wrist_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_wrist_vel_buff->at(i));
                    }
                }else{this->samples_w_vel++;}

                // get the elbow acceleration
                this->r_elbow_vel_buff->push(r_elbow_vel);
                if(this->samples_e_vel==this->N_filter_length-1 && this->r_elbow_vel_buff->full()){
                    for(size_t i=0; i< r_elbow_vel.size();++i)
                    {
                        r_elbow_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_elbow_vel_buff->at(i));
                    }
                }else{this->samples_e_vel++;}

                // get the shoulder acceleration
                this->r_shoulder_vel_buff->push(r_shoulder_vel);
                if(this->samples_s_vel==this->N_filter_length-1 && this->r_shoulder_vel_buff->full()){
                    for(size_t i=0; i< r_shoulder_vel.size();++i)
                    {
                        r_shoulder_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_shoulder_vel_buff->at(i));
                    }
                }else{
                    this->samples_s_vel++;
                    if(sim_robot){
                        this->t_der_past = this->qnode.getSimTime();
                    }else{
                        this->t_der_past_ctrl = boost::chrono::duration_cast<msec>(Clock::now() - this->start_time_point);
                    }
                }
                VectorXd r_arm_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_arm_accelerations_read.data(), r_arm_accelerations_read.size());
                VectorXd r_hand_acc_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_acc_read.data(), r_hand_acc_read.size());
                VectorXd r_hand_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_accelerations_read.data(), r_hand_accelerations_read.size());
                // Jacobian
                this->curr_scene->getHumanoid()->getJacobian(1,r_arm_posture,this->Jacobian);
                this->hand_j_acc = r_hand_acc_read_vec - this->Jacobian*r_arm_accelerations_read_vec;

                vector<double> r_hand_lin_acc(r_hand_acc_read.begin(), r_hand_acc_read.begin()+3);
                vector<double> r_hand_ang_acc(r_hand_acc_read.begin()+3, r_hand_acc_read.begin()+6);
                vector<double> r_wrist_lin_acc(r_wrist_acc_read.begin(), r_wrist_acc_read.begin()+3);
                vector<double> r_wrist_ang_acc(r_wrist_acc_read.begin()+3, r_wrist_acc_read.begin()+6);
                vector<double> r_elbow_lin_acc(r_elbow_acc_read.begin(), r_elbow_acc_read.begin()+3);
                vector<double> r_elbow_ang_acc(r_elbow_acc_read.begin()+3, r_elbow_acc_read.begin()+6);
                vector<double> r_shoulder_lin_acc(r_shoulder_acc_read.begin(), r_shoulder_acc_read.begin()+3);
                vector<double> r_shoulder_ang_acc(r_shoulder_acc_read.begin()+3, r_shoulder_acc_read.begin()+6);


                Vector3d r_hand_pos_vec; Vector3d r_hand_or_q_e; double r_hand_q_w = r_hand_q.at(3);
                Vector3d r_hand_lin_vel_vec; Vector3d r_hand_ang_vel_vec;
                r_hand_pos_vec << r_hand_lin_pos.at(0),r_hand_lin_pos.at(1),r_hand_lin_pos.at(2);
                r_hand_lin_vel_vec << r_hand_lin_vel.at(0),r_hand_lin_vel.at(1),r_hand_lin_vel.at(2);
                r_hand_ang_vel_vec << r_hand_ang_vel.at(0),r_hand_ang_vel.at(1),r_hand_ang_vel.at(2);
                r_hand_or_q_e << r_hand_q.at(0),r_hand_q.at(1),r_hand_q.at(2);

                // current hand angular velocity (quaternion)
                Vector3d r_hand_ang_vel_q_e = 0.5*(r_hand_q_w*I_3*r_hand_ang_vel_vec-r_hand_or_q_e.cross(r_hand_ang_vel_vec));
                double r_hand_ang_vel_q_w = -0.5*(r_hand_ang_vel_q_e.dot(r_hand_ang_vel_vec));


                // error in position + orientation (quaternion)
                Vector3d error_pos = des_hand_pos - r_hand_pos_vec;
                Vector3d error_or = r_hand_q_w*des_hand_or_q_e - des_hand_or_q_w*r_hand_or_q_e - des_hand_or_q_e.cross(r_hand_or_q_e);
                VectorXd error_tot(6); error_tot << error_pos(0),error_pos(1),error_pos(2), error_or(0),error_or(1),error_or(2);
                VectorXd error_tot_init(6);
//                // error in velocity
//                Vector3d error_lin_vel = des_hand_lin_vel - r_hand_lin_vel_vec;
//                Vector3d error_ang_vel = r_hand_ang_vel_q_w*des_hand_or_q_e + r_hand_q_w*des_hand_ang_vel_q_e - des_hand_ang_vel_q_w*r_hand_or_q_e - des_hand_or_q_w*r_hand_ang_vel_q_e
//                                        - des_hand_ang_vel_q_e.cross(r_hand_or_q_e) - des_hand_or_q_e.cross(r_hand_ang_vel_q_e);
//                VectorXd der_error_tot(6); der_error_tot << error_lin_vel(0),error_lin_vel(1),error_lin_vel(2),error_ang_vel(0),error_ang_vel(1),error_ang_vel(2);


                if(sim_robot){
                    this->curr_time = this->qnode.getSimTime() - this->t_past - this->t_der_past;
//                BOOST_LOG_SEV(lg, info) << "curr_time = " << curr_time;
//                BOOST_LOG_SEV(lg, info) << "t_past = " << this->t_past;
//                BOOST_LOG_SEV(lg, info) << "t_der_past = " << this->t_der_past;
//                BOOST_LOG_SEV(lg, info) << "sim_time = " << this->qnode.getSimTime();
                }else{
                    this->curr_time_ctrl = Clock::now() - this->t_past_ctrl - this->t_der_past_ctrl;
                    //double d_curr = (boost::chrono::duration_cast<msec>(this->curr_time_ctrl - this->start_time_point)).count();
                    //BOOST_LOG_SEV(lg, info) << "curr_time_ctrl = " << d_curr/1000;
                    //BOOST_LOG_SEV(lg, info) << "t_past_ctrl = " << this->t_past_ctrl.count();
                    //BOOST_LOG_SEV(lg, info) << "t_der_past_ctrl = " << this->t_der_past_ctrl.count();
                }


                VectorXd trap_hand_pose(JOINTS_ARM); // trapezoidal desired hand pose
                VectorXd trap_hand_vel(JOINTS_ARM);  // trapezoidal desired hand velocity
                VectorXd trap_hand_acc(JOINTS_ARM);  // trapezoidal desired hand acceleration
                VectorXd h_hand_pose(JOINTS_ARM); // human-like desired hand pose
                VectorXd h_hand_vel(JOINTS_ARM);  // human-like desired hand velocity
                VectorXd h_hand_acc(JOINTS_ARM);  // human-like desired hand acceleration
                VectorXd h_fing_pos(JOINTS_HAND); // human-like desired finger position
                VectorXd h_fing_vel(JOINTS_HAND);  // human-like desired finger velocity
                VectorXd h_fing_acc(JOINTS_HAND);  // human-like desired finger acceleration
                Vector3d hand_pos_init_vec;
                Vector3d hand_or_q_e_init_vec; double hand_or_q_w_init_vec = this->h_hand_or_q_init.at(3);
                hand_pos_init_vec << this->h_hand_pos_init.at(0),this->h_hand_pos_init.at(1),this->h_hand_pos_init.at(2);
                hand_or_q_e_init_vec << this->h_hand_or_q_init.at(0),this->h_hand_or_q_init.at(1),this->h_hand_or_q_init.at(2);


                Vector3d error_f_pos = des_hand_pos - hand_pos_init_vec;
                VectorXd error_f_orr(4);
                error_f_orr(0) = des_hand_or_q_x - hand_or_q_e_init_vec(0);
                error_f_orr(1) = des_hand_or_q_y - hand_or_q_e_init_vec(1);
                error_f_orr(2) = des_hand_or_q_z - hand_or_q_e_init_vec(2);
                error_f_orr(3) = des_hand_or_q_w - hand_or_q_w_init_vec;
                VectorXd error_f_tot(7); error_f_tot << error_f_pos(0),error_f_pos(1),error_f_pos(2),error_f_orr(0),error_f_orr(1),error_f_orr(2),error_f_orr(3);

                VectorXd error_f_fing_tot = jointsFinalPosition_hand - jointsInitPosition_hand;

                VectorXd error_trap_tot(6); // error with the trapezoidal pose
                VectorXd der_error_trap_tot(6); // time derivative of the error with the trapezoidal pose
                VectorXd error_h_tot(6); // error with the human-like pose
                VectorXd der_error_h_tot(6); // time derivative of the error with the human-like pose
                VectorXd error_h_fing_tot(JOINTS_HAND); // error with the human-like pose
                VectorXd der_error_h_fing_tot(JOINTS_HAND); // time derivative of the error with the human-like pose
                VectorXd trap_hand_ref_vel(6); // trapezoidal reference velocity
                //VectorXd h_hand_ref_vel(6); // human-like reference velocity
                VectorXd trap_hand_ref_acc(6); // trapezoidal reference acceleration
                VectorXd h_hand_ref_acc(6); // human-like reference acceleration
                VectorXd h_fing_ref_acc(JOINTS_HAND); // human-like reference acceleration

                double g_map = 0.0; double g_map_fing = 0.0; int index = 0;// normalized mapped time
                if(this->ui.checkBox_use_plan_hand_pos->isChecked())
                {

                    if(hl_en)
                    { // human-like velocity profile
                        double tau = 0.1; double dec_rate = 0.1; double diff_w = 0.1;

                        if(stage_descr.compare("plan")==0)
                        {
                            this->mTimeMapdlg->getPlanTimeMapping(tau,dec_rate,diff_w);
                            // normalized mapped time
                            if(sim_robot){
                                g_map = 1 - exp((-dec_rate*this->curr_time)/(tau*(1+diff_w*error_tot.squaredNorm())));
                                //TO DO g_map_fing = 1 - (error_tot.squaredNorm()/error_tot_init.squaredNorm());
                            }else{
                                double d_curr_time = (boost::chrono::duration_cast<msec>(this->curr_time_ctrl-this->start_time_point)).count();
                                g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w*error_tot.squaredNorm())));
                            }
                            index = static_cast<int>(0.5+(n_steps-1)*g_map);

                            //                BOOST_LOG_SEV(lg, info) << "# ----------------Time Mapping ------------------- # ";
                            //                BOOST_LOG_SEV(lg, info) << "g_map = " << g_map;
                            //                BOOST_LOG_SEV(lg, info) << "index = " << index;
                            //                BOOST_LOG_SEV(lg, info) << "sim_time = " << this->qnode.getSimTime();
                            //                BOOST_LOG_SEV(lg, info) << "n_steps = " << n_steps;
                            //                BOOST_LOG_SEV(lg, info) << "error_tot_squared_norm = " << error_tot.squaredNorm();

                            hl_p_pos_coeff = hl_p_pos_coeff_plan; hl_p_or_coeff = hl_p_or_coeff_plan;
                            hl_d_pos_coeff = hl_d_pos_coeff_plan; hl_d_or_coeff = hl_d_or_coeff_plan;

                            Vector3d bounce_hand_pos; bounce_hand_pos << bounce_hand_pos_x,bounce_hand_pos_y,bounce_hand_pos_z;
                            Vector3d error_b_pos = bounce_hand_pos - hand_pos_init_vec;

                            VectorXd error_b_orr(4);
                            error_b_orr(0) = bounce_hand_q_x - hand_or_q_e_init_vec(0);
                            error_b_orr(1) = bounce_hand_q_y - hand_or_q_e_init_vec(1);
                            error_b_orr(2) = bounce_hand_q_z - hand_or_q_e_init_vec(2);
                            error_b_orr(3) = bounce_hand_q_w - hand_or_q_w_init_vec;
                            VectorXd error_b_tot(7); error_b_tot << error_b_pos(0),error_b_pos(1),error_b_pos(2),
                                                    error_b_orr(0),error_b_orr(1),error_b_orr(2),error_b_orr(3);

                            VectorXd error_b_fing_tot = jointsBouncePosition_hand - jointsInitPosition_hand;

                            // human-like desired hand pose
                            h_hand_pose(0) = hand_pos_init_vec(0) + error_f_tot(0)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_lin_vel_init.at(0)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_lin_vel_end.at(0)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_lin_acc_init.at(0)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_lin_acc_end.at(0)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(0)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(1) = hand_pos_init_vec(1) + error_f_tot(1)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_lin_vel_init.at(1)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_lin_vel_end.at(1)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_lin_acc_init.at(1)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_lin_acc_end.at(1)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(1)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(2) = hand_pos_init_vec(2) +error_f_tot(2)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_lin_vel_init.at(2)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_lin_vel_end.at(2)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_lin_acc_init.at(2)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_lin_acc_end.at(2)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(2)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(3) = hand_or_q_e_init_vec(0) + error_f_tot(3)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_init(0)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_end(0)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_init(0)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_end(0)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(3)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(4) = hand_or_q_e_init_vec(1) + error_f_tot(4)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_init(1)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_end(1)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_init(1)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_end(1)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(4)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(5) = hand_or_q_e_init_vec(2) + error_f_tot(5)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_init(2)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_ang_vel_q_e_end(2)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_init(2)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_e_end(2)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(5)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_hand_pose(6) = hand_or_q_w_init_vec + error_f_tot(6)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +h_hand_ang_vel_q_w_init*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +h_hand_ang_vel_q_w_end*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_w_init*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*h_hand_ang_acc_q_w_end*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_tot(6)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            // human-like desired finger positions
                            h_fing_pos(0) = jointsInitPosition_hand(0) + error_f_fing_tot(0)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +jointsInitVelocity_hand(0)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +jointsFinalVelocity_hand(0)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*jointsInitAcceleration_hand(0)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*jointsFinalAcceleration_hand(0)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_fing_tot(0)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_fing_pos(1) = jointsInitPosition_hand(1) + error_f_fing_tot(1)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +jointsInitVelocity_hand(1)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +jointsFinalVelocity_hand(1)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*jointsInitAcceleration_hand(1)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*jointsFinalAcceleration_hand(1)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_fing_tot(1)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_fing_pos(2) = jointsInitPosition_hand(2) + error_f_fing_tot(2)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +jointsInitVelocity_hand(2)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +jointsFinalVelocity_hand(2)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*jointsInitAcceleration_hand(2)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*jointsFinalAcceleration_hand(2)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_fing_tot(2)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);

                            h_fing_pos(3) = jointsInitPosition_hand(3) + error_f_fing_tot(3)*(10*pow(g_map,3)-15*pow(g_map,4)+6*pow(g_map,5))
                                             +jointsInitVelocity_hand(3)*period_T*(g_map-6*pow(g_map,3)+8*pow(g_map,4)-3*pow(g_map,5))
                                             +jointsFinalVelocity_hand(3)*period_T*(-4*pow(g_map,3)+7*pow(g_map,4)-3*pow(g_map,5))
                                             +0.5*jointsInitAcceleration_hand(3)*pow(period_T,2)*(pow(g_map,2)-3*pow(g_map,3)+3*pow(g_map,4)-pow(g_map,5))
                                             +0.5*jointsFinalAcceleration_hand(3)*pow(period_T,2)*(pow(g_map,3)-2*pow(g_map,4)+pow(g_map,5))
                                             +error_b_fing_tot(3)*((g_map*(1-g_map))/(tb*(1-tb)))*pow(sin(M_PI*pow(g_map,phi)),2);


                            // human-like desired hand velocity
                            h_hand_vel(0) = (30/period_T)*error_f_tot(0)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_lin_vel_init.at(0)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_lin_vel_end.at(0)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_init.at(0)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_end.at(0)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(0)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(1) = (30/period_T)*error_f_tot(1)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_lin_vel_init.at(1)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_lin_vel_end.at(1)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_init.at(1)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_end.at(1)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(1)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(2) = (30/period_T)*error_f_tot(2)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_lin_vel_init.at(2)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_lin_vel_end.at(2)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_init.at(2)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_lin_acc_end.at(2)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(2)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(3) = (30/period_T)*error_f_tot(3)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_ang_vel_q_e_init(0)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_ang_vel_q_e_end(0)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_init(0)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_end(0)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(3)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(4) = (30/period_T)*error_f_tot(4)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_ang_vel_q_e_init(1)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_ang_vel_q_e_end(1)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_init(1)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_end(1)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(4)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(5) = (30/period_T)*error_f_tot(5)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_ang_vel_q_e_init(2)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_ang_vel_q_e_end(2)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_init(2)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_e_end(2)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(5)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_vel(6) = (30/period_T)*error_f_tot(6)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +h_hand_ang_vel_q_w_init*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +h_hand_ang_vel_q_w_end*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_w_init*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*h_hand_ang_acc_q_w_end*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_tot(6)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            // human-like desired finger velocity
                            h_fing_vel(0) = (30/period_T)*error_f_fing_tot(0)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +jointsInitVelocity_hand(0)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +jointsFinalVelocity_hand(0)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*jointsInitAcceleration_hand(0)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*jointsFinalAcceleration_hand(0)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_fing_tot(0)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_fing_vel(1) = (30/period_T)*error_f_fing_tot(1)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +jointsInitVelocity_hand(1)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +jointsFinalVelocity_hand(1)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*jointsInitAcceleration_hand(1)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*jointsFinalAcceleration_hand(1)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_fing_tot(1)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_fing_vel(2) = (30/period_T)*error_f_fing_tot(2)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +jointsInitVelocity_hand(2)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +jointsFinalVelocity_hand(2)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*jointsInitAcceleration_hand(2)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*jointsFinalAcceleration_hand(2)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_fing_tot(2)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));

                            h_fing_vel(3) = (30/period_T)*error_f_fing_tot(3)*(pow(g_map,2)-2*pow(g_map,3)+pow(g_map,4))
                                            +jointsInitVelocity_hand(3)*(1-18*pow(g_map,2)+32*pow(g_map,3)-15*pow(g_map,4))
                                            +jointsFinalVelocity_hand(3)*(-12*pow(g_map,2)+28*pow(g_map,3)-15*pow(g_map,4))
                                            +(0.5*jointsInitAcceleration_hand(3)*period_T)*(2*g_map-9*pow(g_map,2)+12*pow(g_map,3)-5*pow(g_map,4))
                                            +(0.5*jointsFinalAcceleration_hand(3)*period_T)*(3*pow(g_map,3)-8*pow(g_map,3)+5*pow(g_map,4))
                                            +(1/(period_T*tb*(1-tb)))*error_b_fing_tot(3)*((1-2*g_map)*pow(sin(M_PI*pow(g_map,phi)),2)+(1-g_map)*M_PI*phi*pow(g_map,phi)*sin(2*M_PI*pow(g_map,phi)));


                            // human-like desired hand acceleration
                            h_hand_acc(0) = (60/pow(period_T,2))*error_f_tot(0)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_init.at(0)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_end.at(0)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_lin_acc_init.at(0)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_lin_acc_end.at(0)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(0)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(1) = (60/pow(period_T,2))*error_f_tot(1)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_init.at(1)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_end.at(1)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_lin_acc_init.at(1)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_lin_acc_end.at(1)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(1)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(2) = (60/pow(period_T,2))*error_f_tot(2)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_init.at(2)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_lin_vel_end.at(2)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_lin_acc_init.at(2)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_lin_acc_end.at(2)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(2)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(3) = (60/pow(period_T,2))*error_f_tot(3)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_init(0)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_end(0)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_init(0)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_end(0)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(3)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(4) = (60/pow(period_T,2))*error_f_tot(4)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_init(1)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_end(1)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_init(1)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_end(1)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(4)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(5) = (60/pow(period_T,2))*error_f_tot(5)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_init(0)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_e_end(0)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_init(0)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_ang_acc_q_e_end(0)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(5)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_hand_acc(6) = (60/pow(period_T,2))*error_f_tot(6)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_w_init*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*h_hand_ang_vel_q_w_end*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +h_hand_ang_acc_q_w_init*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +h_hand_ang_acc_q_w_end*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_tot(6)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            // human-like desired finger acceleration
                            h_fing_acc(0) = (60/pow(period_T,2))*error_f_fing_tot(0)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*jointsInitVelocity_hand(0)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*jointsFinalVelocity_hand(0)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +jointsInitAcceleration_hand(0)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +jointsFinalAcceleration_hand(0)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_fing_tot(0)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));

                            h_fing_acc(1) = (60/pow(period_T,2))*error_f_fing_tot(1)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*jointsInitVelocity_hand(1)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*jointsFinalVelocity_hand(1)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +jointsInitAcceleration_hand(1)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +jointsFinalAcceleration_hand(1)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_fing_tot(1)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));


                            h_fing_acc(2) = (60/pow(period_T,2))*error_f_fing_tot(2)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*jointsInitVelocity_hand(2)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*jointsFinalVelocity_hand(2)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +jointsInitAcceleration_hand(2)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +jointsFinalAcceleration_hand(2)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_fing_tot(2)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));


                            h_fing_acc(3) = (60/pow(period_T,2))*error_f_fing_tot(3)*(g_map-3*pow(g_map,2)+2*pow(g_map,3))
                                            +(12/period_T)*jointsInitVelocity_hand(3)*(-3*g_map+8*pow(g_map,2)-5*pow(g_map,3))
                                            +(12/period_T)*jointsFinalVelocity_hand(3)*(-2*g_map+7*pow(g_map,2)-5*pow(g_map,3))
                                            +jointsInitAcceleration_hand(3)*(1-9*g_map+18*pow(g_map,2)-10*pow(g_map,3))
                                            +jointsFinalAcceleration_hand(3)*(3*g_map-12*pow(g_map,2)+10*pow(g_map,3))
                                            +(2/(pow(period_T,2)*tb*(1-tb)))*error_b_fing_tot(3)*(-pow(sin(M_PI*pow(g_map,phi)),2)+pow(M_PI,2)*pow(phi,2)*(1-g_map)*pow(g_map,2*phi-1)*cos(2*M_PI*pow(g_map,phi))
                                                                                             +(1-2*g_map)*M_PI*phi*pow(g_map,phi-1)*sin(2*M_PI*pow(g_map,phi)));


                            //                    BOOST_LOG_SEV(lg, info) << "# ---------------- plan ------------------- # ";
                            //                    BOOST_LOG_SEV(lg, info) << "error_f_orr x = " << error_f_orr(0);
                            //                    BOOST_LOG_SEV(lg, info) << "error_f_orr y = " << error_f_orr(1);
                            //                    BOOST_LOG_SEV(lg, info) << "error_f_orr z = " << error_f_orr(2);

                            //                    BOOST_LOG_SEV(lg, info) << "h hand pos x = " << h_hand_pose(0);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand pos y = " << h_hand_pose(1);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand pos z = " << h_hand_pose(2);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand or qx = " << h_hand_pose(3);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand or qy = " << h_hand_pose(4);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand or qz = " << h_hand_pose(5);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand or qw = " << h_hand_pose(6);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel x = " << h_hand_vel(0);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel y = " << h_hand_vel(1);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel z = " << h_hand_vel(2);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel x = " << h_hand_vel(3);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel y = " << h_hand_vel(4);
                            //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel z = " << h_hand_vel(5);

                        }else if(stage_descr.compare("approach")==0){

                            this->mTimeMapdlg->getApproachTimeMapping(tau,dec_rate,diff_w);
                            // normalized mapped time
                            if(sim_robot){
                                g_map = 1 - exp((-dec_rate*this->curr_time)/(tau*(1+diff_w*error_tot.squaredNorm())));
                            }else{
                                double d_curr_time = (boost::chrono::duration_cast<msec>(this->curr_time_ctrl-this->start_time_point)).count();
                                g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w*error_tot.squaredNorm())));
                            }
                            index = static_cast<int>(0.5+(n_steps-1)*g_map);

                            //                BOOST_LOG_SEV(lg, info) << "# ----------------Time Mapping approach ------------------- # ";
                            //                BOOST_LOG_SEV(lg, info) << "g_map = " << g_map;
                            //                BOOST_LOG_SEV(lg, info) << "index = " << index;
                            //                BOOST_LOG_SEV(lg, info) << "sim_time = " << this->qnode.getSimTime();
                            //                BOOST_LOG_SEV(lg, info) << "n_steps = " << n_steps;
                            //                BOOST_LOG_SEV(lg, info) << "error_tot_squared_norm = " << error_tot.squaredNorm();

                            hl_p_pos_coeff = hl_p_pos_coeff_app; hl_p_or_coeff = hl_p_or_coeff_app;
                            hl_d_pos_coeff = hl_d_pos_coeff_app; hl_d_or_coeff = hl_d_or_coeff_app;

                            // human-like desired hand pose
                            h_hand_pose(0) = hand_pos_init_vec(0) + 0.25*error_f_tot(0)*(5*g_map-pow(g_map,5));
                            h_hand_pose(1) = hand_pos_init_vec(1) + 0.25*error_f_tot(1)*(5*g_map-pow(g_map,5));
                            h_hand_pose(2) = hand_pos_init_vec(2) + 0.25*error_f_tot(2)*(5*g_map-pow(g_map,5));
                            h_hand_pose(3) = hand_or_q_e_init_vec(0) + 0.25*error_f_tot(3)*(5*g_map-pow(g_map,5));
                            h_hand_pose(4) = hand_or_q_e_init_vec(1) + 0.25*error_f_tot(4)*(5*g_map-pow(g_map,5));
                            h_hand_pose(5) = hand_or_q_e_init_vec(2) + 0.25*error_f_tot(5)*(5*g_map-pow(g_map,5));
                            h_hand_pose(6) = hand_or_q_w_init_vec + 0.25*error_f_tot(6)*(5*g_map-pow(g_map,5));

                            // human-like desired finger position
                            h_fing_pos(0) = jointsInitPosition_hand(0) + 0.25*error_f_fing_tot(0)*(5*g_map-pow(g_map,5));
                            h_fing_pos(1) = jointsInitPosition_hand(1) + 0.25*error_f_fing_tot(1)*(5*g_map-pow(g_map,5));
                            h_fing_pos(2) = jointsInitPosition_hand(2) + 0.25*error_f_fing_tot(2)*(5*g_map-pow(g_map,5));
                            h_fing_pos(3) = jointsInitPosition_hand(3) + 0.25*error_f_fing_tot(3)*(5*g_map-pow(g_map,5));

                            // human-like desired hand velocity
                            h_hand_vel(0) = (5/(4*period_T))*error_f_tot(0)*(1-pow(g_map,4));
                            h_hand_vel(1) = (5/(4*period_T))*error_f_tot(1)*(1-pow(g_map,4));
                            h_hand_vel(2) = (5/(4*period_T))*error_f_tot(2)*(1-pow(g_map,4));
                            h_hand_vel(3) = (5/(4*period_T))*error_f_tot(3)*(1-pow(g_map,4));
                            h_hand_vel(4) = (5/(4*period_T))*error_f_tot(4)*(1-pow(g_map,4));
                            h_hand_vel(5) = (5/(4*period_T))*error_f_tot(5)*(1-pow(g_map,4));
                            h_hand_vel(6) = (5/(4*period_T))*error_f_tot(6)*(1-pow(g_map,4));

                            // human-like desired joints velocity
                            h_fing_vel(0) = (5/(4*period_T))*error_f_fing_tot(0)*(1-pow(g_map,4));
                            h_fing_vel(1) = (5/(4*period_T))*error_f_fing_tot(1)*(1-pow(g_map,4));
                            h_fing_vel(2) = (5/(4*period_T))*error_f_fing_tot(2)*(1-pow(g_map,4));
                            h_fing_vel(3) = (5/(4*period_T))*error_f_fing_tot(3)*(1-pow(g_map,4));


                            // human-like desired hand acceleration
                            h_hand_acc(0) = -(5/pow(period_T,2))*error_f_tot(0)*pow(g_map,3);
                            h_hand_acc(1) = -(5/pow(period_T,2))*error_f_tot(1)*pow(g_map,3);
                            h_hand_acc(2) = -(5/pow(period_T,2))*error_f_tot(2)*pow(g_map,3);
                            h_hand_acc(3) = -(5/pow(period_T,2))*error_f_tot(3)*pow(g_map,3);
                            h_hand_acc(4) = -(5/pow(period_T,2))*error_f_tot(4)*pow(g_map,3);
                            h_hand_acc(5) = -(5/pow(period_T,2))*error_f_tot(5)*pow(g_map,3);
                            h_hand_acc(6) = -(5/pow(period_T,2))*error_f_tot(6)*pow(g_map,3);

                            // human-like desired finger acceleration
                            h_fing_acc(0) = -(5/pow(period_T,2))*error_f_fing_tot(0)*pow(g_map,3);
                            h_fing_acc(1) = -(5/pow(period_T,2))*error_f_fing_tot(1)*pow(g_map,3);
                            h_fing_acc(2) = -(5/pow(period_T,2))*error_f_fing_tot(2)*pow(g_map,3);
                            h_fing_acc(3) = -(5/pow(period_T,2))*error_f_fing_tot(3)*pow(g_map,3);


    //                    BOOST_LOG_SEV(lg, info) << "# ---------------- approach ------------------- # ";
    //                    BOOST_LOG_SEV(lg, info) << "h hand pos x = " << h_hand_pose(0);
    //                    BOOST_LOG_SEV(lg, info) << "h hand pos y = " << h_hand_pose(1);
    //                    BOOST_LOG_SEV(lg, info) << "h hand pos z = " << h_hand_pose(2);
    //                    BOOST_LOG_SEV(lg, info) << "h hand or qx = " << h_hand_pose(3);
    //                    BOOST_LOG_SEV(lg, info) << "h hand or qy = " << h_hand_pose(4);
    //                    BOOST_LOG_SEV(lg, info) << "h hand or qz = " << h_hand_pose(5);
    //                    BOOST_LOG_SEV(lg, info) << "h hand or qw = " << h_hand_pose(6);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel x = " << h_hand_vel(0);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel y = " << h_hand_vel(1);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel z = " << h_hand_vel(2);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel x = " << h_hand_vel(3);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel y = " << h_hand_vel(4);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel z = " << h_hand_vel(5);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin acc x = " << h_hand_acc(0);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin acc y = " << h_hand_acc(1);
    //                    BOOST_LOG_SEV(lg, info) << "h hand lin acc z = " << h_hand_acc(2);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang acc x = " << h_hand_acc(3);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang acc y = " << h_hand_acc(4);
    //                    BOOST_LOG_SEV(lg, info) << "h hand ang acc z = " << h_hand_acc(5);

                        }else if(stage_descr.compare("retreat")==0){

                            this->mTimeMapdlg->getRetreatTimeMapping(tau,dec_rate,diff_w);
                            // normalized mapped time
                            if(sim_robot){
                                g_map = 1 - exp((-dec_rate*this->curr_time)/(tau*(1+diff_w*error_tot.squaredNorm())));
                            }else{
                                double d_curr_time = (boost::chrono::duration_cast<msec>(this->curr_time_ctrl-this->start_time_point)).count();
                                g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w*error_tot.squaredNorm())));
                            }
                            index = static_cast<int>(0.5+(n_steps-1)*g_map);

            //                BOOST_LOG_SEV(lg, info) << "# ----------------Time Mapping ------------------- # ";
            //                BOOST_LOG_SEV(lg, info) << "g_map = " << g_map;
            //                BOOST_LOG_SEV(lg, info) << "index = " << index;
            //                BOOST_LOG_SEV(lg, info) << "sim_time = " << this->qnode.getSimTime();
            //                BOOST_LOG_SEV(lg, info) << "n_steps = " << n_steps;
            //                BOOST_LOG_SEV(lg, info) << "error_tot_squared_norm = " << error_tot.squaredNorm();

                            hl_p_pos_coeff = hl_p_pos_coeff_ret; hl_p_or_coeff = hl_p_or_coeff_ret;
                            hl_d_pos_coeff = hl_d_pos_coeff_ret; hl_d_or_coeff = hl_d_or_coeff_ret;

                            // human-like desired hand pose
                            h_hand_pose(0) = hand_pos_init_vec(0) + 0.33*error_f_tot(0)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(1) = hand_pos_init_vec(1) + 0.33*error_f_tot(1)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(2) = hand_pos_init_vec(2) + 0.33*error_f_tot(2)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(3) = hand_or_q_e_init_vec(0) + 0.33*error_f_tot(3)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(4) = hand_or_q_e_init_vec(1) + 0.33*error_f_tot(4)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(5) = hand_or_q_e_init_vec(2) + 0.33*error_f_tot(5)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_hand_pose(6) = hand_or_q_w_init_vec + 0.33*error_f_tot(6)*(5*pow(g_map,4)-2*pow(g_map,5));

                            // human-like desired finger position
                            h_fing_pos(0) = jointsInitPosition_hand(0) + 0.33*error_f_fing_tot(0)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_fing_pos(1) = jointsInitPosition_hand(1) + 0.33*error_f_fing_tot(1)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_fing_pos(2) = jointsInitPosition_hand(2) + 0.33*error_f_fing_tot(2)*(5*pow(g_map,4)-2*pow(g_map,5));
                            h_fing_pos(3) = jointsInitPosition_hand(3) + 0.33*error_f_fing_tot(3)*(5*pow(g_map,4)-2*pow(g_map,5));

                            // human-like desired hand velocity
                            h_hand_vel(0) = (10/(3*period_T))*error_f_tot(0)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(1) = (10/(3*period_T))*error_f_tot(1)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(2) = (10/(3*period_T))*error_f_tot(2)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(3) = (10/(3*period_T))*error_f_tot(3)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(4) = (10/(3*period_T))*error_f_tot(4)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(5) = (10/(3*period_T))*error_f_tot(5)*(2*pow(g_map,3)-pow(g_map,4));
                            h_hand_vel(6) = (10/(3*period_T))*error_f_tot(6)*(2*pow(g_map,3)-pow(g_map,4));

                            // human-like desired finger velocity
                            h_fing_vel(0) = (10/(3*period_T))*error_f_fing_tot(0)*(2*pow(g_map,3)-pow(g_map,4));
                            h_fing_vel(1) = (10/(3*period_T))*error_f_fing_tot(1)*(2*pow(g_map,3)-pow(g_map,4));
                            h_fing_vel(2) = (10/(3*period_T))*error_f_fing_tot(2)*(2*pow(g_map,3)-pow(g_map,4));
                            h_fing_vel(3) = (10/(3*period_T))*error_f_fing_tot(3)*(2*pow(g_map,3)-pow(g_map,4));


                            // human-like desired hand acceleration
                            h_hand_acc(0) = (20/(3*pow(period_T,2)))*error_f_tot(0)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(1) = (20/(3*pow(period_T,2)))*error_f_tot(1)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(2) = (20/(3*pow(period_T,2)))*error_f_tot(2)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(3) = (20/(3*pow(period_T,2)))*error_f_tot(3)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(4) = (20/(3*pow(period_T,2)))*error_f_tot(4)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(5) = (20/(3*pow(period_T,2)))*error_f_tot(5)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_hand_acc(6) = (20/(3*pow(period_T,2)))*error_f_tot(6)*(3*pow(g_map,2)-2*pow(g_map,3));

                            // human-like desired finger acceleration
                            h_fing_acc(0) = (20/(3*pow(period_T,2)))*error_f_fing_tot(0)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_fing_acc(1) = (20/(3*pow(period_T,2)))*error_f_fing_tot(1)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_fing_acc(2) = (20/(3*pow(period_T,2)))*error_f_fing_tot(2)*(3*pow(g_map,2)-2*pow(g_map,3));
                            h_fing_acc(3) = (20/(3*pow(period_T,2)))*error_f_fing_tot(3)*(3*pow(g_map,2)-2*pow(g_map,3));


        //                    BOOST_LOG_SEV(lg, info) << "# ---------------- retreat ------------------- # ";
        //                    BOOST_LOG_SEV(lg, info) << "h hand pos x = " << h_hand_pose(0);
        //                    BOOST_LOG_SEV(lg, info) << "h hand pos y = " << h_hand_pose(1);
        //                    BOOST_LOG_SEV(lg, info) << "h hand pos z = " << h_hand_pose(2);
        //                    BOOST_LOG_SEV(lg, info) << "h hand or qx = " << h_hand_pose(3);
        //                    BOOST_LOG_SEV(lg, info) << "h hand or qy = " << h_hand_pose(4);
        //                    BOOST_LOG_SEV(lg, info) << "h hand or qz = " << h_hand_pose(5);
        //                    BOOST_LOG_SEV(lg, info) << "h hand or qw = " << h_hand_pose(6);
        //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel x = " << h_hand_vel(0);
        //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel y = " << h_hand_vel(1);
        //                    BOOST_LOG_SEV(lg, info) << "h hand lin vel z = " << h_hand_vel(2);
        //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel x = " << h_hand_vel(3);
        //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel y = " << h_hand_vel(4);
        //                    BOOST_LOG_SEV(lg, info) << "h hand ang vel z = " << h_hand_vel(5);


                        }

                    // hand
                    // error in position
                    Vector3d error_h_pos;
                    error_h_pos(0) = h_hand_pose(0) - r_hand_pos_vec(0);
                    error_h_pos(1) = h_hand_pose(1) - r_hand_pos_vec(1);
                    error_h_pos(2) = h_hand_pose(2) - r_hand_pos_vec(2);
                    // error in orientation
                    Quaterniond h_hand_or_q;
                    h_hand_or_q.x() = h_hand_pose(3); h_hand_or_q.y() = h_hand_pose(4); h_hand_or_q.z() = h_hand_pose(5); h_hand_or_q.w() = h_hand_pose(6);
                    Vector3d h_hand_or_q_e; h_hand_or_q_e << h_hand_pose(3),h_hand_pose(4),h_hand_pose(5); double h_hand_or_q_w = h_hand_pose(6);
                    Vector3d error_h_or = r_hand_q_w*h_hand_or_q_e - h_hand_or_q_w*r_hand_or_q_e - h_hand_or_q_e.cross(r_hand_or_q_e);
                    // total error in position + orientation
                    error_h_tot << error_h_pos(0),error_h_pos(1),error_h_pos(2), error_h_or(0),error_h_or(1),error_h_or(2);                    
                    // fingers
                    error_h_fing_tot(0) = h_fing_pos(0) - r_hand_posture.at(0);
                    error_h_fing_tot(1) = h_fing_pos(1) - r_hand_posture.at(1);
                    error_h_fing_tot(2) = h_fing_pos(2) - r_hand_posture.at(2);
                    error_h_fing_tot(3) = h_fing_pos(3) - r_hand_posture.at(3);

                    // hand
                    // error in linear velocity
                    Vector3d error_h_lin_vel;
                    error_h_lin_vel(0) = h_hand_vel(0) - r_hand_lin_vel_vec(0);
                    error_h_lin_vel(1) = h_hand_vel(1) - r_hand_lin_vel_vec(1);
                    error_h_lin_vel(2) = h_hand_vel(2) - r_hand_lin_vel_vec(2);
                    // error in orientation velocity (quaternion)
                    Vector3d h_hand_ang_vel_q_e; h_hand_ang_vel_q_e << h_hand_vel(3),h_hand_vel(4),h_hand_vel(5); double h_hand_ang_vel_q_w = h_hand_vel(6);
                    Vector3d error_h_der_or = r_hand_ang_vel_q_w*h_hand_or_q_e + r_hand_q_w*h_hand_ang_vel_q_e - h_hand_ang_vel_q_w*r_hand_or_q_e
                                            - h_hand_or_q_w*r_hand_ang_vel_q_e - h_hand_ang_vel_q_e.cross(r_hand_or_q_e) - h_hand_or_q_e.cross(r_hand_ang_vel_q_e);
                    // total error in velocity
                    der_error_h_tot << error_h_lin_vel(0),error_h_lin_vel(1),error_h_lin_vel(2),error_h_der_or(0),error_h_der_or(1),error_h_der_or(2);

                    // fingers
                    der_error_h_fing_tot(0) = h_fing_vel(0) - r_hand_velocities_read.at(0);
                    der_error_h_fing_tot(1) = h_fing_vel(1) - r_hand_velocities_read.at(1);
                    der_error_h_fing_tot(2) = h_fing_vel(2) - r_hand_velocities_read.at(2);
                    der_error_h_fing_tot(3) = h_fing_vel(3) - r_hand_velocities_read.at(3);


//                    // human-like reference velocity
//                    h_hand_ref_vel(0) = h_hand_vel(0);
//                    h_hand_ref_vel(1) = h_hand_vel(1);
//                    h_hand_ref_vel(2) = h_hand_vel(2);
//                    Vector3d h_hand_vel_q_e; h_hand_vel_q_e << h_hand_vel(3), h_hand_vel(4), h_hand_vel(5); double h_hand_vel_q_w = h_hand_vel(6);
//                    Vector3d des_omega = 2*h_hand_or_q_e.cross(h_hand_vel_q_e)+2*h_hand_or_q_w*h_hand_vel_q_e-2*h_hand_vel_q_w*h_hand_or_q_e;
//                    h_hand_ref_vel(3) = des_omega(0);
//                    h_hand_ref_vel(4) = des_omega(1);
//                    h_hand_ref_vel(5) = des_omega(2);

                    // human-like reference hand acceleration
                    h_hand_ref_acc(0) = h_hand_acc(0);
                    h_hand_ref_acc(1) = h_hand_acc(1);
                    h_hand_ref_acc(2) = h_hand_acc(2);
                    Vector3d h_hand_acc_q_e; h_hand_acc_q_e << h_hand_acc(3), h_hand_acc(4), h_hand_acc(5); double h_hand_acc_q_w = h_hand_acc(6);
                    Vector3d des_alpha = 2*h_hand_or_q_e.cross(h_hand_acc_q_e) + 2*h_hand_or_q_w*h_hand_acc_q_e - 2*h_hand_acc_q_w*h_hand_or_q_e;
                    h_hand_ref_acc(3) = des_alpha(0);
                    h_hand_ref_acc(4) = des_alpha(1);
                    h_hand_ref_acc(5) = des_alpha(2);

                    // human-like reference fingers acceleration
                    h_fing_ref_acc(0) = h_fing_acc(0);
                    h_fing_ref_acc(1) = h_fing_acc(1);
                    h_fing_ref_acc(2) = h_fing_acc(2);
                    h_fing_ref_acc(3) = h_fing_acc(3);


    //                BOOST_LOG_SEV(lg, info) << "# ---------------- h_hand_ref_vel ------------------- # ";
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_vel x = " << h_hand_ref_vel(0);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_vel y = " << h_hand_ref_vel(1);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_vel z = " << h_hand_ref_vel(2);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_vel x = " << h_hand_ref_vel(3);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_vel y = " << h_hand_ref_vel(4);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_vel z = " << h_hand_ref_vel(5);

    //                BOOST_LOG_SEV(lg, info) << "# ---------------- h_hand_ref_acc ------------------- # ";
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_acc x = " << h_hand_ref_acc(0);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_acc y = " << h_hand_ref_acc(1);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_lin_acc z = " << h_hand_ref_acc(2);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_acc x = " << h_hand_ref_acc(3);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_acc y = " << h_hand_ref_acc(4);
    //                BOOST_LOG_SEV(lg, info) << "h_hand_ref_ang_acc z = " << h_hand_ref_acc(5);

    //                BOOST_LOG_SEV(lg, info) << "# ---------------- errors ------------------- # ";
    //                BOOST_LOG_SEV(lg, info) << "error pos x = " << error_h_tot(0);
    //                BOOST_LOG_SEV(lg, info) << "error pos y = " << error_h_tot(1);
    //                BOOST_LOG_SEV(lg, info) << "error pos z = " << error_h_tot(2);
    //                BOOST_LOG_SEV(lg, info) << "error or qx = " << error_h_tot(3);
    //                BOOST_LOG_SEV(lg, info) << "error or qy = " << error_h_tot(4);
    //                BOOST_LOG_SEV(lg, info) << "error or qz = " << error_h_tot(5);
    //                BOOST_LOG_SEV(lg, info) << "error vel x = " << der_error_h_tot(0);
    //                BOOST_LOG_SEV(lg, info) << "error vel y = " << der_error_h_tot(1);
    //                BOOST_LOG_SEV(lg, info) << "error vel z = " << der_error_h_tot(2);
    //                BOOST_LOG_SEV(lg, info) << "error ang vel x = " << der_error_h_tot(3);
    //                BOOST_LOG_SEV(lg, info) << "error ang vel y = " << der_error_h_tot(4);
    //                BOOST_LOG_SEV(lg, info) << "error ang vel z = " << der_error_h_tot(5);


                    // finger control
                    MatrixXd Kp_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
                    MatrixXd Kd_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
                    Kp_fing(0,0) = fing_p_coeff; Kp_fing(1,1) = fing_p_coeff; Kp_fing(2,2) = fing_p_coeff; Kp_fing(3,3) = fing_p_coeff;
                    Kd_fing(0,0) = fing_d_coeff; Kd_fing(1,1) = fing_d_coeff; Kd_fing(2,2) = fing_d_coeff; Kd_fing(3,3) = fing_d_coeff;
                    VectorXd r_fing_acc_read_vec = h_fing_ref_acc + Kd_fing*der_error_h_fing_tot + Kp_fing*error_h_fing_tot;
                    VectorXd r_hand_velocities_vec = r_fing_acc_read_vec * time_step;
                    if(mov_type==0 && stage_descr.compare("retreat")==0){
                        // retreat stage of a pick movement
                        r_hand_velocities_vec = VectorXd::Zero(JOINTS_HAND);
                    }
                    VectorXd::Map(&r_hand_velocities[0], r_hand_velocities_vec.size()) = r_hand_velocities_vec;

                    }else{
                        // trapezoidal velocity profile
                        VectorXd vel_trap(JOINTS_ARM);
                        vel_trap(0) = 2*(des_hand_pos(0)-hand_pos_init_vec(0))/t_f_trap;
                        vel_trap(1) = 2*(des_hand_pos(1)-hand_pos_init_vec(1))/t_f_trap;
                        vel_trap(2) = 2*(des_hand_pos(2)-hand_pos_init_vec(2))/t_f_trap;
                        vel_trap(3) = 2*(des_hand_or_q_e(0)-hand_or_q_e_init_vec(0))/t_f_trap;
                        vel_trap(4) = 2*(des_hand_or_q_e(1)-hand_or_q_e_init_vec(1))/t_f_trap;
                        vel_trap(5) = 2*(des_hand_or_q_e(2)-hand_or_q_e_init_vec(2))/t_f_trap;
                        vel_trap(6) = 2*(des_hand_or_q_w-hand_or_q_w_init_vec)/t_f_trap;
                        VectorXd acc_trap(JOINTS_ARM);
                        acc_trap(0) = pow(vel_trap(0),2)/(hand_pos_init_vec(0)-des_hand_pos(0)+vel_trap(0)*t_f_trap);
                        acc_trap(1) = pow(vel_trap(1),2)/(hand_pos_init_vec(1)-des_hand_pos(1)+vel_trap(1)*t_f_trap);
                        acc_trap(2) = pow(vel_trap(2),2)/(hand_pos_init_vec(2)-des_hand_pos(2)+vel_trap(2)*t_f_trap);
                        acc_trap(3) = pow(vel_trap(3),2)/(hand_or_q_e_init_vec(0)-des_hand_or_q_e(0)+vel_trap(3)*t_f_trap);
                        acc_trap(4) = pow(vel_trap(4),2)/(hand_or_q_e_init_vec(1)-des_hand_or_q_e(1)+vel_trap(4)*t_f_trap);
                        acc_trap(5) = pow(vel_trap(5),2)/(hand_or_q_e_init_vec(2)-des_hand_or_q_e(2)+vel_trap(5)*t_f_trap);
                        acc_trap(6) = pow(vel_trap(6),2)/(hand_or_q_w_init_vec-des_hand_or_q_w+vel_trap(6)*t_f_trap);

                        double t_c_trap = t_f_trap/2;
                        double curr_t;
                        if(sim_robot){
                            curr_t = this->curr_time;
                        }else{
                            double d_curr_t = (boost::chrono::duration_cast<msec>(this->curr_time_ctrl-this->start_time_point)).count();
                            curr_t = d_curr_t/1000;
                        }
                        if(curr_t <= t_c_trap)
                        {
                            // hand pose
                            trap_hand_pose(0) = hand_pos_init_vec(0) + 0.5*acc_trap(0)*pow(curr_t,2);
                            trap_hand_pose(1) = hand_pos_init_vec(1) + 0.5*acc_trap(1)*pow(curr_t,2);
                            trap_hand_pose(2) = hand_pos_init_vec(2) + 0.5*acc_trap(2)*pow(curr_t,2);
                            trap_hand_pose(3) = hand_or_q_e_init_vec(0) + 0.5*acc_trap(3)*pow(curr_t,2);
                            trap_hand_pose(4) = hand_or_q_e_init_vec(1) + 0.5*acc_trap(4)*pow(curr_t,2);
                            trap_hand_pose(5) = hand_or_q_e_init_vec(2) + 0.5*acc_trap(5)*pow(curr_t,2);
                            trap_hand_pose(6) = hand_or_q_w_init_vec + 0.5*acc_trap(6)*pow(curr_t,2);

                            // hand velocity
                            trap_hand_vel(0) = acc_trap(0)*curr_t;
                            trap_hand_vel(1) = acc_trap(1)*curr_t;
                            trap_hand_vel(2) = acc_trap(2)*curr_t;
                            trap_hand_vel(3) = acc_trap(3)*curr_t;
                            trap_hand_vel(4) = acc_trap(4)*curr_t;
                            trap_hand_vel(5) = acc_trap(5)*curr_t;
                            trap_hand_vel(6) = acc_trap(6)*curr_t;

                            // hand acceleration
                            trap_hand_acc(0) = acc_trap(0);
                            trap_hand_acc(1) = acc_trap(1);
                            trap_hand_acc(2) = acc_trap(2);
                            trap_hand_acc(3) = acc_trap(3);
                            trap_hand_acc(4) = acc_trap(4);
                            trap_hand_acc(5) = acc_trap(5);
                            trap_hand_acc(6) = acc_trap(6);

                        }else if((curr_t > t_c_trap) && (curr_t <= t_f_trap)){

                            //hand pose
                            trap_hand_pose(0) = des_hand_pos(0) - 0.5*acc_trap(0)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(1) = des_hand_pos(1) - 0.5*acc_trap(1)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(2) = des_hand_pos(2) - 0.5*acc_trap(2)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(3) = des_hand_or_q_e(0) - 0.5*acc_trap(3)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(4) = des_hand_or_q_e(1) - 0.5*acc_trap(4)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(5) = des_hand_or_q_e(2) - 0.5*acc_trap(5)*pow((t_f_trap-curr_t),2);
                            trap_hand_pose(6) = des_hand_or_q_w - 0.5*acc_trap(6)*pow((t_f_trap-curr_t),2);

                            // hand velocity
                            trap_hand_vel(0) = -acc_trap(0)*(t_f_trap-curr_t);
                            trap_hand_vel(1) = -acc_trap(1)*(t_f_trap-curr_t);
                            trap_hand_vel(2) = -acc_trap(2)*(t_f_trap-curr_t);
                            trap_hand_vel(3) = -acc_trap(3)*(t_f_trap-curr_t);
                            trap_hand_vel(4) = -acc_trap(4)*(t_f_trap-curr_t);
                            trap_hand_vel(5) = -acc_trap(5)*(t_f_trap-curr_t);
                            trap_hand_vel(6) = -acc_trap(6)*(t_f_trap-curr_t);

                            // hand acceleration
                            trap_hand_acc(0) = -acc_trap(0);
                            trap_hand_acc(1) = -acc_trap(1);
                            trap_hand_acc(2) = -acc_trap(2);
                            trap_hand_acc(3) = -acc_trap(3);
                            trap_hand_acc(4) = -acc_trap(4);
                            trap_hand_acc(5) = -acc_trap(5);
                            trap_hand_acc(6) = -acc_trap(6);

                        }else if(curr_t > t_f_trap){
                            //hand pose
                            trap_hand_pose(0) = des_hand_pos(0);
                            trap_hand_pose(1) = des_hand_pos(1);
                            trap_hand_pose(2) = des_hand_pos(2);
                            trap_hand_pose(3) = des_hand_or_q_e(0);
                            trap_hand_pose(4) = des_hand_or_q_e(1);
                            trap_hand_pose(5) = des_hand_or_q_e(2);
                            trap_hand_pose(6) = des_hand_or_q_w;

                            // hand velocity
                            trap_hand_vel(0) = 0.0;
                            trap_hand_vel(1) = 0.0;
                            trap_hand_vel(2) = 0.0;
                            trap_hand_vel(3) = 0.0;
                            trap_hand_vel(4) = 0.0;
                            trap_hand_vel(5) = 0.0;
                            trap_hand_vel(6) = 0.0;

                            // hand acceleration
                            trap_hand_acc(0) = 0.0;
                            trap_hand_acc(1) = 0.0;
                            trap_hand_acc(2) = 0.0;
                            trap_hand_acc(3) = 0.0;
                            trap_hand_acc(4) = 0.0;
                            trap_hand_acc(5) = 0.0;
                            trap_hand_acc(6) = 0.0;
                        }


                        // error in position
                        Vector3d error_trap_pos;
                        error_trap_pos(0) = trap_hand_pose(0) - r_hand_pos_vec(0);
                        error_trap_pos(1) = trap_hand_pose(1) - r_hand_pos_vec(1);
                        error_trap_pos(2) = trap_hand_pose(2) - r_hand_pos_vec(2);
                        // error in orientation
                        Quaterniond trap_hand_or_q;
                        trap_hand_or_q.x() = trap_hand_pose(3); trap_hand_or_q.y() = trap_hand_pose(4); trap_hand_or_q.z() = trap_hand_pose(5); trap_hand_or_q.w() = trap_hand_pose(6);
                        Vector3d trap_hand_or_q_e; trap_hand_or_q_e << trap_hand_pose(3),trap_hand_pose(4),trap_hand_pose(5); double trap_hand_or_q_w = trap_hand_pose(6);
                        Vector3d error_trap_or = r_hand_q_w*trap_hand_or_q_e - trap_hand_or_q_w*r_hand_or_q_e - trap_hand_or_q_e.cross(r_hand_or_q_e);
                        // total error in position + orientation
                        error_trap_tot << error_trap_pos(0),error_trap_pos(1),error_trap_pos(2), error_trap_or(0),error_trap_or(1),error_trap_or(2);

                        // error in linear velocity
                        Vector3d error_trap_lin_vel;
                        error_trap_lin_vel(0) = trap_hand_vel(0) - r_hand_lin_vel_vec(0);
                        error_trap_lin_vel(1) = trap_hand_vel(1) - r_hand_lin_vel_vec(1);
                        error_trap_lin_vel(2) = trap_hand_vel(2) - r_hand_lin_vel_vec(2);
                        // error in orientation velocity (quaternion)
                        Vector3d trap_hand_ang_vel_q_e; trap_hand_ang_vel_q_e << trap_hand_vel(3),trap_hand_vel(4),trap_hand_vel(5); double trap_hand_ang_vel_q_w = trap_hand_vel(6);
                        Vector3d error_trap_der_or = r_hand_ang_vel_q_w*trap_hand_or_q_e + r_hand_q_w*trap_hand_ang_vel_q_e - trap_hand_ang_vel_q_w*r_hand_or_q_e
                                                - trap_hand_or_q_w*r_hand_ang_vel_q_e - trap_hand_ang_vel_q_e.cross(r_hand_or_q_e) - trap_hand_or_q_e.cross(r_hand_ang_vel_q_e);
                        // total error in velocity
                        der_error_trap_tot << error_trap_lin_vel(0),error_trap_lin_vel(1),error_trap_lin_vel(2),error_trap_der_or(0),error_trap_der_or(1),error_trap_der_or(2);

                        // triangular reference velocity
                        trap_hand_ref_vel(0) = trap_hand_vel(0);
                        trap_hand_ref_vel(1) = trap_hand_vel(1);
                        trap_hand_ref_vel(2) = trap_hand_vel(2);
                        Vector3d trap_hand_vel_q_e; trap_hand_vel_q_e << trap_hand_vel(3), trap_hand_vel(4), trap_hand_vel(5); double trap_hand_vel_q_w = trap_hand_vel(6);
                        Vector3d des_omega = 2*trap_hand_or_q_e.cross(trap_hand_vel_q_e)+2*trap_hand_or_q_w*trap_hand_vel_q_e-2*trap_hand_vel_q_w*trap_hand_or_q_e;
                        trap_hand_ref_vel(3) = des_omega(0);
                        trap_hand_ref_vel(4) = des_omega(1);
                        trap_hand_ref_vel(5) = des_omega(2);

                        // triangular reference acceleration
                        trap_hand_ref_acc(0) = trap_hand_acc(0);
                        trap_hand_ref_acc(1) = trap_hand_acc(1);
                        trap_hand_ref_acc(2) = trap_hand_acc(2);
                        Vector3d trap_hand_acc_q_e; trap_hand_acc_q_e << trap_hand_acc(3), trap_hand_acc(4), trap_hand_acc(5); double trap_hand_acc_q_w = trap_hand_acc(6);
                        Vector3d des_alpha = 2*trap_hand_or_q_e.cross(trap_hand_acc_q_e) + 2*trap_hand_or_q_w*trap_hand_acc_q_e - 2*trap_hand_acc_q_w*trap_hand_or_q_e;
                        trap_hand_ref_acc(3) = des_alpha(0);
                        trap_hand_ref_acc(4) = des_alpha(1);
                        trap_hand_ref_acc(5) = des_alpha(2);

                    }
                }

                // ------------------- RE-PLANNING STRATEGY ---------------------------------------- //
                if((stage_descr.compare("plan")==0) && (g_map>g_map_th_min_replan && g_map<g_map_th_max_replan)
                        && (abs(der_alpha_predicted)>=swivel_angle_th) && !this->replanning_done && sim_robot)
                {
                    this->exec_command_ctrl = false; // stop the motion
                    this->replanning_done = true;
                    execReplanning_thrd = boost::thread(boost::bind(&MainWindow::execReplanning, this));
                }else{
//                    this->exec_command_ctrl = true;
//                    this->replanning_done = false;
                }
                // ----------------------------------------------------------- //

                // position Koeff
                Vector3d error_abs_pos; Vector3d error_abs_or;
                error_abs_pos << abs(error_pos(0)),abs(error_pos(1)),abs(error_pos(2));
                error_abs_or << abs(error_or(0)),abs(error_or(1)),abs(error_or(2));
                double e_n_pos = error_pos.norm(); double e_n_or = error_or.norm();
                MatrixXd Koeff_p = MatrixXd::Identity(6,6);
                //if((this->ui.checkBox_des_right_hand_pos_x->isChecked()) && (error_abs_pos(0) > error_pos_th)){
                if((this->ui.checkBox_des_right_hand_pos_x->isChecked())){
                    if(hl_en){
                        Koeff_p(0,0) = hl_p_pos_coeff;
                    }else{
                        Koeff_p(0,0) = coeff_p_pos;
                    }
                }else{ Koeff_p(0,0) = 0.0;}
                //if((this->ui.checkBox_des_right_hand_pos_y->isChecked()) && (error_abs_pos(1) > error_pos_th)){
                if((this->ui.checkBox_des_right_hand_pos_y->isChecked())){
                    if(hl_en){
                        Koeff_p(1,1) = hl_p_pos_coeff;
                    }else{
                        Koeff_p(1,1) = coeff_p_pos;
                    }
                }else{ Koeff_p(1,1) = 0.0;}
                //if((this->ui.checkBox_des_right_hand_pos_z->isChecked()) && (error_abs_pos(2) > error_pos_th)){
                if((this->ui.checkBox_des_right_hand_pos_z->isChecked())){
                    if(hl_en){
                        Koeff_p(2,2) = hl_p_pos_coeff;
                    }else{
                        Koeff_p(2,2) = coeff_p_pos;
                    }
                }else{ Koeff_p(2,2) = 0.0;}
                //if((this->ui.checkBox_des_right_hand_q_x->isChecked()) && (error_abs_or(0) > error_or_th)){
                if((this->ui.checkBox_des_right_hand_q_x->isChecked())){
                    if(hl_en){
                        Koeff_p(3,3) = hl_p_or_coeff;
                    }else{
                        Koeff_p(3,3) = coeff_p_or;
                    }
                }else{ Koeff_p(3,3) = 0.0;}
                //if((this->ui.checkBox_des_right_hand_q_y->isChecked()) && (error_abs_or(1) > error_or_th)){
                if((this->ui.checkBox_des_right_hand_q_y->isChecked())){
                    if(hl_en){
                        Koeff_p(4,4) = hl_p_or_coeff;
                    }else{
                        Koeff_p(4,4) = coeff_p_or;
                    }
                }else{ Koeff_p(4,4) = 0.0;}
                //if((this->ui.checkBox_des_right_hand_q_z->isChecked()) && (error_abs_or(2) > error_or_th)){
                if((this->ui.checkBox_des_right_hand_q_z->isChecked())){
                    if(hl_en){
                        Koeff_p(5,5) = hl_p_or_coeff;
                    }else{
                        Koeff_p(5,5) = coeff_p_or;
                    }
                }else{ Koeff_p(5,5) = 0.0;}

//                Vector3d error_abs_lin_vel; Vector3d error_abs_ang_vel;
//                error_abs_lin_vel << abs(error_lin_vel(0)),abs(error_lin_vel(1)),abs(error_lin_vel(2));
//                error_abs_ang_vel << abs(error_ang_vel(0)),abs(error_ang_vel(1)),abs(error_ang_vel(2));
                //double e_n_lin_vel = error_lin_vel.norm(); double e_n_ang_vel = error_ang_vel.norm();
                MatrixXd Koeff_d = MatrixXd::Identity(6,6);
//                if((this->ui.checkBox_des_right_hand_pos_x->isChecked()) && (error_abs_lin_vel(0) > error_lin_vel_th)){
                if((this->ui.checkBox_des_right_hand_pos_x->isChecked())){
                    if(hl_en){
                        Koeff_d(0,0) = hl_d_pos_coeff;
                    }else{
                        Koeff_d(0,0) = coeff_d_pos;
                    }
                }else{ Koeff_d(0,0) = 0.0;}
//                if((this->ui.checkBox_des_right_hand_pos_y->isChecked()) && (error_abs_lin_vel(1) > error_lin_vel_th)){
                if((this->ui.checkBox_des_right_hand_pos_y->isChecked())){
                    if(hl_en){
                        Koeff_d(1,1) = hl_d_pos_coeff;
                    }else{
                        Koeff_d(1,1) = coeff_d_pos;
                    }
                }else{ Koeff_d(1,1) = 0.0;}
//                if((this->ui.checkBox_des_right_hand_pos_z->isChecked()) && (error_abs_lin_vel(2) > error_lin_vel_th)){
                if((this->ui.checkBox_des_right_hand_pos_z->isChecked())){
                    if(hl_en){
                        Koeff_d(2,2) = hl_d_pos_coeff;
                    }else{
                        Koeff_d(2,2) = coeff_d_pos;
                    }
                }else{ Koeff_d(2,2) = 0.0;}
//                if((this->ui.checkBox_des_right_hand_q_x->isChecked()) && (error_abs_ang_vel(0) > error_ang_vel_th)){
                if((this->ui.checkBox_des_right_hand_q_x->isChecked())){
                    if(hl_en){
                        Koeff_d(3,3) = hl_d_or_coeff;
                    }else{
                        Koeff_d(3,3) = coeff_d_or;
                    }
                }else{ Koeff_d(3,3) = 0.0;}
//                if((this->ui.checkBox_des_right_hand_q_y->isChecked()) && (error_abs_ang_vel(1) > error_ang_vel_th)){
                if((this->ui.checkBox_des_right_hand_q_y->isChecked())){
                    if(hl_en){
                        Koeff_d(4,4) = hl_d_or_coeff;
                    }else{
                        Koeff_d(4,4) = coeff_d_or;
                    }
                }else{ Koeff_d(4,4) = 0.0;}
//                if((this->ui.checkBox_des_right_hand_q_z->isChecked()) && (error_abs_ang_vel(2) > error_ang_vel_th)){
                if((this->ui.checkBox_des_right_hand_q_z->isChecked())){
                    if(hl_en){
                        Koeff_d(5,5) = hl_d_or_coeff;
                    }else{
                        Koeff_d(5,5) = coeff_d_or;
                    }
                }else{ Koeff_d(5,5) = 0.0;}


                // closed-loop control
                //VectorXd hand_vel_xd_vec(6);
                VectorXd hand_acc_xd_vec(6);
                if(hl_en){
                    hand_acc_xd_vec = h_hand_ref_acc + Koeff_d*der_error_h_tot + Koeff_p*error_h_tot - this->hand_j_acc;
                }else{
                    hand_acc_xd_vec = trap_hand_ref_acc + Koeff_d*der_error_trap_tot + Koeff_p*error_trap_tot - this->hand_j_acc;
                }
                vector<double> hand_acc_vec; hand_acc_vec.resize(hand_acc_xd_vec.size());
                VectorXd::Map(&hand_acc_vec[0], hand_acc_xd_vec.size()) = hand_acc_xd_vec;

                // check proximity
                if(this->ui.checkBox_use_plan_hand_pos->isChecked())
                {
                    this->qnode.checkProximityObject(this->curr_mov,stage_descr);
                }

                // change desired hand pose
                if(this->ui.checkBox_use_plan_hand_pos->isChecked()){
                    bool condition = (e_n_pos < 1.73*error_pos_th) && (e_n_or < 1.73*error_or_th);
                    if(stages==3 && hl_en && stage_descr.compare("plan")==0)
                    {
                        condition = (g_map >= g_map_th_pa);
                    }else if((stage_descr.compare("approach")==0) && follow_tar && (hand_vel_vec_x.norm()<=5.0)){
                        // the target was being followed and it has stopped
                        follow_tar = false; this->ui.checkBox_follow_target->setChecked(false); // the target has stopped
                        // reset the normalized time, but do not go to the next stage (the approach has to be performed)
                        if(sim_robot){
                            this->qnode.openBarrettHand_to_pos(1,jointsInitPosition_hand_vec); // open the hand to the initial position of the approach stage
                            this->t_past=this->curr_time;
                        }else{
                            this->t_past_ctrl = boost::chrono::duration_cast<msec>(this->curr_time_ctrl - this->start_time_point);
                        }
                    }else if(stages==3 && hl_en && stage_descr.compare("approach")==0){
                        condition = (g_map >= g_map_th_rp);
                    }else if(stages==3 && hl_en && stage_descr.compare("retreat")==0){
                        if(mov_type==0 && !sim_robot){//pick
                            // close the Barrett Hand of ARoS
                            this->qnode.open_close_BH(true);
                        }
                    }
                    if(condition){
                        //this->qnode.log(QNode::Info,string("Simulation Time: ")+boost::lexical_cast<std::string>(this->qnode.getSimTime()));
                        // reset the normalized time, but do not go to the next stage (the approach has to be performed)
                        if(sim_robot){
                            this->t_past=this->curr_time;
                        }else{
                            this->t_past_ctrl = boost::chrono::duration_cast<msec>(this->curr_time_ctrl - this->start_time_point);
                        }
                        if(stages==3 && this->i_ctrl<2){
                            if(!hl_en){
                                if(stage_descr.compare("plan")==0){
                                    this->qnode.openBarrettHand_to_pos(1,jointsFinalPosition_hand_vec);
                                }
                                if(stage_descr.compare("approach")==0){
                                    this->qnode.closeBarrettHand_to_pos(1,jointsFinalPosition_hand_vec);
                                }                          
                            }
                            this->i_ctrl++; // go to the next stage
                        }else if(stages==2 && this->i_ctrl<1){
                            // TO DO
                            this->i_ctrl++;
                        }
                    } // if condition
                }


                // inverse algorithm
                this->curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm2(1,r_arm_posture_mes,hand_acc_vec,r_arm_velocities,r_arm_null_velocities,time_step,jlim_en,sing_en,obsts_en,
                                                                                vel_max,sing_coeff,sing_damping,obst_coeff,obst_damping,obst_coeff_torso,obst_damping_torso,
                                                                                 jlim_th,jlim_rate,jlim_coeff,jlim_damping,obsts_n);


                // execute the control                
                if(this->exec_command_ctrl){
                    if(sim_robot){
                        this->qnode.execKinControl(1,r_arm_posture_mes,r_arm_velocities,r_hand_posture_mes,r_hand_velocities,joints_arm_vel_ctrl,true);
                    }else{
                       this->qnode.execKinRealControl(1,r_arm_velocities,r_hand_velocities,true);
                    }
                }else{
                    // stop the motion
                    vector<double> r_arm_velocities_0(JOINTS_ARM,0.0);vector<double> r_hand_velocities_0(JOINTS_HAND,0.0);
                    if(sim_robot){
                        this->qnode.execKinControl(1,r_arm_posture_mes,r_arm_velocities_0,r_hand_posture_mes,r_hand_velocities_0,joints_arm_vel_ctrl,true);
                    }else{
                       this->qnode.execKinRealControl(1,r_arm_velocities_0,r_hand_velocities_0,true);
                    }
                }

                // ------------- Recording ------------------------------- //

                // record the positions of the joints
                this->jointsPosition_ctrl.conservativeResize(this->jointsPosition_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
                for(size_t jj=0; jj < r_arm_posture_mes.size(); ++jj)
                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,jj) = r_arm_posture_mes.at(jj);
                for(size_t jj=0; jj < r_hand_posture_mes.size(); ++jj)
                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,r_arm_posture_mes.size()+jj) = r_hand_posture_mes.at(jj);

                // record the velocities of the joints
                this->jointsVelocity_ctrl.conservativeResize(this->jointsVelocity_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
                for(size_t jj=0; jj < r_arm_velocities_read.size(); ++jj)
                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,jj) = r_arm_velocities_read.at(jj);
                for(size_t jj=0; jj < r_hand_velocities_read.size(); ++jj)
                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,r_arm_velocities_read.size()+jj) = r_hand_velocities_read.at(jj);

                this->jointsVelocity_null_ctrl.conservativeResize(this->jointsVelocity_null_ctrl.rows()+1,JOINTS_ARM);
                for(size_t jj=0; jj < r_arm_null_velocities.size(); ++jj)
                    this->jointsVelocity_null_ctrl(this->jointsVelocity_null_ctrl.rows()-1,jj) = r_arm_null_velocities(jj);


                // record the acceleration of the joints
                this->jointsAcceleration_ctrl.conservativeResize(this->jointsAcceleration_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
                for(size_t jj=0; jj < r_arm_accelerations_read.size(); ++jj)
                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,jj) = r_arm_accelerations_read.at(jj);
                for(size_t jj=0; jj < r_hand_accelerations_read.size(); ++jj)
                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,r_arm_accelerations_read.size()+jj) = r_hand_accelerations_read.at(jj);

                // desired hand pose
                vector<double> std_h_hand_des_pose;  std_h_hand_des_pose.resize(h_hand_pose.size());
                VectorXd::Map(&std_h_hand_des_pose[0], h_hand_pose.size()) = h_hand_pose;
                this->handPosition_des_ctrl.push_back(std_h_hand_des_pose);

                // desired fingers positions
                vector<double> std_h_fing_pos;  std_h_fing_pos.resize(h_fing_pos.size());
                VectorXd::Map(&std_h_fing_pos[0], h_fing_pos.size()) = h_fing_pos;
                this->fingPosition_des_ctrl.push_back(std_h_fing_pos);

                // fingers positions
                this->fingPosition_ctrl.push_back(r_hand_posture_mes);

                // operational space positions
                this->handPosition_ctrl.push_back(r_hand_lin_pos);
                this->handOrientation_ctrl.push_back(r_hand_ang_pos);
                this->handOrientation_q_ctrl.push_back(r_hand_q);
                this->wristPosition_ctrl.push_back(r_wrist_lin_pos);
                this->wristOrientation_ctrl.push_back(r_wrist_ang_pos);
                this->elbowPosition_ctrl.push_back(r_elbow_lin_pos);
                this->elbowOrientation_ctrl.push_back(r_elbow_ang_pos);
                this->shoulderPosition_ctrl.push_back(r_shoulder_lin_pos);
                this->shoulderOrientation_ctrl.push_back(r_shoulder_ang_pos);

                // operational space velocities
                this->handLinearVelocity_ctrl.push_back(r_hand_lin_vel);
                this->handAngularVelocity_ctrl.push_back(r_hand_ang_vel);
                this->wristLinearVelocity_ctrl.push_back(r_wrist_lin_vel);
                this->wristAngularVelocity_ctrl.push_back(r_wrist_ang_vel);
                this->elbowLinearVelocity_ctrl.push_back(r_elbow_lin_vel);
                this->elbowAngularVelocity_ctrl.push_back(r_elbow_ang_vel);
                this->shoulderLinearVelocity_ctrl.push_back(r_shoulder_lin_vel);
                this->shoulderAngularVelocity_ctrl.push_back(r_shoulder_ang_vel);
                this->handVelocityNorm_ctrl.push_back(sqrt(pow(r_hand_lin_vel.at(0),2)+pow(r_hand_lin_vel.at(1),2)+pow(r_hand_lin_vel.at(2),2)));

                // operational space accelerations
                this->handLinearAcceleration_ctrl.push_back(r_hand_lin_acc);
                this->handAngularAcceleration_ctrl.push_back(r_hand_ang_acc);
                this->wristLinearAcceleration_ctrl.push_back(r_wrist_lin_acc);
                this->wristAngularAcceleration_ctrl.push_back(r_wrist_ang_acc);
                this->elbowLinearAcceleration_ctrl.push_back(r_elbow_lin_acc);
                this->elbowAngularAcceleration_ctrl.push_back(r_elbow_ang_acc);
                this->shoulderLinearAcceleration_ctrl.push_back(r_shoulder_lin_acc);
                this->shoulderAngularAcceleration_ctrl.push_back(r_shoulder_ang_acc);
                this->handAccelerationNorm_ctrl.push_back(sqrt(pow(r_hand_lin_acc.at(0),2)+pow(r_hand_lin_acc.at(1),2)+pow(r_hand_lin_acc.at(2),2)));

                // errors
                if(hl_en){
                    // hand
                    this->error_pos_tot_norm.push_back(error_h_tot.block<3,1>(0,0).norm());
                    this->error_or_tot_norm.push_back(error_h_tot.block<3,1>(3,0).norm());
                    this->error_pos_or_tot_norm.push_back(error_h_tot.norm());
                    this->error_lin_vel_tot_norm.push_back(der_error_h_tot.block<3,1>(0,0).norm());
                    this->error_ang_vel_tot_norm.push_back(der_error_h_tot.block<3,1>(3,0).norm());
                    this->error_vel_tot_norm.push_back(der_error_h_tot.norm());
                    VectorXd error_acc_tot = h_hand_ref_acc - r_hand_acc_read_vec;
                    this->error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
                    this->error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
                    this->error_acc_tot_norm.push_back(error_acc_tot.norm());
                    // fingers
                    vector<double> error_h_fing_tot_vec;  error_h_fing_tot_vec.resize(error_h_fing_tot.size());
                    VectorXd::Map(&error_h_fing_tot_vec[0], error_h_fing_tot.size()) = error_h_fing_tot;
                    this->error_fing_pos.push_back(error_h_fing_tot_vec);
                    vector<double> der_error_h_fing_tot_vec;  der_error_h_fing_tot_vec.resize(der_error_h_fing_tot.size());
                    VectorXd::Map(&der_error_h_fing_tot_vec[0], der_error_h_fing_tot.size()) = der_error_h_fing_tot;
                    this->error_fing_vel.push_back(der_error_h_fing_tot_vec);
                    VectorXd error_fing_acc_tot = h_fing_ref_acc - r_hand_accelerations_read_vec;
                    vector<double> error_fing_acc_tot_vec;  error_fing_acc_tot_vec.resize(error_fing_acc_tot.size());
                    VectorXd::Map(&error_fing_acc_tot_vec[0], error_fing_acc_tot.size()) = error_fing_acc_tot;
                    this->error_fing_acc.push_back(error_fing_acc_tot_vec);
                }else{
                    this->error_pos_tot_norm.push_back(error_trap_tot.block<3,1>(0,0).norm());
                    this->error_or_tot_norm.push_back(error_trap_tot.block<3,1>(3,0).norm());
                    this->error_pos_or_tot_norm.push_back(error_trap_tot.norm());
                    this->error_lin_vel_tot_norm.push_back(der_error_trap_tot.block<3,1>(0,0).norm());
                    this->error_ang_vel_tot_norm.push_back(der_error_trap_tot.block<3,1>(3,0).norm());
                    this->error_vel_tot_norm.push_back(der_error_trap_tot.norm());
                    VectorXd error_acc_tot = trap_hand_ref_acc - r_hand_acc_read_vec;
                    this->error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
                    this->error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
                    this->error_acc_tot_norm.push_back(error_acc_tot.norm());
                }

                // predicted swivel angle
                this->pred_swivel_angle_ctrl.push_back(alpha_predicted);
                this->pred_der_swivel_angle_ctrl.push_back(der_alpha_predicted);

                // time
                if(sim_robot){
                    this->t_j_past = this->qnode.getSimTime();
                    this->sim_time.push_back(this->t_j_past-this->t_der_past);
                }else{
                    this->t_j_past_ctrl = Clock::now();
                    Clock::time_point sim_tp = this->t_j_past_ctrl - this->t_der_past_ctrl;
                    double d_sim_time = (boost::chrono::duration_cast<msec>(sim_tp-this->start_time_point)).count();
                    //BOOST_LOG_SEV(lg, info) << "d_sim_time = " << d_sim_time/1000;
                    this->sim_time.push_back(d_sim_time/1000);
                }

            } // check timestep
        } // if pos control
    } // while exec_control
}

void MainWindow::execVelControl()
{
    while(exec_control)
    {

        if(vel_control)
        {
            boost::unique_lock<boost::mutex> lck(hh_control_mtx);

            double obsts_x_var = 100; // mm
            double obsts_y_var = 100; // mm
            double obsts_z_var = 100; // mm
            double obsts_q_x_var = 0.1;
            double obsts_q_y_var = 0.1;
            double obsts_q_z_var = 0.1;
            double obsts_q_w_var = 0.1;

            double des_hand_vel_x,des_hand_vel_y,des_hand_vel_z,des_hand_vel_wx,des_hand_vel_wy,des_hand_vel_wz;
            if (this->ui.checkBox_draw_ellipse->isChecked()){
                des_hand_vel_x = 40*cos(0.2*this->qnode.getSimTime()-M_PI/2);
                des_hand_vel_y = -60*sin(0.2*this->qnode.getSimTime()-M_PI/2);
                des_hand_vel_z = 0.0;
                des_hand_vel_wx = 0.0;
                des_hand_vel_wy = 0.0;
                des_hand_vel_wz = 0.0;
            }else{
                des_hand_vel_x = this->ui.lineEdit_des_right_hand_vel_x->text().toDouble();
                des_hand_vel_y = this->ui.lineEdit_des_right_hand_vel_y->text().toDouble();
                des_hand_vel_z = this->ui.lineEdit_des_right_hand_vel_z->text().toDouble();
                des_hand_vel_wx = this->ui.lineEdit_des_right_hand_vel_wx->text().toDouble();
                des_hand_vel_wy = this->ui.lineEdit_des_right_hand_vel_wy->text().toDouble();
                des_hand_vel_wz = this->ui.lineEdit_des_right_hand_vel_wz->text().toDouble();
            }

            if(!this->ui.checkBox_des_right_hand_vel_x->isChecked()){
                des_hand_vel_x = 0.0;
            }
            if(!this->ui.checkBox_des_right_hand_vel_y->isChecked()){
                des_hand_vel_y = 0.0;
            }
            if(!this->ui.checkBox_des_right_hand_vel_z->isChecked()){
                des_hand_vel_z = 0.0;
            }
            if(!this->ui.checkBox_des_right_hand_vel_wx->isChecked()){
                des_hand_vel_wx = 0.0;
            }
            if(!this->ui.checkBox_des_right_hand_vel_wy->isChecked()){
                des_hand_vel_wy = 0.0;
            }
            if(!this->ui.checkBox_des_right_hand_vel_wz->isChecked()){
                des_hand_vel_wz = 0.0;
            }
            vector<double> hand_vel_vec = {des_hand_vel_x,des_hand_vel_y,des_hand_vel_z, des_hand_vel_wx,des_hand_vel_wy,des_hand_vel_wz};
            VectorXd hand_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hand_vel_vec.data(), hand_vel_vec.size());

            double vel_max = this->ui.lineEdit_vel_max->text().toDouble()*M_PI/180;

            bool jlim_en = this->ui.checkBox_joints_limits_av->isChecked();
            double jlim_th = 0; double jlim_rate = 1;
            double jlim_coeff = 1; double jlim_damping = 0.001;
            if(jlim_en){
                jlim_th = this->ui.lineEdit_jlim_th->text().toDouble();
                jlim_rate = this->ui.lineEdit_jlim_rate->text().toDouble();
                jlim_coeff = this->ui.lineEdit_jlim_coeff->text().toDouble();
                jlim_damping = this->ui.lineEdit_jlim_damping->text().toDouble();
            }
            bool sing_en = this->ui.checkBox_sing_av->isChecked();
            double sing_coeff = 1; double sing_damping = 0.001;
            if(sing_en){
                sing_damping = this->ui.lineEdit_sing_damping->text().toDouble();
                sing_coeff = this->ui.lineEdit_sing_coeff->text().toDouble();
            }

            double obst_coeff = 1; double obst_damping = 0.001;
            double obst_coeff_torso = 1; double obst_damping_torso = 0.001;
            bool obsts_en = this->ui.checkBox_obsts_av->isChecked();
            vector<objectPtr> obsts_n; // obstacles in the scenario
            if(obsts_en){
                this->curr_scene->getObjects(obsts_n);
                obst_coeff = this->ui.lineEdit_obsts_coeff->text().toDouble();
                obst_damping = this->ui.lineEdit_obsts_damping->text().toDouble();
                obst_coeff_torso = this->ui.lineEdit_obsts_coeff_torso->text().toDouble();
                obst_damping_torso = this->ui.lineEdit_obsts_damping_torso->text().toDouble();
                if(this->ui.checkBox_obsts_noise->isChecked()){
                    vector<objectPtr> obsts; this->curr_scene->getObjects(obsts);
                    obsts_n.resize(obsts.size());
                    objectPtr obs_new;
                    for(size_t i=0; i<obsts.size(); ++i){
                        objectPtr obs = obsts.at(i);
                        string obs_name = obs->getName();
                        obs_new.reset(new Object(obs_name));
                        motion_manager::pos obs_pos;
                        Quaterniond obs_or_q;
                        obs_pos.Xpos = obs->getPos().Xpos - (obsts_x_var/2) + obsts_x_var*(rand() / double(RAND_MAX));
                        obs_pos.Ypos = obs->getPos().Ypos - (obsts_y_var/2) + obsts_y_var*(rand() / double(RAND_MAX));
                        obs_pos.Zpos = obs->getPos().Zpos - (obsts_z_var/2) + obsts_z_var*(rand() / double(RAND_MAX));
                        obs_or_q.x() = obs->getQuaternion().x() - (obsts_q_x_var/2) + obsts_q_x_var*(rand() / double(RAND_MAX));
                        obs_or_q.y() = obs->getQuaternion().y() - (obsts_q_y_var/2) + obsts_q_y_var*(rand() / double(RAND_MAX));
                        obs_or_q.z() = obs->getQuaternion().z() - (obsts_q_z_var/2) + obsts_q_z_var*(rand() / double(RAND_MAX));
                        obs_or_q.w() = obs->getQuaternion().w() - (obsts_q_w_var/2) + obsts_q_w_var*(rand() / double(RAND_MAX));
                        if(this->ui.checkBox_obsts_filter_noise->isChecked()){
                            obs_pos.Xpos = this->lpf_obsts_pos_x->update(obs_pos.Xpos);
                            obs_pos.Ypos = this->lpf_obsts_pos_y->update(obs_pos.Ypos);
                            obs_pos.Zpos = this->lpf_obsts_pos_z->update(obs_pos.Zpos);
                            obs_or_q.x() = this->lpf_obsts_or_q_x->update(obs_or_q.x());
                            obs_or_q.y() = this->lpf_obsts_or_q_y->update(obs_or_q.y());
                            obs_or_q.z() = this->lpf_obsts_or_q_z->update(obs_or_q.z());
                            obs_or_q.w() = this->lpf_obsts_or_q_w->update(obs_or_q.w());
                        }
                        obs_new->setPos(obs_pos,false);
                        obs_new->setOr(obs_or_q,false);
                        obs_new->setSize(obs->getSize());
                        obsts_n.at(i) = obs_new;
                    }// for loop obstacles
                }// noise on obstacles
            }//obstacle avoidance enable


            // ---------------- start the simulation --------------------------- //
            if(!this->qnode.isSimulationRunning() || this->qnode.isSimulationPaused())
            {
                // enable set joints subscriber
                this->qnode.enableSetJoints();

                // start the simulation
                this->qnode.startSim();
            }
            ros::spinOnce(); // handle ROS messages
            double time_step = this->qnode.getSimTimeStep(); // sec

            if(this->qnode.isSimulationRunning() && ((this->qnode.getSimTime()-this->t_j_past)>time_step))
            {
                // posture
                vector<double> r_arm_posture_mes(JOINTS_ARM,0.0); vector<double> r_hand_posture_mes(JOINTS_HAND,0.0);
                vector<double> r_arm_posture(JOINTS_ARM,0.0); vector<double> r_hand_posture(JOINTS_HAND,0.0);
                // velocities
                VectorXd r_arm_null_velocities = VectorXd::Zero(JOINTS_ARM);
                vector<double> r_arm_velocities(JOINTS_ARM,0.0); vector<double> r_hand_velocities(JOINTS_HAND,0.0);
                vector<double> r_arm_velocities_read(JOINTS_ARM,0.0); vector<double> r_hand_velocities_read(JOINTS_HAND,0.0);
                // accelerations
                vector<double> r_arm_accelerations(JOINTS_ARM,0.0); vector<double> r_hand_accelerations(JOINTS_HAND,0.0);

                this->curr_scene->getHumanoid()->getRightArmPosture(r_arm_posture_mes);
                this->curr_scene->getHumanoid()->getRightHandPosture(r_hand_posture_mes);

                // filtering the joint positions
                r_arm_posture.at(0) = this->lpf_joint_pos_1->update(r_arm_posture_mes.at(0));
                r_arm_posture.at(1) = this->lpf_joint_pos_2->update(r_arm_posture_mes.at(1));
                r_arm_posture.at(2) = this->lpf_joint_pos_3->update(r_arm_posture_mes.at(2));
                r_arm_posture.at(3) = this->lpf_joint_pos_4->update(r_arm_posture_mes.at(3));
                r_arm_posture.at(4) = this->lpf_joint_pos_5->update(r_arm_posture_mes.at(4));
                r_arm_posture.at(5) = this->lpf_joint_pos_6->update(r_arm_posture_mes.at(5));
                r_arm_posture.at(6) = this->lpf_joint_pos_7->update(r_arm_posture_mes.at(6));
                r_hand_posture.at(0) = this->lpf_joint_pos_8->update(r_arm_posture_mes.at(0));
                r_hand_posture.at(1) = this->lpf_joint_pos_9->update(r_hand_posture_mes.at(1));
                r_hand_posture.at(2) = this->lpf_joint_pos_10->update(r_hand_posture_mes.at(2));
                r_hand_posture.at(3) = this->lpf_joint_pos_11->update(r_hand_posture_mes.at(3));

                // get the joint velocities
                this->arm_pos_buff->push(r_arm_posture);
                this->hand_pos_buff->push(r_hand_posture);
                if(this->samples_pos==this->N_filter_length-1 && this->arm_pos_buff->full() && this->hand_pos_buff->full()){
                    for(size_t i=0; i< r_arm_posture.size();++i)
                    {
                        r_arm_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_pos_buff->at(i));
                    }
                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
                    {
                        r_hand_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_pos_buff->at(i));
                    }
                }else{this->samples_pos++;}

                // get the joint accelerations
                this->arm_vel_buff->push(r_arm_velocities_read);
                this->hand_vel_buff->push(r_hand_velocities_read);
                if(this->samples_vel==this->N_filter_length-1 && this->arm_vel_buff->full() && this->hand_vel_buff->full()){
                    for(size_t i=0; i< r_arm_velocities_read.size();++i)
                    {
                        r_arm_accelerations.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_vel_buff->at(i));
                    }
                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
                    {
                        r_hand_accelerations.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_vel_buff->at(i));
                    }
                }else{this->samples_vel++;}

                vector<double> r_hand_pos; vector<double> r_wrist_pos; vector<double> r_elbow_pos; vector<double> r_shoulder_pos;
                this->curr_scene->getHumanoid()->getAllPos(1,r_hand_pos,r_wrist_pos,r_elbow_pos,r_shoulder_pos,r_arm_posture);
                VectorXd r_hand_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_pos.data(), r_hand_pos.size());

                vector<double> r_hand_lin_pos(r_hand_pos.begin(), r_hand_pos.begin()+3);
                vector<double> r_hand_ang_pos(r_hand_pos.begin()+3, r_hand_pos.begin()+6);
                vector<double> r_wrist_lin_pos(r_wrist_pos.begin(), r_wrist_pos.begin()+3);
                vector<double> r_wrist_ang_pos(r_wrist_pos.begin()+3, r_wrist_pos.begin()+6);
                vector<double> r_elbow_lin_pos(r_elbow_pos.begin(), r_elbow_pos.begin()+3);
                vector<double> r_elbow_ang_pos(r_elbow_pos.begin()+3, r_elbow_pos.begin()+6);
                vector<double> r_shoulder_lin_pos(r_shoulder_pos.begin(), r_shoulder_pos.begin()+3);
                vector<double> r_shoulder_ang_pos(r_shoulder_pos.begin()+3, r_shoulder_pos.begin()+6);

                vector<double> r_hand_vel; vector<double> r_wrist_vel; vector<double> r_elbow_vel; vector<double> r_shoulder_vel;
                this->curr_scene->getHumanoid()->getAllVel(1,r_hand_vel,r_wrist_vel,r_elbow_vel,r_shoulder_vel,r_arm_posture,r_arm_velocities_read);
                VectorXd r_hand_vel_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_vel.data(), r_hand_vel.size());

                vector<double> r_hand_lin_vel(r_hand_vel.begin(), r_hand_vel.begin()+3);
                vector<double> r_hand_ang_vel(r_hand_vel.begin()+3, r_hand_vel.begin()+6);
                vector<double> r_wrist_lin_vel(r_wrist_vel.begin(), r_wrist_vel.begin()+3);
                vector<double> r_wrist_ang_vel(r_wrist_vel.begin()+3, r_wrist_vel.begin()+6);
                vector<double> r_elbow_lin_vel(r_elbow_vel.begin(), r_elbow_vel.begin()+3);
                vector<double> r_elbow_ang_vel(r_elbow_vel.begin()+3, r_elbow_vel.begin()+6);
                vector<double> r_shoulder_lin_vel(r_shoulder_vel.begin(), r_shoulder_vel.begin()+3);
                vector<double> r_shoulder_ang_vel(r_shoulder_vel.begin()+3, r_shoulder_vel.begin()+6);

                // get the wrist acceleration
                vector<double> r_wrist_acc(6,0.0);
                this->r_wrist_vel_buff->push(r_wrist_vel);
                if(this->samples_w_vel==this->N_filter_length-1 && this->r_wrist_vel_buff->full()){
                    for(size_t i=0; i< r_wrist_vel.size();++i)
                    {
                        r_wrist_acc.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_wrist_vel_buff->at(i));
                    }
                }else{this->samples_w_vel++;}

                // get the elbow acceleration
                vector<double> r_elbow_acc(6,0.0);
                this->r_elbow_vel_buff->push(r_elbow_vel);
                if(this->samples_e_vel==this->N_filter_length-1 && this->r_elbow_vel_buff->full()){
                    for(size_t i=0; i< r_elbow_vel.size();++i)
                    {
                        r_elbow_acc.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_elbow_vel_buff->at(i));
                    }
                }else{this->samples_e_vel++;}

                // get the shoulder acceleration
                vector<double> r_shoulder_acc(6,0.0);
                this->r_shoulder_vel_buff->push(r_shoulder_vel);
                if(this->samples_s_vel==this->N_filter_length-1 && this->r_shoulder_vel_buff->full()){
                    for(size_t i=0; i< r_shoulder_vel.size();++i)
                    {
                        r_shoulder_acc.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_shoulder_vel_buff->at(i));
                    }
                }else{this->samples_s_vel++;}

                // get the hand acceleration
                vector<double> r_hand_acc(6,0.0);
                this->r_hand_vel_buff->push(r_hand_vel);
                if(this->samples_h_vel==this->N_filter_length-1 && this->r_hand_vel_buff->full()){
                    for(size_t i=0; i< r_hand_vel.size();++i)
                    {
                        r_hand_acc.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_hand_vel_buff->at(i));
                    }
                }else{
                    this->samples_h_vel++;
                    this->t_der_past = this->qnode.getSimTime();
                }
                VectorXd r_hand_acc_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_acc.data(), r_hand_acc.size());

                vector<double> r_hand_lin_acc(r_hand_acc.begin(), r_hand_acc.begin()+3);
                vector<double> r_hand_ang_acc(r_hand_acc.begin()+3, r_hand_acc.begin()+6);
                vector<double> r_wrist_lin_acc(r_wrist_acc.begin(), r_wrist_acc.begin()+3);
                vector<double> r_wrist_ang_acc(r_wrist_acc.begin()+3, r_wrist_acc.begin()+6);
                vector<double> r_elbow_lin_acc(r_elbow_acc.begin(), r_elbow_acc.begin()+3);
                vector<double> r_elbow_ang_acc(r_elbow_acc.begin()+3, r_elbow_acc.begin()+6);
                vector<double> r_shoulder_lin_acc(r_shoulder_acc.begin(), r_shoulder_acc.begin()+3);
                vector<double> r_shoulder_ang_acc(r_shoulder_acc.begin()+3, r_shoulder_acc.begin()+6);

                if(this->r_hand_init_pos.empty()){this->r_hand_init_pos=r_hand_pos;}
                VectorXd r_hand_init_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->r_hand_init_pos.data(), this->r_hand_init_pos.size());
                VectorXd hand_pos(6); VectorXd hand_acc(6);
                if (this->ui.checkBox_draw_ellipse->isChecked()){
                    // desired position
                    hand_pos(0) = r_hand_init_pos_vec(0)/0.2 + (40/0.2)*sin(0.2*this->qnode.getSimTime() - M_PI/2);
                    hand_pos(1) = r_hand_init_pos_vec(1)/0.2 + (60/0.2)*cos(0.2*this->qnode.getSimTime() - M_PI/2);
                    hand_pos(2) = r_hand_init_pos_vec(2);
                    hand_pos(3) = r_hand_init_pos_vec(3);
                    hand_pos(4) = r_hand_init_pos_vec(4);
                    hand_pos(5) = r_hand_init_pos_vec(5);
                    //desired acceleration
                    hand_acc(0) = -40*0.2*sin(0.2*this->qnode.getSimTime() - M_PI/2);
                    hand_acc(1) = -60*0.2*cos(0.2*this->qnode.getSimTime() - M_PI/2);
                    hand_acc(2) = 0.0;
                    hand_acc(3) = 0.0;
                    hand_acc(4) = 0.0;
                    hand_acc(5) = 0.0;
                }else{
                    // desired position
                    hand_pos = r_hand_init_pos_vec + hand_vel*this->qnode.getSimTime();
                    // desired acceleration
                    hand_acc = hand_vel/this->qnode.getSimTime();
                }

                // inverse differential kinematics
                this->curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm(1,r_arm_posture,hand_vel_vec,r_arm_velocities,r_arm_null_velocities,jlim_en,sing_en,obsts_en,
                                                                                vel_max,sing_coeff,sing_damping,obst_coeff,obst_damping,obst_coeff_torso,obst_damping_torso,
                                                                                jlim_th,jlim_rate,jlim_coeff,jlim_damping,obsts_n);


                // execute the control
                bool joints_arm_vel_ctrl = this->ui.checkBox_joints_velocity_ctrl->isChecked();
                this->qnode.execKinControl(1,r_arm_posture,r_arm_velocities,r_hand_posture,r_hand_velocities,joints_arm_vel_ctrl,false);

                // -------------- Recording ---------------------------- //

                // record the positions of the joints
                this->jointsPosition_ctrl.conservativeResize(this->jointsPosition_ctrl.rows()+1,11);
                for(size_t jj=0; jj < r_arm_posture.size(); ++jj)
                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,jj) = r_arm_posture.at(jj);
                for(size_t jj=0; jj < r_hand_posture.size(); ++jj)
                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,r_arm_posture.size()+jj) = r_hand_posture.at(jj);

                // record the velocities of the joints
                this->jointsVelocity_ctrl.conservativeResize(this->jointsVelocity_ctrl.rows()+1,11);
                for(size_t jj=0; jj < r_arm_velocities_read.size(); ++jj)
                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,jj) = r_arm_velocities_read.at(jj);
                for(size_t jj=0; jj < r_hand_velocities_read.size(); ++jj)
                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,r_arm_velocities_read.size()+jj) = r_hand_velocities_read.at(jj);

                this->jointsVelocity_null_ctrl.conservativeResize(this->jointsVelocity_null_ctrl.rows()+1,JOINTS_ARM);
                for(size_t jj=0; jj < r_arm_null_velocities.size(); ++jj)
                    this->jointsVelocity_null_ctrl(this->jointsVelocity_null_ctrl.rows()-1,jj) = r_arm_null_velocities(jj);

                // record the acceleration of the joints
                this->jointsAcceleration_ctrl.conservativeResize(this->jointsAcceleration_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
                for(size_t jj=0; jj < r_arm_accelerations.size(); ++jj)
                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,jj) = r_arm_accelerations.at(jj);
                for(size_t jj=0; jj < r_hand_accelerations.size(); ++jj)
                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,r_arm_accelerations.size()+jj) = r_hand_accelerations.at(jj);


                // operational space positions
                this->handPosition_ctrl.push_back(r_hand_lin_pos);
                this->handOrientation_ctrl.push_back(r_hand_ang_pos);
                this->wristPosition_ctrl.push_back(r_wrist_lin_pos);
                this->wristOrientation_ctrl.push_back(r_wrist_ang_pos);
                this->elbowPosition_ctrl.push_back(r_elbow_lin_pos);
                this->elbowOrientation_ctrl.push_back(r_elbow_ang_pos);
                this->shoulderPosition_ctrl.push_back(r_shoulder_lin_pos);
                this->shoulderOrientation_ctrl.push_back(r_shoulder_ang_pos);

                // operational space velocities
                this->handLinearVelocity_ctrl.push_back(r_hand_lin_vel);
                this->handAngularVelocity_ctrl.push_back(r_hand_ang_vel);
                this->wristLinearVelocity_ctrl.push_back(r_wrist_lin_vel);
                this->wristAngularVelocity_ctrl.push_back(r_wrist_ang_vel);
                this->elbowLinearVelocity_ctrl.push_back(r_elbow_lin_vel);
                this->elbowAngularVelocity_ctrl.push_back(r_elbow_ang_vel);
                this->shoulderLinearVelocity_ctrl.push_back(r_shoulder_lin_vel);
                this->shoulderAngularVelocity_ctrl.push_back(r_shoulder_ang_vel);
                this->handVelocityNorm_ctrl.push_back(this->curr_scene->getHumanoid()->getHandVelNorm(1,r_arm_posture,r_arm_velocities_read));

                // operational space accelerations
                this->handLinearAcceleration_ctrl.push_back(r_hand_lin_acc);
                this->handAngularAcceleration_ctrl.push_back(r_hand_ang_acc);
                this->wristLinearAcceleration_ctrl.push_back(r_wrist_lin_acc);
                this->wristAngularAcceleration_ctrl.push_back(r_wrist_ang_acc);
                this->elbowLinearAcceleration_ctrl.push_back(r_elbow_lin_acc);
                this->elbowAngularAcceleration_ctrl.push_back(r_elbow_ang_acc);
                this->shoulderLinearAcceleration_ctrl.push_back(r_shoulder_lin_acc);
                this->shoulderAngularAcceleration_ctrl.push_back(r_shoulder_ang_acc);
                this->handAccelerationNorm_ctrl.push_back(sqrt(pow(r_hand_lin_acc.at(0),2)+pow(r_hand_lin_acc.at(1),2)+pow(r_hand_lin_acc.at(2),2)));

                //errors
                VectorXd error_pos_tot = hand_pos - r_hand_pos_vec;
                this->error_pos_tot_norm.push_back(error_pos_tot.block<3,1>(0,0).norm());
                this->error_or_tot_norm.push_back(error_pos_tot.block<3,1>(3,0).norm());
                this->error_pos_or_tot_norm.push_back(error_pos_tot.norm());
                VectorXd error_vel_tot = hand_vel - r_hand_vel_vec;
                this->error_lin_vel_tot_norm.push_back(error_vel_tot.block<3,1>(0,0).norm());
                this->error_ang_vel_tot_norm.push_back(error_vel_tot.block<3,1>(3,0).norm());
                this->error_vel_tot_norm.push_back(error_vel_tot.norm());
                VectorXd error_acc_tot = hand_acc - r_hand_acc_vec;
                this->error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
                this->error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
                this->error_acc_tot_norm.push_back(error_acc_tot.norm());

                // time
                this->t_j_past = this->qnode.getSimTime();
                this->sim_time.push_back(this->qnode.getSimTime()-this->t_der_past);
            } // check simulation step
        } // if vel control
    }// while execcontrol
}

//void MainWindow::execVelControl()
//{
//    while(exec_control)
//    {

//        if(vel_control)
//        {
//            boost::unique_lock<boost::mutex> lck(hh_control_mtx);

//            double des_hand_acc_x,des_hand_acc_y,des_hand_acc_z,des_hand_acc_wx,des_hand_acc_wy,des_hand_acc_wz;
//            if (this->ui.checkBox_draw_ellipse->isChecked()){
//                des_hand_acc_x = 400*cos(0.2*this->qnode.getSimTime()-M_PI/2);
//                des_hand_acc_y = -600*sin(0.2*this->qnode.getSimTime()-M_PI/2);
//                des_hand_acc_z = 0.0;
//                des_hand_acc_wx = 0.0;
//                des_hand_acc_wy = 0.0;
//                des_hand_acc_wz = 0.0;
//            }else{
//                des_hand_acc_x = this->ui.lineEdit_des_right_hand_vel_x->text().toDouble();
//                des_hand_acc_y = this->ui.lineEdit_des_right_hand_vel_y->text().toDouble();
//                des_hand_acc_z = this->ui.lineEdit_des_right_hand_vel_z->text().toDouble();
//                des_hand_acc_wx = this->ui.lineEdit_des_right_hand_vel_wx->text().toDouble();
//                des_hand_acc_wy = this->ui.lineEdit_des_right_hand_vel_wy->text().toDouble();
//                des_hand_acc_wz = this->ui.lineEdit_des_right_hand_vel_wz->text().toDouble();
//            }
//            // desired acceleration
//            VectorXd h_hand_ref_acc(6);
//            h_hand_ref_acc(0) = des_hand_acc_x;
//            h_hand_ref_acc(1) = des_hand_acc_y;
//            h_hand_ref_acc(2) = des_hand_acc_z;
//            h_hand_ref_acc(3) = des_hand_acc_wx;
//            h_hand_ref_acc(4) = des_hand_acc_wy;
//            h_hand_ref_acc(5) = des_hand_acc_wz;

//            double vel_max = this->ui.lineEdit_vel_max->text().toDouble()*M_PI/180;

//            bool jlim_en = this->ui.checkBox_joints_limits_av->isChecked();
//            double jlim_th = 0; double jlim_rate = 1;
//            double jlim_coeff = 1; double jlim_damping = 0.001;
//            if(jlim_en){
//                jlim_th = this->ui.lineEdit_jlim_th->text().toDouble();
//                jlim_rate = this->ui.lineEdit_jlim_rate->text().toDouble();
//                jlim_coeff = this->ui.lineEdit_jlim_coeff->text().toDouble();
//                jlim_damping = this->ui.lineEdit_jlim_damping->text().toDouble();
//            }
//            bool sing_en = this->ui.checkBox_sing_av->isChecked();
//            double sing_coeff = 1; double sing_damping = 0.001;
//            if(sing_en){
//                sing_damping = this->ui.lineEdit_sing_damping->text().toDouble();
//                sing_coeff = this->ui.lineEdit_sing_coeff->text().toDouble();
//            }

//            double obst_coeff = 1; double obst_damping = 0.001;
//            double obst_coeff_torso = 1; double obst_damping_torso = 0.001;
//            bool obsts_en = this->ui.checkBox_obsts_av->isChecked();
//            vector<objectPtr> obsts; this->curr_scene->getObjects(obsts);
//            if(obsts_en){
//                obst_coeff = this->ui.lineEdit_obsts_coeff->text().toDouble();
//                obst_damping = this->ui.lineEdit_obsts_damping->text().toDouble();
//                obst_coeff_torso = this->ui.lineEdit_obsts_coeff_torso->text().toDouble();
//                obst_damping_torso = this->ui.lineEdit_obsts_damping_torso->text().toDouble();
//                if(this->ui.checkBox_obsts_noise->isChecked()){
//                    objectPtr obs_new;
//                    double obsts_x_var = 100; // mm
//                    double obsts_y_var = 100; // mm
//                    double obsts_z_var = 100; // mm
//                    double obsts_roll_var = 0.5; // rad
//                    double obsts_pitch_var = 0.5; // rad
//                    double obsts_yaw_var = 0.5; // rad
//                    for(size_t i=0; i<obsts.size(); ++i){
//                        std::srand(std::time(NULL));
//                        objectPtr obs = obsts.at(i);
//                        string obs_name = obs->getName();
//                        obs_new.reset(new Object(obs_name));
//                        motion_manager::pos obs_pos;
//                        motion_manager::orient obs_or;
//                        obs_pos.Xpos = obs->getPos().Xpos - (obsts_x_var/2) + obsts_x_var*(rand() / double(RAND_MAX));
//                        obs_pos.Ypos = obs->getPos().Ypos - (obsts_y_var/2) + obsts_y_var*(rand() / double(RAND_MAX));
//                        obs_pos.Zpos = obs->getPos().Zpos - (obsts_z_var/2) + obsts_z_var*(rand() / double(RAND_MAX));
//                        obs_or.roll = obs->getOr().roll - (obsts_roll_var/2) + obsts_roll_var*(rand() / double(RAND_MAX));
//                        obs_or.pitch = obs->getOr().pitch - (obsts_pitch_var/2) + obsts_pitch_var*(rand() / double(RAND_MAX));
//                        obs_or.yaw = obs->getOr().yaw - (obsts_yaw_var/2) + obsts_yaw_var*(rand() / double(RAND_MAX));
//                        if(this->ui.checkBox_obsts_noise->isChecked()){
//                            obs_pos.Xpos = this->lpf_obsts_pos_x->update(obs_pos.Xpos);
//                            obs_pos.Ypos = this->lpf_obsts_pos_y->update(obs_pos.Ypos);
//                            obs_pos.Zpos = this->lpf_obsts_pos_z->update(obs_pos.Zpos);
//                            obs_or.roll = DEGTORAD*this->lpf_obsts_or_roll->update(RADTODEG*obs_or.roll);
//                            obs_or.pitch = DEGTORAD*this->lpf_obsts_or_pitch->update(RADTODEG*obs_or.pitch);
//                            obs_or.yaw = DEGTORAD*this->lpf_obsts_or_yaw->update(RADTODEG*obs_or.yaw);
//                        }
//                        obs_new->setPos(obs_pos,false);
//                        obs_new->setOr(obs_or,false);
//                        obs_new->setSize(obs->getSize());
//                        this->curr_scene->setObject(i,obs_new);
//                    }// for loop obstacles
//                }// noise on obstacles
//            }//obstacle avoidance enable

//            VectorXd hand_acc(6);
//            hand_acc << des_hand_acc_x,des_hand_acc_y,des_hand_acc_z,
//                        des_hand_acc_wx,des_hand_acc_wy,des_hand_acc_wz;

//            if(!this->ui.checkBox_des_right_hand_vel_x->isChecked()){
//                hand_acc(0) = 0.0;
//            }
//            if(!this->ui.checkBox_des_right_hand_vel_y->isChecked()){
//                hand_acc(1) = 0.0;
//            }
//            if(!this->ui.checkBox_des_right_hand_vel_z->isChecked()){
//                hand_acc(2) = 0.0;
//            }
//            if(!this->ui.checkBox_des_right_hand_vel_wx->isChecked()){
//                hand_acc(3) = 0.0;
//            }
//            if(!this->ui.checkBox_des_right_hand_vel_wy->isChecked()){
//                hand_acc(4) = 0.0;
//            }
//            if(!this->ui.checkBox_des_right_hand_vel_wz->isChecked()){
//                hand_acc(5) = 0.0;
//            }

//            // ---------------- start the simulation --------------------------- //
//            if(!this->qnode.isSimulationRunning() || this->qnode.isSimulationPaused())
//            {
//                // enable set joints subscriber
//                this->qnode.enableSetJoints();

//                // start the simulation
//                this->qnode.startSim();
//            }
//            ros::spinOnce(); // handle ROS messages
//            double time_step = this->qnode.getSimTimeStep(); // sec

//            if(this->qnode.isSimulationRunning() && ((this->qnode.getSimTime()-this->t_j_past)>time_step))
//            {

//                // posture
//                vector<double> r_arm_posture_mes(JOINTS_ARM,0.0); vector<double> r_hand_posture_mes(JOINTS_HAND,0.0);
//                vector<double> r_arm_posture(JOINTS_ARM,0.0); vector<double> r_hand_posture(JOINTS_HAND,0.0);
//                // velocities
//                VectorXd r_arm_null_velocities = VectorXd::Zero(JOINTS_ARM);
//                vector<double> r_arm_velocities(JOINTS_ARM,0.0); vector<double> r_hand_velocities(JOINTS_HAND,0.0);
//                vector<double> r_arm_velocities_read(JOINTS_ARM,0.0); vector<double> r_hand_velocities_read(JOINTS_HAND,0.0);
//                // accelerations
//                //vector<double> r_arm_accelerations(JOINTS_ARM,0.0);
//                vector<double> r_arm_accelerations_read(JOINTS_ARM,0.0); vector<double> r_hand_accelerations_read(JOINTS_HAND,0.0);
//                vector<double> r_hand_acc_read(6,0.0);
//                vector<double> r_wrist_acc_read(6,0.0);
//                vector<double> r_elbow_acc_read(6,0.0);
//                vector<double> r_shoulder_acc_read(6,0.0);


//                this->curr_scene->getHumanoid()->getRightArmPosture(r_arm_posture_mes);
//                this->curr_scene->getHumanoid()->getRightHandPosture(r_hand_posture_mes);
//                // filtering the joint positions
//                r_arm_posture.at(0) = this->lpf_joint_pos_1->update(r_arm_posture_mes.at(0));
//                r_arm_posture.at(1) = this->lpf_joint_pos_2->update(r_arm_posture_mes.at(1));
//                r_arm_posture.at(2) = this->lpf_joint_pos_3->update(r_arm_posture_mes.at(2));
//                r_arm_posture.at(3) = this->lpf_joint_pos_4->update(r_arm_posture_mes.at(3));
//                r_arm_posture.at(4) = this->lpf_joint_pos_5->update(r_arm_posture_mes.at(4));
//                r_arm_posture.at(5) = this->lpf_joint_pos_6->update(r_arm_posture_mes.at(5));
//                r_arm_posture.at(6) = this->lpf_joint_pos_7->update(r_arm_posture_mes.at(6));
//                r_hand_posture.at(0) = this->lpf_joint_pos_8->update(r_hand_posture_mes.at(0));
//                r_hand_posture.at(1) = this->lpf_joint_pos_9->update(r_hand_posture_mes.at(1));
//                r_hand_posture.at(2) = this->lpf_joint_pos_10->update(r_hand_posture_mes.at(2));
//                r_hand_posture.at(3) = this->lpf_joint_pos_11->update(r_hand_posture_mes.at(3));

//                vector<double> r_hand_pos; vector<double> r_wrist_pos; vector<double> r_elbow_pos; vector<double> r_shoulder_pos;
//                this->curr_scene->getHumanoid()->getAllPos(1,r_hand_pos,r_wrist_pos,r_elbow_pos,r_shoulder_pos,r_arm_posture);
//                vector<double> r_hand_pos_q; vector<double> r_wrist_pos_q; vector<double> r_elbow_pos_q; vector<double> r_shoulder_pos_q;
//                this->curr_scene->getHumanoid()->getAllPos_q(1,r_hand_pos_q,r_wrist_pos_q,r_elbow_pos_q,r_shoulder_pos_q,r_arm_posture);

//                if(this->r_hand_init_pos.empty()){this->r_hand_init_pos=r_hand_pos;}
//                VectorXd r_hand_init_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->r_hand_init_pos.data(), this->r_hand_init_pos.size());
//                // desired position
//                VectorXd h_hand_ref_pos = r_hand_init_pos_vec + 0.5*h_hand_ref_acc*pow(this->qnode.getSimTime(),2);
//                // desired velocity
//                VectorXd h_hand_ref_vel = h_hand_ref_acc*this->qnode.getSimTime();


//                vector<double> r_hand_lin_pos(r_hand_pos.begin(), r_hand_pos.begin()+3);
//                vector<double> r_hand_ang_pos(r_hand_pos.begin()+3, r_hand_pos.begin()+6);
//                vector<double> r_hand_q(r_hand_pos_q.begin()+3, r_hand_pos_q.begin()+7);
//                vector<double> r_wrist_lin_pos(r_wrist_pos.begin(), r_wrist_pos.begin()+3);
//                vector<double> r_wrist_ang_pos(r_wrist_pos.begin()+3, r_wrist_pos.begin()+6);
//                vector<double> r_elbow_lin_pos(r_elbow_pos.begin(), r_elbow_pos.begin()+3);
//                vector<double> r_elbow_ang_pos(r_elbow_pos.begin()+3, r_elbow_pos.begin()+6);
//                vector<double> r_shoulder_lin_pos(r_shoulder_pos.begin(), r_shoulder_pos.begin()+3);
//                vector<double> r_shoulder_ang_pos(r_shoulder_pos.begin()+3, r_shoulder_pos.begin()+6);

//                // get the joint velocities
//                this->arm_pos_buff->push(r_arm_posture);
//                this->hand_pos_buff->push(r_hand_posture);
//                if(this->samples_pos==this->N_filter_length-1 && this->arm_pos_buff->full() && this->hand_pos_buff->full()){
//                    for(size_t i=0; i< r_arm_posture.size();++i)
//                    {
//                        r_arm_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_pos_buff->at(i));
//                    }
//                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
//                    {
//                        r_hand_velocities_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_pos_buff->at(i));
//                    }
//                }else{this->samples_pos++;}

//                vector<double> r_hand_vel; vector<double> r_wrist_vel; vector<double> r_elbow_vel; vector<double> r_shoulder_vel;
//                this->curr_scene->getHumanoid()->getAllVel(1,r_hand_vel,r_wrist_vel,r_elbow_vel,r_shoulder_vel,r_arm_posture,r_arm_velocities_read);

//                vector<double> r_hand_lin_vel(r_hand_vel.begin(), r_hand_vel.begin()+3);
//                vector<double> r_hand_ang_vel(r_hand_vel.begin()+3, r_hand_vel.begin()+6);
//                vector<double> r_wrist_lin_vel(r_wrist_vel.begin(), r_wrist_vel.begin()+3);
//                vector<double> r_wrist_ang_vel(r_wrist_vel.begin()+3, r_wrist_vel.begin()+6);
//                vector<double> r_elbow_lin_vel(r_elbow_vel.begin(), r_elbow_vel.begin()+3);
//                vector<double> r_elbow_ang_vel(r_elbow_vel.begin()+3, r_elbow_vel.begin()+6);
//                vector<double> r_shoulder_lin_vel(r_shoulder_vel.begin(), r_shoulder_vel.begin()+3);
//                vector<double> r_shoulder_ang_vel(r_shoulder_vel.begin()+3, r_shoulder_vel.begin()+6);

//                // get the joint accelerations
//                this->arm_vel_buff->push(r_arm_velocities_read);
//                this->hand_vel_buff->push(r_hand_velocities_read);
//                if(this->samples_vel==this->N_filter_length-1 && this->arm_vel_buff->full() && this->hand_vel_buff->full()){
//                    for(size_t i=0; i< r_arm_velocities_read.size();++i)
//                    {
//                        r_arm_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_vel_buff->at(i));
//                    }
//                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
//                    {
//                        r_hand_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_vel_buff->at(i));
//                    }
//                }else{this->samples_vel++;}

//                // get the joint accelerations
//                this->arm_vel_buff->push(r_arm_velocities_read);
//                this->hand_vel_buff->push(r_hand_velocities_read);
//                if(this->samples_vel==this->N_filter_length-1 && this->arm_vel_buff->full() && this->hand_vel_buff->full()){
//                    for(size_t i=0; i< r_arm_velocities_read.size();++i)
//                    {
//                        r_arm_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->arm_vel_buff->at(i));
//                    }
//                    for(size_t i=0; i< r_hand_velocities_read.size();++i)
//                    {
//                        r_hand_accelerations_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->hand_vel_buff->at(i));
//                    }
//                }else{this->samples_vel++;}


//                // get the hand acceleration
//                this->r_hand_vel_buff->push(r_hand_vel);
//                if(this->samples_h_vel==this->N_filter_length-1 && this->r_hand_vel_buff->full()){
//                    for(size_t i=0; i< r_hand_vel.size();++i)
//                    {
//                        r_hand_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_hand_vel_buff->at(i));
//                    }
//                }else{this->samples_h_vel++;}


//                // get the wrist acceleration
//                this->r_wrist_vel_buff->push(r_wrist_vel);
//                if(this->samples_w_vel==this->N_filter_length-1 && this->r_wrist_vel_buff->full()){
//                    for(size_t i=0; i< r_wrist_vel.size();++i)
//                    {
//                        r_wrist_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_wrist_vel_buff->at(i));
//                    }
//                }else{this->samples_w_vel++;}

//                // get the elbow acceleration
//                this->r_elbow_vel_buff->push(r_elbow_vel);
//                if(this->samples_e_vel==this->N_filter_length-1 && this->r_elbow_vel_buff->full()){
//                    for(size_t i=0; i< r_elbow_vel.size();++i)
//                    {
//                        r_elbow_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_elbow_vel_buff->at(i));
//                    }
//                }else{this->samples_e_vel++;}

//                // get the shoulder acceleration
//                this->r_shoulder_vel_buff->push(r_shoulder_vel);
//                if(this->samples_s_vel==this->N_filter_length-1 && this->r_shoulder_vel_buff->full()){
//                    for(size_t i=0; i< r_shoulder_vel.size();++i)
//                    {
//                        r_shoulder_acc_read.at(i) = this->getNoiseRobustDerivate(this->N_filter_length,time_step,this->r_shoulder_vel_buff->at(i));
//                    }
//                }else{
//                    this->samples_s_vel++;
//                    this->t_der_past = this->qnode.getSimTime();
//                }

//                VectorXd r_arm_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_arm_accelerations_read.data(), r_arm_accelerations_read.size());
//                VectorXd r_hand_pos_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_pos.data(), r_hand_pos.size());
//                VectorXd r_hand_vel_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_vel.data(), r_hand_vel.size());
//                VectorXd r_hand_acc_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_acc_read.data(), r_hand_acc_read.size());
//                // Jacobian
//                this->curr_scene->getHumanoid()->getJacobian(1,r_arm_posture,this->Jacobian);
//                this->hand_j_acc = r_hand_acc_read_vec - this->Jacobian*r_arm_accelerations_read_vec;

//                vector<double> r_hand_lin_acc(r_hand_acc_read.begin(), r_hand_acc_read.begin()+3);
//                vector<double> r_hand_ang_acc(r_hand_acc_read.begin()+3, r_hand_acc_read.begin()+6);
//                vector<double> r_wrist_lin_acc(r_wrist_acc_read.begin(), r_wrist_acc_read.begin()+3);
//                vector<double> r_wrist_ang_acc(r_wrist_acc_read.begin()+3, r_wrist_acc_read.begin()+6);
//                vector<double> r_elbow_lin_acc(r_elbow_acc_read.begin(), r_elbow_acc_read.begin()+3);
//                vector<double> r_elbow_ang_acc(r_elbow_acc_read.begin()+3, r_elbow_acc_read.begin()+6);
//                vector<double> r_shoulder_lin_acc(r_shoulder_acc_read.begin(), r_shoulder_acc_read.begin()+3);
//                vector<double> r_shoulder_ang_acc(r_shoulder_acc_read.begin()+3, r_shoulder_acc_read.begin()+6);

//                VectorXd hand_acc_xd_vec = h_hand_ref_acc - this->hand_j_acc;
//                vector<double> hand_acc_vec; hand_acc_vec.resize(hand_acc_xd_vec.size());
//                VectorXd::Map(&hand_acc_vec[0], hand_acc_xd_vec.size()) = hand_acc_xd_vec;

//                this->curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm2(1,r_arm_posture_mes,r_arm_velocities_read,hand_acc_vec,r_arm_velocities,r_arm_null_velocities,time_step,jlim_en,sing_en,obsts_en,
//                                                                                vel_max,sing_coeff,sing_damping,obst_coeff,obst_damping,obst_coeff_torso,obst_damping_torso,
//                                                                                 jlim_th,jlim_rate,jlim_coeff,jlim_damping,obsts);

//                // execute the control
//                bool joints_arm_vel_ctrl = this->ui.checkBox_joints_velocity_ctrl->isChecked();
//                this->qnode.execKinControl(1,r_arm_posture_mes,r_arm_velocities,r_hand_posture_mes,r_hand_velocities,joints_arm_vel_ctrl);

////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec x = " << hand_acc_xd_vec(0);
////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec y = " << hand_acc_xd_vec(1);
////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec z = " << hand_acc_xd_vec(2);
////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec wx = " << hand_acc_xd_vec(3);
////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec wy = " << hand_acc_xd_vec(4);
////                BOOST_LOG_SEV(lg, info) << "hand_acc_xd_vec wz = " << hand_acc_xd_vec(5);

////                BOOST_LOG_SEV(lg, info) << "r_arm_velocities 0 = " << r_arm_velocities.at(0);
////                BOOST_LOG_SEV(lg, info) << "r_arm_velocities 1 = " << r_arm_velocities.at(1);
////                BOOST_LOG_SEV(lg, info) << "r_arm_velocities 2 = " << r_arm_velocities.at(2);

//                // ------------- Recording ------------------------------- //

//                // record the positions of the joints
//                this->jointsPosition_ctrl.conservativeResize(this->jointsPosition_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
//                for(size_t jj=0; jj < r_arm_posture_mes.size(); ++jj)
//                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,jj) = r_arm_posture_mes.at(jj);
//                for(size_t jj=0; jj < r_hand_posture_mes.size(); ++jj)
//                    this->jointsPosition_ctrl(this->jointsPosition_ctrl.rows()-1,r_arm_posture_mes.size()+jj) = r_hand_posture_mes.at(jj);

//                // record the velocities of the joints
//                this->jointsVelocity_ctrl.conservativeResize(this->jointsVelocity_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
//                for(size_t jj=0; jj < r_arm_velocities_read.size(); ++jj)
//                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,jj) = r_arm_velocities_read.at(jj);
//                for(size_t jj=0; jj < r_hand_velocities_read.size(); ++jj)
//                    this->jointsVelocity_ctrl(this->jointsVelocity_ctrl.rows()-1,r_arm_velocities_read.size()+jj) = r_hand_velocities_read.at(jj);

//                this->jointsVelocity_null_ctrl.conservativeResize(this->jointsVelocity_null_ctrl.rows()+1,JOINTS_ARM);
//                for(size_t jj=0; jj < r_arm_null_velocities.size(); ++jj)
//                    this->jointsVelocity_null_ctrl(this->jointsVelocity_null_ctrl.rows()-1,jj) = r_arm_null_velocities(jj);


//                // record the acceleration of the joints
//                this->jointsAcceleration_ctrl.conservativeResize(this->jointsAcceleration_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
//                for(size_t jj=0; jj < r_arm_accelerations_read.size(); ++jj)
//                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,jj) = r_arm_accelerations_read.at(jj);
//                for(size_t jj=0; jj < r_hand_accelerations_read.size(); ++jj)
//                    this->jointsAcceleration_ctrl(this->jointsAcceleration_ctrl.rows()-1,r_arm_accelerations_read.size()+jj) = r_hand_accelerations_read.at(jj);

//                // operational space positions
//                this->handPosition_ctrl.push_back(r_hand_lin_pos);
//                this->handOrientation_ctrl.push_back(r_hand_ang_pos);
//                this->wristPosition_ctrl.push_back(r_wrist_lin_pos);
//                this->wristOrientation_ctrl.push_back(r_wrist_ang_pos);
//                this->elbowPosition_ctrl.push_back(r_elbow_lin_pos);
//                this->elbowOrientation_ctrl.push_back(r_elbow_ang_pos);
//                this->shoulderPosition_ctrl.push_back(r_shoulder_lin_pos);
//                this->shoulderOrientation_ctrl.push_back(r_shoulder_ang_pos);

//                // operational space velocities
//                this->handLinearVelocity_ctrl.push_back(r_hand_lin_vel);
//                this->handAngularVelocity_ctrl.push_back(r_hand_ang_vel);
//                this->wristLinearVelocity_ctrl.push_back(r_wrist_lin_vel);
//                this->wristAngularVelocity_ctrl.push_back(r_wrist_ang_vel);
//                this->elbowLinearVelocity_ctrl.push_back(r_elbow_lin_vel);
//                this->elbowAngularVelocity_ctrl.push_back(r_elbow_ang_vel);
//                this->shoulderLinearVelocity_ctrl.push_back(r_shoulder_lin_vel);
//                this->shoulderAngularVelocity_ctrl.push_back(r_shoulder_ang_vel);
//                this->handVelocityNorm_ctrl.push_back(sqrt(pow(r_hand_lin_vel.at(0),2)+pow(r_hand_lin_vel.at(1),2)+pow(r_hand_lin_vel.at(2),2)));

//                // operational space accelerations
//                this->handLinearAcceleration_ctrl.push_back(r_hand_lin_acc);
//                this->handAngularAcceleration_ctrl.push_back(r_hand_ang_acc);
//                this->wristLinearAcceleration_ctrl.push_back(r_wrist_lin_acc);
//                this->wristAngularAcceleration_ctrl.push_back(r_wrist_ang_acc);
//                this->elbowLinearAcceleration_ctrl.push_back(r_elbow_lin_acc);
//                this->elbowAngularAcceleration_ctrl.push_back(r_elbow_ang_acc);
//                this->shoulderLinearAcceleration_ctrl.push_back(r_shoulder_lin_acc);
//                this->shoulderAngularAcceleration_ctrl.push_back(r_shoulder_ang_acc);
//                this->handAccelerationNorm_ctrl.push_back(sqrt(pow(r_hand_lin_acc.at(0),2)+pow(r_hand_lin_acc.at(1),2)+pow(r_hand_lin_acc.at(2),2)));

//                VectorXd error_pos_tot = h_hand_ref_pos - r_hand_pos_read_vec;
//                this->error_pos_tot_norm.push_back(error_pos_tot.block<3,1>(0,0).norm());
//                this->error_or_tot_norm.push_back(error_pos_tot.block<3,1>(3,0).norm());
//                this->error_pos_or_tot_norm.push_back(error_pos_tot.norm());
//                VectorXd error_vel_tot = h_hand_ref_vel - r_hand_vel_read_vec;
//                this->error_lin_vel_tot_norm.push_back(error_vel_tot.block<3,1>(0,0).norm());
//                this->error_ang_vel_tot_norm.push_back(error_vel_tot.block<3,1>(3,0).norm());
//                this->error_vel_tot_norm.push_back(error_vel_tot.norm());
//                VectorXd error_acc_tot = h_hand_ref_acc - r_hand_acc_read_vec;
//                this->error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
//                this->error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
//                this->error_acc_tot_norm.push_back(error_acc_tot.norm());

//                // time
//                this->t_j_past = this->qnode.getSimTime();
//                this->sim_time.push_back(this->qnode.getSimTime()-this->t_der_past);
//            } // check simulation step
//        } // if vel control
//    }// while execcontrol
//}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::init()
{

    logging::add_file_log
    (
        keywords::file_name = "QNode_%N.log",                                        /*< file name pattern >*/
        keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
        keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0,0,0), /*< ...or at midnight >*/
        keywords::format = "[%TimeStamp%]: %Message%",                                 /*< log record format >*/
        keywords::target = "Boost_logs"
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
}

void MainWindow::updateLoggingView()
{
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateRosStatus(bool c)
{

    if (c){
        ui.labelRosComm->setText(QString("connected"));
        ui.actionVrep_Communication->setEnabled(true);
        ui.labelStatusVrep->setEnabled(true);
        ui.labelVrepComm->setEnabled(true);
    }else{

        ui.labelRosComm->setText(QString("disconnected"));
        ui.actionVrep_Communication->setEnabled(false);
        ui.labelStatusVrep->setEnabled(false);
        ui.labelVrepComm->setEnabled(false);
    }


}

void MainWindow::updateVrepStatus(bool c)
{

    if (c){
        ui.labelVrepComm->setText(QString("on-line"));
#if MOVEIT==1
        ui.actionRViz_Communication->setEnabled(true);
        ui.labelStatusRViz->setEnabled(true);
        ui.labelRVizComm->setEnabled(true);
#elif MOVEIT==0
        ui.tab_scenario->setEnabled(true);
        ui.groupBox_selectScenario->setEnabled(true);

#endif
        //ui.tab_scenario->setEnabled(true);
        //ui.groupBox_selectScenario->setEnabled(true);
        //ui.listWidget_scenario->setCurrentRow(0);
    }else{
        ui.labelVrepComm->setText(QString("off-line"));
        ui.actionRViz_Communication->setEnabled(false);
        ui.labelStatusRViz->setEnabled(false);
        ui.labelRVizComm->setEnabled(false);
        //ui.tab_scenario->setEnabled(false);
        //ui.groupBox_selectScenario->setEnabled(false);
        //ui.listWidget_scenario->setCurrentRow(0);
    }
    ui.pushButton_loadScenario->setEnabled(false);
    ui.groupBox_getElements->setEnabled(false);
}

void MainWindow::updateRVizStatus(bool c)
{
    if(c){
        ui.labelRVizComm->setText(QString("on-line"));
        ui.tab_scenario->setEnabled(true);
        ui.groupBox_selectScenario->setEnabled(true);
        //ui.listWidget_scenario->setCurrentRow(0);
    }else{
        ui.labelRVizComm->setText(QString("off-line"));
        ui.tab_scenario->setEnabled(false);
        ui.groupBox_selectScenario->setEnabled(false);
        //ui.listWidget_scenario->setCurrentRow(0);
    }
    ui.pushButton_loadScenario->setEnabled(false);
    ui.groupBox_getElements->setEnabled(false);
}

void MainWindow::addElement(string value)
{

    ui.listWidget_elements->addItem(QString(value.c_str()));
    ui.listWidget_elements->setCurrentRow(0);

}

void MainWindow::updateElement(int id, string value)
{

    QListWidgetItem* curr_item = ui.listWidget_elements->takeItem(id);
    delete curr_item;
    ui.listWidget_elements->insertItem(id,QString(value.c_str()));

}


void MainWindow::addObject(string value)
{

   ui.comboBox_objects->addItem(QString(value.c_str()));
   ui.comboBox_objects_eng->addItem(QString(value.c_str()));

   ui.comboBox_objects_left->addItem(QString(value.c_str()));
   ui.comboBox_objects_eng_left->addItem(QString(value.c_str()));

}

void MainWindow::addPose(string value)
{

   ui.comboBox_poses->addItem(QString(value.c_str()));
   ui.comboBox_poses_left->addItem(QString(value.c_str()));
}


void MainWindow::updateHomePosture(string value)
{

    ui.listWidget_homePosture->addItem(QString(value.c_str()));
    ui.listWidget_homePosture->setCurrentRow(0);

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/


void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About the motion manager"),tr("<h2>motion_manager version 1.10</h2><p>Copyright: Gianpaolo Gulletta</p><p>The motion manager is a ROS package."
                                                           "This software is designed to plan the movements of the arms for any humanoid robot</p>"));
}

void MainWindow::on_actionRos_Communication_triggered()
{

    mrosCommdlg->show();

}

void MainWindow::on_actionVrep_Communication_triggered()
{

    mvrepCommdlg->show();
}
#if MOVEIT==1
void MainWindow::on_actionRViz_Communication_triggered()
{

    mrvizCommdlg->show();
}
#endif

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::on_pushButton_tuning_clicked()
{

    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    int arm_sel = prob->getMovement()->getArm();
    switch(planner_id){
    case 0: // HUMP
        if(arm_sel!=0){
            // unimanual motion
            mTolHumpdlg->setInitJointsVel(this->jointsEndVelocity_mov);
            mTolHumpdlg->setInitJointsAcc(this->jointsEndAcceleration_mov);
            mTolHumpdlg->show();
        }else{
            //bimanual motion
            if(!this->jointsEndVelocity_mov.empty() && !this->jointsEndAcceleration_mov.empty())
            {
                std::vector<double> end_vel_right(this->jointsEndVelocity_mov.begin(),this->jointsEndVelocity_mov.begin()+JOINTS_ARM+JOINTS_HAND);
                std::vector<double> end_vel_left(this->jointsEndVelocity_mov.begin()+JOINTS_ARM+JOINTS_HAND,this->jointsEndVelocity_mov.end());
                std::vector<double> end_acc_right(this->jointsEndAcceleration_mov.begin(),this->jointsEndAcceleration_mov.begin()+JOINTS_ARM+JOINTS_HAND);
                std::vector<double> end_acc_left(this->jointsEndAcceleration_mov.begin()+JOINTS_ARM+JOINTS_HAND,this->jointsEndAcceleration_mov.end());

                mTolHumpDualdlg->setInitJointsVelRight(end_vel_right);
                mTolHumpDualdlg->setInitJointsVelLeft(end_vel_left);
                mTolHumpDualdlg->setInitJointsAccRight(end_acc_right);
                mTolHumpDualdlg->setInitJointsAccLeft(end_acc_left);
            }
            mTolHumpDualdlg->show();
        }
        break;
    case 1: // RRT
        mRRTdlg->show();
        break;
    case 2: //RRTConnect
        mRRTConnectdlg->show();
        break;
    case 3: //RRTstar
        mRRTstardlg->show();
        break;
    case 4: //PRM
        mPRMdlg->show();
        break;
    case 5: // PRMstar
        mPRMstardlg->show();
        break;
    }

}


void MainWindow::on_pushButton_loadScenario_clicked()
{


    //this->scenario_id = ui.listWidget_scenario->currentRow();
    QString scenario_text = ui.listWidget_scenario->currentItem()->text();
    int equal;
    for(int i=0; i<scenarios.size();++i){
         equal = scenario_text.compare(scenarios.at(i));
         if(equal==0){
             // Empty scenario with ARoS
             string path_vrep_emptyscene_aros = PATH_SCENARIOS+string("/vrep/empty_aros.ttt");
             // Empty scenario: empty scenario with ARoS and NO self collisions
             string path_vrep_emptyscene_aros_no_self_coll = PATH_SCENARIOS+string("/vrep/empty_aros_no_coll.ttt");
             // Toy vehicle scenario with ARoS
             //string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros.ttt");
             string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_bill.ttt");
             // Drinking Service task with ARoS
             string path_vrep_drinking_aros = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_aros_bill.ttt");
             // Toy vehicle scenario with Jarde
             string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
             //string path_rviz_toyscene_jarde = PATH_SCENARIOS+string("/rviz/toy_vehicle_jarde.scene");
             // Challengingscenario with ARoS
             string path_vrep_challenge_aros = PATH_SCENARIOS+string("/vrep/NarrowShelf_aros.ttt");
             // Toy vehicle bi-manual manipulation to swap the columns
             string path_vrep_toyscene_aros_dual_arm_cols = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_dual_arm_cols.ttt");
             // Human assistance scenario: Moving a tray with ARoS (bi-manual manipulation)
             string path_vrep_drinking_aros_dual_arm_tray = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_aros_dual_arm_tray.ttt");
             // Natural obstacle avoidance with ARoS
             string path_vrep_natural_obst_av = PATH_SCENARIOS+string("/vrep/Natural_obst_avoidance_aros_1.ttt");
             // Learning tasks: reaching with one obstacle
             string path_vrep_learning_tasks_reaching_1 = PATH_SCENARIOS+string("/vrep/Learning_Reaching_1.ttt");
             // Learning tasks: reaching with many obstacles
             string path_vrep_learning_tasks_reaching_2 = PATH_SCENARIOS+string("/vrep/Learning_Reaching_2.ttt");
             // Learning tasks: picking the blue column
             string path_vrep_learning_tasks_picking_1 = PATH_SCENARIOS+string("/vrep/Learning_Picking_column_1.ttt");
             // Controlling: scenario without objects
             string path_vrep_controlling_no_objs = PATH_SCENARIOS+string("/vrep/Controlling_no_objs.ttt");
             // Controlling: scenario without objects for singularities
             string path_vrep_controlling_no_objs_sing = PATH_SCENARIOS+string("/vrep/Controlling_no_objs_sing.ttt");
             // Controlling: scenario with one obstacle and drawing an ellipse on the XY plane
             string path_vrep_controlling_obsts_av_ellipse = PATH_SCENARIOS+string("/vrep/Controlling_obsts_av_ellipse.ttt");
             // Controlling: pick a red column
             string path_vrep_controlling_pick = PATH_SCENARIOS+string("/vrep/Controlling_pick_ToyVehicle_aros.ttt");
             // Controlling: follow a moving red column
             string path_vrep_controlling_follow = PATH_SCENARIOS+string("/vrep/Controlling_pick_ToyVehicle_aros_moving_column.ttt");

             switch(i){
             case 0: // Assembly scenario

#if HAND == 0
             // Assembly scenario: the Toy vehicle with Jarde
             this->scenario_id = 1;

             if (qnode.loadScenario(path_vrep_toyscene_jarde,this->scenario_id)){
                 qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with Jarde HAS BEEN LOADED"));
                 ui.groupBox_getElements->setEnabled(true);
                 ui.groupBox_homePosture->setEnabled(true);
                 //ui.pushButton_loadScenario->setEnabled(false);
                 string title = string("Assembly scenario: the Toy vehicle with Jarde");
                 init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                 curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));

             }else{

                 qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with Jarde HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                 ui.groupBox_getElements->setEnabled(false);
                 ui.groupBox_homePosture->setEnabled(false);
                 ui.pushButton_loadScenario->setEnabled(true);
             }

             break;

#elif HAND == 1
                 // Assembly scenario: the Toy vehicle with ARoS
                 this->scenario_id = 0;

                 if (qnode.loadScenario(path_vrep_toyscene_aros,this->scenario_id)){
                     qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Assembly scenario: the Toy vehicle with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif

                 }else{
                     qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
                 break;

#endif

             case 1:// Empty scenario
#if HAND==0

#elif HAND==1
                 // Empty scenario with ARoS
                 this->scenario_id = 2;
                 if (qnode.loadScenario(path_vrep_emptyscene_aros,this->scenario_id)){
                     qnode.log(QNode::Info,string("Empty scenario: empty scenario with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Empty scenario: empty with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif

                 }else{
                     qnode.log(QNode::Error,std::string("Empty scenario: empty scenario with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }

#endif
                 break;
             case 2: //Empty scenario: empty scenario with ARoS and NO self collisions
#if HAND==0

#elif HAND==1
                 this->scenario_id = 3;
                 if (qnode.loadScenario(path_vrep_emptyscene_aros_no_self_coll,this->scenario_id)){
                     qnode.log(QNode::Info,string("Empty scenario: empty scenario with ARoS and NO collisions HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Empty scenario: empty scenario with ARoS and NO collisions");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif

                 }else{
                     qnode.log(QNode::Error,std::string("Empty scenario: empty scenario with ARoS and NO collisions HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 3:// Human assistance with ARoS
#if HAND==0

#elif HAND==1
                this->scenario_id = 4;
                 if (qnode.loadScenario(path_vrep_drinking_aros,this->scenario_id)){
                     qnode.log(QNode::Info,string("Human assistance scenario: Serving a drink with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Human assistance scenario: Serving a drink with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Human assistance scenario: Serving a drink with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 4: // Challenging scenario: picking a cup from a shelf with ARoS
#if HAND==0

#elif HAND==1
                this->scenario_id = 5;
                 if (qnode.loadScenario(path_vrep_challenge_aros,this->scenario_id)){
                     qnode.log(QNode::Info,string("Challenging scenario: picking a cup from a shelf with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Challenging scenario: picking a cup from a shelf with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Challenging scenario: picking a cup from a shelf with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             case 5: // Toy vehicle dual arm to swap columns
#if HAND==0

#elif HAND==1
                this->scenario_id = 6;
                 if (qnode.loadScenario(path_vrep_toyscene_aros_dual_arm_cols,this->scenario_id)){
                     qnode.log(QNode::Info,string("Assembly scenario: swap two columns of the toy vehicle HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Assembly scenario: swap two columns of the toy vehicle");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Assembly scenario: swap two columns of the toy vehicle HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif

                 break;                 
             case 6: // Human assistance scenario: Moving a tray with ARoS (dual-arms)
#if HAND==0

#elif HAND==1
                this->scenario_id = 7;
                 if (qnode.loadScenario(path_vrep_drinking_aros_dual_arm_tray,this->scenario_id)){
                     qnode.log(QNode::Info,string("Human assistance scenario: Moving a tray with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Human assistance scenario: Moving a tray with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Human assistance scenario: Moving a tray with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             case 7: // Natural obstacle avoidance with ARoS
#if HAND==0

#elif HAND==1
                this->scenario_id = 8;
                 if (qnode.loadScenario(path_vrep_natural_obst_av,this->scenario_id)){
                     qnode.log(QNode::Info,string("Natural obstacle avoidance with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Natural obstacle avoidance with ARoS");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Natural obstacle avoidance with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             case 8: // Learning Tasks: reaching with one obstacle
#if HAND==0

#elif HAND==1
                this->scenario_id = 9;
                 if (qnode.loadScenario(path_vrep_learning_tasks_reaching_1,this->scenario_id)){
                     qnode.log(QNode::Info,string("Learning tasks: reaching with one obstacle HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Learning tasks: reaching with one obstacle");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Learning tasks: reaching with one obstacle HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 9: // Learning Tasks: reaching with many obstacles
#if HAND==0

#elif HAND==1
                this->scenario_id = 10;
                 if (qnode.loadScenario(path_vrep_learning_tasks_reaching_2,this->scenario_id)){
                     qnode.log(QNode::Info,string("Learning tasks: reaching with many obstacles HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Learning tasks: reaching with many obstacles");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Learning tasks: reaching with many obstacles HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 10: // Learning Tasks: picking the blue column
#if HAND==0

#elif HAND==1
                this->scenario_id = 11;
                 if (qnode.loadScenario(path_vrep_learning_tasks_picking_1,this->scenario_id)){
                     qnode.log(QNode::Info,string("Learning tasks: picking the blue column HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Learning tasks: picking the blue column of tea");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Learning tasks: picking the blue column HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 11: // Controlling: scenario with no objects
#if HAND==0

#elif HAND==1
                this->scenario_id = 12;
                 if (qnode.loadScenario(path_vrep_controlling_no_objs,this->scenario_id)){
                     qnode.log(QNode::Info,string("Controlling: scenario without objects HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Controlling: scenario without objects");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Controlling: scenario without objects HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 12: // Controlling: scenario with no objects for showing singularities
#if HAND==0

#elif HAND==1
                this->scenario_id = 13;
                 if (qnode.loadScenario(path_vrep_controlling_no_objs_sing,this->scenario_id)){
                     qnode.log(QNode::Info,string("Controlling: scenario with no obstacles for showing the effects of singularities HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Controlling: scenario with no obstacles for showing the effects of singularities");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Controlling: scenario with no obstacles for showing the effects of singularities HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;
             case 13: // Controlling: scenario with one obstacle and drawing an ellipse on the XY plane
#if HAND==0

#elif HAND==1
                this->scenario_id = 14;
                 if (qnode.loadScenario(path_vrep_controlling_obsts_av_ellipse,this->scenario_id)){
                     qnode.log(QNode::Info,string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             case 14: // Controlling: pick a red column
#if HAND==0

#elif HAND==1
                this->scenario_id = 15;
                 if (qnode.loadScenario(path_vrep_controlling_pick,this->scenario_id)){
                     qnode.log(QNode::Info,string("Controlling: pick a red column HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Controlling: pick a red column");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Controlling: pick a red column HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             case 15: // Controlling: follow a moving red column
#if HAND==0

#elif HAND==1
                this->scenario_id = 16;
                 if (qnode.loadScenario(path_vrep_controlling_follow,this->scenario_id)){
                     qnode.log(QNode::Info,string("Controlling: follow a moving red column HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Controlling: follow a moving red column");
                     init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                     curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
#if MOVEIT==1
                     //this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));
#endif
                 }else{
                     qnode.log(QNode::Error,std::string("Controlling: follow a moving red column HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

             }

         }
    }

}

void MainWindow::on_pushButton_getElements_pressed()
{

    qnode.log(QNode::Info,string("getting the elements of the scenario . . ."));
    ui.pushButton_getElements->setCheckable(true);
    ui.pushButton_loadScenario->setEnabled(false);


}

void MainWindow::on_pushButton_plan_pressed()
{

    qnode.log(QNode::Info,string("planning the selected movement. . ."));
    ui.pushButton_plan->setCheckable(true);
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_mov->clear();
}



void MainWindow::on_pushButton_getElements_clicked()
{

    ui.listWidget_elements->clear();
    try{
        if (qnode.getElements(this->curr_scene)){

            this->init_scene = scenarioPtr(new Scenario(*(this->curr_scene.get()))); //set the init scenario
            this->curr_task = taskPtr(new Task());
            ui.pushButton_getElements->setEnabled(false);
            ui.tab_plan->setEnabled(true);
            ui.tab_learn->setEnabled(true);
            ui.tab_control->setEnabled(true);
            ui.tab_results->setEnabled(true);
            ui.groupBox_specs->setEnabled(true);
            ui.groupBox_task->setEnabled(false);
            ui.tabWidget_sol->setEnabled(false);
            // load the objects into RViz
            std::vector<objectPtr> objs; this->curr_scene->getObjects(objs);
#if MOVEIT==1
            qnode.loadRVizScenario(objs);
#endif
            qnode.log(QNode::Info,string("The elements of the scenario are now available"));

        }else{

            ui.pushButton_getElements->setEnabled(true);
            //ui.tab_plan->setEnabled(false);
            qnode.log(QNode::Error,string("Error in getting the elements of the scenario"));
        }

         ui.comboBox_objects_eng->setEnabled(false);

    }catch(std::string str){
        qnode.log(QNode::Error,str);
    }catch(std::exception e){
        qnode.log(QNode::Error,e.what());
    }


}


void MainWindow::on_pushButton_addMov_clicked()
{
    ui.pushButton_save_task->setEnabled(false);
    int planner_id = ui.comboBox_planner->currentIndex();

        bool add = true;
        /*
        int rows = ui.listWidget_movs->count();
        if (rows > 0){
            this->curr_task->getProblem(rows-1); // previous problem
            if(!pre_prob->getMovement()->getExecuted()){
                int reply = QMessageBox::warning(this,tr("Warning"),
                                     tr("<p>The previous movement has not been executed yet. Do you really want to add this movement?</p>"),QMessageBox::Yes,QMessageBox::No);
               add = (reply == QMessageBox::Yes);
            }
        }
        */


        if(add){
            bool success = false;
            int mov_id = ui.comboBox_mov->currentIndex();
            int arm_sel;
            if (ui.comboBox_Task->currentIndex()==0){
                //single-arm
                if(ui.radioButton_right->isChecked()){
                    arm_sel=1; // right arm
                }else{
                    arm_sel=2; // left arm
                }
            }else{
                //dual-arm
                arm_sel=0;
            }

            if(arm_sel!=0){
             // single arm
             if (ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled()){
                 // engage movements
                string obj_name = ui.comboBox_objects->currentText().toStdString();
                string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString();

                objectPtr obj = curr_scene->getObject(obj_name);
                objectPtr obj_eng = curr_scene->getObject(obj_eng_name);

                if(obj!=NULL && obj_eng!=NULL){
                    //int grip_id = ui.comboBox_grip->currentIndex();
                    bool prec = ui.radioButton_prec->isChecked();
                    //bool full = ui.radioButton_full->isChecked();
                    switch (arm_sel){
                    case 0: // dual arm
                        // TO DO
                    case 1: // right arm
                         obj->setTargetRightEnabled(true);
                         obj->setTargetLeftEnabled(false);
                        break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        break;
                    }
                    if(planner_id==0){
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get()))));
                    }else{
#if MOVEIT==1
                       curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    success=true;
                }else{qnode.log(QNode::Error,std::string("The movement requires two objects"));}
             }else if(ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled()){
                 // disengage movements
                string obj_name = ui.comboBox_objects->currentText().toStdString();
                string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString();
                objectPtr obj = curr_scene->getObject(obj_name);
                objectPtr obj_eng = curr_scene->getObject(obj_eng_name);
                string pose_name = ui.comboBox_poses->currentText().toStdString();
                posePtr pose = curr_scene->getPose(pose_name);

                if(obj!=NULL && obj_eng!=NULL && pose!=NULL){
                    //int grip_id = ui.comboBox_grip->currentIndex();
                    bool prec = ui.radioButton_prec->isChecked();
                    //bool full = ui.radioButton_full->isChecked();
                    switch (arm_sel){
                    case 0: // dual arm
                        // TO DO
                    case 1: // right arm
                         obj->setTargetRightEnabled(true);
                         obj->setTargetLeftEnabled(false);
                        break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        break;
                    }
                    if(planner_id==0){
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,pose,prec),new Scenario(*(this->curr_scene.get()))));
                    }else{
#if MOVEIT==1
                       curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,pose,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    success=true;
                }else{qnode.log(QNode::Error,std::string("The movement requires two objects and a pose"));}
             }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled()){
                  // reach-to-grasp movements
                 string obj_name = ui.comboBox_objects->currentText().toStdString();
                 objectPtr obj = curr_scene->getObject(obj_name);
                 if(obj!=NULL){
                     //int grip_id = ui.comboBox_grip->currentIndex();
                     bool prec = ui.radioButton_prec->isChecked();
                     //bool full = ui.radioButton_full->isChecked();
                     switch (arm_sel){
                     case 0: // dual arm
                         // TO DO
                     case 1: // right arm
                          obj->setTargetRightEnabled(true);
                          obj->setTargetLeftEnabled(false);
                         break;
                     case 2: // left arm
                         obj->setTargetLeftEnabled(true);
                         obj->setTargetRightEnabled(false);
                         break;
                     }
                     if(planner_id==0){
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,prec),new Scenario(*(this->curr_scene.get()))));
                     }else{
#if MOVEIT==1
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                     }
                     success=true;
                 }else{qnode.log(QNode::Error,std::string("The movement requires an object"));}
             }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled()){
                 // transport movements
                 string obj_name = ui.comboBox_objects->currentText().toStdString();
                 objectPtr obj = curr_scene->getObject(obj_name);
                 string pose_name = ui.comboBox_poses->currentText().toStdString();
                 posePtr pose = curr_scene->getPose(pose_name);
                 if(obj!=NULL || pose!=NULL){
                     //int grip_id = ui.comboBox_grip->currentIndex();
                     bool prec = ui.radioButton_prec->isChecked();
                     if(planner_id==0){
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,pose,prec),new Scenario(*(this->curr_scene.get()))));
                     }else{
#if MOVEIT==1
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,pose,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                     }
                     success=true;
                 }else{qnode.log(QNode::Error,std::string("The movement requires an object and a pose"));}
             }else{
                 // go-park movement and reaching movements
                 std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);
                 for(size_t i=0;i<objects.size();++i){
                     this->curr_scene->getObject(i)->setTargetRightEnabled(false);
                     this->curr_scene->getObject(i)->setTargetLeftEnabled(false);
                 }
                 if(planner_id==0){
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));
                }else{
#if MOVEIT==1
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                 }
                 success=true;
             }
             if(success){
                 qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
                 ui.groupBox_task->setEnabled(true);
                 ui.listWidget_movs->clear();
                 for (int i = 0; i < curr_task->getProblemNumber();i++ ){
                    ui.listWidget_movs->addItem(QString(curr_task->getProblemInfo(i).c_str()));
                 }
                 ui.listWidget_movs->setCurrentRow(ui.listWidget_movs->count()-1);

             }

             // add obstacles in the exception list
             ui.listWidget_obsts_except->clear();
             problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
             std::vector<objectPtr> obsts; prob->getObstacles(obsts);
             for(size_t j=0;j<obsts.size();++j){
                 objectPtr obs = obsts.at(j);
                 string obs_name = obs->getName();
                 ui.listWidget_obsts_except->addItem(QString(obs_name.c_str()));
             }

        }else{
            // dual arm
            bool success=true;
            int mov_id_r = ui.comboBox_mov->currentIndex();
            int mov_id_l = ui.comboBox_mov_left->currentIndex();

            objectPtr obj_right; posePtr pose_right; bool prec_right;
            if (ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled()){
                // right arm engage movements
            }else if(ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled()){
                // right arm disengage movements
            }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled()){
               // right arm reach-to-grasp movement
                string obj_name = ui.comboBox_objects->currentText().toStdString();
                obj_right = curr_scene->getObject(obj_name);
                if(obj_right!=NULL){
                    //int grip_id = ui.comboBox_grip->currentIndex();
                    prec_right = ui.radioButton_prec->isChecked();
                    obj_right->setTargetRightEnabled(true);
                    obj_right->setTargetLeftEnabled(false);
                }else{
                    success = false;
                }
               // if(planner_id==0){
                    // HUMP
                   //curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,prec),new Scenario(*(this->curr_scene.get()))));
                //}else{
                    //TO DO
                //}
            }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled()){
                // right arm transport movements
                string obj_name = ui.comboBox_objects->currentText().toStdString();
                obj_right = curr_scene->getObject(obj_name);
                string pose_name = ui.comboBox_poses->currentText().toStdString();
                pose_right = curr_scene->getPose(pose_name);
                if(obj_right!=NULL || pose_right!=NULL){
                    prec_right = ui.radioButton_prec->isChecked();
                }else{
                    success = false;
                }
            }else{
                // right arm go-park movement and reaching movements
                std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);
                for(size_t i=0;i<objects.size();++i){
                    this->curr_scene->getObject(i)->setTargetRightEnabled(false);
                }
            }
            objectPtr obj_left; posePtr pose_left; bool prec_left;
            if (ui.comboBox_objects_left->isEnabled() && ui.comboBox_objects_eng_left->isEnabled() && ui.groupBox_grip_left->isEnabled() && !ui.comboBox_poses_left->isEnabled()){
                // left arm engage movements
            }else if(ui.comboBox_objects_left->isEnabled() && ui.comboBox_objects_eng_left->isEnabled() && ui.groupBox_grip_left->isEnabled() && ui.comboBox_poses_left->isEnabled()){
                // left arm disengage movements
            }else if(ui.comboBox_objects_left->isEnabled() && ui.groupBox_grip_left->isEnabled() && !ui.comboBox_poses_left->isEnabled()){
               // left arm reach-to-grasp movement
                string obj_name = ui.comboBox_objects_left->currentText().toStdString();
                obj_left = curr_scene->getObject(obj_name);
                if(obj_left!=NULL){
                    //int grip_id = ui.comboBox_grip->currentIndex();
                    prec_left = ui.radioButton_prec_left->isChecked();
                    obj_left->setTargetRightEnabled(false);
                    obj_left->setTargetLeftEnabled(true);
                }else{
                    success = false;
                }
               // if(planner_id==0){
                    // HUMP
                   //curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,prec),new Scenario(*(this->curr_scene.get()))));
                //}else{
                    //TO DO
                //}
            }else if(ui.comboBox_objects_left->isEnabled() && ui.groupBox_grip_left->isEnabled() && ui.comboBox_poses_left->isEnabled()){
                // left arm transport movements
                string obj_name = ui.comboBox_objects_left->currentText().toStdString();
                obj_left = curr_scene->getObject(obj_name);
                string pose_name = ui.comboBox_poses_left->currentText().toStdString();
                pose_left = curr_scene->getPose(pose_name);
                if(obj_left!=NULL || pose_left!=NULL){
                    prec_left = ui.radioButton_prec_left->isChecked();
                }else{
                    success = false;
                }
            }else{
                // left arm go-park movement and reaching movements
                std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);
                for(size_t i=0;i<objects.size();++i){
                    this->curr_scene->getObject(i)->setTargetLeftEnabled(false);
                }
            }

            if(success){
                if(planner_id==0){
                   // HUMP
                   if(obj_right==NULL || obj_left==NULL){
                       curr_task->addProblem(new Problem(planner_id,new Movement(mov_id_r,mov_id_l,0),new Scenario(*(this->curr_scene.get()))));
                   }else{
                       if(pose_right==NULL || pose_left==NULL){
                          curr_task->addProblem(new Problem(planner_id,new Movement(mov_id_r,mov_id_l,0,obj_right,prec_right, obj_left,prec_left),new Scenario(*(this->curr_scene.get()))));
                       }else{
                          curr_task->addProblem(new Problem(planner_id,new Movement(mov_id_r,mov_id_l,0,obj_right,pose_right,prec_right,obj_left,pose_left,prec_left),new Scenario(*(this->curr_scene.get()))));
                       }
                   }
                }
                qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
                ui.groupBox_task->setEnabled(true);
                ui.listWidget_movs->clear();
                for (int i = 0; i < curr_task->getProblemNumber();i++ ){
                   ui.listWidget_movs->addItem(QString(curr_task->getProblemInfo(i).c_str()));
                }
                ui.listWidget_movs->setCurrentRow(ui.listWidget_movs->count()-1);
            }

        } // if arm_sel
        }// if add

} // add movement


void MainWindow::on_pushButton_plan_clicked()
{

    ui.tabWidget_sol->setCurrentIndex(0);    
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    int arm_sel = prob->getMovement()->getArm();

    std::vector<double> move_target;
    std::vector<double> move_final_hand;
    std::vector<double> move_final_arm;
    std::vector<double> move_target_right;
    std::vector<double> move_target_left;
    std::vector<double> move_final_hand_right;
    std::vector<double> move_final_hand_left;
    std::vector<double> move_final_arm_right;
    std::vector<double> move_final_arm_left;

    bool use_final;
    bool use_final_right;
    bool use_final_left;
#if MOVEIT==1
    moveit_planning::moveit_params m_params;
#endif
    bool moveit_plan = false;

    bool solved = false;
 try{
    switch(planner_id){

    case 0: // HUMP
        moveit_plan = false;
        if(arm_sel!=0)
        { // single-arm movement

            mTolHumpdlg->setInfo(prob->getInfoLine());
            // --- Tolerances for the final posture selection ---- //
            tols.tolTarPos = mTolHumpdlg->getTolTarPos(); // target position tolerances
            tols.tolTarOr = mTolHumpdlg->getTolTarOr(); // target orientation tolerances
            mTolHumpdlg->getTolsArm(tols.tolsArm);// tolerances of the arm : radius in [mm]
            mTolHumpdlg->getTolsHand(tols.tolsHand);// tolerances of the hand: radius in [mm]
            tols.target_avoidance = mTolHumpdlg->getTargetAvoidance();// target avoidance
            tols.obstacle_avoidance = mTolHumpdlg->getObstacleAvoidance(); //obstacle avoidance
            mTolHumpdlg->getLambda(tols.lambda_final); // joint expense factors
            mTolHumpdlg->getLambda(tols.lambda_bounce); // joint expense factors
            // --- Tolerances for the bounce posture selection ---- //
            tols.w_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getWMax()*M_PI/180)); // max joint velocity
            tols.alpha_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration
            mTolHumpdlg->getInitVel(tols.bounds.vel_0); // initial velocity
            mTolHumpdlg->getFinalVel(tols.bounds.vel_f); // final velocity
            mTolHumpdlg->getInitAcc(tols.bounds.acc_0); // initial acceleration
            mTolHumpdlg->getFinalAcc(tols.bounds.acc_f); // final acceleration
            //mTolHumpdlg->getVelApproach(tols.vel_approach); // velocity approach
            //mTolHumpdlg->getAccApproach(tols.acc_approach); // acceleration approach
            // tolerances for the obstacles
            mTolHumpdlg->getTolsObstacles(tols.final_tolsObstacles); // final posture tols
            tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
            tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
            mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));
            // tolerances for the target
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
            tols.singleArm_tolsTarget.at(1) = tols.singleArm_tolsTarget.at(0)/100;
            tols.singleArm_tolsTarget.at(2) = 0*tols.singleArm_tolsTarget.at(0);
            //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(1));
            //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(2));
            // pick / place settings
            tols.mov_specs.approach = mTolHumpdlg->getApproach();
            tols.mov_specs.retreat = mTolHumpdlg->getRetreat();
            mTolHumpdlg->getPreGraspApproach(tols.mov_specs.pre_grasp_approach); // pick approach
            mTolHumpdlg->getPostGraspRetreat(tols.mov_specs.post_grasp_retreat); // pick retreat
            mTolHumpdlg->getPrePlaceApproach(tols.mov_specs.pre_place_approach); // place approach
            mTolHumpdlg->getPostPlaceRetreat(tols.mov_specs.post_place_retreat); // place retreat
            tols.mov_specs.rand_init = mTolHumpdlg->getRandInit(); // random initialization for "plan" stages
            tols.mov_specs.coll = mTolHumpdlg->getColl(); // collisions option
            tols.coll_body = mTolHumpdlg->getCollBody(); // collisions with the body
            tols.mov_specs.straight_line = mTolHumpdlg->get_straight_line(); // hand straight line trajectory
            tols.mov_specs.w_red_app_max = mTolHumpdlg->getW_red_app(); // set the max velocity reduction when approaching
            tols.mov_specs.w_red_ret_max = mTolHumpdlg->getW_red_ret(); // set the max velocity reduction when retreating
            // move settings
            mTolHumpdlg->getTargetMove(move_target);
            mTolHumpdlg->getFinalHand(move_final_hand);
            mTolHumpdlg->getFinalArm(move_final_arm);
            use_final = mTolHumpdlg->get_use_final_posture();
            prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
            tols.mov_specs.use_move_plane = mTolHumpdlg->get_add_plane();
            mTolHumpdlg->getPlaneParameters(tols.mov_specs.plane_params);
            // warm start option
            if(mTolHumpdlg->getWarmStartOption()){
                tols.mov_specs.warm_start = true;
                // plan
                if(mTolHumpdlg->getWarmStartPlanOption()){
                    HUMotion::warm_start_params final_plan;
                    final_plan.valid = true;
                    final_plan.description = "plan";
                    int warm_steps;
                    mTolHumpdlg->getPlanData(final_plan.x,final_plan.zL,final_plan.zU,final_plan.dual_vars,warm_steps);
                    tols.mov_specs.final_warm_start_params.push_back(final_plan);
                    tols.mov_specs.warm_n_steps = warm_steps;
                }
                // approach
                if(mTolHumpdlg->getWarmStartApproachOption()){
                    HUMotion::warm_start_params final_approach;
                    final_approach.valid = true;
                    final_approach.description = "approach";
                    mTolHumpdlg->getApproachData(final_approach.x,final_approach.zL,final_approach.zU,final_approach.dual_vars);
                    tols.mov_specs.final_warm_start_params.push_back(final_approach);
                }
                // retreat
                if(mTolHumpdlg->getWarmStartRetreatOption()){
                    HUMotion::warm_start_params final_retreat;
                    final_retreat.valid = true;
                    final_retreat.description = "retreat";
                    mTolHumpdlg->getRetreatData(final_retreat.x,final_retreat.zL,final_retreat.zU,final_retreat.dual_vars);
                    tols.mov_specs.final_warm_start_params.push_back(final_retreat);
                }
                // bounce
                if(mTolHumpdlg->getWarmStartBounceOption()){
                    HUMotion::warm_start_params bounce;
                    bounce.valid = true;
                    bounce.description = "bounce";
                    mTolHumpdlg->getBounceData(bounce.x,bounce.zL,bounce.zU,bounce.dual_vars);
                    tols.mov_specs.bounce_warm_start_params = bounce;
                }
            }else{
                tols.mov_specs.warm_start = false;
            }

            h_results = prob->solve(tols); // plan the movement

            ui.pushButton_plan->setCheckable(false);
            if(h_results!=nullptr){
                if(h_results->status==0){
                    qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
                    this->curr_mov = prob->getMovement();
                    this->timesteps_mov.clear();
                    this->jointsPosition_mov.clear(); this->jointsPosition_mov = h_results->trajectory_stages;
                    this->jointsVelocity_mov.clear(); this->jointsVelocity_mov = h_results->velocity_stages;
                    this->jointsAcceleration_mov.clear(); this->jointsAcceleration_mov = h_results->acceleration_stages;
                    this->traj_descr_mov.clear(); this->traj_descr_mov = h_results->trajectory_descriptions;
                    this->final_warm_start_res_mov = h_results->final_warm_start_res;
                    this->bounce_warm_start_res_mov = h_results->bounce_warm_start_res;
                    std::vector<double> timesteps_stage_aux;
                    for(size_t i=0; i<h_results->trajectory_stages.size();++i){
                        timesteps_stage_aux.clear();
                        double t_stage = h_results->time_steps.at(i);
                        MatrixXd traj_stage = h_results->trajectory_stages.at(i);
                        if(i==0){
                            // plan stage
                            this->warm_n_steps_mov = traj_stage.rows()-1;
                        }
                        for(int j=0;j<traj_stage.rows();++j){
                            if(j==traj_stage.rows()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(t_stage);
                            }
                        }
                        this->timesteps_mov.push_back(timesteps_stage_aux);
                    }
                    this->moveit_mov = false;
                    solved=true;
                }else{
                    ui.tableWidget_sol_mov->clear();
                    qnode.log(QNode::Error,std::string("The planning has failed: ")+h_results->status_msg);
                }
            }else{
                ui.tableWidget_sol_mov->clear();
                qnode.log(QNode::Error,std::string("The planning has failed: unknown status"));
            }

            // make a copy of the human-like parameters for controlling
            this->tols_ctrl = this->tols;
            this->tols_ctrl.mov_specs.warm_start = true;
            this->tols_ctrl.mov_specs.warm_n_steps = this->warm_n_steps_mov;
            this->tols_ctrl.mov_specs.final_warm_start_params = this->final_warm_start_res_mov;
            this->tols_ctrl.mov_specs.bounce_warm_start_params = this->bounce_warm_start_res_mov;

        }else{
            // dual-arm movement

            mTolHumpDualdlg->setInfo(prob->getInfoLine());
            // --- Tolerances for the final posture selection ---- //
            dual_tols.tolTarPos_right = mTolHumpDualdlg->getTolTarPosRight(); // right target position tolerances
            dual_tols.tolTarPos_left = mTolHumpDualdlg->getTolTarPosLeft(); // left target position tolerances
            dual_tols.tolTarOr_right = mTolHumpDualdlg->getTolTarOrRight(); // right target orientation tolerances
            dual_tols.tolTarOr_left = mTolHumpDualdlg->getTolTarOrLeft(); // left target orientation tolerances
            mTolHumpDualdlg->getTolsArmRight(dual_tols.tolsArm_right);// tolerances of the right arm : radius in [mm]
            mTolHumpDualdlg->getTolsArmLeft(dual_tols.tolsArm_left);// tolerances of the left arm : radius in [mm]
            mTolHumpDualdlg->getTolsHandRight(dual_tols.tolsHand_right);// tolerances of the right hand: radius in [mm]
            mTolHumpDualdlg->getTolsHandLeft(dual_tols.tolsHand_left);// tolerances of the left hand: radius in [mm]
            dual_tols.target_avoidance = mTolHumpDualdlg->getTargetAvoidance();// target avoidance
            dual_tols.obstacle_avoidance = mTolHumpDualdlg->getObstacleAvoidance(); //obstacle avoidance
            mTolHumpDualdlg->getLambdaRight(dual_tols.lambda_final_right); // joint expense factors (right)
            mTolHumpDualdlg->getLambdaLeft(dual_tols.lambda_final_left); // joint expense factors (left)
            mTolHumpDualdlg->getLambdaRight(dual_tols.lambda_bounce_right); // joint expense factors (right)
            mTolHumpDualdlg->getLambdaLeft(dual_tols.lambda_bounce_left); // joint expense factors (left)
            // --- Tolerances for the bounce posture selection ---- //
            dual_tols.w_max = std::vector<double>(dual_tols.lambda_final_right.size()*2,(mTolHumpDualdlg->getWMax()*M_PI/180)); // max joint velocity
            dual_tols.alpha_max = std::vector<double>(dual_tols.lambda_final_right.size()*2,(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration
            mTolHumpDualdlg->getInitVelRight(dual_tols.bounds_right.vel_0); // initial velocity (right)
            mTolHumpDualdlg->getInitVelLeft(dual_tols.bounds_left.vel_0); // initial velocity (left)
            mTolHumpDualdlg->getFinalVelRight(dual_tols.bounds_right.vel_f); // final velocity (right)
            mTolHumpDualdlg->getFinalVelLeft(dual_tols.bounds_left.vel_f); // final velocity (left)
            mTolHumpDualdlg->getInitAccRight(dual_tols.bounds_right.acc_0); // initial acceleration (right)
            mTolHumpDualdlg->getInitAccLeft(dual_tols.bounds_left.acc_0); // initial acceleration (left)
            mTolHumpDualdlg->getFinalAccRight(dual_tols.bounds_right.acc_f); // final acceleration (right)
            mTolHumpDualdlg->getFinalAccLeft(dual_tols.bounds_left.acc_f); // final acceleration (left)
            // tolerances for the obstacles
            mTolHumpDualdlg->getTolsObstaclesRight(dual_tols.final_tolsObstacles_right); // final posture tols (right)
            mTolHumpDualdlg->getTolsObstaclesLeft(dual_tols.final_tolsObstacles_left); // final posture tols (left)
            dual_tols.singleArm_tolsObstacles_right.push_back(MatrixXd::Constant(3,6,1)); // bounce posture obstacles tols (right)
            dual_tols.singleArm_tolsObstacles_right.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpDualdlg->getTolsObstaclesRight(dual_tols.singleArm_tolsObstacles_right.at(0));
            mTolHumpDualdlg->getTolsObstaclesRight(dual_tols.singleArm_tolsObstacles_right.at(1));
            dual_tols.singleArm_tolsObstacles_left.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols (left)
            dual_tols.singleArm_tolsObstacles_left.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpDualdlg->getTolsObstaclesLeft(dual_tols.singleArm_tolsObstacles_left.at(0));
            mTolHumpDualdlg->getTolsObstaclesLeft(dual_tols.singleArm_tolsObstacles_left.at(1));
            // tolerances for the target
            dual_tols.singleArm_tolsTarget_right.push_back(MatrixXd::Constant(3,6,1)); // bounce posture target tols (right)
            dual_tols.singleArm_tolsTarget_right.push_back(MatrixXd::Constant(3,6,1));
            dual_tols.singleArm_tolsTarget_right.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpDualdlg->getTolsTargetRight(dual_tols.singleArm_tolsTarget_right.at(0));
            dual_tols.singleArm_tolsTarget_right.at(1) = dual_tols.singleArm_tolsTarget_right.at(0)/100;
            dual_tols.singleArm_tolsTarget_right.at(2) = 0*dual_tols.singleArm_tolsTarget_right.at(0);
            dual_tols.singleArm_tolsTarget_left.push_back(MatrixXd::Constant(3,6,1)); // bounce posture target tols (left)
            dual_tols.singleArm_tolsTarget_left.push_back(MatrixXd::Constant(3,6,1));
            dual_tols.singleArm_tolsTarget_left.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpDualdlg->getTolsTargetLeft(dual_tols.singleArm_tolsTarget_left.at(0));
            dual_tols.singleArm_tolsTarget_left.at(1) = dual_tols.singleArm_tolsTarget_left.at(0)/100;
            dual_tols.singleArm_tolsTarget_left.at(2) = 0*dual_tols.singleArm_tolsTarget_left.at(0);
            // pick / place settings
            dual_tols.mov_specs_right.approach = mTolHumpDualdlg->getApproach();
            dual_tols.mov_specs_left.approach = mTolHumpDualdlg->getApproach();
            dual_tols.mov_specs_right.retreat = mTolHumpDualdlg->getRetreat();
            dual_tols.mov_specs_left.retreat = mTolHumpDualdlg->getRetreat();
            mTolHumpDualdlg->getPreGraspApproachRight(dual_tols.mov_specs_right.pre_grasp_approach); // pick approach (right)
            mTolHumpDualdlg->getPreGraspApproachLeft(dual_tols.mov_specs_left.pre_grasp_approach); // pick approach (left)
            mTolHumpDualdlg->getPostGraspRetreatRight(dual_tols.mov_specs_right.post_grasp_retreat); // pick retreat (right)
            mTolHumpDualdlg->getPostGraspRetreatLeft(dual_tols.mov_specs_left.post_grasp_retreat); // pick retreat (left)
            mTolHumpDualdlg->getPrePlaceApproachRight(dual_tols.mov_specs_right.pre_place_approach); // place approach (right)
            mTolHumpDualdlg->getPrePlaceApproachLeft(dual_tols.mov_specs_left.pre_place_approach); // place approach (left)
            mTolHumpDualdlg->getPostPlaceRetreatRight(dual_tols.mov_specs_right.post_place_retreat); // place retreat (right)
            mTolHumpDualdlg->getPostPlaceRetreatLeft(dual_tols.mov_specs_left.post_place_retreat); // place retreat (left)
            dual_tols.mov_specs_right.rand_init = mTolHumpDualdlg->getRandInit(); // random initialization for "plan" stages (right)
            dual_tols.mov_specs_left.rand_init = mTolHumpDualdlg->getRandInit(); // random initialization for "plan" stages (left)
            dual_tols.mov_specs_right.coll = mTolHumpDualdlg->getColl(); // collisions option (right)
            dual_tols.mov_specs_left.coll = mTolHumpDualdlg->getColl(); // collisions option (left)
            dual_tols.coll_body = mTolHumpDualdlg->getCollBody(); // collisions with the body
            dual_tols.coll_arms = mTolHumpDualdlg->getCollArms(); // collisions between arms
            dual_tols.mov_specs_right.straight_line = mTolHumpDualdlg->get_straight_line_right(); // right hand straight line trajectory
            dual_tols.mov_specs_left.straight_line = mTolHumpDualdlg->get_straight_line_left(); // left hand straight line trajectory
            dual_tols.mov_specs_right.w_red_app_max = mTolHumpDualdlg->getW_red_app_right(); // set the max velocity reduction when approaching (right)
            dual_tols.mov_specs_left.w_red_app_max = mTolHumpDualdlg->getW_red_app_left(); // set the max velocity reduction when approaching (left)
            dual_tols.mov_specs_right.w_red_ret_max = mTolHumpDualdlg->getW_red_ret_right(); // set the max velocity reduction when retreating (right)
            dual_tols.mov_specs_left.w_red_ret_max = mTolHumpDualdlg->getW_red_ret_left(); // set the max velocity reduction when retreating (left)
            // move settings
            mTolHumpDualdlg->getTargetMoveRight(move_target_right);
            mTolHumpDualdlg->getTargetMoveLeft(move_target_left);
            mTolHumpDualdlg->getFinalHandRight(move_final_hand_right);
            mTolHumpDualdlg->getFinalHandLeft(move_final_hand_left);
            mTolHumpDualdlg->getFinalArmRight(move_final_arm_right);
            mTolHumpDualdlg->getFinalArmLeft(move_final_arm_left);
            use_final_right = mTolHumpDualdlg->get_use_final_posture_right();
            use_final_left = mTolHumpDualdlg->get_use_final_posture_left();
            prob->setMoveSettings(move_target_right,move_target_left,
                                  move_final_hand_right,move_final_hand_left,
                                  move_final_arm_right,move_final_arm_left,
                                  use_final_right,use_final_left);

            h_dual_results = prob->solve(dual_tols); // plan the movement
            ui.pushButton_plan->setCheckable(false);
            if(h_dual_results!=nullptr){
                if(h_dual_results->status==0){
                    qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
                    this->curr_mov = prob->getMovement();
                    this->timesteps_mov.clear();
                    this->jointsPosition_mov.clear(); this->jointsPosition_mov = h_dual_results->trajectory_stages;
                    this->jointsVelocity_mov.clear(); this->jointsVelocity_mov = h_dual_results->velocity_stages;
                    this->jointsAcceleration_mov.clear(); this->jointsAcceleration_mov = h_dual_results->acceleration_stages;
                    this->traj_descr_mov.clear(); this->traj_descr_mov = h_dual_results->trajectory_descriptions;
                    std::vector<double> timesteps_stage_aux;
                    for(size_t i=0; i < h_dual_results->trajectory_stages.size();++i){
                        timesteps_stage_aux.clear();
                        double t_stage = h_dual_results->time_steps.at(i);
                        MatrixXd traj_stage = h_dual_results->trajectory_stages.at(i);
                        for(int j=0;j<traj_stage.rows();++j){
                            if(j==traj_stage.rows()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(t_stage);
                            }
                        }
                        this->timesteps_mov.push_back(timesteps_stage_aux);
                    }
                    this->moveit_mov = false;
                    solved=true;
                }else{
                    ui.tableWidget_sol_mov->clear();
                    qnode.log(QNode::Error,std::string("The planning has failed: ")+h_dual_results->status_msg);
                }
            }else{
                ui.tableWidget_sol_mov->clear();
                qnode.log(QNode::Error,std::string("The planning has failed: unknown status"));
            }
        }
        break;
    case 1: // RRT
#if MOVEIT==1
        moveit_plan = true;
        mRRTdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTdlg->getConfig();
        // pick/place settings
        m_params.approach = mRRTdlg->getApproach();
        m_params.retreat = mRRTdlg->getRetreat();
        mRRTdlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mRRTdlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mRRTdlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mRRTdlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat
        // move settings
        mRRTdlg->getTargetMove(move_target);
        mRRTdlg->getFinalHand(move_final_hand);
        mRRTdlg->getFinalArm(move_final_arm);
        use_final = mRRTdlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        m_params.use_move_plane = mRRTdlg->get_add_plane();
        mRRTdlg->getPlaneParameters(m_params.plane_params,m_params.plane_point1,m_params.plane_point2,m_params.plane_point3);

        m_results = prob->solve(m_params); // plan the movement
        ui.pushButton_plan->setCheckable(false);
#endif
        break;

    case 2: // RRT Connect
#if MOVEIT==1
        moveit_plan = true;
        mRRTConnectdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTConnectdlg->getConfig();
        // pick/place settings
        m_params.approach = mRRTConnectdlg->getApproach();
        m_params.retreat = mRRTConnectdlg->getRetreat();
        mRRTConnectdlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mRRTConnectdlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mRRTConnectdlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mRRTConnectdlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat
        // move settings
        mRRTConnectdlg->getTargetMove(move_target);
        mRRTConnectdlg->getFinalHand(move_final_hand);
        mRRTConnectdlg->getFinalArm(move_final_arm);
        use_final = mRRTConnectdlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        m_params.use_move_plane = mRRTConnectdlg->get_add_plane();
        mRRTConnectdlg->getPlaneParameters(m_params.plane_params,m_params.plane_point1,m_params.plane_point2,m_params.plane_point3);

        m_results = prob->solve(m_params); // plan the movement
        ui.pushButton_plan->setCheckable(false);
#endif

        break;

    case 3: // RRT star
#if MOVEIT==1
        moveit_plan = true;
        mRRTstardlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTstardlg->getConfig();
        // pick/place settings
        m_params.approach = mRRTstardlg->getApproach();
        m_params.retreat = mRRTstardlg->getRetreat();
        mRRTstardlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mRRTstardlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mRRTstardlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mRRTstardlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat
        // move settings
        mRRTstardlg->getTargetMove(move_target);
        mRRTstardlg->getFinalHand(move_final_hand);
        mRRTstardlg->getFinalArm(move_final_arm);
        use_final = mRRTstardlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        m_params.use_move_plane = mRRTstardlg->get_add_plane();
        mRRTstardlg->getPlaneParameters(m_params.plane_params,m_params.plane_point1,m_params.plane_point2,m_params.plane_point3);

        m_results = prob->solve(m_params); // plan the movement
        ui.pushButton_plan->setCheckable(false);
#endif
        break;

    case 4: // PRM
#if MOVEIT==1
        moveit_plan = true;
        mPRMdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mPRMdlg->getConfig();
        // pick/place settings
        m_params.approach = mPRMdlg->getApproach();
        m_params.retreat = mPRMdlg->getRetreat();
        mPRMdlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mPRMdlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mPRMdlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mPRMdlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat
        // move settings
        mPRMdlg->getTargetMove(move_target);
        mPRMdlg->getFinalHand(move_final_hand);
        mPRMdlg->getFinalArm(move_final_arm);
        use_final = mPRMdlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        m_params.use_move_plane = mPRMdlg->get_add_plane();
        mPRMdlg->getPlaneParameters(m_params.plane_params,m_params.plane_point1,m_params.plane_point2,m_params.plane_point3);

        m_results = prob->solve(m_params); // plan the movement
        ui.pushButton_plan->setCheckable(false);
#endif
        break;

    case 5: // PRM star
#if MOVEIT==1
        moveit_plan = true;
        mPRMstardlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mPRMstardlg->getConfig();
        // pick/place settings
        m_params.approach = mPRMstardlg->getApproach();
        m_params.retreat = mPRMstardlg->getRetreat();
        mPRMstardlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mPRMstardlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mPRMstardlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mPRMstardlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat
        // move settings
        mPRMstardlg->getTargetMove(move_target);
        mPRMstardlg->getFinalHand(move_final_hand);
        mPRMstardlg->getFinalArm(move_final_arm);
        use_final = mPRMstardlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        m_params.use_move_plane = mPRMstardlg->get_add_plane();
        mPRMstardlg->getPlaneParameters(m_params.plane_params,m_params.plane_point1,m_params.plane_point2,m_params.plane_point3);

        m_results = prob->solve(m_params); // plan the movement
        ui.pushButton_plan->setCheckable(false);
#endif
        break;

    } // switch planners

}catch (const std::string message){qnode.log(QNode::Error,std::string("Plan failure: ")+message);
}catch(const std::exception exc){qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());}

#if MOVEIT==1
    if(moveit_plan){
        if(m_results!=nullptr){
            if(m_results->status==1){
                qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
                this->curr_mov = prob->getMovement();
                MatrixXd jointsPosition_stage_plan; MatrixXd jointsVelocity_stage_plan; MatrixXd jointsAcceleration_stage_plan;
                MatrixXd jointsPosition_stage_approach; MatrixXd jointsVelocity_stage_approach; MatrixXd jointsAcceleration_stage_approach;
                MatrixXd jointsPosition_stage_retreat; MatrixXd jointsVelocity_stage_retreat; MatrixXd jointsAcceleration_stage_retreat;
                std::vector<double> timesteps_stage_plan; std::vector<double> timesteps_stage_approach; std::vector<double> timesteps_stage_retreat;

                // get the initial posture of the fingers
                std::vector<double> init_hand_pos = std::vector<double>(JOINTS_HAND,0);
                std::vector<double> init_hand_vel = std::vector<double>(JOINTS_HAND,0);
                std::vector<double> init_hand_acc = std::vector<double>(JOINTS_HAND,0);
                int arm_code = this->curr_mov->getArm();
                switch(arm_code){
                case 0:// both arm
                    // TO DO
                    break;
                case 1:// right arm
                    this->curr_scene->getHumanoid()->getRightHandPosture(init_hand_pos);
                    break;
                case 2: // left arm
                    this->curr_scene->getHumanoid()->getLeftHandPosture(init_hand_pos);
                    break;
                }

                // positions of the fingers in the Barrett Hand
                int fing_base = 0; int fing_1 = 1; int fing_2 = 4; int fing_3 = 6;
                bool move = false;
                if(m_results->trajectory_stages.size()==1)
                    move=true;

                //double time_from_start = 0.0;
                for(size_t i=0; i<m_results->trajectory_stages.size(); ++i){
                    std::string traj_descr = m_results->trajectory_descriptions.at(i);
                    moveit_msgs::RobotTrajectory rob_traj;
                    vector<trajectory_msgs::JointTrajectoryPoint> points;
                    std::vector<double> timesteps_stage_aux;
                    MatrixXd jointsPosition_stage_aux; MatrixXd jointsVelocity_stage_aux; MatrixXd jointsAcceleration_stage_aux;
                    if(strcmp(traj_descr.c_str(),"plan")==0){
                        timesteps_stage_plan.clear(); timesteps_stage_aux.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<traj_pnt.positions.size()+JOINTS_HAND;++k){
                                if(k<traj_pnt.positions.size()){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(k);
                                }else{
                                    jointsPosition_stage_aux(j,k) = init_hand_pos.at(k-JOINTS_ARM);
                                    jointsVelocity_stage_aux(j,k) = init_hand_vel.at(k-JOINTS_ARM);
                                    jointsAcceleration_stage_aux(j,k) = init_hand_acc.at(k-JOINTS_ARM);
                                }
                            }
                        }// points
                        //time_from_start += points.at(points.size()-1).time_from_start.toSec();
                        timesteps_stage_plan = timesteps_stage_aux;
                        jointsPosition_stage_plan = jointsPosition_stage_aux;
                        jointsVelocity_stage_plan = jointsVelocity_stage_aux;
                        jointsAcceleration_stage_plan = jointsAcceleration_stage_aux;
                    }else if(strcmp(traj_descr.c_str(),"pre_grasp")==0){
                        timesteps_stage_aux.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<JOINTS_ARM+JOINTS_HAND;++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_plan(jointsPosition_stage_plan.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_plan(jointsVelocity_stage_plan.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_plan(jointsAcceleration_stage_plan.rows()-1,k);
                                }else if(k==JOINTS_ARM+fing_base){
                                    //jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_base);
                                    //jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_base);
                                    //jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_base);
                                    jointsPosition_stage_aux(j,k) = 0.0;
                                    jointsVelocity_stage_aux(j,k) = 0.0;
                                    jointsAcceleration_stage_aux(j,k) = 0.0;
                                }else if(k==JOINTS_ARM+1){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_1);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_1);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_1);
                                }else if(k==JOINTS_ARM+2){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_2);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_2);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_2);
                                }else if(k==JOINTS_ARM+3){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_3);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_3);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_3);
                                }
                            }
                        }// points
                        timesteps_stage_plan.insert(timesteps_stage_plan.end(), timesteps_stage_aux.begin(), timesteps_stage_aux.end());
                        MatrixXd tmp_pos(jointsPosition_stage_plan.rows()+jointsPosition_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_pos << jointsPosition_stage_plan,
                                   jointsPosition_stage_aux;
                        jointsPosition_stage_plan = tmp_pos;
                        MatrixXd tmp_vel(jointsVelocity_stage_plan.rows()+jointsVelocity_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_vel << jointsVelocity_stage_plan,
                                   jointsVelocity_stage_aux;
                        jointsVelocity_stage_plan = tmp_vel;
                        MatrixXd tmp_acc(jointsAcceleration_stage_plan.rows()+jointsAcceleration_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_acc << jointsAcceleration_stage_plan,
                                   jointsAcceleration_stage_aux;
                        jointsAcceleration_stage_plan = tmp_acc;
                    }else if(strcmp(traj_descr.c_str(),"approach")==0){
                        timesteps_stage_approach.clear(); timesteps_stage_aux.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<traj_pnt.positions.size()+JOINTS_HAND;++k){
                                if(k<traj_pnt.positions.size()){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(k);
                                }else{
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_plan(jointsPosition_stage_plan.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_plan(jointsVelocity_stage_plan.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_plan(jointsAcceleration_stage_plan.rows()-1,k);
                                }
                            }
                        }// points
                        //time_from_start += points.at(points.size()-1).time_from_start.toSec();
                        timesteps_stage_approach = timesteps_stage_aux;
                        jointsPosition_stage_approach = jointsPosition_stage_aux;
                        jointsVelocity_stage_approach = jointsVelocity_stage_aux;
                        jointsAcceleration_stage_approach = jointsAcceleration_stage_aux;

                    }else if(strcmp(traj_descr.c_str(),"grasp")==0){
                        timesteps_stage_aux.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<JOINTS_ARM+JOINTS_HAND;++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_approach(jointsPosition_stage_approach.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_approach(jointsVelocity_stage_approach.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_approach(jointsAcceleration_stage_approach.rows()-1,k);
                                }else if(k==JOINTS_ARM+fing_base){
                                    //jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_base);
                                    //jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_base);
                                    //jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_base);
                                    jointsPosition_stage_aux(j,k) = 0.0;
                                    jointsVelocity_stage_aux(j,k) = 0.0;
                                    jointsAcceleration_stage_aux(j,k) = 0.0;
                                }else if(k==JOINTS_ARM+1){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_1);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_1);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_1);
                                }else if(k==JOINTS_ARM+2){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_2);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_2);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_2);
                                }else if(k==JOINTS_ARM+3){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_3);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_3);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_3);
                                }
                            }
                        }// points
                        timesteps_stage_approach.insert(timesteps_stage_approach.end(), timesteps_stage_aux.begin(), timesteps_stage_aux.end());
                        MatrixXd tmp_pos(jointsPosition_stage_approach.rows()+jointsPosition_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_pos << jointsPosition_stage_approach,
                                   jointsPosition_stage_aux;
                        jointsPosition_stage_approach = tmp_pos;
                        MatrixXd tmp_vel(jointsVelocity_stage_approach.rows()+jointsVelocity_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_vel << jointsVelocity_stage_approach,
                                   jointsVelocity_stage_aux;
                        jointsVelocity_stage_approach = tmp_vel;
                        MatrixXd tmp_acc(jointsAcceleration_stage_approach.rows()+jointsAcceleration_stage_aux.rows(),JOINTS_ARM+JOINTS_HAND);
                        tmp_acc << jointsAcceleration_stage_approach,
                                   jointsAcceleration_stage_aux;
                        jointsAcceleration_stage_approach = tmp_acc;
                    }else if(strcmp(traj_descr.c_str(),"retreat")==0){
                        timesteps_stage_aux.clear(); timesteps_stage_retreat.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<traj_pnt.positions.size()+JOINTS_HAND;++k){
                                if(k<traj_pnt.positions.size()){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(k);
                                }else{
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_approach(jointsPosition_stage_approach.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_approach(jointsVelocity_stage_approach.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_approach(jointsAcceleration_stage_approach.rows()-1,k);
                                }
                            }
                        }// points
                        //time_from_start += points.at(points.size()-1).time_from_start.toSec();
                        timesteps_stage_retreat = timesteps_stage_aux;
                        jointsPosition_stage_retreat = jointsPosition_stage_aux;
                        jointsVelocity_stage_retreat = jointsVelocity_stage_aux;
                        jointsAcceleration_stage_retreat = jointsAcceleration_stage_aux;

                    }else if(strcmp(traj_descr.c_str(),"target")==0){
                        timesteps_stage_plan.clear(); timesteps_stage_aux.clear();
                        rob_traj = m_results->trajectory_stages.at(i);
                        points = rob_traj.joint_trajectory.points;
                        jointsPosition_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsVelocity_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        jointsAcceleration_stage_aux = MatrixXd::Zero(points.size(),JOINTS_ARM+JOINTS_HAND);
                        for(size_t j=0; j<points.size();++j){
                            if(j==points.size()-1){
                                timesteps_stage_aux.push_back(0.0);
                            }else{
                                timesteps_stage_aux.push_back(points.at(j+1).time_from_start.toSec()-points.at(j).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<traj_pnt.positions.size();++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(k);
                                }else if(k==JOINTS_ARM+fing_base){
                                    //jointsPosition_stage_aux(j,JOINTS_ARM+fing_base) = traj_pnt.positions.at(k);
                                    //jointsVelocity_stage_aux(j,JOINTS_ARM+fing_base) = traj_pnt.velocities.at(k);
                                    //jointsAcceleration_stage_aux(j,JOINTS_ARM+fing_base) = traj_pnt.accelerations.at(k);
                                    jointsPosition_stage_aux(j,JOINTS_ARM+fing_base) = 0.0;
                                    jointsVelocity_stage_aux(j,JOINTS_ARM+fing_base) = 0.0;
                                    jointsAcceleration_stage_aux(j,JOINTS_ARM+fing_base) = 0.0;
                                }else if(k==JOINTS_ARM+fing_1){
                                    jointsPosition_stage_aux(j,JOINTS_ARM+1) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,JOINTS_ARM+1) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,JOINTS_ARM+1) = traj_pnt.accelerations.at(k);
                                }else if(k==JOINTS_ARM+fing_2){
                                    jointsPosition_stage_aux(j,JOINTS_ARM+2) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,JOINTS_ARM+2) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,JOINTS_ARM+2) = traj_pnt.accelerations.at(k);
                                }else if(k==JOINTS_ARM+fing_3){
                                    jointsPosition_stage_aux(j,JOINTS_ARM+3) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,JOINTS_ARM+3) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,JOINTS_ARM+3) = traj_pnt.accelerations.at(k);
                                }
                            }
                        }// points
                        //time_from_start += points.at(points.size()-1).time_from_start.toSec();
                        timesteps_stage_plan = timesteps_stage_aux;
                        jointsPosition_stage_plan = jointsPosition_stage_aux;
                        jointsVelocity_stage_plan = jointsVelocity_stage_aux;
                        jointsAcceleration_stage_plan = jointsAcceleration_stage_aux;

                    }

                } // for loop stages
                if(move){
                    //positions
                    this->jointsPosition_mov.clear();
                    this->jointsPosition_mov.push_back(jointsPosition_stage_plan);
                    //velocities
                    this->jointsVelocity_mov.clear();
                    this->jointsVelocity_mov.push_back(jointsVelocity_stage_plan);
                    //accelerations
                    this->jointsAcceleration_mov.clear();
                    this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_plan);
                    //time steps
                    this->timesteps_mov.clear();
                    this->timesteps_mov.push_back(timesteps_stage_plan);
                    // descriptions
                    this->traj_descr_mov.clear();
                    this->traj_descr_mov.push_back("plan");
                }else{
                    this->traj_descr_mov.clear();
                    //positions
                    this->jointsPosition_mov.clear();
                    this->jointsPosition_mov.push_back(jointsPosition_stage_plan);
                    this->traj_descr_mov.push_back("plan");
                    if(jointsPosition_stage_approach.rows()!=0){
                        this->jointsPosition_mov.push_back(jointsPosition_stage_approach);
                        this->traj_descr_mov.push_back("approach");
                    }
                    if(jointsPosition_stage_retreat.rows()!=0){
                        this->jointsPosition_mov.push_back(jointsPosition_stage_retreat);
                        this->traj_descr_mov.push_back("retreat");
                    }
                    //velocities
                    this->jointsVelocity_mov.clear();
                    this->jointsVelocity_mov.push_back(jointsVelocity_stage_plan);
                    if(jointsVelocity_stage_approach.rows()!=0)
                        this->jointsVelocity_mov.push_back(jointsVelocity_stage_approach);
                    if(jointsVelocity_stage_retreat.rows()!=0)
                        this->jointsVelocity_mov.push_back(jointsVelocity_stage_retreat);
                    //accelerations
                    this->jointsAcceleration_mov.clear();
                    this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_plan);
                    if(jointsAcceleration_stage_approach.rows()!=0)
                        this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_approach);
                    if(jointsAcceleration_stage_retreat.rows()!=0)
                        this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_retreat);
                    //time steps
                    this->timesteps_mov.clear();
                    this->timesteps_mov.push_back(timesteps_stage_plan);
                    if(!timesteps_stage_approach.empty())
                        this->timesteps_mov.push_back(timesteps_stage_approach);
                    if(!timesteps_stage_retreat.empty())
                        this->timesteps_mov.push_back(timesteps_stage_retreat);
                }
                this->moveit_mov = true;
                solved=true;

            }else{
                ui.tableWidget_sol_mov->clear();
                qnode.log(QNode::Error,std::string("The planning has failed: ")+m_results->status_msg);
            }
        }else{
            ui.tableWidget_sol_mov->clear();
            qnode.log(QNode::Error,std::string("The planning has failed: unknown status"));
        }
    }
#endif


// --- RESULTS --- //
if(solved){
    // time taken to solve the problem
    this->prob_time_mov = prob->getTime();
    ui.label_solving_time->setText(QString::number(this->prob_time_mov));
    ui.label_solv_time_dual_value->setText(QString::number(this->prob_time_mov));

    uint tot_steps=0;
    QStringList h_headers; bool h_head=false; QStringList v_headers;
    double mov_duration = 0.0;
    vector<double> time; QVector<double> tot_timesteps;
    std::vector<std::vector<QString>> mov_steps;
    for(size_t k=0; k< this->jointsPosition_mov.size();++k){
        MatrixXd jointPosition_stage = this->jointsPosition_mov.at(k);
        MatrixXd jointVelocity_stage = this->jointsVelocity_mov.at(k);
        MatrixXd jointAcceleration_stage = this->jointsAcceleration_mov.at(k);
        std::vector<double> timestep_stage = this->timesteps_mov.at(k);
        std::vector<QString> stage_step;
        double time_init;
        if(time.empty()){
            time_init=0.0;
        }else{
            time_init=time.at(time.size()-1);
        }
        vector<double> time_stage(timestep_stage.size());
        time_stage.at(0) = time_init;
        double stage_duration = 0.0;
        for(int i = 0; i< jointPosition_stage.rows(); ++i){
            tot_steps++;
            tot_timesteps.push_back(timestep_stage.at(i));
            if(i>0){
                stage_duration += timestep_stage.at(i);
                time_stage.at(i) = time_stage.at(i-1) + timestep_stage.at(i-1);
            }
            stage_step.clear();
            v_headers.push_back(QString("Step ")+QString::number(i));
            for (int j=0; j<jointPosition_stage.cols();++j){
                stage_step.push_back(
                        QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                        QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                        QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                if(!h_head){h_headers.push_back(QString("Joint ")+QString::number(j+1));}
            } // columns
            h_head = true;
            mov_steps.push_back(stage_step);
        }// rows
        mov_duration += stage_duration;
        time.reserve(time_stage.size());
        std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
    }// movements
    this->qtime_mov = QVector<double>::fromStdVector(time);

    // show the results
    ui.tableWidget_sol_mov->setColumnCount(h_headers.size());
    ui.tableWidget_sol_mov->setHorizontalHeaderLabels(h_headers);
    ui.tableWidget_sol_mov->setRowCount(v_headers.size());
    ui.tableWidget_sol_mov->setVerticalHeaderLabels(v_headers);
    for(int i =0; i < v_headers.size(); ++i){
        std::vector<QString> row = mov_steps.at(i);
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           ui.tableWidget_sol_mov->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    ui.label_totalTime_value_mov->setText(QString::number(mov_duration).toStdString().c_str());
    ui.tabWidget_sol->setEnabled(true);
    if(this->moveit_mov){
        ui.pushButton_execMov_moveit->setEnabled(true);
    }else{
        ui.pushButton_execMov_moveit->setEnabled(false);
    }

    this->tols_stop_mov.clear();
    double tol_stop = ui.lineEdit_tol_stop_mov->text().toDouble();
    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
        this->tols_stop_mov.push_back(tol_stop);
    }

    if(this->curr_mov->getArm()!=0)
    { // single-arm movement
        ui.tabWidget_plan_mov->setTabEnabled(0,true);
        ui.tabWidget_plan_mov->setTabEnabled(1,false);

        // compute the hand values, positions and accelerations
        //hand
        this->jointsPosition_mov_ctrl.resize(this->jointsPosition_mov.size());
        this->jointsVelocity_mov_ctrl.resize(this->jointsVelocity_mov.size());
        this->jointsAcceleration_mov_ctrl.resize(this->jointsAcceleration_mov.size());
        this->des_handPosition.clear(); this->des_handOrientation.clear(); this->des_handOrientation_q.clear();
        this->handPosition_mov.resize(tot_steps); this->handPosition_mov_stages.resize(this->jointsPosition_mov.size());
        this->handOrientation_mov.resize(tot_steps); this->handOrientation_q_mov.resize(tot_steps);
        this->handOrientation_mov_stages.resize(this->jointsPosition_mov.size());
        this->handOrientation_q_mov_stages.resize(this->jointsPosition_mov.size());
        this->wristPosition_mov.resize(tot_steps); this->wristOrientation_mov.resize(tot_steps);
        this->elbowPosition_mov.resize(tot_steps); this->elbowOrientation_mov.resize(tot_steps);
        this->shoulderPosition_mov.resize(tot_steps); this->shoulderOrientation_mov.resize(tot_steps);
        this->handVelocityNorm_mov.resize(tot_steps);
        this->handLinearVelocity_mov.resize(tot_steps); this->handLinearVelocity_mov_stages.resize(this->jointsPosition_mov.size());
        this->handAngularVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov_stages.resize(this->jointsPosition_mov.size());
        this->handLinearAcceleration_mov_stages.resize(this->jointsPosition_mov.size());
        this->handAngularAcceleration_mov_stages.resize(this->jointsPosition_mov.size());
        // wrist
        this->wristVelocityNorm_mov.resize(tot_steps);
        this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
        // elbow
        this->elbowVelocityNorm_mov.resize(tot_steps);
        this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
        //shoulder
        this->shoulderVelocityNorm_mov.resize(tot_steps);
        this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);
        // swivel angle
        this->swivel_angle_mov.resize(tot_steps);
        this->swivel_angle_mov_stages.resize(this->jointsPosition_mov.size());
        this->der_swivel_angle_mov_stages.resize(this->jointsPosition_mov.size());
        this->der_swivel_angle_mov_max_stages.resize(this->jointsPosition_mov.size());
        this->der_swivel_angle_mov_min_stages.resize(this->jointsPosition_mov.size());
        this->der_swivel_angle_mov_average_stages.resize(this->jointsPosition_mov.size());

        vector<double> timesteps_mov_tot(tot_steps,0.0);
        int step = 0;
        int arm_code = prob->getMovement()->getArm();
        // bounce posture data
        vector<double> bounce_posture = this->h_results->bounce_warm_start_res.x;
        vector<double> bounce_arm_posture(bounce_posture.begin(),bounce_posture.begin()+JOINTS_ARM);
        this->curr_scene->getHumanoid()->getHandPos(arm_code,this->bounce_handPosition,bounce_arm_posture);
        this->curr_scene->getHumanoid()->getHandOr(arm_code,this->bounce_handOrientation,bounce_arm_posture);
        this->curr_scene->getHumanoid()->getHandOr_q(arm_code,this->bounce_handOrientation_q,bounce_arm_posture);

        for (size_t k=0; k< this->jointsPosition_mov.size();++k){
            MatrixXd pos_stage = this->jointsPosition_mov.at(k);
            MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
            MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
            vector<double> timesteps_mov_stage = this->timesteps_mov.at(k);
            //this->curr_scene->getHumanoid()->getHandAcceleration(arm_code,pos_stage,vel_stage,acc_stage,this->timesteps_mov.at(k),this->handLinearAcceleration_mov_stages.at(k),this->handAngularAcceleration_mov_stages.at(k));
            vector<vector<double>> h_pos(pos_stage.rows());
            vector<vector<double>> h_or(pos_stage.rows());
            vector<vector<double>> h_or_q(pos_stage.rows());
            vector<vector<double>> h_lin_vel(pos_stage.rows());
            vector<vector<double>> h_ang_vel(pos_stage.rows());
            vector<double> swivel_angles_stage(pos_stage.rows());
            for(int i=0;i<pos_stage.rows();++i){
                timesteps_mov_tot.at(step) = timesteps_mov_stage.at(i);
                // position
                VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> posture; posture.resize(pos_row.size());
                VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
                // hand
                this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_mov.at(step),posture);
                this->curr_scene->getHumanoid()->getHandPos(arm_code,h_pos.at(i),posture);
                this->curr_scene->getHumanoid()->getHandOr(arm_code,this->handOrientation_mov.at(step),posture);
                this->curr_scene->getHumanoid()->getHandOr(arm_code,h_or.at(i),posture);
                this->curr_scene->getHumanoid()->getHandOr_q(arm_code,h_or_q.at(i),posture);
                this->curr_scene->getHumanoid()->getHandOr_q(arm_code,this->handOrientation_q_mov.at(step),posture);
                // wrist
                this->curr_scene->getHumanoid()->getWristPos(arm_code,this->wristPosition_mov.at(step),posture);
                this->curr_scene->getHumanoid()->getWristOr(arm_code,this->wristOrientation_mov.at(step),posture);
                // elbow
                this->curr_scene->getHumanoid()->getElbowPos(arm_code,this->elbowPosition_mov.at(step),posture);
                this->curr_scene->getHumanoid()->getElbowOr(arm_code,this->elbowOrientation_mov.at(step),posture);
                // shoulder
                this->curr_scene->getHumanoid()->getShoulderPos(arm_code,this->shoulderPosition_mov.at(step),posture);
                this->curr_scene->getHumanoid()->getShoulderOr(arm_code,this->shoulderOrientation_mov.at(step),posture);
                // swivel angle
                this->swivel_angle_mov.at(step) = this->curr_scene->getHumanoid()->getSwivelAngle(arm_code,posture);
                swivel_angles_stage.at(i) = this->curr_scene->getHumanoid()->getSwivelAngle(arm_code,posture);
                // velocities
                VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> velocities; velocities.resize(vel_row.size());
                VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                // hand velocity
                this->handVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
                vector<double> hand_vel; this->curr_scene->getHumanoid()->getHandVel(arm_code,hand_vel,posture,velocities);
                this->handLinearVelocity_mov.at(step) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
                this->handAngularVelocity_mov.at(step) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
                h_lin_vel.at(i) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
                h_ang_vel.at(i) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
                // wrist velocity
                this->wristVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getWristVelNorm(arm_code,posture,velocities);
                vector<double> wrist_vel; this->curr_scene->getHumanoid()->getWristVel(arm_code,wrist_vel,posture,velocities);
                this->wristLinearVelocity_mov.at(step) = {wrist_vel.at(0),wrist_vel.at(1),wrist_vel.at(2)};
                this->wristAngularVelocity_mov.at(step) = {wrist_vel.at(3),wrist_vel.at(4),wrist_vel.at(5)};
                // elbow velocity
                this->elbowVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getElbowVelNorm(arm_code,posture,velocities);
                vector<double> elbow_vel; this->curr_scene->getHumanoid()->getElbowVel(arm_code,elbow_vel,posture,velocities);
                this->elbowLinearVelocity_mov.at(step) = {elbow_vel.at(0),elbow_vel.at(1),elbow_vel.at(2)};
                this->elbowAngularVelocity_mov.at(step) = {elbow_vel.at(3),elbow_vel.at(4),elbow_vel.at(5)};
                // shoulder velocity
                this->shoulderVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getShoulderVelNorm(arm_code,posture,velocities);
                vector<double> shoulder_vel; this->curr_scene->getHumanoid()->getShoulderVel(arm_code,shoulder_vel,posture,velocities);
                this->shoulderLinearVelocity_mov.at(step) = {shoulder_vel.at(0),shoulder_vel.at(1),shoulder_vel.at(2)};
                this->shoulderAngularVelocity_mov.at(step) = {shoulder_vel.at(3),shoulder_vel.at(4),shoulder_vel.at(5)};

                step++;
            }// loop steps in the stage

            // add microsteps for controlling
//            vector<vector<double>> h_micro_pos;
//            vector<vector<double>> h_micro_or_q;
//            vector<vector<double>> h_micro_lin_vel;
//            vector<vector<double>> h_micro_ang_vel;
//            MatrixXd pos_micro_stage(0,JOINTS_ARM+JOINTS_HAND);
//            this->add_microsteps(h_pos,h_micro_pos,MICROSTEPS_CTRL);
//            this->add_microsteps(h_or_q,h_micro_or_q,MICROSTEPS_CTRL);
//            this->add_microsteps(h_lin_vel,h_micro_lin_vel,MICROSTEPS_CTRL);
//            this->add_microsteps(h_ang_vel,h_micro_ang_vel,MICROSTEPS_CTRL);
//            this->add_microsteps(pos_stage,pos_micro_stage,MICROSTEPS_CTRL);

//            this->jointsPosition_mov_ctrl.at(k) = pos_micro_stage;
//            this->handPosition_mov_stages.at(k) = h_micro_pos;
//            this->handOrientation_q_mov_stages.at(k) = h_micro_or_q;
//            this->handLinearVelocity_mov_stages.at(k) = h_micro_lin_vel;
//            this->handAngularVelocity_mov_stages.at(k) = h_micro_ang_vel;

            this->jointsPosition_mov_ctrl.at(k) = pos_stage;
            this->jointsVelocity_mov_ctrl.at(k) = vel_stage;
            this->jointsAcceleration_mov_ctrl.at(k) = acc_stage;
            this->handPosition_mov_stages.at(k) = h_pos;
            this->handOrientation_mov_stages.at(k) = h_or;
            this->handOrientation_q_mov_stages.at(k) = h_or_q;
            this->handLinearVelocity_mov_stages.at(k) = h_lin_vel;
            this->handAngularVelocity_mov_stages.at(k) = h_ang_vel;
            this->getDerivative(this->handLinearVelocity_mov_stages.at(k),this->timesteps_mov.at(k),this->handLinearAcceleration_mov_stages.at(k));
            this->getDerivative(this->handAngularVelocity_mov_stages.at(k),this->timesteps_mov.at(k),this->handAngularAcceleration_mov_stages.at(k));
            this->des_handPosition.push_back(this->handPosition_mov.at(step-1));
            this->des_handOrientation.push_back(this->handOrientation_mov.at(step-1));
            this->des_handOrientation_q.push_back(this->handOrientation_q_mov.at(step-1));
            this->swivel_angle_mov_stages.at(k) = swivel_angles_stage;
            this->getDerivative(this->swivel_angle_mov_stages.at(k),this->timesteps_mov.at(k),this->der_swivel_angle_mov_stages.at(k));
            this->der_swivel_angle_mov_max_stages.at(k) = *std::max_element(this->der_swivel_angle_mov_stages.at(k).begin(),this->der_swivel_angle_mov_stages.at(k).end());
            this->der_swivel_angle_mov_min_stages.at(k) = *std::min_element(this->der_swivel_angle_mov_stages.at(k).begin(),this->der_swivel_angle_mov_stages.at(k).end());
            this->der_swivel_angle_mov_average_stages.at(k) = std::accumulate(this->der_swivel_angle_mov_stages.at(k).begin(),this->der_swivel_angle_mov_stages.at(k).end(),0.0)/this->der_swivel_angle_mov_stages.at(k).size();

        }// loop stages

        // max and min swivel angles
        this->swivel_angle_mov_max = *std::max_element(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end());
        this->swivel_angle_mov_min = *std::min_element(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end());
        this->swivel_angle_mov_average = std::accumulate(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end(),0.0)/this->swivel_angle_mov.size();

        // accelerations
        this->getDerivative(this->handLinearVelocity_mov,timesteps_mov_tot,this->handLinearAcceleration_mov);
        this->getDerivative(this->handAngularVelocity_mov,timesteps_mov_tot,this->handAngularAcceleration_mov);
        this->getDerivative(this->wristLinearVelocity_mov,timesteps_mov_tot,this->wristLinearAcceleration_mov);
        this->getDerivative(this->wristAngularVelocity_mov,timesteps_mov_tot,this->wristAngularAcceleration_mov);
        this->getDerivative(this->elbowLinearVelocity_mov,timesteps_mov_tot,this->elbowLinearAcceleration_mov);
        this->getDerivative(this->elbowAngularVelocity_mov,timesteps_mov_tot,this->elbowAngularAcceleration_mov);
        this->getDerivative(this->shoulderLinearVelocity_mov,timesteps_mov_tot,this->shoulderLinearAcceleration_mov);
        this->getDerivative(this->shoulderAngularVelocity_mov,timesteps_mov_tot,this->shoulderAngularAcceleration_mov);

        // -- normlized jerk cost of the hand -- //
        QVector<double> handPosition_mov_x; QVector<double> handPosition_mov_y; QVector<double> handPosition_mov_z;
        QVector<double> der_1_handPosition_mov_x; QVector<double> der_1_handPosition_mov_y; QVector<double> der_1_handPosition_mov_z;
        QVector<double> der_2_handPosition_mov_x; QVector<double> der_2_handPosition_mov_y; QVector<double> der_2_handPosition_mov_z;
        QVector<double> der_3_handPosition_mov_x; QVector<double> der_3_handPosition_mov_y; QVector<double> der_3_handPosition_mov_z;

        for(size_t i=0; i<this->handPosition_mov.size();++i){
            vector<double> position_i = this->handPosition_mov.at(i);
            handPosition_mov_x.push_back(position_i.at(0));
            handPosition_mov_y.push_back(position_i.at(1));
            handPosition_mov_z.push_back(position_i.at(2));
        }

        // derivatives
        this->getDerivative(handPosition_mov_x,tot_timesteps,der_1_handPosition_mov_x);
        this->getDerivative(der_1_handPosition_mov_x,tot_timesteps,der_2_handPosition_mov_x);
        this->getDerivative(der_2_handPosition_mov_x,tot_timesteps,der_3_handPosition_mov_x);
        this->getDerivative(handPosition_mov_y,tot_timesteps,der_1_handPosition_mov_y);
        this->getDerivative(der_1_handPosition_mov_y,tot_timesteps,der_2_handPosition_mov_y);
        this->getDerivative(der_2_handPosition_mov_y,tot_timesteps,der_3_handPosition_mov_y);
        this->getDerivative(handPosition_mov_z,tot_timesteps,der_1_handPosition_mov_z);
        this->getDerivative(der_1_handPosition_mov_z,tot_timesteps,der_2_handPosition_mov_z);
        this->getDerivative(der_2_handPosition_mov_z,tot_timesteps,der_3_handPosition_mov_z);

        QVector<double> jerk_hand;
        for(size_t i=0;i<handPosition_mov_x.size();++i){
            jerk_hand.push_back(sqrt(pow(der_3_handPosition_mov_x.at(i),2)+pow(der_3_handPosition_mov_y.at(i),2)+pow(der_3_handPosition_mov_z.at(i),2)));
        }
        double duration = this->qtime_mov.at(this->qtime_mov.size()-1);
        double length = sqrt(pow((handPosition_mov_x.at(handPosition_mov_x.size()-1)-handPosition_mov_x.at(0)),2)+
                             pow((handPosition_mov_y.at(handPosition_mov_y.size()-1)-handPosition_mov_y.at(0)),2)+
                             pow((handPosition_mov_z.at(handPosition_mov_z.size()-1)-handPosition_mov_z.at(0)),2));
        double total_cost_jerk_hand=0.0;
        for(size_t i=0;i<tot_timesteps.size();++i){
            total_cost_jerk_hand += pow(jerk_hand.at(i),2)*tot_timesteps.at(i);
        }
        total_cost_jerk_hand = sqrt(0.5*total_cost_jerk_hand*(pow(duration,5)/pow(length,2)));
        ui.label_cost_hand_value->setText(QString::number(total_cost_jerk_hand));
        this->njs_mov = total_cost_jerk_hand;

        // -- compute the number of movement units -- //
        this->nmu_mov = this->getNumberMovementUnits(this->handVelocityNorm_mov,this->qtime_mov);
        ui.label_nmu->setText(QString::number(this->nmu_mov));
    }else{
        // dual-arm movement
        ui.tabWidget_plan_mov->setTabEnabled(0,false);
        ui.tabWidget_plan_mov->setTabEnabled(1,true);

        // right
        // hand
        this->handPosition_mov.resize(tot_steps); this->handOrientation_mov.resize(tot_steps);
        this->handVelocityNorm_mov.resize(tot_steps);
        this->handLinearVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov.resize(tot_steps);
        this->handLinearAcceleration_mov.resize(tot_steps); this->handAngularAcceleration_mov.resize(tot_steps);
        // wrist
        this->wristPosition_mov.resize(tot_steps); this->wristOrientation_mov.resize(tot_steps);
        this->wristVelocityNorm_mov.resize(tot_steps);
        this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
        this->wristLinearAcceleration_mov.resize(tot_steps); this->wristAngularAcceleration_mov.resize(tot_steps);
        // elbow
        this->elbowPosition_mov.resize(tot_steps); this->elbowOrientation_mov.resize(tot_steps);
        this->elbowVelocityNorm_mov.resize(tot_steps);
        this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
        this->elbowLinearAcceleration_mov.resize(tot_steps); this->elbowAngularAcceleration_mov.resize(tot_steps);
        //shoulder
        this->shoulderPosition_mov.resize(tot_steps); this->shoulderOrientation_mov.resize(tot_steps);
        this->shoulderVelocityNorm_mov.resize(tot_steps);
        this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);
        this->shoulderLinearAcceleration_mov.resize(tot_steps); this->shoulderAngularAcceleration_mov.resize(tot_steps);

        // left
        // hand
        this->handPosition_mov_left.resize(tot_steps); this->handOrientation_mov_left.resize(tot_steps);
        this->handVelocityNorm_mov_left.resize(tot_steps);
        this->handLinearVelocity_mov_left.resize(tot_steps); this->handAngularVelocity_mov_left.resize(tot_steps);
        this->handLinearAcceleration_mov_left.resize(tot_steps); this->handAngularAcceleration_mov_left.resize(tot_steps);
        // wrist
        this->wristPosition_mov_left.resize(tot_steps); this->wristOrientation_mov_left.resize(tot_steps);
        this->wristVelocityNorm_mov_left.resize(tot_steps);
        this->wristLinearVelocity_mov_left.resize(tot_steps); this->wristAngularVelocity_mov_left.resize(tot_steps);
        this->wristLinearAcceleration_mov_left.resize(tot_steps); this->wristAngularAcceleration_mov_left.resize(tot_steps);
        // elbow
        this->elbowPosition_mov_left.resize(tot_steps); this->elbowOrientation_mov_left.resize(tot_steps);
        this->elbowVelocityNorm_mov_left.resize(tot_steps);
        this->elbowLinearVelocity_mov_left.resize(tot_steps); this->elbowAngularVelocity_mov_left.resize(tot_steps);
        this->elbowLinearAcceleration_mov_left.resize(tot_steps); this->elbowAngularAcceleration_mov_left.resize(tot_steps);
        //shoulder
        this->shoulderPosition_mov_left.resize(tot_steps); this->shoulderOrientation_mov_left.resize(tot_steps);
        this->shoulderVelocityNorm_mov_left.resize(tot_steps);
        this->shoulderLinearVelocity_mov_left.resize(tot_steps); this->shoulderAngularVelocity_mov_left.resize(tot_steps);
        this->shoulderLinearAcceleration_mov_left.resize(tot_steps); this->shoulderAngularAcceleration_mov_left.resize(tot_steps);

        int step = 0;
        vector<double> timesteps_mov_tot(tot_steps,0.0);
        //int l_init = JOINTS_ARM+JOINTS_HAND; int l_end = JOINTS_ARM+JOINTS_HAND+JOINTS_ARM;
        //int arm_code = prob->getMovement()->getArm();
        for (size_t k=0; k< this->jointsPosition_mov.size();++k){
            MatrixXd pos_stage = this->jointsPosition_mov.at(k);
            MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
            MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
            vector<double> timesteps_mov_stage = this->timesteps_mov.at(k);
            for(int i=0;i < pos_stage.rows();++i){
                timesteps_mov_tot.at(step) = timesteps_mov_stage.at(i);
                // positions
                VectorXd pos_row_right = pos_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> posture_right; posture_right.resize(pos_row_right.size());
                VectorXd::Map(&posture_right[0], pos_row_right.size()) = pos_row_right;
                VectorXd pos_row_left = pos_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                vector<double> posture_left; posture_left.resize(pos_row_left.size());
                VectorXd::Map(&posture_left[0], pos_row_left.size()) = pos_row_left;
                // hand
                this->curr_scene->getHumanoid()->getHandPos(1,this->handPosition_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getHandPos(2,this->handPosition_mov_left.at(step),posture_left);
                this->curr_scene->getHumanoid()->getHandOr(1,this->handOrientation_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getHandOr(2,this->handOrientation_mov_left.at(step),posture_left);
                // wrist
                this->curr_scene->getHumanoid()->getWristPos(1,this->wristPosition_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getWristPos(2,this->wristPosition_mov_left.at(step),posture_left);
                this->curr_scene->getHumanoid()->getWristOr(1,this->wristOrientation_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getWristOr(2,this->wristOrientation_mov_left.at(step),posture_left);
                // elbow
                this->curr_scene->getHumanoid()->getElbowPos(1,this->elbowPosition_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getElbowPos(2,this->elbowPosition_mov_left.at(step),posture_left);
                this->curr_scene->getHumanoid()->getElbowOr(1,this->elbowOrientation_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getElbowOr(2,this->elbowOrientation_mov_left.at(step),posture_left);
                // shoulder
                this->curr_scene->getHumanoid()->getShoulderPos(1,this->shoulderPosition_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getShoulderPos(2,this->shoulderPosition_mov_left.at(step),posture_left);
                this->curr_scene->getHumanoid()->getShoulderOr(1,this->shoulderOrientation_mov.at(step),posture_right);
                this->curr_scene->getHumanoid()->getShoulderOr(2,this->shoulderOrientation_mov_left.at(step),posture_left);
                // velocities
                VectorXd vel_row_right = vel_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> velocities_right; velocities_right.resize(vel_row_right.size());
                VectorXd::Map(&velocities_right[0], vel_row_right.size()) = vel_row_right;
                VectorXd vel_row_left = vel_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                vector<double> velocities_left; velocities_left.resize(vel_row_left.size());
                VectorXd::Map(&velocities_left[0], vel_row_left.size()) = vel_row_left;
                // hand velocity
                this->handVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(1,posture_right,velocities_right);
                this->handVelocityNorm_mov_left.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(2,posture_left,velocities_left);
                vector<double> hand_vel_right; this->curr_scene->getHumanoid()->getHandVel(1,hand_vel_right,posture_right,velocities_right);
                vector<double> hand_vel_left; this->curr_scene->getHumanoid()->getHandVel(2,hand_vel_left,posture_left,velocities_left);
                this->handLinearVelocity_mov.at(step) = {hand_vel_right.at(0),hand_vel_right.at(1),hand_vel_right.at(2)};
                this->handAngularVelocity_mov.at(step) = {hand_vel_right.at(3),hand_vel_right.at(4),hand_vel_right.at(5)};
                this->handLinearVelocity_mov_left.at(step) = {hand_vel_left.at(0),hand_vel_left.at(1),hand_vel_left.at(2)};
                this->handAngularVelocity_mov_left.at(step) = {hand_vel_left.at(3),hand_vel_left.at(4),hand_vel_left.at(5)};
                // wrist velocity
                this->wristVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getWristVelNorm(1,posture_right,velocities_right);
                this->wristVelocityNorm_mov_left.at(step) = this->curr_scene->getHumanoid()->getWristVelNorm(2,posture_left,velocities_left);
                vector<double> wrist_vel_right; this->curr_scene->getHumanoid()->getWristVel(1,wrist_vel_right,posture_right,velocities_right);
                vector<double> wrist_vel_left; this->curr_scene->getHumanoid()->getWristVel(2,wrist_vel_left,posture_left,velocities_left);
                this->wristLinearVelocity_mov.at(step) = {wrist_vel_right.at(0),wrist_vel_right.at(1),wrist_vel_right.at(2)};
                this->wristAngularVelocity_mov.at(step) = {wrist_vel_right.at(3),wrist_vel_right.at(4),wrist_vel_right.at(5)};
                this->wristLinearVelocity_mov_left.at(step) = {wrist_vel_left.at(0),wrist_vel_left.at(1),wrist_vel_left.at(2)};
                this->wristAngularVelocity_mov_left.at(step) = {wrist_vel_left.at(3),wrist_vel_left.at(4),wrist_vel_left.at(5)};
                // elbow velocity
                this->elbowVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getElbowVelNorm(1,posture_right,velocities_right);
                this->elbowVelocityNorm_mov_left.at(step) = this->curr_scene->getHumanoid()->getElbowVelNorm(2,posture_left,velocities_left);
                vector<double> elbow_vel_right; this->curr_scene->getHumanoid()->getElbowVel(1,elbow_vel_right,posture_right,velocities_right);
                vector<double> elbow_vel_left; this->curr_scene->getHumanoid()->getElbowVel(2,elbow_vel_left,posture_left,velocities_left);
                this->elbowLinearVelocity_mov.at(step) = {elbow_vel_right.at(0),elbow_vel_right.at(1),elbow_vel_right.at(2)};
                this->elbowAngularVelocity_mov.at(step) = {elbow_vel_right.at(3),elbow_vel_right.at(4),elbow_vel_right.at(5)};
                this->elbowLinearVelocity_mov_left.at(step) = {elbow_vel_left.at(0),elbow_vel_left.at(1),elbow_vel_left.at(2)};
                this->elbowAngularVelocity_mov_left.at(step) = {elbow_vel_left.at(3),elbow_vel_left.at(4),elbow_vel_left.at(5)};
                // shoulder velocity
                this->shoulderVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getShoulderVelNorm(1,posture_right,velocities_right);
                this->shoulderVelocityNorm_mov_left.at(step) = this->curr_scene->getHumanoid()->getShoulderVelNorm(2,posture_left,velocities_left);
                vector<double> shoulder_vel_right; this->curr_scene->getHumanoid()->getShoulderVel(1,shoulder_vel_right,posture_right,velocities_right);
                vector<double> shoulder_vel_left; this->curr_scene->getHumanoid()->getShoulderVel(2,shoulder_vel_left,posture_left,velocities_left);
                this->shoulderLinearVelocity_mov.at(step) = {shoulder_vel_right.at(0),shoulder_vel_right.at(1),shoulder_vel_right.at(2)};
                this->shoulderAngularVelocity_mov.at(step) = {shoulder_vel_right.at(3),shoulder_vel_right.at(4),shoulder_vel_right.at(5)};
                this->shoulderLinearVelocity_mov_left.at(step) = {shoulder_vel_left.at(0),shoulder_vel_left.at(1),shoulder_vel_left.at(2)};
                this->shoulderAngularVelocity_mov_left.at(step) = {shoulder_vel_left.at(3),shoulder_vel_left.at(4),shoulder_vel_left.at(5)};


                step++;
            }// loop steps in stage
        }// loop stages

        // accelerations
        this->getDerivative(this->handLinearVelocity_mov,timesteps_mov_tot,this->handLinearAcceleration_mov);
        this->getDerivative(this->handAngularVelocity_mov,timesteps_mov_tot,this->handAngularAcceleration_mov);
        this->getDerivative(this->wristLinearVelocity_mov,timesteps_mov_tot,this->wristLinearAcceleration_mov);
        this->getDerivative(this->wristAngularVelocity_mov,timesteps_mov_tot,this->wristAngularAcceleration_mov);
        this->getDerivative(this->elbowLinearVelocity_mov,timesteps_mov_tot,this->elbowLinearAcceleration_mov);
        this->getDerivative(this->elbowAngularVelocity_mov,timesteps_mov_tot,this->elbowAngularAcceleration_mov);
        this->getDerivative(this->shoulderLinearVelocity_mov,timesteps_mov_tot,this->shoulderLinearAcceleration_mov);
        this->getDerivative(this->shoulderAngularVelocity_mov,timesteps_mov_tot,this->shoulderAngularAcceleration_mov);

        this->getDerivative(this->handLinearVelocity_mov_left,timesteps_mov_tot,this->handLinearAcceleration_mov_left);
        this->getDerivative(this->handAngularVelocity_mov_left,timesteps_mov_tot,this->handAngularAcceleration_mov_left);
        this->getDerivative(this->wristLinearVelocity_mov_left,timesteps_mov_tot,this->wristLinearAcceleration_mov_left);
        this->getDerivative(this->wristAngularVelocity_mov_left,timesteps_mov_tot,this->wristAngularAcceleration_mov_left);
        this->getDerivative(this->elbowLinearVelocity_mov_left,timesteps_mov_tot,this->elbowLinearAcceleration_mov_left);
        this->getDerivative(this->elbowAngularVelocity_mov_left,timesteps_mov_tot,this->elbowAngularAcceleration_mov_left);
        this->getDerivative(this->shoulderLinearVelocity_mov_left,timesteps_mov_tot,this->shoulderLinearAcceleration_mov_left);
        this->getDerivative(this->shoulderAngularVelocity_mov_left,timesteps_mov_tot,this->shoulderAngularAcceleration_mov_left);

        // -- normlized jerk cost of the hands -- //
        // right
        QVector<double> handPosition_mov_x; QVector<double> handPosition_mov_y; QVector<double> handPosition_mov_z;
        QVector<double> der_1_handPosition_mov_x; QVector<double> der_1_handPosition_mov_y; QVector<double> der_1_handPosition_mov_z;
        QVector<double> der_2_handPosition_mov_x; QVector<double> der_2_handPosition_mov_y; QVector<double> der_2_handPosition_mov_z;
        QVector<double> der_3_handPosition_mov_x; QVector<double> der_3_handPosition_mov_y; QVector<double> der_3_handPosition_mov_z;
        // left
        QVector<double> handPosition_mov_left_x; QVector<double> handPosition_mov_left_y; QVector<double> handPosition_mov_left_z;
        QVector<double> der_1_handPosition_mov_left_x; QVector<double> der_1_handPosition_mov_left_y; QVector<double> der_1_handPosition_mov_left_z;
        QVector<double> der_2_handPosition_mov_left_x; QVector<double> der_2_handPosition_mov_left_y; QVector<double> der_2_handPosition_mov_left_z;
        QVector<double> der_3_handPosition_mov_left_x; QVector<double> der_3_handPosition_mov_left_y; QVector<double> der_3_handPosition_mov_left_z;

        for(size_t i=0; i < this->handPosition_mov.size();++i){
            vector<double> position_i = this->handPosition_mov.at(i);
            handPosition_mov_x.push_back(position_i.at(0));
            handPosition_mov_y.push_back(position_i.at(1));
            handPosition_mov_z.push_back(position_i.at(2));
        }

        for(size_t i=0; i < this->handPosition_mov_left.size();++i){
            vector<double> position_i = this->handPosition_mov_left.at(i);
            handPosition_mov_left_x.push_back(position_i.at(0));
            handPosition_mov_left_y.push_back(position_i.at(1));
            handPosition_mov_left_z.push_back(position_i.at(2));
        }

        // derivatives
        // right
        this->getDerivative(handPosition_mov_x,tot_timesteps,der_1_handPosition_mov_x);
        this->getDerivative(der_1_handPosition_mov_x,tot_timesteps,der_2_handPosition_mov_x);
        this->getDerivative(der_2_handPosition_mov_x,tot_timesteps,der_3_handPosition_mov_x);
        this->getDerivative(handPosition_mov_y,tot_timesteps,der_1_handPosition_mov_y);
        this->getDerivative(der_1_handPosition_mov_y,tot_timesteps,der_2_handPosition_mov_y);
        this->getDerivative(der_2_handPosition_mov_y,tot_timesteps,der_3_handPosition_mov_y);
        this->getDerivative(handPosition_mov_z,tot_timesteps,der_1_handPosition_mov_z);
        this->getDerivative(der_1_handPosition_mov_z,tot_timesteps,der_2_handPosition_mov_z);
        this->getDerivative(der_2_handPosition_mov_z,tot_timesteps,der_3_handPosition_mov_z);
        // left
        this->getDerivative(handPosition_mov_left_x,tot_timesteps,der_1_handPosition_mov_left_x);
        this->getDerivative(der_1_handPosition_mov_left_x,tot_timesteps,der_2_handPosition_mov_left_x);
        this->getDerivative(der_2_handPosition_mov_left_x,tot_timesteps,der_3_handPosition_mov_left_x);
        this->getDerivative(handPosition_mov_left_y,tot_timesteps,der_1_handPosition_mov_left_y);
        this->getDerivative(der_1_handPosition_mov_left_y,tot_timesteps,der_2_handPosition_mov_left_y);
        this->getDerivative(der_2_handPosition_mov_left_y,tot_timesteps,der_3_handPosition_mov_left_y);
        this->getDerivative(handPosition_mov_left_z,tot_timesteps,der_1_handPosition_mov_left_z);
        this->getDerivative(der_1_handPosition_mov_left_z,tot_timesteps,der_2_handPosition_mov_left_z);
        this->getDerivative(der_2_handPosition_mov_left_z,tot_timesteps,der_3_handPosition_mov_left_z);

        QVector<double> jerk_hand_right; QVector<double> jerk_hand_left;
        for(size_t i=0;i < handPosition_mov_x.size();++i){
            jerk_hand_right.push_back(sqrt(pow(der_3_handPosition_mov_x.at(i),2)+pow(der_3_handPosition_mov_y.at(i),2)+pow(der_3_handPosition_mov_z.at(i),2)));
            jerk_hand_left.push_back(sqrt(pow(der_3_handPosition_mov_left_x.at(i),2)+pow(der_3_handPosition_mov_left_y.at(i),2)+pow(der_3_handPosition_mov_left_z.at(i),2)));
        }
        double duration = this->qtime_mov.at(this->qtime_mov.size()-1);
        double length_right = sqrt(pow((handPosition_mov_x.at(handPosition_mov_x.size()-1)-handPosition_mov_x.at(0)),2)+
                             pow((handPosition_mov_y.at(handPosition_mov_y.size()-1)-handPosition_mov_y.at(0)),2)+
                             pow((handPosition_mov_z.at(handPosition_mov_z.size()-1)-handPosition_mov_z.at(0)),2));
        double length_left = sqrt(pow((handPosition_mov_left_x.at(handPosition_mov_left_x.size()-1)-handPosition_mov_left_x.at(0)),2)+
                             pow((handPosition_mov_left_y.at(handPosition_mov_left_y.size()-1)-handPosition_mov_left_y.at(0)),2)+
                             pow((handPosition_mov_left_z.at(handPosition_mov_left_z.size()-1)-handPosition_mov_left_z.at(0)),2));

        double total_cost_jerk_hand_right=0.0; double total_cost_jerk_hand_left=0.0;
        for(size_t i=0;i<tot_timesteps.size();++i){
            total_cost_jerk_hand_right += pow(jerk_hand_right.at(i),2)*tot_timesteps.at(i);
            total_cost_jerk_hand_left += pow(jerk_hand_left.at(i),2)*tot_timesteps.at(i);
        }
        total_cost_jerk_hand_right = sqrt(0.5*total_cost_jerk_hand_right*(pow(duration,5)/pow(length_right,2)));
        total_cost_jerk_hand_left = sqrt(0.5*total_cost_jerk_hand_left*(pow(duration,5)/pow(length_left,2)));
        this->njs_mov = total_cost_jerk_hand_right;
        this->njs_mov_left = total_cost_jerk_hand_left;
        ui.label_cost_hand_right_value->setText(QString::number(this->njs_mov));
        ui.label_cost_hand_left_value->setText(QString::number(this->njs_mov_left));

        // -- compute the number of movement units -- //
        this->nmu_mov = this->getNumberMovementUnits(this->handVelocityNorm_mov,this->qtime_mov);
        this->nmu_mov_left = this->getNumberMovementUnits(this->handVelocityNorm_mov_left,this->qtime_mov);
        ui.label_nmu_hand_right_value->setText(QString::number(this->nmu_mov));
        ui.label_nmu_hand_left_value->setText(QString::number(this->nmu_mov_left));
    }

} // if the problem has been solved

}


void MainWindow::on_pushButton_plan_trials_clicked()
{
    int trials = 100;
    int success = 0;
    for (int i =0; i<trials;++i){
        this->on_pushButton_plan_clicked();
        if(this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getSolved()){
            success++;
            this->on_pushButton_append_mov_clicked();
        }
    }

    double rate = 100*success/trials;
    ui.label_rate_task->setText(QString::number(rate));

}
void MainWindow::on_pushButton_plan_3d_power_law_clicked()
{
    int n_traj=100;
    double wmax = 50.0; // max joint velocity [deg/sec]
    std::vector<double> move_target;

    //humanoidPtr hh = this->curr_scene->getHumanoid();
    //Matrix4d T_hand; Matrix3d R_hand; vector<double> pos_hand;

    // see Brami et al. 2003
    // mm
    double x; double x_min = -600; double x_max = -100;
    double y; double y_min = 200; double y_max = 800;
    double z; double z_min = 900; double z_max = 1500;
    // rad
    double roll = 0.79; //double roll_min = -3.14; double roll_max = 3.14;
    double pitch = -1.57; //double pitch_min = -3.14 ; double pitch_max = 3.14;
    double yaw = 0; //double yaw_min = 1; double yaw_max = 1.5;
/*
    double roll; double roll_min = -0.7; double roll_max = 0.7;
    double pitch; double pitch_min = -3.14 ; double pitch_max = 0;
    double yaw; double yaw_min = 0.3; double yaw_max = 0.6;
    //double yaw; double yaw_min = 1; double yaw_max = 1;
*/


    for(int i =0; i<n_traj;++i){

        // add a reaching problem
        int planner_id = ui.comboBox_planner->currentIndex(); // planner
        ui.comboBox_mov->setCurrentIndex(1); // reaching movement
        ui.comboBox_Task->setCurrentIndex(0); // single arm movement
        ui.radioButton_right->setChecked(true); // right arm
        this->on_pushButton_addMov_clicked();

        bool solved=false;

        while(!solved){

            // generate random numbers
            std::srand(std::time(NULL));
            x = x_min + (x_max-x_min)*(rand() / double(RAND_MAX));
            y = y_min + (y_max-y_min)*(rand() / double(RAND_MAX));
            z = z_min + (z_max-z_min)*(rand() / double(RAND_MAX));

/*
            Vector4d p(x,y,z,1);
            hh->getRightHandPos(pos_hand); Vector3d p_hand(pos_hand.at(0),pos_hand.at(1),pos_hand.at(2));
            hh->getRightHandOr(R_hand);
            T_hand.block<3,3>(0,0) = R_hand;
            T_hand.block<3,1>(0,3) = p_hand;
            T_hand(3,0)=0; T_hand(3,1)=0; T_hand(3,2)=0; T_hand(3,3)=1;
            Vector4d p_point = (T_hand.inverse())*p; Vector3d point(p_point(0),p_point(1),p_point(2));
            double mov_dist = point.norm();
            double mov_dir = atan2(point(2),point(1));
            roll = 0.33*mov_dir-0.7;
            //roll = 0.33*mov_dir+0.7;
            if(roll<roll_min)
                roll=roll_min;
            if(roll>roll_max)
                roll=roll_max;
            //pitch = 0.00175* mov_dist-0.08;
            pitch = -0.00175* mov_dist+0.1;
            if (pitch<pitch_min)
                pitch=pitch_min;
            if(pitch>pitch_max)
                pitch=pitch_max;



            roll = roll_min + (roll_max-roll_min)*(rand() / double(RAND_MAX));
            pitch = pitch_min + (pitch_max-pitch_min)*(rand() / double(RAND_MAX));

            yaw = yaw_min + (yaw_max-yaw_min)*(rand() / double(RAND_MAX));
            */

            // set the parameters
            move_target.clear();
            move_target.push_back(x);
            move_target.push_back(y);
            move_target.push_back(z);
            move_target.push_back(roll);
            move_target.push_back(pitch);
            move_target.push_back(yaw);

            switch(planner_id){
            case 0: // HUMP
                mTolHumpdlg->setTargetMove(move_target);
                mTolHumpdlg->setWMax(wmax);
                mTolHumpdlg->setRandInit(false); // disable random initialization
                mTolHumpdlg->setColl(false); // disable collisions
                break;
            case 1: // RRT
                mRRTdlg->setTargetMove(move_target);
                break;
            case 2: // RRT Connect
                mRRTConnectdlg->setTargetMove(move_target);
                break;
            case 3: // RRT Star
                mRRTstardlg->setTargetMove(move_target);
                mRRTstardlg->setConfig(0); // PathLengthOptimizationObjective
                break;
            case 4: // PRM
                mPRMdlg->setTargetMove(move_target);
                break;
            case 5: // PRM Star
                mPRMstardlg->setTargetMove(move_target);
                mPRMstardlg->setConfig(0); // PathLengthOptimizationObjective
                break;

            }

            this->on_pushButton_plan_clicked();
            problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
            if(prob->getSolved()){
                this->on_pushButton_append_mov_clicked();
                if(planner_id!=0){
#if MOVEIT==1
                    this->on_pushButton_execMov_moveit_clicked();
#endif
                }
                this->on_pushButton_execMov_clicked();

                solved = true;
            }else{
                solved = false;
            }


            sleep(1);

        }
    }

}


void MainWindow::on_pushButton_plan_2d_power_law_clicked()
{
    int n_traj=100;
    double wmax = 50.0; // 50 deg/sec
    int plane=0; // 0 = transverse plane, 1 = coronal plane, 2 = sagittal plane
    if (ui.radioButton_transverse->isChecked())
    {
        plane=0;
    }else if(ui.radioButton_coronal->isChecked()){
        plane=1;
    }else if(ui.radioButton_sagittal->isChecked()){
        plane=2;
    }
    std::vector<double> move_target;


    // mm
    double x; double x_min = -600; double x_max = -100;
    double y; double y_min = 200; double y_max = 800;
    double z; double z_min = 1000; double z_max = 1500;
    // rad
    double roll = 0;
    double pitch = -1.57;
    double yaw = 0;

    // plane
    //std::vector<double> point1={-400,0,1000};
    //std::vector<double> point2={-100,500,1000};
    //std::vector<double> point3={-200,600,1000};


    for(int i =0; i<n_traj;++i){

        // add a reaching problem
        int planner_id = ui.comboBox_planner->currentIndex(); // planner
        ui.comboBox_mov->setCurrentIndex(1); // reaching movement
        ui.comboBox_Task->setCurrentIndex(0); // single arm movement
        ui.radioButton_right->setChecked(true); // right arm
        this->on_pushButton_addMov_clicked();

        bool solved=false;

        while(!solved){

            // generate random numbers
            std::srand(std::time(NULL));

            switch(plane){
            case 0: // transverse plane
                x = x_min + (x_max-x_min)*(rand() / double(RAND_MAX));
                y = y_min + (y_max-y_min)*(rand() / double(RAND_MAX));
                z = z_min;
                break;
            case 1: // coronal plane
                x = x_min;
                y = y_min + (y_max-y_min)*(rand() / double(RAND_MAX));
                z = z_min + (z_max-z_min)*(rand() / double(RAND_MAX));
                break;
            case 2: // sagittal plane
                x = x_min + (x_max-x_min)*(rand() / double(RAND_MAX));
                y = y_min+200;
                z = z_min + (z_max-z_min)*(rand() / double(RAND_MAX));
                break;
            }


            // set the parameters
            move_target.clear();
            move_target.push_back(x);
            move_target.push_back(y);
            move_target.push_back(z);
            move_target.push_back(roll);
            move_target.push_back(pitch);
            move_target.push_back(yaw);

            switch(planner_id){
            case 0: // HUMP
                mTolHumpdlg->setTargetMove(move_target);
                mTolHumpdlg->setWMax(wmax);
                mTolHumpdlg->setRandInit(false); // disable random initialization
                mTolHumpdlg->setColl(false); // disable collisions
                // plane constraints
                //mTolHumpdlg->set_add_plane(true);
                //mTolHumpdlg->setPlaneParameters(point1,point2,point3);
                break;
            case 1: // RRT
                mRRTdlg->setTargetMove(move_target);
                // plane constraints
                //mRRTdlg->set_add_plane(true);
                //mRRTdlg->setPlaneParameters(point1,point2,point3);
                break;
            case 2: // RRT Connect
                mRRTConnectdlg->setTargetMove(move_target);
                // plane constraints
                //mRRTConnectdlg->set_add_plane(true);
                //mRRTConnectdlg->setPlaneParameters(point1,point2,point3);
                break;
            case 3: // RRT Star
                mRRTstardlg->setTargetMove(move_target);
                mRRTstardlg->setConfig(0); // PathLengthOptimizationObjective
                // plane constraints
                //mRRTstardlg->set_add_plane(true);
                //mRRTstardlg->setPlaneParameters(point1,point2,point3);
                break;
            case 4: // PRM
                mPRMdlg->setTargetMove(move_target);
                // plane constraints
                //mPRMdlg->set_add_plane(true);
                //mPRMdlg->setPlaneParameters(point1,point2,point3);
                break;
            case 5: // PRM Star
                mPRMstardlg->setTargetMove(move_target);
                mPRMstardlg->setConfig(0); // PathLengthOptimizationObjective
                // plane constraints
                //mPRMstardlg->set_add_plane(true);
                //mPRMstardlg->setPlaneParameters(point1,point2,point3);
                break;

            }

            this->on_pushButton_plan_clicked();
            problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
            if(prob->getSolved()){
                this->on_pushButton_append_mov_clicked();
                if(planner_id!=0){
#if MOVEIT==1
                    this->on_pushButton_execMov_moveit_clicked();
#endif
                }
                this->on_pushButton_execMov_clicked();

                solved = true;
            }else{
                solved = false;
            }

            sleep(1);

        }
    }
}


void MainWindow::on_pushButton_execMov_pressed()
{

    qnode.log(QNode::Info,std::string("Executing the movement in V-REP . . ."));

}

void MainWindow::on_pushButton_execMov_moveit_pressed()
{
    qnode.log(QNode::Info,std::string("Executing the movement in RViz . . ."));
}


void MainWindow::on_pushButton_execMov_clicked()
{
    bool vel_mode = this->ui.checkBox_exec_vel_mode->isChecked();
    qnode.execMovement(this->jointsPosition_mov,this->jointsVelocity_mov,this->timesteps_mov, this->tols_stop_mov, this->traj_descr_mov, this->curr_mov, this->curr_scene,vel_mode);
}
#if MOVEIT==1
void MainWindow::on_pushButton_execMov_moveit_clicked()
{
    if(this->moveit_mov)
        this->m_planner->execute(m_results);
}
#endif

void MainWindow::on_pushButton_stop_mov_clicked()
{

    qnode.stopSim();
    qnode.resetSimTime();
    qnode.resetGlobals();
}

void MainWindow::on_pushButton_stop_task_clicked()
{
    qnode.stopSim();
    qnode.resetSimTime();
    qnode.resetGlobals();
}

void MainWindow::on_pushButton_save_end_posture_clicked()
{
    this->jointsEndAcceleration_mov.clear();
    this->jointsEndVelocity_mov.clear();
    this->jointsEndPosition_mov.clear();

    if(!this->jointsPosition_mov.empty()){
        MatrixXd joints_pos = this->jointsPosition_mov.back();
        MatrixXd joints_vel = this->jointsVelocity_mov.back();
        MatrixXd joints_acc = this->jointsAcceleration_mov.back();
        VectorXd end_pos = joints_pos.row(joints_pos.rows()-1);
        VectorXd end_vel = joints_vel.row(joints_vel.rows()-1);
        VectorXd end_acc = joints_acc.row(joints_acc.rows()-1);
        this->jointsEndPosition_mov.resize(end_pos.size());
        VectorXd::Map(&this->jointsEndPosition_mov[0], end_pos.size()) = end_pos;
        this->jointsEndVelocity_mov.resize(end_vel.size());
        VectorXd::Map(&this->jointsEndVelocity_mov[0], end_vel.size()) = end_vel;
        this->jointsEndAcceleration_mov.resize(end_acc.size());
        VectorXd::Map(&this->jointsEndAcceleration_mov[0], end_acc.size()) = end_acc;
    }
}

void MainWindow::on_pushButton_execTask_pressed(){

    qnode.log(QNode::Info,std::string("Executing the task . . ."));
}


void MainWindow::on_pushButton_execTask_clicked()
{
    bool vel_mode = this->ui.checkBox_exec_vel_mode->isChecked();
    if(ui.checkBox_comp_exec->isChecked()){
        qnode.execTask_complete(this->jointsPosition_task,this->jointsVelocity_task,this->timesteps_task, this->tols_stop_task, this->traj_descr_task,this->curr_task, this->curr_scene,vel_mode);
    }else{
        qnode.execTask(this->jointsPosition_task,this->jointsVelocity_task,this->timesteps_task, this->tols_stop_task, this->traj_descr_task,this->curr_task, this->curr_scene,vel_mode);
    }
}


void MainWindow::on_pushButton_load_task_clicked()
{


    int plan_id; QString plan_type;
    int mov_id; QString mov_type;
    int arm_code; QString arm_type;
    QString obj_str; objectPtr obj;
    QString obj_left_str; objectPtr obj_left;
    QString obj_eng_str; objectPtr obj_eng;
    QString obj_eng_left_str; objectPtr obj_eng_left;
    QString pose_str; posePtr pose;
    QString pose_left_str; posePtr pose_left;
    bool prec; QString grip_type;
    bool prec_left; QString grip_type_left;
    int row=0;
    MatrixXd pos_stage;
    MatrixXd vel_stage;
    MatrixXd acc_stage;

    //clear
    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->traj_descr_task.clear();
    this->timesteps_task.clear();
    this->tols_stop_task.clear();
    this->njs_task.clear(); this->njs_task_left.clear();
    this->nmu_task.clear(); this->nmu_task_left.clear();
    this->prob_time_task.clear();
    this->jointsPosition_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsAcceleration_mov.clear();
    this->traj_descr_mov.clear();
    this->timesteps_mov.clear();
    this->tols_stop_mov.clear();
    ui.tableWidget_sol_task->clear();
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->curr_task->clearProblems();

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the task trajectory"),
                                                    QString(MAIN_PATH)+"/Tasks",
                                                    "All Files (*.*);; Task Files (*.task)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;

        vector<MatrixXd> t_mov;
        vector<MatrixXd> w_mov;
        vector<MatrixXd> a_mov;
        vector<vector<double>> timesteps_mov;
        vector<double> timesteps_stage;
        vector<double> tols_stop_mov;
        vector<string> descr_mov;

        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)==QChar('#')){

                // the previous stage has finished
                if(pos_stage.rows()!=0){
                    t_mov.push_back(pos_stage);
                    w_mov.push_back(vel_stage);
                    a_mov.push_back(acc_stage);
                    timesteps_mov.push_back(timesteps_stage);
                }
                // the previous movement has finished
                if(!t_mov.empty()){
                    this->jointsPosition_task.push_back(t_mov);
                    this->jointsVelocity_task.push_back(w_mov);
                    this->jointsAcceleration_task.push_back(a_mov);
                    this->timesteps_task.push_back(timesteps_mov);
                    this->tols_stop_task.push_back(tols_stop_mov);
                    this->traj_descr_task.push_back(descr_mov);
                }
                // new movement in the task
                t_mov.clear();
                w_mov.clear();
                a_mov.clear();
                timesteps_mov.clear();
                tols_stop_mov.clear();
                descr_mov.clear();
                // new stage in the movement
                pos_stage.resize(0,0);
                vel_stage.resize(0,0);
                acc_stage.resize(0,0);
                timesteps_stage.clear();

                if((line.at(1)==QChar('E')) && (line.at(2)==QChar('N')) && (line.at(3)==QChar('D'))){break;}

                QStringList fields = line.split(",");
                QString tmp = fields.at(0); tmp.remove(QChar('#')); fields[0]=tmp;
                for(int i=0; i< fields.size(); ++i){
                    QStringList fields1 = fields.at(i).split(":");
                    if (QString::compare(fields1.at(0).simplified(),QString("Planner"),Qt::CaseInsensitive)==0){
                       plan_type = fields1.at(1).simplified();
                    }else if (QString::compare(fields1.at(0).simplified(),QString("Movement"),Qt::CaseInsensitive)==0){
                        mov_type = fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Arm"),Qt::CaseInsensitive)==0){
                        arm_type=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object"),Qt::CaseInsensitive)==0){
                        obj_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object right"),Qt::CaseInsensitive)==0){
                        obj_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object left"),Qt::CaseInsensitive)==0){
                        obj_left_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object Engaged"),Qt::CaseInsensitive)==0){
                        obj_eng_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object right Engaged"),Qt::CaseInsensitive)==0){
                        obj_eng_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object left Engaged"),Qt::CaseInsensitive)==0){
                        obj_eng_left_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Pose"),Qt::CaseInsensitive)==0){
                        pose_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Pose right"),Qt::CaseInsensitive)==0){
                        pose_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Pose left"),Qt::CaseInsensitive)==0){
                        pose_left_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Grip Type"),Qt::CaseInsensitive)==0){
                        grip_type=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Grip Type right"),Qt::CaseInsensitive)==0){
                        grip_type=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Grip Type left"),Qt::CaseInsensitive)==0){
                        grip_type_left=fields1.at(1).simplified();
                    }
                }

                //get the planner id
                this->moveit_task = false;
                if(QString::compare(plan_type,QString("HUMP"),Qt::CaseInsensitive)==0){
                    plan_id=0;
                }else if(QString::compare(plan_type,QString("RRT"),Qt::CaseInsensitive)==0){
                    plan_id=1; this->moveit_task = true;
                }else if(QString::compare(plan_type,QString("RRTConnect"),Qt::CaseInsensitive)==0){
                    plan_id=2; this->moveit_task = true;
                }else if(QString::compare(plan_type,QString("RRTstar"),Qt::CaseInsensitive)==0){
                    plan_id=3; this->moveit_task = true;
                }else if(QString::compare(plan_type,QString("PRM"),Qt::CaseInsensitive)==0){
                    plan_id=4; this->moveit_task = true;
                }else if(QString::compare(plan_type,QString("PRMstar"),Qt::CaseInsensitive)==0){
                    plan_id=5; this->moveit_task = true;
                }

                // get the grip type
                if(QString::compare(grip_type,QString("Precision"),Qt::CaseInsensitive)==0){
                    prec=true;
                }else{
                    prec=false;
                }
                if(QString::compare(grip_type_left,QString("Precision"),Qt::CaseInsensitive)==0){
                    prec_left=true;
                }else{
                    prec_left=false;
                }

                //get the arm
                if(QString::compare(arm_type,QString("both"),Qt::CaseInsensitive)==0){
                    arm_code=0;
                    ui.tabWidget_plan_task->setTabEnabled(0,false);
                    ui.tabWidget_plan_task->setTabEnabled(1,true);
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                    obj_left = this->curr_scene->getObject(obj_left_str.toStdString());
                }else if(QString::compare(arm_type,QString("right"),Qt::CaseInsensitive)==0){
                    arm_code=1;
                    ui.tabWidget_plan_task->setTabEnabled(0,true);
                    ui.tabWidget_plan_task->setTabEnabled(1,false);
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                }else if(QString::compare(arm_type,QString("left"),Qt::CaseInsensitive)==0){
                    arm_code=2;
                    ui.tabWidget_plan_task->setTabEnabled(0,true);
                    ui.tabWidget_plan_task->setTabEnabled(1,false);
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                }

                // get the movement type
                if(QString::compare(mov_type,QString("Reach-to-grasp"),Qt::CaseInsensitive)==0){
                    mov_id=0;
                    //get the object
                    switch (arm_code){
                    case 0: // dual arm
                        obj->setTargetRightEnabled(true); obj->setTargetLeftEnabled(false);
                        obj_left->setTargetRightEnabled(false); obj_left->setTargetLeftEnabled(true);
                        break;
                    case 1: // right arm
                         obj->setTargetRightEnabled(true);
                         obj->setTargetLeftEnabled(false);                        
                         break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        break;
                    }
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,prec),new Scenario(*(this->curr_scene.get()))));
                        }else{//dual-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id,mov_id,0,obj,prec,obj_left,prec_left),new Scenario(*(this->curr_scene.get()))));
                        }
                    }else{
#if MOVEIT==1
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Reaching"),Qt::CaseInsensitive)==0){
                    mov_id=1;
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));
                        }else{ // dual-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id,mov_id,0),new Scenario(*(this->curr_scene.get()))));
                        }
                    }else{
#if MOVEIT==1
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Transport"),Qt::CaseInsensitive)==0){
                    mov_id=2;
                    problemPtr prob;
                    //get the object and the pose
                    switch (arm_code){
                    case 0: // dual arm
                        obj->setTargetRightEnabled(true); obj->setTargetLeftEnabled(false);
                        obj_left->setTargetRightEnabled(false); obj_left->setTargetLeftEnabled(true);
                        pose = this->curr_scene->getPose(pose_str.toStdString());
                        pose_left = this->curr_scene->getPose(pose_left_str.toStdString());
                        break;
                    case 1: // right arm
                         obj->setTargetRightEnabled(true);
                         obj->setTargetLeftEnabled(false);
                         pose = this->curr_scene->getPose(pose_str.toStdString());
                         break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        pose = this->curr_scene->getPose(pose_str.toStdString());
                        break;
                    }
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,pose,prec),new Scenario(*(this->curr_scene.get()))));
                        }else{// dual-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id,mov_id,0,obj,pose,prec,obj_left,pose_left,prec_left),new Scenario(*(this->curr_scene.get()))));
                        }
                    }else{
#if MOVEIT==1
                       prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,pose,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Engage"),Qt::CaseInsensitive)==0){
                    mov_id=3;
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                    // get the object engaged

                    switch (arm_code){
                    case 0: // dual arm
                        obj->setTargetRightEnabled(true); obj->setTargetLeftEnabled(false);
                        obj_left->setTargetRightEnabled(false); obj_left->setTargetLeftEnabled(true);
                        obj_eng = this->curr_scene->getObject(obj_eng_str.toStdString());
                        obj_eng_left = this->curr_scene->getObject(obj_eng_left_str.toStdString());
                        break;
                    case 1: // right arm
                         obj->setTargetRightEnabled(true);
                         obj->setTargetLeftEnabled(false);
                         obj_eng = this->curr_scene->getObject(obj_eng_str.toStdString());
                         break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        obj_eng = this->curr_scene->getObject(obj_eng_str.toStdString());
                        break;
                    }
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get()))));
                        }else{ // dual-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id,mov_id,0,obj,obj_eng,prec,obj_left,obj_eng_left,prec_left),new Scenario(*(this->curr_scene.get()))));
                        }
                    }else{
#if MOVEIT==1
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Disengage"),Qt::CaseInsensitive)==0){
                    mov_id=4;
                    // TO DO
                }else if(QString::compare(mov_type,QString("Go park"),Qt::CaseInsensitive)==0){
                    mov_id=5;
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));
                        }else{//dual-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id,mov_id,0),new Scenario(*(this->curr_scene.get()))));
                        }
                    }else{
#if MOVEIT==1
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get())),this->m_planner));
#endif
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }

                //logging
                qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
                ui.groupBox_task->setEnabled(true);
                ui.listWidget_movs->clear();
                for (int i = 0; i < this->curr_task->getProblemNumber();i++ ){
                   ui.listWidget_movs->addItem(QString(this->curr_task->getProblemInfo(i).c_str()));
                }
                ui.listWidget_movs->setCurrentRow(0);


            }else if(line.at(0)==QChar('M')){
                QStringList fields = line.split(":");
                if(QString::compare(fields.at(0).simplified(),QString("Movement stage"),Qt::CaseInsensitive)==0){
                    descr_mov.push_back((fields.at(1).simplified()).toStdString());
                }
                // the previous stage has finished
                if(pos_stage.rows()!=0){
                    t_mov.push_back(pos_stage);
                    w_mov.push_back(vel_stage);
                    a_mov.push_back(acc_stage);
                    timesteps_mov.push_back(timesteps_stage);
                }
                // new stage in the movement
                pos_stage.resize(0,0);
                vel_stage.resize(0,0);
                acc_stage.resize(0,0);
                timesteps_stage.clear();
                row=0;
            }else if(line.at(0)==QChar('t')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("tol stop"),Qt::CaseInsensitive)==0){
                    tols_stop_mov.push_back(fields.at(1).toDouble());
                }
            }else if((line.at(0)==QChar('n')) && (line.at(1)==QChar('j'))){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("njs"),Qt::CaseInsensitive)==0){
                    this->njs_task.push_back(fields.at(1).toDouble());
                }else if(QString::compare(fields.at(0).simplified(),QString("njs_right"),Qt::CaseInsensitive)==0){
                    this->njs_task.push_back(fields.at(1).toDouble());
                }else if(QString::compare(fields.at(0).simplified(),QString("njs_left"),Qt::CaseInsensitive)==0){
                    this->njs_task_left.push_back(fields.at(1).toDouble());
                }
            }else if((line.at(0)==QChar('n')) && (line.at(1)==QChar('m'))){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("nmu"),Qt::CaseInsensitive)==0){
                    this->nmu_task.push_back(fields.at(1).toDouble());
                }else if(QString::compare(fields.at(0).simplified(),QString("nmu_right"),Qt::CaseInsensitive)==0){
                    this->nmu_task.push_back(fields.at(1).toDouble());
                }else if(QString::compare(fields.at(0).simplified(),QString("nmu_left"),Qt::CaseInsensitive)==0){
                    this->nmu_task_left.push_back(fields.at(1).toDouble());
                }
            }else if(line.at(0)==QChar('p')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("prob_time"),Qt::CaseInsensitive)==0){
                    this->prob_time_task.push_back(fields.at(1).toDouble());
                }
            }else{
                int n_joints;
                if(arm_code!=0){
                    //single-arm
                    n_joints = JOINTS_ARM+JOINTS_HAND;

                }else{
                    //dual-arm
                    n_joints = JOINTS_ARM+JOINTS_HAND+JOINTS_ARM+JOINTS_HAND;
                }
                pos_stage.conservativeResize(pos_stage.rows()+1,n_joints);
                vel_stage.conservativeResize(vel_stage.rows()+1,n_joints);
                acc_stage.conservativeResize(acc_stage.rows()+1,n_joints);

                QStringList fields = line.split(",");                               
                for(int i=0; i <fields.size();++i){
                    QStringList fields1 = fields.at(i).split("=");
                    if(QString::compare(fields1.at(0).simplified(),QString("time step"),Qt::CaseInsensitive)==0){
                        timesteps_stage.push_back(fields1.at(1).toDouble());
                    }
                    for(int k=0; k < n_joints; ++k){
                        if(QString::compare(fields1.at(0).simplified(),QString("Joint ")+QString::number(k+1),Qt::CaseInsensitive)==0){
                            QStringList fields2 = fields1.at(1).split("|");
                            pos_stage(row,k) = ((double)fields2.at(0).toDouble()*M_PI)/180;
                            vel_stage(row,k) = ((double)fields2.at(1).toDouble()*M_PI)/180;
                            acc_stage(row,k) = ((double)fields2.at(2).toDouble()*M_PI)/180;
                        }else if(QString::compare(fields1.at(0).simplified(),QString("step"),Qt::CaseInsensitive)==0){break;}
                    }
                } // for loop columns
                row++;
            }

        } // while loop rows

        qnode.log(QNode::Info,std::string("The task has been loaded"));

        QStringList h_headers; bool h_head=false; QStringList v_headers;
        std::vector<std::vector<QString>> task_steps;
        vector<MatrixXd> pos_mov; vector<MatrixXd> vel_mov; vector<MatrixXd> acc_mov; vector<vector<double>> tstep_mov; vector<double> tstep_stage;
        double task_duration = 0.0; double mov_duration = 0.0; double stage_duration = 0.0;
        vector<double> time_task; uint tot_steps = 0;
        for(size_t h=0; h< this->jointsPosition_task.size();++h){ // for loop movements
            pos_mov = this->jointsPosition_task.at(h);
            vel_mov = this->jointsVelocity_task.at(h);
            acc_mov = this->jointsAcceleration_task.at(h);
            tstep_mov = this->timesteps_task.at(h);
            mov_duration = 0.0;

            for (size_t k=0; k< pos_mov.size();++k){ // for loop stages
                MatrixXd jointPosition_stage = pos_mov.at(k);
                MatrixXd jointVelocity_stage = vel_mov.at(k);
                MatrixXd jointAcceleration_stage = acc_mov.at(k);
                tstep_stage = tstep_mov.at(k);
                vector<double> time_stage(tstep_stage.size());
                double time_init;
                if(time_task.empty()){
                    time_init=0.0;
                }else{
                    time_init=time_task.at(time_task.size()-1);
                }
                time_stage.at(0) = time_init;
                stage_duration = 0.0;
                std::vector<QString> stage_step;
                stage_duration = 0.0;
                for(int i =0; i< jointPosition_stage.rows(); ++i){ // for loop steps
                    tot_steps++;
                    if(i>0){
                        time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                        stage_duration += tstep_stage.at(i);
                    }
                    stage_step.clear();
                    v_headers.push_back(QString("Step ")+QString::number(i));
                    for (int j=0; j<jointPosition_stage.cols();++j){ // for loop joints
                        stage_step.push_back(
                                QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                        if(!h_head){h_headers.push_back(QString("Joint ")+QString::number(j+1));}
                    } // stage columns
                    h_head = true;
                    task_steps.push_back(stage_step);
                }// stage rows
                mov_duration += stage_duration;
                time_task.reserve(time_stage.size());
                std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_task));
            }// movements
            task_duration +=mov_duration;
        }//task
        this->qtime_task = QVector<double>::fromStdVector(time_task);

        ui.tableWidget_sol_task->setColumnCount(h_headers.size());
        ui.tableWidget_sol_task->setHorizontalHeaderLabels(h_headers);
        ui.tableWidget_sol_task->setRowCount(v_headers.size());
        ui.tableWidget_sol_task->setVerticalHeaderLabels(v_headers);
        for(int i =0; i < v_headers.size(); ++i){
            std::vector<QString> row = task_steps.at(i);
            for(int j=0; j < h_headers.size(); ++j){
               QString item = row.at(j);
               ui.tableWidget_sol_task->setItem(i,j,new QTableWidgetItem(item));
            }
        }
        ui.label_totalTime_value_task->setText(QString::number(task_duration).toStdString().c_str());
        ui.tabWidget_sol->setEnabled(true);
        ui.tabWidget_sol->setCurrentIndex(1);

        // compute the hand values
        this->handPosition_task.resize(tot_steps); this->handVelocityNorm_task.resize(tot_steps);
        this->handLinearVelocity_task.resize(tot_steps);
        if(arm_code==0){
            this->handPosition_task_left.resize(tot_steps); this->handVelocityNorm_task_left.resize(tot_steps);
        }
        int step = 0;
        for(size_t j=0;j<this->jointsPosition_task.size();++j){ // for loop movs
            vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
            vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);
            for (size_t k=0; k< pos_mov.size();++k){// for loop stages
                MatrixXd pos_stage = pos_mov.at(k);
                MatrixXd vel_stage = vel_mov.at(k);
                for(int i=0;i<pos_stage.rows();++i){ // for loop steps
                    if(arm_code!=0){
                        //single-arm
                        // position
                        VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                        vector<double> posture; posture.resize(pos_row.size());
                        VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
                        this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_task.at(step),posture);
                        // velocity norm
                        VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                        vector<double> velocities; velocities.resize(vel_row.size());
                        VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                        this->handVelocityNorm_task.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
                        this->curr_scene->getHumanoid()->getHandVel(arm_code,this->handLinearVelocity_task.at(step),posture,velocities);
                    }else{
                        // dual-arm
                        // right hand position
                        VectorXd r_pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                        vector<double> r_posture; r_posture.resize(r_pos_row.size());
                        VectorXd::Map(&r_posture[0], r_pos_row.size()) = r_pos_row;
                        this->curr_scene->getHumanoid()->getHandPos(1,this->handPosition_task.at(step),r_posture);
                        // left hand position
                        VectorXd l_pos_row = pos_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                        vector<double> l_posture; l_posture.resize(l_pos_row.size());
                        VectorXd::Map(&l_posture[0], l_pos_row.size()) = l_pos_row;
                        this->curr_scene->getHumanoid()->getHandPos(2,this->handPosition_task_left.at(step),l_posture);
                        // right hand velocity norm
                        VectorXd r_vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                        vector<double> r_velocities; r_velocities.resize(r_vel_row.size());
                        VectorXd::Map(&r_velocities[0], r_vel_row.size()) = r_vel_row;
                        this->handVelocityNorm_task.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(1,r_posture,r_velocities);
                        // left hand velocity norm
                        VectorXd l_vel_row = vel_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                        vector<double> l_velocities; l_velocities.resize(l_vel_row.size());
                        VectorXd::Map(&l_velocities[0], l_vel_row.size()) = l_vel_row;
                        this->handVelocityNorm_task_left.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(2,l_posture,l_velocities);
                    }
                    step++;
                }
            }
        }

        // compute njs, nmu and planning time
        if(arm_code!=0){
            //single-arm
            // njs
            double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
            double mean_njs = ((double)sum_njs) / this->njs_task.size();
            string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
            boost::replace_all(mean_njs_str,",",".");
            double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
            double stdev_njs = std::sqrt((((double)sq_sum_njs) / this->njs_task.size()) - pow(mean_njs,2));
            string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
            boost::replace_all(stdev_njs_str,",",".");
            ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));
            // nmu
            double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
            double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
            string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
            boost::replace_all(mean_nmu_str,",",".");
            double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
            double stdev_nmu = std::sqrt((((double)sq_sum_nmu) / this->nmu_task.size()) - pow(mean_nmu,2));
            string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
            boost::replace_all(stdev_nmu_str,",",".");
            ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));
        }else{
            //dual-arm
            // right njs
            double sum_r_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
            double mean_r_njs = ((double)sum_r_njs) / this->njs_task.size();
            string mean_r_njs_str =  boost::str(boost::format("%.2f") % (mean_r_njs));
            boost::replace_all(mean_r_njs_str,",",".");
            double sq_sum_r_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
            double stdev_r_njs = std::sqrt((((double)sq_sum_r_njs) / this->njs_task.size()) - pow(mean_r_njs,2));
            string stdev_r_njs_str =  boost::str(boost::format("%.2f") % (stdev_r_njs));
            boost::replace_all(stdev_r_njs_str,",",".");
            ui.label_cost_hand_right_value_task->setText(QString::fromStdString(mean_r_njs_str)+QString("(")+QString::fromStdString(stdev_r_njs_str)+QString(")"));

            // left njs
            double sum_l_njs = std::accumulate(this->njs_task_left.begin(), this->njs_task_left.end(), 0.0);
            double mean_l_njs = ((double)sum_l_njs) / this->njs_task_left.size();
            string mean_l_njs_str =  boost::str(boost::format("%.2f") % (mean_l_njs));
            boost::replace_all(mean_l_njs_str,",",".");
            double sq_sum_l_njs = std::inner_product(this->njs_task_left.begin(), this->njs_task_left.end(), this->njs_task_left.begin(), 0.0);
            double stdev_l_njs = std::sqrt((((double)sq_sum_l_njs) / this->njs_task_left.size()) - pow(mean_l_njs,2));
            string stdev_l_njs_str =  boost::str(boost::format("%.2f") % (stdev_l_njs));
            boost::replace_all(stdev_l_njs_str,",",".");
            ui.label_cost_hand_left_value_task->setText(QString::fromStdString(mean_l_njs_str)+QString("(")+QString::fromStdString(stdev_l_njs_str)+QString(")"));

            // right nmu
            double sum_r_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
            double mean_r_nmu = ((double)sum_r_nmu) / this->nmu_task.size();
            string mean_r_nmu_str =  boost::str(boost::format("%.2f") % (mean_r_nmu));
            boost::replace_all(mean_r_nmu_str,",",".");
            double sq_sum_r_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
            double stdev_r_nmu = std::sqrt((((double)sq_sum_r_nmu) / this->nmu_task.size()) - pow(mean_r_nmu,2));
            string stdev_r_nmu_str =  boost::str(boost::format("%.2f") % (stdev_r_nmu));
            boost::replace_all(stdev_r_nmu_str,",",".");
            ui.label_nmu_hand_right_value_task->setText(QString::fromStdString(mean_r_nmu_str)+QString("(")+QString::fromStdString(stdev_r_nmu_str)+QString(")"));

            // left nmu
            double sum_l_nmu = std::accumulate(this->nmu_task_left.begin(), this->nmu_task_left.end(), 0.0);
            double mean_l_nmu = ((double)sum_l_nmu) / this->nmu_task_left.size();
            string mean_l_nmu_str =  boost::str(boost::format("%.2f") % (mean_l_nmu));
            boost::replace_all(mean_l_nmu_str,",",".");
            double sq_sum_l_nmu = std::inner_product(this->nmu_task_left.begin(), this->nmu_task_left.end(), this->nmu_task_left.begin(), 0.0);
            double stdev_l_nmu = std::sqrt((((double)sq_sum_l_nmu) / this->nmu_task_left.size()) - pow(mean_l_nmu,2));
            string stdev_l_nmu_str =  boost::str(boost::format("%.2f") % (stdev_l_nmu));
            boost::replace_all(stdev_l_nmu_str,",",".");
            ui.label_nmu_hand_left_value_task->setText(QString::fromStdString(mean_l_nmu_str)+QString("(")+QString::fromStdString(stdev_l_nmu_str)+QString(")"));

        }
        // planning time
        double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
        double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
        string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
        boost::replace_all(mean_prob_str,",",".");
        double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
        double stdev_prob = std::sqrt((((double)sq_sum_prob) / this->prob_time_task.size()) - pow(mean_prob,2));
        string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
        boost::replace_all(stdev_prob_str,",",".");
        if(arm_code!=0){
           ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
        }else{
            ui.label_solv_time_task_dual_value->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
        }
    }
    f.close();




}


void MainWindow::on_pushButton_save_task_clicked()
{


    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the task trajectory"),
                                                    QString(MAIN_PATH)+"/Tasks",
                                                    "All Files (*.*);;Task Files (*.task)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        int h=0; // numbers of problems that are not in the current task
        int arm_code = curr_task->getArm();
        for(int i=0; i <ui.listWidget_movs->count(); ++i){
            if((curr_task->getProblem(i)->getSolved()) && curr_task->getProblem(i)->getPartOfTask()){
                stream << "# " << ui.listWidget_movs->item(i)->text().toStdString().c_str()<< endl;
                vector< MatrixXd > traj_mov = this->jointsPosition_task.at(i-h);
                vector< MatrixXd > vel_mov = this->jointsVelocity_task.at(i-h);
                vector< MatrixXd > acc_mov = this->jointsAcceleration_task.at(i-h);
                vector< vector< double > > timesteps_mov = this->timesteps_task.at(i-h);
                vector< double > tols_stop_mov = this->tols_stop_task.at(i-h);
                vector< string > traj_descr_mov = this->traj_descr_task.at(i-h);
                //stream << "arm_code="<< QString::number(arm_code).toStdString().c_str()<< endl;
                if(arm_code!=0){
                    double njs = this->njs_task.at(i-h);
                    int nmu = this->nmu_task.at(i-h);
                    stream << "njs="<< QString::number(njs).toStdString().c_str()<< endl;
                    stream << "nmu="<< QString::number(nmu).toStdString().c_str()<< endl;
                }else{
                    double r_njs = this->njs_task.at(i-h);
                    int r_nmu = this->nmu_task.at(i-h);
                    stream << "njs_right="<< QString::number(r_njs).toStdString().c_str()<< endl;
                    stream << "nmu_right="<< QString::number(r_nmu).toStdString().c_str()<< endl;
                    double l_njs = this->njs_task_left.at(i-h);
                    int l_nmu = this->nmu_task_left.at(i-h);
                    stream << "njs_left="<< QString::number(l_njs).toStdString().c_str()<< endl;
                    stream << "nmu_left="<< QString::number(l_nmu).toStdString().c_str()<< endl;
                }
                double prob_time = this->prob_time_task.at(i-h);
                stream << "prob_time="<< QString::number(prob_time).toStdString().c_str()<< endl;

                for(size_t j=0;j < traj_mov.size(); ++j){
                    string descr_stage = traj_descr_mov.at(j);
                    //stream << "Movement stage: "<< QString::number(j+1).toStdString().c_str()<< endl;
                    stream << "Movement stage: "<< descr_stage.c_str()<< endl;
                    MatrixXd traj = traj_mov.at(j);
                    MatrixXd vel = vel_mov.at(j);
                    MatrixXd acc = acc_mov.at(j);
                    vector< double > timestep_stage  = timesteps_mov.at(j);
                    double tol_stop = tols_stop_mov.at(j);
                    stream << "tol stop="<< QString::number(tol_stop).toStdString().c_str()<< endl;

                    for(int r=0; r < traj.rows(); ++r){
                        double timestep = timestep_stage.at(r);
                        stream << "step="<< QString::number(r).toStdString().c_str()<<", ";
                        stream << "time step="<< QString::number(timestep).toStdString().c_str()<< ", ";
                        for(int c=0; c < traj.cols(); ++c){
                            stream << "Joint "<<QString::number(c+1).toStdString().c_str()<<"="<<
                                      QString::number(((double)traj(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                      QString::number(((double)vel(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                      QString::number(((double)acc(r,c)*180)/M_PI,'f',6).toStdString().c_str();
                            if(c==traj.cols()-1){stream << endl;}else{stream<<", ";}
                        }// for loop columns
                    }// for loop rows
                }// for loop movement
            }else{
                h++;
            }
        }// for loop task
        stream << "#END" <<endl;


    }// open file
    f.close();


}


void MainWindow::on_pushButton_scene_reset_clicked()
{

    // reset the movements
    ui.tableWidget_sol_mov->clear();
    //ui.listWidget_movs->clear();
    ui.label_totalTime_value_mov->clear();
    this->jointsAcceleration_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsPosition_mov.clear();
    this->timesteps_mov.clear();
    this->tols_stop_mov.clear();
#if MOVEIT==1
    this->m_planner.reset(new moveit_planning::HumanoidPlanner(this->init_scene->getName()));
#endif

    this->curr_scene = scenarioPtr(new Scenario(*(this->init_scene.get())));
    qnode.resetSimTime();
    qnode.resetGlobals();
    qnode.log(QNode::Info,std::string("Tha scenario has been reset"));

    int scene_id = this->scenario_id;
    string path;
    string title;
    string success;
    string failure;

    // Empty Scenario with ARoS
    string path_vrep_empty_aros = PATH_SCENARIOS+string("/vrep/empty_aros.ttt");

    // Empty scenario: empty scenario with ARoS and NO self collisions
    string path_vrep_emptyscene_aros_no_self_coll = PATH_SCENARIOS+string("/vrep/empty_aros_no_coll.ttt");

    // Toy vehicle scenario with ARoS
    //string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros.ttt");
    string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_bill.ttt");
    //string path_rviz_toyscene_aros = PATH_SCENARIOS+string("/rviz/toy_vehicle_aros.scene");

    // Drinking Service task with ARoS
    string path_vrep_drinking_aros = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_aros_bill.ttt");

    // Toy vehicle scenario with Jarde
    string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
    //string path_rviz_toyscene_jarde = PATH_SCENARIOS+string("/rviz/toy_vehicle_jarde.scene");

    // Challengingscenario with ARoS
    string path_vrep_challenge_aros = PATH_SCENARIOS+string("/vrep/NarrowShelf_aros.ttt");

    // Toy vehicle scenario with ARoS swaps two columns
    string path_vrep_toyscene_aros_dual = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_dual_arm_cols.ttt");

    // Drinking service scenario with ARoS holding a tray
    string path_vrep_drinkscene_aros_dual = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_aros_dual_arm_tray.ttt");

    // Natural obstacle avoidance with ARoS
    string path_vrep_natural_obst_av = PATH_SCENARIOS+string("/vrep/Natural_obst_avoidance_aros_1.ttt");

    // Learning tasks: reaching with one obstacle
    string path_vrep_learning_tasks_reaching_1 = PATH_SCENARIOS+string("/vrep/Learning_Reaching_1.ttt");

    // Learning tasks: reaching with many obstacles
    string path_vrep_learning_tasks_reaching_2 = PATH_SCENARIOS+string("/vrep/Learning_Reaching_2.ttt");

    // Learning tasks: picking the blue column
    string path_vrep_learning_tasks_picking_1 = PATH_SCENARIOS+string("/vrep/Learning_Picking_column_1.ttt");

    // Controlling: scenario without objects
    string path_vrep_controlling_no_objs = PATH_SCENARIOS+string("/vrep/Controlling_no_objs.ttt");

    // Controlling: scenario without objects for singularities
    string path_vrep_controlling_no_objs_sing = PATH_SCENARIOS+string("/vrep/Controlling_no_objs_sing.ttt");

    // Controlling: scenario with one obstacle and drawing an ellipse on the XY plane
    string path_vrep_controlling_obsts_av_ellipse = PATH_SCENARIOS+string("/vrep/Controlling_obsts_av_ellipse.ttt");

    switch(scene_id){

    case 0:
        // Assembly scenario: the Toy vehicle with ARoS
        path = path_vrep_toyscene_aros;
        title = string("Assembly scenario: the Toy vehicle with ARoS");
        success = string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with ARoS HAS NOT BEEN LOADED");
        break;
    case 1:
        // Assembly scenario: the Toy vehicle with Avatar
        path = path_vrep_toyscene_jarde;
        title = string("Assembly scenario: the Toy vehicle with the Avatar");
        success = string("Assembly scenario: the Toy vehicle with the Avatar HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with the Avatar HAS NOT BEEN LOADED");
        break;
    case 2:
        // Empty scenario with ARoS
        path = path_vrep_empty_aros;
        title = string("Empty scenario with ARoS");
        success = string("Empty scenario with ARoS HAS BEEN LOADED");
        failure = string("Empty scenario with ARoS HAS NOT BEEN LOADED");
        break;        
    case 3:
        // Empty scenario: empty scenario with ARoS and NO self collisions
        path = path_vrep_emptyscene_aros_no_self_coll;
        title = string("Empty scenario: empty scenario with ARoS and NO collisions");
        success = string("Empty scenario: empty scenario with ARoS and NO collisions with ARoS HAS BEEN LOADED");
        failure = string("Empty scenario: empty scenario with ARoS and NO collisions HAS NOT BEEN LOADED");
        break;
    case 4:
        // Assistive scenario: beverages with ARoS
        path = path_vrep_drinking_aros;
        title = string("Human assistance scenario: Serving a drink with ARoS");
        success = string("Human assistance scenario: Serving a drink with ARoS HAS BEEN LOADED");
        failure = string("Human assistance scenario: Serving a drink with ARoS HAS NOT BEEN LOADED");
        break;
    case 5:
        // Challenge scenario with ARoS
        path = path_vrep_challenge_aros;
        title = string("Challenging scenario: picking a cup from a shelf with ARoS");
        success = string("Challenging scenario: picking a cup from a shelf with ARoS HAS BEEN LOADED");
        failure = string("Challenging scenario: picking a cup from a shelf with ARoS HAS NOT BEEN LOADED");
        break;
    case 6:
        // Toy vehicle scenario with ARoS swaps two columns
        path = path_vrep_toyscene_aros_dual;
        title = string("Assembly scenario: the Toy vehicle with ARoS swaps columns");
        success = string("Assembly scenario: the Toy vehicle with ARoS swaps columns HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with ARoS swaps columns HAS NOT BEEN LOADED");
        break;
    case 7:
        // Drinking service scenario with ARoS holding a tray
        path = path_vrep_drinkscene_aros_dual;
        title = string("Human assistance scenario: Moving a tray with ARoS");
        success = string("Human assistance scenario: Moving a tray with ARoS HAS BEEN LOADED");
        failure = string("Human assistance scenario: Moving a tray with ARoS HAS NOT BEEN LOADED");
        break;
    case 8:
        // Natural obstacle avoidance with ARoS
        path = path_vrep_natural_obst_av;
        title = string("Natural obstacle avoidance with ARoS");
        success = string("Natural obstacle avoidance with ARoS HAS BEEN LOADED");
        failure = string("Natural obstacle avoidance with ARoS HAS NOT BEEN LOADED");
        break;
    case 9:
        // Learning Tasks: reaching with one obstacle
        path = path_vrep_learning_tasks_reaching_1;
        title = string("Learning tasks: reaching with one obstacle");
        success = string("Learning tasks: reaching with one obstacle HAS BEEN LOADED");
        failure = string("Learning tasks: reaching with one obstacle HAS NOT BEEN LOADED");
        break;
    case 10:
        // Learning Tasks: reaching with many obstacles
        path = path_vrep_learning_tasks_reaching_2;
        title = string("Learning tasks: reaching with many obstacles");
        success = string("Learning tasks: reaching with many obstacles HAS BEEN LOADED");
        failure = string("Learning tasks: reaching with many obstacles HAS NOT BEEN LOADED");
        break;
    case 11:
        // Learning Tasks: picking the blue column
        path = path_vrep_learning_tasks_picking_1;
        title = string("Learning tasks: picking the blue column");
        success = string("Learning tasks: picking the blue column HAS BEEN LOADED");
        failure = string("Learning tasks: picking the blue column HAS NOT BEEN LOADED");
        break;
    case 12:
        // Controlling: scenario with no objects
        path = path_vrep_controlling_no_objs;
        title = string("Controlling: scenario without objects");
        success = string("Controlling: scenario without objects HAS BEEN LOADED");
        failure = string("Controlling: scenario without objects HAS NOT BEEN LOADED");
        break;
    case 13:
        // Controlling: scenario with no objects for singularities
        path = path_vrep_controlling_no_objs_sing;
        title = string("Controlling: scenario with no obstacles for showing the effects of singularities");
        success = string("Controlling: scenario with no obstacles for showing the effects of singularities HAS BEEN LOADED");
        failure = string("Controlling: scenario with no obstacles for showing the effects of singularities HAS NOT BEEN LOADED");
        break;
    case 14:
        // Controlling: scenario with one obstacle and drawing an ellipse on the XY plane
        path = path_vrep_controlling_obsts_av_ellipse;
        title = string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane");
        success = string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane HAS BEEN LOADED");
        failure = string("Controlling: scenario with one obstacle and drawing an ellipse on the XY plane HAS NOT BEEN LOADED");
        break;
    }

    if (qnode.loadScenario(path,1)){
        qnode.log(QNode::Info,success);
        ui.groupBox_getElements->setEnabled(true);
        ui.groupBox_homePosture->setEnabled(true);
        std::vector<objectPtr> objs; this->curr_scene->getObjects(objs);
#if MOVEIT==1
        qnode.loadRVizScenario(objs);
#endif
    }else{
        qnode.log(QNode::Error,failure);
        ui.groupBox_getElements->setEnabled(false);
        ui.groupBox_homePosture->setEnabled(false);
        ui.pushButton_loadScenario->setEnabled(true);
    }


}


void MainWindow::on_pushButton_append_mov_clicked()
{

    ui.tableWidget_sol_task->clear();
    ui.pushButton_save_task->setEnabled(true);
    this->moveit_task = false;

    if(curr_task->getProblem(ui.listWidget_movs->currentRow())->getSolved()){
        if(curr_task->getProblem(ui.listWidget_movs->currentRow())->getPlannerID()!=0){this->moveit_task = true;}
        int arm_code = curr_task->getProblem(ui.listWidget_movs->currentRow())->getMovement()->getArm();
        if(curr_task->getProblemNumber()==1)
        {
            curr_task->setArm(arm_code);
        }else{
            if(arm_code!=curr_task->getArm()){
                return;
            }
        }
        if(arm_code!=0){
            ui.tabWidget_plan_task->setTabEnabled(0,true);
            ui.tabWidget_plan_task->setTabEnabled(1,false);
        }else{
            ui.tabWidget_plan_task->setTabEnabled(0,false);
            ui.tabWidget_plan_task->setTabEnabled(1,true);
        }
        this->jointsPosition_task.push_back(this->jointsPosition_mov);
        this->jointsVelocity_task.push_back(this->jointsVelocity_mov);
        this->jointsAcceleration_task.push_back(this->jointsAcceleration_mov);
        this->timesteps_task.push_back(this->timesteps_mov);
        this->tols_stop_task.push_back(this->tols_stop_mov);
        this->traj_descr_task.push_back(this->traj_descr_mov);
        this->njs_task.push_back(this->njs_mov);
        this->nmu_task.push_back(this->nmu_mov);
        if(arm_code==0){
            this->njs_task_left.push_back(this->njs_mov_left);
            this->nmu_task_left.push_back(this->nmu_mov_left);
        }
        this->prob_time_task.push_back(this->prob_time_mov);
         QStringList h_headers; bool h_head=false; QStringList v_headers;
         std::vector<std::vector<QString>> task_steps;
         vector<MatrixXd> pos_mov; vector<MatrixXd> vel_mov; vector<MatrixXd> acc_mov; vector<vector<double>> tstep_mov;
         double task_duration = 0.0; double mov_duration = 0.0; double stage_duration = 0.0;
         vector<double> time_task; uint tot_steps = 0;
         for(size_t h=0; h< this->jointsPosition_task.size();++h){ // for loop movs
             pos_mov = this->jointsPosition_task.at(h);
             vel_mov = this->jointsVelocity_task.at(h);
             acc_mov = this->jointsAcceleration_task.at(h);
             tstep_mov = this->timesteps_task.at(h);
             mov_duration = 0.0;
             for (size_t k=0; k < pos_mov.size();++k){ // for loop stages
                 MatrixXd jointPosition_stage = pos_mov.at(k);
                 MatrixXd jointVelocity_stage = vel_mov.at(k);
                 MatrixXd jointAcceleration_stage = acc_mov.at(k);
                 vector<double> tstep_stage = tstep_mov.at(k);
                 vector<double> time_stage(tstep_stage.size());
                 double time_init;
                 if(time_task.empty()){
                     time_init=0.0;
                 }else{
                     time_init=time_task.at(time_task.size()-1);
                 }
                 time_stage.at(0) = time_init;
                 stage_duration = 0.0;
                 std::vector<QString> stage_step;
                 for(int i =0; i< jointPosition_stage.rows(); ++i){ // for loop steps
                     tot_steps++;
                     if(i>0){
                         time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                         stage_duration += tstep_stage.at(i);
                     }
                     stage_step.clear();
                     v_headers.push_back(QString("Step ")+QString::number(i));
                     for (int j=0; j<jointPosition_stage.cols();++j){ // for loop joints
                         stage_step.push_back(
                                 QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                 QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                 QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                         if(!h_head){h_headers.push_back(QString("Joint ")+QString::number(j+1));}
                     } // stage columns
                     h_head = true;
                     task_steps.push_back(stage_step);
                 }// stage rows
                 mov_duration +=stage_duration;
                 time_task.reserve(time_stage.size());
                 std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_task));
             }// movements
             task_duration +=mov_duration;
         }//task
         this->qtime_task = QVector<double>::fromStdVector(time_task);

         ui.tableWidget_sol_task->setColumnCount(h_headers.size());
         ui.tableWidget_sol_task->setHorizontalHeaderLabels(h_headers);
         ui.tableWidget_sol_task->setRowCount(v_headers.size());
         ui.tableWidget_sol_task->setVerticalHeaderLabels(v_headers);
         for(int i =0; i < v_headers.size(); ++i){
             std::vector<QString> row = task_steps.at(i);
             for(int j=0; j < h_headers.size(); ++j){
                 QString item = row.at(j);
                ui.tableWidget_sol_task->setItem(i,j,new QTableWidgetItem(item));
             }
         }
         ui.label_totalTime_value_task->setText(QString::number(task_duration).toStdString().c_str());
         ui.tabWidget_sol->setCurrentIndex(1);

         // set part of the task
         curr_task->getProblem(ui.listWidget_movs->currentRow())->setPartOfTask(true);

         // compute the hand values
         this->handPosition_task.resize(tot_steps); this->handVelocityNorm_task.resize(tot_steps);
         this->handLinearVelocity_task.resize(tot_steps);
         if(arm_code==0){
            this->handPosition_task_left.resize(tot_steps); this->handVelocityNorm_task_left.resize(tot_steps);
         }
         int step = 0;
         //int arm_code = this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getMovement()->getArm();
         for(size_t j=0;j<this->jointsPosition_task.size();++j){ // for loop movs
             vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
             vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);
             for (size_t k=0; k< pos_mov.size();++k){ // for loop stages
                 MatrixXd pos_stage = pos_mov.at(k);
                 MatrixXd vel_stage = vel_mov.at(k);
                 for(int i=0;i<pos_stage.rows();++i){ // for loop steps
                     if(arm_code!=0){
                         //single-arm
                         // position
                         VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                         vector<double> posture; posture.resize(pos_row.size());
                         VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
                         this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_task.at(step),posture);
                         // velocity norm
                         VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                         vector<double> velocities; velocities.resize(vel_row.size());
                         VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                         this->handVelocityNorm_task.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
                         this->curr_scene->getHumanoid()->getHandVel(arm_code,this->handLinearVelocity_task.at(step),posture,velocities);
                     }else{
                         // dual-arm
                         // right hand position
                         VectorXd r_pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                         vector<double> r_posture; r_posture.resize(r_pos_row.size());
                         VectorXd::Map(&r_posture[0], r_pos_row.size()) = r_pos_row;
                         this->curr_scene->getHumanoid()->getHandPos(1,this->handPosition_task.at(step),r_posture);
                         // left hand position
                         VectorXd l_pos_row = pos_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                         vector<double> l_posture; l_posture.resize(l_pos_row.size());
                         VectorXd::Map(&l_posture[0], l_pos_row.size()) = l_pos_row;
                         this->curr_scene->getHumanoid()->getHandPos(2,this->handPosition_task_left.at(step),l_posture);
                         // right hand velocity norm
                         VectorXd r_vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                         vector<double> r_velocities; r_velocities.resize(r_vel_row.size());
                         VectorXd::Map(&r_velocities[0], r_vel_row.size()) = r_vel_row;
                         this->handVelocityNorm_task.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(1,r_posture,r_velocities);
                         // left hand velocity norm
                         VectorXd l_vel_row = vel_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                         vector<double> l_velocities; l_velocities.resize(l_vel_row.size());
                         VectorXd::Map(&l_velocities[0], l_vel_row.size()) = l_vel_row;
                         this->handVelocityNorm_task_left.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(2,l_posture,l_velocities);
                     }
                     step++;
                 }
             }
         }

         // compute njs, nmu and planning time
         if(arm_code!=0){
             //single-arm
             // njs
             double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
             double mean_njs = ((double)sum_njs) / this->njs_task.size();
             string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
             boost::replace_all(mean_njs_str,",",".");
             double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
             double stdev_njs = std::sqrt((((double)sq_sum_njs) / this->njs_task.size()) - pow(mean_njs,2));
             string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
             boost::replace_all(stdev_njs_str,",",".");
             ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));
             // nmu
             double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
             double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
             string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
             boost::replace_all(mean_nmu_str,",",".");
             double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
             double stdev_nmu = std::sqrt((((double)sq_sum_nmu) / this->nmu_task.size()) - pow(mean_nmu,2));
             string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
             boost::replace_all(stdev_nmu_str,",",".");
             ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));
         }else{
             //dual-arm
             // right njs
             double sum_r_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
             double mean_r_njs = ((double)sum_r_njs) / this->njs_task.size();
             string mean_r_njs_str =  boost::str(boost::format("%.2f") % (mean_r_njs));
             boost::replace_all(mean_r_njs_str,",",".");
             double sq_sum_r_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
             double stdev_r_njs = std::sqrt((((double)sq_sum_r_njs) / this->njs_task.size()) - pow(mean_r_njs,2));
             string stdev_r_njs_str =  boost::str(boost::format("%.2f") % (stdev_r_njs));
             boost::replace_all(stdev_r_njs_str,",",".");
             ui.label_cost_hand_right_value_task->setText(QString::fromStdString(mean_r_njs_str)+QString("(")+QString::fromStdString(stdev_r_njs_str)+QString(")"));

             // left njs
             double sum_l_njs = std::accumulate(this->njs_task_left.begin(), this->njs_task_left.end(), 0.0);
             double mean_l_njs = ((double)sum_l_njs) / this->njs_task_left.size();
             string mean_l_njs_str =  boost::str(boost::format("%.2f") % (mean_l_njs));
             boost::replace_all(mean_l_njs_str,",",".");
             double sq_sum_l_njs = std::inner_product(this->njs_task_left.begin(), this->njs_task_left.end(), this->njs_task_left.begin(), 0.0);
             double stdev_l_njs = std::sqrt((((double)sq_sum_l_njs) / this->njs_task_left.size()) - pow(mean_l_njs,2));
             string stdev_l_njs_str =  boost::str(boost::format("%.2f") % (stdev_l_njs));
             boost::replace_all(stdev_l_njs_str,",",".");
             ui.label_cost_hand_left_value_task->setText(QString::fromStdString(mean_l_njs_str)+QString("(")+QString::fromStdString(stdev_l_njs_str)+QString(")"));

             // right nmu
             double sum_r_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
             double mean_r_nmu = ((double)sum_r_nmu) / this->nmu_task.size();
             string mean_r_nmu_str =  boost::str(boost::format("%.2f") % (mean_r_nmu));
             boost::replace_all(mean_r_nmu_str,",",".");
             double sq_sum_r_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
             double stdev_r_nmu = std::sqrt((((double)sq_sum_r_nmu) / this->nmu_task.size()) - pow(mean_r_nmu,2));
             string stdev_r_nmu_str =  boost::str(boost::format("%.2f") % (stdev_r_nmu));
             boost::replace_all(stdev_r_nmu_str,",",".");
             ui.label_nmu_hand_right_value_task->setText(QString::fromStdString(mean_r_nmu_str)+QString("(")+QString::fromStdString(stdev_r_nmu_str)+QString(")"));

             // left nmu
             double sum_l_nmu = std::accumulate(this->nmu_task_left.begin(), this->nmu_task_left.end(), 0.0);
             double mean_l_nmu = ((double)sum_l_nmu) / this->nmu_task_left.size();
             string mean_l_nmu_str =  boost::str(boost::format("%.2f") % (mean_l_nmu));
             boost::replace_all(mean_l_nmu_str,",",".");
             double sq_sum_l_nmu = std::inner_product(this->nmu_task_left.begin(), this->nmu_task_left.end(), this->nmu_task_left.begin(), 0.0);
             double stdev_l_nmu = std::sqrt((((double)sq_sum_l_nmu) / this->nmu_task_left.size()) - pow(mean_l_nmu,2));
             string stdev_l_nmu_str =  boost::str(boost::format("%.2f") % (stdev_l_nmu));
             boost::replace_all(stdev_l_nmu_str,",",".");
             ui.label_nmu_hand_left_value_task->setText(QString::fromStdString(mean_l_nmu_str)+QString("(")+QString::fromStdString(stdev_l_nmu_str)+QString(")"));

         }
         // planning time
         double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
         double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
         string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
         boost::replace_all(mean_prob_str,",",".");
         double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
         double stdev_prob = std::sqrt((((double)sq_sum_prob) / this->prob_time_task.size()) - pow(mean_prob,2));
         string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
         boost::replace_all(stdev_prob_str,",",".");
         if(arm_code!=0){
            ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
         }else{
             ui.label_solv_time_task_dual_value->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
         }
    } // if the problem has been solved
}


void MainWindow::on_pushButton_clear_task_clicked()
{

    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->jointsAcceleration_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsPosition_mov.clear();
    this->timesteps_task.clear();
    this->timesteps_mov.clear();
    this->tols_stop_task.clear();
    this->tols_stop_mov.clear();

    this->handPosition_mov.clear(); this->handPosition_mov_left.clear();
    this->handOrientation_mov.clear(); this->handOrientation_mov_left.clear();
    this->handLinearVelocity_mov.clear(); this->handLinearVelocity_mov_left.clear();
    this->handAngularVelocity_mov.clear(); this->handAngularVelocity_mov_left.clear();
    this->handVelocityNorm_mov.clear(); this->handVelocityNorm_mov_left.clear();
    this->handPosition_task.clear(); this->handPosition_task_left.clear();
    this->handOrientation_task.clear(); this->handOrientation_task_left.clear();
    this->handLinearVelocity_task.clear(); this->handLinearVelocity_task_left.clear();
    this->handAngularVelocity_task.clear(); this->handAngularVelocity_task_left.clear();
    this->handVelocityNorm_task.clear(); this->handVelocityNorm_task_left.clear();
    this->nmu_task.clear(); this->nmu_task_left.clear();
    this->njs_task.clear(); this->njs_task_left.clear();

    this->prob_time_task.clear();

    ui.tableWidget_sol_task->clear();
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->curr_task->clearProblems();


}


void MainWindow::on_comboBox_Task_currentIndexChanged(int i)
{

   switch (i){

       case 0:
           // Single-arm task
           ui.radioButton_right->setEnabled(true);
           ui.radioButton_left->setEnabled(true);
           ui.comboBox_mov_left->setEnabled(false);
           ui.label_type_left->setEnabled(false);

           ui.label_type->setText(QString("Type of Movement"));
           ui.label_objects->setText(QString("Involved object"));
           ui.label_poses->setText(QString("Involved pose"));
           ui.label_objects_eng->setText(QString("Engaged object"));
           ui.groupBox_grip->setTitle(QString("Type of grip"));

           ui.label_objects_left->setEnabled(false);
           ui.comboBox_objects_left->setEnabled(false);
           ui.label_poses_left->setEnabled(false);
           ui.comboBox_poses_left->setEnabled(false);
           ui.groupBox_grip_left->setEnabled(false);
           ui.radioButton_prec_left->setEnabled(false);
           ui.radioButton_full_left->setEnabled(false);

           ui.groupBox_power_law->setEnabled(true);
           ui.pushButton_plan_3d_power_law->setEnabled(true);

       break;

       case 1:
           //Dual-arm task
           ui.radioButton_right->setEnabled(false);
           ui.radioButton_left->setEnabled(false);

           ui.label_type->setText(QString("Type of Movement (right)"));
           ui.label_objects->setText(QString("Involved object (right)"));
           ui.label_poses->setText(QString("Involved pose (right)"));
           ui.label_objects_eng->setText(QString("Engaged object (right)"));
           ui.groupBox_grip->setTitle(QString("Type of grip (right)"));

           ui.comboBox_mov_left->setEnabled(true);
           ui.label_type_left->setEnabled(true);
           ui.label_objects_left->setEnabled(true);
           ui.comboBox_objects_left->setEnabled(true);

           ui.groupBox_grip_left->setEnabled(true);
           ui.radioButton_prec_left->setEnabled(true);
           ui.radioButton_full_left->setEnabled(true);
           ui.groupBox_power_law->setEnabled(false);
           ui.pushButton_plan_3d_power_law->setEnabled(false);

       break;

   }
}


void MainWindow::on_comboBox_mov_currentIndexChanged(int i)
{

    switch (i){

        case 0:
        // Reach-to-grasp
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            ui.comboBox_poses->setEnabled(false);
            ui.label_poses->setEnabled(false);
            break;
        case 1:
        // Reaching
            ui.comboBox_objects->setEnabled(false);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(false);
            ui.groupBox_grip->setEnabled(false);
            ui.comboBox_poses->setEnabled(true);
            ui.label_poses->setEnabled(true);
            break;
        case 2:
        // Transport
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            ui.comboBox_poses->setEnabled(true);
            ui.label_poses->setEnabled(true);
            break;
        case 3:
        //Engage
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(true);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            ui.comboBox_poses->setEnabled(false);
            ui.label_poses->setEnabled(false);
            break;
        case 4:
        //Disengage
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(true);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            ui.comboBox_poses->setEnabled(true);
            ui.label_poses->setEnabled(true);
            break;
        case 5:
        // Go park
            ui.comboBox_objects->setEnabled(false);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(false);
            ui.groupBox_grip->setEnabled(false);
            ui.comboBox_poses->setEnabled(false);
            ui.label_poses->setEnabled(false);
            break;

    }

}

void MainWindow::on_comboBox_mov_left_currentIndexChanged(int i)
{

    switch (i){

        case 0:
        // Reach-to-grasp
            ui.comboBox_objects_left->setEnabled(true);
            ui.comboBox_objects_eng_left->setEnabled(false);
            ui.label_objects_left->setEnabled(true);
            ui.groupBox_grip_left->setEnabled(true);
            ui.comboBox_poses_left->setEnabled(false);
            ui.label_poses_left->setEnabled(false);
            break;
        case 1:
        // Reaching
            ui.comboBox_objects_left->setEnabled(false);
            ui.comboBox_objects_eng_left->setEnabled(false);
            ui.label_objects_left->setEnabled(false);
            ui.groupBox_grip_left->setEnabled(false);
            ui.comboBox_poses_left->setEnabled(true);
            ui.label_poses_left->setEnabled(true);
            break;
        case 2:
        // Transport
            ui.comboBox_objects_left->setEnabled(true);
            ui.comboBox_objects_eng_left->setEnabled(false);
            ui.label_objects_left->setEnabled(true);
            ui.groupBox_grip_left->setEnabled(true);
            ui.comboBox_poses_left->setEnabled(true);
            ui.label_poses_left->setEnabled(true);
            break;
        case 3:
        //Engage
            ui.comboBox_objects_left->setEnabled(true);
            ui.comboBox_objects_eng_left->setEnabled(true);
            ui.label_objects_left->setEnabled(true);
            ui.groupBox_grip_left->setEnabled(true);
            ui.comboBox_poses_left->setEnabled(false);
            ui.label_poses_left->setEnabled(false);
            break;
        case 4:
        //Disengage
            ui.comboBox_objects_left->setEnabled(true);
            ui.comboBox_objects_eng_left->setEnabled(true);
            ui.label_objects_left->setEnabled(true);
            ui.groupBox_grip_left->setEnabled(true);
            ui.comboBox_poses_left->setEnabled(true);
            ui.label_poses_left->setEnabled(true);
            break;
        case 5:
        // Go park
            ui.comboBox_objects_left->setEnabled(false);
            ui.comboBox_objects_eng_left->setEnabled(false);
            ui.label_objects_left->setEnabled(false);
            ui.groupBox_grip_left->setEnabled(false);
            ui.comboBox_poses_left->setEnabled(false);
            ui.label_poses_left->setEnabled(false);
            break;

    }

}


void MainWindow::onListScenarioItemClicked(QListWidgetItem *item)
{

    ui.pushButton_loadScenario->setEnabled(true);

#if HAND==0

    for(int i=0; i<ui.listWidget_scenario->size().height(); ++i){
        if (ui.listWidget_scenario->item(i)== item) {
            switch(i){

            case(0):
                // Assembly scenario: the Toy vehicle with Jarde
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "Jarde has to assemble a toy vehicle on a table in front of him"));

                break;

            case(1):
                //Assistive scenario: beverages with Jarde

                break;

            case(2):
                //Organizing scenario: shelfs and objects with Jarde

                break;

            }

        }

    }

#elif HAND==1

    for(int i=0; i<ui.listWidget_scenario->size().height(); ++i){
        if (ui.listWidget_scenario->item(i)== item) {
            switch(i){

            case 0:
                // Assembly scenario: the Toy vehicle with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS has to assemble a toy vehicle on a table in front of him"));
                break;
            case 1:
                //Empty scenario: empty with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS moves in a empty workspace"));
                break;

            case 2:
                //Empty scenario: empty with ARoS and No collisions
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS moves in a empty workspace and no collisions are taken into account"));
                break;

            case 3:
                //Human assistance scenario: beverages with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS serves a drink to a human patient"));
                break;

            case 4:
                //Challenging scenario: picking a cup from a shelf with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS picks and places a cup on a narrow shelf"));
                break;

            case 5:
                // Assembly scenario: swap the two columns of the toy vehcile
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS picks and swaps the two columns of the toy vehicle."));

                break;

            case 6:
                // Human assistance scenario: Moving a tray with ARoS (dual-arms)
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS is moving a tray with both hands in a kitchen ."));

                break;

            case 7:
                // Natural obstacle avoidance with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "This scenario is designed to study the human-likeness of the obstacle avoidance approach."));

                break;

            case 8:
                // Learning Tasks: reaching with one obstacle
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS learns reaching movements with one obstacle in the scenario."));

                break;

            case 9:
                // Learning Tasks: reaching with many obstacles
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS learns reaching movements with many obstacles in the scenario."));

                break;

            case 10:
                // Learning Tasks: picking the blue column
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS learns picking movements with no objects in the scenario."));

                break;

            case 11:
                // Controlling: scenario with no objects
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS controls movements with no objects in the scenario."));

                break;

            }

        }

    }
#endif

}


void MainWindow::on_pushButton_plot_mov_clicked()
{
    // plot the 3D hand position
    this->handPosPlot_mov_ptr.reset(new HandPosPlot(this->handPosition_mov));
    this->handPosPlot_mov_ptr->setParent(this->ui.plot_hand_pos_mov);
    this->handPosPlot_mov_ptr->resize(522,329);
    this->handPosPlot_mov_ptr->show();

    // plot the hand velocity norm
    if(!this->handVelocityNorm_mov.empty()){
        QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_mov);
        ui.plot_hand_vel_mov->plotLayout()->clear();
        ui.plot_hand_vel_mov->clearGraphs();
        ui.plot_hand_vel_mov->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_mov);
        wideAxisRect->setupFullAxesBox(true);
        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_mov);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_hand_vel_mov->axisRects())
        {
          for (QCPAxis *axis : rect->axes())
          {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
          }
        }
        QString title("Hand velocity");
        ui.plot_hand_vel_mov->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_mov,title));
        ui.plot_hand_vel_mov->plotLayout()->addElement(1, 0, wideAxisRect);

        ui.plot_hand_vel_mov->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_hand_vel_mov->graph(0)->setPen(QPen(Qt::red));
        ui.plot_hand_vel_mov->graph(0)->setName(title);
        ui.plot_hand_vel_mov->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_hand_vel_mov->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_hand_vel_mov->graph(0)->setData(this->qtime_mov, qhand_vel);
        ui.plot_hand_vel_mov->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                          *std::max_element(qhand_vel.begin(), qhand_vel.end()));
        ui.plot_hand_vel_mov->graph(0)->rescaleAxes();
        ui.plot_hand_vel_mov->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_hand_vel_mov->replot();
    }else{
        ui.plot_hand_vel_mov->plotLayout()->clear();
        ui.plot_hand_vel_mov->clearGraphs();
    }
}

void MainWindow::on_pushButton_plot_mov_dual_clicked()
{
    // plot the 3D right hand position
    this->handPosPlot_mov_ptr.reset(new HandPosPlot(this->handPosition_mov));
    this->handPosPlot_mov_ptr->setParent(this->ui.plot_hand_pos_mov_right);
    this->handPosPlot_mov_ptr->resize(421,214);
    this->handPosPlot_mov_ptr->set_title("Right Hand position [mm]");
    this->handPosPlot_mov_ptr->show();

    // plot the 3D left hand position
    this->handPosPlot_mov_left_ptr.reset(new HandPosPlot(this->handPosition_mov_left));
    this->handPosPlot_mov_left_ptr->setParent(this->ui.plot_hand_pos_mov_left);
    this->handPosPlot_mov_left_ptr->resize(421,214);
    this->handPosPlot_mov_left_ptr->set_title("Left Hand position [mm]");
    this->handPosPlot_mov_left_ptr->show();

    // plot the right hand velocity norm
    if(!this->handVelocityNorm_mov.empty()){
        QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_mov);
        ui.plot_hand_vel_mov_right->plotLayout()->clear();
        ui.plot_hand_vel_mov_right->clearGraphs();
        ui.plot_hand_vel_mov_right->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_mov_right);
        wideAxisRect->setupFullAxesBox(true);
        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_mov_right);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_hand_vel_mov_right->axisRects())
        {
          for (QCPAxis *axis : rect->axes())
          {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
          }
        }
        QString title("Right Hand velocity");
        ui.plot_hand_vel_mov_right->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_mov_right,title));
        ui.plot_hand_vel_mov_right->plotLayout()->addElement(1, 0, wideAxisRect);

        ui.plot_hand_vel_mov_right->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_hand_vel_mov_right->graph(0)->setPen(QPen(Qt::red));
        ui.plot_hand_vel_mov_right->graph(0)->setName(title);
        ui.plot_hand_vel_mov_right->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_hand_vel_mov_right->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_hand_vel_mov_right->graph(0)->setData(this->qtime_mov, qhand_vel);
        ui.plot_hand_vel_mov_right->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                          *std::max_element(qhand_vel.begin(), qhand_vel.end()));
        ui.plot_hand_vel_mov_right->graph(0)->rescaleAxes();
        ui.plot_hand_vel_mov_right->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_hand_vel_mov_right->replot();
    }else{
        ui.plot_hand_vel_mov_right->plotLayout()->clear();
        ui.plot_hand_vel_mov_right->clearGraphs();
    }

    // plot the left hand velocity norm
    if(!this->handVelocityNorm_mov_left.empty()){
        QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_mov_left);
        ui.plot_hand_vel_mov_left->plotLayout()->clear();
        ui.plot_hand_vel_mov_left->clearGraphs();
        ui.plot_hand_vel_mov_left->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_mov_left);
        wideAxisRect->setupFullAxesBox(true);
        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_mov_left);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_hand_vel_mov_left->axisRects())
        {
          for (QCPAxis *axis : rect->axes())
          {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
          }
        }
        QString title("Left Hand velocity");
        ui.plot_hand_vel_mov_left->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_mov_left,title));
        ui.plot_hand_vel_mov_left->plotLayout()->addElement(1, 0, wideAxisRect);

        ui.plot_hand_vel_mov_left->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_hand_vel_mov_left->graph(0)->setPen(QPen(Qt::red));
        ui.plot_hand_vel_mov_left->graph(0)->setName(title);
        ui.plot_hand_vel_mov_left->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_hand_vel_mov_left->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_hand_vel_mov_left->graph(0)->setData(this->qtime_mov, qhand_vel);
        ui.plot_hand_vel_mov_left->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                          *std::max_element(qhand_vel.begin(), qhand_vel.end()));
        ui.plot_hand_vel_mov_left->graph(0)->rescaleAxes();
        ui.plot_hand_vel_mov_left->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_hand_vel_mov_left->replot();
    }else{
        ui.plot_hand_vel_mov_left->plotLayout()->clear();
        ui.plot_hand_vel_mov_left->clearGraphs();
    }

}

void MainWindow::on_pushButton_plot_task_clicked()
{
    if(!this->handPosition_task.empty() && !this->handVelocityNorm_task.empty())
    {
        // plot the 3D hand position
        this->handPosPlot_task_ptr.reset(new HandPosPlot(this->handPosition_task));
        this->handPosPlot_task_ptr->setParent(this->ui.plot_hand_pos_task);
        this->handPosPlot_task_ptr->resize(522,329);
        this->handPosPlot_task_ptr->show();

        // plot the hand velocity norm
        if(!this->handVelocityNorm_task.empty()){
            QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_task);
            ui.plot_hand_vel_task->plotLayout()->clear();
            ui.plot_hand_vel_task->clearGraphs();
            ui.plot_hand_vel_task->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_task);
            wideAxisRect->setupFullAxesBox(true);
            QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_task);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui.plot_hand_vel_task->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            QString title("Hand velocity");
            ui.plot_hand_vel_task->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_task,title));
            ui.plot_hand_vel_task->plotLayout()->addElement(1, 0, wideAxisRect);

            ui.plot_hand_vel_task->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui.plot_hand_vel_task->graph(0)->setPen(QPen(Qt::red));
            ui.plot_hand_vel_task->graph(0)->setName(title);
            ui.plot_hand_vel_task->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
            ui.plot_hand_vel_task->graph(0)->keyAxis()->setLabel("time [s]");
            ui.plot_hand_vel_task->graph(0)->setData(this->qtime_task, qhand_vel);
            ui.plot_hand_vel_task->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                              *std::max_element(qhand_vel.begin(), qhand_vel.end()));
            ui.plot_hand_vel_task->graph(0)->rescaleAxes();
            ui.plot_hand_vel_task->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui.plot_hand_vel_task->replot();
        }else{
            ui.plot_hand_vel_task->plotLayout()->clear();
            ui.plot_hand_vel_task->clearGraphs();
        }
    }
}

void MainWindow::on_pushButton_plot_task_dual_clicked()
{
    if(!this->handPosition_task.empty() && !this->handPosition_task_left.empty() && !this->handVelocityNorm_task.empty()&& !this->handVelocityNorm_task_left.empty())
    {
        // plot the 3D right hand position
        this->handPosPlot_task_ptr.reset(new HandPosPlot(this->handPosition_task));
        this->handPosPlot_task_ptr->setParent(this->ui.plot_hand_pos_task_right);
        this->handPosPlot_task_ptr->resize(421,214);
        this->handPosPlot_task_ptr->set_title("Right Hand position [mm]");
        this->handPosPlot_task_ptr->show();

        // plot the 3D left hand position
        this->handPosPlot_task_left_ptr.reset(new HandPosPlot(this->handPosition_task_left));
        this->handPosPlot_task_left_ptr->setParent(this->ui.plot_hand_pos_task_left);
        this->handPosPlot_task_left_ptr->resize(421,214);
        this->handPosPlot_task_left_ptr->set_title("Left Hand position [mm]");
        this->handPosPlot_task_left_ptr->show();

        // plot the right hand velocity norm
        if(!this->handVelocityNorm_task.empty()){
            QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_task);
            ui.plot_hand_vel_task_right->plotLayout()->clear();
            ui.plot_hand_vel_task_right->clearGraphs();
            ui.plot_hand_vel_task_right->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_task_right);
            wideAxisRect->setupFullAxesBox(true);
            QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_task_right);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui.plot_hand_vel_task_right->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            QString title("Right Hand velocity");
            ui.plot_hand_vel_task_right->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_task_right,title));
            ui.plot_hand_vel_task_right->plotLayout()->addElement(1, 0, wideAxisRect);

            ui.plot_hand_vel_task_right->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui.plot_hand_vel_task_right->graph(0)->setPen(QPen(Qt::red));
            ui.plot_hand_vel_task_right->graph(0)->setName(title);
            ui.plot_hand_vel_task_right->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
            ui.plot_hand_vel_task_right->graph(0)->keyAxis()->setLabel("time [s]");
            ui.plot_hand_vel_task_right->graph(0)->setData(this->qtime_task, qhand_vel);
            ui.plot_hand_vel_task_right->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                              *std::max_element(qhand_vel.begin(), qhand_vel.end()));
            ui.plot_hand_vel_task_right->graph(0)->rescaleAxes();
            ui.plot_hand_vel_task_right->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui.plot_hand_vel_task_right->replot();
        }else{
            ui.plot_hand_vel_task_right->plotLayout()->clear();
            ui.plot_hand_vel_task_right->clearGraphs();
        }

        // plot the left hand velocity norm
        if(!this->handVelocityNorm_task_left.empty()){
            QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_task_left);
            ui.plot_hand_vel_task_left->plotLayout()->clear();
            ui.plot_hand_vel_task_left->clearGraphs();
            ui.plot_hand_vel_task_left->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_task_left);
            wideAxisRect->setupFullAxesBox(true);
            QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_task_left);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui.plot_hand_vel_task_left->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            QString title("Left Hand velocity");
            ui.plot_hand_vel_task_left->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_task_left,title));
            ui.plot_hand_vel_task_left->plotLayout()->addElement(1, 0, wideAxisRect);

            ui.plot_hand_vel_task_left->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui.plot_hand_vel_task_left->graph(0)->setPen(QPen(Qt::red));
            ui.plot_hand_vel_task_left->graph(0)->setName(title);
            ui.plot_hand_vel_task_left->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
            ui.plot_hand_vel_task_left->graph(0)->keyAxis()->setLabel("time [s]");
            ui.plot_hand_vel_task_left->graph(0)->setData(this->qtime_task, qhand_vel);
            ui.plot_hand_vel_task_left->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                              *std::max_element(qhand_vel.begin(), qhand_vel.end()));
            ui.plot_hand_vel_task_left->graph(0)->rescaleAxes();
            ui.plot_hand_vel_task_left->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui.plot_hand_vel_task_left->replot();
        }else{
            ui.plot_hand_vel_task_left->plotLayout()->clear();
            ui.plot_hand_vel_task_left->clearGraphs();
        }
    }
}

void MainWindow::on_pushButton_joints_results_mov_clicked()
{
    if(!this->jointsPosition_mov.empty())
    {
        this->mResultsJointsdlg->setDual(false);this->mResultsJointsdlg->setRight(true);
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_mov,this->jointsVelocity_mov,this->jointsAcceleration_mov,this->timesteps_mov);
    }

    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_mov_right_clicked()
{
    if(!this->jointsPosition_mov.empty())
    {
        this->mResultsJointsdlg->setDual(true);this->mResultsJointsdlg->setRight(true);
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_mov,this->jointsVelocity_mov,this->jointsAcceleration_mov,this->timesteps_mov);
    }

    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_mov_left_clicked()
{
    if(!this->jointsPosition_mov.empty())
    {
        this->mResultsJointsdlg->setDual(true);this->mResultsJointsdlg->setRight(false);
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_mov,this->jointsVelocity_mov,this->jointsAcceleration_mov,this->timesteps_mov);
    }

    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_task_clicked()
{
    if(!this->jointsPosition_task.empty())
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_task,this->jointsVelocity_task,this->jointsAcceleration_task,this->timesteps_task);
    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_task_right_clicked()
{
    if(!this->jointsPosition_task.empty())
    {
        this->mResultsJointsdlg->setDual(true);this->mResultsJointsdlg->setRight(true);
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_task,this->jointsVelocity_task,this->jointsAcceleration_task,this->timesteps_task);
    }

    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_task_left_clicked()
{
    if(!this->jointsPosition_task.empty())
    {
        this->mResultsJointsdlg->setDual(true);this->mResultsJointsdlg->setRight(false);
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_task,this->jointsVelocity_task,this->jointsAcceleration_task,this->timesteps_task);
    }

    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_power_law_clicked()
{
    if(!this->handPosition_task.empty())
        this->mPowerLawdlg->setupPlots(this->handPosition_task,this->timesteps_task);
    this->mPowerLawdlg->show();
}

void MainWindow::on_pushButton_power_law_3D_clicked()
{
    if(!this->handPosition_task.empty())
        this->mPowerLaw3Ddlg->setupPlots(this->handPosition_task,this->timesteps_task);
    this->mPowerLaw3Ddlg->show();
}

void MainWindow::on_pushButton_nat_coll_av_clicked()
{
    if(!this->handPosition_task.empty())
        this->mNatCollAvdlg->setupPlots(this->handLinearVelocity_task,this->handPosition_task,this->timesteps_task);
    this->mNatCollAvdlg->show();
}

void MainWindow::on_pushButton_comp_vel_mov_clicked()
{
    if(!this->shoulderLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->shoulderPosition_mov,this->shoulderOrientation_mov,
                                      this->shoulderLinearVelocity_mov,this->shoulderAngularVelocity_mov,
                                      this->shoulderLinearAcceleration_mov,this->shoulderAngularAcceleration_mov,
                                      this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->elbowPosition_mov,this->elbowOrientation_mov,
                                      this->elbowLinearVelocity_mov,this->elbowAngularVelocity_mov,
                                      this->elbowLinearAcceleration_mov,this->elbowAngularAcceleration_mov,
                                      this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wristPosition_mov,this->wristOrientation_mov,
                                      this->wristLinearVelocity_mov,this->wristAngularVelocity_mov,
                                      this->wristLinearAcceleration_mov,this->wristAngularAcceleration_mov,
                                      this->qtime_mov,2);
    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handPosition_mov,this->handOrientation_mov,
                                      this->handLinearVelocity_mov,this->handAngularVelocity_mov,
                                      this->handLinearAcceleration_mov,this->handAngularAcceleration_mov,
                                      this->qtime_mov,3);

    this->mCompVeldlg->setDual(false); this->mCompVeldlg->setRight(true);
    this->mCompVeldlg->show();
}

void MainWindow::on_pushButton_warm_start_res_clicked()
{

    this->mWarmdlg->enablePlanData(false);
    this->mWarmdlg->enableApproachData(false);
    this->mWarmdlg->enableRetreatData(false);
    this->mWarmdlg->enableBounceData(false);

    if(!this->final_warm_start_res_mov.empty())
    {

        if(this->final_warm_start_res_mov.size()==3)
        {
            this->mWarmdlg->enablePlanData(true);
            HUMotion::warm_start_params plan_tar = this->final_warm_start_res_mov.at(1);
            this->mWarmdlg->setPlanData(plan_tar.iterations,plan_tar.cpu_time,plan_tar.obj_value,plan_tar.x,plan_tar.zL,plan_tar.zU,plan_tar.dual_vars,this->warm_n_steps_mov);

            this->mWarmdlg->enableApproachData(true);
            HUMotion::warm_start_params approach_tar = this->final_warm_start_res_mov.at(0);
            this->mWarmdlg->setApproachData(approach_tar.iterations,approach_tar.cpu_time,approach_tar.obj_value,approach_tar.x,approach_tar.zL,approach_tar.zU,approach_tar.dual_vars);

            this->mWarmdlg->enableRetreatData(true);
            HUMotion::warm_start_params retreat_tar = this->final_warm_start_res_mov.at(2);
            this->mWarmdlg->setRetreatData(retreat_tar.iterations,retreat_tar.cpu_time,retreat_tar.obj_value,retreat_tar.x,retreat_tar.zL,retreat_tar.zU,retreat_tar.dual_vars);

        }else if(this->final_warm_start_res_mov.size()==2){

            if(this->traj_descr_mov.at(1).compare("approach")==0)
            {
                this->mWarmdlg->enableApproachData(true);
                HUMotion::warm_start_params approach_tar = this->final_warm_start_res_mov.at(0);
                this->mWarmdlg->setApproachData(approach_tar.iterations,approach_tar.cpu_time,approach_tar.obj_value,approach_tar.x,approach_tar.zL,approach_tar.zU,approach_tar.dual_vars);

                this->mWarmdlg->enablePlanData(true);
                HUMotion::warm_start_params plan_tar = this->final_warm_start_res_mov.at(1);
                this->mWarmdlg->setPlanData(plan_tar.iterations,plan_tar.cpu_time,plan_tar.obj_value,plan_tar.x,plan_tar.zL,plan_tar.zU,plan_tar.dual_vars,this->warm_n_steps_mov);

            }else{

                this->mWarmdlg->enablePlanData(true);
                HUMotion::warm_start_params plan_tar = this->final_warm_start_res_mov.at(0);
                this->mWarmdlg->setPlanData(plan_tar.iterations,plan_tar.cpu_time,plan_tar.obj_value,plan_tar.x,plan_tar.zL,plan_tar.zU,plan_tar.dual_vars,this->warm_n_steps_mov);

                this->mWarmdlg->enableRetreatData(true);
                HUMotion::warm_start_params retreat_tar = this->final_warm_start_res_mov.at(1);
                this->mWarmdlg->setRetreatData(retreat_tar.iterations,retreat_tar.cpu_time,retreat_tar.obj_value,retreat_tar.x,retreat_tar.zL,retreat_tar.zU,retreat_tar.dual_vars);

            }
        }else{
            this->mWarmdlg->enablePlanData(true);
            HUMotion::warm_start_params plan_tar = this->final_warm_start_res_mov.at(0);
            this->mWarmdlg->setPlanData(plan_tar.iterations,plan_tar.cpu_time,plan_tar.obj_value,plan_tar.x,plan_tar.zL,plan_tar.zU,plan_tar.dual_vars,this->warm_n_steps_mov);
        }
    }

    // bounce posture data
    if (this->bounce_warm_start_res_mov.valid)
    {
        this->mWarmdlg->enableBounceData(true);
        this->mWarmdlg->setBounceData(this->bounce_warm_start_res_mov.iterations,this->bounce_warm_start_res_mov.cpu_time,this->bounce_warm_start_res_mov.obj_value,this->bounce_warm_start_res_mov.x,this->bounce_warm_start_res_mov.zL,this->bounce_warm_start_res_mov.zU,this->bounce_warm_start_res_mov.dual_vars);
    }else{
        this->mWarmdlg->enableBounceData(false);
    }

    this->mWarmdlg->show();
}

void MainWindow::on_pushButton_comp_vel_mov_right_clicked()
{
    if(!this->shoulderLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->shoulderPosition_mov,this->shoulderOrientation_mov,
                                      this->shoulderLinearVelocity_mov,this->shoulderAngularVelocity_mov,
                                      this->shoulderLinearAcceleration_mov,this->shoulderAngularAcceleration_mov,
                                      this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->elbowPosition_mov,this->elbowOrientation_mov,
                                      this->elbowLinearVelocity_mov,this->elbowAngularVelocity_mov,
                                      this->elbowLinearAcceleration_mov,this->elbowAngularAcceleration_mov,
                                      this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wristPosition_mov,this->wristOrientation_mov,
                                      this->wristLinearVelocity_mov,this->wristAngularVelocity_mov,
                                      this->wristLinearAcceleration_mov,this->wristAngularAcceleration_mov,
                                      this->qtime_mov,2);
    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handPosition_mov,this->handOrientation_mov,
                                      this->handLinearVelocity_mov,this->handAngularVelocity_mov,
                                      this->handLinearAcceleration_mov,this->handAngularAcceleration_mov,
                                      this->qtime_mov,3);

    this->mCompVeldlg->setDual(true); this->mCompVeldlg->setRight(true);
    this->mCompVeldlg->show();
}

void MainWindow::on_pushButton_comp_vel_mov_left_clicked()
{

    if(!this->shoulderLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->shoulderPosition_mov_left,this->shoulderOrientation_mov_left,
                                      this->shoulderLinearVelocity_mov_left,this->shoulderAngularVelocity_mov_left,
                                      this->shoulderLinearAcceleration_mov_left,this->shoulderAngularAcceleration_mov_left,
                                      this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->elbowPosition_mov_left,this->elbowOrientation_mov_left,
                                      this->elbowLinearVelocity_mov_left,this->elbowAngularVelocity_mov_left,
                                      this->elbowLinearAcceleration_mov_left,this->elbowAngularAcceleration_mov_left,
                                      this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->wristPosition_mov_left,this->wristOrientation_mov_left,
                                      this->wristLinearVelocity_mov_left,this->wristAngularVelocity_mov_left,
                                      this->wristLinearAcceleration_mov_left,this->wristAngularAcceleration_mov_left,
                                      this->qtime_mov,2);
    if(!this->handLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->handPosition_mov_left,this->handOrientation_mov_left,
                                      this->handLinearVelocity_mov_left,this->handAngularVelocity_mov_left,
                                      this->handLinearAcceleration_mov_left,this->handAngularAcceleration_mov_left,
                                      this->qtime_mov,3);

    this->mCompVeldlg->setDual(true); this->mCompVeldlg->setRight(false);
    this->mCompVeldlg->show();
}


void MainWindow::on_pushButton_save_res_mov_clicked()
{

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/mov", &st) == -1) {
        mkdir("results/planning/mov", 0700);
    }
    QString path("results/planning/mov/");
    ui.plot_hand_vel_mov->savePdf(path+QString("hand_vel_mov.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));

    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");
    string hand_pos_file = path.toStdString()+string("hand_pos_mov.pdf");
    IO::save(this->handPosPlot_mov_ptr.get(), hand_pos_file.c_str(),  "PDF" );

    // results
    string filename("results_mov.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# NORMALIZED JERK SCORE \n");
    string njs_str =  boost::str(boost::format("%.2f") % (this->njs_mov));
    boost::replace_all(njs_str,",",".");
    results << string("njs =")+njs_str+string(";\n");

    results << string("# NUMBER OF MOVEMENT UNITS \n");
    string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_mov));
    boost::replace_all(nmu_str,",",".");
    results << string("nmu =")+nmu_str+string(";\n");

    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [ms] \n");
    string time_str =  boost::str(boost::format("%.2f") % (this->prob_time_mov));
    boost::replace_all(time_str,",",".");
    results << string("prob_time =")+time_str+string(";\n");

    results.close();

    // hand position
    if(!this->handPosition_mov.empty()){
        string filename_hand_pos("hand_pos_mov.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_mov.size();++i){
            vector<double> point = this->handPosition_mov.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }
    // hand velocity
    if(!this->handVelocityNorm_mov.empty()){
        string filename_hand_vel("hand_vel_mov.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_mov.size();++i){
            double vel = this->handVelocityNorm_mov.at(i);
            double time = this->qtime_mov.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_mov.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_mov.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_mov.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_mov.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());
}

void MainWindow::on_pushButton_save_res_mov_dual_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/mov", &st) == -1) {
        mkdir("results/planning/mov", 0700);
    }
    QString path("results/planning/mov/");
    ui.plot_hand_vel_mov_right->savePdf(path+QString("hand_vel_mov_right.pdf"),true,0,0,QString(),QString("Module of the Right Hand velocity"));
    ui.plot_hand_vel_mov_left->savePdf(path+QString("hand_vel_mov_left.pdf"),true,0,0,QString(),QString("Module of the Left Hand velocity"));

    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");
    string right_hand_pos_file = path.toStdString()+string("hand_pos_mov_right.pdf");
    string left_hand_pos_file = path.toStdString()+string("hand_pos_mov_left.pdf");
    IO::save(this->handPosPlot_mov_ptr.get(), right_hand_pos_file.c_str(),  "PDF" );
    IO::save(this->handPosPlot_mov_left_ptr.get(), left_hand_pos_file.c_str(),  "PDF" );

    // results
    string filename("results_mov_dual.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# RIGHT NORMALIZED JERK SCORE \n");
    string njs_right_str =  boost::str(boost::format("%.2f") % (this->njs_mov));
    boost::replace_all(njs_right_str,",",".");
    results << string("njs_right =")+njs_right_str+string(";\n");

    results << string("# LEFT NORMALIZED JERK SCORE \n");
    string njs_left_str =  boost::str(boost::format("%.2f") % (this->njs_mov_left));
    boost::replace_all(njs_left_str,",",".");
    results << string("njs_left =")+njs_left_str+string(";\n");

    results << string("# RIGHT NUMBER OF MOVEMENT UNITS \n");
    string nmu_right_str =  boost::str(boost::format("%.2f") % (this->nmu_mov));
    boost::replace_all(nmu_right_str,",",".");
    results << string("nmu_right =")+nmu_right_str+string(";\n");

    results << string("# LEFT NUMBER OF MOVEMENT UNITS \n");
    string nmu_left_str =  boost::str(boost::format("%.2f") % (this->nmu_mov_left));
    boost::replace_all(nmu_left_str,",",".");
    results << string("nmu_left =")+nmu_left_str+string(";\n");

    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [ms] \n");
    string time_str =  boost::str(boost::format("%.2f") % (this->prob_time_mov));
    boost::replace_all(time_str,",",".");
    results << string("prob_time =")+time_str+string(";\n");

    results.close();

    // right hand position
    if(!this->handPosition_mov.empty()){
        string filename_hand_pos("hand_pos_mov_right.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i < this->handPosition_mov.size();++i){
            vector<double> point = this->handPosition_mov.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // left hand position
    if(!this->handPosition_mov_left.empty()){
        string filename_hand_pos("hand_pos_mov_left.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i < this->handPosition_mov_left.size();++i){
            vector<double> point = this->handPosition_mov_left.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // right hand velocity
    if(!this->handVelocityNorm_mov.empty()){
        string filename_hand_vel("hand_vel_mov_right.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_mov.size();++i){
            double vel = this->handVelocityNorm_mov.at(i);
            double time = this->qtime_mov.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    // left hand velocity
    if(!this->handVelocityNorm_mov_left.empty()){
        string filename_hand_vel("hand_vel_mov_left.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_mov_left.size();++i){
            double vel = this->handVelocityNorm_mov_left.at(i);
            double time = this->qtime_mov.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_mov_right.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_mov_right.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_mov_right.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_mov_right.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_pos_mov_left.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_mov_left.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_mov_left.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_mov_left.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

}

void MainWindow::on_pushButton_save_res_task_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/task", &st) == -1) {
        mkdir("results/planning/task", 0700);
    }
    QString path("results/planning/task/");

    // txt
    string filename("results_task.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# NORMALIZED JERK SCORE \n");
    results << string("njs = ");
    for(size_t i=0;i<this->njs_task.size();++i){
        string njs_str =  boost::str(boost::format("%.2f") % (this->njs_task.at(i)));
        boost::replace_all(njs_str,",",".");
        if(i==(this->njs_task.size()-1)){
            results << njs_str+" \n";
        }else{
            results << njs_str+" ";
        }
    }
    // mean
    double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
    double mean_njs = ((double)sum_njs) / this->njs_task.size();
    string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
    boost::replace_all(mean_njs_str,",",".");
    results << string("mean njs = ")+mean_njs_str+string(" \n");
    // standard deviation
    double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
    double stdev_njs = std::sqrt(((double)sq_sum_njs) / this->njs_task.size() - mean_njs * mean_njs);
    string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
    boost::replace_all(stdev_njs_str,",",".");
    results << string("sd njs = ")+stdev_njs_str+string(" \n");
    //median
    double median_njs = this->getMedian(this->njs_task);
    string median_njs_str =  boost::str(boost::format("%.2f") % (median_njs));
    boost::replace_all(median_njs_str,",",".");
    results << string("median njs = ")+median_njs_str+string(" \n");
    // 1st quartile
    double first_quartile_njs = this->getFirstQuartile(this->njs_task);
    string first_quartile_njs_str =  boost::str(boost::format("%.2f") % (first_quartile_njs));
    boost::replace_all(first_quartile_njs_str,",",".");
    results << string("first quartile njs = ")+first_quartile_njs_str+string(" \n");
    // 3rd quartile
    double third_quartile_njs = this->getThirdQuartile(this->njs_task);
    string third_quartile_njs_str =  boost::str(boost::format("%.2f") % (third_quartile_njs));
    boost::replace_all(third_quartile_njs_str,",",".");
    results << string("third quartile njs = ")+third_quartile_njs_str+string(" \n");

    results << string("# NUMBER OF MOVEMENT UNITS \n");
    results << string("nmu = ");
    for(size_t i=0;i<this->nmu_task.size();++i){
        string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_task.at(i)));
        boost::replace_all(nmu_str,",",".");
        if(i==(this->nmu_task.size()-1)){
            results << nmu_str+" \n";
        }else{
            results << nmu_str+" ";
        }
    }
    // mean
    double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
    double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
    string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
    boost::replace_all(mean_nmu_str,",",".");
    results << string("mean nmu = ")+mean_nmu_str+string(" \n");
    // standard deviation
    double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
    double stdev_nmu = std::sqrt(((double)sq_sum_nmu) / this->nmu_task.size() - mean_nmu * mean_nmu);
    string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
    boost::replace_all(stdev_nmu_str,",",".");
    results << string("sd nmu = ")+stdev_nmu_str+string(" \n");
    //median
    double median_nmu = this->getMedian(this->nmu_task);
    string median_nmu_str =  boost::str(boost::format("%.2f") % (median_nmu));
    boost::replace_all(median_nmu_str,",",".");
    results << string("median nmu = ")+median_nmu_str+string(" \n");
    // 1st quartile
    double first_quartile_nmu = this->getFirstQuartile(this->nmu_task);
    string first_quartile_nmu_str =  boost::str(boost::format("%.2f") % (first_quartile_nmu));
    boost::replace_all(first_quartile_nmu_str,",",".");
    results << string("first quartile nmu = ")+first_quartile_nmu_str+string(" \n");
    // 3rd quartile
    double third_quartile_nmu = this->getThirdQuartile(this->nmu_task);
    string third_quartile_nmu_str =  boost::str(boost::format("%.2f") % (third_quartile_nmu));
    boost::replace_all(third_quartile_nmu_str,",",".");
    results << string("third quartile nmu = ")+third_quartile_nmu_str+string(" \n");


    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [ms] \n");
    results << string("prob_time = ");
    for(size_t i=0;i<this->prob_time_task.size();++i){
        string prob_str =  boost::str(boost::format("%.2f") % (this->prob_time_task.at(i)));
        boost::replace_all(prob_str,",",".");
        if(i==(this->prob_time_task.size()-1)){
            results << prob_str+" \n";
        }else{
            results << prob_str+" ";
        }
    }
    // mean
    double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
    double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
    string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
    boost::replace_all(mean_prob_str,",",".");
    results << string("mean plan time = ")+mean_prob_str+string(" \n");
    // standard deviation
    double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
    double stdev_prob = std::sqrt(((double)sq_sum_prob) / this->prob_time_task.size() - mean_prob * mean_prob);
    string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
    boost::replace_all(stdev_prob_str,",",".");
    results << string("sd plan time = ")+stdev_prob_str+string(" \n");
    //median
    double median_prob = this->getMedian(this->prob_time_task);
    string median_prob_str =  boost::str(boost::format("%.2f") % (median_prob));
    boost::replace_all(median_prob_str,",",".");
    results << string("median prob = ")+median_prob_str+string(" \n");
    // 1st quartile
    double first_quartile_prob = this->getFirstQuartile(this->prob_time_task);
    string first_quartile_prob_str =  boost::str(boost::format("%.2f") % (first_quartile_prob));
    boost::replace_all(first_quartile_prob_str,",",".");
    results << string("first quartile plan time = ")+first_quartile_prob_str+string(" \n");
    // 3rd quartile
    double third_quartile_prob = this->getThirdQuartile(this->prob_time_task);
    string third_quartile_prob_str =  boost::str(boost::format("%.2f") % (third_quartile_prob));
    boost::replace_all(third_quartile_prob_str,",",".");
    results << string("third quartile plan time = ")+third_quartile_prob_str+string(" \n");

    string rate_success = ui.label_rate_task->text().toStdString();
    results << string("rate of success [%] = ")+rate_success+string(" \n");


    results.close();

    // csv
    string filename_csv("results_task.csv");
    ofstream results_csv;
    results_csv.open(path.toStdString()+filename_csv);
    results_csv << "TRAJ,NJS,NMU,PLANNING TIME [ms] \n";
    for(size_t i=0;i<this->njs_task.size();++i){
        string njs_str =  boost::str(boost::format("%.8f") % (this->njs_task.at(i)));
        string nmu_str =  boost::str(boost::format("%.8f") % (this->nmu_task.at(i)));
        string prob_str =  boost::str(boost::format("%.8f") % (this->prob_time_task.at(i)));
        boost::replace_all(njs_str,",","."); boost::replace_all(nmu_str,",","."); boost::replace_all(prob_str,",",".");
        results_csv << QString::number(i+1).toStdString()+","+njs_str+","+nmu_str+","+prob_str+" \n";
    }
    results_csv.close();

    ui.plot_hand_vel_task->savePdf(path+QString("hand_vel_task.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));

    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");
    string hand_pos_file = path.toStdString()+string("hand_pos_task.pdf");
    if(this->handPosPlot_task_ptr!=nullptr){
        IO::save(this->handPosPlot_task_ptr.get(), hand_pos_file.c_str(),  "PDF" );
    }

    // hand position
    if(!this->handPosition_task.empty()){
        string filename_hand_pos("hand_pos_task.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_task.size();++i){
            vector<double> point = this->handPosition_task.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // hand velocity
    if(!this->handVelocityNorm_task.empty()){
        string filename_hand_vel("hand_vel_task.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_task.size();++i){
            double vel = this->handVelocityNorm_task.at(i);
            double time = this->qtime_task.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }


    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_task.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_task.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_task.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_task.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());


}

void MainWindow::on_pushButton_save_res_task_dual_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/task", &st) == -1) {
        mkdir("results/planning/task", 0700);
    }
    QString path("results/planning/task/");

    // txt
    string filename("results_task_dual.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# RIGHT NORMALIZED JERK SCORE \n");
    results << string("njs_right = ");
    for(size_t i=0;i<this->njs_task.size();++i){
        string njs_str =  boost::str(boost::format("%.2f") % (this->njs_task.at(i)));
        boost::replace_all(njs_str,",",".");
        if(i==(this->njs_task.size()-1)){
            results << njs_str+" \n";
        }else{
            results << njs_str+" ";
        }
    }
    // mean
    double sum_r_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
    double mean_r_njs = ((double)sum_r_njs) / this->njs_task.size();
    string mean_r_njs_str =  boost::str(boost::format("%.2f") % (mean_r_njs));
    boost::replace_all(mean_r_njs_str,",",".");
    results << string("mean njs_right = ")+mean_r_njs_str+string(" \n");
    // standard deviation
    double sq_sum_r_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
    double stdev_r_njs = std::sqrt(((double)sq_sum_r_njs) / this->njs_task.size() - mean_r_njs * mean_r_njs);
    string stdev_r_njs_str =  boost::str(boost::format("%.2f") % (stdev_r_njs));
    boost::replace_all(stdev_r_njs_str,",",".");
    results << string("sd njs_right = ")+stdev_r_njs_str+string(" \n");
    //median
    double median_r_njs = this->getMedian(this->njs_task);
    string median_r_njs_str =  boost::str(boost::format("%.2f") % (median_r_njs));
    boost::replace_all(median_r_njs_str,",",".");
    results << string("median njs_right = ")+median_r_njs_str+string(" \n");
    // 1st quartile
    double first_quartile_r_njs = this->getFirstQuartile(this->njs_task);
    string first_quartile_r_njs_str =  boost::str(boost::format("%.2f") % (first_quartile_r_njs));
    boost::replace_all(first_quartile_r_njs_str,",",".");
    results << string("first quartile njs_right = ")+first_quartile_r_njs_str+string(" \n");
    // 3rd quartile
    double third_quartile_r_njs = this->getThirdQuartile(this->njs_task);
    string third_quartile_r_njs_str =  boost::str(boost::format("%.2f") % (third_quartile_r_njs));
    boost::replace_all(third_quartile_r_njs_str,",",".");
    results << string("third quartile njs_right = ")+third_quartile_r_njs_str+string(" \n");

    results << string("# LEFT NORMALIZED JERK SCORE \n");
    results << string("njs_left = ");
    for(size_t i=0;i<this->njs_task_left.size();++i){
        string njs_str =  boost::str(boost::format("%.2f") % (this->njs_task_left.at(i)));
        boost::replace_all(njs_str,",",".");
        if(i==(this->njs_task_left.size()-1)){
            results << njs_str+" \n";
        }else{
            results << njs_str+" ";
        }
    }
    // mean
    double sum_l_njs = std::accumulate(this->njs_task_left.begin(), this->njs_task_left.end(), 0.0);
    double mean_l_njs = ((double)sum_l_njs) / this->njs_task_left.size();
    string mean_l_njs_str =  boost::str(boost::format("%.2f") % (mean_l_njs));
    boost::replace_all(mean_l_njs_str,",",".");
    results << string("mean njs_left = ")+mean_l_njs_str+string(" \n");
    // standard deviation
    double sq_sum_l_njs = std::inner_product(this->njs_task_left.begin(), this->njs_task_left.end(), this->njs_task_left.begin(), 0.0);
    double stdev_l_njs = std::sqrt(((double)sq_sum_l_njs) / this->njs_task_left.size() - mean_l_njs * mean_l_njs);
    string stdev_l_njs_str =  boost::str(boost::format("%.2f") % (stdev_l_njs));
    boost::replace_all(stdev_l_njs_str,",",".");
    results << string("sd njs_left = ")+stdev_l_njs_str+string(" \n");
    //median
    double median_l_njs = this->getMedian(this->njs_task_left);
    string median_l_njs_str =  boost::str(boost::format("%.2f") % (median_l_njs));
    boost::replace_all(median_l_njs_str,",",".");
    results << string("median njs_left = ")+median_l_njs_str+string(" \n");
    // 1st quartile
    double first_quartile_l_njs = this->getFirstQuartile(this->njs_task_left);
    string first_quartile_l_njs_str =  boost::str(boost::format("%.2f") % (first_quartile_l_njs));
    boost::replace_all(first_quartile_l_njs_str,",",".");
    results << string("first quartile njs_left = ")+first_quartile_l_njs_str+string(" \n");
    // 3rd quartile
    double third_quartile_l_njs = this->getThirdQuartile(this->njs_task_left);
    string third_quartile_l_njs_str =  boost::str(boost::format("%.2f") % (third_quartile_l_njs));
    boost::replace_all(third_quartile_l_njs_str,",",".");
    results << string("third quartile njs_left = ")+third_quartile_l_njs_str+string(" \n");

    results << string("# RIGHT NUMBER OF MOVEMENT UNITS \n");
    results << string("nmu_right = ");
    for(size_t i=0;i<this->nmu_task.size();++i){
        string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_task.at(i)));
        boost::replace_all(nmu_str,",",".");
        if(i==(this->nmu_task.size()-1)){
            results << nmu_str+" \n";
        }else{
            results << nmu_str+" ";
        }
    }
    // mean
    double sum_r_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
    double mean_r_nmu = ((double)sum_r_nmu) / this->nmu_task.size();
    string mean_r_nmu_str =  boost::str(boost::format("%.2f") % (mean_r_nmu));
    boost::replace_all(mean_r_nmu_str,",",".");
    results << string("mean nmu_right = ")+mean_r_nmu_str+string(" \n");
    // standard deviation
    double sq_sum_r_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
    double stdev_r_nmu = std::sqrt(((double)sq_sum_r_nmu) / this->nmu_task.size() - mean_r_nmu * mean_r_nmu);
    string stdev_r_nmu_str =  boost::str(boost::format("%.2f") % (stdev_r_nmu));
    boost::replace_all(stdev_r_nmu_str,",",".");
    results << string("sd nmu_right = ")+stdev_r_nmu_str+string(" \n");
    //median
    double median_r_nmu = this->getMedian(this->nmu_task);
    string median_r_nmu_str =  boost::str(boost::format("%.2f") % (median_r_nmu));
    boost::replace_all(median_r_nmu_str,",",".");
    results << string("median nmu_right = ")+median_r_nmu_str+string(" \n");
    // 1st quartile
    double first_quartile_r_nmu = this->getFirstQuartile(this->nmu_task);
    string first_quartile_r_nmu_str =  boost::str(boost::format("%.2f") % (first_quartile_r_nmu));
    boost::replace_all(first_quartile_r_nmu_str,",",".");
    results << string("first quartile nmu_right = ")+first_quartile_r_nmu_str+string(" \n");
    // 3rd quartile
    double third_quartile_r_nmu = this->getThirdQuartile(this->nmu_task);
    string third_quartile_r_nmu_str =  boost::str(boost::format("%.2f") % (third_quartile_r_nmu));
    boost::replace_all(third_quartile_r_nmu_str,",",".");
    results << string("third quartile nmu_right = ")+third_quartile_r_nmu_str+string(" \n");

    results << string("# LEFT NUMBER OF MOVEMENT UNITS \n");
    results << string("nmu_left = ");
    for(size_t i=0;i<this->nmu_task_left.size();++i){
        string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_task_left.at(i)));
        boost::replace_all(nmu_str,",",".");
        if(i==(this->nmu_task_left.size()-1)){
            results << nmu_str+" \n";
        }else{
            results << nmu_str+" ";
        }
    }
    // mean
    double sum_l_nmu = std::accumulate(this->nmu_task_left.begin(), this->nmu_task_left.end(), 0.0);
    double mean_l_nmu = ((double)sum_l_nmu) / this->nmu_task_left.size();
    string mean_l_nmu_str =  boost::str(boost::format("%.2f") % (mean_l_nmu));
    boost::replace_all(mean_l_nmu_str,",",".");
    results << string("mean nmu_left = ")+mean_l_nmu_str+string(" \n");
    // standard deviation
    double sq_sum_l_nmu = std::inner_product(this->nmu_task_left.begin(), this->nmu_task_left.end(), this->nmu_task_left.begin(), 0.0);
    double stdev_l_nmu = std::sqrt(((double)sq_sum_l_nmu) / this->nmu_task_left.size() - mean_l_nmu * mean_l_nmu);
    string stdev_l_nmu_str =  boost::str(boost::format("%.2f") % (stdev_l_nmu));
    boost::replace_all(stdev_l_nmu_str,",",".");
    results << string("sd nmu_left = ")+stdev_l_nmu_str+string(" \n");
    //median
    double median_l_nmu = this->getMedian(this->nmu_task_left);
    string median_l_nmu_str =  boost::str(boost::format("%.2f") % (median_l_nmu));
    boost::replace_all(median_l_nmu_str,",",".");
    results << string("median nmu_left = ")+median_l_nmu_str+string(" \n");
    // 1st quartile
    double first_quartile_l_nmu = this->getFirstQuartile(this->nmu_task_left);
    string first_quartile_l_nmu_str =  boost::str(boost::format("%.2f") % (first_quartile_l_nmu));
    boost::replace_all(first_quartile_l_nmu_str,",",".");
    results << string("first quartile nmu_left = ")+first_quartile_l_nmu_str+string(" \n");
    // 3rd quartile
    double third_quartile_l_nmu = this->getThirdQuartile(this->nmu_task_left);
    string third_quartile_l_nmu_str =  boost::str(boost::format("%.2f") % (third_quartile_l_nmu));
    boost::replace_all(third_quartile_l_nmu_str,",",".");
    results << string("third quartile nmu_left = ")+third_quartile_l_nmu_str+string(" \n");


    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [ms] \n");
    results << string("prob_time = ");
    for(size_t i=0;i<this->prob_time_task.size();++i){
        string prob_str =  boost::str(boost::format("%.2f") % (this->prob_time_task.at(i)));
        boost::replace_all(prob_str,",",".");
        if(i==(this->prob_time_task.size()-1)){
            results << prob_str+" \n";
        }else{
            results << prob_str+" ";
        }
    }
    // mean
    double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
    double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
    string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
    boost::replace_all(mean_prob_str,",",".");
    results << string("mean plan time = ")+mean_prob_str+string(" \n");
    // standard deviation
    double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
    double stdev_prob = std::sqrt(((double)sq_sum_prob) / this->prob_time_task.size() - mean_prob * mean_prob);
    string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
    boost::replace_all(stdev_prob_str,",",".");
    results << string("sd plan time = ")+stdev_prob_str+string(" \n");
    //median
    double median_prob = this->getMedian(this->prob_time_task);
    string median_prob_str =  boost::str(boost::format("%.2f") % (median_prob));
    boost::replace_all(median_prob_str,",",".");
    results << string("median prob = ")+median_prob_str+string(" \n");
    // 1st quartile
    double first_quartile_prob = this->getFirstQuartile(this->prob_time_task);
    string first_quartile_prob_str =  boost::str(boost::format("%.2f") % (first_quartile_prob));
    boost::replace_all(first_quartile_prob_str,",",".");
    results << string("first quartile plan time = ")+first_quartile_prob_str+string(" \n");
    // 3rd quartile
    double third_quartile_prob = this->getThirdQuartile(this->prob_time_task);
    string third_quartile_prob_str =  boost::str(boost::format("%.2f") % (third_quartile_prob));
    boost::replace_all(third_quartile_prob_str,",",".");
    results << string("third quartile plan time = ")+third_quartile_prob_str+string(" \n");

    string rate_success = ui.label_rate_task->text().toStdString();
    results << string("rate of success [%] = ")+rate_success+string(" \n");


    results.close();

    // right csv
    string filename_csv_right("results_task_right.csv");
    ofstream results_csv_right;
    results_csv_right.open(path.toStdString()+filename_csv_right);
    results_csv_right << "TRAJ,NJS,NMU,PLANNING TIME [ms] \n";
    for(size_t i=0;i < this->njs_task.size();++i){
        string njs_str =  boost::str(boost::format("%.8f") % (this->njs_task.at(i)));
        string nmu_str =  boost::str(boost::format("%.8f") % (this->nmu_task.at(i)));
        string prob_str =  boost::str(boost::format("%.8f") % (this->prob_time_task.at(i)));
        boost::replace_all(njs_str,",","."); boost::replace_all(nmu_str,",","."); boost::replace_all(prob_str,",",".");
        results_csv_right << QString::number(i+1).toStdString()+","+njs_str+","+nmu_str+","+prob_str+" \n";
    }
    results_csv_right.close();

    // left csv
    string filename_csv_left("results_task_left.csv");
    ofstream results_csv_left;
    results_csv_left.open(path.toStdString()+filename_csv_left);
    results_csv_left << "TRAJ,NJS,NMU,PLANNING TIME [ms] \n";
    for(size_t i=0;i<this->njs_task_left.size();++i){
        string njs_str =  boost::str(boost::format("%.8f") % (this->njs_task_left.at(i)));
        string nmu_str =  boost::str(boost::format("%.8f") % (this->nmu_task_left.at(i)));
        string prob_str =  boost::str(boost::format("%.8f") % (this->prob_time_task.at(i)));
        boost::replace_all(njs_str,",","."); boost::replace_all(nmu_str,",","."); boost::replace_all(prob_str,",",".");
        results_csv_left << QString::number(i+1).toStdString()+","+njs_str+","+nmu_str+","+prob_str+" \n";
    }
    results_csv_left.close();

    ui.plot_hand_vel_task_right->savePdf(path+QString("hand_vel_task_right.pdf"),true,0,0,QString(),QString("Module of the Right Hand velocity"));
    ui.plot_hand_vel_task_left->savePdf(path+QString("hand_vel_task_left.pdf"),true,0,0,QString(),QString("Module of the Left Hand velocity"));


    VectorWriter* r_handler = (VectorWriter*)IO::outputHandler("PDF");
    r_handler->setTextMode(VectorWriter::NATIVE);
    r_handler->setFormat("PDF");
    string r_hand_pos_file = path.toStdString()+string("hand_pos_task_right.pdf");
    if(this->handPosPlot_task_ptr!=nullptr){
        IO::save(this->handPosPlot_task_ptr.get(), r_hand_pos_file.c_str(),  "PDF" );
    }
    VectorWriter* l_handler = (VectorWriter*)IO::outputHandler("PDF");
    l_handler->setTextMode(VectorWriter::NATIVE);
    l_handler->setFormat("PDF");
    string l_hand_pos_file = path.toStdString()+string("hand_pos_task_left.pdf");
    if(this->handPosPlot_task_left_ptr!=nullptr){
        IO::save(this->handPosPlot_task_left_ptr.get(), l_hand_pos_file.c_str(),  "PDF" );
    }

    // right hand position
    if(!this->handPosition_task.empty()){
        string filename_hand_pos("hand_pos_task_right.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# RIGHT HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_task.size();++i){
            vector<double> point = this->handPosition_task.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // left hand position
    if(!this->handPosition_task_left.empty()){
        string filename_hand_pos("hand_pos_task_left.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# LEFT HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_task_left.size();++i){
            vector<double> point = this->handPosition_task_left.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // right hand velocity
    if(!this->handVelocityNorm_task.empty()){
        string filename_hand_vel("hand_vel_task_right.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# RIGHT HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_task.size();++i){
            double vel = this->handVelocityNorm_task.at(i);
            double time = this->qtime_task.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    // left hand velocity
    if(!this->handVelocityNorm_task_left.empty()){
        string filename_hand_vel("hand_vel_task_left.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# LEFT HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_task_left.size();++i){
            double vel = this->handVelocityNorm_task_left.at(i);
            double time = this->qtime_task.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_task_right.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_task_right.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_pos_task_left.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_task_left.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_task_right.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_task_right.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel_task_left.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_task_left.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());


}

// -----------------------------------------------------
// Learning

void MainWindow::on_pushButton_load_learn_prim_duals_clicked()
{
    this->sol_loaded = false;
    this->ui.label_loaded->setText("Unloaded");
    this->ui.label_trials->setEnabled(false);
    this->ui.lineEdit_trials->setEnabled(false);
    this->ui.pushButton_plan_collect->setEnabled(false);
    this->ui.label_n_predictions->setEnabled(false);
    this->ui.lineEdit_n_predictions->setEnabled(false);
    this->ui.pushButton_pred_plan->setEnabled(false);

    this->sol_plan = false;
    this->sol_approach = false;
    this->sol_retreat = false;
    this->sol_bounce = false;
    // plan
    x_plan.clear(); zL_plan.clear(); zU_plan.clear(); dual_plan.clear();
    // approach
    x_approach.clear(); zL_approach.clear(); zU_approach.clear(); dual_approach.clear();
    // retreat
    x_retreat.clear(); zL_retreat.clear(); zU_retreat.clear(); dual_retreat.clear();
    // bounce
    x_bounce.clear(); zL_bounce.clear(); zU_bounce.clear(); dual_bounce.clear();

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the solution of the original problem"),
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
                    //this->ui->tabWidget_warm_start->setTabEnabled(0,true);
                    this->sol_plan = true;
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
                    //this->ui->tabWidget_warm_start->setTabEnabled(1,true);
                    this->sol_approach = true;
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
                    //this->ui->tabWidget_warm_start->setTabEnabled(2,true);
                    this->sol_retreat = true;
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
                    //this->ui->tabWidget_warm_start->setTabEnabled(3,true);
                    this->sol_bounce = true;
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
        this->sol_loaded = true;
        this->ui.label_loaded->setText("Loaded");
        this->ui.label_trials->setEnabled(true);
        this->ui.lineEdit_trials->setEnabled(true);
        this->ui.pushButton_plan_collect->setEnabled(true);
        this->ui.label_n_predictions->setEnabled(true);
        this->ui.lineEdit_n_predictions->setEnabled(true);
        this->ui.pushButton_pred_plan->setEnabled(true);
    }
}

void MainWindow::on_pushButton_plan_collect_pressed()
{
    qnode.log(QNode::Info,std::string("Planning and collecting started"));
    qnode.log(QNode::Info,std::string("Planning and collecting . . . "));
}

void MainWindow::on_pushButton_plan_collect_clicked()
{
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    int arm_sel = prob->getMovement()->getArm();
    int mov_type = prob->getMovement()->getType();
    string collect_dir = this->ui.lineEdit_collections->text().toStdString();
    HUMotion::hump_params  tols;
    std::vector<double> move_target;
    std::vector<double> move_target_mod;
    std::vector<double> move_final_hand;
    std::vector<double> move_final_arm;
    bool use_final;
    HUMotion::planning_result_ptr h_results_tmp;
    int trials = ui.lineEdit_trials->text().toInt();
    ui.pushButton_plan_collect->setCheckable(false);

    QList<QListWidgetItem*> obsts_items = this->ui.listWidget_obsts_except->selectedItems();
    vector<string> obsts_except;
    for(int ii=0;ii<obsts_items.count();++ii){
        obsts_except.push_back(obsts_items.at(ii)->text().toStdString());
    }

    try{
        switch(planner_id){
        case 0: // HUMP
            if(arm_sel!=0)
            {// single-arm
                mTolHumpdlg->setInfo(prob->getInfoLine());
                // --- Tolerances for the final posture selection ---- //
                tols.tolTarPos = mTolHumpdlg->getTolTarPos(); // target position tolerances
                tols.tolTarOr = mTolHumpdlg->getTolTarOr(); // target orientation tolerances
                mTolHumpdlg->getTolsArm(tols.tolsArm);// tolerances of the arm : radius in [mm]
                mTolHumpdlg->getTolsHand(tols.tolsHand);// tolerances of the hand: radius in [mm]
                tols.target_avoidance = mTolHumpdlg->getTargetAvoidance();// target avoidance
                tols.obstacle_avoidance = mTolHumpdlg->getObstacleAvoidance(); //obstacle avoidance
                mTolHumpdlg->getLambda(tols.lambda_final); // joint expense factors
                mTolHumpdlg->getLambda(tols.lambda_bounce); // joint expense factors
                // --- Tolerances for the bounce posture selection ---- //
                tols.w_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getWMax()*M_PI/180)); // max joint velocity
                tols.alpha_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration
                mTolHumpdlg->getInitVel(tols.bounds.vel_0); // initial velocity
                mTolHumpdlg->getFinalVel(tols.bounds.vel_f); // final velocity
                mTolHumpdlg->getInitAcc(tols.bounds.acc_0); // initial acceleration
                mTolHumpdlg->getFinalAcc(tols.bounds.acc_f); // final acceleration
                // tolerances for the obstacles
                mTolHumpdlg->getTolsObstacles(tols.final_tolsObstacles); // final posture tols
                tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
                tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1));
                mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
                mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));
                // tolerances for the target
                tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
                tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
                tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
                mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
                tols.singleArm_tolsTarget.at(1) = tols.singleArm_tolsTarget.at(0)/100;
                tols.singleArm_tolsTarget.at(2) = 0*tols.singleArm_tolsTarget.at(0);
                //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(1));
                //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(2));
                // pick / place settings
                tols.mov_specs.approach = mTolHumpdlg->getApproach();
                tols.mov_specs.retreat = mTolHumpdlg->getRetreat();
                mTolHumpdlg->getPreGraspApproach(tols.mov_specs.pre_grasp_approach); // pick approach
                mTolHumpdlg->getPostGraspRetreat(tols.mov_specs.post_grasp_retreat); // pick retreat
                mTolHumpdlg->getPrePlaceApproach(tols.mov_specs.pre_place_approach); // place approach
                mTolHumpdlg->getPostPlaceRetreat(tols.mov_specs.post_place_retreat); // place retreat
                tols.mov_specs.rand_init = mTolHumpdlg->getRandInit(); // random initialization for "plan" stages
                tols.mov_specs.coll = mTolHumpdlg->getColl(); // collisions option
                tols.coll_body = mTolHumpdlg->getCollBody(); // collisions with the body
                tols.mov_specs.straight_line = mTolHumpdlg->get_straight_line(); // hand straight line trajectory
                tols.mov_specs.w_red_app_max = mTolHumpdlg->getW_red_app(); // set the max velocity reduction when approaching
                tols.mov_specs.w_red_ret_max = mTolHumpdlg->getW_red_ret(); // set the max velocity reduction when retreating
                // move settings
                mTolHumpdlg->getTargetMove(move_target);
                move_target_mod.resize(move_target.size());
                mTolHumpdlg->getFinalHand(move_final_hand);
                mTolHumpdlg->getFinalArm(move_final_arm);
                use_final = mTolHumpdlg->get_use_final_posture();
                prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
                tols.mov_specs.use_move_plane = mTolHumpdlg->get_add_plane();
                mTolHumpdlg->getPlaneParameters(tols.mov_specs.plane_params);
                //warm start settings
                tols.mov_specs.warm_start = false;
                // Maximum variations of the target
                double tar_x_var = ui.lineEdit_tar_x_var->text().toDouble();
                double tar_y_var = ui.lineEdit_tar_y_var->text().toDouble();
                double tar_z_var = ui.lineEdit_tar_z_var->text().toDouble();
                double tar_roll_var = ui.lineEdit_tar_roll_var->text().toDouble();
                double tar_pitch_var = ui.lineEdit_tar_pitch_var->text().toDouble();
                double tar_yaw_var = ui.lineEdit_tar_yaw_var->text().toDouble();
                // Maximum variations of the obstacles
                double obsts_x_var = ui.lineEdit_obsts_x_var->text().toDouble();
                double obsts_y_var = ui.lineEdit_obsts_y_var->text().toDouble();
                double obsts_z_var = ui.lineEdit_obsts_z_var->text().toDouble();
                double obsts_roll_var = ui.lineEdit_obsts_roll_var->text().toDouble();
                double obsts_pitch_var = ui.lineEdit_obsts_pitch_var->text().toDouble();
                double obsts_yaw_var = ui.lineEdit_obsts_yaw_var->text().toDouble();
                std::vector<objectPtr> obsts; prob->getObstacles(obsts);     // get the obstacles of the scenario

                bool solved = false;
                // csv
                if(!this->ui.lineEdit_collections->text().isEmpty())
                {
                    string filename_csv("learning_data.csv");
                    ofstream data_csv;
                    data_csv.open(collect_dir+string("/")+filename_csv);
                    // headers
                    data_csv << "target_x_mm,target_y_mm,target_z_mm,target_roll_rad,target_pitch_rad,target_yaw_rad";                    
                    for(size_t j=0;j<obsts.size();++j){
                        data_csv << ",obstacle_"+to_string(j+1)+"_x_mm,obstacle_"+to_string(j+1)+"_y_mm,obstacle_"+to_string(j+1)+"_z_mm,obstacle_"+to_string(j+1)+"_roll_rad,obstacle_"+to_string(j+1)+"_pitch_rad,obstacle_"+to_string(j+1)+"_yaw_rad";
                    }
                    if(mov_type==0){ // reach-to-grasp movement

                        // final plan posture selection
                        data_csv << ",xf_plan_1_rad,xf_plan_2_rad,xf_plan_3_rad,xf_plan_4_rad,xf_plan_5_rad,xf_plan_6_rad,xf_plan_7_rad";
                        data_csv << ",zf_L_plan_1,zf_L_plan_2,zf_L_plan_3,zf_L_plan_4,zf_L_plan_5,zf_L_plan_6,zf_L_plan_7";
                        data_csv << ",zf_U_plan_1,zf_U_plan_2,zf_U_plan_3,zf_U_plan_4,zf_U_plan_5,zf_U_plan_6,zf_U_plan_7";
                        for(size_t j=0;j<dual_plan.size();++j){
                            data_csv << ",dual_f_plan_"+to_string(j);
                        }

                        // final approach posture selection
                        data_csv << ",xf_approach_1_rad,xf_approach_2_rad,xf_approach_3_rad,xf_approach_4_rad,xf_approach_5_rad,xf_approach_6_rad,xf_approach_7_rad";
                        data_csv << ",zf_L_approach_1,zf_L_approach_2,zf_L_approach_3,zf_L_approach_4,zf_L_approach_5,zf_L_approach_6,zf_L_approach_7";
                        data_csv << ",zf_U_approach_1,zf_U_approach_2,zf_U_approach_3,zf_U_approach_4,zf_U_approach_5,zf_U_approach_6,zf_U_approach_7";
                        for(size_t j=0;j<dual_approach.size();++j){
                            data_csv << ",dual_f_approach_"+to_string(j);
                        }

                        // final retreat posture selection
                        data_csv << ",xf_retreat_1_rad,xf_retreat_2_rad,xf_retreat_3_rad,xf_retreat_4_rad,xf_retreat_5_rad,xf_retreat_6_rad,xf_retreat_7_rad";
                        data_csv << ",zf_L_retreat_1,zf_L_retreat_2,zf_L_retreat_3,zf_L_retreat_4,zf_L_retreat_5,zf_L_retreat_6,zf_L_retreat_7";
                        data_csv << ",zf_U_retreat_1,zf_U_retreat_2,zf_U_retreat_3,zf_U_retreat_4,zf_U_retreat_5,zf_U_retreat_6,zf_U_retreat_7";
                        for(size_t j=0;j<dual_retreat.size();++j){
                            data_csv << ",dual_f_retreat_"+to_string(j);
                        }

                        // bounce posture selection
                        data_csv << ",x_bounce_1_rad,x_bounce_2_rad,x_bounce_3_rad,x_bounce_4_rad,x_bounce_5_rad,x_bounce_6_rad,x_bounce_7_rad,x_bounce_8_rad,x_bounce_9_rad";
                        data_csv << ",zb_L_1,zb_L_2,zb_L_3,zb_L_4,zb_L_5,zb_L_6,zb_L_7,zb_L_8,zb_L_9";
                        data_csv << ",zb_U_1,zb_U_2,zb_U_3,zb_U_4,zb_U_5,zb_U_6,zb_U_7,zb_U_8,zb_U_9";
                        for(size_t j=0;j<dual_bounce.size();++j){
                            data_csv << ",dual_bounce_"+to_string(j);
                        }

                    }else if(mov_type==1 || mov_type==5){ // move movement

                        // final posture selection
                        data_csv << ",xf_plan_1_rad,xf_plan_2_rad,xf_plan_3_rad,xf_plan_4_rad,xf_plan_5_rad,xf_plan_6_rad,xf_plan_7_rad";
                        data_csv << ",zf_L_plan_1,zf_L_plan_2,zf_L_plan_3,zf_L_plan_4,zf_L_plan_5,zf_L_plan_6,zf_L_plan_7";
                        data_csv << ",zf_U_plan_1,zf_U_plan_2,zf_U_plan_3,zf_U_plan_4,zf_U_plan_5,zf_U_plan_6,zf_U_plan_7";
                        for(size_t j=0;j<dual_plan.size();++j){
                            data_csv << ",dual_f_plan_"+to_string(j);
                        }

                        // bounce posture selection
                        data_csv << ",x_bounce_1_rad,x_bounce_2_rad,x_bounce_3_rad,x_bounce_4_rad,x_bounce_5_rad,x_bounce_6_rad,x_bounce_7_rad,x_bounce_8_rad,x_bounce_9_rad";
                        data_csv << ",zb_L_1,zb_L_2,zb_L_3,zb_L_4,zb_L_5,zb_L_6,zb_L_7,zb_L_8,zb_L_9";
                        data_csv << ",zb_U_1,zb_U_2,zb_U_3,zb_U_4,zb_U_5,zb_U_6,zb_U_7,zb_U_8,zb_U_9";
                        for(size_t j=0;j<dual_bounce.size();++j){
                            data_csv << ",dual_bounce_"+to_string(j);
                        }

                    }
                    data_csv << " \n";
                    objectPtr obj_tar_or; // the original object involved in the movement
                    if(mov_type==0){ // reach-to-grasp movement
                        obj_tar_or = prob->getMovement()->getObject();
                    }

                    for(int i=0;i<trials;++i){
                        solved = false;
                        do{
                            sleep(1);
                            qnode.log(QNode::Info,std::string("Trial: ")+to_string(i));
                            std::srand(std::time(NULL));
                            if(mov_type==0){ // reach-to-grasp movement
                                objectPtr obj_tar = objectPtr(new Object(*(obj_tar_or.get())));
                                motion_manager::pos obj_pos;
                                motion_manager::orient obj_or;
                                if(this->ui.lineEdit_tar_x_var->isEnabled()){
                                    obj_pos.Xpos = obj_tar_or->getPos().Xpos - (tar_x_var/2) + tar_x_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_pos.Xpos = obj_tar_or->getPos().Xpos;
                                }
                                if(this->ui.lineEdit_tar_y_var->isEnabled()){
                                    obj_pos.Ypos = obj_tar_or->getPos().Ypos - (tar_y_var/2) + tar_y_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_pos.Ypos = obj_tar_or->getPos().Ypos;
                                }
                                if(this->ui.lineEdit_tar_z_var->isEnabled()){
                                    obj_pos.Zpos = obj_tar_or->getPos().Zpos - (tar_z_var/2) + tar_z_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_pos.Zpos = obj_tar_or->getPos().Zpos;
                                }
                                if(this->ui.lineEdit_tar_roll_var->isEnabled()){
                                    obj_or.roll = obj_tar_or->getOr().roll - (tar_roll_var/2) + tar_roll_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_or.roll = obj_tar_or->getOr().roll;
                                }
                                if(this->ui.lineEdit_tar_pitch_var->isEnabled()){
                                    obj_or.pitch = obj_tar_or->getOr().pitch - (tar_pitch_var/2) + tar_pitch_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_or.pitch = obj_tar_or->getOr().pitch;
                                }
                                if(this->ui.lineEdit_tar_yaw_var->isEnabled()){
                                    obj_or.yaw = obj_tar_or->getOr().yaw - (tar_yaw_var/2) + tar_yaw_var*(rand() / double(RAND_MAX));
                                }else{
                                    obj_or.yaw = obj_tar_or->getOr().yaw;
                                }
                                obj_tar->setPos(obj_pos,true);
                                obj_tar->setOr(obj_or,true);
                                prob->getMovement()->setObject(obj_tar);

                            }else if(mov_type==1 || mov_type==5){ // move movement
                                // modify target data (move_target)
                                if(this->ui.lineEdit_tar_x_var->isEnabled()){
                                    move_target_mod.at(0) = move_target.at(0) - (tar_x_var/2) + tar_x_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(0) = move_target.at(0);
                                }
                                if(this->ui.lineEdit_tar_y_var->isEnabled()){
                                    move_target_mod.at(1) = move_target.at(1) - (tar_y_var/2) + tar_y_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(1) = move_target.at(1);
                                }
                                if(this->ui.lineEdit_tar_z_var->isEnabled()){
                                    move_target_mod.at(2) = move_target.at(2) - (tar_z_var/2) + tar_z_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(2) = move_target.at(2);
                                }
                                if(this->ui.lineEdit_tar_roll_var->isEnabled()){
                                    move_target_mod.at(3) = move_target.at(3) - (tar_roll_var/2) + tar_roll_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(3) = move_target.at(3);
                                }
                                if(this->ui.lineEdit_tar_pitch_var->isEnabled()){
                                    move_target_mod.at(4) = move_target.at(4) - (tar_pitch_var/2) + tar_pitch_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(4) = move_target.at(4);
                                }
                                if(this->ui.lineEdit_tar_yaw_var->isEnabled()){
                                    move_target_mod.at(5) = move_target.at(5) - (tar_yaw_var/2) + tar_yaw_var*(rand() / double(RAND_MAX));
                                }else{
                                    move_target_mod.at(5) = move_target.at(5);
                                }
                                prob->setMoveSettings(move_target_mod,move_final_hand,move_final_arm,use_final);
                            }

                            // modify obstacles data
                            objectPtr obs_new;
                            for(size_t j=0;j<obsts.size();++j){
                                std::srand(std::time(NULL));
                                objectPtr obs = obsts.at(j);
                                string obs_name = obs->getName();
                                if (std::find(obsts_except.begin(), obsts_except.end(), obs_name) != obsts_except.end())
                                {
                                    continue;
                                }
                                obs_new.reset(new Object(obs_name));
                                motion_manager::pos obs_pos;
                                motion_manager::orient obs_or;
                                if(this->ui.lineEdit_obsts_x_var->isEnabled()){
                                    obs_pos.Xpos = obs->getPos().Xpos - (obsts_x_var/2) + obsts_x_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_pos.Xpos = obs->getPos().Xpos;
                                }
                                if(this->ui.lineEdit_obsts_y_var->isEnabled()){
                                    obs_pos.Ypos = obs->getPos().Ypos - (obsts_y_var/2) + obsts_y_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_pos.Ypos = obs->getPos().Ypos;
                                }
                                if(this->ui.lineEdit_obsts_z_var->isEnabled()){
                                    obs_pos.Zpos = obs->getPos().Zpos - (obsts_z_var/2) + obsts_z_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_pos.Zpos = obs->getPos().Zpos;
                                }
                                if(this->ui.lineEdit_obsts_roll_var->isEnabled()){
                                    obs_or.roll = obs->getOr().roll - (obsts_roll_var/2) + obsts_roll_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_or.roll = obs->getOr().roll;
                                }
                                if(this->ui.lineEdit_obsts_pitch_var->isEnabled()){
                                    obs_or.pitch = obs->getOr().pitch - (obsts_pitch_var/2) + obsts_pitch_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_or.pitch = obs->getOr().pitch;
                                }
                                if(this->ui.lineEdit_obsts_yaw_var->isEnabled()){
                                    obs_or.yaw = obs->getOr().yaw - (obsts_yaw_var/2) + obsts_yaw_var*(rand() / double(RAND_MAX));
                                }else{
                                    obs_or.yaw = obs->getOr().yaw;
                                }
                                obs_new->setPos(obs_pos,false);
                                obs_new->setOr(obs_or,false);
                                obs_new->setSize(obs->getSize());
                                prob->setObstacle(obs_new,j);
                            }// for loop obstacles

                            // plan the movement
                            h_results_tmp = prob->solve(tols);
                            if(h_results_tmp!=nullptr){
                                if(h_results_tmp->status==0){
                                    // movement planned successfully
                                    HUMotion::warm_start_params b_res = h_results_tmp->bounce_warm_start_res;
                                    if(b_res.dual_vars.size() == dual_bounce.size())
                                    {
                                        qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
                                        solved=true;
                                        // target
                                        if(mov_type==0){ // reach-to-grasp movement
                                            objectPtr obj_tar = prob->getMovement()->getObject();
                                            string tar_x_pos_str =  boost::str(boost::format("%.8f") % (obj_tar->getPos().Xpos)); boost::replace_all(tar_x_pos_str,",",".");
                                            string tar_y_pos_str =  boost::str(boost::format("%.8f") % (obj_tar->getPos().Ypos)); boost::replace_all(tar_y_pos_str,",",".");
                                            string tar_z_pos_str =  boost::str(boost::format("%.8f") % (obj_tar->getPos().Zpos)); boost::replace_all(tar_z_pos_str,",",".");
                                            string tar_roll_str =  boost::str(boost::format("%.8f") % (obj_tar->getOr().roll)); boost::replace_all(tar_roll_str,",",".");
                                            string tar_pitch_str =  boost::str(boost::format("%.8f") % (obj_tar->getOr().pitch)); boost::replace_all(tar_pitch_str,",",".");
                                            string tar_yaw_str =  boost::str(boost::format("%.8f") % (obj_tar->getOr().yaw)); boost::replace_all(tar_yaw_str,",",".");
                                            data_csv << tar_x_pos_str+","+tar_y_pos_str+","+tar_z_pos_str+","+tar_roll_str+","+tar_pitch_str+","+tar_yaw_str+",";
                                        }else if(mov_type==1 || mov_type==5){ // move movement
                                            string tar_x_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(0))); boost::replace_all(tar_x_pos_str,",",".");
                                            string tar_y_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(1))); boost::replace_all(tar_y_pos_str,",",".");
                                            string tar_z_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(2))); boost::replace_all(tar_z_pos_str,",",".");
                                            string tar_roll_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(3))); boost::replace_all(tar_roll_str,",",".");
                                            string tar_pitch_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(4))); boost::replace_all(tar_pitch_str,",",".");
                                            string tar_yaw_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(5))); boost::replace_all(tar_yaw_str,",",".");
                                            data_csv << tar_x_pos_str+","+tar_y_pos_str+","+tar_z_pos_str+","+tar_roll_str+","+tar_pitch_str+","+tar_yaw_str+",";
                                        }
                                        // obstacles
                                        std::vector<objectPtr> obsts_new; prob->getObstacles(obsts_new);
                                        for(size_t j=0;j<obsts_new.size();++j){
                                            objectPtr obs = obsts_new.at(j);
                                            string obs_x_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Xpos)); boost::replace_all(obs_x_pos_str,",",".");
                                            string obs_y_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Ypos)); boost::replace_all(obs_y_pos_str,",",".");
                                            string obs_z_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Zpos)); boost::replace_all(obs_z_pos_str,",",".");
                                            string obs_roll_str =  boost::str(boost::format("%.8f") % (obs->getOr().roll)); boost::replace_all(obs_roll_str,",",".");
                                            string obs_pitch_str =  boost::str(boost::format("%.8f") % (obs->getOr().pitch)); boost::replace_all(obs_pitch_str,",",".");
                                            string obs_yaw_str =  boost::str(boost::format("%.8f") % (obs->getOr().yaw)); boost::replace_all(obs_yaw_str,",",".");
                                            data_csv << obs_x_pos_str+","+obs_y_pos_str+","+obs_z_pos_str+","+obs_roll_str+","+obs_pitch_str+","+obs_yaw_str+",";
                                        }
                                        // final posture selections
                                        if(h_results_tmp->final_warm_start_res.size()==3){
                                            // plan
                                            HUMotion::warm_start_params f_plan_res = h_results_tmp->final_warm_start_res.at(1);
                                            for(int h=0;h<f_plan_res.x.size();++h){
                                                string x_str =  boost::str(boost::format("%.8f") % (f_plan_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                data_csv << x_str+",";
                                            }
                                            for(size_t h=0;h<f_plan_res.zL.size();++h){
                                                string zL_str =  boost::str(boost::format("%.8f") % (f_plan_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                data_csv << zL_str+",";
                                            }
                                            for(size_t h=0;h<f_plan_res.zU.size();++h){
                                                string zU_str =  boost::str(boost::format("%.8f") % (f_plan_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                data_csv << zU_str+",";
                                            }
                                            for(size_t h=0;h<f_plan_res.dual_vars.size();++h){
                                                string dual_str =  boost::str(boost::format("%.8f") % (f_plan_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                data_csv << dual_str+",";
                                            }
                                            // approach
                                            HUMotion::warm_start_params f_app_res = h_results_tmp->final_warm_start_res.at(0);
                                            for(int h=0;h<f_app_res.x.size();++h){
                                                string x_str =  boost::str(boost::format("%.8f") % (f_app_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                data_csv << x_str+",";
                                            }
                                            for(size_t h=0;h<f_app_res.zL.size();++h){
                                                string zL_str =  boost::str(boost::format("%.8f") % (f_app_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                data_csv << zL_str+",";
                                            }
                                            for(size_t h=0;h<f_app_res.zU.size();++h){
                                                string zU_str =  boost::str(boost::format("%.8f") % (f_app_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                data_csv << zU_str+",";
                                            }
                                            for(size_t h=0;h<f_app_res.dual_vars.size();++h){
                                                string dual_str =  boost::str(boost::format("%.8f") % (f_app_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                data_csv << dual_str+",";
                                            }
                                            //retreat
                                            HUMotion::warm_start_params f_ret_res = h_results_tmp->final_warm_start_res.at(2);
                                            for(int h=0;h<f_ret_res.x.size();++h){
                                                string x_str =  boost::str(boost::format("%.8f") % (f_ret_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                data_csv << x_str+",";
                                            }
                                            for(size_t h=0;h<f_ret_res.zL.size();++h){
                                                string zL_str =  boost::str(boost::format("%.8f") % (f_ret_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                data_csv << zL_str+",";
                                            }
                                            for(size_t h=0;h<f_ret_res.zU.size();++h){
                                                string zU_str =  boost::str(boost::format("%.8f") % (f_ret_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                data_csv << zU_str+",";
                                            }
                                            for(size_t h=0;h<f_ret_res.dual_vars.size();++h){
                                                string dual_str =  boost::str(boost::format("%.8f") % (f_ret_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                data_csv << dual_str+",";
                                            }
                                        }else if(h_results_tmp->final_warm_start_res.size()==2){
                                            if(h_results_tmp->trajectory_descriptions.at(1).compare("approach")==0)
                                            {
                                                // plan
                                                HUMotion::warm_start_params f_plan_res = h_results_tmp->final_warm_start_res.at(1);
                                                for(int h=0;h<f_plan_res.x.size();++h){
                                                    string x_str =  boost::str(boost::format("%.8f") % (f_plan_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                    data_csv << x_str+",";
                                                }
                                                for(size_t h=0;h<f_plan_res.zL.size();++h){
                                                    string zL_str =  boost::str(boost::format("%.8f") % (f_plan_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                    data_csv << zL_str+",";
                                                }
                                                for(size_t h=0;h<f_plan_res.zU.size();++h){
                                                    string zU_str =  boost::str(boost::format("%.8f") % (f_plan_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                    data_csv << zU_str+",";
                                                }
                                                //approach
                                                HUMotion::warm_start_params f_app_res = h_results_tmp->final_warm_start_res.at(0);
                                                for(int h=0;h<f_app_res.x.size();++h){
                                                    string x_str =  boost::str(boost::format("%.8f") % (f_app_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                    data_csv << x_str+",";
                                                }
                                                for(size_t h=0;h<f_app_res.zL.size();++h){
                                                    string zL_str =  boost::str(boost::format("%.8f") % (f_app_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                    data_csv << zL_str+",";
                                                }
                                                for(size_t h=0;h<f_app_res.zU.size();++h){
                                                    string zU_str =  boost::str(boost::format("%.8f") % (f_app_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                    data_csv << zU_str+",";
                                                }
                                                for(size_t h=0;h<f_app_res.dual_vars.size();++h){
                                                    string dual_str =  boost::str(boost::format("%.8f") % (f_app_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                    data_csv << dual_str+",";
                                                }
                                            }else{
                                                // plan
                                                HUMotion::warm_start_params f_plan_res = h_results_tmp->final_warm_start_res.at(0);
                                                for(int h=0;h<f_plan_res.x.size();++h){
                                                    string x_str =  boost::str(boost::format("%.8f") % (f_plan_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                    data_csv << x_str+",";
                                                }
                                                for(size_t h=0;h<f_plan_res.zL.size();++h){
                                                    string zL_str =  boost::str(boost::format("%.8f") % (f_plan_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                    data_csv << zL_str+",";
                                                }
                                                for(size_t h=0;h<f_plan_res.zU.size();++h){
                                                    string zU_str =  boost::str(boost::format("%.8f") % (f_plan_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                    data_csv << zU_str+",";
                                                }
                                                // retreat
                                                HUMotion::warm_start_params f_ret_res = h_results_tmp->final_warm_start_res.at(1);
                                                for(int h=0;h<f_ret_res.x.size();++h){
                                                    string x_str =  boost::str(boost::format("%.8f") % (f_ret_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                    data_csv << x_str+",";
                                                }
                                                for(size_t h=0;h<f_ret_res.zL.size();++h){
                                                    string zL_str =  boost::str(boost::format("%.8f") % (f_ret_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                    data_csv << zL_str+",";
                                                }
                                                for(size_t h=0;h<f_ret_res.zU.size();++h){
                                                    string zU_str =  boost::str(boost::format("%.8f") % (f_ret_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                    data_csv << zU_str+",";
                                                }
                                                for(size_t h=0;h<f_ret_res.dual_vars.size();++h){
                                                    string dual_str =  boost::str(boost::format("%.8f") % (f_ret_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                    data_csv << dual_str+",";
                                                }
                                            }
                                        }else if(h_results_tmp->final_warm_start_res.size()==1){
                                            // plan
                                            HUMotion::warm_start_params f_res = h_results_tmp->final_warm_start_res.at(0);
                                            for(int h=0;h<f_res.x.size();++h){
                                                string x_str =  boost::str(boost::format("%.8f") % (f_res.x.at(h))); boost::replace_all(x_str,",",".");
                                                data_csv << x_str+",";
                                            }
                                            for(size_t h=0;h<f_res.zL.size();++h){
                                                string zL_str =  boost::str(boost::format("%.8f") % (f_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                                data_csv << zL_str+",";
                                            }
                                            for(size_t h=0;h<f_res.zU.size();++h){
                                                string zU_str =  boost::str(boost::format("%.8f") % (f_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                                data_csv << zU_str+",";
                                            }
                                            for(size_t h=0;h<f_res.dual_vars.size();++h){
                                                string dual_str =  boost::str(boost::format("%.8f") % (f_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                                data_csv << dual_str+",";
                                            }
                                        }
                                        // bounce posture selection
                                        for(size_t h=0;h<b_res.x.size();++h){
                                            string x_str =  boost::str(boost::format("%.8f") % (b_res.x.at(h))); boost::replace_all(x_str,",",".");
                                            data_csv << x_str+",";
                                        }
                                        for(size_t h=0;h<b_res.zL.size();++h){
                                            string zL_str =  boost::str(boost::format("%.8f") % (b_res.zL.at(h))); boost::replace_all(zL_str,",",".");
                                            data_csv << zL_str+",";
                                        }
                                        for(size_t h=0;h<b_res.zU.size();++h){
                                            string zU_str =  boost::str(boost::format("%.8f") % (b_res.zU.at(h))); boost::replace_all(zU_str,",",".");
                                            data_csv << zU_str+",";
                                        }
                                        for(size_t h=0;h<b_res.dual_vars.size();++h){
                                            string dual_str =  boost::str(boost::format("%.8f") % (b_res.dual_vars.at(h))); boost::replace_all(dual_str,",",".");
                                            if(h == b_res.dual_vars.size()-1){
                                                //last column
                                                data_csv << dual_str;
                                            }else{
                                                data_csv << dual_str+",";
                                            }
                                        }
                                        data_csv << "\n";
                                    }else{qnode.log(QNode::Info,std::string("Number of steps different than the original"));}
                                }else{qnode.log(QNode::Info,std::string("Planning failed"));}
                            }else{qnode.log(QNode::Info,std::string("Planning failed"));}
                        }while(!solved);
                    }// foor loop trials
                    data_csv.close();
                }
                ui.pushButton_plan_collect->setCheckable(true);
                qnode.log(QNode::Info,std::string("Planning and collecting ended"));
            }
            break;

        default:
            break;
        }
    }catch (const std::string message){qnode.log(QNode::Error,std::string("Plan failure: ")+message);
    }catch(const std::exception exc){qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());}

}

void MainWindow::on_pushButton_py_train_file_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Select the python file for training"),
                                                    QString(MAIN_PATH)+"/scripts",
                                                    "Python Files (*.py)");
    this->ui.lineEdit_py_train_file->setText(filename);
}

void MainWindow::on_pushButton_train_data_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Select the data file for training"),
                                                    QString(MAIN_PATH),
                                                    "CSV Files (*.csv)");
    this->ui.lineEdit_train_data->setText(filename);
}

void MainWindow::on_pushButton_models_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select the Directory to store trained models"),
                                                    QString(MAIN_PATH),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    this->ui.lineEdit_models->setText(dir);
}

void MainWindow::on_pushButton_train_pressed()
{
    qnode.log(QNode::Info,std::string("Training started"));
    qnode.log(QNode::Info,std::string("Training . . . "));
}

bool MainWindow::on_pushButton_train_clicked()
{
    if (!this->ui.lineEdit_py_train_file->text().isEmpty() && !this->ui.lineEdit_train_data->text().isEmpty() && !this->ui.lineEdit_models->text().isEmpty())
    {
        string cmdLine;
        string py_file = this->ui.lineEdit_py_train_file->text().toStdString();
        string data_file = this->ui.lineEdit_train_data->text().toStdString();
        string models_dir = this->ui.lineEdit_models->text().toStdString();

        cmdLine = string("python3 ") + py_file + string(" ") + data_file + string(" ") + models_dir;

        int status = system(cmdLine.c_str());
        qnode.log(QNode::Info,std::string("Training ended"));
        return(status==0);        
    }else{
        qnode.log(QNode::Info,std::string("Training ended"));
        return false;
    }

}

void MainWindow::on_pushButton_py_pred_file_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Select the python file for prediction"),
                                                    QString(MAIN_PATH)+"/scripts",
                                                    "Python Files (*.py)");
    this->ui.lineEdit_py_pred_file->setText(filename);
}

void MainWindow::on_pushButton_predictions_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select the Directory to store the predicted solutions"),
                                                    QString(MAIN_PATH),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    this->ui.lineEdit_predictions->setText(dir);
}

void MainWindow::on_pushButton_collections_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select the Directory to store the collected solutions"),
                                                    QString(MAIN_PATH),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    this->ui.lineEdit_collections->setText(dir);
}

void MainWindow::on_pushButton_pred_plan_pressed()
{
    qnode.log(QNode::Info,std::string("Prediction and planning started"));
    qnode.log(QNode::Info,std::string("Prediction and planning . . . "));
}

void MainWindow::on_pushButton_pred_plan_clicked()
{
    if (!this->ui.lineEdit_py_pred_file->text().isEmpty() && !this->ui.lineEdit_train_data->text().isEmpty() && !this->ui.lineEdit_predictions->text().isEmpty())
    {
        string cmdLine;
        string py_file = this->ui.lineEdit_py_pred_file->text().toStdString();
        string data_file = this->ui.lineEdit_train_data->text().toStdString();
        string models_dir = this->ui.lineEdit_models->text().toStdString();
        string pred_dir = this->ui.lineEdit_predictions->text().toStdString();

        problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
        int planner_id = prob->getPlannerID();
        int arm_sel = prob->getMovement()->getArm();
        int mov_type = prob->getMovement()->getType();
        HUMotion::hump_params  tols;
        std::vector<double> move_target;
        std::vector<double> move_target_mod;
        std::vector<double> move_final_hand;
        std::vector<double> move_final_arm;
        bool use_final;
        HUMotion::planning_result_ptr h_results_cold;
        HUMotion::planning_result_ptr h_results_ws_or;
        HUMotion::planning_result_ptr h_results_ws_rdm_pred;
        HUMotion::planning_result_ptr h_results_ws_nn_pred;
        HUMotion::planning_result_ptr h_results_ws_svm_pred;
        HUMotion::planning_result_ptr h_results_ws_knn_pred;
        int n_pred = ui.lineEdit_n_predictions->text().toInt();
        ui.pushButton_pred_plan->setCheckable(false);

        QList<QListWidgetItem*> obsts_items = this->ui.listWidget_obsts_except->selectedItems();
        vector<string> obsts_except;
        for(int ii=0;ii<obsts_items.count();++ii){
            obsts_except.push_back(obsts_items.at(ii)->text().toStdString());
        }

        // clear the results
        // plan
        success_cold_plan.clear(); iter_cold_plan.clear(); cpu_cold_plan.clear(); obj_cold_plan.clear();
        success_ws_or_plan.clear(); iter_ws_or_plan.clear(); cpu_ws_or_plan.clear(); obj_ws_or_plan.clear();
        success_ws_rdm_plan.clear(); iter_ws_rdm_plan.clear(); cpu_ws_rdm_plan.clear(); obj_ws_rdm_plan.clear();
        success_ws_nn_plan.clear(); iter_ws_nn_plan.clear(); cpu_ws_nn_plan.clear(); obj_ws_nn_plan.clear();
        success_ws_svm_plan.clear(); iter_ws_svm_plan.clear(); cpu_ws_svm_plan.clear(); obj_ws_svm_plan.clear();
        success_ws_knn_plan.clear(); iter_ws_knn_plan.clear(); cpu_ws_knn_plan.clear(); obj_ws_knn_plan.clear();
        // approach
        success_cold_app.clear(); iter_cold_app.clear(); cpu_cold_app.clear(); obj_cold_app.clear();
        success_ws_or_app.clear(); iter_ws_or_app.clear(); cpu_ws_or_app.clear(); obj_ws_or_app.clear();
        success_ws_rdm_app.clear(); iter_ws_rdm_app.clear(); cpu_ws_rdm_app.clear(); obj_ws_rdm_app.clear();
        success_ws_nn_app.clear(); iter_ws_nn_app.clear(); cpu_ws_nn_app.clear(); obj_ws_nn_app.clear();
        success_ws_svm_app.clear(); iter_ws_svm_app.clear(); cpu_ws_svm_app.clear(); obj_ws_svm_app.clear();
        success_ws_knn_app.clear(); iter_ws_knn_app.clear(); cpu_ws_knn_app.clear(); obj_ws_knn_app.clear();
        // retreat
        success_cold_ret.clear(); iter_cold_ret.clear(); cpu_cold_ret.clear(); obj_cold_ret.clear();
        success_ws_or_ret.clear(); iter_ws_or_ret.clear(); cpu_ws_or_ret.clear(); obj_ws_or_ret.clear();
        success_ws_rdm_ret.clear(); iter_ws_rdm_ret.clear(); cpu_ws_rdm_ret.clear(); obj_ws_rdm_ret.clear();
        success_ws_nn_ret.clear(); iter_ws_nn_ret.clear(); cpu_ws_nn_ret.clear(); obj_ws_nn_ret.clear();
        success_ws_svm_ret.clear(); iter_ws_svm_ret.clear(); cpu_ws_svm_ret.clear(); obj_ws_svm_ret.clear();
        success_ws_knn_ret.clear(); iter_ws_knn_ret.clear(); cpu_ws_knn_ret.clear(); obj_ws_knn_ret.clear();
        // bounce
        success_cold_bounce.clear(); iter_cold_bounce.clear(); cpu_cold_bounce.clear(); obj_cold_bounce.clear();
        success_ws_or_bounce.clear(); iter_ws_or_bounce.clear(); cpu_ws_or_bounce.clear(); obj_ws_or_bounce.clear();
        success_ws_rdm_bounce.clear(); iter_ws_rdm_bounce.clear(); cpu_ws_rdm_bounce.clear(); obj_ws_rdm_bounce.clear();
        success_ws_nn_bounce.clear(); iter_ws_nn_bounce.clear(); cpu_ws_nn_bounce.clear(); obj_ws_nn_bounce.clear();
        success_ws_svm_bounce.clear(); iter_ws_svm_bounce.clear(); cpu_ws_svm_bounce.clear(); obj_ws_svm_bounce.clear();
        success_ws_knn_bounce.clear(); iter_ws_knn_bounce.clear(); cpu_ws_knn_bounce.clear(); obj_ws_knn_bounce.clear();

        if(mov_type==1 || mov_type==5){ // move movement
            this->ui.tabWidget_learning_res->setTabEnabled(0,true);
            this->ui.tabWidget_learning_res->setTabEnabled(1,false);
            this->ui.tabWidget_learning_res->setTabEnabled(2,false);
            this->ui.tabWidget_learning_res->setTabEnabled(3,true);        
        }else{
            this->ui.tabWidget_learning_res->setTabEnabled(0,true);
            this->ui.tabWidget_learning_res->setTabEnabled(1,true);
            this->ui.tabWidget_learning_res->setTabEnabled(2,true);
            this->ui.tabWidget_learning_res->setTabEnabled(3,true);
        }

        try{
            switch (planner_id) {
            case 0: // HUMP
                if(arm_sel!=0)
                { // single-arm
                    mTolHumpdlg->setInfo(prob->getInfoLine());
                    // --- Tolerances for the final posture selection ---- //
                    tols.tolTarPos = mTolHumpdlg->getTolTarPos(); // target position tolerances
                    tols.tolTarOr = mTolHumpdlg->getTolTarOr(); // target orientation tolerances
                    mTolHumpdlg->getTolsArm(tols.tolsArm);// tolerances of the arm : radius in [mm]
                    mTolHumpdlg->getTolsHand(tols.tolsHand);// tolerances of the hand: radius in [mm]
                    tols.target_avoidance = mTolHumpdlg->getTargetAvoidance();// target avoidance
                    tols.obstacle_avoidance = mTolHumpdlg->getObstacleAvoidance(); //obstacle avoidance
                    mTolHumpdlg->getLambda(tols.lambda_final); // joint expense factors
                    mTolHumpdlg->getLambda(tols.lambda_bounce); // joint expense factors
                    // --- Tolerances for the bounce posture selection ---- //
                    tols.w_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getWMax()*M_PI/180)); // max joint velocity
                    tols.alpha_max = std::vector<double>(tols.lambda_final.size(),(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration
                    mTolHumpdlg->getInitVel(tols.bounds.vel_0); // initial velocity
                    mTolHumpdlg->getFinalVel(tols.bounds.vel_f); // final velocity
                    mTolHumpdlg->getInitAcc(tols.bounds.acc_0); // initial acceleration
                    mTolHumpdlg->getFinalAcc(tols.bounds.acc_f); // final acceleration
                    // tolerances for the obstacles
                    mTolHumpdlg->getTolsObstacles(tols.final_tolsObstacles); // final posture tols
                    tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
                    tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1));
                    mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
                    mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));
                    // tolerances for the target
                    tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
                    tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
                    tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
                    mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
                    tols.singleArm_tolsTarget.at(1) = tols.singleArm_tolsTarget.at(0)/100;
                    tols.singleArm_tolsTarget.at(2) = 0*tols.singleArm_tolsTarget.at(0);
                    //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(1));
                    //mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(2));
                    // pick / place settings
                    tols.mov_specs.approach = mTolHumpdlg->getApproach();
                    tols.mov_specs.retreat = mTolHumpdlg->getRetreat();
                    mTolHumpdlg->getPreGraspApproach(tols.mov_specs.pre_grasp_approach); // pick approach
                    mTolHumpdlg->getPostGraspRetreat(tols.mov_specs.post_grasp_retreat); // pick retreat
                    mTolHumpdlg->getPrePlaceApproach(tols.mov_specs.pre_place_approach); // place approach
                    mTolHumpdlg->getPostPlaceRetreat(tols.mov_specs.post_place_retreat); // place retreat
                    tols.mov_specs.rand_init = mTolHumpdlg->getRandInit(); // random initialization for "plan" stages
                    tols.mov_specs.coll = mTolHumpdlg->getColl(); // collisions option
                    tols.coll_body = mTolHumpdlg->getCollBody(); // collisions with the body
                    tols.mov_specs.straight_line = mTolHumpdlg->get_straight_line(); // hand straight line trajectory
                    tols.mov_specs.w_red_app_max = mTolHumpdlg->getW_red_app(); // set the max velocity reduction when approaching
                    tols.mov_specs.w_red_ret_max = mTolHumpdlg->getW_red_ret(); // set the max velocity reduction when retreating
                    // move settings
                    mTolHumpdlg->getTargetMove(move_target);
                    move_target_mod.resize(move_target.size());
                    mTolHumpdlg->getFinalHand(move_final_hand);
                    mTolHumpdlg->getFinalArm(move_final_arm);
                    use_final = mTolHumpdlg->get_use_final_posture();
                    prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
                    tols.mov_specs.use_move_plane = mTolHumpdlg->get_add_plane();
                    mTolHumpdlg->getPlaneParameters(tols.mov_specs.plane_params);
                    // Maximum variations of the target
                    double tar_x_var = ui.lineEdit_tar_x_var->text().toDouble();
                    double tar_y_var = ui.lineEdit_tar_y_var->text().toDouble();
                    double tar_z_var = ui.lineEdit_tar_z_var->text().toDouble();
                    double tar_roll_var = ui.lineEdit_tar_roll_var->text().toDouble();
                    double tar_pitch_var = ui.lineEdit_tar_pitch_var->text().toDouble();
                    double tar_yaw_var = ui.lineEdit_tar_yaw_var->text().toDouble();
                    // Maximum variations of the obstacles
                    double obsts_x_var = ui.lineEdit_obsts_x_var->text().toDouble();
                    double obsts_y_var = ui.lineEdit_obsts_y_var->text().toDouble();
                    double obsts_z_var = ui.lineEdit_obsts_z_var->text().toDouble();
                    double obsts_roll_var = ui.lineEdit_obsts_roll_var->text().toDouble();
                    double obsts_pitch_var = ui.lineEdit_obsts_pitch_var->text().toDouble();
                    double obsts_yaw_var = ui.lineEdit_obsts_yaw_var->text().toDouble();
                    // get the obstacles of the scenario
                    std::vector<objectPtr> obsts; prob->getObstacles(obsts);

                    // csv
                    string filename_csv("learning_predictions.csv");
                    ofstream pred_csv;
                    pred_csv.open(pred_dir+string("/")+filename_csv);
                    // headers
                    pred_csv << "target_x_mm,target_y_mm,target_z_mm,target_roll_rad,target_pitch_rad,target_yaw_rad";
                    for(size_t j=0;j<obsts.size();++j){
                        pred_csv << ",obstacle_"+to_string(j+1)+"_x_mm,obstacle_"+to_string(j+1)+"_y_mm,obstacle_"+to_string(j+1)+"_z_mm,obstacle_"+to_string(j+1)+"_roll_rad,obstacle_"+to_string(j+1)+"_pitch_rad,obstacle_"+to_string(j+1)+"_yaw_rad";
                    }
                    if(mov_type==1 || mov_type==5){ // move movement
                        // cold start
                        pred_csv << ",Success_f_cold,Iterations_f_plan_cold,Cpu_time_f_plan_cold,Obj_f_plan_cold,";
                        pred_csv << "Success_b_cold,Iterations_bounce_cold,Cpu_time_bounce_cold,Obj_bounce_cold,";

                        // warm start with the original solution
                        pred_csv << "Success_f_ws_or,Iterations_f_plan_ws_or,Cpu_time_f_plan_ws_or,Obj_f_plan_ws_or,";
                        pred_csv << "Success_b_ws_or,Iterations_bounce_ws_or,Cpu_time_bounce_ws_or,Obj_bounce_ws_or,";

                        // warm start with the random solution
                        pred_csv << "Success_f_ws_rdm,Iterations_f_plan_ws_rdm,Cpu_time_f_plan_ws_rdm,Obj_f_plan_ws_rdm,";
                        pred_csv << "Success_b_ws_rdm,Iterations_bounce_ws_rdm,Cpu_time_bounce_ws_rdm,Obj_bounce_ws_rdm,";

                        // warm start with the neural network
                        pred_csv << "Success_f_ws_nn_pred,Iterations_f_plan_ws_nn_pred,Cpu_time_f_plan_ws_nn_pred,Obj_f_plan_ws_nn_pred,";
                        pred_csv << "Success_b_ws_nn_pred,Iterations_bounce_ws_nn_pred,Cpu_time_bounce_ws_nn_pred,Obj_bounce_ws_nn_pred,";

                        // warm start with the support vector machines
                        pred_csv << "Success_f_ws_svm_pred,Iterations_f_plan_ws_svm_pred,Cpu_time_f_plan_ws_svm_pred,Obj_f_plan_ws_svm_pred,";
                        pred_csv << "Success_b_ws_svm_pred,Iterations_bounce_ws_svm_pred,Cpu_time_bounce_ws_svm_pred,Obj_bounce_ws_svm_pred,";

                        // warm start with the k-nearest neighbors
                        pred_csv << "Success_f_ws_knn_pred,Iterations_f_plan_ws_knn_pred,Cpu_time_f_plan_ws_knn_pred,Obj_f_plan_ws_knn_pred,";
                        pred_csv << "Success_b_ws_knn_pred,Iterations_bounce_ws_knn_pred,Cpu_time_bounce_ws_knn_pred,Obj_bounce_ws_knn_pred";

                    }else{

                        // cold start
                        pred_csv << ",Success_f_cold,Iterations_f_plan_cold,Cpu_time_f_plan_cold,Obj_f_plan_cold,";
                        pred_csv << "Success_f_app_cold,Iterations_f_approach_cold,Cpu_time_f_approach_cold,Obj_f_approach_cold,";
                        pred_csv << "Success_f_ret_cold,Iterations_f_retreat_cold,Cpu_time_f_retreat_cold,Obj_f_retreat_cold,";
                        pred_csv << "Success_b_cold,Iterations_bounce_cold,Cpu_time_bounce_cold,Obj_bounce_cold,";

                        // warm start with the original solution
                        pred_csv << "Success_f_ws_or,Iterations_f_plan_ws_or,Cpu_time_f_plan_ws_or,Obj_f_plan_ws_or,";
                        pred_csv << "Success_f_app_ws_or,Iterations_f_approach_ws_or,Cpu_time_f_approach_ws_or,Obj_f_approach_ws_or,";
                        pred_csv << "Success_f_ret_ws_or,Iterations_f_retreat_ws_or,Cpu_time_f_retreat_ws_or,Obj_f_retreat_ws_or,";
                        pred_csv << "Success_b_ws_or,Iterations_bounce_ws_or,Cpu_time_bounce_ws_or,Obj_bounce_ws_or,";

                        // warm start with the random solution
                        pred_csv << "Success_f_ws_rdm,Iterations_f_plan_ws_rdm,Cpu_time_f_plan_ws_rdm,Obj_f_plan_ws_rdm,";
                        pred_csv << "Success_f_app_ws_rdm,Iterations_f_approach_ws_rdm,Cpu_time_f_approach_ws_rdm,Obj_f_approach_ws_rdm,";
                        pred_csv << "Success_f_ret_ws_rdm,Iterations_f_retreat_ws_rdm,Cpu_time_f_retreat_ws_rdm,Obj_f_retreat_ws_rdm,";
                        pred_csv << "Success_b_ws_rdm,Iterations_bounce_ws_rdm,Cpu_time_bounce_ws_rdm,Obj_bounce_ws_rdm,";

                        // warm start with the neural network
                        pred_csv << "Success_f_ws_nn_pred,Iterations_f_plan_ws_nn_pred,Cpu_time_f_plan_ws_nn_pred,Obj_f_plan_ws_nn_pred,";
                        pred_csv << "Success_f_app_ws_nn_pred,Iterations_f_approach_ws_nn_pred,Cpu_time_f_approach_ws_nn_pred,Obj_f_approach_ws_nn_pred,";
                        pred_csv << "Success_f_ret_ws_nn_pred,Iterations_f_retreat_ws_nn_pred,Cpu_time_f_retreat_ws_nn_pred,Obj_f_retreat_ws_nn_pred,";
                        pred_csv << "Success_b_ws_nn_pred,Iterations_bounce_ws_nn_pred,Cpu_time_bounce_ws_nn_pred,Obj_bounce_ws_nn_pred,";

                        // warm start with the support vector machines
                        pred_csv << "Success_f_ws_svm_pred,Iterations_f_plan_ws_svm_pred,Cpu_time_f_plan_ws_svm_pred,Obj_f_plan_ws_svm_pred,";
                        pred_csv << "Success_f_app_ws_svm_pred,Iterations_f_approach_ws_svm_pred,Cpu_time_f_approach_ws_svm_pred,Obj_f_approach_ws_svm_pred,";
                        pred_csv << "Success_f_ret_ws_svm_pred,Iterations_f_retreat_ws_svm_pred,Cpu_time_f_retreat_ws_svm_pred,Obj_f_retreat_ws_svm_pred,";
                        pred_csv << "Success_b_ws_svm_pred,Iterations_bounce_ws_svm_pred,Cpu_time_bounce_ws_svm_pred,Obj_bounce_ws_svm_pred,";

                        // warm start with the k-nearest neighbors
                        pred_csv << "Success_f_ws_knn_pred,Iterations_f_plan_ws_knn_pred,Cpu_time_f_plan_ws_knn_pred,Obj_f_plan_ws_knn_pred,";
                        pred_csv << "Success_f_app_ws_knn_pred,Iterations_f_approach_ws_knn_pred,Cpu_time_f_approach_ws_knn_pred,Obj_f_approach_ws_knn_pred,";
                        pred_csv << "Success_f_ret_ws_knn_pred,Iterations_f_retreat_ws_knn_pred,Cpu_time_f_retreat_ws_knn_pred,Obj_f_retreat_ws_knn_pred,";
                        pred_csv << "Success_b_ws_knn_pred,Iterations_bounce_ws_knn_pred,Cpu_time_bounce_ws_knn_pred,Obj_bounce_ws_knn_pred";

                    }
                    pred_csv << " \n";

                    objectPtr obj_tar_or; // the original object involved in the movement
                    if(mov_type==0){ // reach-to-grasp movement
                        obj_tar_or = prob->getMovement()->getObject();
                    }

                    for(int i=0;i<n_pred;++i){

                        sleep(1);
                        qnode.log(QNode::Info,std::string("Prediction: ")+to_string(i));
                        std::srand(std::time(NULL));

                        string tar_x_pos_str;
                        string tar_y_pos_str;
                        string tar_z_pos_str;
                        string tar_roll_str;
                        string tar_pitch_str;
                        string tar_yaw_str;

                        if(mov_type==0){ // reach-to-grasp movement
                            objectPtr obj_tar = objectPtr(new Object(*(obj_tar_or.get())));
                            motion_manager::pos obj_pos;
                            motion_manager::orient obj_or;
                            if(this->ui.lineEdit_tar_x_var->isEnabled()){
                                obj_pos.Xpos = obj_tar_or->getPos().Xpos - (tar_x_var/2) + tar_x_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_pos.Xpos = obj_tar_or->getPos().Xpos;
                            }
                            if(this->ui.lineEdit_tar_y_var->isEnabled()){
                                obj_pos.Ypos = obj_tar_or->getPos().Ypos - (tar_y_var/2) + tar_y_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_pos.Ypos = obj_tar_or->getPos().Ypos;
                            }
                            if(this->ui.lineEdit_tar_z_var->isEnabled()){
                                obj_pos.Zpos = obj_tar_or->getPos().Zpos - (tar_z_var/2) + tar_z_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_pos.Zpos = obj_tar_or->getPos().Zpos;
                            }
                            if(this->ui.lineEdit_tar_roll_var->isEnabled()){
                                obj_or.roll = obj_tar_or->getOr().roll - (tar_roll_var/2) + tar_roll_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_or.roll = obj_tar_or->getOr().roll;
                            }
                            if(this->ui.lineEdit_tar_pitch_var->isEnabled()){
                                obj_or.pitch = obj_tar_or->getOr().pitch - (tar_pitch_var/2) + tar_pitch_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_or.pitch = obj_tar_or->getOr().pitch;
                            }
                            if(this->ui.lineEdit_tar_yaw_var->isEnabled()){
                                obj_or.yaw = obj_tar_or->getOr().yaw - (tar_yaw_var/2) + tar_yaw_var*(rand() / double(RAND_MAX));
                            }else{
                                obj_or.yaw = obj_tar_or->getOr().yaw;
                            }
                            obj_tar->setPos(obj_pos,true);
                            obj_tar->setOr(obj_or,true);
                            prob->getMovement()->setObject(obj_tar);

                            tar_x_pos_str =  boost::str(boost::format("%.8f") % (obj_pos.Xpos)); boost::replace_all(tar_x_pos_str,",",".");
                            tar_y_pos_str =  boost::str(boost::format("%.8f") % (obj_pos.Ypos)); boost::replace_all(tar_y_pos_str,",",".");
                            tar_z_pos_str =  boost::str(boost::format("%.8f") % (obj_pos.Zpos)); boost::replace_all(tar_z_pos_str,",",".");
                            tar_roll_str =  boost::str(boost::format("%.8f") % (obj_or.roll)); boost::replace_all(tar_roll_str,",",".");
                            tar_pitch_str =  boost::str(boost::format("%.8f") % (obj_or.pitch)); boost::replace_all(tar_pitch_str,",",".");
                            tar_yaw_str =  boost::str(boost::format("%.8f") % (obj_or.yaw)); boost::replace_all(tar_yaw_str,",",".");

                        }else if(mov_type==1 || mov_type==5){ // move movement

                            // modify target data (move_target)
                            if(this->ui.lineEdit_tar_x_var->isEnabled()){
                                move_target_mod.at(0) = move_target.at(0) - (tar_x_var/2) + tar_x_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(0) = move_target.at(0);
                            }
                            if(this->ui.lineEdit_tar_y_var->isEnabled()){
                                move_target_mod.at(1) = move_target.at(1) - (tar_y_var/2) + tar_y_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(1) = move_target.at(1);
                            }
                            if(this->ui.lineEdit_tar_z_var->isEnabled()){
                                move_target_mod.at(2) = move_target.at(2) - (tar_z_var/2) + tar_z_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(2) = move_target.at(2);
                            }
                            if(this->ui.lineEdit_tar_roll_var->isEnabled()){
                                move_target_mod.at(3) = move_target.at(3) - (tar_roll_var/2) + tar_roll_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(3) = move_target.at(3);
                            }
                            if(this->ui.lineEdit_tar_pitch_var->isEnabled()){
                                move_target_mod.at(4) = move_target.at(4) - (tar_pitch_var/2) + tar_pitch_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(4) = move_target.at(4);
                            }
                            if(this->ui.lineEdit_tar_yaw_var->isEnabled()){
                                move_target_mod.at(5) = move_target.at(5) - (tar_yaw_var/2) + tar_yaw_var*(rand() / double(RAND_MAX));
                            }else{
                                move_target_mod.at(5) = move_target.at(5);
                            }
                            prob->setMoveSettings(move_target_mod,move_final_hand,move_final_arm,use_final);

                            tar_x_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(0))); boost::replace_all(tar_x_pos_str,",",".");
                            tar_y_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(1))); boost::replace_all(tar_y_pos_str,",",".");
                            tar_z_pos_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(2))); boost::replace_all(tar_z_pos_str,",",".");
                            tar_roll_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(3))); boost::replace_all(tar_roll_str,",",".");
                            tar_pitch_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(4))); boost::replace_all(tar_pitch_str,",",".");
                            tar_yaw_str =  boost::str(boost::format("%.8f") % (move_target_mod.at(5))); boost::replace_all(tar_yaw_str,",",".");
                        }

                        string input_data_str = tar_x_pos_str + string(",") +
                                                tar_y_pos_str + string(",") +
                                                tar_z_pos_str + string(",") +
                                                tar_roll_str + string(",") +
                                                tar_pitch_str + string(",") +
                                                tar_yaw_str;

                        // modify obstacles data
                        objectPtr obs_new;
                        for(size_t j=0;j<obsts.size();++j){
                            std::srand(std::time(NULL));
                            objectPtr obs = obsts.at(j);
                            string obs_name = obs->getName();
                            if (std::find(obsts_except.begin(), obsts_except.end(), obs_name) != obsts_except.end())
                            {
                                continue;
                            }
                            obs_new.reset(new Object(obs_name));
                            motion_manager::pos obs_pos;
                            motion_manager::orient obs_or;
                            if(this->ui.lineEdit_obsts_x_var->isEnabled()){
                                obs_pos.Xpos = obs->getPos().Xpos - (obsts_x_var/2) + obsts_x_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_pos.Xpos = obs->getPos().Xpos;
                            }
                            if(this->ui.lineEdit_obsts_y_var->isEnabled()){
                                obs_pos.Ypos = obs->getPos().Ypos - (obsts_y_var/2) + obsts_y_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_pos.Ypos = obs->getPos().Ypos;
                            }
                            if(this->ui.lineEdit_obsts_z_var->isEnabled()){
                                obs_pos.Zpos = obs->getPos().Zpos - (obsts_z_var/2) + obsts_z_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_pos.Zpos = obs->getPos().Zpos;
                            }
                            if(this->ui.lineEdit_obsts_roll_var->isEnabled()){
                                obs_or.roll = obs->getOr().roll - (obsts_roll_var/2) + obsts_roll_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_or.roll = obs->getOr().roll;
                            }
                            if(this->ui.lineEdit_obsts_pitch_var->isEnabled()){
                                obs_or.pitch = obs->getOr().pitch - (obsts_pitch_var/2) + obsts_pitch_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_or.pitch = obs->getOr().pitch;
                            }
                            if(this->ui.lineEdit_obsts_yaw_var->isEnabled()){
                                obs_or.yaw = obs->getOr().yaw - (obsts_yaw_var/2) + obsts_yaw_var*(rand() / double(RAND_MAX));
                            }else{
                                obs_or.yaw = obs->getOr().yaw;
                            }
                            obs_new->setPos(obs_pos,false);
                            obs_new->setOr(obs_or,false);
                            obs_new->setSize(obs->getSize());
                            string obs_x_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Xpos)); boost::replace_all(obs_x_pos_str,",",".");
                            string obs_y_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Ypos)); boost::replace_all(obs_y_pos_str,",",".");
                            string obs_z_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Zpos)); boost::replace_all(obs_z_pos_str,",",".");
                            string obs_roll_str =  boost::str(boost::format("%.8f") % (obs->getOr().roll)); boost::replace_all(obs_roll_str,",",".");
                            string obs_pitch_str =  boost::str(boost::format("%.8f") % (obs->getOr().pitch)); boost::replace_all(obs_pitch_str,",",".");
                            string obs_yaw_str =  boost::str(boost::format("%.8f") % (obs->getOr().yaw)); boost::replace_all(obs_yaw_str,",",".");
                            string obs_str = obs_x_pos_str + string(",") +
                                             obs_y_pos_str + string(",") +
                                             obs_z_pos_str + string(",") +
                                             obs_roll_str + string(",") +
                                             obs_pitch_str + string(",") +
                                             obs_yaw_str;
                            input_data_str.append(string(",") + obs_str);
                            prob->setObstacle(obs_new,j);
                        }// for loop obstacles

                        // predict the solution
                        string pred_file = pred_dir +string("/pred_sol_")+to_string(i)+string(".dual");
                        cmdLine = string("python3 ") + py_file + string(" ") + data_file + string(" ") + models_dir + string(" ") + pred_file + string(" ") + input_data_str;
                        //std::cout << cmdLine << std::endl;
                        int status = system(cmdLine.c_str());

                        // Random solution
                        bool pred_sol_rdm_plan = false, pred_sol_rdm_app = false, pred_sol_rdm_ret = false, pred_sol_rdm_bounce = false;
                        std::vector<double> pred_x_rdm_plan, pred_zL_rdm_plan, pred_zU_rdm_plan, pred_dual_rdm_plan;
                        std::vector<double> pred_x_rdm_app, pred_zL_rdm_app, pred_zU_rdm_app, pred_dual_rdm_app;
                        std::vector<double> pred_x_rdm_ret, pred_zL_rdm_ret, pred_zU_rdm_ret, pred_dual_rdm_ret;
                        std::vector<double> pred_x_rdm_bounce, pred_zL_rdm_bounce, pred_zU_rdm_bounce, pred_dual_rdm_bounce;
                        // Neural Network solution
                        bool pred_sol_nn_plan = false, pred_sol_nn_app = false, pred_sol_nn_ret = false, pred_sol_nn_bounce = false;
                        std::vector<double> pred_x_nn_plan, pred_zL_nn_plan, pred_zU_nn_plan, pred_dual_nn_plan;
                        std::vector<double> pred_x_nn_app, pred_zL_nn_app, pred_zU_nn_app, pred_dual_nn_app;
                        std::vector<double> pred_x_nn_ret, pred_zL_nn_ret, pred_zU_nn_ret, pred_dual_nn_ret;
                        std::vector<double> pred_x_nn_bounce, pred_zL_nn_bounce, pred_zU_nn_bounce, pred_dual_nn_bounce;
                        // Support Vector Machines solution
                        bool pred_sol_svm_plan = false, pred_sol_svm_app = false, pred_sol_svm_ret = false, pred_sol_svm_bounce = false;
                        std::vector<double> pred_x_svm_plan, pred_zL_svm_plan, pred_zU_svm_plan, pred_dual_svm_plan;
                        std::vector<double> pred_x_svm_app, pred_zL_svm_app, pred_zU_svm_app, pred_dual_svm_app;
                        std::vector<double> pred_x_svm_ret, pred_zL_svm_ret, pred_zU_svm_ret, pred_dual_svm_ret;
                        std::vector<double> pred_x_svm_bounce, pred_zL_svm_bounce, pred_zU_svm_bounce, pred_dual_svm_bounce;
                        // K-Nearest Neighbors solution
                        bool pred_sol_knn_plan = false, pred_sol_knn_app = false, pred_sol_knn_ret = false, pred_sol_knn_bounce = false;
                        std::vector<double> pred_x_knn_plan, pred_zL_knn_plan, pred_zU_knn_plan, pred_dual_knn_plan;
                        std::vector<double> pred_x_knn_app, pred_zL_knn_app, pred_zU_knn_app, pred_dual_knn_app;
                        std::vector<double> pred_x_knn_ret, pred_zL_knn_ret, pred_zU_knn_ret, pred_dual_knn_ret;
                        std::vector<double> pred_x_knn_bounce, pred_zL_knn_bounce, pred_zU_knn_bounce, pred_dual_knn_bounce;

                        if(status==0)
                        { // successful prediction, read the prediced solution
                            QString q_pred_file = QString::fromStdString(pred_file);
                            QFile f(q_pred_file);
                            if(f.open( QIODevice::ReadOnly )){
                                QTextStream stream( &f );
                                QString line;
                                while(!stream.atEnd()){
                                    line = f.readLine();
                                    if(line.at(0)!=QChar('#')){
                                        QStringList fields = line.split("=");
                                        // random solution
                                        if (QString::compare(fields.at(0),QString("X_rdm_plan"),Qt::CaseInsensitive)==0){
                                            pred_sol_rdm_plan = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_rdm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZL_rdm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_rdm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZU_rdm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_rdm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("Dual_rdm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_rdm_plan.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_rdm_approach"),Qt::CaseInsensitive)==0){
                                            pred_sol_rdm_app = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_rdm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_rdm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_rdm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_rdm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_rdm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_rdm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_rdm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_rdm_retreat"),Qt::CaseInsensitive)==0){
                                            pred_sol_rdm_ret = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_rdm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_rdm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_rdm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_rdm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_rdm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_rdm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_rdm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_rdm_bounce"),Qt::CaseInsensitive)==0){
                                            pred_sol_rdm_bounce = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_rdm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_rdm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_rdm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_rdm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_rdm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_rdm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_rdm_bounce.push_back(data.at(i).toDouble());
                                        }
                                        // neural network solution
                                        if (QString::compare(fields.at(0),QString("X_nn_plan"),Qt::CaseInsensitive)==0){
                                            pred_sol_nn_plan = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_nn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZL_nn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_nn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZU_nn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_nn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("Dual_nn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_nn_plan.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_nn_approach"),Qt::CaseInsensitive)==0){
                                            pred_sol_nn_app = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_nn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_nn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_nn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_nn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_nn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_nn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_nn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_nn_retreat"),Qt::CaseInsensitive)==0){
                                            pred_sol_nn_ret = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_nn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_nn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_nn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_nn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_nn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_nn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_nn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_nn_bounce"),Qt::CaseInsensitive)==0){
                                            pred_sol_nn_bounce = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_nn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_nn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_nn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_nn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_nn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_nn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_nn_bounce.push_back(data.at(i).toDouble());
                                        }
                                        // support vector machines solution
                                        if (QString::compare(fields.at(0),QString("X_svm_plan"),Qt::CaseInsensitive)==0){
                                            pred_sol_svm_plan = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_svm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZL_svm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_svm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZU_svm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_svm_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("Dual_svm_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_svm_plan.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_svm_approach"),Qt::CaseInsensitive)==0){
                                            pred_sol_svm_app = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_svm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_svm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_svm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_svm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_svm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_svm_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_svm_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_svm_retreat"),Qt::CaseInsensitive)==0){
                                            pred_sol_svm_ret = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_svm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_svm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_svm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_svm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_svm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_svm_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_svm_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_svm_bounce"),Qt::CaseInsensitive)==0){
                                            pred_sol_svm_bounce = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_svm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_svm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_svm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_svm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_svm_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_svm_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_svm_bounce.push_back(data.at(i).toDouble());
                                        }
                                        // k-nearest neighbors solution
                                        if (QString::compare(fields.at(0),QString("X_knn_plan"),Qt::CaseInsensitive)==0){
                                            pred_sol_knn_plan = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_knn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZL_knn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_knn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("ZU_knn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_knn_plan.push_back(data.at(i).toDouble());
                                        }else if (QString::compare(fields.at(0),QString("Dual_knn_plan"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_knn_plan.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_knn_approach"),Qt::CaseInsensitive)==0){
                                            pred_sol_knn_app = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_knn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_knn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_knn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_knn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_knn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_knn_approach"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_knn_app.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_knn_retreat"),Qt::CaseInsensitive)==0){
                                            pred_sol_knn_ret = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_knn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_knn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_knn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_knn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_knn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_knn_retreat"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_knn_ret.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("X_knn_bounce"),Qt::CaseInsensitive)==0){
                                            pred_sol_knn_bounce = true;
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_x_knn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZL_knn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zL_knn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("ZU_knn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_zU_knn_bounce.push_back(data.at(i).toDouble());
                                        }else if(QString::compare(fields.at(0),QString("Dual_knn_bounce"),Qt::CaseInsensitive)==0){
                                            QStringList data = fields.at(1).split("|");
                                            for (size_t i=0; i<data.size();++i)
                                                pred_dual_knn_bounce.push_back(data.at(i).toDouble());
                                        }
                                    }
                                }
                                f.close();
                            }
                        }

                        // plan the movement

                        // cold start
                        tols.mov_specs.warm_start = false;
                        h_results_cold = prob->solve(tols);

                        // warm start with the original solution
                        tols.mov_specs.warm_start = true;
                        if(this->sol_plan)
                        {
                            HUMotion::warm_start_params final_plan;
                            final_plan.valid = true;
                            final_plan.description = "plan";
                            final_plan.x = this->x_plan;
                            final_plan.zL = this->zL_plan;
                            final_plan.zU = this->zU_plan;
                            final_plan.dual_vars = this->dual_plan;
                            tols.mov_specs.final_warm_start_params.push_back(final_plan);
                        }
                        if(this->sol_approach)
                        {
                            HUMotion::warm_start_params final_app;
                            final_app.valid = true;
                            final_app.description = "approach";
                            final_app.x = this->x_approach;
                            final_app.zL = this->zL_approach;
                            final_app.zU = this->zU_approach;
                            final_app.dual_vars = this->dual_approach;
                            tols.mov_specs.final_warm_start_params.push_back(final_app);
                        }
                        if(this->sol_retreat)
                        {
                            HUMotion::warm_start_params final_ret;
                            final_ret.valid = true;
                            final_ret.description = "retreat";
                            final_ret.x = this->x_retreat;
                            final_ret.zL = this->zL_retreat;
                            final_ret.zU = this->zU_retreat;
                            final_ret.dual_vars = this->dual_retreat;
                            tols.mov_specs.final_warm_start_params.push_back(final_ret);
                        }
                        if(this->sol_bounce)
                        {
                            HUMotion::warm_start_params bounce;
                            bounce.valid = true;
                            bounce.description = "bounce";
                            bounce.x = this->x_bounce;
                            bounce.zL = this->zL_bounce;
                            bounce.zU = this->zU_bounce;
                            bounce.dual_vars = this->dual_bounce;
                            tols.mov_specs.bounce_warm_start_params = bounce;
                        }
                        h_results_ws_or = prob->solve(tols);
                        tols.mov_specs.final_warm_start_params.clear();

                        // warm start with the random solution
                        tols.mov_specs.warm_start = true;
                        if(pred_sol_rdm_plan)
                        {
                            HUMotion::warm_start_params final_plan;
                            final_plan.valid = true;
                            final_plan.description = "plan";
                            final_plan.x = pred_x_rdm_plan;
                            final_plan.zL = pred_zL_rdm_plan;
                            final_plan.zU = pred_zU_rdm_plan;
                            final_plan.dual_vars = pred_dual_rdm_plan;
                            tols.mov_specs.final_warm_start_params.push_back(final_plan);
                        }
                        if(pred_sol_rdm_app)
                        {
                            HUMotion::warm_start_params final_app;
                            final_app.valid = true;
                            final_app.description = "approach";
                            final_app.x = pred_x_rdm_app;
                            final_app.zL = pred_zL_rdm_app;
                            final_app.zU = pred_zU_rdm_app;
                            final_app.dual_vars = pred_dual_rdm_app;
                            tols.mov_specs.final_warm_start_params.push_back(final_app);
                        }
                        if(pred_sol_rdm_ret)
                        {
                            HUMotion::warm_start_params final_ret;
                            final_ret.valid = true;
                            final_ret.description = "retreat";
                            final_ret.x = pred_x_rdm_ret;
                            final_ret.zL = pred_zL_rdm_ret;
                            final_ret.zU = pred_zU_rdm_ret;
                            final_ret.dual_vars = pred_dual_rdm_app;
                            tols.mov_specs.final_warm_start_params.push_back(final_ret);
                        }
                        if(pred_sol_rdm_bounce)
                        {
                            HUMotion::warm_start_params bounce;
                            bounce.valid = true;
                            bounce.description = "bounce";
                            bounce.x = pred_x_rdm_bounce;
                            bounce.zL = pred_zL_rdm_bounce;
                            bounce.zU = pred_zU_rdm_bounce;
                            bounce.dual_vars = pred_dual_rdm_bounce;
                            tols.mov_specs.bounce_warm_start_params = bounce;
                        }
                        h_results_ws_rdm_pred = prob->solve(tols);
                        tols.mov_specs.final_warm_start_params.clear();

                        // warm start with the neural network solution
                        tols.mov_specs.warm_start = true;
                        if(pred_sol_nn_plan)
                        {
                            HUMotion::warm_start_params final_plan;
                            final_plan.valid = true;
                            final_plan.description = "plan";
                            final_plan.x = pred_x_nn_plan;
                            final_plan.zL = pred_zL_nn_plan;
                            final_plan.zU = pred_zU_nn_plan;
                            final_plan.dual_vars = pred_dual_nn_plan;
                            tols.mov_specs.final_warm_start_params.push_back(final_plan);
                        }
                        if(pred_sol_nn_app)
                        {
                            HUMotion::warm_start_params final_app;
                            final_app.valid = true;
                            final_app.description = "approach";
                            final_app.x = pred_x_nn_app;
                            final_app.zL = pred_zL_nn_app;
                            final_app.zU = pred_zU_nn_app;
                            final_app.dual_vars = pred_dual_nn_app;
                            tols.mov_specs.final_warm_start_params.push_back(final_app);
                        }
                        if(pred_sol_nn_ret)
                        {
                            HUMotion::warm_start_params final_ret;
                            final_ret.valid = true;
                            final_ret.description = "retreat";
                            final_ret.x = pred_x_nn_ret;
                            final_ret.zL = pred_zL_nn_ret;
                            final_ret.zU = pred_zU_nn_ret;
                            final_ret.dual_vars = pred_dual_nn_ret;
                            tols.mov_specs.final_warm_start_params.push_back(final_ret);
                        }
                        if(pred_sol_nn_bounce)
                        {
                            HUMotion::warm_start_params bounce;
                            bounce.valid = true;
                            bounce.description = "bounce";
                            bounce.x = pred_x_nn_bounce;
                            bounce.zL = pred_zL_nn_bounce;
                            bounce.zU = pred_zU_nn_bounce;
                            bounce.dual_vars = pred_dual_nn_bounce;
                            tols.mov_specs.bounce_warm_start_params = bounce;
                        }
                        h_results_ws_nn_pred = prob->solve(tols);
                        tols.mov_specs.final_warm_start_params.clear();

                        // warm start with the support vector machines solution
                        tols.mov_specs.warm_start = true;
                        if(pred_sol_svm_plan)
                        {
                            HUMotion::warm_start_params final_plan;
                            final_plan.valid = true;
                            final_plan.description = "plan";
                            final_plan.x = pred_x_svm_plan;
                            final_plan.zL = pred_zL_svm_plan;
                            final_plan.zU = pred_zU_svm_plan;
                            final_plan.dual_vars = pred_dual_svm_plan;
                            tols.mov_specs.final_warm_start_params.push_back(final_plan);
                        }
                        if(pred_sol_svm_app)
                        {
                            HUMotion::warm_start_params final_app;
                            final_app.valid = true;
                            final_app.description = "approach";
                            final_app.x = pred_x_svm_app;
                            final_app.zL = pred_zL_svm_app;
                            final_app.zU = pred_zU_svm_app;
                            final_app.dual_vars = pred_dual_svm_app;
                            tols.mov_specs.final_warm_start_params.push_back(final_app);
                        }
                        if(pred_sol_svm_ret)
                        {
                            HUMotion::warm_start_params final_ret;
                            final_ret.valid = true;
                            final_ret.description = "retreat";
                            final_ret.x = pred_x_svm_ret;
                            final_ret.zL = pred_zL_svm_ret;
                            final_ret.zU = pred_zU_svm_ret;
                            final_ret.dual_vars = pred_dual_svm_ret;
                            tols.mov_specs.final_warm_start_params.push_back(final_ret);
                        }
                        if(pred_sol_svm_bounce)
                        {
                            HUMotion::warm_start_params bounce;
                            bounce.valid = true;
                            bounce.description = "bounce";
                            bounce.x = pred_x_svm_bounce;
                            bounce.zL = pred_zL_svm_bounce;
                            bounce.zU = pred_zU_svm_bounce;
                            bounce.dual_vars = pred_dual_svm_bounce;
                            tols.mov_specs.bounce_warm_start_params = bounce;
                        }
                        h_results_ws_svm_pred = prob->solve(tols);
                        tols.mov_specs.final_warm_start_params.clear();

                        // warm start with the k-nearest neighbors solution
                        tols.mov_specs.warm_start = true;
                        if(pred_sol_knn_plan)
                        {
                            HUMotion::warm_start_params final_plan;
                            final_plan.valid = true;
                            final_plan.description = "plan";
                            final_plan.x = pred_x_knn_plan;
                            final_plan.zL = pred_zL_knn_plan;
                            final_plan.zU = pred_zU_knn_plan;
                            final_plan.dual_vars = pred_dual_knn_plan;
                            tols.mov_specs.final_warm_start_params.push_back(final_plan);
                        }
                        if(pred_sol_knn_app)
                        {
                            HUMotion::warm_start_params final_app;
                            final_app.valid = true;
                            final_app.description = "approach";
                            final_app.x = pred_x_knn_app;
                            final_app.zL = pred_zL_knn_app;
                            final_app.zU = pred_zU_knn_app;
                            final_app.dual_vars = pred_dual_knn_app;
                            tols.mov_specs.final_warm_start_params.push_back(final_app);
                        }
                        if(pred_sol_knn_ret)
                        {
                            HUMotion::warm_start_params final_ret;
                            final_ret.valid = true;
                            final_ret.description = "retreat";
                            final_ret.x = pred_x_knn_ret;
                            final_ret.zL = pred_zL_knn_ret;
                            final_ret.zU = pred_zU_knn_ret;
                            final_ret.dual_vars = pred_dual_knn_ret;
                            tols.mov_specs.final_warm_start_params.push_back(final_ret);
                        }
                        if(pred_sol_knn_bounce)
                        {
                            HUMotion::warm_start_params bounce;
                            bounce.valid = true;
                            bounce.description = "bounce";
                            bounce.x = pred_x_knn_bounce;
                            bounce.zL = pred_zL_knn_bounce;
                            bounce.zU = pred_zU_knn_bounce;
                            bounce.dual_vars = pred_dual_knn_bounce;
                            tols.mov_specs.bounce_warm_start_params = bounce;
                        }
                        h_results_ws_knn_pred = prob->solve(tols);
                        tols.mov_specs.final_warm_start_params.clear();


                        // collection of the results
                        // target
                        pred_csv << tar_x_pos_str+","+tar_y_pos_str+","+tar_z_pos_str+","+tar_roll_str+","+tar_pitch_str+","+tar_yaw_str;
                        // obstacles
                        std::vector<objectPtr> obsts_new; prob->getObstacles(obsts_new);
                        for(size_t j=0;j<obsts_new.size();++j){
                            objectPtr obs = obsts_new.at(j);
                            string obs_x_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Xpos)); boost::replace_all(obs_x_pos_str,",",".");
                            string obs_y_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Ypos)); boost::replace_all(obs_y_pos_str,",",".");
                            string obs_z_pos_str =  boost::str(boost::format("%.8f") % (obs->getPos().Zpos)); boost::replace_all(obs_z_pos_str,",",".");
                            string obs_roll_str =  boost::str(boost::format("%.8f") % (obs->getOr().roll)); boost::replace_all(obs_roll_str,",",".");
                            string obs_pitch_str =  boost::str(boost::format("%.8f") % (obs->getOr().pitch)); boost::replace_all(obs_pitch_str,",",".");
                            string obs_yaw_str =  boost::str(boost::format("%.8f") % (obs->getOr().yaw)); boost::replace_all(obs_yaw_str,",",".");
                            pred_csv << ","+obs_x_pos_str+","+obs_y_pos_str+","+obs_z_pos_str+","+obs_roll_str+","+obs_pitch_str+","+obs_yaw_str;
                        }

                        // cold start
                        if(h_results_cold!=nullptr){
                            if(h_results_cold->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_cold->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_cold->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_cold_plan.push_back(1); iter_cold_plan.push_back(plan_tar.iterations); cpu_cold_plan.push_back(plan_tar.cpu_time); obj_cold_plan.push_back(plan_tar.obj_value);
                                    success_cold_bounce.push_back(1); iter_cold_bounce.push_back(bounce_ws.iterations); cpu_cold_bounce.push_back(bounce_ws.cpu_time); obj_cold_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_cold->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_cold->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_cold->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_cold->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_cold_plan.push_back(1); iter_cold_plan.push_back(plan_tar.iterations); cpu_cold_plan.push_back(plan_tar.cpu_time); obj_cold_plan.push_back(plan_tar.obj_value);
                                    success_cold_app.push_back(1); iter_cold_app.push_back(approach_tar.iterations); cpu_cold_app.push_back(approach_tar.cpu_time); obj_cold_app.push_back(approach_tar.obj_value);
                                    success_cold_ret.push_back(1); iter_cold_ret.push_back(retreat_tar.iterations); cpu_cold_ret.push_back(retreat_tar.cpu_time); obj_cold_ret.push_back(retreat_tar.obj_value);
                                    success_cold_bounce.push_back(1); iter_cold_bounce.push_back(bounce_ws.iterations); cpu_cold_bounce.push_back(bounce_ws.cpu_time); obj_cold_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_cold_plan.push_back(0);
                                    success_cold_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_cold_plan.push_back(0);
                                    success_cold_app.push_back(0);
                                    success_cold_ret.push_back(0);
                                    success_cold_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_cold_plan.push_back(0);
                                success_cold_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_cold_plan.push_back(0);
                                success_cold_app.push_back(0);
                                success_cold_ret.push_back(0);
                                success_cold_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                        }

                        // warm start with the original solution
                        if(h_results_ws_or!=nullptr){
                            if(h_results_ws_or->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_or->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_or->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_or_plan.push_back(1); iter_ws_or_plan.push_back(plan_tar.iterations); cpu_ws_or_plan.push_back(plan_tar.cpu_time); obj_ws_or_plan.push_back(plan_tar.obj_value);
                                    success_ws_or_bounce.push_back(1); iter_ws_or_bounce.push_back(bounce_ws.iterations); cpu_ws_or_bounce.push_back(bounce_ws.cpu_time); obj_ws_or_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_or->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_ws_or->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_ws_or->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_or->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_or_plan.push_back(1); iter_ws_or_plan.push_back(plan_tar.iterations); cpu_ws_or_plan.push_back(plan_tar.cpu_time); obj_ws_or_plan.push_back(plan_tar.obj_value);
                                    success_ws_or_app.push_back(1); iter_ws_or_app.push_back(approach_tar.iterations); cpu_ws_or_app.push_back(approach_tar.cpu_time); obj_ws_or_app.push_back(approach_tar.obj_value);
                                    success_ws_or_ret.push_back(1); iter_ws_or_ret.push_back(retreat_tar.iterations); cpu_ws_or_ret.push_back(retreat_tar.cpu_time); obj_ws_or_ret.push_back(retreat_tar.obj_value);
                                    success_ws_or_bounce.push_back(1); iter_ws_or_bounce.push_back(bounce_ws.iterations); cpu_ws_or_bounce.push_back(bounce_ws.cpu_time); obj_ws_or_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_or_plan.push_back(0);
                                    success_ws_or_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_or_plan.push_back(0);
                                    success_ws_or_app.push_back(0);
                                    success_ws_or_ret.push_back(0);
                                    success_ws_or_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_or_plan.push_back(0);
                                success_ws_or_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_or_plan.push_back(0);
                                success_ws_or_app.push_back(0);
                                success_ws_or_ret.push_back(0);
                                success_ws_or_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                        }

                        // warm start with the random solution
                        if(h_results_ws_rdm_pred!=nullptr){
                            if(h_results_ws_rdm_pred->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_rdm_pred->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_rdm_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_rdm_plan.push_back(1); iter_ws_rdm_plan.push_back(plan_tar.iterations); cpu_ws_rdm_plan.push_back(plan_tar.cpu_time); obj_ws_rdm_plan.push_back(plan_tar.obj_value);
                                    success_ws_rdm_bounce.push_back(1); iter_ws_rdm_bounce.push_back(bounce_ws.iterations); cpu_ws_rdm_bounce.push_back(bounce_ws.cpu_time); obj_ws_rdm_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_rdm_pred->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_ws_rdm_pred->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_ws_rdm_pred->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_rdm_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_rdm_plan.push_back(1); iter_ws_rdm_plan.push_back(plan_tar.iterations); cpu_ws_rdm_plan.push_back(plan_tar.cpu_time); obj_ws_rdm_plan.push_back(plan_tar.obj_value);
                                    success_ws_rdm_app.push_back(1); iter_ws_rdm_app.push_back(approach_tar.iterations); cpu_ws_rdm_app.push_back(approach_tar.cpu_time); obj_ws_rdm_app.push_back(approach_tar.obj_value);
                                    success_ws_rdm_ret.push_back(1); iter_ws_rdm_ret.push_back(retreat_tar.iterations); cpu_ws_rdm_ret.push_back(retreat_tar.cpu_time); obj_ws_rdm_ret.push_back(retreat_tar.obj_value);
                                    success_ws_rdm_bounce.push_back(1); iter_ws_rdm_bounce.push_back(bounce_ws.iterations); cpu_ws_rdm_bounce.push_back(bounce_ws.cpu_time); obj_ws_rdm_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_rdm_plan.push_back(0);
                                    success_ws_rdm_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_rdm_plan.push_back(0);
                                    success_ws_rdm_app.push_back(0);
                                    success_ws_rdm_ret.push_back(0);
                                    success_ws_rdm_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_rdm_plan.push_back(0);
                                success_ws_rdm_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_rdm_plan.push_back(0);
                                success_ws_rdm_app.push_back(0);
                                success_ws_rdm_ret.push_back(0);
                                success_ws_rdm_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with cold start failed"));
                        }

                        // warm start with the neural network solution
                        if(h_results_ws_nn_pred!=nullptr){
                            if(h_results_ws_nn_pred->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_nn_pred->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_nn_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_nn_plan.push_back(1); iter_ws_nn_plan.push_back(plan_tar.iterations); cpu_ws_nn_plan.push_back(plan_tar.cpu_time); obj_ws_nn_plan.push_back(plan_tar.obj_value);
                                    success_ws_nn_bounce.push_back(1); iter_ws_nn_bounce.push_back(bounce_ws.iterations); cpu_ws_nn_bounce.push_back(bounce_ws.cpu_time); obj_ws_nn_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_nn_pred->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_ws_nn_pred->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_ws_nn_pred->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_nn_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_nn_plan.push_back(1); iter_ws_nn_plan.push_back(plan_tar.iterations); cpu_ws_nn_plan.push_back(plan_tar.cpu_time); obj_ws_nn_plan.push_back(plan_tar.obj_value);
                                    success_ws_nn_app.push_back(1); iter_ws_nn_app.push_back(approach_tar.iterations); cpu_ws_nn_app.push_back(approach_tar.cpu_time); obj_ws_nn_app.push_back(approach_tar.obj_value);
                                    success_ws_nn_ret.push_back(1); iter_ws_nn_ret.push_back(retreat_tar.iterations); cpu_ws_nn_ret.push_back(retreat_tar.cpu_time); obj_ws_nn_ret.push_back(retreat_tar.obj_value);
                                    success_ws_nn_bounce.push_back(1); iter_ws_nn_bounce.push_back(bounce_ws.iterations); cpu_ws_nn_bounce.push_back(bounce_ws.cpu_time); obj_ws_nn_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_nn_plan.push_back(0);
                                    success_ws_nn_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_nn_plan.push_back(0);
                                    success_ws_nn_app.push_back(0);
                                    success_ws_nn_ret.push_back(0);
                                    success_ws_nn_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with warm start NN failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_nn_plan.push_back(0);
                                success_ws_nn_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_nn_plan.push_back(0);
                                success_ws_nn_app.push_back(0);
                                success_ws_nn_ret.push_back(0);
                                success_ws_nn_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with warm start NN failed"));
                        }

                        // warm start with the support vector machines solution
                        if(h_results_ws_svm_pred!=nullptr){
                            if(h_results_ws_svm_pred->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_svm_pred->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_svm_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_svm_plan.push_back(1); iter_ws_svm_plan.push_back(plan_tar.iterations); cpu_ws_svm_plan.push_back(plan_tar.cpu_time); obj_ws_svm_plan.push_back(plan_tar.obj_value);
                                    success_ws_svm_bounce.push_back(1); iter_ws_svm_bounce.push_back(bounce_ws.iterations); cpu_ws_svm_bounce.push_back(bounce_ws.cpu_time); obj_ws_svm_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_svm_pred->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_ws_svm_pred->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_ws_svm_pred->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_svm_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_svm_plan.push_back(1); iter_ws_svm_plan.push_back(plan_tar.iterations); cpu_ws_svm_plan.push_back(plan_tar.cpu_time); obj_ws_svm_plan.push_back(plan_tar.obj_value);
                                    success_ws_svm_app.push_back(1); iter_ws_svm_app.push_back(approach_tar.iterations); cpu_ws_svm_app.push_back(approach_tar.cpu_time); obj_ws_svm_app.push_back(approach_tar.obj_value);
                                    success_ws_svm_ret.push_back(1); iter_ws_svm_ret.push_back(retreat_tar.iterations); cpu_ws_svm_ret.push_back(retreat_tar.cpu_time); obj_ws_svm_ret.push_back(retreat_tar.obj_value);
                                    success_ws_svm_bounce.push_back(1); iter_ws_svm_bounce.push_back(bounce_ws.iterations); cpu_ws_svm_bounce.push_back(bounce_ws.cpu_time); obj_ws_svm_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_svm_plan.push_back(0);
                                    success_ws_svm_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_svm_plan.push_back(0);
                                    success_ws_svm_app.push_back(0);
                                    success_ws_svm_ret.push_back(0);
                                    success_ws_svm_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with warm start SVM failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_svm_plan.push_back(0);
                                success_ws_svm_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_svm_plan.push_back(0);
                                success_ws_svm_app.push_back(0);
                                success_ws_svm_ret.push_back(0);
                                success_ws_svm_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with warm start SVM failed"));
                        }

                        // warm start with the k-nearest neighbors solution
                        if(h_results_ws_knn_pred!=nullptr){
                            if(h_results_ws_knn_pred->status==0){
                                // movement planned successfully
                                if(mov_type==1 || mov_type==5){ // move movement
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_knn_pred->final_warm_start_res).at(0);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_knn_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");
                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_knn_plan.push_back(1); iter_ws_knn_plan.push_back(plan_tar.iterations); cpu_ws_knn_plan.push_back(plan_tar.cpu_time); obj_ws_knn_plan.push_back(plan_tar.obj_value);
                                    success_ws_knn_bounce.push_back(1); iter_ws_knn_bounce.push_back(bounce_ws.iterations); cpu_ws_knn_bounce.push_back(bounce_ws.cpu_time); obj_ws_knn_bounce.push_back(bounce_ws.obj_value);
                                }else{
                                    // plan stage
                                    HUMotion::warm_start_params plan_tar = (h_results_ws_knn_pred->final_warm_start_res).at(1);
                                    string iter_f_plan_str = to_string(plan_tar.iterations);
                                    string cpu_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.cpu_time)); boost::replace_all(cpu_f_plan_str,",",".");
                                    string obj_f_plan_str =  boost::str(boost::format("%.8f") % (plan_tar.obj_value)); boost::replace_all(obj_f_plan_str,",",".");
                                    // approach stage
                                    HUMotion::warm_start_params approach_tar = (h_results_ws_knn_pred->final_warm_start_res).at(0);
                                    string iter_f_approach_str = to_string(approach_tar.iterations);
                                    string cpu_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.cpu_time)); boost::replace_all(cpu_f_approach_str,",",".");
                                    string obj_f_approach_str =  boost::str(boost::format("%.8f") % (approach_tar.obj_value)); boost::replace_all(obj_f_approach_str,",",".");
                                    // retreat stage
                                    HUMotion::warm_start_params retreat_tar = (h_results_ws_knn_pred->final_warm_start_res).at(2);
                                    string iter_f_retreat_str = to_string(retreat_tar.iterations);
                                    string cpu_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.cpu_time)); boost::replace_all(cpu_f_retreat_str,",",".");
                                    string obj_f_retreat_str =  boost::str(boost::format("%.8f") % (retreat_tar.obj_value)); boost::replace_all(obj_f_retreat_str,",",".");
                                    // bounce
                                    HUMotion::warm_start_params bounce_ws = h_results_ws_knn_pred->bounce_warm_start_res;
                                    string iter_bounce_str = to_string(bounce_ws.iterations);
                                    string cpu_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.cpu_time)); boost::replace_all(cpu_bounce_str,",",".");
                                    string obj_bounce_str =  boost::str(boost::format("%.8f") % (bounce_ws.obj_value)); boost::replace_all(obj_bounce_str,",",".");

                                    pred_csv << ",1,"+iter_f_plan_str+","+cpu_f_plan_str+","+obj_f_plan_str+
                                                ",1,"+iter_f_approach_str+","+cpu_f_approach_str+","+obj_f_approach_str+
                                                ",1,"+iter_f_retreat_str+","+cpu_f_retreat_str+","+obj_f_retreat_str+
                                                ",1,"+iter_bounce_str+","+cpu_bounce_str+","+obj_bounce_str;
                                    success_ws_knn_plan.push_back(1); iter_ws_knn_plan.push_back(plan_tar.iterations); cpu_ws_knn_plan.push_back(plan_tar.cpu_time); obj_ws_knn_plan.push_back(plan_tar.obj_value);
                                    success_ws_knn_app.push_back(1); iter_ws_knn_app.push_back(approach_tar.iterations); cpu_ws_knn_app.push_back(approach_tar.cpu_time); obj_ws_knn_app.push_back(approach_tar.obj_value);
                                    success_ws_knn_ret.push_back(1); iter_ws_knn_ret.push_back(retreat_tar.iterations); cpu_ws_knn_ret.push_back(retreat_tar.cpu_time); obj_ws_knn_ret.push_back(retreat_tar.obj_value);
                                    success_ws_knn_bounce.push_back(1); iter_ws_knn_bounce.push_back(bounce_ws.iterations); cpu_ws_knn_bounce.push_back(bounce_ws.cpu_time); obj_ws_knn_bounce.push_back(bounce_ws.obj_value);
                                }
                            }else{
                                if(mov_type==1 || mov_type==5){ // move movement
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_knn_plan.push_back(0);
                                    success_ws_knn_bounce.push_back(0);
                                }else{
                                    pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                    success_ws_knn_plan.push_back(0);
                                    success_ws_knn_app.push_back(0);
                                    success_ws_knn_ret.push_back(0);
                                    success_ws_knn_bounce.push_back(0);
                                }
                                qnode.log(QNode::Info,std::string("Planning with warm start KNN failed"));
                            }
                        }else{
                            if(mov_type==1 || mov_type==5){ // move movement
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_knn_plan.push_back(0);
                                success_ws_knn_bounce.push_back(0);
                            }else{
                                pred_csv << ",0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan,0,nan,nan,nan";
                                success_ws_knn_plan.push_back(0);
                                success_ws_knn_app.push_back(0);
                                success_ws_knn_ret.push_back(0);
                                success_ws_knn_bounce.push_back(0);
                            }
                            qnode.log(QNode::Info,std::string("Planning with warm start KNN failed"));
                        }
                        pred_csv << "\n";
                    }// for loop predictions
                    pred_csv.close();
                }
                break;
            default:
                break;
            }

            // diplay the results
            std::unordered_map<int,int> m; int n1;

            // plan stage
            if(!success_cold_plan.empty() && !iter_cold_plan.empty()){
                // cold start
                count_occurrence(m, success_cold_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_cold_plan = 100*(n1/double(success_cold_plan.size()));
                this->ui.label_rate_cold_plan_value->setText(QString::number(succ_rate_cold_plan) + QString(" %"));

                double iter_cold_plan_mean = accumulate( iter_cold_plan.begin(), iter_cold_plan.end(), 0.0)/iter_cold_plan.size();
                this->ui.label_iter_mean_cold_plan_value->setText(QString::number(iter_cold_plan_mean));
                double iter_cold_plan_sq_sum = std::inner_product(iter_cold_plan.begin(), iter_cold_plan.end(), iter_cold_plan.begin(), 0.0);
                double iter_cold_plan_std = std::sqrt(iter_cold_plan_sq_sum / iter_cold_plan.size() - iter_cold_plan_mean * iter_cold_plan_mean);
                this->ui.label_iter_std_cold_plan_value->setText(QString::number(iter_cold_plan_std));
                double iter_cold_plan_median = median(iter_cold_plan);
                this->ui.label_iter_median_cold_plan_value->setText(QString::number(iter_cold_plan_median));
                double iter_cold_plan_max= *std::max_element(iter_cold_plan.begin(),iter_cold_plan.end());
                this->ui.label_iter_max_cold_plan_value->setText(QString::number(iter_cold_plan_max));

                double cpu_cold_plan_mean = accumulate( cpu_cold_plan.begin(), cpu_cold_plan.end(), 0.0)/cpu_cold_plan.size();
                this->ui.label_cpu_mean_cold_plan_value->setText(QString::number(cpu_cold_plan_mean));
                double cpu_cold_plan_sq_sum = std::inner_product(cpu_cold_plan.begin(), cpu_cold_plan.end(), cpu_cold_plan.begin(), 0.0);
                double cpu_cold_plan_std = std::sqrt(cpu_cold_plan_sq_sum / cpu_cold_plan.size() - cpu_cold_plan_mean * cpu_cold_plan_mean);
                this->ui.label_cpu_std_cold_plan_value->setText(QString::number(cpu_cold_plan_std));
                double cpu_cold_plan_median = median(cpu_cold_plan);
                this->ui.label_cpu_median_cold_plan_value->setText(QString::number(cpu_cold_plan_median));
                double cpu_cold_plan_max= *std::max_element(cpu_cold_plan.begin(),cpu_cold_plan.end());
                this->ui.label_cpu_max_cold_plan_value->setText(QString::number(cpu_cold_plan_max));

                double obj_cold_plan_mean = accumulate( obj_cold_plan.begin(), obj_cold_plan.end(), 0.0)/obj_cold_plan.size();
                this->ui.label_obj_mean_cold_plan_value->setText(QString::number(obj_cold_plan_mean));
                double obj_cold_plan_sq_sum = std::inner_product(obj_cold_plan.begin(), obj_cold_plan.end(), obj_cold_plan.begin(), 0.0);
                double obj_cold_plan_std = std::sqrt(obj_cold_plan_sq_sum / obj_cold_plan.size() - obj_cold_plan_mean * obj_cold_plan_mean);
                this->ui.label_obj_std_cold_plan_value->setText(QString::number(obj_cold_plan_std));
                double obj_cold_plan_median = median(obj_cold_plan);
                this->ui.label_obj_median_cold_plan_value->setText(QString::number(obj_cold_plan_median));
                double obj_cold_plan_max= *std::max_element(obj_cold_plan.begin(),obj_cold_plan.end());
                this->ui.label_obj_max_cold_plan_value->setText(QString::number(obj_cold_plan_max));
            }else{
                this->ui.label_rate_cold_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_cold_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_cold_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_cold_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_cold_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_cold_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_cold_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_cold_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_cold_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_cold_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_cold_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_cold_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_cold_plan_value->setText(QString("nan"));
            }
            if(!success_ws_or_plan.empty() && !iter_ws_or_plan.empty()){
                // warm start with the original solution
                count_occurrence(m, success_ws_or_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_or_plan = 100*(n1/double(success_ws_or_plan.size()));
                this->ui.label_rate_ws_or_plan_value->setText(QString::number(succ_rate_ws_or_plan) + QString(" %"));

                double iter_ws_or_plan_mean = accumulate( iter_ws_or_plan.begin(), iter_ws_or_plan.end(), 0.0)/iter_ws_or_plan.size();
                this->ui.label_iter_mean_ws_or_plan_value->setText(QString::number(iter_ws_or_plan_mean));
                double iter_ws_or_plan_sq_sum = std::inner_product(iter_ws_or_plan.begin(), iter_ws_or_plan.end(), iter_ws_or_plan.begin(), 0.0);
                double iter_ws_or_plan_std = std::sqrt(iter_ws_or_plan_sq_sum / iter_ws_or_plan.size() - iter_ws_or_plan_mean * iter_ws_or_plan_mean);
                this->ui.label_iter_std_ws_or_plan_value->setText(QString::number(iter_ws_or_plan_std));
                double iter_ws_or_plan_median = median(iter_ws_or_plan);
                this->ui.label_iter_median_ws_or_plan_value->setText(QString::number(iter_ws_or_plan_median));
                double iter_ws_or_plan_max= *std::max_element(iter_ws_or_plan.begin(),iter_ws_or_plan.end());
                this->ui.label_iter_max_ws_or_plan_value->setText(QString::number(iter_ws_or_plan_max));

                double cpu_ws_or_plan_mean = accumulate( cpu_ws_or_plan.begin(), cpu_ws_or_plan.end(), 0.0)/cpu_ws_or_plan.size();
                this->ui.label_cpu_mean_ws_or_plan_value->setText(QString::number(cpu_ws_or_plan_mean));
                double cpu_ws_or_plan_sq_sum = std::inner_product(cpu_ws_or_plan.begin(), cpu_ws_or_plan.end(), cpu_ws_or_plan.begin(), 0.0);
                double cpu_ws_or_plan_std = std::sqrt(cpu_ws_or_plan_sq_sum / cpu_ws_or_plan.size() - cpu_ws_or_plan_mean * cpu_ws_or_plan_mean);
                this->ui.label_cpu_std_ws_or_plan_value->setText(QString::number(cpu_ws_or_plan_std));
                double cpu_ws_or_plan_median = median(cpu_ws_or_plan);
                this->ui.label_cpu_median_ws_or_plan_value->setText(QString::number(cpu_ws_or_plan_median));
                double cpu_ws_or_plan_max= *std::max_element(cpu_ws_or_plan.begin(),cpu_ws_or_plan.end());
                this->ui.label_cpu_max_ws_or_plan_value->setText(QString::number(cpu_ws_or_plan_max));

                double obj_ws_or_plan_mean = accumulate( obj_ws_or_plan.begin(), obj_ws_or_plan.end(), 0.0)/obj_ws_or_plan.size();
                this->ui.label_obj_mean_ws_or_plan_value->setText(QString::number(obj_ws_or_plan_mean));
                double obj_ws_or_plan_sq_sum = std::inner_product(obj_ws_or_plan.begin(), obj_ws_or_plan.end(), obj_ws_or_plan.begin(), 0.0);
                double obj_ws_or_plan_std = std::sqrt(obj_ws_or_plan_sq_sum / obj_ws_or_plan.size() - obj_ws_or_plan_mean * obj_ws_or_plan_mean);
                this->ui.label_obj_std_ws_or_plan_value->setText(QString::number(obj_ws_or_plan_std));
                double obj_ws_or_plan_median = median(obj_ws_or_plan);
                this->ui.label_obj_median_ws_or_plan_value->setText(QString::number(obj_ws_or_plan_median));
                double obj_ws_or_plan_max= *std::max_element(obj_ws_or_plan.begin(),obj_ws_or_plan.end());
                this->ui.label_obj_max_ws_or_plan_value->setText(QString::number(obj_ws_or_plan_max));
            }else{
                this->ui.label_rate_ws_or_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_or_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_or_plan_value->setText(QString("nan"));
            }
            if(!success_ws_rdm_plan.empty() && !iter_ws_rdm_plan.empty()){
                // warm start with the random solution
                count_occurrence(m, success_ws_rdm_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_rdm_plan = 100*(n1/double(success_ws_rdm_plan.size()));
                this->ui.label_rate_ws_rdm_plan_value->setText(QString::number(succ_rate_ws_rdm_plan) + QString(" %"));

                double iter_ws_rdm_plan_mean = accumulate( iter_ws_rdm_plan.begin(), iter_ws_rdm_plan.end(), 0.0)/iter_ws_rdm_plan.size();
                this->ui.label_iter_mean_ws_rdm_plan_value->setText(QString::number(iter_ws_rdm_plan_mean));
                double iter_ws_rdm_plan_sq_sum = std::inner_product(iter_ws_rdm_plan.begin(), iter_ws_rdm_plan.end(), iter_ws_rdm_plan.begin(), 0.0);
                double iter_ws_rdm_plan_std = std::sqrt(iter_ws_rdm_plan_sq_sum / iter_ws_rdm_plan.size() - iter_ws_rdm_plan_mean * iter_ws_rdm_plan_mean);
                this->ui.label_iter_std_ws_rdm_plan_value->setText(QString::number(iter_ws_rdm_plan_std));
                double iter_ws_rdm_plan_median = median(iter_ws_rdm_plan);
                this->ui.label_iter_median_ws_rdm_plan_value->setText(QString::number(iter_ws_rdm_plan_median));
                double iter_ws_rdm_plan_max= *std::max_element(iter_ws_rdm_plan.begin(),iter_ws_rdm_plan.end());
                this->ui.label_iter_max_ws_rdm_plan_value->setText(QString::number(iter_ws_rdm_plan_max));

                double cpu_ws_rdm_plan_mean = accumulate( cpu_ws_rdm_plan.begin(), cpu_ws_rdm_plan.end(), 0.0)/cpu_ws_rdm_plan.size();
                this->ui.label_cpu_mean_ws_rdm_plan_value->setText(QString::number(cpu_ws_rdm_plan_mean));
                double cpu_ws_rdm_plan_sq_sum = std::inner_product(cpu_ws_rdm_plan.begin(), cpu_ws_rdm_plan.end(), cpu_ws_rdm_plan.begin(), 0.0);
                double cpu_ws_rdm_plan_std = std::sqrt(cpu_ws_rdm_plan_sq_sum / cpu_ws_rdm_plan.size() - cpu_ws_rdm_plan_mean * cpu_ws_rdm_plan_mean);
                this->ui.label_cpu_std_ws_rdm_plan_value->setText(QString::number(cpu_ws_rdm_plan_std));
                double cpu_ws_rdm_plan_median = median(cpu_ws_rdm_plan);
                this->ui.label_cpu_median_ws_rdm_plan_value->setText(QString::number(cpu_ws_rdm_plan_median));
                double cpu_ws_rdm_plan_max= *std::max_element(cpu_ws_rdm_plan.begin(),cpu_ws_rdm_plan.end());
                this->ui.label_cpu_max_ws_rdm_plan_value->setText(QString::number(cpu_ws_rdm_plan_max));

                double obj_ws_rdm_plan_mean = accumulate( obj_ws_rdm_plan.begin(), obj_ws_rdm_plan.end(), 0.0)/obj_ws_rdm_plan.size();
                this->ui.label_obj_mean_ws_rdm_plan_value->setText(QString::number(obj_ws_rdm_plan_mean));
                double obj_ws_rdm_plan_sq_sum = std::inner_product(obj_ws_rdm_plan.begin(), obj_ws_rdm_plan.end(), obj_ws_rdm_plan.begin(), 0.0);
                double obj_ws_rdm_plan_std = std::sqrt(obj_ws_rdm_plan_sq_sum / obj_ws_rdm_plan.size() - obj_ws_rdm_plan_mean * obj_ws_rdm_plan_mean);
                this->ui.label_obj_std_ws_rdm_plan_value->setText(QString::number(obj_ws_rdm_plan_std));
                double obj_ws_rdm_plan_median = median(obj_ws_rdm_plan);
                this->ui.label_obj_median_ws_rdm_plan_value->setText(QString::number(obj_ws_rdm_plan_median));
                double obj_ws_rdm_plan_max= *std::max_element(obj_ws_rdm_plan.begin(),obj_ws_rdm_plan.end());
                this->ui.label_obj_max_ws_rdm_plan_value->setText(QString::number(obj_ws_rdm_plan_max));
            }else{
                this->ui.label_rate_ws_rdm_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_rdm_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_rdm_plan_value->setText(QString("nan"));
            }
            if(!success_ws_nn_plan.empty() && !iter_ws_nn_plan.empty()){
                // warm start with the neural network solution
                count_occurrence(m, success_ws_nn_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_nn_plan = 100*(n1/double(success_ws_nn_plan.size()));
                this->ui.label_rate_ws_nn_plan_value->setText(QString::number(succ_rate_ws_nn_plan) + QString(" %"));

                double iter_ws_nn_plan_mean = accumulate( iter_ws_nn_plan.begin(), iter_ws_nn_plan.end(), 0.0)/iter_ws_nn_plan.size();
                this->ui.label_iter_mean_ws_nn_plan_value->setText(QString::number(iter_ws_nn_plan_mean));
                double iter_ws_nn_plan_sq_sum = std::inner_product(iter_ws_nn_plan.begin(), iter_ws_nn_plan.end(), iter_ws_nn_plan.begin(), 0.0);
                double iter_ws_nn_plan_std = std::sqrt(iter_ws_nn_plan_sq_sum / iter_ws_nn_plan.size() - iter_ws_nn_plan_mean * iter_ws_nn_plan_mean);
                this->ui.label_iter_std_ws_nn_plan_value->setText(QString::number(iter_ws_nn_plan_std));
                double iter_ws_nn_plan_median = median(iter_ws_nn_plan);
                this->ui.label_iter_median_ws_nn_plan_value->setText(QString::number(iter_ws_nn_plan_median));
                double iter_ws_nn_plan_max= *std::max_element(iter_ws_nn_plan.begin(),iter_ws_nn_plan.end());
                this->ui.label_iter_max_ws_nn_plan_value->setText(QString::number(iter_ws_nn_plan_max));

                double cpu_ws_nn_plan_mean = accumulate( cpu_ws_nn_plan.begin(), cpu_ws_nn_plan.end(), 0.0)/cpu_ws_nn_plan.size();
                this->ui.label_cpu_mean_ws_nn_plan_value->setText(QString::number(cpu_ws_nn_plan_mean));
                double cpu_ws_nn_plan_sq_sum = std::inner_product(cpu_ws_nn_plan.begin(), cpu_ws_nn_plan.end(), cpu_ws_nn_plan.begin(), 0.0);
                double cpu_ws_nn_plan_std = std::sqrt(cpu_ws_nn_plan_sq_sum / cpu_ws_nn_plan.size() - cpu_ws_nn_plan_mean * cpu_ws_nn_plan_mean);
                this->ui.label_cpu_std_ws_nn_plan_value->setText(QString::number(cpu_ws_nn_plan_std));
                double cpu_ws_nn_plan_median = median(cpu_ws_nn_plan);
                this->ui.label_cpu_median_ws_nn_plan_value->setText(QString::number(cpu_ws_nn_plan_median));
                double cpu_ws_nn_plan_max= *std::max_element(cpu_ws_nn_plan.begin(),cpu_ws_nn_plan.end());
                this->ui.label_cpu_max_ws_nn_plan_value->setText(QString::number(cpu_ws_nn_plan_max));

                double obj_ws_nn_plan_mean = accumulate( obj_ws_nn_plan.begin(), obj_ws_nn_plan.end(), 0.0)/obj_ws_nn_plan.size();
                this->ui.label_obj_mean_ws_nn_plan_value->setText(QString::number(obj_ws_nn_plan_mean));
                double obj_ws_nn_plan_sq_sum = std::inner_product(obj_ws_nn_plan.begin(), obj_ws_nn_plan.end(), obj_ws_nn_plan.begin(), 0.0);
                double obj_ws_nn_plan_std = std::sqrt(obj_ws_nn_plan_sq_sum / obj_ws_nn_plan.size() - obj_ws_nn_plan_mean * obj_ws_nn_plan_mean);
                this->ui.label_obj_std_ws_nn_plan_value->setText(QString::number(obj_ws_nn_plan_std));
                double obj_ws_nn_plan_median = median(obj_ws_nn_plan);
                this->ui.label_obj_median_ws_nn_plan_value->setText(QString::number(obj_ws_nn_plan_median));
                double obj_ws_nn_plan_max= *std::max_element(obj_ws_nn_plan.begin(),obj_ws_nn_plan.end());
                this->ui.label_obj_max_ws_nn_plan_value->setText(QString::number(obj_ws_nn_plan_max));
            }else{
                this->ui.label_rate_ws_nn_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_nn_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_nn_plan_value->setText(QString("nan"));
            }
            if(!success_ws_svm_plan.empty() && !iter_ws_svm_plan.empty()){
                // warm start with the support vector machines solution
                count_occurrence(m, success_ws_svm_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_svm_plan = 100*(n1/double(success_ws_svm_plan.size()));
                this->ui.label_rate_ws_svm_plan_value->setText(QString::number(succ_rate_ws_svm_plan) + QString(" %"));

                double iter_ws_svm_plan_mean = accumulate( iter_ws_svm_plan.begin(), iter_ws_svm_plan.end(), 0.0)/iter_ws_svm_plan.size();
                this->ui.label_iter_mean_ws_svm_plan_value->setText(QString::number(iter_ws_svm_plan_mean));
                double iter_ws_svm_plan_sq_sum = std::inner_product(iter_ws_svm_plan.begin(), iter_ws_svm_plan.end(), iter_ws_svm_plan.begin(), 0.0);
                double iter_ws_svm_plan_std = std::sqrt(iter_ws_svm_plan_sq_sum / iter_ws_svm_plan.size() - iter_ws_svm_plan_mean * iter_ws_svm_plan_mean);
                this->ui.label_iter_std_ws_svm_plan_value->setText(QString::number(iter_ws_svm_plan_std));
                double iter_ws_svm_plan_median = median(iter_ws_svm_plan);
                this->ui.label_iter_median_ws_svm_plan_value->setText(QString::number(iter_ws_svm_plan_median));
                double iter_ws_svm_plan_max= *std::max_element(iter_ws_svm_plan.begin(),iter_ws_svm_plan.end());
                this->ui.label_iter_max_ws_svm_plan_value->setText(QString::number(iter_ws_svm_plan_max));

                double cpu_ws_svm_plan_mean = accumulate( cpu_ws_svm_plan.begin(), cpu_ws_svm_plan.end(), 0.0)/cpu_ws_svm_plan.size();
                this->ui.label_cpu_mean_ws_svm_plan_value->setText(QString::number(cpu_ws_svm_plan_mean));
                double cpu_ws_svm_plan_sq_sum = std::inner_product(cpu_ws_svm_plan.begin(), cpu_ws_svm_plan.end(), cpu_ws_svm_plan.begin(), 0.0);
                double cpu_ws_svm_plan_std = std::sqrt(cpu_ws_svm_plan_sq_sum / cpu_ws_svm_plan.size() - cpu_ws_svm_plan_mean * cpu_ws_svm_plan_mean);
                this->ui.label_cpu_std_ws_svm_plan_value->setText(QString::number(cpu_ws_svm_plan_std));
                double cpu_ws_svm_plan_median = median(cpu_ws_svm_plan);
                this->ui.label_cpu_median_ws_svm_plan_value->setText(QString::number(cpu_ws_svm_plan_median));
                double cpu_ws_svm_plan_max= *std::max_element(cpu_ws_svm_plan.begin(),cpu_ws_svm_plan.end());
                this->ui.label_cpu_max_ws_svm_plan_value->setText(QString::number(cpu_ws_svm_plan_max));

                double obj_ws_svm_plan_mean = accumulate( obj_ws_svm_plan.begin(), obj_ws_svm_plan.end(), 0.0)/obj_ws_svm_plan.size();
                this->ui.label_obj_mean_ws_svm_plan_value->setText(QString::number(obj_ws_svm_plan_mean));
                double obj_ws_svm_plan_sq_sum = std::inner_product(obj_ws_svm_plan.begin(), obj_ws_svm_plan.end(), obj_ws_svm_plan.begin(), 0.0);
                double obj_ws_svm_plan_std = std::sqrt(obj_ws_svm_plan_sq_sum / obj_ws_svm_plan.size() - obj_ws_svm_plan_mean * obj_ws_svm_plan_mean);
                this->ui.label_obj_std_ws_svm_plan_value->setText(QString::number(obj_ws_svm_plan_std));
                double obj_ws_svm_plan_median = median(obj_ws_svm_plan);
                this->ui.label_obj_median_ws_svm_plan_value->setText(QString::number(obj_ws_svm_plan_median));
                double obj_ws_svm_plan_max= *std::max_element(obj_ws_svm_plan.begin(),obj_ws_svm_plan.end());
                this->ui.label_obj_max_ws_svm_plan_value->setText(QString::number(obj_ws_svm_plan_max));
            }else{
                this->ui.label_rate_ws_svm_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_svm_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_svm_plan_value->setText(QString("nan"));
            }
            if(!success_ws_knn_plan.empty() && !iter_ws_knn_plan.empty()){
                // warm start with the k-nearest neighbors solution
                count_occurrence(m, success_ws_knn_plan); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_knn_plan = 100*(n1/double(success_ws_knn_plan.size()));
                this->ui.label_rate_ws_knn_plan_value->setText(QString::number(succ_rate_ws_knn_plan) + QString(" %"));

                double iter_ws_knn_plan_mean = accumulate( iter_ws_knn_plan.begin(), iter_ws_knn_plan.end(), 0.0)/iter_ws_knn_plan.size();
                this->ui.label_iter_mean_ws_knn_plan_value->setText(QString::number(iter_ws_knn_plan_mean));
                double iter_ws_knn_plan_sq_sum = std::inner_product(iter_ws_knn_plan.begin(), iter_ws_knn_plan.end(), iter_ws_knn_plan.begin(), 0.0);
                double iter_ws_knn_plan_std = std::sqrt(iter_ws_knn_plan_sq_sum / iter_ws_knn_plan.size() - iter_ws_knn_plan_mean * iter_ws_knn_plan_mean);
                this->ui.label_iter_std_ws_knn_plan_value->setText(QString::number(iter_ws_knn_plan_std));
                double iter_ws_knn_plan_median = median(iter_ws_knn_plan);
                this->ui.label_iter_median_ws_knn_plan_value->setText(QString::number(iter_ws_knn_plan_median));
                double iter_ws_knn_plan_max= *std::max_element(iter_ws_knn_plan.begin(),iter_ws_knn_plan.end());
                this->ui.label_iter_max_ws_knn_plan_value->setText(QString::number(iter_ws_knn_plan_max));

                double cpu_ws_knn_plan_mean = accumulate( cpu_ws_knn_plan.begin(), cpu_ws_knn_plan.end(), 0.0)/cpu_ws_knn_plan.size();
                this->ui.label_cpu_mean_ws_knn_plan_value->setText(QString::number(cpu_ws_knn_plan_mean));
                double cpu_ws_knn_plan_sq_sum = std::inner_product(cpu_ws_knn_plan.begin(), cpu_ws_knn_plan.end(), cpu_ws_knn_plan.begin(), 0.0);
                double cpu_ws_knn_plan_std = std::sqrt(cpu_ws_knn_plan_sq_sum / cpu_ws_knn_plan.size() - cpu_ws_knn_plan_mean * cpu_ws_knn_plan_mean);
                this->ui.label_cpu_std_ws_knn_plan_value->setText(QString::number(cpu_ws_knn_plan_std));
                double cpu_ws_knn_plan_median = median(cpu_ws_knn_plan);
                this->ui.label_cpu_median_ws_knn_plan_value->setText(QString::number(cpu_ws_knn_plan_median));
                double cpu_ws_knn_plan_max= *std::max_element(cpu_ws_knn_plan.begin(),cpu_ws_knn_plan.end());
                this->ui.label_cpu_max_ws_knn_plan_value->setText(QString::number(cpu_ws_knn_plan_max));

                double obj_ws_knn_plan_mean = accumulate( obj_ws_knn_plan.begin(), obj_ws_knn_plan.end(), 0.0)/obj_ws_knn_plan.size();
                this->ui.label_obj_mean_ws_knn_plan_value->setText(QString::number(obj_ws_knn_plan_mean));
                double obj_ws_knn_plan_sq_sum = std::inner_product(obj_ws_knn_plan.begin(), obj_ws_knn_plan.end(), obj_ws_knn_plan.begin(), 0.0);
                double obj_ws_knn_plan_std = std::sqrt(obj_ws_knn_plan_sq_sum / obj_ws_knn_plan.size() - obj_ws_knn_plan_mean * obj_ws_knn_plan_mean);
                this->ui.label_obj_std_ws_knn_plan_value->setText(QString::number(obj_ws_knn_plan_std));
                double obj_ws_knn_plan_median = median(obj_ws_knn_plan);
                this->ui.label_obj_median_ws_knn_plan_value->setText(QString::number(obj_ws_knn_plan_median));
                double obj_ws_knn_plan_max= *std::max_element(obj_ws_knn_plan.begin(),obj_ws_knn_plan.end());
                this->ui.label_obj_max_ws_knn_plan_value->setText(QString::number(obj_ws_knn_plan_max));
            }else{
                this->ui.label_rate_ws_knn_plan_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_knn_plan_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_knn_plan_value->setText(QString("nan"));
            }

            // approach stage
            if(!success_cold_app.empty() && !iter_cold_app.empty()){
                // cold start
                count_occurrence(m, success_cold_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_cold_app = 100*(n1/double(success_cold_app.size()));
                this->ui.label_rate_cold_app_value->setText(QString::number(succ_rate_cold_app) + QString(" %"));

                double iter_cold_app_mean = accumulate( iter_cold_app.begin(), iter_cold_app.end(), 0.0)/iter_cold_app.size();
                this->ui.label_iter_mean_cold_app_value->setText(QString::number(iter_cold_app_mean));
                double iter_cold_app_sq_sum = std::inner_product(iter_cold_app.begin(), iter_cold_app.end(), iter_cold_app.begin(), 0.0);
                double iter_cold_app_std = std::sqrt(iter_cold_app_sq_sum / iter_cold_app.size() - iter_cold_app_mean * iter_cold_app_mean);
                this->ui.label_iter_std_cold_app_value->setText(QString::number(iter_cold_app_std));
                double iter_cold_app_median = median(iter_cold_app);
                this->ui.label_iter_median_cold_app_value->setText(QString::number(iter_cold_app_median));
                double iter_cold_app_max= *std::max_element(iter_cold_app.begin(),iter_cold_app.end());
                this->ui.label_iter_max_cold_app_value->setText(QString::number(iter_cold_app_max));

                double cpu_cold_app_mean = accumulate( cpu_cold_app.begin(), cpu_cold_app.end(), 0.0)/cpu_cold_app.size();
                this->ui.label_cpu_mean_cold_app_value->setText(QString::number(cpu_cold_app_mean));
                double cpu_cold_app_sq_sum = std::inner_product(cpu_cold_app.begin(), cpu_cold_app.end(), cpu_cold_app.begin(), 0.0);
                double cpu_cold_app_std = std::sqrt(cpu_cold_app_sq_sum / cpu_cold_app.size() - cpu_cold_app_mean * cpu_cold_app_mean);
                this->ui.label_cpu_std_cold_app_value->setText(QString::number(cpu_cold_app_std));
                double cpu_cold_app_median = median(cpu_cold_app);
                this->ui.label_cpu_median_cold_app_value->setText(QString::number(cpu_cold_app_median));
                double cpu_cold_app_max= *std::max_element(cpu_cold_app.begin(),cpu_cold_app.end());
                this->ui.label_cpu_max_cold_app_value->setText(QString::number(cpu_cold_app_max));

                double obj_cold_app_mean = accumulate( obj_cold_app.begin(), obj_cold_app.end(), 0.0)/obj_cold_app.size();
                this->ui.label_obj_mean_cold_app_value->setText(QString::number(obj_cold_app_mean));
                double obj_cold_app_sq_sum = std::inner_product(obj_cold_app.begin(), obj_cold_app.end(), obj_cold_app.begin(), 0.0);
                double obj_cold_app_std = std::sqrt(obj_cold_app_sq_sum / obj_cold_app.size() - obj_cold_app_mean * obj_cold_app_mean);
                this->ui.label_obj_std_cold_app_value->setText(QString::number(obj_cold_app_std));
                double obj_cold_app_median = median(obj_cold_app);
                this->ui.label_obj_median_cold_app_value->setText(QString::number(obj_cold_app_median));
                double obj_cold_app_max= *std::max_element(obj_cold_app.begin(),obj_cold_app.end());
                this->ui.label_obj_max_cold_app_value->setText(QString::number(obj_cold_app_max));
            }else{
                this->ui.label_rate_cold_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_cold_app_value->setText(QString("nan"));
                this->ui.label_iter_std_cold_app_value->setText(QString("nan"));
                this->ui.label_iter_median_cold_app_value->setText(QString("nan"));
                this->ui.label_iter_max_cold_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_cold_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_cold_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_cold_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_cold_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_cold_app_value->setText(QString("nan"));
                this->ui.label_obj_std_cold_app_value->setText(QString("nan"));
                this->ui.label_obj_median_cold_app_value->setText(QString("nan"));
                this->ui.label_obj_max_cold_app_value->setText(QString("nan"));
            }
            if(!success_ws_or_app.empty() && !iter_ws_or_app.empty()){
                // warm start with the original solution
                count_occurrence(m, success_ws_or_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_or_app = 100*(n1/double(success_ws_or_app.size()));
                this->ui.label_rate_ws_or_app_value->setText(QString::number(succ_rate_ws_or_app) + QString(" %"));

                double iter_ws_or_app_mean = accumulate( iter_ws_or_app.begin(), iter_ws_or_app.end(), 0.0)/iter_ws_or_app.size();
                this->ui.label_iter_mean_ws_or_app_value->setText(QString::number(iter_ws_or_app_mean));
                double iter_ws_or_app_sq_sum = std::inner_product(iter_ws_or_app.begin(), iter_ws_or_app.end(), iter_ws_or_app.begin(), 0.0);
                double iter_ws_or_app_std = std::sqrt(iter_ws_or_app_sq_sum / iter_ws_or_app.size() - iter_ws_or_app_mean * iter_ws_or_app_mean);
                this->ui.label_iter_std_ws_or_app_value->setText(QString::number(iter_ws_or_app_std));
                double iter_ws_or_app_median = median(iter_ws_or_app);
                this->ui.label_iter_median_ws_or_app_value->setText(QString::number(iter_ws_or_app_median));
                double iter_ws_or_app_max= *std::max_element(iter_ws_or_app.begin(),iter_ws_or_app.end());
                this->ui.label_iter_max_ws_or_app_value->setText(QString::number(iter_ws_or_app_max));

                double cpu_ws_or_app_mean = accumulate( cpu_ws_or_app.begin(), cpu_ws_or_app.end(), 0.0)/cpu_ws_or_app.size();
                this->ui.label_cpu_mean_ws_or_app_value->setText(QString::number(cpu_ws_or_app_mean));
                double cpu_ws_or_app_sq_sum = std::inner_product(cpu_ws_or_app.begin(), cpu_ws_or_app.end(), cpu_ws_or_app.begin(), 0.0);
                double cpu_ws_or_app_std = std::sqrt(cpu_ws_or_app_sq_sum / cpu_ws_or_app.size() - cpu_ws_or_app_mean * cpu_ws_or_app_mean);
                this->ui.label_cpu_std_ws_or_app_value->setText(QString::number(cpu_ws_or_app_std));
                double cpu_ws_or_app_median = median(cpu_ws_or_app);
                this->ui.label_cpu_median_ws_or_app_value->setText(QString::number(cpu_ws_or_app_median));
                double cpu_ws_or_app_max= *std::max_element(cpu_ws_or_app.begin(),cpu_ws_or_app.end());
                this->ui.label_cpu_max_ws_or_app_value->setText(QString::number(cpu_ws_or_app_max));

                double obj_ws_or_app_mean = accumulate( obj_ws_or_app.begin(), obj_ws_or_app.end(), 0.0)/obj_ws_or_app.size();
                this->ui.label_obj_mean_ws_or_app_value->setText(QString::number(obj_ws_or_app_mean));
                double obj_ws_or_app_sq_sum = std::inner_product(obj_ws_or_app.begin(), obj_ws_or_app.end(), obj_ws_or_app.begin(), 0.0);
                double obj_ws_or_app_std = std::sqrt(obj_ws_or_app_sq_sum / obj_ws_or_app.size() - obj_ws_or_app_mean * obj_ws_or_app_mean);
                this->ui.label_obj_std_ws_or_app_value->setText(QString::number(obj_ws_or_app_std));
                double obj_ws_or_app_median = median(obj_ws_or_app);
                this->ui.label_obj_median_ws_or_app_value->setText(QString::number(obj_ws_or_app_median));
                double obj_ws_or_app_max= *std::max_element(obj_ws_or_app.begin(),obj_ws_or_app.end());
                this->ui.label_obj_max_ws_or_app_value->setText(QString::number(obj_ws_or_app_max));
            }else{
                this->ui.label_rate_ws_or_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_or_app_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_or_app_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_or_app_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_or_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_or_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_or_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_or_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_or_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_or_app_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_or_app_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_or_app_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_or_app_value->setText(QString("nan"));
            }
            if(!success_ws_rdm_app.empty() && !iter_ws_rdm_app.empty()){
                // warm start with the random solution
                count_occurrence(m, success_ws_rdm_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_rdm_app = 100*(n1/double(success_ws_rdm_app.size()));
                this->ui.label_rate_ws_rdm_app_value->setText(QString::number(succ_rate_ws_rdm_app) + QString(" %"));

                double iter_ws_rdm_app_mean = accumulate( iter_ws_rdm_app.begin(), iter_ws_rdm_app.end(), 0.0)/iter_ws_rdm_app.size();
                this->ui.label_iter_mean_ws_rdm_app_value->setText(QString::number(iter_ws_rdm_app_mean));
                double iter_ws_rdm_app_sq_sum = std::inner_product(iter_ws_rdm_app.begin(), iter_ws_rdm_app.end(), iter_ws_rdm_app.begin(), 0.0);
                double iter_ws_rdm_app_std = std::sqrt(iter_ws_rdm_app_sq_sum / iter_ws_rdm_app.size() - iter_ws_rdm_app_mean * iter_ws_rdm_app_mean);
                this->ui.label_iter_std_ws_rdm_app_value->setText(QString::number(iter_ws_rdm_app_std));
                double iter_ws_rdm_app_median = median(iter_ws_rdm_app);
                this->ui.label_iter_median_ws_rdm_app_value->setText(QString::number(iter_ws_rdm_app_median));
                double iter_ws_rdm_app_max= *std::max_element(iter_ws_rdm_app.begin(),iter_ws_rdm_app.end());
                this->ui.label_iter_max_ws_rdm_app_value->setText(QString::number(iter_ws_rdm_app_max));

                double cpu_ws_rdm_app_mean = accumulate( cpu_ws_rdm_app.begin(), cpu_ws_rdm_app.end(), 0.0)/cpu_ws_rdm_app.size();
                this->ui.label_cpu_mean_ws_rdm_app_value->setText(QString::number(cpu_ws_rdm_app_mean));
                double cpu_ws_rdm_app_sq_sum = std::inner_product(cpu_ws_rdm_app.begin(), cpu_ws_rdm_app.end(), cpu_ws_rdm_app.begin(), 0.0);
                double cpu_ws_rdm_app_std = std::sqrt(cpu_ws_rdm_app_sq_sum / cpu_ws_rdm_app.size() - cpu_ws_rdm_app_mean * cpu_ws_rdm_app_mean);
                this->ui.label_cpu_std_ws_rdm_app_value->setText(QString::number(cpu_ws_rdm_app_std));
                double cpu_ws_rdm_app_median = median(cpu_ws_rdm_app);
                this->ui.label_cpu_median_ws_rdm_app_value->setText(QString::number(cpu_ws_rdm_app_median));
                double cpu_ws_rdm_app_max= *std::max_element(cpu_ws_rdm_app.begin(),cpu_ws_rdm_app.end());
                this->ui.label_cpu_max_ws_rdm_app_value->setText(QString::number(cpu_ws_rdm_app_max));

                double obj_ws_rdm_app_mean = accumulate( obj_ws_rdm_app.begin(), obj_ws_rdm_app.end(), 0.0)/obj_ws_rdm_app.size();
                this->ui.label_obj_mean_ws_rdm_app_value->setText(QString::number(obj_ws_rdm_app_mean));
                double obj_ws_rdm_app_sq_sum = std::inner_product(obj_ws_rdm_app.begin(), obj_ws_rdm_app.end(), obj_ws_rdm_app.begin(), 0.0);
                double obj_ws_rdm_app_std = std::sqrt(obj_ws_rdm_app_sq_sum / obj_ws_rdm_app.size() - obj_ws_rdm_app_mean * obj_ws_rdm_app_mean);
                this->ui.label_obj_std_ws_rdm_app_value->setText(QString::number(obj_ws_rdm_app_std));
                double obj_ws_rdm_app_median = median(obj_ws_rdm_app);
                this->ui.label_obj_median_ws_rdm_app_value->setText(QString::number(obj_ws_rdm_app_median));
                double obj_ws_rdm_app_max= *std::max_element(obj_ws_rdm_app.begin(),obj_ws_rdm_app.end());
                this->ui.label_obj_max_ws_rdm_app_value->setText(QString::number(obj_ws_rdm_app_max));
            }else{
                this->ui.label_rate_ws_rdm_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_rdm_app_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_rdm_app_value->setText(QString("nan"));
            }
            if(!success_ws_nn_app.empty() && !iter_ws_nn_app.empty()){
                // warm start with the neural network solution
                count_occurrence(m, success_ws_nn_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_nn_app = 100*(n1/double(success_ws_nn_app.size()));
                this->ui.label_rate_ws_nn_app_value->setText(QString::number(succ_rate_ws_nn_app) + QString(" %"));

                double iter_ws_nn_app_mean = accumulate( iter_ws_nn_app.begin(), iter_ws_nn_app.end(), 0.0)/iter_ws_nn_app.size();
                this->ui.label_iter_mean_ws_nn_app_value->setText(QString::number(iter_ws_nn_app_mean));
                double iter_ws_nn_app_sq_sum = std::inner_product(iter_ws_nn_app.begin(), iter_ws_nn_app.end(), iter_ws_nn_app.begin(), 0.0);
                double iter_ws_nn_app_std = std::sqrt(iter_ws_nn_app_sq_sum / iter_ws_nn_app.size() - iter_ws_nn_app_mean * iter_ws_nn_app_mean);
                this->ui.label_iter_std_ws_nn_app_value->setText(QString::number(iter_ws_nn_app_std));
                double iter_ws_nn_app_median = median(iter_ws_nn_app);
                this->ui.label_iter_median_ws_nn_app_value->setText(QString::number(iter_ws_nn_app_median));
                double iter_ws_nn_app_max= *std::max_element(iter_ws_nn_app.begin(),iter_ws_nn_app.end());
                this->ui.label_iter_max_ws_nn_app_value->setText(QString::number(iter_ws_nn_app_max));

                double cpu_ws_nn_app_mean = accumulate( cpu_ws_nn_app.begin(), cpu_ws_nn_app.end(), 0.0)/cpu_ws_nn_app.size();
                this->ui.label_cpu_mean_ws_nn_app_value->setText(QString::number(cpu_ws_nn_app_mean));
                double cpu_ws_nn_app_sq_sum = std::inner_product(cpu_ws_nn_app.begin(), cpu_ws_nn_app.end(), cpu_ws_nn_app.begin(), 0.0);
                double cpu_ws_nn_app_std = std::sqrt(cpu_ws_nn_app_sq_sum / cpu_ws_nn_app.size() - cpu_ws_nn_app_mean * cpu_ws_nn_app_mean);
                this->ui.label_cpu_std_ws_nn_app_value->setText(QString::number(cpu_ws_nn_app_std));
                double cpu_ws_nn_app_median = median(cpu_ws_nn_app);
                this->ui.label_cpu_median_ws_nn_app_value->setText(QString::number(cpu_ws_nn_app_median));
                double cpu_ws_nn_app_max= *std::max_element(cpu_ws_nn_app.begin(),cpu_ws_nn_app.end());
                this->ui.label_cpu_max_ws_nn_app_value->setText(QString::number(cpu_ws_nn_app_max));

                double obj_ws_nn_app_mean = accumulate( obj_ws_nn_app.begin(), obj_ws_nn_app.end(), 0.0)/obj_ws_nn_app.size();
                this->ui.label_obj_mean_ws_nn_app_value->setText(QString::number(obj_ws_nn_app_mean));
                double obj_ws_nn_app_sq_sum = std::inner_product(obj_ws_nn_app.begin(), obj_ws_nn_app.end(), obj_ws_nn_app.begin(), 0.0);
                double obj_ws_nn_app_std = std::sqrt(obj_ws_nn_app_sq_sum / obj_ws_nn_app.size() - obj_ws_nn_app_mean * obj_ws_nn_app_mean);
                this->ui.label_obj_std_ws_nn_app_value->setText(QString::number(obj_ws_nn_app_std));
                double obj_ws_nn_app_median = median(obj_ws_nn_app);
                this->ui.label_obj_median_ws_nn_app_value->setText(QString::number(obj_ws_nn_app_median));
                double obj_ws_nn_app_max= *std::max_element(obj_ws_nn_app.begin(),obj_ws_nn_app.end());
                this->ui.label_obj_max_ws_nn_app_value->setText(QString::number(obj_ws_nn_app_max));
            }else{
                this->ui.label_rate_ws_nn_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_nn_app_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_nn_app_value->setText(QString("nan"));
            }
            if(!success_ws_svm_app.empty() && !iter_ws_svm_app.empty()){
                // warm start with the support vector machines solution
                count_occurrence(m, success_ws_svm_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_svm_app = 100*(n1/double(success_ws_svm_app.size()));
                this->ui.label_rate_ws_svm_app_value->setText(QString::number(succ_rate_ws_svm_app) + QString(" %"));

                double iter_ws_svm_app_mean = accumulate( iter_ws_svm_app.begin(), iter_ws_svm_app.end(), 0.0)/iter_ws_svm_app.size();
                this->ui.label_iter_mean_ws_svm_app_value->setText(QString::number(iter_ws_svm_app_mean));
                double iter_ws_svm_app_sq_sum = std::inner_product(iter_ws_svm_app.begin(), iter_ws_svm_app.end(), iter_ws_svm_app.begin(), 0.0);
                double iter_ws_svm_app_std = std::sqrt(iter_ws_svm_app_sq_sum / iter_ws_svm_app.size() - iter_ws_svm_app_mean * iter_ws_svm_app_mean);
                this->ui.label_iter_std_ws_svm_app_value->setText(QString::number(iter_ws_svm_app_std));
                double iter_ws_svm_app_median = median(iter_ws_svm_app);
                this->ui.label_iter_median_ws_svm_app_value->setText(QString::number(iter_ws_svm_app_median));
                double iter_ws_svm_app_max= *std::max_element(iter_ws_svm_app.begin(),iter_ws_svm_app.end());
                this->ui.label_iter_max_ws_svm_app_value->setText(QString::number(iter_ws_svm_app_max));

                double cpu_ws_svm_app_mean = accumulate( cpu_ws_svm_app.begin(), cpu_ws_svm_app.end(), 0.0)/cpu_ws_svm_app.size();
                this->ui.label_cpu_mean_ws_svm_app_value->setText(QString::number(cpu_ws_svm_app_mean));
                double cpu_ws_svm_app_sq_sum = std::inner_product(cpu_ws_svm_app.begin(), cpu_ws_svm_app.end(), cpu_ws_svm_app.begin(), 0.0);
                double cpu_ws_svm_app_std = std::sqrt(cpu_ws_svm_app_sq_sum / cpu_ws_svm_app.size() - cpu_ws_svm_app_mean * cpu_ws_svm_app_mean);
                this->ui.label_cpu_std_ws_svm_app_value->setText(QString::number(cpu_ws_svm_app_std));
                double cpu_ws_svm_app_median = median(cpu_ws_svm_app);
                this->ui.label_cpu_median_ws_svm_app_value->setText(QString::number(cpu_ws_svm_app_median));
                double cpu_ws_svm_app_max= *std::max_element(cpu_ws_svm_app.begin(),cpu_ws_svm_app.end());
                this->ui.label_cpu_max_ws_svm_app_value->setText(QString::number(cpu_ws_svm_app_max));

                double obj_ws_svm_app_mean = accumulate( obj_ws_svm_app.begin(), obj_ws_svm_app.end(), 0.0)/obj_ws_svm_app.size();
                this->ui.label_obj_mean_ws_svm_app_value->setText(QString::number(obj_ws_svm_app_mean));
                double obj_ws_svm_app_sq_sum = std::inner_product(obj_ws_svm_app.begin(), obj_ws_svm_app.end(), obj_ws_svm_app.begin(), 0.0);
                double obj_ws_svm_app_std = std::sqrt(obj_ws_svm_app_sq_sum / obj_ws_svm_app.size() - obj_ws_svm_app_mean * obj_ws_svm_app_mean);
                this->ui.label_obj_std_ws_svm_app_value->setText(QString::number(obj_ws_svm_app_std));
                double obj_ws_svm_app_median = median(obj_ws_svm_app);
                this->ui.label_obj_median_ws_svm_app_value->setText(QString::number(obj_ws_svm_app_median));
                double obj_ws_svm_app_max= *std::max_element(obj_ws_svm_app.begin(),obj_ws_svm_app.end());
                this->ui.label_obj_max_ws_svm_app_value->setText(QString::number(obj_ws_svm_app_max));
            }else{
                this->ui.label_rate_ws_svm_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_svm_app_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_svm_app_value->setText(QString("nan"));
            }
            if(!success_ws_knn_app.empty() && !iter_ws_knn_app.empty()){
                // warm start with the k-nearest neighbors solution
                count_occurrence(m, success_ws_knn_app); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_knn_app = 100*(n1/double(success_ws_knn_app.size()));
                this->ui.label_rate_ws_knn_app_value->setText(QString::number(succ_rate_ws_knn_app) + QString(" %"));

                double iter_ws_knn_app_mean = accumulate( iter_ws_knn_app.begin(), iter_ws_knn_app.end(), 0.0)/iter_ws_knn_app.size();
                this->ui.label_iter_mean_ws_knn_app_value->setText(QString::number(iter_ws_knn_app_mean));
                double iter_ws_knn_app_sq_sum = std::inner_product(iter_ws_knn_app.begin(), iter_ws_knn_app.end(), iter_ws_knn_app.begin(), 0.0);
                double iter_ws_knn_app_std = std::sqrt(iter_ws_knn_app_sq_sum / iter_ws_knn_app.size() - iter_ws_knn_app_mean * iter_ws_knn_app_mean);
                this->ui.label_iter_std_ws_knn_app_value->setText(QString::number(iter_ws_knn_app_std));
                double iter_ws_knn_app_median = median(iter_ws_knn_app);
                this->ui.label_iter_median_ws_knn_app_value->setText(QString::number(iter_ws_knn_app_median));
                double iter_ws_knn_app_max= *std::max_element(iter_ws_knn_app.begin(),iter_ws_knn_app.end());
                this->ui.label_iter_max_ws_knn_app_value->setText(QString::number(iter_ws_knn_app_max));

                double cpu_ws_knn_app_mean = accumulate( cpu_ws_knn_app.begin(), cpu_ws_knn_app.end(), 0.0)/cpu_ws_knn_app.size();
                this->ui.label_cpu_mean_ws_knn_app_value->setText(QString::number(cpu_ws_knn_app_mean));
                double cpu_ws_knn_app_sq_sum = std::inner_product(cpu_ws_knn_app.begin(), cpu_ws_knn_app.end(), cpu_ws_knn_app.begin(), 0.0);
                double cpu_ws_knn_app_std = std::sqrt(cpu_ws_knn_app_sq_sum / cpu_ws_knn_app.size() - cpu_ws_knn_app_mean * cpu_ws_knn_app_mean);
                this->ui.label_cpu_std_ws_knn_app_value->setText(QString::number(cpu_ws_knn_app_std));
                double cpu_ws_knn_app_median = median(cpu_ws_knn_app);
                this->ui.label_cpu_median_ws_knn_app_value->setText(QString::number(cpu_ws_knn_app_median));
                double cpu_ws_knn_app_max= *std::max_element(cpu_ws_knn_app.begin(),cpu_ws_knn_app.end());
                this->ui.label_cpu_max_ws_knn_app_value->setText(QString::number(cpu_ws_knn_app_max));

                double obj_ws_knn_app_mean = accumulate( obj_ws_knn_app.begin(), obj_ws_knn_app.end(), 0.0)/obj_ws_knn_app.size();
                this->ui.label_obj_mean_ws_knn_app_value->setText(QString::number(obj_ws_knn_app_mean));
                double obj_ws_knn_app_sq_sum = std::inner_product(obj_ws_knn_app.begin(), obj_ws_knn_app.end(), obj_ws_knn_app.begin(), 0.0);
                double obj_ws_knn_app_std = std::sqrt(obj_ws_knn_app_sq_sum / obj_ws_knn_app.size() - obj_ws_knn_app_mean * obj_ws_knn_app_mean);
                this->ui.label_obj_std_ws_knn_app_value->setText(QString::number(obj_ws_knn_app_std));
                double obj_ws_knn_app_median = median(obj_ws_knn_app);
                this->ui.label_obj_median_ws_knn_app_value->setText(QString::number(obj_ws_knn_app_median));
                double obj_ws_knn_app_max= *std::max_element(obj_ws_knn_app.begin(),obj_ws_knn_app.end());
                this->ui.label_obj_max_ws_knn_app_value->setText(QString::number(obj_ws_knn_app_max));
            }else{
                this->ui.label_rate_ws_knn_app_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_knn_app_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_knn_app_value->setText(QString("nan"));
            }

            // retreat stage
            if(!success_cold_ret.empty() && !iter_cold_ret.empty()){
                // cold start
                count_occurrence(m, success_cold_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_cold_ret = 100*(n1/double(success_cold_ret.size()));
                this->ui.label_rate_cold_ret_value->setText(QString::number(succ_rate_cold_ret) + QString(" %"));

                double iter_cold_ret_mean = accumulate( iter_cold_ret.begin(), iter_cold_ret.end(), 0.0)/iter_cold_ret.size();
                this->ui.label_iter_mean_cold_ret_value->setText(QString::number(iter_cold_ret_mean));
                double iter_cold_ret_sq_sum = std::inner_product(iter_cold_ret.begin(), iter_cold_ret.end(), iter_cold_ret.begin(), 0.0);
                double iter_cold_ret_std = std::sqrt(iter_cold_ret_sq_sum / iter_cold_ret.size() - iter_cold_ret_mean * iter_cold_ret_mean);
                this->ui.label_iter_std_cold_ret_value->setText(QString::number(iter_cold_ret_std));
                double iter_cold_ret_median = median(iter_cold_ret);
                this->ui.label_iter_median_cold_ret_value->setText(QString::number(iter_cold_ret_median));
                double iter_cold_ret_max= *std::max_element(iter_cold_ret.begin(),iter_cold_ret.end());
                this->ui.label_iter_max_cold_ret_value->setText(QString::number(iter_cold_ret_max));

                double cpu_cold_ret_mean = accumulate( cpu_cold_ret.begin(), cpu_cold_ret.end(), 0.0)/cpu_cold_ret.size();
                this->ui.label_cpu_mean_cold_ret_value->setText(QString::number(cpu_cold_ret_mean));
                double cpu_cold_ret_sq_sum = std::inner_product(cpu_cold_ret.begin(), cpu_cold_ret.end(), cpu_cold_ret.begin(), 0.0);
                double cpu_cold_ret_std = std::sqrt(cpu_cold_ret_sq_sum / cpu_cold_ret.size() - cpu_cold_ret_mean * cpu_cold_ret_mean);
                this->ui.label_cpu_std_cold_ret_value->setText(QString::number(cpu_cold_ret_std));
                double cpu_cold_ret_median = median(cpu_cold_ret);
                this->ui.label_cpu_median_cold_ret_value->setText(QString::number(cpu_cold_ret_median));
                double cpu_cold_ret_max= *std::max_element(cpu_cold_ret.begin(),cpu_cold_ret.end());
                this->ui.label_cpu_max_cold_ret_value->setText(QString::number(cpu_cold_ret_max));

                double obj_cold_ret_mean = accumulate( obj_cold_ret.begin(), obj_cold_ret.end(), 0.0)/obj_cold_ret.size();
                this->ui.label_obj_mean_cold_ret_value->setText(QString::number(obj_cold_ret_mean));
                double obj_cold_ret_sq_sum = std::inner_product(obj_cold_ret.begin(), obj_cold_ret.end(), obj_cold_ret.begin(), 0.0);
                double obj_cold_ret_std = std::sqrt(obj_cold_ret_sq_sum / obj_cold_ret.size() - obj_cold_ret_mean * obj_cold_ret_mean);
                this->ui.label_obj_std_cold_ret_value->setText(QString::number(obj_cold_ret_std));
                double obj_cold_ret_median = median(obj_cold_ret);
                this->ui.label_obj_median_cold_ret_value->setText(QString::number(obj_cold_ret_median));
                double obj_cold_ret_max= *std::max_element(obj_cold_ret.begin(),obj_cold_ret.end());
                this->ui.label_obj_max_cold_ret_value->setText(QString::number(obj_cold_ret_max));
            }else{
                this->ui.label_rate_cold_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_cold_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_cold_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_cold_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_cold_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_cold_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_cold_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_cold_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_cold_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_cold_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_cold_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_cold_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_cold_ret_value->setText(QString("nan"));
            }
            if(!success_ws_or_ret.empty() && !iter_ws_or_ret.empty()){
                // warm start with the original solution
                count_occurrence(m, success_ws_or_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_or_ret = 100*(n1/double(success_ws_or_ret.size()));
                this->ui.label_rate_ws_or_ret_value->setText(QString::number(succ_rate_ws_or_ret) + QString(" %"));

                double iter_ws_or_ret_mean = accumulate( iter_ws_or_ret.begin(), iter_ws_or_ret.end(), 0.0)/iter_ws_or_ret.size();
                this->ui.label_iter_mean_ws_or_ret_value->setText(QString::number(iter_ws_or_ret_mean));
                double iter_ws_or_ret_sq_sum = std::inner_product(iter_ws_or_ret.begin(), iter_ws_or_ret.end(), iter_ws_or_ret.begin(), 0.0);
                double iter_ws_or_ret_std = std::sqrt(iter_ws_or_ret_sq_sum / iter_ws_or_ret.size() - iter_ws_or_ret_mean * iter_ws_or_ret_mean);
                this->ui.label_iter_std_ws_or_ret_value->setText(QString::number(iter_ws_or_ret_std));
                double iter_ws_or_ret_median = median(iter_ws_or_ret);
                this->ui.label_iter_median_ws_or_ret_value->setText(QString::number(iter_ws_or_ret_median));
                double iter_ws_or_ret_max= *std::max_element(iter_ws_or_ret.begin(),iter_ws_or_ret.end());
                this->ui.label_iter_max_ws_or_ret_value->setText(QString::number(iter_ws_or_ret_max));

                double cpu_ws_or_ret_mean = accumulate( cpu_ws_or_ret.begin(), cpu_ws_or_ret.end(), 0.0)/cpu_ws_or_ret.size();
                this->ui.label_cpu_mean_ws_or_ret_value->setText(QString::number(cpu_ws_or_ret_mean));
                double cpu_ws_or_ret_sq_sum = std::inner_product(cpu_ws_or_ret.begin(), cpu_ws_or_ret.end(), cpu_ws_or_ret.begin(), 0.0);
                double cpu_ws_or_ret_std = std::sqrt(cpu_ws_or_ret_sq_sum / cpu_ws_or_ret.size() - cpu_ws_or_ret_mean * cpu_ws_or_ret_mean);
                this->ui.label_cpu_std_ws_or_ret_value->setText(QString::number(cpu_ws_or_ret_std));
                double cpu_ws_or_ret_median = median(cpu_ws_or_ret);
                this->ui.label_cpu_median_ws_or_ret_value->setText(QString::number(cpu_ws_or_ret_median));
                double cpu_ws_or_ret_max= *std::max_element(cpu_ws_or_ret.begin(),cpu_ws_or_ret.end());
                this->ui.label_cpu_max_ws_or_ret_value->setText(QString::number(cpu_ws_or_ret_max));

                double obj_ws_or_ret_mean = accumulate( obj_ws_or_ret.begin(), obj_ws_or_ret.end(), 0.0)/obj_ws_or_ret.size();
                this->ui.label_obj_mean_ws_or_ret_value->setText(QString::number(obj_ws_or_ret_mean));
                double obj_ws_or_ret_sq_sum = std::inner_product(obj_ws_or_ret.begin(), obj_ws_or_ret.end(), obj_ws_or_ret.begin(), 0.0);
                double obj_ws_or_ret_std = std::sqrt(obj_ws_or_ret_sq_sum / obj_ws_or_ret.size() - obj_ws_or_ret_mean * obj_ws_or_ret_mean);
                this->ui.label_obj_std_ws_or_ret_value->setText(QString::number(obj_ws_or_ret_std));
                double obj_ws_or_ret_median = median(obj_ws_or_ret);
                this->ui.label_obj_median_ws_or_ret_value->setText(QString::number(obj_ws_or_ret_median));
                double obj_ws_or_ret_max= *std::max_element(obj_ws_or_ret.begin(),obj_ws_or_ret.end());
                this->ui.label_obj_max_ws_or_ret_value->setText(QString::number(obj_ws_or_ret_max));
            }else{
                this->ui.label_rate_ws_or_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_or_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_or_ret_value->setText(QString("nan"));
            }
            if(!success_ws_rdm_ret.empty() && !iter_ws_rdm_ret.empty()){
                // warm start with the random solution
                count_occurrence(m, success_ws_rdm_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_rdm_ret = 100*(n1/double(success_ws_rdm_ret.size()));
                this->ui.label_rate_ws_rdm_ret_value->setText(QString::number(succ_rate_ws_rdm_ret) + QString(" %"));

                double iter_ws_rdm_ret_mean = accumulate( iter_ws_rdm_ret.begin(), iter_ws_rdm_ret.end(), 0.0)/iter_ws_rdm_ret.size();
                this->ui.label_iter_mean_ws_rdm_ret_value->setText(QString::number(iter_ws_rdm_ret_mean));
                double iter_ws_rdm_ret_sq_sum = std::inner_product(iter_ws_rdm_ret.begin(), iter_ws_rdm_ret.end(), iter_ws_rdm_ret.begin(), 0.0);
                double iter_ws_rdm_ret_std = std::sqrt(iter_ws_rdm_ret_sq_sum / iter_ws_rdm_ret.size() - iter_ws_rdm_ret_mean * iter_ws_rdm_ret_mean);
                this->ui.label_iter_std_ws_rdm_ret_value->setText(QString::number(iter_ws_rdm_ret_std));
                double iter_ws_rdm_ret_median = median(iter_ws_rdm_ret);
                this->ui.label_iter_median_ws_rdm_ret_value->setText(QString::number(iter_ws_rdm_ret_median));
                double iter_ws_rdm_ret_max= *std::max_element(iter_ws_rdm_ret.begin(),iter_ws_rdm_ret.end());
                this->ui.label_iter_max_ws_rdm_ret_value->setText(QString::number(iter_ws_rdm_ret_max));

                double cpu_ws_rdm_ret_mean = accumulate( cpu_ws_rdm_ret.begin(), cpu_ws_rdm_ret.end(), 0.0)/cpu_ws_rdm_ret.size();
                this->ui.label_cpu_mean_ws_rdm_ret_value->setText(QString::number(cpu_ws_rdm_ret_mean));
                double cpu_ws_rdm_ret_sq_sum = std::inner_product(cpu_ws_rdm_ret.begin(), cpu_ws_rdm_ret.end(), cpu_ws_rdm_ret.begin(), 0.0);
                double cpu_ws_rdm_ret_std = std::sqrt(cpu_ws_rdm_ret_sq_sum / cpu_ws_rdm_ret.size() - cpu_ws_rdm_ret_mean * cpu_ws_rdm_ret_mean);
                this->ui.label_cpu_std_ws_rdm_ret_value->setText(QString::number(cpu_ws_rdm_ret_std));
                double cpu_ws_rdm_ret_median = median(cpu_ws_rdm_ret);
                this->ui.label_cpu_median_ws_rdm_ret_value->setText(QString::number(cpu_ws_rdm_ret_median));
                double cpu_ws_rdm_ret_max= *std::max_element(cpu_ws_rdm_ret.begin(),cpu_ws_rdm_ret.end());
                this->ui.label_cpu_max_ws_rdm_ret_value->setText(QString::number(cpu_ws_rdm_ret_max));

                double obj_ws_rdm_ret_mean = accumulate( obj_ws_rdm_ret.begin(), obj_ws_rdm_ret.end(), 0.0)/obj_ws_rdm_ret.size();
                this->ui.label_obj_mean_ws_rdm_ret_value->setText(QString::number(obj_ws_rdm_ret_mean));
                double obj_ws_rdm_ret_sq_sum = std::inner_product(obj_ws_rdm_ret.begin(), obj_ws_rdm_ret.end(), obj_ws_rdm_ret.begin(), 0.0);
                double obj_ws_rdm_ret_std = std::sqrt(obj_ws_rdm_ret_sq_sum / obj_ws_rdm_ret.size() - obj_ws_rdm_ret_mean * obj_ws_rdm_ret_mean);
                this->ui.label_obj_std_ws_rdm_ret_value->setText(QString::number(obj_ws_rdm_ret_std));
                double obj_ws_rdm_ret_median = median(obj_ws_rdm_ret);
                this->ui.label_obj_median_ws_rdm_ret_value->setText(QString::number(obj_ws_rdm_ret_median));
                double obj_ws_rdm_ret_max= *std::max_element(obj_ws_rdm_ret.begin(),obj_ws_rdm_ret.end());
                this->ui.label_obj_max_ws_rdm_ret_value->setText(QString::number(obj_ws_rdm_ret_max));
            }else{
                this->ui.label_rate_ws_rdm_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_rdm_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_rdm_ret_value->setText(QString("nan"));
            }
            if(!success_ws_nn_ret.empty() && !iter_ws_nn_ret.empty()){
                // warm start with the neural network solution
                count_occurrence(m, success_ws_nn_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_nn_ret = 100*(n1/double(success_ws_nn_ret.size()));
                this->ui.label_rate_ws_nn_ret_value->setText(QString::number(succ_rate_ws_nn_ret) + QString(" %"));

                double iter_ws_nn_ret_mean = accumulate( iter_ws_nn_ret.begin(), iter_ws_nn_ret.end(), 0.0)/iter_ws_nn_ret.size();
                this->ui.label_iter_mean_ws_nn_ret_value->setText(QString::number(iter_ws_nn_ret_mean));
                double iter_ws_nn_ret_sq_sum = std::inner_product(iter_ws_nn_ret.begin(), iter_ws_nn_ret.end(), iter_ws_nn_ret.begin(), 0.0);
                double iter_ws_nn_ret_std = std::sqrt(iter_ws_nn_ret_sq_sum / iter_ws_nn_ret.size() - iter_ws_nn_ret_mean * iter_ws_nn_ret_mean);
                this->ui.label_iter_std_ws_nn_ret_value->setText(QString::number(iter_ws_nn_ret_std));
                double iter_ws_nn_ret_median = median(iter_ws_nn_ret);
                this->ui.label_iter_median_ws_nn_ret_value->setText(QString::number(iter_ws_nn_ret_median));
                double iter_ws_nn_ret_max= *std::max_element(iter_ws_nn_ret.begin(),iter_ws_nn_ret.end());
                this->ui.label_iter_max_ws_nn_ret_value->setText(QString::number(iter_ws_nn_ret_max));

                double cpu_ws_nn_ret_mean = accumulate( cpu_ws_nn_ret.begin(), cpu_ws_nn_ret.end(), 0.0)/cpu_ws_nn_ret.size();
                this->ui.label_cpu_mean_ws_nn_ret_value->setText(QString::number(cpu_ws_nn_ret_mean));
                double cpu_ws_nn_ret_sq_sum = std::inner_product(cpu_ws_nn_ret.begin(), cpu_ws_nn_ret.end(), cpu_ws_nn_ret.begin(), 0.0);
                double cpu_ws_nn_ret_std = std::sqrt(cpu_ws_nn_ret_sq_sum / cpu_ws_nn_ret.size() - cpu_ws_nn_ret_mean * cpu_ws_nn_ret_mean);
                this->ui.label_cpu_std_ws_nn_ret_value->setText(QString::number(cpu_ws_nn_ret_std));
                double cpu_ws_nn_ret_median = median(cpu_ws_nn_ret);
                this->ui.label_cpu_median_ws_nn_ret_value->setText(QString::number(cpu_ws_nn_ret_median));
                double cpu_ws_nn_ret_max= *std::max_element(cpu_ws_nn_ret.begin(),cpu_ws_nn_ret.end());
                this->ui.label_cpu_max_ws_nn_ret_value->setText(QString::number(cpu_ws_nn_ret_max));

                double obj_ws_nn_ret_mean = accumulate( obj_ws_nn_ret.begin(), obj_ws_nn_ret.end(), 0.0)/obj_ws_nn_ret.size();
                this->ui.label_obj_mean_ws_nn_ret_value->setText(QString::number(obj_ws_nn_ret_mean));
                double obj_ws_nn_ret_sq_sum = std::inner_product(obj_ws_nn_ret.begin(), obj_ws_nn_ret.end(), obj_ws_nn_ret.begin(), 0.0);
                double obj_ws_nn_ret_std = std::sqrt(obj_ws_nn_ret_sq_sum / obj_ws_nn_ret.size() - obj_ws_nn_ret_mean * obj_ws_nn_ret_mean);
                this->ui.label_obj_std_ws_nn_ret_value->setText(QString::number(obj_ws_nn_ret_std));
                double obj_ws_nn_ret_median = median(obj_ws_nn_ret);
                this->ui.label_obj_median_ws_nn_ret_value->setText(QString::number(obj_ws_nn_ret_median));
                double obj_ws_nn_ret_max= *std::max_element(obj_ws_nn_ret.begin(),obj_ws_nn_ret.end());
                this->ui.label_obj_max_ws_nn_ret_value->setText(QString::number(obj_ws_nn_ret_max));
            }else{
                this->ui.label_rate_ws_nn_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_nn_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_nn_ret_value->setText(QString("nan"));
            }
            if(!success_ws_svm_ret.empty() && !iter_ws_svm_ret.empty()){
                // warm start with the support vector machines solution
                count_occurrence(m, success_ws_svm_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_svm_ret = 100*(n1/double(success_ws_svm_ret.size()));
                this->ui.label_rate_ws_svm_ret_value->setText(QString::number(succ_rate_ws_svm_ret) + QString(" %"));

                double iter_ws_svm_ret_mean = accumulate( iter_ws_svm_ret.begin(), iter_ws_svm_ret.end(), 0.0)/iter_ws_svm_ret.size();
                this->ui.label_iter_mean_ws_svm_ret_value->setText(QString::number(iter_ws_svm_ret_mean));
                double iter_ws_svm_ret_sq_sum = std::inner_product(iter_ws_svm_ret.begin(), iter_ws_svm_ret.end(), iter_ws_svm_ret.begin(), 0.0);
                double iter_ws_svm_ret_std = std::sqrt(iter_ws_svm_ret_sq_sum / iter_ws_svm_ret.size() - iter_ws_svm_ret_mean * iter_ws_svm_ret_mean);
                this->ui.label_iter_std_ws_svm_ret_value->setText(QString::number(iter_ws_svm_ret_std));
                double iter_ws_svm_ret_median = median(iter_ws_svm_ret);
                this->ui.label_iter_median_ws_svm_ret_value->setText(QString::number(iter_ws_svm_ret_median));
                double iter_ws_svm_ret_max= *std::max_element(iter_ws_svm_ret.begin(),iter_ws_svm_ret.end());
                this->ui.label_iter_max_ws_svm_ret_value->setText(QString::number(iter_ws_svm_ret_max));

                double cpu_ws_svm_ret_mean = accumulate( cpu_ws_svm_ret.begin(), cpu_ws_svm_ret.end(), 0.0)/cpu_ws_svm_ret.size();
                this->ui.label_cpu_mean_ws_svm_ret_value->setText(QString::number(cpu_ws_svm_ret_mean));
                double cpu_ws_svm_ret_sq_sum = std::inner_product(cpu_ws_svm_ret.begin(), cpu_ws_svm_ret.end(), cpu_ws_svm_ret.begin(), 0.0);
                double cpu_ws_svm_ret_std = std::sqrt(cpu_ws_svm_ret_sq_sum / cpu_ws_svm_ret.size() - cpu_ws_svm_ret_mean * cpu_ws_svm_ret_mean);
                this->ui.label_cpu_std_ws_svm_ret_value->setText(QString::number(cpu_ws_svm_ret_std));
                double cpu_ws_svm_ret_median = median(cpu_ws_svm_ret);
                this->ui.label_cpu_median_ws_svm_ret_value->setText(QString::number(cpu_ws_svm_ret_median));
                double cpu_ws_svm_ret_max= *std::max_element(cpu_ws_svm_ret.begin(),cpu_ws_svm_ret.end());
                this->ui.label_cpu_max_ws_svm_ret_value->setText(QString::number(cpu_ws_svm_ret_max));

                double obj_ws_svm_ret_mean = accumulate( obj_ws_svm_ret.begin(), obj_ws_svm_ret.end(), 0.0)/obj_ws_svm_ret.size();
                this->ui.label_obj_mean_ws_svm_ret_value->setText(QString::number(obj_ws_svm_ret_mean));
                double obj_ws_svm_ret_sq_sum = std::inner_product(obj_ws_svm_ret.begin(), obj_ws_svm_ret.end(), obj_ws_svm_ret.begin(), 0.0);
                double obj_ws_svm_ret_std = std::sqrt(obj_ws_svm_ret_sq_sum / obj_ws_svm_ret.size() - obj_ws_svm_ret_mean * obj_ws_svm_ret_mean);
                this->ui.label_obj_std_ws_svm_ret_value->setText(QString::number(obj_ws_svm_ret_std));
                double obj_ws_svm_ret_median = median(obj_ws_svm_ret);
                this->ui.label_obj_median_ws_svm_ret_value->setText(QString::number(obj_ws_svm_ret_median));
                double obj_ws_svm_ret_max= *std::max_element(obj_ws_svm_ret.begin(),obj_ws_svm_ret.end());
                this->ui.label_obj_max_ws_svm_ret_value->setText(QString::number(obj_ws_svm_ret_max));
            }else{
                this->ui.label_rate_ws_svm_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_svm_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_svm_ret_value->setText(QString("nan"));
            }
            if(!success_ws_knn_ret.empty() && !iter_ws_knn_ret.empty()){
                // warm start with the k-nearest neighbors solution
                count_occurrence(m, success_ws_knn_ret); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_knn_ret = 100*(n1/double(success_ws_knn_ret.size()));
                this->ui.label_rate_ws_knn_ret_value->setText(QString::number(succ_rate_ws_knn_ret) + QString(" %"));

                double iter_ws_knn_ret_mean = accumulate( iter_ws_knn_ret.begin(), iter_ws_knn_ret.end(), 0.0)/iter_ws_knn_ret.size();
                this->ui.label_iter_mean_ws_knn_ret_value->setText(QString::number(iter_ws_knn_ret_mean));
                double iter_ws_knn_ret_sq_sum = std::inner_product(iter_ws_knn_ret.begin(), iter_ws_knn_ret.end(), iter_ws_knn_ret.begin(), 0.0);
                double iter_ws_knn_ret_std = std::sqrt(iter_ws_knn_ret_sq_sum / iter_ws_knn_ret.size() - iter_ws_knn_ret_mean * iter_ws_knn_ret_mean);
                this->ui.label_iter_std_ws_knn_ret_value->setText(QString::number(iter_ws_knn_ret_std));
                double iter_ws_knn_ret_median = median(iter_ws_knn_ret);
                this->ui.label_iter_median_ws_knn_ret_value->setText(QString::number(iter_ws_knn_ret_median));
                double iter_ws_knn_ret_max= *std::max_element(iter_ws_knn_ret.begin(),iter_ws_knn_ret.end());
                this->ui.label_iter_max_ws_knn_ret_value->setText(QString::number(iter_ws_knn_ret_max));

                double cpu_ws_knn_ret_mean = accumulate( cpu_ws_knn_ret.begin(), cpu_ws_knn_ret.end(), 0.0)/cpu_ws_knn_ret.size();
                this->ui.label_cpu_mean_ws_knn_ret_value->setText(QString::number(cpu_ws_knn_ret_mean));
                double cpu_ws_knn_ret_sq_sum = std::inner_product(cpu_ws_knn_ret.begin(), cpu_ws_knn_ret.end(), cpu_ws_knn_ret.begin(), 0.0);
                double cpu_ws_knn_ret_std = std::sqrt(cpu_ws_knn_ret_sq_sum / cpu_ws_knn_ret.size() - cpu_ws_knn_ret_mean * cpu_ws_knn_ret_mean);
                this->ui.label_cpu_std_ws_knn_ret_value->setText(QString::number(cpu_ws_knn_ret_std));
                double cpu_ws_knn_ret_median = median(cpu_ws_knn_ret);
                this->ui.label_cpu_median_ws_knn_ret_value->setText(QString::number(cpu_ws_knn_ret_median));
                double cpu_ws_knn_ret_max= *std::max_element(cpu_ws_knn_ret.begin(),cpu_ws_knn_ret.end());
                this->ui.label_cpu_max_ws_knn_ret_value->setText(QString::number(cpu_ws_knn_ret_max));

                double obj_ws_knn_ret_mean = accumulate( obj_ws_knn_ret.begin(), obj_ws_knn_ret.end(), 0.0)/obj_ws_knn_ret.size();
                this->ui.label_obj_mean_ws_knn_ret_value->setText(QString::number(obj_ws_knn_ret_mean));
                double obj_ws_knn_ret_sq_sum = std::inner_product(obj_ws_knn_ret.begin(), obj_ws_knn_ret.end(), obj_ws_knn_ret.begin(), 0.0);
                double obj_ws_knn_ret_std = std::sqrt(obj_ws_knn_ret_sq_sum / obj_ws_knn_ret.size() - obj_ws_knn_ret_mean * obj_ws_knn_ret_mean);
                this->ui.label_obj_std_ws_knn_ret_value->setText(QString::number(obj_ws_knn_ret_std));
                double obj_ws_knn_ret_median = median(obj_ws_knn_ret);
                this->ui.label_obj_median_ws_knn_ret_value->setText(QString::number(obj_ws_knn_ret_median));
                double obj_ws_knn_ret_max= *std::max_element(obj_ws_knn_ret.begin(),obj_ws_knn_ret.end());
                this->ui.label_obj_max_ws_knn_ret_value->setText(QString::number(obj_ws_knn_ret_max));
            }else{
                this->ui.label_rate_ws_knn_ret_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_knn_ret_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_knn_ret_value->setText(QString("nan"));
            }

            // bounce
            if(!success_cold_bounce.empty() && !iter_cold_bounce.empty()){
                // cold start
                count_occurrence(m, success_cold_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_cold_bounce = 100*(n1/double(success_cold_bounce.size()));
                this->ui.label_rate_cold_bounce_value->setText(QString::number(succ_rate_cold_bounce) + QString(" %"));

                double iter_cold_bounce_mean = accumulate( iter_cold_bounce.begin(), iter_cold_bounce.end(), 0.0)/iter_cold_bounce.size();
                this->ui.label_iter_mean_cold_bounce_value->setText(QString::number(iter_cold_bounce_mean));
                double iter_cold_bounce_sq_sum = std::inner_product(iter_cold_bounce.begin(), iter_cold_bounce.end(), iter_cold_bounce.begin(), 0.0);
                double iter_cold_bounce_std = std::sqrt(iter_cold_bounce_sq_sum / iter_cold_bounce.size() - iter_cold_bounce_mean * iter_cold_bounce_mean);
                this->ui.label_iter_std_cold_bounce_value->setText(QString::number(iter_cold_bounce_std));
                double iter_cold_bounce_median = median(iter_cold_bounce);
                this->ui.label_iter_median_cold_bounce_value->setText(QString::number(iter_cold_bounce_median));
                double iter_cold_bounce_max= *std::max_element(iter_cold_bounce.begin(),iter_cold_bounce.end());
                this->ui.label_iter_max_cold_bounce_value->setText(QString::number(iter_cold_bounce_max));

                double cpu_cold_bounce_mean = accumulate( cpu_cold_bounce.begin(), cpu_cold_bounce.end(), 0.0)/cpu_cold_bounce.size();
                this->ui.label_cpu_mean_cold_bounce_value->setText(QString::number(cpu_cold_bounce_mean));
                double cpu_cold_bounce_sq_sum = std::inner_product(cpu_cold_bounce.begin(), cpu_cold_bounce.end(), cpu_cold_bounce.begin(), 0.0);
                double cpu_cold_bounce_std = std::sqrt(cpu_cold_bounce_sq_sum / cpu_cold_bounce.size() - cpu_cold_bounce_mean * cpu_cold_bounce_mean);
                this->ui.label_cpu_std_cold_bounce_value->setText(QString::number(cpu_cold_bounce_std));
                double cpu_cold_bounce_median = median(cpu_cold_bounce);
                this->ui.label_cpu_median_cold_bounce_value->setText(QString::number(cpu_cold_bounce_median));
                double cpu_cold_bounce_max= *std::max_element(cpu_cold_bounce.begin(),cpu_cold_bounce.end());
                this->ui.label_cpu_max_cold_bounce_value->setText(QString::number(cpu_cold_bounce_max));

                double obj_cold_bounce_mean = accumulate( obj_cold_bounce.begin(), obj_cold_bounce.end(), 0.0)/obj_cold_bounce.size();
                this->ui.label_obj_mean_cold_bounce_value->setText(QString::number(obj_cold_bounce_mean));
                double obj_cold_bounce_sq_sum = std::inner_product(obj_cold_bounce.begin(), obj_cold_bounce.end(), obj_cold_bounce.begin(), 0.0);
                double obj_cold_bounce_std = std::sqrt(obj_cold_bounce_sq_sum / obj_cold_bounce.size() - obj_cold_bounce_mean * obj_cold_bounce_mean);
                this->ui.label_obj_std_cold_bounce_value->setText(QString::number(obj_cold_bounce_std));
                double obj_cold_bounce_median = median(obj_cold_bounce);
                this->ui.label_obj_median_cold_bounce_value->setText(QString::number(obj_cold_bounce_median));
                double obj_cold_bounce_max= *std::max_element(obj_cold_bounce.begin(),obj_cold_bounce.end());
                this->ui.label_obj_max_cold_bounce_value->setText(QString::number(obj_cold_bounce_max));
            }else{
                this->ui.label_rate_cold_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_cold_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_cold_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_cold_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_cold_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_cold_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_cold_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_cold_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_cold_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_cold_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_cold_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_cold_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_cold_bounce_value->setText(QString("nan"));
            }

            if(!success_ws_or_bounce.empty() && !iter_ws_or_bounce.empty()){
                // warm start with the original solution
                count_occurrence(m, success_ws_or_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_or_bounce = 100*(n1/double(success_ws_or_bounce.size()));
                this->ui.label_rate_ws_or_bounce_value->setText(QString::number(succ_rate_ws_or_bounce) + QString(" %"));

                double iter_ws_or_bounce_mean = accumulate( iter_ws_or_bounce.begin(), iter_ws_or_bounce.end(), 0.0)/iter_ws_or_bounce.size();
                this->ui.label_iter_mean_ws_or_bounce_value->setText(QString::number(iter_ws_or_bounce_mean));
                double iter_ws_or_bounce_sq_sum = std::inner_product(iter_ws_or_bounce.begin(), iter_ws_or_bounce.end(), iter_ws_or_bounce.begin(), 0.0);
                double iter_ws_or_bounce_std = std::sqrt(iter_ws_or_bounce_sq_sum / iter_ws_or_bounce.size() - iter_ws_or_bounce_mean * iter_ws_or_bounce_mean);
                this->ui.label_iter_std_ws_or_bounce_value->setText(QString::number(iter_ws_or_bounce_std));
                double iter_ws_or_bounce_median = median(iter_ws_or_bounce);
                this->ui.label_iter_median_ws_or_bounce_value->setText(QString::number(iter_ws_or_bounce_median));
                double iter_ws_or_bounce_max= *std::max_element(iter_ws_or_bounce.begin(),iter_ws_or_bounce.end());
                this->ui.label_iter_max_ws_or_bounce_value->setText(QString::number(iter_ws_or_bounce_max));

                double cpu_ws_or_bounce_mean = accumulate( cpu_ws_or_bounce.begin(), cpu_ws_or_bounce.end(), 0.0)/cpu_ws_or_bounce.size();
                this->ui.label_cpu_mean_ws_or_bounce_value->setText(QString::number(cpu_ws_or_bounce_mean));
                double cpu_ws_or_bounce_sq_sum = std::inner_product(cpu_ws_or_bounce.begin(), cpu_ws_or_bounce.end(), cpu_ws_or_bounce.begin(), 0.0);
                double cpu_ws_or_bounce_std = std::sqrt(cpu_ws_or_bounce_sq_sum / cpu_ws_or_bounce.size() - cpu_ws_or_bounce_mean * cpu_ws_or_bounce_mean);
                this->ui.label_cpu_std_ws_or_bounce_value->setText(QString::number(cpu_ws_or_bounce_std));
                double cpu_ws_or_bounce_median = median(cpu_ws_or_bounce);
                this->ui.label_cpu_median_ws_or_bounce_value->setText(QString::number(cpu_ws_or_bounce_median));
                double cpu_ws_or_bounce_max= *std::max_element(cpu_ws_or_bounce.begin(),cpu_ws_or_bounce.end());
                this->ui.label_cpu_max_ws_or_bounce_value->setText(QString::number(cpu_ws_or_bounce_max));

                double obj_ws_or_bounce_mean = accumulate( obj_ws_or_bounce.begin(), obj_ws_or_bounce.end(), 0.0)/obj_ws_or_bounce.size();
                this->ui.label_obj_mean_ws_or_bounce_value->setText(QString::number(obj_ws_or_bounce_mean));
                double obj_ws_or_bounce_sq_sum = std::inner_product(obj_ws_or_bounce.begin(), obj_ws_or_bounce.end(), obj_ws_or_bounce.begin(), 0.0);
                double obj_ws_or_bounce_std = std::sqrt(obj_ws_or_bounce_sq_sum / obj_ws_or_bounce.size() - obj_ws_or_bounce_mean * obj_ws_or_bounce_mean);
                this->ui.label_obj_std_ws_or_bounce_value->setText(QString::number(obj_ws_or_bounce_std));
                double obj_ws_or_bounce_median = median(obj_ws_or_bounce);
                this->ui.label_obj_median_ws_or_bounce_value->setText(QString::number(obj_ws_or_bounce_median));
                double obj_ws_or_bounce_max= *std::max_element(obj_ws_or_bounce.begin(),obj_ws_or_bounce.end());
                this->ui.label_obj_max_ws_or_bounce_value->setText(QString::number(obj_ws_or_bounce_max));
            }else{
                this->ui.label_rate_ws_or_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_or_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_or_bounce_value->setText(QString("nan"));
            }

            if(!success_ws_rdm_bounce.empty() && !iter_ws_rdm_bounce.empty()){
                // warm start with the random solution
                count_occurrence(m, success_ws_rdm_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_rdm_bounce = 100*(n1/double(success_ws_rdm_bounce.size()));
                this->ui.label_rate_ws_rdm_bounce_value->setText(QString::number(succ_rate_ws_rdm_bounce) + QString(" %"));

                double iter_ws_rdm_bounce_mean = accumulate( iter_ws_rdm_bounce.begin(), iter_ws_rdm_bounce.end(), 0.0)/iter_ws_rdm_bounce.size();
                this->ui.label_iter_mean_ws_rdm_bounce_value->setText(QString::number(iter_ws_rdm_bounce_mean));
                double iter_ws_rdm_bounce_sq_sum = std::inner_product(iter_ws_rdm_bounce.begin(), iter_ws_rdm_bounce.end(), iter_ws_rdm_bounce.begin(), 0.0);
                double iter_ws_rdm_bounce_std = std::sqrt(iter_ws_rdm_bounce_sq_sum / iter_ws_rdm_bounce.size() - iter_ws_rdm_bounce_mean * iter_ws_rdm_bounce_mean);
                this->ui.label_iter_std_ws_rdm_bounce_value->setText(QString::number(iter_ws_rdm_bounce_std));
                double iter_ws_rdm_bounce_median = median(iter_ws_rdm_bounce);
                this->ui.label_iter_median_ws_rdm_bounce_value->setText(QString::number(iter_ws_rdm_bounce_median));
                double iter_ws_rdm_bounce_max= *std::max_element(iter_ws_rdm_bounce.begin(),iter_ws_rdm_bounce.end());
                this->ui.label_iter_max_ws_rdm_bounce_value->setText(QString::number(iter_ws_rdm_bounce_max));

                double cpu_ws_rdm_bounce_mean = accumulate( cpu_ws_rdm_bounce.begin(), cpu_ws_rdm_bounce.end(), 0.0)/cpu_ws_rdm_bounce.size();
                this->ui.label_cpu_mean_ws_rdm_bounce_value->setText(QString::number(cpu_ws_rdm_bounce_mean));
                double cpu_ws_rdm_bounce_sq_sum = std::inner_product(cpu_ws_rdm_bounce.begin(), cpu_ws_rdm_bounce.end(), cpu_ws_rdm_bounce.begin(), 0.0);
                double cpu_ws_rdm_bounce_std = std::sqrt(cpu_ws_rdm_bounce_sq_sum / cpu_ws_rdm_bounce.size() - cpu_ws_rdm_bounce_mean * cpu_ws_rdm_bounce_mean);
                this->ui.label_cpu_std_ws_rdm_bounce_value->setText(QString::number(cpu_ws_rdm_bounce_std));
                double cpu_ws_rdm_bounce_median = median(cpu_ws_rdm_bounce);
                this->ui.label_cpu_median_ws_rdm_bounce_value->setText(QString::number(cpu_ws_rdm_bounce_median));
                double cpu_ws_rdm_bounce_max= *std::max_element(cpu_ws_rdm_bounce.begin(),cpu_ws_rdm_bounce.end());
                this->ui.label_cpu_max_ws_rdm_bounce_value->setText(QString::number(cpu_ws_rdm_bounce_max));

                double obj_ws_rdm_bounce_mean = accumulate( obj_ws_rdm_bounce.begin(), obj_ws_rdm_bounce.end(), 0.0)/obj_ws_rdm_bounce.size();
                this->ui.label_obj_mean_ws_rdm_bounce_value->setText(QString::number(obj_ws_rdm_bounce_mean));
                double obj_ws_rdm_bounce_sq_sum = std::inner_product(obj_ws_rdm_bounce.begin(), obj_ws_rdm_bounce.end(), obj_ws_rdm_bounce.begin(), 0.0);
                double obj_ws_rdm_bounce_std = std::sqrt(obj_ws_rdm_bounce_sq_sum / obj_ws_rdm_bounce.size() - obj_ws_rdm_bounce_mean * obj_ws_rdm_bounce_mean);
                this->ui.label_obj_std_ws_rdm_bounce_value->setText(QString::number(obj_ws_rdm_bounce_std));
                double obj_ws_rdm_bounce_median = median(obj_ws_rdm_bounce);
                this->ui.label_obj_median_ws_rdm_bounce_value->setText(QString::number(obj_ws_rdm_bounce_median));
                double obj_ws_rdm_bounce_max= *std::max_element(obj_ws_rdm_bounce.begin(),obj_ws_rdm_bounce.end());
                this->ui.label_obj_max_ws_rdm_bounce_value->setText(QString::number(obj_ws_rdm_bounce_max));
            }else{
                this->ui.label_rate_ws_rdm_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_rdm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_rdm_bounce_value->setText(QString("nan"));
            }

            if(!success_ws_nn_bounce.empty() && !iter_ws_nn_bounce.empty()){
                // warm start with the neural network solution
                count_occurrence(m, success_ws_nn_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_nn_bounce = 100*(n1/double(success_ws_nn_bounce.size()));
                this->ui.label_rate_ws_nn_bounce_value->setText(QString::number(succ_rate_ws_nn_bounce) + QString(" %"));

                double iter_ws_nn_bounce_mean = accumulate( iter_ws_nn_bounce.begin(), iter_ws_nn_bounce.end(), 0.0)/iter_ws_nn_bounce.size();
                this->ui.label_iter_mean_ws_nn_bounce_value->setText(QString::number(iter_ws_nn_bounce_mean));
                double iter_ws_nn_bounce_sq_sum = std::inner_product(iter_ws_nn_bounce.begin(), iter_ws_nn_bounce.end(), iter_ws_nn_bounce.begin(), 0.0);
                double iter_ws_nn_bounce_std = std::sqrt(iter_ws_nn_bounce_sq_sum / iter_ws_nn_bounce.size() - iter_ws_nn_bounce_mean * iter_ws_nn_bounce_mean);
                this->ui.label_iter_std_ws_nn_bounce_value->setText(QString::number(iter_ws_nn_bounce_std));
                double iter_ws_nn_bounce_median = median(iter_ws_nn_bounce);
                this->ui.label_iter_median_ws_nn_bounce_value->setText(QString::number(iter_ws_nn_bounce_median));
                double iter_ws_nn_bounce_max= *std::max_element(iter_ws_nn_bounce.begin(),iter_ws_nn_bounce.end());
                this->ui.label_iter_max_ws_nn_bounce_value->setText(QString::number(iter_ws_nn_bounce_max));

                double cpu_ws_nn_bounce_mean = accumulate( cpu_ws_nn_bounce.begin(), cpu_ws_nn_bounce.end(), 0.0)/cpu_ws_nn_bounce.size();
                this->ui.label_cpu_mean_ws_nn_bounce_value->setText(QString::number(cpu_ws_nn_bounce_mean));
                double cpu_ws_nn_bounce_sq_sum = std::inner_product(cpu_ws_nn_bounce.begin(), cpu_ws_nn_bounce.end(), cpu_ws_nn_bounce.begin(), 0.0);
                double cpu_ws_nn_bounce_std = std::sqrt(cpu_ws_nn_bounce_sq_sum / cpu_ws_nn_bounce.size() - cpu_ws_nn_bounce_mean * cpu_ws_nn_bounce_mean);
                this->ui.label_cpu_std_ws_nn_bounce_value->setText(QString::number(cpu_ws_nn_bounce_std));
                double cpu_ws_nn_bounce_median = median(cpu_ws_nn_bounce);
                this->ui.label_cpu_median_ws_nn_bounce_value->setText(QString::number(cpu_ws_nn_bounce_median));
                double cpu_ws_nn_bounce_max= *std::max_element(cpu_ws_nn_bounce.begin(),cpu_ws_nn_bounce.end());
                this->ui.label_cpu_max_ws_nn_bounce_value->setText(QString::number(cpu_ws_nn_bounce_max));

                double obj_ws_nn_bounce_mean = accumulate( obj_ws_nn_bounce.begin(), obj_ws_nn_bounce.end(), 0.0)/obj_ws_nn_bounce.size();
                this->ui.label_obj_mean_ws_nn_bounce_value->setText(QString::number(obj_ws_nn_bounce_mean));
                double obj_ws_nn_bounce_sq_sum = std::inner_product(obj_ws_nn_bounce.begin(), obj_ws_nn_bounce.end(), obj_ws_nn_bounce.begin(), 0.0);
                double obj_ws_nn_bounce_std = std::sqrt(obj_ws_nn_bounce_sq_sum / obj_ws_nn_bounce.size() - obj_ws_nn_bounce_mean * obj_ws_nn_bounce_mean);
                this->ui.label_obj_std_ws_nn_bounce_value->setText(QString::number(obj_ws_nn_bounce_std));
                double obj_ws_nn_bounce_median = median(obj_ws_nn_bounce);
                this->ui.label_obj_median_ws_nn_bounce_value->setText(QString::number(obj_ws_nn_bounce_median));
                double obj_ws_nn_bounce_max= *std::max_element(obj_ws_nn_bounce.begin(),obj_ws_nn_bounce.end());
                this->ui.label_obj_max_ws_nn_bounce_value->setText(QString::number(obj_ws_nn_bounce_max));
            }else{
                this->ui.label_rate_ws_nn_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_nn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_nn_bounce_value->setText(QString("nan"));
            }

            if(!success_ws_svm_bounce.empty() && !iter_ws_svm_bounce.empty()){
                // warm start with the support vector machines solution
                count_occurrence(m, success_ws_svm_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_svm_bounce = 100*(n1/double(success_ws_svm_bounce.size()));
                this->ui.label_rate_ws_svm_bounce_value->setText(QString::number(succ_rate_ws_svm_bounce) + QString(" %"));

                double iter_ws_svm_bounce_mean = accumulate( iter_ws_svm_bounce.begin(), iter_ws_svm_bounce.end(), 0.0)/iter_ws_svm_bounce.size();
                this->ui.label_iter_mean_ws_svm_bounce_value->setText(QString::number(iter_ws_svm_bounce_mean));
                double iter_ws_svm_bounce_sq_sum = std::inner_product(iter_ws_svm_bounce.begin(), iter_ws_svm_bounce.end(), iter_ws_svm_bounce.begin(), 0.0);
                double iter_ws_svm_bounce_std = std::sqrt(iter_ws_svm_bounce_sq_sum / iter_ws_svm_bounce.size() - iter_ws_svm_bounce_mean * iter_ws_svm_bounce_mean);
                this->ui.label_iter_std_ws_svm_bounce_value->setText(QString::number(iter_ws_svm_bounce_std));
                double iter_ws_svm_bounce_median = median(iter_ws_svm_bounce);
                this->ui.label_iter_median_ws_svm_bounce_value->setText(QString::number(iter_ws_svm_bounce_median));
                double iter_ws_svm_bounce_max= *std::max_element(iter_ws_svm_bounce.begin(),iter_ws_svm_bounce.end());
                this->ui.label_iter_max_ws_svm_bounce_value->setText(QString::number(iter_ws_svm_bounce_max));

                double cpu_ws_svm_bounce_mean = accumulate( cpu_ws_svm_bounce.begin(), cpu_ws_svm_bounce.end(), 0.0)/cpu_ws_svm_bounce.size();
                this->ui.label_cpu_mean_ws_svm_bounce_value->setText(QString::number(cpu_ws_svm_bounce_mean));
                double cpu_ws_svm_bounce_sq_sum = std::inner_product(cpu_ws_svm_bounce.begin(), cpu_ws_svm_bounce.end(), cpu_ws_svm_bounce.begin(), 0.0);
                double cpu_ws_svm_bounce_std = std::sqrt(cpu_ws_svm_bounce_sq_sum / cpu_ws_svm_bounce.size() - cpu_ws_svm_bounce_mean * cpu_ws_svm_bounce_mean);
                this->ui.label_cpu_std_ws_svm_bounce_value->setText(QString::number(cpu_ws_svm_bounce_std));
                double cpu_ws_svm_bounce_median = median(cpu_ws_svm_bounce);
                this->ui.label_cpu_median_ws_svm_bounce_value->setText(QString::number(cpu_ws_svm_bounce_median));
                double cpu_ws_svm_bounce_max= *std::max_element(cpu_ws_svm_bounce.begin(),cpu_ws_svm_bounce.end());
                this->ui.label_cpu_max_ws_svm_bounce_value->setText(QString::number(cpu_ws_svm_bounce_max));

                double obj_ws_svm_bounce_mean = accumulate( obj_ws_svm_bounce.begin(), obj_ws_svm_bounce.end(), 0.0)/obj_ws_svm_bounce.size();
                this->ui.label_obj_mean_ws_svm_bounce_value->setText(QString::number(obj_ws_svm_bounce_mean));
                double obj_ws_svm_bounce_sq_sum = std::inner_product(obj_ws_svm_bounce.begin(), obj_ws_svm_bounce.end(), obj_ws_svm_bounce.begin(), 0.0);
                double obj_ws_svm_bounce_std = std::sqrt(obj_ws_svm_bounce_sq_sum / obj_ws_svm_bounce.size() - obj_ws_svm_bounce_mean * obj_ws_svm_bounce_mean);
                this->ui.label_obj_std_ws_svm_bounce_value->setText(QString::number(obj_ws_svm_bounce_std));
                double obj_ws_svm_bounce_median = median(obj_ws_svm_bounce);
                this->ui.label_obj_median_ws_svm_bounce_value->setText(QString::number(obj_ws_svm_bounce_median));
                double obj_ws_svm_bounce_max= *std::max_element(obj_ws_svm_bounce.begin(),obj_ws_svm_bounce.end());
                this->ui.label_obj_max_ws_svm_bounce_value->setText(QString::number(obj_ws_svm_bounce_max));
            }else{
                this->ui.label_rate_ws_svm_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_svm_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_svm_bounce_value->setText(QString("nan"));
            }

            if(!success_ws_knn_bounce.empty() && !iter_ws_knn_bounce.empty()){
                // warm start with the k-nearest neighbors solution
                count_occurrence(m, success_ws_knn_bounce); n1 = m[1];
                //std::cout<<m[1]<<std::endl; //print the number of occurrences of 1
                double succ_rate_ws_knn_bounce = 100*(n1/double(success_ws_knn_bounce.size()));
                this->ui.label_rate_ws_knn_bounce_value->setText(QString::number(succ_rate_ws_knn_bounce) + QString(" %"));

                double iter_ws_knn_bounce_mean = accumulate( iter_ws_knn_bounce.begin(), iter_ws_knn_bounce.end(), 0.0)/iter_ws_knn_bounce.size();
                this->ui.label_iter_mean_ws_knn_bounce_value->setText(QString::number(iter_ws_knn_bounce_mean));
                double iter_ws_knn_bounce_sq_sum = std::inner_product(iter_ws_knn_bounce.begin(), iter_ws_knn_bounce.end(), iter_ws_knn_bounce.begin(), 0.0);
                double iter_ws_knn_bounce_std = std::sqrt(iter_ws_knn_bounce_sq_sum / iter_ws_knn_bounce.size() - iter_ws_knn_bounce_mean * iter_ws_knn_bounce_mean);
                this->ui.label_iter_std_ws_knn_bounce_value->setText(QString::number(iter_ws_knn_bounce_std));
                double iter_ws_knn_bounce_median = median(iter_ws_knn_bounce);
                this->ui.label_iter_median_ws_knn_bounce_value->setText(QString::number(iter_ws_knn_bounce_median));
                double iter_ws_knn_bounce_max= *std::max_element(iter_ws_knn_bounce.begin(),iter_ws_knn_bounce.end());
                this->ui.label_iter_max_ws_knn_bounce_value->setText(QString::number(iter_ws_knn_bounce_max));

                double cpu_ws_knn_bounce_mean = accumulate( cpu_ws_knn_bounce.begin(), cpu_ws_knn_bounce.end(), 0.0)/cpu_ws_knn_bounce.size();
                this->ui.label_cpu_mean_ws_knn_bounce_value->setText(QString::number(cpu_ws_knn_bounce_mean));
                double cpu_ws_knn_bounce_sq_sum = std::inner_product(cpu_ws_knn_bounce.begin(), cpu_ws_knn_bounce.end(), cpu_ws_knn_bounce.begin(), 0.0);
                double cpu_ws_knn_bounce_std = std::sqrt(cpu_ws_knn_bounce_sq_sum / cpu_ws_knn_bounce.size() - cpu_ws_knn_bounce_mean * cpu_ws_knn_bounce_mean);
                this->ui.label_cpu_std_ws_knn_bounce_value->setText(QString::number(cpu_ws_knn_bounce_std));
                double cpu_ws_knn_bounce_median = median(cpu_ws_knn_bounce);
                this->ui.label_cpu_median_ws_knn_bounce_value->setText(QString::number(cpu_ws_knn_bounce_median));
                double cpu_ws_knn_bounce_max= *std::max_element(cpu_ws_knn_bounce.begin(),cpu_ws_knn_bounce.end());
                this->ui.label_cpu_max_ws_knn_bounce_value->setText(QString::number(cpu_ws_knn_bounce_max));

                double obj_ws_knn_bounce_mean = accumulate( obj_ws_knn_bounce.begin(), obj_ws_knn_bounce.end(), 0.0)/obj_ws_knn_bounce.size();
                this->ui.label_obj_mean_ws_knn_bounce_value->setText(QString::number(obj_ws_knn_bounce_mean));
                double obj_ws_knn_bounce_sq_sum = std::inner_product(obj_ws_knn_bounce.begin(), obj_ws_knn_bounce.end(), obj_ws_knn_bounce.begin(), 0.0);
                double obj_ws_knn_bounce_std = std::sqrt(obj_ws_knn_bounce_sq_sum / obj_ws_knn_bounce.size() - obj_ws_knn_bounce_mean * obj_ws_knn_bounce_mean);
                this->ui.label_obj_std_ws_knn_bounce_value->setText(QString::number(obj_ws_knn_bounce_std));
                double obj_ws_knn_bounce_median = median(obj_ws_knn_bounce);
                this->ui.label_obj_median_ws_knn_bounce_value->setText(QString::number(obj_ws_knn_bounce_median));
                double obj_ws_knn_bounce_max= *std::max_element(obj_ws_knn_bounce.begin(),obj_ws_knn_bounce.end());
                this->ui.label_obj_max_ws_knn_bounce_value->setText(QString::number(obj_ws_knn_bounce_max));
            }else{
                this->ui.label_rate_ws_knn_bounce_value->setText(QString("0 %"));
                this->ui.label_iter_mean_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_std_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_median_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_iter_max_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_mean_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_std_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_median_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_cpu_max_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_mean_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_std_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_median_ws_knn_bounce_value->setText(QString("nan"));
                this->ui.label_obj_max_ws_knn_bounce_value->setText(QString("nan"));
            }

        }catch (const std::string message){qnode.log(QNode::Error,std::string("Plan failure: ")+message);
        }catch(const std::exception exc){qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());}
    }
    qnode.log(QNode::Info,std::string("Prediction and planning ended"));
}

void MainWindow::on_pushButton_save_learning_res_clicked()
{
    string pred_dir = this->ui.lineEdit_predictions->text().toStdString();
    // txt
    string filename("results_learn_pred.txt");
    ofstream results;
    results.open(pred_dir+string("/")+filename);

    results << string("# STATISTICS OF THE PREDICTIONS  \n");
    // plan
    if(this->ui.tabWidget_learning_res->isTabEnabled(0))
    {
        results << string("# Plan target posture selection results  \n");
        results << string(" # Cold start\n");
        results << string("success_rate_cold_plan =")+this->ui.label_rate_cold_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_plan_mean =")+this->ui.label_iter_mean_cold_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_plan_std =")+this->ui.label_iter_std_cold_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_plan_median =")+this->ui.label_iter_median_cold_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_plan_max =")+this->ui.label_iter_max_cold_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_plan_mean =")+this->ui.label_cpu_mean_cold_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_plan_std =")+this->ui.label_cpu_std_cold_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_plan_median =")+this->ui.label_cpu_median_cold_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_plan_max =")+this->ui.label_cpu_max_cold_plan_value->text().toStdString()+string(";\n");
        results << string("obj_cold_plan_mean =")+this->ui.label_obj_mean_cold_plan_value->text().toStdString()+string(";\n");
        results << string("obj_cold_plan_std =")+this->ui.label_obj_std_cold_plan_value->text().toStdString()+string(";\n");
        results << string("obj_cold_plan_median =")+this->ui.label_obj_median_cold_plan_value->text().toStdString()+string(";\n");
        results << string("obj_cold_plan_max =")+this->ui.label_obj_max_cold_plan_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the original solution \n");
        results << string("success_rate_ws_or_plan =")+this->ui.label_rate_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_plan_mean =")+this->ui.label_iter_mean_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_plan_std =")+this->ui.label_iter_std_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_plan_median =")+this->ui.label_iter_median_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_plan_max =")+this->ui.label_iter_max_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_plan_mean =")+this->ui.label_cpu_mean_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_plan_std =")+this->ui.label_cpu_std_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_plan_median =")+this->ui.label_cpu_median_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_plan_max =")+this->ui.label_cpu_max_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_plan_mean =")+this->ui.label_obj_mean_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_plan_std =")+this->ui.label_obj_std_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_plan_median =")+this->ui.label_obj_median_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_plan_max =")+this->ui.label_obj_max_ws_or_plan_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the random solution \n");
        results << string("success_rate_ws_rdm_plan =")+this->ui.label_rate_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_plan_mean =")+this->ui.label_iter_mean_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_plan_std =")+this->ui.label_iter_std_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_plan_median =")+this->ui.label_iter_median_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_plan_max =")+this->ui.label_iter_max_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_plan_mean =")+this->ui.label_cpu_mean_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_plan_std =")+this->ui.label_cpu_std_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_plan_median =")+this->ui.label_cpu_median_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_plan_max =")+this->ui.label_cpu_max_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_plan_mean =")+this->ui.label_obj_mean_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_plan_std =")+this->ui.label_obj_std_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_plan_median =")+this->ui.label_obj_median_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_plan_max =")+this->ui.label_obj_max_ws_rdm_plan_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Neural Network \n");
        results << string("success_rate_ws_nn_plan =")+this->ui.label_rate_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_plan_mean =")+this->ui.label_iter_mean_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_plan_std =")+this->ui.label_iter_std_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_plan_median =")+this->ui.label_iter_median_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_plan_max =")+this->ui.label_iter_max_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_plan_mean =")+this->ui.label_cpu_mean_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_plan_std =")+this->ui.label_cpu_std_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_plan_median =")+this->ui.label_cpu_median_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_plan_max =")+this->ui.label_cpu_max_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_plan_mean =")+this->ui.label_obj_mean_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_plan_std =")+this->ui.label_obj_std_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_plan_median =")+this->ui.label_obj_median_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_plan_max =")+this->ui.label_obj_max_ws_nn_plan_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Support Vector Machines \n");
        results << string("success_rate_ws_svm_plan =")+this->ui.label_rate_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_plan_mean =")+this->ui.label_iter_mean_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_plan_std =")+this->ui.label_iter_std_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_plan_median =")+this->ui.label_iter_median_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_plan_max =")+this->ui.label_iter_max_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_plan_mean =")+this->ui.label_cpu_mean_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_plan_std =")+this->ui.label_cpu_std_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_plan_median =")+this->ui.label_cpu_median_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_plan_max =")+this->ui.label_cpu_max_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_plan_mean =")+this->ui.label_obj_mean_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_plan_std =")+this->ui.label_obj_std_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_plan_median =")+this->ui.label_obj_median_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_plan_max =")+this->ui.label_obj_max_ws_svm_plan_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with K-Nearest Neighbors \n");
        results << string("success_rate_ws_knn_plan =")+this->ui.label_rate_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_plan_mean =")+this->ui.label_iter_mean_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_plan_std =")+this->ui.label_iter_std_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_plan_median =")+this->ui.label_iter_median_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_plan_max =")+this->ui.label_iter_max_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_plan_mean =")+this->ui.label_cpu_mean_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_plan_std =")+this->ui.label_cpu_std_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_plan_median =")+this->ui.label_cpu_median_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_plan_max =")+this->ui.label_cpu_max_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_plan_mean =")+this->ui.label_obj_mean_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_plan_std =")+this->ui.label_obj_std_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_plan_median =")+this->ui.label_obj_median_ws_knn_plan_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_plan_max =")+this->ui.label_obj_max_ws_knn_plan_value->text().toStdString()+string(";\n");

    }
    // approach
    if(this->ui.tabWidget_learning_res->isTabEnabled(1))
    {
        results << string("# Approach target posture selection results  \n");
        results << string(" # Cold start\n");
        results << string("success_rate_cold_app =")+this->ui.label_rate_cold_app_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_app_mean =")+this->ui.label_iter_mean_cold_app_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_app_std =")+this->ui.label_iter_std_cold_app_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_app_median =")+this->ui.label_iter_median_cold_app_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_app_max =")+this->ui.label_iter_max_cold_app_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_app_mean =")+this->ui.label_cpu_mean_cold_app_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_app_std =")+this->ui.label_cpu_std_cold_app_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_app_median =")+this->ui.label_cpu_median_cold_app_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_app_max =")+this->ui.label_cpu_max_cold_app_value->text().toStdString()+string(";\n");
        results << string("obj_cold_app_mean =")+this->ui.label_obj_mean_cold_app_value->text().toStdString()+string(";\n");
        results << string("obj_cold_app_std =")+this->ui.label_obj_std_cold_app_value->text().toStdString()+string(";\n");
        results << string("obj_cold_app_median =")+this->ui.label_obj_median_cold_app_value->text().toStdString()+string(";\n");
        results << string("obj_cold_app_max =")+this->ui.label_obj_max_cold_app_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the original solution \n");
        results << string("success_rate_ws_or_app =")+this->ui.label_rate_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_app_mean =")+this->ui.label_iter_mean_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_app_std =")+this->ui.label_iter_std_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_app_median =")+this->ui.label_iter_median_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_app_max =")+this->ui.label_iter_max_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_app_mean =")+this->ui.label_cpu_mean_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_app_std =")+this->ui.label_cpu_std_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_app_median =")+this->ui.label_cpu_median_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_app_max =")+this->ui.label_cpu_max_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_app_mean =")+this->ui.label_obj_mean_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_app_std =")+this->ui.label_obj_std_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_app_median =")+this->ui.label_obj_median_ws_or_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_app_max =")+this->ui.label_obj_max_ws_or_app_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the random solution \n");
        results << string("success_rate_ws_rdm_app =")+this->ui.label_rate_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_app_mean =")+this->ui.label_iter_mean_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_app_std =")+this->ui.label_iter_std_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_app_median =")+this->ui.label_iter_median_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_app_max =")+this->ui.label_iter_max_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_app_mean =")+this->ui.label_cpu_mean_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_app_std =")+this->ui.label_cpu_std_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_app_median =")+this->ui.label_cpu_median_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_app_max =")+this->ui.label_cpu_max_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_app_mean =")+this->ui.label_obj_mean_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_app_std =")+this->ui.label_obj_std_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_app_median =")+this->ui.label_obj_median_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_app_max =")+this->ui.label_obj_max_ws_rdm_app_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Neural Network \n");
        results << string("success_rate_ws_nn_app =")+this->ui.label_rate_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_app_mean =")+this->ui.label_iter_mean_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_app_std =")+this->ui.label_iter_std_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_app_median =")+this->ui.label_iter_median_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_app_max =")+this->ui.label_iter_max_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_app_mean =")+this->ui.label_cpu_mean_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_app_std =")+this->ui.label_cpu_std_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_app_median =")+this->ui.label_cpu_median_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_app_max =")+this->ui.label_cpu_max_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_app_mean =")+this->ui.label_obj_mean_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_app_std =")+this->ui.label_obj_std_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_app_median =")+this->ui.label_obj_median_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_app_max =")+this->ui.label_obj_max_ws_nn_app_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Support Vector Machines \n");
        results << string("success_rate_ws_svm_app =")+this->ui.label_rate_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_app_mean =")+this->ui.label_iter_mean_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_app_std =")+this->ui.label_iter_std_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_app_median =")+this->ui.label_iter_median_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_app_max =")+this->ui.label_iter_max_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_app_mean =")+this->ui.label_cpu_mean_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_app_std =")+this->ui.label_cpu_std_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_app_median =")+this->ui.label_cpu_median_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_app_max =")+this->ui.label_cpu_max_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_app_mean =")+this->ui.label_obj_mean_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_app_std =")+this->ui.label_obj_std_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_app_median =")+this->ui.label_obj_median_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_app_max =")+this->ui.label_obj_max_ws_svm_app_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with K-Nearest Neighbors \n");
        results << string("success_rate_ws_knn_app =")+this->ui.label_rate_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_app_mean =")+this->ui.label_iter_mean_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_app_std =")+this->ui.label_iter_std_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_app_median =")+this->ui.label_iter_median_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_app_max =")+this->ui.label_iter_max_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_app_mean =")+this->ui.label_cpu_mean_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_app_std =")+this->ui.label_cpu_std_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_app_median =")+this->ui.label_cpu_median_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_app_max =")+this->ui.label_cpu_max_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_app_mean =")+this->ui.label_obj_mean_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_app_std =")+this->ui.label_obj_std_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_app_median =")+this->ui.label_obj_median_ws_knn_app_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_app_max =")+this->ui.label_obj_max_ws_knn_app_value->text().toStdString()+string(";\n");

    }
    // retreat
    if(this->ui.tabWidget_learning_res->isTabEnabled(2))
    {
        results << string("# Retreat target posture selection results  \n");
        results << string(" # Cold start\n");
        results << string("success_rate_cold_ret =")+this->ui.label_rate_cold_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_ret_mean =")+this->ui.label_iter_mean_cold_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_ret_std =")+this->ui.label_iter_std_cold_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_ret_median =")+this->ui.label_iter_median_cold_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_ret_max =")+this->ui.label_iter_max_cold_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_ret_mean =")+this->ui.label_cpu_mean_cold_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_ret_std =")+this->ui.label_cpu_std_cold_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_ret_median =")+this->ui.label_cpu_median_cold_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_ret_max =")+this->ui.label_cpu_max_cold_ret_value->text().toStdString()+string(";\n");
        results << string("obj_cold_ret_mean =")+this->ui.label_obj_mean_cold_ret_value->text().toStdString()+string(";\n");
        results << string("obj_cold_ret_std =")+this->ui.label_obj_std_cold_ret_value->text().toStdString()+string(";\n");
        results << string("obj_cold_ret_median =")+this->ui.label_obj_median_cold_ret_value->text().toStdString()+string(";\n");
        results << string("obj_cold_ret_max =")+this->ui.label_obj_max_cold_ret_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the original solution \n");
        results << string("success_rate_ws_or_ret =")+this->ui.label_rate_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_ret_mean =")+this->ui.label_iter_mean_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_ret_std =")+this->ui.label_iter_std_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_ret_median =")+this->ui.label_iter_median_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_ret_max =")+this->ui.label_iter_max_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_ret_mean =")+this->ui.label_cpu_mean_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_ret_std =")+this->ui.label_cpu_std_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_ret_median =")+this->ui.label_cpu_median_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_ret_max =")+this->ui.label_cpu_max_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_ret_mean =")+this->ui.label_obj_mean_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_ret_std =")+this->ui.label_obj_std_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_ret_median =")+this->ui.label_obj_median_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_ret_max =")+this->ui.label_obj_max_ws_or_ret_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the random solution \n");
        results << string("success_rate_ws_rdm_ret =")+this->ui.label_rate_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_ret_mean =")+this->ui.label_iter_mean_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_ret_std =")+this->ui.label_iter_std_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_ret_median =")+this->ui.label_iter_median_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_ret_max =")+this->ui.label_iter_max_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_ret_mean =")+this->ui.label_cpu_mean_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_ret_std =")+this->ui.label_cpu_std_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_ret_median =")+this->ui.label_cpu_median_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_ret_max =")+this->ui.label_cpu_max_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_ret_mean =")+this->ui.label_obj_mean_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_ret_std =")+this->ui.label_obj_std_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_ret_median =")+this->ui.label_obj_median_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_ret_max =")+this->ui.label_obj_max_ws_rdm_ret_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Neural Network \n");
        results << string("success_rate_ws_nn_ret =")+this->ui.label_rate_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_ret_mean =")+this->ui.label_iter_mean_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_ret_std =")+this->ui.label_iter_std_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_ret_median =")+this->ui.label_iter_median_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_ret_max =")+this->ui.label_iter_max_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_ret_mean =")+this->ui.label_cpu_mean_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_ret_std =")+this->ui.label_cpu_std_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_ret_median =")+this->ui.label_cpu_median_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_ret_max =")+this->ui.label_cpu_max_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_ret_mean =")+this->ui.label_obj_mean_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_ret_std =")+this->ui.label_obj_std_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_ret_median =")+this->ui.label_obj_median_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_ret_max =")+this->ui.label_obj_max_ws_nn_ret_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Support Vector Machines \n");
        results << string("success_rate_ws_svm_ret =")+this->ui.label_rate_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_ret_mean =")+this->ui.label_iter_mean_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_ret_std =")+this->ui.label_iter_std_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_ret_median =")+this->ui.label_iter_median_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_ret_max =")+this->ui.label_iter_max_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_ret_mean =")+this->ui.label_cpu_mean_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_ret_std =")+this->ui.label_cpu_std_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_ret_median =")+this->ui.label_cpu_median_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_ret_max =")+this->ui.label_cpu_max_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_ret_mean =")+this->ui.label_obj_mean_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_ret_std =")+this->ui.label_obj_std_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_ret_median =")+this->ui.label_obj_median_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_ret_max =")+this->ui.label_obj_max_ws_svm_ret_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with K-Nearest Neighbors \n");
        results << string("success_rate_ws_knn_ret =")+this->ui.label_rate_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_ret_mean =")+this->ui.label_iter_mean_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_ret_std =")+this->ui.label_iter_std_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_ret_median =")+this->ui.label_iter_median_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_ret_max =")+this->ui.label_iter_max_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_ret_mean =")+this->ui.label_cpu_mean_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_ret_std =")+this->ui.label_cpu_std_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_ret_median =")+this->ui.label_cpu_median_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_ret_max =")+this->ui.label_cpu_max_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_ret_mean =")+this->ui.label_obj_mean_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_ret_std =")+this->ui.label_obj_std_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_ret_median =")+this->ui.label_obj_median_ws_knn_ret_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_ret_max =")+this->ui.label_obj_max_ws_knn_ret_value->text().toStdString()+string(";\n");

    }
    // bounce
    if(this->ui.tabWidget_learning_res->isTabEnabled(3))
    {
        results << string("# Bounce target posture selection results  \n");
        results << string(" # Cold start\n");
        results << string("success_rate_cold_bounce =")+this->ui.label_rate_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_bounce_mean =")+this->ui.label_iter_mean_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_bounce_std =")+this->ui.label_iter_std_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_bounce_median =")+this->ui.label_iter_median_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_cold_bounce_max =")+this->ui.label_iter_max_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_bounce_mean =")+this->ui.label_cpu_mean_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_bounce_std =")+this->ui.label_cpu_std_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_bounce_median =")+this->ui.label_cpu_median_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_cold_bounce_max =")+this->ui.label_cpu_max_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_cold_bounce_mean =")+this->ui.label_obj_mean_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_cold_bounce_std =")+this->ui.label_obj_std_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_cold_bounce_median =")+this->ui.label_obj_median_cold_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_cold_bounce_max =")+this->ui.label_obj_max_cold_bounce_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the original solution \n");
        results << string("success_rate_ws_or_bounce =")+this->ui.label_rate_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_bounce_mean =")+this->ui.label_iter_mean_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_bounce_std =")+this->ui.label_iter_std_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_bounce_median =")+this->ui.label_iter_median_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_or_bounce_max =")+this->ui.label_iter_max_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_bounce_mean =")+this->ui.label_cpu_mean_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_bounce_std =")+this->ui.label_cpu_std_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_bounce_median =")+this->ui.label_cpu_median_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_or_bounce_max =")+this->ui.label_cpu_max_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_bounce_mean =")+this->ui.label_obj_mean_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_bounce_std =")+this->ui.label_obj_std_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_bounce_median =")+this->ui.label_obj_median_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_or_bounce_max =")+this->ui.label_obj_max_ws_or_bounce_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with the random solution \n");
        results << string("success_rate_ws_rdm_bounce =")+this->ui.label_rate_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_bounce_mean =")+this->ui.label_iter_mean_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_bounce_std =")+this->ui.label_iter_std_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_bounce_median =")+this->ui.label_iter_median_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_rdm_bounce_max =")+this->ui.label_iter_max_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_bounce_mean =")+this->ui.label_cpu_mean_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_bounce_std =")+this->ui.label_cpu_std_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_bounce_median =")+this->ui.label_cpu_median_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_rdm_bounce_max =")+this->ui.label_cpu_max_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_bounce_mean =")+this->ui.label_obj_mean_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_bounce_std =")+this->ui.label_obj_std_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_bounce_median =")+this->ui.label_obj_median_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_rdm_bounce_max =")+this->ui.label_obj_max_ws_rdm_bounce_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Neural Network \n");
        results << string("success_rate_ws_nn_bounce =")+this->ui.label_rate_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_bounce_mean =")+this->ui.label_iter_mean_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_bounce_std =")+this->ui.label_iter_std_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_bounce_median =")+this->ui.label_iter_median_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_nn_bounce_max =")+this->ui.label_iter_max_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_bounce_mean =")+this->ui.label_cpu_mean_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_bounce_std =")+this->ui.label_cpu_std_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_bounce_median =")+this->ui.label_cpu_median_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_nn_bounce_max =")+this->ui.label_cpu_max_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_bounce_mean =")+this->ui.label_obj_mean_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_bounce_std =")+this->ui.label_obj_std_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_bounce_median =")+this->ui.label_obj_median_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_nn_bounce_max =")+this->ui.label_obj_max_ws_nn_bounce_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with Support Vector Machines \n");
        results << string("success_rate_ws_svm_bounce =")+this->ui.label_rate_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_bounce_mean =")+this->ui.label_iter_mean_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_bounce_std =")+this->ui.label_iter_std_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_bounce_median =")+this->ui.label_iter_median_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_svm_bounce_max =")+this->ui.label_iter_max_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_bounce_mean =")+this->ui.label_cpu_mean_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_bounce_std =")+this->ui.label_cpu_std_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_bounce_median =")+this->ui.label_cpu_median_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_svm_bounce_max =")+this->ui.label_cpu_max_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_bounce_mean =")+this->ui.label_obj_mean_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_bounce_std =")+this->ui.label_obj_std_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_bounce_median =")+this->ui.label_obj_median_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_svm_bounce_max =")+this->ui.label_obj_max_ws_svm_bounce_value->text().toStdString()+string(";\n");
        results << string(" # Warm start with K-Nearest Neighbors \n");
        results << string("success_rate_ws_knn_bounce =")+this->ui.label_rate_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_bounce_mean =")+this->ui.label_iter_mean_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_bounce_std =")+this->ui.label_iter_std_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_bounce_median =")+this->ui.label_iter_median_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("iterations_ws_knn_bounce_max =")+this->ui.label_iter_max_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_bounce_mean =")+this->ui.label_cpu_mean_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_bounce_std =")+this->ui.label_cpu_std_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_bounce_median =")+this->ui.label_cpu_median_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("cpu_ws_knn_bounce_max =")+this->ui.label_cpu_max_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_bounce_mean =")+this->ui.label_obj_mean_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_bounce_std =")+this->ui.label_obj_std_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_bounce_median =")+this->ui.label_obj_median_ws_knn_bounce_value->text().toStdString()+string(";\n");
        results << string("obj_ws_knn_bounce_max =")+this->ui.label_obj_max_ws_knn_bounce_value->text().toStdString()+string(";\n");
    }
    results.close();
    qnode.log(QNode::Info,std::string("Results of learning saved at ")+pred_dir+std::string("/")+filename);

}


void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "motion_manager");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();

    mrosCommdlg->setMasterUrl(master_url);
    mrosCommdlg->setHostUrl(host_url);

    bool remember = settings.value("remember_settings", false).toBool();
    mrosCommdlg->setRememberCheckbox(remember);

    bool checked = settings.value("use_environment_variables", false).toBool();
    mrosCommdlg->setUseEnvCheckbox(checked);

    mrosCommdlg->enableMasterUrl(!checked);
    mrosCommdlg->enableHostUrl(!checked);


}


void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "motion_manager");
    settings.setValue("master_url", mrosCommdlg->getMasterUrl());
    settings.setValue("host_url", mrosCommdlg->getHostUrl());
    settings.setValue("use_environment_variables",QVariant(mrosCommdlg->getUseEnvCheckbox()));
    settings.setValue("remember_settings",QVariant(mrosCommdlg->getRememberCheckbox()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}


void MainWindow::getDerivative(vector<vector<double>> &function, vector<double> &step_values, vector<vector<double>> &derFunction)
{
    vector<double> func(function.size());
    vector<double> der_func;
    derFunction.resize(function.size());


    for(size_t j=0;j<function.at(0).size();++j)
    {
        for(size_t i=0;i<function.size();++i)
        {
            func.at(i) = function[i][j];
        }
        this->getDerivative(func,step_values,der_func);
        for(size_t i=0;i<der_func.size();++i)
        {
            derFunction.at(i).resize(function.at(0).size());
            derFunction[i][j] = der_func.at(i);
        }
        der_func.clear();
    }
}

void MainWindow::getDerivative(vector<double> &function, vector<double> &step_values, vector<double> &derFunction)
{
    const double MIN_STEP_VALUE = 0.1;

    // Formula of the numerical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


       int h = 1;
       int tnsample;
       double f0;
       double f1;
       double f2;
       double f3;
       double f4;
       double step_value;

       // 1st point
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       tnsample = 0;
       f0 = function.at(tnsample);
       f1 = function.at(tnsample+1);
       f2 = function.at(tnsample+2);
       f3 = function.at(tnsample+3);
       f4 = function.at(tnsample+4);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=MIN_STEP_VALUE;
       derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = function.at(tnsample-1);
       f1 = function.at(tnsample);
       f2 = function.at(tnsample+1);
       f3 = function.at(tnsample+2);
       f4 = function.at(tnsample+3);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=MIN_STEP_VALUE;
       derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< function.size() -2;++i){     // centered
           f0 = function.at(i-2);
           f1 = function.at(i-1);
           f2 = function.at(i);
           f3 = function.at(i+1);
           f4 = function.at(i+2);
           step_value = step_values.at(i);
           if(step_value==0)
               step_value=step_values.at(i-1);
           derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
       }

       // 4th point
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       tnsample = function.size()-2;
       f0 = function.at(tnsample-3);
       f1 = function.at(tnsample-2);
       f2 = function.at(tnsample-1);
       f3 = function.at(tnsample);
       f4 = function.at(tnsample+1);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=MIN_STEP_VALUE;
       derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));

       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = function.size()-1;
       f0 = function.at(tnsample-4);
       f1 = function.at(tnsample-3);
       f2 = function.at(tnsample-2);
       f3 = function.at(tnsample-1);
       f4 = function.at(tnsample);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=MIN_STEP_VALUE;
       derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));
}

void MainWindow::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
{
       const double MIN_STEP_VALUE = 0.1;

       // Formula of the numarical differentiation with 5 points
          // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
          // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
          // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
          // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
          // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


          int h = 1;
          int tnsample;
          double f0;
          double f1;
          double f2;
          double f3;
          double f4;
          double step_value;

          // 1st point
          // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
          tnsample = 0;
          f0 = function.at(tnsample);
          f1 = function.at(tnsample+1);
          f2 = function.at(tnsample+2);
          f3 = function.at(tnsample+3);
          f4 = function.at(tnsample+4);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));

          // 2nd point
          // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
          tnsample = 1;
          f0 = function.at(tnsample-1);
          f1 = function.at(tnsample);
          f2 = function.at(tnsample+1);
          f3 = function.at(tnsample+2);
          f4 = function.at(tnsample+3);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));

          // 3rd point
          // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
          for (int i=2; i< function.size() -2;++i){     // centered
              f0 = function.at(i-2);
              f1 = function.at(i-1);
              f2 = function.at(i);
              f3 = function.at(i+1);
              f4 = function.at(i+2);
              step_value = step_values.at(i);
              if(step_value==0)
                  step_value=step_values.at(i-1);
              derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
          }

          // 4th point
          // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
          tnsample = function.size()-2;
          f0 = function.at(tnsample-3);
          f1 = function.at(tnsample-2);
          f2 = function.at(tnsample-1);
          f3 = function.at(tnsample);
          f4 = function.at(tnsample+1);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));

          // 5th point
          // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
          tnsample = function.size()-1;
          f0 = function.at(tnsample-4);
          f1 = function.at(tnsample-3);
          f2 = function.at(tnsample-2);
          f3 = function.at(tnsample-1);
          f4 = function.at(tnsample);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));



}

int MainWindow::getNumberMovementUnits(vector<double> &function, QVector<double> &time)
{
    int nmu = 0;
    vector<float> ffunc(function.begin(),function.end());
    double maxfunc = *std::max_element(ffunc.begin(), ffunc.end());
    double perc = ((double)10)/100;
    double threshold = maxfunc*perc;

    Persistence1D p;
    p.RunPersistence(ffunc);

    //Get all extrema with a persistence larger than perst.
    double perst = 5;
    vector< TPairedExtrema > Extrema;
    p.GetPairedExtrema(Extrema, perst);

    //Print all found pairs - pairs are sorted ascending wrt. persistence.
    for(vector< TPairedExtrema >::iterator it = Extrema.begin(); it != Extrema.end(); it++)
    {

        /*
        cout << "Persistence: " << (*it).Persistence
             << " minimum: " << ffunc.at((*it).MinIndex) << " minimum index: " << (*it).MinIndex
             << " maximum: " << ffunc.at((*it).MaxIndex) << " maximum index: " << (*it).MaxIndex
             << std::endl;
        */


        double slope = (ffunc.at((*it).MaxIndex)-ffunc.at((*it).MinIndex))/(time.at((*it).MaxIndex)-time.at((*it).MinIndex));
        if(abs(slope)>threshold)
            nmu++;
    }


    return nmu;

}

double MainWindow::getMedian(vector<double> v)
{
    double median;
    size_t size = v.size();

    sort(v.begin(), v.end());

    if (size  % 2 == 0)
    {
        median = (v[size / 2 - 1] + v[size / 2]) / 2;
    }
    else
    {
        median = v[size / 2];
    }

    return median;
}

double MainWindow::getFirstQuartile(vector<double> v)
{
    double first_quartile;
    size_t size = v.size()/4;

    sort(v.begin(), v.end());

    if (size  % 2 == 0)
    {
        first_quartile = (v[size / 2 - 1] + v[size / 2]) / 2;
    }
    else
    {
        first_quartile = v[size / 2];
    }

    return first_quartile;
}

double MainWindow::getThirdQuartile(vector<double> v)
{
    double third_quartile;
    size_t size = 3*(v.size()/4);
    size_t size_1 = v.size()/4;

    sort(v.begin(), v.end());

    if (size_1  % 2 == 0)
    {
        third_quartile = (v[size + size_1/2 - 1] + v[size + size_1/2]) / 2;
    }
    else
    {
        third_quartile = v[size + size_1/2];
    }

    return third_quartile;
}

double MainWindow::getMedian(vector<int> v)
{
    double median;
    size_t size = v.size();

    sort(v.begin(), v.end());

    if (size  % 2 == 0)
    {
        median = (v[size / 2 - 1] + v[size / 2]) / 2;
    }
    else
    {
        median = v[size / 2];
    }

    return median;
}

double MainWindow::getFirstQuartile(vector<int> v)
{
    double first_quartile;
    size_t size = v.size()/4;

    sort(v.begin(), v.end());

    if (size  % 2 == 0)
    {
        first_quartile = (v[size / 2 - 1] + v[size / 2]) / 2;
    }
    else
    {
        first_quartile = v[size / 2];
    }

    return first_quartile;
}

double MainWindow::getThirdQuartile(vector<int> v)
{
    double third_quartile;
    size_t size = 3*(v.size()/4);
    size_t size_1 = v.size()/4;

    sort(v.begin(), v.end());

    if (size_1  % 2 == 0)
    {
        third_quartile = (v[size + size_1/2 - 1] + v[size + size_1/2]) / 2;
    }
    else
    {
        third_quartile = v[size + size_1/2];
    }

    return third_quartile;
}

int MainWindow::binomialCoeff(int n, int k)
{
    // https://www.geeksforgeeks.org/binomial-coefficient-dp-9/

    if (k>=0 && k<=n)
    {
        int C[n + 1][k + 1];
        int i, j;

        // Caculate value of Binomial Coefficient
        // in bottom up manner
        for (i = 0; i <= n; i++)
        {
            for (j = 0; j <= min(i, k); j++)
            {
                // Base Cases
                if (j == 0 || j == i)
                    C[i][j] = 1;

                // Calculate value using previosly
                // stored values
                else
                    C[i][j] = C[i - 1][j - 1] +
                              C[i - 1][j];
            }
        }

        return C[n][k];
    }else{
        return 0;
    }
}

double MainWindow::getNoiseRobustDerivate(int N, double h, std::deque<double>& buff)
{
    // http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
    static const std::runtime_error ex_cap(std::string("The capacity of the buffer differs the given filter length"));
    static const std::runtime_error ex_length(std::string("The filter length must be greater or equal to 5"));
    if(N>=5)
    {
        if(N==buff.size())
        {
            int M = (N-1)/2;
            int m = (N-3)/2;
            double sum=0.0;
            for(int k=1;k<=M;++k)
            {
                double c1 = this->binomialCoeff(2*m,(m-k+1));
                double c2 = this->binomialCoeff(2*m,(m-k-1));
                double ck = (1/pow(2,((2*m)+1)))*(c1-c2);
                sum+=ck*(buff[M+k]-buff[M-k]);
            }
            double der = sum/h;
            return der;
        }else{
            throw ex_cap;
        }
    }else{
        throw ex_length;
    }
}

double MainWindow::getPredictedSwivelAngle(VectorXd hand_pos)
{
    // position
    double hand_x = hand_pos(0); double hand_y = hand_pos(1); double hand_z = hand_pos(2);
//    double hand_x = -hand_pos(0); double hand_y = hand_pos(1); double hand_z = hand_pos(2);
    Quaterniond hand_or_q;
    hand_or_q.x() = hand_pos(3);
    hand_or_q.y() = hand_pos(4);
    hand_or_q.z() = hand_pos(5);
    hand_or_q.w() = hand_pos(6);
    Vector3d rpy = hand_or_q.toRotationMatrix().eulerAngles(2,1,2); // ZYZ angles
    double hand_roll = rpy(0); double hand_pitch = rpy(1); double hand_yaw = rpy(2);


    double alpha = 0.88*cos(hand_pitch+2.30)+0.49*cos(hand_pitch+hand_yaw+2.98)
                    -0.21*atan2(hand_z,hand_y)-0.64*sin(hand_roll)-1.39
                    -0.5*atan2(hand_y,hand_x)-0.22*cos(hand_roll+hand_pitch+hand_yaw)
                    +0.47*cos(hand_yaw+0.6)-0.15*atan2(hand_z,hand_x)
                    +0.34*cos(hand_roll+hand_yaw+1.07)-0.4*cos(hand_roll+hand_pitch);

//    double alpha_r = (2*M_PI+alpha)-M_PI/2;
    return alpha;
}


double MainWindow::getDerivativePredictedSwivelAngle(VectorXd hand_pos, VectorXd hand_vel)
{
    // position
    Vector3d hand_p = hand_pos.block<3,1>(0,0);
    double hand_x = hand_p(0); double hand_y = hand_p(1); double hand_z = hand_p(2);
    Vector3d hand_or_q_e = hand_pos.block<3,1>(3,0); double hand_or_q_w = hand_pos(6);
    Quaterniond hand_or_q;
    hand_or_q.x() = hand_pos(3);
    hand_or_q.y() = hand_pos(4);
    hand_or_q.z() = hand_pos(5);
    hand_or_q.w() = hand_pos(6);
    Vector3d rpy = hand_or_q.toRotationMatrix().eulerAngles(2,1,2); // ZYZ angles
    double hand_roll = rpy(0); double hand_pitch = rpy(1); double hand_yaw = rpy(2);
    Matrix3d T;
    T(0,0)=0; T(0,1)=-sin(hand_roll); T(0,2)=cos(hand_roll)*sin(hand_pitch);
    T(1,0)=0; T(1,1)=cos(hand_roll); T(1,2)=sin(hand_roll)*sin(hand_pitch);
    T(2,0)=1; T(2,1)=0; T(2,2)=cos(hand_pitch);
    // velocity
    Vector3d hand_lin_vel = hand_vel.block<3,1>(0,0);
    double hand_vel_x = hand_lin_vel(0); double hand_vel_y = hand_lin_vel(1); double hand_vel_z = hand_lin_vel(2);
    Vector3d hand_ang_vel_q_e = hand_vel.block<3,1>(3,0); double hand_ang_vel_q_w = hand_vel(6);
    Vector3d omega = 2*hand_or_q_e.cross(hand_ang_vel_q_e)+2*hand_or_q_w*hand_ang_vel_q_e-2*hand_ang_vel_q_w*hand_or_q_e;
    Vector3d rpy_vel; // ZYZ velocities
    if(abs(sin(hand_pitch))<0.0001){
        // singularity
        rpy_vel(0) = omega(2);
        rpy_vel(1) = sqrt(pow(omega(0),2)+pow(omega(1),2));
        rpy_vel(2) = 0;
    }else{
        rpy_vel = T.inverse()*omega;
    }
    double hand_vel_roll = rpy_vel(0); double hand_vel_pitch = rpy_vel(1); double hand_vel_yaw = rpy_vel(2);

    double datan2_zy = (hand_vel_z*hand_y-hand_vel_y*hand_z)/(pow(hand_y,2)+pow(hand_z,2));
    double datan2_yx = (hand_vel_y*hand_x-hand_vel_x*hand_y)/(pow(hand_x,2)+pow(hand_y,2));
    double datan2_zx = (hand_vel_z*hand_x-hand_vel_x*hand_z)/(pow(hand_x,2)+pow(hand_z,2));

    double der_alpha = -0.88*hand_vel_pitch*sin(hand_pitch+2.3)-0.49*(hand_vel_pitch+hand_vel_yaw)*sin(hand_pitch+hand_yaw+2.98)
                        -0.21*datan2_zy-0.64*hand_vel_roll*cos(hand_roll)
                        -0.5*datan2_yx+0.22*(hand_vel_roll+hand_vel_pitch+hand_vel_yaw)*sin(hand_roll+hand_pitch+hand_yaw)
                        -0.47*hand_vel_pitch*sin(hand_pitch+0.6)-0.15*datan2_zx
                        -0.34*(hand_vel_roll+hand_vel_yaw)*sin(hand_roll+hand_yaw+1.07)+0.4*(hand_vel_roll+hand_vel_pitch)*sin(hand_roll+hand_pitch);
    return der_alpha;
}

void MainWindow::updatePlanningResults(problemPtr prob, HUMotion::planning_result_ptr results)
{
    this->curr_mov = prob->getMovement();
    this->timesteps_mov.clear();
    this->jointsPosition_mov.clear(); this->jointsPosition_mov = results->trajectory_stages;
    this->jointsVelocity_mov.clear(); this->jointsVelocity_mov = results->velocity_stages;
    this->jointsAcceleration_mov.clear(); this->jointsAcceleration_mov = results->acceleration_stages;
    this->traj_descr_mov.clear(); this->traj_descr_mov = results->trajectory_descriptions;
    this->final_warm_start_res_mov = results->final_warm_start_res;
    this->bounce_warm_start_res_mov = results->bounce_warm_start_res;
    std::vector<double> timesteps_stage_aux;
    for(size_t i=0; i<results->trajectory_stages.size();++i){
        timesteps_stage_aux.clear();
        double t_stage = results->time_steps.at(i);
        MatrixXd traj_stage = results->trajectory_stages.at(i);
        if(i==0){
            // plan stage
            this->warm_n_steps_mov = traj_stage.rows()-1;
        }
        for(int j=0;j<traj_stage.rows();++j){
            if(j==traj_stage.rows()-1){
                timesteps_stage_aux.push_back(0.0);
            }else{
                timesteps_stage_aux.push_back(t_stage);
            }
        }
        this->timesteps_mov.push_back(timesteps_stage_aux);
    }
    this->moveit_mov=false;

    // make a copy of the human-like parameters for controlling
    this->tols_ctrl = this->tols;
    this->tols_ctrl.mov_specs.warm_start = true;
    this->tols_ctrl.mov_specs.warm_n_steps = this->warm_n_steps_mov;
    this->tols_ctrl.mov_specs.final_warm_start_params = this->final_warm_start_res_mov;
    this->tols_ctrl.mov_specs.bounce_warm_start_params = this->bounce_warm_start_res_mov;

    // time taken to solve the problem
    this->prob_time_mov = prob->getTime();
    ui.label_solving_time->setText(QString::number(this->prob_time_mov));
    ui.label_solv_time_dual_value->setText(QString::number(this->prob_time_mov));

    uint tot_steps=0;
    QStringList h_headers; bool h_head=false; QStringList v_headers;
    double mov_duration = 0.0;
    vector<double> time; QVector<double> tot_timesteps;
    std::vector<std::vector<QString>> mov_steps;
    for(size_t k=0; k< this->jointsPosition_mov.size();++k){
        MatrixXd jointPosition_stage = this->jointsPosition_mov.at(k);
        MatrixXd jointVelocity_stage = this->jointsVelocity_mov.at(k);
        MatrixXd jointAcceleration_stage = this->jointsAcceleration_mov.at(k);
        std::vector<double> timestep_stage = this->timesteps_mov.at(k);
        std::vector<QString> stage_step;
        double time_init;
        if(time.empty()){
            time_init=0.0;
        }else{
            time_init=time.at(time.size()-1);
        }
        vector<double> time_stage(timestep_stage.size());
        time_stage.at(0) = time_init;
        double stage_duration = 0.0;
        for(int i = 0; i< jointPosition_stage.rows(); ++i){
            tot_steps++;
            tot_timesteps.push_back(timestep_stage.at(i));
            if(i>0){
                stage_duration += timestep_stage.at(i);
                time_stage.at(i) = time_stage.at(i-1) + timestep_stage.at(i-1);
            }
            stage_step.clear();
            v_headers.push_back(QString("Step ")+QString::number(i));
            for (int j=0; j<jointPosition_stage.cols();++j){
                stage_step.push_back(
                        QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                        QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                        QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                if(!h_head){h_headers.push_back(QString("Joint ")+QString::number(j+1));}
            } // columns
            h_head = true;
            mov_steps.push_back(stage_step);
        }// rows
        mov_duration += stage_duration;
        time.reserve(time_stage.size());
        std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
    }// movements
    this->qtime_mov = QVector<double>::fromStdVector(time);

    // show the results
    ui.tableWidget_sol_mov->setColumnCount(h_headers.size());
    ui.tableWidget_sol_mov->setHorizontalHeaderLabels(h_headers);
    ui.tableWidget_sol_mov->setRowCount(v_headers.size());
    ui.tableWidget_sol_mov->setVerticalHeaderLabels(v_headers);
    for(int i =0; i < v_headers.size(); ++i){
        std::vector<QString> row = mov_steps.at(i);
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           ui.tableWidget_sol_mov->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    ui.label_totalTime_value_mov->setText(QString::number(mov_duration).toStdString().c_str());
    ui.tabWidget_sol->setEnabled(true);
    if(this->moveit_mov){
        ui.pushButton_execMov_moveit->setEnabled(true);
    }else{
        ui.pushButton_execMov_moveit->setEnabled(false);
    }

    this->tols_stop_mov.clear();
    double tol_stop = ui.lineEdit_tol_stop_mov->text().toDouble();
    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
        this->tols_stop_mov.push_back(tol_stop);
    }

    // compute the hand values, positions and accelerations
    //hand
    this->jointsPosition_mov_ctrl.resize(this->jointsPosition_mov.size());
    this->des_handPosition.clear(); this->des_handOrientation.clear(); this->des_handOrientation_q.clear();
    this->handPosition_mov.resize(tot_steps); this->handPosition_mov_stages.resize(this->jointsPosition_mov.size());
    this->handOrientation_mov.resize(tot_steps); this->handOrientation_q_mov.resize(tot_steps);
    this->handOrientation_mov_stages.resize(this->jointsPosition_mov.size());
    this->handOrientation_q_mov_stages.resize(this->jointsPosition_mov.size());
    this->wristPosition_mov.resize(tot_steps); this->wristOrientation_mov.resize(tot_steps);
    this->elbowPosition_mov.resize(tot_steps); this->elbowOrientation_mov.resize(tot_steps);
    this->shoulderPosition_mov.resize(tot_steps); this->shoulderOrientation_mov.resize(tot_steps);
    this->handVelocityNorm_mov.resize(tot_steps);
    this->handLinearVelocity_mov.resize(tot_steps); this->handLinearVelocity_mov_stages.resize(this->jointsPosition_mov.size());
    this->handAngularVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov_stages.resize(this->jointsPosition_mov.size());
    this->handLinearAcceleration_mov_stages.resize(this->jointsPosition_mov.size());
    this->handAngularAcceleration_mov_stages.resize(this->jointsPosition_mov.size());
    // wrist
    this->wristVelocityNorm_mov.resize(tot_steps);
    this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
    // elbow
    this->elbowVelocityNorm_mov.resize(tot_steps);
    this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
    //shoulder
    this->shoulderVelocityNorm_mov.resize(tot_steps);
    this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);
    // swivel angle
    this->swivel_angle_mov.resize(tot_steps);

    vector<double> timesteps_mov_tot(tot_steps,0.0);
    int step = 0;
    int arm_code = prob->getMovement()->getArm();
    // bounce posture data
    vector<double> bounce_posture = this->h_results->bounce_warm_start_res.x;
    vector<double> bounce_arm_posture(bounce_posture.begin(),bounce_posture.begin()+JOINTS_ARM);
    this->curr_scene->getHumanoid()->getHandPos(arm_code,this->bounce_handPosition,bounce_arm_posture);
    this->curr_scene->getHumanoid()->getHandOr(arm_code,this->bounce_handOrientation,bounce_arm_posture);
    this->curr_scene->getHumanoid()->getHandOr_q(arm_code,this->bounce_handOrientation_q,bounce_arm_posture);

    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
        MatrixXd pos_stage = this->jointsPosition_mov.at(k);
        MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
        MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
        vector<double> timesteps_mov_stage = this->timesteps_mov.at(k);
        //this->curr_scene->getHumanoid()->getHandAcceleration(arm_code,pos_stage,vel_stage,acc_stage,this->timesteps_mov.at(k),this->handLinearAcceleration_mov_stages.at(k),this->handAngularAcceleration_mov_stages.at(k));
        vector<vector<double>> h_pos(pos_stage.rows());
        vector<vector<double>> h_or(pos_stage.rows());
        vector<vector<double>> h_or_q(pos_stage.rows());
        vector<vector<double>> h_lin_vel(pos_stage.rows());
        vector<vector<double>> h_ang_vel(pos_stage.rows());
        for(int i=0;i<pos_stage.rows();++i){
            timesteps_mov_tot.at(step) = timesteps_mov_stage.at(i);
            // position
            VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
            vector<double> posture; posture.resize(pos_row.size());
            VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
            // hand
            this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_mov.at(step),posture);
            this->curr_scene->getHumanoid()->getHandPos(arm_code,h_pos.at(i),posture);
            this->curr_scene->getHumanoid()->getHandOr(arm_code,this->handOrientation_mov.at(step),posture);
            this->curr_scene->getHumanoid()->getHandOr(arm_code,h_or.at(i),posture);
            this->curr_scene->getHumanoid()->getHandOr_q(arm_code,h_or_q.at(i),posture);
            this->curr_scene->getHumanoid()->getHandOr_q(arm_code,this->handOrientation_q_mov.at(step),posture);
            // wrist
            this->curr_scene->getHumanoid()->getWristPos(arm_code,this->wristPosition_mov.at(step),posture);
            this->curr_scene->getHumanoid()->getWristOr(arm_code,this->wristOrientation_mov.at(step),posture);
            // elbow
            this->curr_scene->getHumanoid()->getElbowPos(arm_code,this->elbowPosition_mov.at(step),posture);
            this->curr_scene->getHumanoid()->getElbowOr(arm_code,this->elbowOrientation_mov.at(step),posture);
            // shoulder
            this->curr_scene->getHumanoid()->getShoulderPos(arm_code,this->shoulderPosition_mov.at(step),posture);
            this->curr_scene->getHumanoid()->getShoulderOr(arm_code,this->shoulderOrientation_mov.at(step),posture);
            // swivel angle
            this->swivel_angle_mov.at(step) = this->curr_scene->getHumanoid()->getSwivelAngle(arm_code,posture);
            // velocities
            VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
            vector<double> velocities; velocities.resize(vel_row.size());
            VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
            // hand velocity
            this->handVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
            vector<double> hand_vel; this->curr_scene->getHumanoid()->getHandVel(arm_code,hand_vel,posture,velocities);
            this->handLinearVelocity_mov.at(step) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
            this->handAngularVelocity_mov.at(step) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
            h_lin_vel.at(i) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
            h_ang_vel.at(i) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
            // wrist velocity
            this->wristVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getWristVelNorm(arm_code,posture,velocities);
            vector<double> wrist_vel; this->curr_scene->getHumanoid()->getWristVel(arm_code,wrist_vel,posture,velocities);
            this->wristLinearVelocity_mov.at(step) = {wrist_vel.at(0),wrist_vel.at(1),wrist_vel.at(2)};
            this->wristAngularVelocity_mov.at(step) = {wrist_vel.at(3),wrist_vel.at(4),wrist_vel.at(5)};
            // elbow velocity
            this->elbowVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getElbowVelNorm(arm_code,posture,velocities);
            vector<double> elbow_vel; this->curr_scene->getHumanoid()->getElbowVel(arm_code,elbow_vel,posture,velocities);
            this->elbowLinearVelocity_mov.at(step) = {elbow_vel.at(0),elbow_vel.at(1),elbow_vel.at(2)};
            this->elbowAngularVelocity_mov.at(step) = {elbow_vel.at(3),elbow_vel.at(4),elbow_vel.at(5)};
            // shoulder velocity
            this->shoulderVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getShoulderVelNorm(arm_code,posture,velocities);
            vector<double> shoulder_vel; this->curr_scene->getHumanoid()->getShoulderVel(arm_code,shoulder_vel,posture,velocities);
            this->shoulderLinearVelocity_mov.at(step) = {shoulder_vel.at(0),shoulder_vel.at(1),shoulder_vel.at(2)};
            this->shoulderAngularVelocity_mov.at(step) = {shoulder_vel.at(3),shoulder_vel.at(4),shoulder_vel.at(5)};

            step++;
        }// loop steps in the stage

        this->jointsPosition_mov_ctrl.at(k) = pos_stage;
        this->handPosition_mov_stages.at(k) = h_pos;
        this->handOrientation_mov_stages.at(k) = h_or;
        this->handOrientation_q_mov_stages.at(k) = h_or_q;
        this->handLinearVelocity_mov_stages.at(k) = h_lin_vel;
        this->handAngularVelocity_mov_stages.at(k) = h_ang_vel;
        this->getDerivative(this->handLinearVelocity_mov_stages.at(k),this->timesteps_mov.at(k),this->handLinearAcceleration_mov_stages.at(k));
        this->getDerivative(this->handAngularVelocity_mov_stages.at(k),this->timesteps_mov.at(k),this->handAngularAcceleration_mov_stages.at(k));
        this->des_handPosition.push_back(this->handPosition_mov.at(step-1));
        this->des_handOrientation.push_back(this->handOrientation_mov.at(step-1));
        this->des_handOrientation_q.push_back(this->handOrientation_q_mov.at(step-1));

    }// loop stages

    // max and min swivel angles
    this->swivel_angle_mov_max = *std::max_element(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end());
    this->swivel_angle_mov_min = *std::min_element(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end());
    this->swivel_angle_mov_average = std::accumulate(this->swivel_angle_mov.begin(),this->swivel_angle_mov.end(),0.0)/this->swivel_angle_mov.size();

    // accelerations
    this->getDerivative(this->handLinearVelocity_mov,timesteps_mov_tot,this->handLinearAcceleration_mov);
    this->getDerivative(this->handAngularVelocity_mov,timesteps_mov_tot,this->handAngularAcceleration_mov);
    this->getDerivative(this->wristLinearVelocity_mov,timesteps_mov_tot,this->wristLinearAcceleration_mov);
    this->getDerivative(this->wristAngularVelocity_mov,timesteps_mov_tot,this->wristAngularAcceleration_mov);
    this->getDerivative(this->elbowLinearVelocity_mov,timesteps_mov_tot,this->elbowLinearAcceleration_mov);
    this->getDerivative(this->elbowAngularVelocity_mov,timesteps_mov_tot,this->elbowAngularAcceleration_mov);
    this->getDerivative(this->shoulderLinearVelocity_mov,timesteps_mov_tot,this->shoulderLinearAcceleration_mov);
    this->getDerivative(this->shoulderAngularVelocity_mov,timesteps_mov_tot,this->shoulderAngularAcceleration_mov);

    // -- normlized jerk cost of the hand -- //
    QVector<double> handPosition_mov_x; QVector<double> handPosition_mov_y; QVector<double> handPosition_mov_z;
    QVector<double> der_1_handPosition_mov_x; QVector<double> der_1_handPosition_mov_y; QVector<double> der_1_handPosition_mov_z;
    QVector<double> der_2_handPosition_mov_x; QVector<double> der_2_handPosition_mov_y; QVector<double> der_2_handPosition_mov_z;
    QVector<double> der_3_handPosition_mov_x; QVector<double> der_3_handPosition_mov_y; QVector<double> der_3_handPosition_mov_z;

    for(size_t i=0; i<this->handPosition_mov.size();++i){
        vector<double> position_i = this->handPosition_mov.at(i);
        handPosition_mov_x.push_back(position_i.at(0));
        handPosition_mov_y.push_back(position_i.at(1));
        handPosition_mov_z.push_back(position_i.at(2));
    }

    // derivatives
    this->getDerivative(handPosition_mov_x,tot_timesteps,der_1_handPosition_mov_x);
    this->getDerivative(der_1_handPosition_mov_x,tot_timesteps,der_2_handPosition_mov_x);
    this->getDerivative(der_2_handPosition_mov_x,tot_timesteps,der_3_handPosition_mov_x);
    this->getDerivative(handPosition_mov_y,tot_timesteps,der_1_handPosition_mov_y);
    this->getDerivative(der_1_handPosition_mov_y,tot_timesteps,der_2_handPosition_mov_y);
    this->getDerivative(der_2_handPosition_mov_y,tot_timesteps,der_3_handPosition_mov_y);
    this->getDerivative(handPosition_mov_z,tot_timesteps,der_1_handPosition_mov_z);
    this->getDerivative(der_1_handPosition_mov_z,tot_timesteps,der_2_handPosition_mov_z);
    this->getDerivative(der_2_handPosition_mov_z,tot_timesteps,der_3_handPosition_mov_z);

    QVector<double> jerk_hand;
    for(size_t i=0;i<handPosition_mov_x.size();++i){
        jerk_hand.push_back(sqrt(pow(der_3_handPosition_mov_x.at(i),2)+pow(der_3_handPosition_mov_y.at(i),2)+pow(der_3_handPosition_mov_z.at(i),2)));
    }
    double duration = this->qtime_mov.at(this->qtime_mov.size()-1);
    double length = sqrt(pow((handPosition_mov_x.at(handPosition_mov_x.size()-1)-handPosition_mov_x.at(0)),2)+
                         pow((handPosition_mov_y.at(handPosition_mov_y.size()-1)-handPosition_mov_y.at(0)),2)+
                         pow((handPosition_mov_z.at(handPosition_mov_z.size()-1)-handPosition_mov_z.at(0)),2));
    double total_cost_jerk_hand=0.0;
    for(size_t i=0;i<tot_timesteps.size();++i){
        total_cost_jerk_hand += pow(jerk_hand.at(i),2)*tot_timesteps.at(i);
    }
    total_cost_jerk_hand = sqrt(0.5*total_cost_jerk_hand*(pow(duration,5)/pow(length,2)));
    ui.label_cost_hand_value->setText(QString::number(total_cost_jerk_hand));
    this->njs_mov = total_cost_jerk_hand;

    // -- compute the number of movement units -- //
    this->nmu_mov = this->getNumberMovementUnits(this->handVelocityNorm_mov,this->qtime_mov);
    ui.label_nmu->setText(QString::number(this->nmu_mov));

}

void MainWindow::clear_control_variables()
{
    this->handPosition_ctrl.clear();
    this->handOrientation_ctrl.clear();
    this->handOrientation_q_ctrl.clear();
    this->handPosition_des_ctrl.clear();
    this->fingPosition_ctrl.clear();
    this->fingPosition_des_ctrl.clear();
    this->handLinearVelocity_ctrl.clear();
    this->handAngularVelocity_ctrl.clear();
    this->handLinearAcceleration_ctrl.clear();
    this->handAngularAcceleration_ctrl.clear();
    this->wristPosition_ctrl.clear();
    this->wristOrientation_ctrl.clear();
    this->wristLinearVelocity_ctrl.clear();
    this->wristAngularVelocity_ctrl.clear();
    this->wristLinearAcceleration_ctrl.clear();
    this->wristAngularAcceleration_ctrl.clear();
    this->elbowPosition_ctrl.clear();
    this->elbowOrientation_ctrl.clear();
    this->elbowLinearVelocity_ctrl.clear();
    this->elbowAngularVelocity_ctrl.clear();
    this->elbowLinearAcceleration_ctrl.clear();
    this->elbowAngularAcceleration_ctrl.clear();
    this->shoulderPosition_ctrl.clear();
    this->shoulderOrientation_ctrl.clear();
    this->shoulderLinearVelocity_ctrl.clear();
    this->shoulderAngularVelocity_ctrl.clear();
    this->shoulderLinearAcceleration_ctrl.clear();
    this->shoulderAngularAcceleration_ctrl.clear();
    this->handVelocityNorm_ctrl.clear();
    this->handAccelerationNorm_ctrl.clear();
    this->jointsPosition_ctrl.resize(0,0);
    this->jointsVelocity_ctrl.resize(0,0);
    this->jointsVelocity_null_ctrl.resize(0,0);
    this->jointsAcceleration_ctrl.resize(0,0);
    this->r_hand_init_pos.clear();
    this->sim_time.clear();
    this->error_fing_pos.clear();
    this->error_fing_vel.clear();
    this->error_fing_acc.clear();
    this->error_pos_tot_norm.clear();
    this->error_or_tot_norm.clear();
    this->error_pos_or_tot_norm.clear();
    this->error_lin_vel_tot_norm.clear();
    this->error_ang_vel_tot_norm.clear();
    this->error_vel_tot_norm.clear();
    this->error_lin_acc_tot_norm.clear();
    this->error_ang_acc_tot_norm.clear();
    this->error_acc_tot_norm.clear();
    this->pred_swivel_angle_ctrl.clear();
    this->pred_der_swivel_angle_ctrl.clear();
    this->i_ctrl=0;
    this->t_past=0.0;
    this->t_j_past=0.0;
    this->t_der_past=0.0;
    this->t_der_past_ctrl=sec::zero();
    this->t_past_ctrl=sec::zero();
    this->samples_des_hand_pose=0;
    this->samples_des_hand_vel=0;
    this->samples_pos=0;
    this->samples_vel=0;
    this->samples_h_vel=0;
    this->samples_w_vel=0;
    this->samples_e_vel=0;
    this->samples_s_vel=0;
    this->exec_command_ctrl=true;
    this->replanning_succeed=false;
    this->replanning_done=false;
    this->des_hand_pose_buff = boost::make_shared<CircularBuffers<double>>(7, this->N_filter_length);
    this->des_hand_vel_buff = boost::make_shared<CircularBuffers<double>>(7, this->N_filter_length);
    this->arm_pos_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_ARM, this->N_filter_length);
    this->hand_pos_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_HAND, this->N_filter_length);
    this->arm_vel_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_ARM, this->N_filter_length);
    this->hand_vel_buff = boost::make_shared<CircularBuffers<double>>(JOINTS_HAND, this->N_filter_length);
    this->r_hand_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_wrist_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_elbow_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->r_shoulder_vel_buff = boost::make_shared<CircularBuffers<double>>(6, this->N_filter_length);
    this->hand_j_acc = VectorXd::Zero(6);
    this->Jacobian = MatrixXd::Zero(6,JOINTS_ARM);
}


void MainWindow::count_occurrence(std::unordered_map<int,int>& m, std::vector<int>& v){
    m.clear();
    for (auto itr = v.begin(); itr != v.end(); ++itr){
        ++m[*itr];
    }
}

double MainWindow::median(std::vector<double>& len)
{
    if(!len.empty())
    {
        if (len.size() % 2 == 0) {
            const auto median_it1 = len.begin() + len.size() / 2 - 1;
            const auto median_it2 = len.begin() + len.size() / 2;

            std::nth_element(len.begin(), median_it1 , len.end());
            const auto e1 = *median_it1;

            std::nth_element(len.begin(), median_it2 , len.end());
            const auto e2 = *median_it2;

            return (e1 + e2) / 2;

        } else {
            const auto median_it = len.begin() + len.size() / 2;
            std::nth_element(len.begin(), median_it , len.end());
            return *median_it;
        }
    }else{
        return -1.0;
    }
}

void MainWindow::add_microsteps(std::vector<vector<double>>& h_func, std::vector<vector<double>>& h_micro_func, int microsteps)
{
    h_micro_func.push_back(h_func.at(0));
    for(size_t i=0;i<h_func.size()-1;++i)
    {// for loop steps
        vector<double> func_i = h_func.at(i);
        vector<double> func_i1 = h_func.at(i+1);
        for(int k=1;k<=microsteps;++k)
        {// for loop microsteps
            vector<double> func_k(func_i.size());
            for(size_t j=0;j<func_i.size();++j)
            {// for loop variables
                double y_a = func_i.at(j);
                double y_b = func_i1.at(j);
                func_k.at(j) = y_a + (y_b-y_a)*k/(microsteps+1);
            }// for loop variables
            h_micro_func.push_back(func_k);
        }// for loop microsteps
        h_micro_func.push_back(func_i1);
    }// for loop steps
}

void MainWindow::add_microsteps(MatrixXd& h_func, MatrixXd& h_micro_func, int microsteps)
{
    VectorXd joint_0 = h_func.row(0);
    h_micro_func.conservativeResize(h_micro_func.rows() + 1, Eigen::NoChange);
    h_micro_func.row(h_micro_func.rows()-1) = joint_0;

    for(int i=0;i<h_func.rows()-1;++i)
    {// for loop steps
        VectorXd joint_i = h_func.row(i);
        VectorXd joint_i1 = h_func.row(i+1);
        for(int k=1;k<=microsteps;++k)
        {// for loop microsteps
            VectorXd func_k(joint_i.size());
            for(int j=0;j<joint_i.size();++j)
            {// for loop joints
                double y_a = joint_i(j); double y_b = joint_i1(j);
                func_k(j) = y_a + (y_b-y_a)*k/(microsteps+1);
            }// for loop joints
            h_micro_func.conservativeResize(h_micro_func.rows() + 1, Eigen::NoChange);
            h_micro_func.row(h_micro_func.rows()-1) = func_k;
        }// for loop microsteps
        h_micro_func.conservativeResize(h_micro_func.rows() + 1, Eigen::NoChange);
        h_micro_func.row(h_micro_func.rows()-1) = joint_i1;
    }// for loop steps
}

void MainWindow::getQuaternion(std::vector<double>& q, Matrix3d& Rot)
{
    Quaterniond qq(Rot);
    q.clear();
    q.push_back(qq.x());
    q.push_back(qq.y());
    q.push_back(qq.z());
    q.push_back(qq.w());

}

void MainWindow::Rot_matrix(Matrix3d &Rot,std::vector<double>& rpy)
{
    Rot = Matrix3d::Zero();

    double roll = rpy.at(0); // around z
    double pitch = rpy.at(1); // around y
    double yaw = rpy.at(2); // around x

    // Rot = Rot_z * Rot_y * Rot_x

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

}

void MainWindow::check_tar_x_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_x_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_x_var->setEnabled(true);
    }
}

void MainWindow::check_tar_y_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_y_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_y_var->setEnabled(true);
    }
}

void MainWindow::check_tar_z_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_z_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_z_var->setEnabled(true);
    }
}

void MainWindow::check_tar_roll_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_roll_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_roll_var->setEnabled(true);
    }
}

void MainWindow::check_tar_pitch_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_pitch_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_pitch_var->setEnabled(true);
    }
}

void MainWindow::check_tar_yaw_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_tar_yaw_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_tar_yaw_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_x_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_x_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_x_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_y_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_y_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_y_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_z_pos_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_z_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_z_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_roll_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_roll_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_roll_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_pitch_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_pitch_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_pitch_var->setEnabled(true);
    }
}

void MainWindow::check_obsts_yaw_var(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_obsts_yaw_var->setEnabled(false);
    }else{
        // checked
        this->ui.lineEdit_obsts_yaw_var->setEnabled(true);
    }
}

void MainWindow::check_right_hand_status(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_right_hand_comp->setEnabled(false);
    }else{
        // checked
        this->ui.groupBox_right_hand_comp->setEnabled(true);
    }
}

void MainWindow::check_use_vel_control(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_des_right_hand_pos->setEnabled(true);
        this->ui.groupBox_des_right_hand_vel->setEnabled(false);
    }else{
        // checked
        this->ui.groupBox_des_right_hand_pos->setEnabled(false);
        this->ui.groupBox_des_right_hand_vel->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_pos_x(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_pos_x->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_pos_x->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_pos_y(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_pos_y->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_pos_y->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_pos_z(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_pos_z->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_pos_z->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_q_x(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_q_x->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_q_x->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_q_y(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_q_y->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_q_y->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_q_z(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_q_z->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_q_z->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_q_w(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_q_w->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_q_w->setEnabled(true);
    }
}

void MainWindow::check_use_plan_hand_pos(int state)
{
    if(state==0){
        // unchecked
        this->ui.checkBox_des_right_hand_pos_x->setEnabled(true);
        this->ui.lineEdit_des_right_hand_pos_x->setEnabled(true);
        this->ui.checkBox_des_right_hand_pos_y->setEnabled(true);
        this->ui.lineEdit_des_right_hand_pos_y->setEnabled(true);
        this->ui.checkBox_des_right_hand_pos_z->setEnabled(true);
        this->ui.lineEdit_des_right_hand_pos_z->setEnabled(true);
        this->ui.checkBox_des_right_hand_q_x->setEnabled(true);
        this->ui.lineEdit_des_right_hand_q_x->setEnabled(true);
        this->ui.checkBox_des_right_hand_q_y->setEnabled(true);
        this->ui.lineEdit_des_right_hand_q_y->setEnabled(true);
        this->ui.checkBox_des_right_hand_q_z->setEnabled(true);
        this->ui.lineEdit_des_right_hand_q_z->setEnabled(true);
        this->ui.checkBox_des_right_hand_q_w->setEnabled(true);
        this->ui.lineEdit_des_right_hand_q_w->setEnabled(true);
    }else{
       // checked
        this->ui.checkBox_des_right_hand_pos_x->setEnabled(false);
        this->ui.lineEdit_des_right_hand_pos_x->setEnabled(false);
        this->ui.checkBox_des_right_hand_pos_y->setEnabled(false);
        this->ui.lineEdit_des_right_hand_pos_y->setEnabled(false);
        this->ui.checkBox_des_right_hand_pos_z->setEnabled(false);
        this->ui.lineEdit_des_right_hand_pos_z->setEnabled(false);
        this->ui.checkBox_des_right_hand_q_x->setEnabled(false);
        this->ui.lineEdit_des_right_hand_q_x->setEnabled(false);
        this->ui.checkBox_des_right_hand_q_y->setEnabled(false);
        this->ui.lineEdit_des_right_hand_q_y->setEnabled(false);
        this->ui.checkBox_des_right_hand_q_z->setEnabled(false);
        this->ui.lineEdit_des_right_hand_q_z->setEnabled(false);
        this->ui.checkBox_des_right_hand_q_w->setEnabled(false);
        this->ui.lineEdit_des_right_hand_q_w->setEnabled(false);
    }
}

void MainWindow::check_des_right_hand_vel_x(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_x->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_x->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_vel_y(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_y->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_y->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_vel_z(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_z->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_z->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_vel_wx(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_wx->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_wx->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_vel_wy(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_wy->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_wy->setEnabled(true);
    }
}

void MainWindow::check_des_right_hand_vel_wz(int state)
{
    if(state==0){
        // unchecked
        this->ui.lineEdit_des_right_hand_vel_wz->setEnabled(false);
    }else{
       // checked
       this->ui.lineEdit_des_right_hand_vel_wz->setEnabled(true);
    }
}

void MainWindow::check_ctrl_joints_limits_av(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_jlim_params->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_jlim_params->setEnabled(true);
    }
}

void MainWindow::check_ctrl_sing_av(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_sing_av_params->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_sing_av_params->setEnabled(true);
    }
}

void MainWindow::check_ctrl_obsts_av(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_obsts_av_params->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_obsts_av_params->setEnabled(true);
    }
}

void MainWindow::check_ctrl_obsts_filter_noise(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_obsts_filter_noise->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_obsts_filter_noise->setEnabled(true);
    }
}

void MainWindow::check_ctrl_tar_filter_noise(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_tar_filter_noise->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_tar_filter_noise->setEnabled(true);
    }
}

void MainWindow::check_ctrl_hl_add(int state)
{
    if(state==0){
        // unchecked
        this->ui.groupBox_hl_add_params->setEnabled(false);
    }else{
       // checked
       this->ui.groupBox_hl_add_params->setEnabled(true);
    }
}

void MainWindow::check_draw_ellipse(int state)
{
    if(state==0){
        // unchecked
        this->ui.checkBox_des_right_hand_vel_x->setEnabled(true);
        this->ui.checkBox_des_right_hand_vel_y->setEnabled(true);
        this->ui.checkBox_des_right_hand_vel_z->setEnabled(true);
        this->ui.checkBox_des_right_hand_vel_wx->setEnabled(true);
        this->ui.checkBox_des_right_hand_vel_wy->setEnabled(true);
        this->ui.checkBox_des_right_hand_vel_wz->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_x->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_y->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_z->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_wx->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_wy->setEnabled(true);
        this->ui.lineEdit_des_right_hand_vel_wz->setEnabled(true);
    }else{
       // checked
        this->ui.checkBox_des_right_hand_vel_x->setEnabled(false);
        this->ui.checkBox_des_right_hand_vel_y->setEnabled(false);
        this->ui.checkBox_des_right_hand_vel_z->setEnabled(false);
        this->ui.checkBox_des_right_hand_vel_wx->setEnabled(false);
        this->ui.checkBox_des_right_hand_vel_wy->setEnabled(false);
        this->ui.checkBox_des_right_hand_vel_wz->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_x->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_y->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_z->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_wx->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_wy->setEnabled(false);
        this->ui.lineEdit_des_right_hand_vel_wz->setEnabled(false);
    }
}

void MainWindow::on_pushButton_time_map_clicked()
{
    this->mTimeMapdlg->show();
}

void MainWindow::on_pushButton_control_plot_joints_clicked()
{
    if(this->jointsVelocity_ctrl.rows()!=0){
        vector<double> max_limits;
        vector<double> min_limits;
        this->curr_scene->getHumanoid()->getRightMaxLimits(max_limits);
        this->curr_scene->getHumanoid()->getRightMinLimits(min_limits);
        this->mResultsCtrlJointsdlg->setupPlots(this->jointsPosition_ctrl,this->jointsVelocity_ctrl,this->jointsAcceleration_ctrl,this->jointsVelocity_null_ctrl,max_limits,min_limits,this->sim_time);
    }
    this->mResultsCtrlJointsdlg->show();
}

void MainWindow::on_pushButton_pred_swivel_angle_clicked()
{
    if(!this->pred_swivel_angle_ctrl.empty())
    {
        this->mResultsCtrlPredSwivelAngledlg->setupPlots(this->pred_swivel_angle_ctrl,this->pred_der_swivel_angle_ctrl,this->sim_time);
    }
    this->mResultsCtrlPredSwivelAngledlg->show();
}

void MainWindow::on_pushButton_control_plot_pos_vel_comps_clicked()
{
    if(!this->shoulderLinearVelocity_ctrl.empty())
        this->mCompCtrldlg->setupPlots(this->shoulderPosition_ctrl,this->shoulderOrientation_ctrl,
                                       this->shoulderLinearVelocity_ctrl,this->shoulderAngularVelocity_ctrl,
                                       this->shoulderLinearAcceleration_ctrl,this->shoulderAngularAcceleration_ctrl,
                                       this->sim_time,0);
    if(!this->elbowLinearVelocity_ctrl.empty())
        this->mCompCtrldlg->setupPlots(this->elbowPosition_ctrl,this->elbowOrientation_ctrl,
                                       this->elbowLinearVelocity_ctrl,this->elbowAngularVelocity_ctrl,
                                       this->elbowLinearAcceleration_ctrl,this->elbowAngularAcceleration_ctrl,
                                       this->sim_time,1);
    if(!this->wristLinearVelocity_ctrl.empty())
        this->mCompCtrldlg->setupPlots(this->wristPosition_ctrl,this->wristOrientation_ctrl,
                                       this->wristLinearVelocity_ctrl,this->wristAngularVelocity_ctrl,
                                       this->wristLinearAcceleration_ctrl,this->wristAngularAcceleration_ctrl,
                                       this->sim_time,2);
    if(!this->handLinearVelocity_ctrl.empty())
        this->mCompCtrldlg->setupPlots(this->handPosition_ctrl,this->handOrientation_ctrl,
                                       this->handLinearVelocity_ctrl,this->handAngularVelocity_ctrl,
                                       this->handLinearAcceleration_ctrl,this->handAngularAcceleration_ctrl,
                                       this->sim_time,3);

    this->mCompCtrldlg->show();

}

void MainWindow::on_pushButton_errors_ctrl_clicked()
{
    if(!this->error_pos_tot_norm.empty())
        this->mErrCtrldlg->setupHandPlots(this->error_pos_tot_norm,this->error_or_tot_norm,this->error_pos_or_tot_norm,
                                      this->error_lin_vel_tot_norm,this->error_ang_vel_tot_norm,this->error_vel_tot_norm,
                                      this->error_lin_acc_tot_norm,this->error_ang_acc_tot_norm,this->error_acc_tot_norm,
                                      this->sim_time);

    if(!this->error_fing_pos.empty())
        this->mErrCtrldlg->setupFingersPlots(this->error_fing_pos,this->error_fing_vel,this->error_fing_acc,this->sim_time);


    this->mErrCtrldlg->show();
}

void MainWindow::on_pushButton_tracking_ctrl_clicked()
{
    if(!this->handPosition_ctrl.empty())
    {
        this->mCompTrackCtrldlg->setupPlots(this->handPosition_ctrl,this->handOrientation_q_ctrl,this->handPosition_des_ctrl,
                                            this->fingPosition_ctrl,this->fingPosition_des_ctrl,
                                            this->sim_time);
    }

    this->mCompTrackCtrldlg->show();
}


// -----------------------------------------------------------------------
// Controlling

void MainWindow::on_pushButton_start_control_pressed()
{
    this->qnode.reset_open_close_BH();
    this->ui.groupBox_sim_real->setEnabled(false);
    if(this->ui.checkBox_use_plan_hand_pos->isChecked())
    {
        // plan the original movement
        this->on_pushButton_plan_pressed();
        this->on_pushButton_plan_clicked();
    }
    if(this->ui.radioButton_sim->isChecked()){
        qnode.log(QNode::Info,string("Simulation Started"));
    }
    qnode.log(QNode::Info,string("Control Started"));
//    BOOST_LOG_SEV(lg, info) << "# ---------------- Control started ------------------- # ";


    this->clear_control_variables();
    this->qnode.resetSimTime();

    if(this->ui.checkBox_use_plan_hand_pos->isChecked())
    {
        this->mov_type_ctrl = this->curr_mov->getType();
        this->dHO_ctrl = this->tols.mov_specs.dHO;
        if(this->mov_type_ctrl==0){
            // pick
            this->approach_ctrl = this->tols.mov_specs.pre_grasp_approach;
            this->retreat_ctrl = this->tols.mov_specs.post_grasp_retreat;
        }else if(this->mov_type_ctrl==2 || this->mov_type_ctrl==3 || this->mov_type_ctrl==4){
            // place
            this->approach_ctrl = this->tols.mov_specs.pre_place_approach;
            this->retreat_ctrl = this->tols.mov_specs.post_place_retreat;
        }

        int mov_type = this->curr_mov->getType();
        if(mov_type==0){ // pick
            vector<objectPtr> objs; this->curr_scene->getObjects(objs);
            string obj_tar_name = this->curr_mov->getObject()->getName();
            for(size_t i=0;i<objs.size();++i)
            {
                if(obj_tar_name.compare(objs.at(i)->getName())==0)
                {
                    this->i_tar_ctrl = i;
                    break;
                }
            }
            Vector3d tar_pos;
            targetPtr tar = this->curr_scene->getObject(this->i_tar_ctrl)->getTargetRight();
            this->curr_scene->setHandTarget(tar);
            tar_pos(0) = tar->getPos().Xpos;
            tar_pos(1) = tar->getPos().Ypos;
            tar_pos(2) = tar->getPos().Zpos;
            Matrix3d Rot_tar; tar->RPY_matrix(Rot_tar);
            Quaterniond tar_q(Rot_tar);
//            this->tar_position= tar_pos;
//            this->tar_quaternion = tar_q;
        }else if(mov_type==2 || mov_type==3 || mov_type==4){ // place
            vector<posePtr> poses; this->curr_scene->getPoses(poses);
            string tar_name = this->curr_mov->getPose()->getName();
            for(size_t i=0;i<poses.size();++i)
            {
                if(tar_name.compare(poses.at(i)->getName())==0)
                {
                    this->i_tar_ctrl = i;
                    break;
                }
            }
            Vector3d tar_pos;
            posePtr tar = this->curr_scene->getPose(this->i_tar_ctrl);
            this->curr_scene->setHandPose(tar);
            tar_pos(0) = tar->getPos().Xpos;
            tar_pos(1) = tar->getPos().Ypos;
            tar_pos(2) = tar->getPos().Zpos;
            Matrix3d Rot_tar; tar->RPY_matrix(Rot_tar);
            Quaterniond tar_q(Rot_tar);
//            this->tar_position= tar_pos;
//            this->tar_quaternion = tar_q;
        }
    }

    this->lpf_tar_pos_x.reset(new LowPassFilter());
    this->lpf_tar_pos_y.reset(new LowPassFilter());
    this->lpf_tar_pos_z.reset(new LowPassFilter());
    this->lpf_tar_or_q_x.reset(new LowPassFilter());
    this->lpf_tar_or_q_y.reset(new LowPassFilter());
    this->lpf_tar_or_q_z.reset(new LowPassFilter());
    this->lpf_tar_or_q_w.reset(new LowPassFilter());

    this->lpf_obsts_pos_x.reset(new LowPassFilter());
    this->lpf_obsts_pos_y.reset(new LowPassFilter());
    this->lpf_obsts_pos_z.reset(new LowPassFilter());
    this->lpf_obsts_or_q_x.reset(new LowPassFilter());
    this->lpf_obsts_or_q_y.reset(new LowPassFilter());
    this->lpf_obsts_or_q_z.reset(new LowPassFilter());
    this->lpf_obsts_or_q_w.reset(new LowPassFilter());

    this->lpf_joint_pos_1.reset(new LowPassFilter());
    this->lpf_joint_pos_2.reset(new LowPassFilter());
    this->lpf_joint_pos_3.reset(new LowPassFilter());
    this->lpf_joint_pos_4.reset(new LowPassFilter());
    this->lpf_joint_pos_5.reset(new LowPassFilter());
    this->lpf_joint_pos_6.reset(new LowPassFilter());
    this->lpf_joint_pos_7.reset(new LowPassFilter());
    this->lpf_joint_pos_8.reset(new LowPassFilter());
    this->lpf_joint_pos_9.reset(new LowPassFilter());
    this->lpf_joint_pos_10.reset(new LowPassFilter());
    this->lpf_joint_pos_11.reset(new LowPassFilter());

    // noise filtering
    bool obsts_filter_noise = this->ui.checkBox_obsts_filter_noise->isChecked();
    double filter_obsts_cut_off_freq = 0.1; double filter_obsts_timestep = 0.05;
    bool tar_filter_noise = this->ui.checkBox_tar_filter_noise->isChecked();
    double filter_tar_cut_off_freq = 0.1; double filter_tar_timestep = 0.05;
    if(obsts_filter_noise){
        filter_obsts_cut_off_freq = this->ui.lineEdit_obsts_f_cutoff->text().toDouble();
        filter_obsts_timestep = this->ui.lineEdit_obsts_timestep->text().toDouble();
    }
    if(tar_filter_noise){
        filter_tar_cut_off_freq = this->ui.lineEdit_tar_f_cutoff->text().toDouble();
        filter_tar_timestep = this->ui.lineEdit_tar_timestep->text().toDouble();
    }

    this->lpf_tar_pos_x->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_pos_x->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_pos_y->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_pos_y->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_pos_z->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_pos_z->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_or_q_x->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_or_q_x->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_or_q_y->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_or_q_y->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_or_q_z->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_or_q_z->setDeltaTime(filter_tar_timestep);
    this->lpf_tar_or_q_w->setCutOffFrequency(filter_tar_cut_off_freq); this->lpf_tar_or_q_w->setDeltaTime(filter_tar_timestep);

    this->lpf_obsts_pos_x->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_pos_x->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_pos_y->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_pos_y->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_pos_z->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_pos_z->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_or_q_x->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_or_q_x->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_or_q_y->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_or_q_y->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_or_q_z->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_or_q_z->setDeltaTime(filter_obsts_timestep);
    this->lpf_obsts_or_q_w->setCutOffFrequency(filter_obsts_cut_off_freq); this->lpf_obsts_or_q_w->setDeltaTime(filter_obsts_timestep);

    double filter_cut_off_freq_j_pos = this->ui.lineEdit_cutoff_freq_joint_pos->text().toDouble();
    double filter_time_step_j_pos = this->ui.lineEdit_timestep_joint_pos->text().toDouble();
    this->lpf_joint_pos_1->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_1->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_2->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_2->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_3->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_3->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_4->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_4->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_5->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_5->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_6->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_6->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_7->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_7->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_8->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_8->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_9->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_9->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_10->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_10->setDeltaTime(filter_time_step_j_pos);
    this->lpf_joint_pos_11->setCutOffFrequency(filter_cut_off_freq_j_pos); this->lpf_joint_pos_11->setDeltaTime(filter_time_step_j_pos);
}

void MainWindow::on_pushButton_start_control_clicked()
{

    if(this->ui.checkBox_use_vel_control->isChecked())
    {
        pos_control = false;
        vel_control = true;
    }else{
        pos_control = true;
        vel_control = false;
    }
    this->start_time_point = Clock::now();
}

void MainWindow::on_pushButton_stop_control_pressed()
{    
    this->ui.groupBox_sim_real->setEnabled(true);
    this->qnode.log(QNode::Info,string("Control Stopped"));
    // stop the motion
    if(this->ui.radioButton_sim->isChecked()){
        this->qnode.log(QNode::Info,string("Simulation Stopped"));
    }else{
        this->exec_command_ctrl=false;
        sleep(1);
    }
    this->qnode.reset_open_close_BH();
}

void MainWindow::on_pushButton_stop_control_clicked()
{

    // restore the home posture
    vector<double> r_p; vector<double> l_p;
    this->curr_scene->getHumanoid()->getRightHomePosture(r_p);
    this->curr_scene->getHumanoid()->getLeftHomePosture(l_p);
    this->curr_scene->getHumanoid()->setRightPosture(r_p);
    this->curr_scene->getHumanoid()->setLeftPosture(l_p);
    this->exec_command_ctrl=false;
    this->replanning_succeed=false;
    this->replanning_done=false;
    pos_control = false;
    vel_control = false;
    this->qnode.stopSim();
    this->qnode.resetSimTime();
    if(this->ui.checkBox_use_plan_hand_pos->isChecked())
    {
        if (this->handLinearAcceleration_ctrl.size()>1){
            double timestep = this->qnode.getSimTimeStep();
            QVector<double> tot_timesteps(this->handLinearAcceleration_ctrl.size(),timestep);


            // -- normlized jerk cost of the hand -- //
            QVector<double> handPosition_mov_x; QVector<double> handPosition_mov_y; QVector<double> handPosition_mov_z;
            QVector<double> handAcceleration_mov_x; QVector<double> handAcceleration_mov_y; QVector<double> handAcceleration_mov_z;
            QVector<double> handJerk_mov_x; QVector<double> handJerk_mov_y; QVector<double> handJerk_mov_z;

            for(size_t i=0; i<this->handLinearAcceleration_ctrl.size();++i){
                vector<double> pos_i = this->handPosition_ctrl.at(i);
                handPosition_mov_x.push_back(pos_i.at(0));
                handPosition_mov_y.push_back(pos_i.at(1));
                handPosition_mov_z.push_back(pos_i.at(2));
                vector<double> acc_i = this->handLinearAcceleration_ctrl.at(i);
                handAcceleration_mov_x.push_back(acc_i.at(0));
                handAcceleration_mov_y.push_back(acc_i.at(1));
                handAcceleration_mov_z.push_back(acc_i.at(2));
            }

            // derivatives
            this->getDerivative(handAcceleration_mov_x,tot_timesteps,handJerk_mov_x);
            this->getDerivative(handAcceleration_mov_y,tot_timesteps,handJerk_mov_y);
            this->getDerivative(handAcceleration_mov_z,tot_timesteps,handJerk_mov_z);

            QVector<double> jerk_hand;
            for(size_t i=0;i<handAcceleration_mov_x.size();++i){
                jerk_hand.push_back(sqrt(pow(handJerk_mov_x.at(i),2)+pow(handJerk_mov_y.at(i),2)+pow(handJerk_mov_z.at(i),2)));
            }
            double duration = this->sim_time.at(this->sim_time.size()-1);
            double length = sqrt(pow((handPosition_mov_x.at(handPosition_mov_x.size()-1)-handPosition_mov_x.at(0)),2)+
                                 pow((handPosition_mov_y.at(handPosition_mov_y.size()-1)-handPosition_mov_y.at(0)),2)+
                                 pow((handPosition_mov_z.at(handPosition_mov_z.size()-1)-handPosition_mov_z.at(0)),2));
            double total_cost_jerk_hand=0.0;
            for(size_t i=0;i<tot_timesteps.size();++i){
                total_cost_jerk_hand += pow(jerk_hand.at(i),2)*tot_timesteps.at(i);
            }
            total_cost_jerk_hand = sqrt(0.5*total_cost_jerk_hand*(pow(duration,5)/pow(length,2)));
            this->ui.label_cost_hand_ctrl_value->setText(QString::number(total_cost_jerk_hand));
            this->njs_mov_ctrl = total_cost_jerk_hand;

            // -- compute the number of movement units -- //
            QVector<double> sim_time_q = QVector<double>::fromStdVector(this->sim_time);
            this->nmu_mov_ctrl = this->getNumberMovementUnits(this->handVelocityNorm_ctrl,sim_time_q);
            this->ui.label_nmu_hand_value_ctrl->setText(QString::number(this->nmu_mov_ctrl));
        }
    }
}

void MainWindow::on_pushButton_control_plot_clicked()
{
    int n_samples = this->ui.lineEdit_n_samples_value->text().toInt();

    // plot the 3D hand position   
    int inc_pos = round(this->handPosition_ctrl.size()/n_samples);
    vector<vector<double>> hand_pos(n_samples);
    for(size_t i=0, j = 0; (i < this->handPosition_ctrl.size() && j < n_samples); i += inc_pos,++j)
    {
        hand_pos.at(j) = this->handPosition_ctrl.at(i);
    }

    this->handPosPlot_ctrl_ptr.reset(new HandPosPlot(hand_pos));
    this->handPosPlot_ctrl_ptr->setParent(this->ui.plot_control_hand_pos);
    this->handPosPlot_ctrl_ptr->resize(522,329);
    this->handPosPlot_ctrl_ptr->show();

    // plot the hand velocity norm
    double f_th = this->ui.lineEdit_f_cutoff->text().toDouble();
    double timestep = this->ui.lineEdit_time_step->text().toDouble();
    LowPassFilter lpf_hand_vel(f_th, timestep);

    this->handVelocityNorm_ctrl_plot.clear();
    if(!this->handVelocityNorm_ctrl.empty()){
        for(int k=0;k<this->handVelocityNorm_ctrl.size();++k){
            this->handVelocityNorm_ctrl_plot.push_back(lpf_hand_vel.update(this->handVelocityNorm_ctrl.at(k)));
        }
        QVector<double> qtime = QVector<double>::fromStdVector(this->sim_time);
        ui.plot_control_hand_vel->plotLayout()->clear();
        ui.plot_control_hand_vel->clearGraphs();
        ui.plot_control_hand_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_control_hand_vel);
        wideAxisRect->setupFullAxesBox(true);
        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_control_hand_vel);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_control_hand_vel->axisRects())
        {
          for (QCPAxis *axis : rect->axes())
          {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
          }
        }
        QString title("Hand velocity");
        ui.plot_control_hand_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_control_hand_vel,title));
        ui.plot_control_hand_vel->plotLayout()->addElement(1, 0, wideAxisRect);

        ui.plot_control_hand_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_control_hand_vel->graph(0)->setPen(QPen(Qt::red));
        ui.plot_control_hand_vel->graph(0)->setName(title);
        ui.plot_control_hand_vel->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_control_hand_vel->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_control_hand_vel->graph(0)->setData(qtime, this->handVelocityNorm_ctrl_plot);
        ui.plot_control_hand_vel->graph(0)->valueAxis()->setRange(*std::min_element(this->handVelocityNorm_ctrl_plot.begin(), this->handVelocityNorm_ctrl_plot.end()),
                                                          *std::max_element(this->handVelocityNorm_ctrl_plot.begin(), this->handVelocityNorm_ctrl_plot.end()));
        ui.plot_control_hand_vel->graph(0)->rescaleAxes();
        ui.plot_control_hand_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_control_hand_vel->replot();
    }else{
        ui.plot_control_hand_vel->plotLayout()->clear();
        ui.plot_control_hand_vel->clearGraphs();
    }

}

void MainWindow::on_pushButton_save_ctrl_params_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file with the controlling parameters"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "ctrl Files (*.ctrl)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        stream << "### Parameters of controlling ###" << endl;
        stream << "# Joints limits parameters #" << endl;
        if (this->ui.checkBox_joints_limits_av->isChecked()){ stream << "Jlim_av=true"<< endl;}else{stream << "Jlim_av=false"<< endl;}
        stream << "Jlim_th=" << this->ui.lineEdit_jlim_th->text().toStdString().c_str() << endl;
        stream << "Jlim_rate=" << this->ui.lineEdit_jlim_rate->text().toStdString().c_str() << endl;
        stream << "Jlim_coeff=" << this->ui.lineEdit_jlim_coeff->text().toStdString().c_str() << endl;
        stream << "Jlim_damping=" << this->ui.lineEdit_jlim_damping->text().toStdString().c_str() << endl;
        stream << "# Singularities avoidance parameters #" << endl;
        if (this->ui.checkBox_sing_av->isChecked()){ stream << "Sing_av=true"<< endl;}else{stream << "Sing_av=false"<< endl;}
        stream << "Sing_coeff=" << this->ui.lineEdit_sing_coeff->text().toStdString().c_str() << endl;
        stream << "Sing_damping=" << this->ui.lineEdit_sing_damping->text().toStdString().c_str() << endl;
        stream << "# Obstacle avoidance parameters #" << endl;
        if (this->ui.checkBox_obsts_av->isChecked()){ stream << "Obsts_av=true"<< endl;}else{stream << "Obsts_av=false"<< endl;}
        stream << "Obsts_coeff=" << this->ui.lineEdit_obsts_coeff->text().toStdString().c_str() << endl;
        stream << "Obsts_damping=" << this->ui.lineEdit_obsts_damping->text().toStdString().c_str() << endl;
        stream << "Obsts_coeff_torso=" << this->ui.lineEdit_obsts_coeff_torso->text().toStdString().c_str() << endl;
        stream << "Obsts_damping_torso=" << this->ui.lineEdit_obsts_damping_torso->text().toStdString().c_str() << endl;
        if (this->ui.checkBox_obsts_noise->isChecked()){ stream << "Obsts_noise=true"<< endl;}else{stream << "Obsts_noise=false"<< endl;}
        if (this->ui.checkBox_obsts_filter_noise->isChecked()){ stream << "Obsts_filter_noise=true"<< endl;}else{stream << "Obsts_filter_noise=false"<< endl;}
        stream << "Obsts_f_cut_off=" << this->ui.lineEdit_obsts_f_cutoff->text().toStdString().c_str() << endl;
        stream << "Obsts_f_timestep=" << this->ui.lineEdit_obsts_timestep->text().toStdString().c_str() << endl;
        if (this->ui.checkBox_use_plan_hand_pos->isChecked()){ stream << "use_plan_hand_pos=true"<< endl;}else{stream << "use_plan_hand_pos=false"<< endl;}
        if (this->ui.checkBox_use_vel_control->isChecked()){ stream << "use_vel_control=true"<< endl;}else{stream << "use_vel_control=false"<< endl;}
        stream << "# Human-likeness addition parameters #" << endl;
        if (this->ui.checkBox_hl_add->isChecked()){ stream << "Hl_add=true"<< endl;}else{stream << "Hl_add=false"<< endl;}
        stream << "Hl_add_p_pos_coeff_plan=" << this->ui.lineEdit_hl_p_pos_coeff_plan->text().toStdString().c_str() << endl;
        stream << "Hl_add_p_or_coeff_plan=" << this->ui.lineEdit_hl_p_or_coeff_plan->text().toStdString().c_str() << endl;
        stream << "Hl_add_p_pos_coeff_app=" << this->ui.lineEdit_hl_p_pos_coeff_app->text().toStdString().c_str() << endl;
        stream << "Hl_add_p_or_coeff_app=" << this->ui.lineEdit_hl_p_or_coeff_app->text().toStdString().c_str() << endl;
        stream << "Hl_add_p_pos_coeff_ret=" << this->ui.lineEdit_hl_p_pos_coeff_ret->text().toStdString().c_str() << endl;
        stream << "Hl_add_p_or_coeff_ret=" << this->ui.lineEdit_hl_p_or_coeff_ret->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_pos_coeff_plan=" << this->ui.lineEdit_hl_d_pos_coeff_plan->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_or_coeff_plan=" << this->ui.lineEdit_hl_d_or_coeff_plan->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_pos_coeff_app=" << this->ui.lineEdit_hl_d_pos_coeff_app->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_or_coeff_app=" << this->ui.lineEdit_hl_d_or_coeff_app->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_pos_coeff_ret=" << this->ui.lineEdit_hl_d_pos_coeff_ret->text().toStdString().c_str() << endl;
        stream << "Hl_add_d_or_coeff_ret=" << this->ui.lineEdit_hl_d_or_coeff_ret->text().toStdString().c_str() << endl;
        stream << "Hl_add_g_th_pa=" << this->ui.lineEdit_g_th_plan_app->text().toStdString().c_str() << endl;
        stream << "Hl_add_g_th_rp=" << this->ui.lineEdit_g_th_ret_plan->text().toStdString().c_str() << endl;
        stream << "Hl_fing_p_coeff=" << this->ui.lineEdit_fing_p_coeff->text().toStdString().c_str() << endl;
        stream << "Hl_fing_d_coeff=" << this->ui.lineEdit_fing_d_coeff->text().toStdString().c_str() << endl;
        stream << "# Control coefficients and error threshold #" << endl;
        stream << "Pos_p_control_coeff=" << this->ui.lineEdit_coeff_p_pos->text().toStdString().c_str() << endl;
        stream << "Pos_p_error_th=" << this->ui.lineEdit_err_p_pos->text().toStdString().c_str() << endl;
        stream << "Or_p_control_coeff=" << this->ui.lineEdit_coeff_p_or->text().toStdString().c_str() << endl;
        stream << "Or_p_error_th=" << this->ui.lineEdit_err_p_or->text().toStdString().c_str() << endl;
        stream << "Pos_d_control_coeff=" << this->ui.lineEdit_coeff_d_pos->text().toStdString().c_str() << endl;
        stream << "Pos_d_error_th=" << this->ui.lineEdit_err_d_pos->text().toStdString().c_str() << endl;
        stream << "Or_d_control_coeff=" << this->ui.lineEdit_coeff_d_or->text().toStdString().c_str() << endl;
        stream << "Or_d_error_th=" << this->ui.lineEdit_err_d_or->text().toStdString().c_str() << endl;
        stream << "t_f_trap=" << this->ui.lineEdit_t_f_trap->text().toStdString().c_str() << endl;
        if (this->ui.checkBox_joints_velocity_ctrl->isChecked()){ stream << "joints_vel_control=true"<< endl;}else{stream << "joints_vel_control=false"<< endl;}
        stream << "# Noise filtering #" << endl;
        if (this->ui.radioButton_N_5->isChecked()){stream << "N_5=true"<< endl;}else{stream << "N_5=false"<< endl;}
        if (this->ui.radioButton_N_7->isChecked()){stream << "N_7=true"<< endl;}else{stream << "N_7=false"<< endl;}
        if (this->ui.radioButton_N_9->isChecked()){stream << "N_9=true"<< endl;}else{stream << "N_9=false"<< endl;}
        if (this->ui.radioButton_N_11->isChecked()){stream << "N_11=true"<< endl;}else{stream << "N_11=false"<< endl;}
        if (this->ui.radioButton_N_19->isChecked()){stream << "N_19=true"<< endl;}else{stream << "N_19=false"<< endl;}
        if (this->ui.radioButton_N_25->isChecked()){stream << "N_25=true"<< endl;}else{stream << "N_25=false"<< endl;}
        if (this->ui.radioButton_N_35->isChecked()){stream << "N_35=true"<< endl;}else{stream << "N_35=false"<< endl;}
        if (this->ui.radioButton_N_45->isChecked()){stream << "N_45=true"<< endl;}else{stream << "N_45=false"<< endl;}
        stream << "freq_cutoff_joint_pos=" << this->ui.lineEdit_cutoff_freq_joint_pos->text().toStdString().c_str() << endl;
        stream << "timestep_joint_pos=" << this->ui.lineEdit_timestep_joint_pos->text().toStdString().c_str() << endl;
        if (this->ui.checkBox_tar_noise->isChecked()){ stream << "Tar_noise=true"<< endl;}else{stream << "Tar_noise=false"<< endl;}
        if (this->ui.checkBox_tar_filter_noise->isChecked()){ stream << "Tar_filter_noise=true"<< endl;}else{stream << "Tar_filter_noise=false"<< endl;}
        stream << "Tar_f_cut_off=" << this->ui.lineEdit_tar_f_cutoff->text().toStdString().c_str() << endl;
        stream << "Tar_f_timestep=" << this->ui.lineEdit_tar_timestep->text().toStdString().c_str() << endl;
        if (this->ui.checkBox_follow_target->isChecked()){ stream << "follow_target=true"<< endl;}else{stream << "follow_target=false"<< endl;}
        stream << "# Maximum allowed velocity of the joints #" << endl;
        stream << "vel_max=" << this->ui.lineEdit_vel_max->text().toStdString().c_str() << endl;
        stream << "# Replanning #" << endl;
        stream << "Swivel_angle_th=" << this->ui.lineEdit_swivel_angle_th->text().toStdString().c_str() << endl;
        stream << "g_map_th_max=" << this->ui.lineEdit_g_th_max_replan->text().toStdString().c_str() << endl;
        stream << "g_map_th_min=" << this->ui.lineEdit_g_th_min_replan->text().toStdString().c_str() << endl;
        f.close();
    }
}

void MainWindow::on_pushButton_load_ctrl_params_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of warm start settings"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "ctrl Files (*.ctrl)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){
        QTextStream stream( &f );
        QString line;
        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)!=QChar('#')){
                QStringList fields = line.split("=");
                if (QString::compare(fields.at(0),QString("Jlim_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_joints_limits_av->setChecked(true);
                        this->ui.groupBox_jlim_params->setEnabled(true);
                    }else{
                        this->ui.checkBox_joints_limits_av->setChecked(false);
                        this->ui.groupBox_jlim_params->setEnabled(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Jlim_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_jlim_th->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Jlim_rate"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_jlim_rate->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Jlim_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_jlim_coeff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Jlim_damping"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_jlim_damping->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Sing_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_sing_av->setChecked(true);
                        this->ui.groupBox_sing_av_params->setEnabled(true);
                    }else{
                        this->ui.checkBox_sing_av->setChecked(false);
                        this->ui.groupBox_sing_av_params->setEnabled(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Sing_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_sing_coeff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Sing_damping"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_sing_damping->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_obsts_av->setChecked(true);
                        this->ui.groupBox_obsts_av_params->setEnabled(true);
                    }else{
                        this->ui.checkBox_obsts_av->setChecked(false);
                        this->ui.groupBox_obsts_av_params->setEnabled(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Obsts_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_coeff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_damping"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_damping->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_coeff_torso"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_coeff_torso->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_damping_torso"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_damping_torso->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_noise"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_obsts_noise->setChecked(true);
                    }else{
                        this->ui.checkBox_obsts_noise->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Obsts_filter_noise"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_obsts_filter_noise->setChecked(true);
                    }else{
                        this->ui.checkBox_obsts_filter_noise->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Obsts_f_cut_off"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_f_cutoff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Obsts_f_timestep"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_obsts_timestep->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("use_plan_hand_pos"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_use_plan_hand_pos->setChecked(true);
                    }else{
                        this->ui.checkBox_use_plan_hand_pos->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("use_vel_control"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_use_vel_control->setChecked(true);
                    }else{
                        this->ui.checkBox_use_vel_control->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Hl_add"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_hl_add->setChecked(true);
                        this->ui.groupBox_hl_add_params->setEnabled(true);
                    }else{
                        this->ui.checkBox_hl_add->setChecked(false);
                        this->ui.groupBox_hl_add_params->setEnabled(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_pos_coeff_plan"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_pos_coeff_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_or_coeff_plan"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_or_coeff_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_pos_coeff_app"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_pos_coeff_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_or_coeff_app"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_or_coeff_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_pos_coeff_ret"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_pos_coeff_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_p_or_coeff_ret"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_p_or_coeff_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_pos_coeff_plan"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_pos_coeff_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_or_coeff_plan"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_or_coeff_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_pos_coeff_app"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_pos_coeff_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_or_coeff_app"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_or_coeff_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_pos_coeff_ret"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_pos_coeff_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_d_or_coeff_ret"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_hl_d_or_coeff_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_g_th_pa"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_g_th_plan_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_add_g_th_rp"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_g_th_ret_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_fing_p_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_fing_p_coeff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hl_fing_d_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_fing_d_coeff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Pos_p_control_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_coeff_p_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Pos_p_error_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_err_p_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Or_p_control_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_coeff_p_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Or_p_error_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_err_p_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Pos_d_control_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_coeff_d_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Pos_d_error_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_err_d_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Or_d_control_coeff"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_coeff_d_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Or_d_error_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_err_d_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("vel_max"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_vel_max->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("t_f_trap"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_t_f_trap->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("N_5"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_5->setChecked(true);
                    }else{
                        this->ui.radioButton_N_5->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_7"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_7->setChecked(true);
                    }else{
                        this->ui.radioButton_N_7->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_9"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_9->setChecked(true);
                    }else{
                        this->ui.radioButton_N_9->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_11"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_11->setChecked(true);
                    }else{
                        this->ui.radioButton_N_11->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_19"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_19->setChecked(true);
                    }else{
                        this->ui.radioButton_N_19->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_25"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_25->setChecked(true);
                    }else{
                        this->ui.radioButton_N_25->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_35"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_35->setChecked(true);
                    }else{
                        this->ui.radioButton_N_35->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("N_45"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.radioButton_N_45->setChecked(true);
                    }else{
                        this->ui.radioButton_N_45->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("freq_cutoff_joint_pos"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_cutoff_freq_joint_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("timestep_joint_pos"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_timestep_joint_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("joints_vel_control"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_joints_velocity_ctrl->setChecked(true);
                    }else{
                        this->ui.checkBox_joints_velocity_ctrl->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Tar_noise"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_tar_noise->setChecked(true);
                    }else{
                        this->ui.checkBox_tar_noise->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Tar_filter_noise"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_tar_filter_noise->setChecked(true);
                    }else{
                        this->ui.checkBox_tar_filter_noise->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Tar_f_cut_off"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_tar_f_cutoff->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Tar_f_timestep"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_tar_timestep->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("follow_target"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("true\n"),Qt::CaseInsensitive)==0){
                        this->ui.checkBox_follow_target->setChecked(true);
                    }else{
                        this->ui.checkBox_follow_target->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("Swivel_angle_th"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_swivel_angle_th->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("g_map_th_max"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_g_th_max_replan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("g_map_th_min"),Qt::CaseInsensitive)==0){
                    this->ui.lineEdit_g_th_min_replan->setText(fields.at(1));
                }
            }
        }// while
        f.close();
    }
}

void MainWindow::on_pushButton_control_save_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }

    QString path("results/controlling/");
    ui.plot_control_hand_vel->savePdf(path+QString("hand_vel.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));

    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");
    string hand_pos_file = path.toStdString()+string("hand_pos.pdf");
    IO::save(this->handPosPlot_ctrl_ptr.get(), hand_pos_file.c_str(),  "PDF" );

    // results
    string filename("results_mov_ctrl.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# NORMALIZED JERK SCORE \n");
    string njs_str =  boost::str(boost::format("%.2f") % (this->njs_mov_ctrl));
    boost::replace_all(njs_str,",",".");
    results << string("njs =")+njs_str+string(";\n");

    results << string("# NUMBER OF MOVEMENT UNITS \n");
    string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_mov_ctrl));
    boost::replace_all(nmu_str,",",".");
    results << string("nmu =")+nmu_str+string(";\n");


    results.close();


    // hand position
    if(!this->handPosition_ctrl.empty()){
        string filename_hand_pos("hand_pos.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_ctrl.size();++i){
            vector<double> point = this->handPosition_ctrl.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // hand velocity
    if(!this->handVelocityNorm_ctrl_plot.empty()){
        string filename_hand_vel("hand_vel.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_ctrl_plot.size();++i){
            double vel = this->handVelocityNorm_ctrl_plot.at(i);
            double time = this->sim_time.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_vel.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

}

void MainWindow::on_radioButton_N_5_clicked()
{
    this->N_filter_length=5;
}

void MainWindow::on_radioButton_N_7_clicked()
{
    this->N_filter_length=7;
}

void MainWindow::on_radioButton_N_9_clicked()
{
    this->N_filter_length=9;
}

void MainWindow::on_radioButton_N_11_clicked()
{
    this->N_filter_length=11;
}

void MainWindow::on_radioButton_N_19_clicked()
{
    this->N_filter_length=19;
}

void MainWindow::on_radioButton_N_25_clicked()
{
    this->N_filter_length=25;
}

void MainWindow::on_radioButton_N_35_clicked()
{
    this->N_filter_length=35;
}

void MainWindow::on_radioButton_N_45_clicked()
{
    this->N_filter_length=45;
}

void MainWindow::on_radioButton_N_55_clicked()
{
    this->N_filter_length=55;
}


}  // namespace motion_manager

