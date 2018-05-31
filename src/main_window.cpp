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

namespace motion_manager {

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

#endif

    for (size_t i=0; i< scenarios.size();++i){
        ui.listWidget_scenario->addItem(scenarios.at(i));
    }



}

MainWindow::~MainWindow()
{


}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


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
    if (c){
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
             // Toy vehicle bi-manual manipulation
             string path_vrep_toyscene_aros_dual_arm_cols = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_dual_arm_cols.ttt");

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
             case 4: // Challenging scenario
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

        }else{
            // dual arm
            bool success=true;
            int mov_id_r = ui.comboBox_mov->currentIndex();
            int mov_id_l = ui.comboBox_mov_left->currentIndex();

            objectPtr obj_right; bool prec_right;
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
            }else{
                // right arm go-park movement and reaching movements
            }
            objectPtr obj_left; bool prec_left;
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
            }else{
                // left arm go-park movement and reaching movements
            }

            if(success){

                if(planner_id==0){
                   // HUMP
                   curr_task->addProblem(new Problem(planner_id,new Movement(mov_id_r,mov_id_l,0,obj_right,prec_right, obj_left,prec_left),new Scenario(*(this->curr_scene.get()))));
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
    HUMotion::hump_params  tols;
    HUMotion::hump_dual_params  dual_tols;
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
                    std::vector<double> timesteps_stage_aux;
                    for(size_t i=0; i<h_results->trajectory_stages.size();++i){
                        timesteps_stage_aux.clear();
                        double t_stage = h_results->time_steps.at(i);
                        MatrixXd traj_stage = h_results->trajectory_stages.at(i);
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
    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
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
        this->handPosition_mov.resize(tot_steps); this->handVelocityNorm_mov.resize(tot_steps);
        this->handLinearVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov.resize(tot_steps);
        // wrist
        this->wristVelocityNorm_mov.resize(tot_steps);
        this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
        // elbow
        this->elbowVelocityNorm_mov.resize(tot_steps);
        this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
        //shoulder
        this->shoulderVelocityNorm_mov.resize(tot_steps);
        this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);

        int step = 0;
        int arm_code = prob->getMovement()->getArm();
        for (size_t k=0; k< this->jointsPosition_mov.size();++k){
            MatrixXd pos_stage = this->jointsPosition_mov.at(k);
            MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
            MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
            for(int i=0;i<pos_stage.rows();++i){
                // position
                VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> posture; posture.resize(pos_row.size());
                VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
                this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_mov.at(step),posture);
                // velocities
                VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> velocities; velocities.resize(vel_row.size());
                VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                // hand velocity
                this->handVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
                vector<double> hand_vel; this->curr_scene->getHumanoid()->getHandVel(arm_code,hand_vel,posture,velocities);
                this->handLinearVelocity_mov.at(step) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
                this->handAngularVelocity_mov.at(step) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
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
            }
        }
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

        // right -- compute the hand values, positions and accelerations
        // hand
        this->handPosition_mov.resize(tot_steps); this->handVelocityNorm_mov.resize(tot_steps);
        this->handLinearVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov.resize(tot_steps);
        // wrist
        this->wristVelocityNorm_mov.resize(tot_steps);
        this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
        // elbow
        this->elbowVelocityNorm_mov.resize(tot_steps);
        this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
        //shoulder
        this->shoulderVelocityNorm_mov.resize(tot_steps);
        this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);

        // left -- compute the hand values, positions and accelerations
        // hand
        this->handPosition_mov_left.resize(tot_steps); this->handVelocityNorm_mov_left.resize(tot_steps);
        this->handLinearVelocity_mov_left.resize(tot_steps); this->handAngularVelocity_mov_left.resize(tot_steps);
        // wrist
        this->wristVelocityNorm_mov_left.resize(tot_steps);
        this->wristLinearVelocity_mov_left.resize(tot_steps); this->wristAngularVelocity_mov_left.resize(tot_steps);
        // elbow
        this->elbowVelocityNorm_mov_left.resize(tot_steps);
        this->elbowLinearVelocity_mov_left.resize(tot_steps); this->elbowAngularVelocity_mov_left.resize(tot_steps);
        //shoulder
        this->shoulderVelocityNorm_mov_left.resize(tot_steps);
        this->shoulderLinearVelocity_mov_left.resize(tot_steps); this->shoulderAngularVelocity_mov_left.resize(tot_steps);

        int step = 0;
        //int l_init = JOINTS_ARM+JOINTS_HAND; int l_end = JOINTS_ARM+JOINTS_HAND+JOINTS_ARM;
        //int arm_code = prob->getMovement()->getArm();
        for (size_t k=0; k< this->jointsPosition_mov.size();++k){
            MatrixXd pos_stage = this->jointsPosition_mov.at(k);
            MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
            MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
            for(int i=0;i < pos_stage.rows();++i){
                // position
                VectorXd pos_row_right = pos_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> posture_right; posture_right.resize(pos_row_right.size());
                VectorXd::Map(&posture_right[0], pos_row_right.size()) = pos_row_right;
                this->curr_scene->getHumanoid()->getHandPos(1,this->handPosition_mov.at(step),posture_right);
                VectorXd pos_row_left = pos_stage.block<1,JOINTS_ARM>(i,JOINTS_ARM+JOINTS_HAND);
                vector<double> posture_left; posture_left.resize(pos_row_left.size());
                VectorXd::Map(&posture_left[0], pos_row_left.size()) = pos_row_left;
                this->curr_scene->getHumanoid()->getHandPos(2,this->handPosition_mov_left.at(step),posture_left);
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
            }
        }
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
    qnode.execMovement(this->jointsPosition_mov,this->jointsVelocity_mov,this->timesteps_mov, this->tols_stop_mov, this->traj_descr_mov, this->curr_mov, this->curr_scene);
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
    if(ui.checkBox_comp_exec->isChecked()){
        qnode.execTask_complete(this->jointsPosition_task,this->jointsVelocity_task,this->timesteps_task, this->tols_stop_task, this->traj_descr_task,this->curr_task, this->curr_scene);
    }else{
        qnode.execTask(this->jointsPosition_task,this->jointsVelocity_task,this->timesteps_task, this->tols_stop_task, this->traj_descr_task,this->curr_task, this->curr_scene);
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
                    //obj = this->curr_scene->getObject(obj_str.toStdString());
                    //obj_left = this->curr_scene->getObject(obj_left_str.toStdString());
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
                            //TO DO
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
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                    // get the pose
                    pose = this->curr_scene->getPose(pose_str.toStdString());
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,pose,prec),new Scenario(*(this->curr_scene.get()))));
                        }else{// dual-arm
                            // TO DO
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
                    obj_eng = this->curr_scene->getObject(obj_eng_str.toStdString());
                    switch (arm_code){
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
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get()))));
                        }else{ // dual-arm
                            // TO DO
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
                }else if(QString::compare(mov_type,QString("Go park"),Qt::CaseInsensitive)==0){
                    mov_id=5;
                    problemPtr prob;
                    if(plan_id==0){
                        if(arm_code!=0){//single-arm
                            prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));
                        }else{//dual-arm
                            // TO DO
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
    string path_vrep_touscene_aros_dual = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_dual_arm_cols.ttt");

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
        path = path_vrep_touscene_aros_dual;
        title = string("Assembly scenario: the Toy vehicle with ARoS swaps columns");
        success = string("Assembly scenario: the Toy vehicle with ARoS swaps columns HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with ARoS swaps columns HAS NOT BEEN LOADED");
        break;
    case 7:
        //TO DO
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

void MainWindow::on_pushButton_comp_vel_mov_clicked()
{
    if(!this->shoulderLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->shoulderLinearVelocity_mov,this->shoulderAngularVelocity_mov,this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->elbowLinearVelocity_mov,this->elbowAngularVelocity_mov,this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wristLinearVelocity_mov,this->wristAngularVelocity_mov,this->qtime_mov,2);
    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handLinearVelocity_mov,this->handAngularVelocity_mov,this->qtime_mov,3);

    this->mCompVeldlg->setDual(false); this->mCompVeldlg->setRight(true);
    this->mCompVeldlg->show();
}

void MainWindow::on_pushButton_comp_vel_mov_right_clicked()
{
    if(!this->shoulderLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->shoulderLinearVelocity_mov,this->shoulderAngularVelocity_mov,this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->elbowLinearVelocity_mov,this->elbowAngularVelocity_mov,this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wristLinearVelocity_mov,this->wristAngularVelocity_mov,this->qtime_mov,2);
    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handLinearVelocity_mov,this->handAngularVelocity_mov,this->qtime_mov,3);

    this->mCompVeldlg->setDual(true); this->mCompVeldlg->setRight(true);
    this->mCompVeldlg->show();
}

void MainWindow::on_pushButton_comp_vel_mov_left_clicked()
{
    if(!this->shoulderLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->shoulderLinearVelocity_mov_left,this->shoulderAngularVelocity_mov_left,this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->elbowLinearVelocity_mov_left,this->elbowAngularVelocity_mov_left,this->qtime_mov,1);
    if(!this->wristLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->wristLinearVelocity_mov_left,this->wristAngularVelocity_mov_left,this->qtime_mov,2);
    if(!this->handLinearVelocity_mov_left.empty())
        this->mCompVeldlg->setupPlots(this->handLinearVelocity_mov_left,this->handAngularVelocity_mov_left,this->qtime_mov,3);

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
                  step_value=0.01;
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



}  // namespace motion_manager

