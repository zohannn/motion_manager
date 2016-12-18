/**
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

    // create RViz Communication dialog
    mrvizCommdlg = new RVizCommDialog(&qnode, this);
    mrvizCommdlg->setModal(true);

    //create HUML Tuning dialog
    mTolHumldlg = new TolDialogHUML(this);
    mTolHumldlg->setModal(true);

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

    // create the Hand velocity components dialog
    mHandVeldlg = new HandVelocityDialog(this);
    mHandVeldlg->setModal(false);


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

    // RViz connectedsignal
    QObject::connect(mrvizCommdlg, SIGNAL(rvizConnected(bool)), this, SLOT(updateRVizStatus(bool)));


    // new element in the scenario signal
    QObject::connect(&qnode, SIGNAL(newElement(string)),this,SLOT(addElement(string)));
    QObject::connect(&qnode, SIGNAL(newObject(string)),this,SLOT(addObject(string)));

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

    // scenarios
#if HAND == 0
    ui.listWidget_scenario->addItem(QString("Assembly scenario: Toy vehicle with Jarde"));
    //ui.listWidget_scenario->addItem(QString("Assistive scenario: beverages with Jarde"));
    //ui.listWidget_scenario->addItem(QString("Organizing scenario: shelfs and objects with Jarde"));

#elif HAND == 1

    ui.listWidget_scenario->addItem(QString("Assembly scenario: Toy vehicle with ARoS"));
    ui.listWidget_scenario->addItem(QString("Empty scenario: empty scenario with ARoS"));
    //ui.listWidget_scenario->addItem(QString("Assistive scenario: beverages with ARoS"));
    //ui.listWidget_scenario->addItem(QString("Organizing scenario: shelfs and objects with ARoS"));


#endif



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
        ui.actionRViz_Communication->setEnabled(true);
        ui.labelStatusRViz->setEnabled(true);
        ui.labelRVizComm->setEnabled(true);
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
    //ui.pushButton_loadScenario->setEnabled(false);
    //ui.groupBox_getElements->setEnabled(false);
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
    QMessageBox::about(this, tr("About the motion manager"),tr("<h2>motion_manager version 0.10</h2><p>Copyright: Gianpaolo Gulletta</p><p>The motion manager is a ROS package."
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

void MainWindow::on_actionRViz_Communication_triggered()
{

    mrvizCommdlg->show();
}


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::on_pushButton_tuning_clicked()
{

    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    switch(planner_id){
    case 0: // HUML
        mTolHumldlg->show();
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

    // scenarios
    QVector<QString> scenarios;
        //0 --> Toy vehicle scenario
        //1 --> Empty scenario
        //2 --> Beverages scenario
        //3 --> shelfs and objects scenario
#if HAND == 0

    scenarios.push_back(QString("Assembly scenario: Toy vehicle with Jarde"));
    //scenarios.push_back(QString("Assistive scenario: beverages with Jarde"));
    //scenarios.push_back(QString("Organizing scenario: shelfs and objects with Jarde"));

#elif HAND == 1

    scenarios.push_back(QString("Assembly scenario: Toy vehicle with ARoS"));
    scenarios.push_back(QString("Empty scenario: empty scenario with ARoS"));
    //scenarios.push_back(QString("Assistive scenario: beverages with ARoS"));
    //scenarios.push_back(QString("Organizing scenario: shelfs and objects with ARoS"));

#endif

    //this->scenario_id = ui.listWidget_scenario->currentRow();
    QString scenario_text = ui.listWidget_scenario->currentItem()->text();
    int equal;
    for(int i=0; i<scenarios.size();++i){
         equal = scenario_text.compare(scenarios.at(i));
         if(equal==0){
             // Empty scenario with ARoS
             string path_vrep_emptyscene_aros = PATH_SCENARIOS+string("/vrep/empty_aros.ttt");
             // Toy vehicle scenario with ARoS
             string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros.ttt");
             //string path_rviz_toyscene_aros = PATH_SCENARIOS+string("/rviz/toy_vehicle_aros.scene");
             // Toy vehicle scenario with Jarde
             string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
             //string path_rviz_toyscene_jarde = PATH_SCENARIOS+string("/rviz/toy_vehicle_jarde.scene");
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
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));

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
                     this->m_planner.reset(new moveit_planning::HumanoidPlanner(title));

                 }else{
                     qnode.log(QNode::Error,std::string("Empty scenario: empty scenario with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                     ui.groupBox_getElements->setEnabled(false);
                     ui.groupBox_homePosture->setEnabled(false);
                     ui.pushButton_loadScenario->setEnabled(true);
                 }

#endif
                 break;
             case 2:// Assistive scenario: beverages
                 //TO DO
                 break;
             case 3: // Organizing scenario: shelfs ad objects
                 //TO DO
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
    ui.tableWidget_sol_mov->clear();//ui.listWidget_sol_mov->clear();
    ui.label_totalTime_value_mov->clear();
}



void MainWindow::on_pushButton_getElements_clicked()
{


    try{
        if (qnode.getElements(init_scene)){

            this->curr_scene = scenarioPtr(new Scenario(*(this->init_scene.get()))); //update the current scenario
            this->curr_task = taskPtr(new Task());
            ui.pushButton_getElements->setEnabled(false);
            ui.tab_plan->setEnabled(true);
            ui.tab_results->setEnabled(true);
            ui.groupBox_specs->setEnabled(true);
            ui.groupBox_task->setEnabled(false);
            ui.tabWidget_sol->setEnabled(false);
            // load the objects into RViz
            std::vector<objectPtr> objs; this->curr_scene->getObjects(objs);
            qnode.loadRVizScenario(objs);
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

             if (ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled()){
                 // engage, disengage movements
                string obj_name = ui.comboBox_objects->currentText().toStdString();
                string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString();

                objectPtr obj = curr_scene->getObject(obj_name);
                objectPtr obj_eng = curr_scene->getObject(obj_eng_name);

                if(obj!=NULL && obj_eng!=NULL){
                    int grip_id = ui.comboBox_grip->currentIndex();
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
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
                    }else{
                       curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,grip_id,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
                    }
                    success=true;

                }else{qnode.log(QNode::Error,std::string("The movement requires two objects"));}

             }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled()){
                  // reach-to- grasp movement, transport movements
                 string obj_name = ui.comboBox_objects->currentText().toStdString();
                 objectPtr obj = curr_scene->getObject(obj_name);

                 if(obj!=NULL){
                     int grip_id = ui.comboBox_grip->currentIndex();
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
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
                     }else{
                        curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,grip_id,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
                     }
                     success=true;

                 }else{qnode.log(QNode::Error,std::string("The movement requires an object"));}
             }else{
                 // go-park and reaching movements
                 std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);
                 for(size_t i=0;i<objects.size();++i){
                     this->curr_scene->getObject(i)->setTargetRightEnabled(false);
                     this->curr_scene->getObject(i)->setTargetLeftEnabled(false);
                 }
                 if(planner_id==0){
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));
                }else{
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get())),this->m_planner));
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


        }
} // add movement


void MainWindow::on_pushButton_plan_clicked()
{

    ui.tabWidget_sol->setCurrentIndex(0);    
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    HUMotion::huml_params  tols;
    std::vector<double> move_target;
    std::vector<double> move_final_hand;
    std::vector<double> move_final_arm;
    bool use_final;
    moveit_planning::moveit_params m_params;
    bool moveit_plan = false;

    bool solved = false;
 try{
    switch(planner_id){

    case 0: // HUML
        moveit_plan = false;
        mTolHumldlg->setInfo(prob->getInfoLine());
        // --- Tolerances for the final posture selection ---- //
        tols.tolTarPos = mTolHumldlg->getTolTarPos(); // target position tolerances
        tols.tolTarOr = mTolHumldlg->getTolTarOr(); // target orientation tolerances
        mTolHumldlg->getTolsArm(tols.tolsArm);// tolerances of the arm : radius in [mm]
        mTolHumldlg->getTolsHand(tols.tolsHand);// tolerances of the hand: radius in [mm]
        tols.target_avoidance = mTolHumldlg->getTargetAvoidance();// target avoidance
        tols.obstacle_avoidance = mTolHumldlg->getObstacleAvoidance(); //obstacle avoidance
        mTolHumldlg->getLambda(tols.lambda_final); // joint expense factors
        mTolHumldlg->getLambda(tols.lambda_bounce); // joint expense factors
        // --- Tolerances for the bounce posture selection ---- //
        tols.w_max = std::vector<double>(tols.lambda_final.size(),mTolHumldlg->getWMax()); // max joint velocity
        mTolHumldlg->getInitVel(tols.bounds.vel_0); // initial velocity
        mTolHumldlg->getFinalVel(tols.bounds.vel_f); // final velocity
        mTolHumldlg->getInitAcc(tols.bounds.acc_0); // initial acceleration
        mTolHumldlg->getFinalAcc(tols.bounds.acc_f); // final acceleration
        mTolHumldlg->getVelApproach(tols.vel_approach); // velocity approach
        mTolHumldlg->getAccApproach(tols.acc_approach); // acceleration approach
        // tolerances for the obstacles
        mTolHumldlg->getTolsObstacles(tols.final_tolsObstacles); // final posture tols
        tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
        tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1));
        mTolHumldlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
        mTolHumldlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));
        // tolerances for the target
        tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
        tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
        tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
        mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
        mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(1));
        mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(2));
        tols.steps = mTolHumldlg->getSteps();// number of steps
        tols.totalTime = 1.0; // 1 = plan the 100% of the movement
        // pick / place settings
        tols.mov_specs.approach = mTolHumldlg->getApproach();
        tols.mov_specs.retreat = mTolHumldlg->getRetreat();
        mTolHumldlg->getPreGraspApproach(tols.mov_specs.pre_grasp_approach); // pick approach
        mTolHumldlg->getPostGraspRetreat(tols.mov_specs.post_grasp_retreat); // pick retreat
        mTolHumldlg->getPrePlaceApproach(tols.mov_specs.pre_place_approach); // place approach
        mTolHumldlg->getPostPlaceRetreat(tols.mov_specs.post_place_retreat); // place retreat
        tols.mov_specs.rand_init = mTolHumldlg->getRandInit(); // random initialization for "plan" stages
        // move settings
        mTolHumldlg->getTargetMove(move_target);
        mTolHumldlg->getFinalHand(move_final_hand);
        mTolHumldlg->getFinalArm(move_final_arm);
        use_final = mTolHumldlg->get_use_final_posture();
        prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
        tols.mov_specs.use_move_plane = mTolHumldlg->get_add_plane();
        mTolHumldlg->getPlaneParameters(tols.mov_specs.plane_params);

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
                std::vector<double> timesteps_stage_aux;
                for(size_t i=0; i<h_results->trajectory_stages.size();++i){
                    timesteps_stage_aux.clear();
                    double t_stage = h_results->time_steps.at(i);
                    MatrixXd traj_stage = h_results->trajectory_stages.at(i);
                    for(int j=0;j<traj_stage.rows();++j){
                        timesteps_stage_aux.push_back(t_stage);
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
        break;
    case 1: // RRT
        moveit_plan = true;
        mRRTdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTdlg->getConfig();
        // pick/place settings
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

        break;

    case 2: // RRT Connect
        moveit_plan = true;
        mRRTConnectdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTConnectdlg->getConfig();
        // pick/place settings
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

        break;

    case 3: // RRT star
        moveit_plan = true;
        mRRTstardlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTstardlg->getConfig();
        // pick/place settings
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

        break;

    case 4: // PRM
        moveit_plan = true;
        mPRMdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mPRMdlg->getConfig();
        // pick/place settings
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

        break;

    case 5: // PRM star
        moveit_plan = true;
        mPRMstardlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTdlg->getConfig();
        // pick/place settings
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

        break;

    } // switch planners

}catch (const std::string message){qnode.log(QNode::Error,std::string("Plan failure: ")+message);
}catch(const std::exception exc){qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());}

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
                std::vector<double> init_hand_pos;
                std::vector<double> init_hand_vel;
                std::vector<double> init_hand_acc;
                moveit_msgs::RobotTrajectory rob_traj_pre;
                vector<trajectory_msgs::JointTrajectoryPoint> points_pre;
                trajectory_msgs::JointTrajectoryPoint traj_pnt_pre;
                // positions of the fingers in the Barrett Hand
                int fing_base = 0; int fing_1 = 1; int fing_2 = 4; int fing_3 = 6;
                bool move;
                if(m_results->trajectory_stages.size()>1){
                    // pick and place movements
                    move=false;
                    rob_traj_pre = m_results->trajectory_stages.at(1);
                    points_pre = rob_traj_pre.joint_trajectory.points;
                    traj_pnt_pre = points_pre.at(0);
                    //positions
                    init_hand_pos.push_back(traj_pnt_pre.positions.at(fing_base));
                    init_hand_pos.push_back(traj_pnt_pre.positions.at(fing_1));
                    init_hand_pos.push_back(traj_pnt_pre.positions.at(fing_2));
                    init_hand_pos.push_back(traj_pnt_pre.positions.at(fing_3));
                    //velocities
                    init_hand_vel.push_back(traj_pnt_pre.velocities.at(fing_base));
                    init_hand_vel.push_back(traj_pnt_pre.velocities.at(fing_1));
                    init_hand_vel.push_back(traj_pnt_pre.velocities.at(fing_2));
                    init_hand_vel.push_back(traj_pnt_pre.velocities.at(fing_3));
                    //accelerations
                    init_hand_acc.push_back(traj_pnt_pre.accelerations.at(fing_base));
                    init_hand_acc.push_back(traj_pnt_pre.accelerations.at(fing_1));
                    init_hand_acc.push_back(traj_pnt_pre.accelerations.at(fing_2));
                    init_hand_acc.push_back(traj_pnt_pre.accelerations.at(fing_3));
                }else{move=true;}


                double time_from_start = 0.0;
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
                            if(j==0){
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec());
                            }else{
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec()-points.at(j-1).time_from_start.toSec());
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
                        time_from_start += points.at(points.size()-1).time_from_start.toSec();
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
                            timesteps_stage_aux.push_back(0.0);
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<JOINTS_ARM+JOINTS_HAND;++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_plan(jointsPosition_stage_plan.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_plan(jointsVelocity_stage_plan.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_plan(jointsAcceleration_stage_plan.rows()-1,k);
                                }else if(k==JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_base);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_base);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_base);
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
                            if(j==0){
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec());
                            }else{
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec()-points.at(j-1).time_from_start.toSec());
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
                        time_from_start += points.at(points.size()-1).time_from_start.toSec();
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
                            timesteps_stage_aux.push_back(0.0);
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<JOINTS_ARM+JOINTS_HAND;++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = jointsPosition_stage_approach(jointsPosition_stage_approach.rows()-1,k);
                                    jointsVelocity_stage_aux(j,k) = jointsVelocity_stage_approach(jointsVelocity_stage_approach.rows()-1,k);
                                    jointsAcceleration_stage_aux(j,k) = jointsAcceleration_stage_approach(jointsAcceleration_stage_approach.rows()-1,k);
                                }else if(k==JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(fing_base);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(fing_base);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(fing_base);
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
                            if(j==0){
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec());
                            }else{
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec()-points.at(j-1).time_from_start.toSec());
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
                        time_from_start += points.at(points.size()-1).time_from_start.toSec();
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
                            if(j==0){
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec());
                            }else{
                                timesteps_stage_aux.push_back(points.at(j).time_from_start.toSec()-points.at(j-1).time_from_start.toSec());
                            }
                            trajectory_msgs::JointTrajectoryPoint traj_pnt = points.at(j);
                            for(size_t k=0; k<traj_pnt.positions.size();++k){
                                if(k<JOINTS_ARM){
                                    jointsPosition_stage_aux(j,k) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,k) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,k) = traj_pnt.accelerations.at(k);
                                }else if(k==JOINTS_ARM+fing_base){
                                    jointsPosition_stage_aux(j,JOINTS_ARM) = traj_pnt.positions.at(k);
                                    jointsVelocity_stage_aux(j,JOINTS_ARM) = traj_pnt.velocities.at(k);
                                    jointsAcceleration_stage_aux(j,JOINTS_ARM) = traj_pnt.accelerations.at(k);
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
                        time_from_start += points.at(points.size()-1).time_from_start.toSec();
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
                }else{
                    //positions
                    this->jointsPosition_mov.clear();
                    this->jointsPosition_mov.push_back(jointsPosition_stage_plan);
                    this->jointsPosition_mov.push_back(jointsPosition_stage_approach);
                    this->jointsPosition_mov.push_back(jointsPosition_stage_retreat);
                    //velocities
                    this->jointsVelocity_mov.clear();
                    this->jointsVelocity_mov.push_back(jointsVelocity_stage_plan);
                    this->jointsVelocity_mov.push_back(jointsVelocity_stage_approach);
                    this->jointsVelocity_mov.push_back(jointsVelocity_stage_retreat);
                    //accelerations
                    this->jointsAcceleration_mov.clear();
                    this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_plan);
                    this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_approach);
                    this->jointsAcceleration_mov.push_back(jointsAcceleration_stage_retreat);
                    //time steps
                    this->timesteps_mov.clear();
                    this->timesteps_mov.push_back(timesteps_stage_plan);
                    this->timesteps_mov.push_back(timesteps_stage_approach);
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


// --- RESULTS --- //
if(solved){
    // time taken to solve the problem
    this->prob_time_mov = prob->getTime();
    ui.label_solving_time->setText(QString::number(this->prob_time_mov));

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

    // compute the hand values, positions and accelerations
    this->handPosition_mov.resize(tot_steps); this->handVelocityNorm_mov.resize(tot_steps);
    this->handLinearVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov.resize(tot_steps);
    int step = 0;
    int arm_code = prob->getMovement()->getArm();
    /*
    // positions
    VectorXd jointsPosition1_mov;
    VectorXd jointsPosition2_mov;
    VectorXd jointsPosition3_mov;
    VectorXd jointsPosition4_mov;
    VectorXd jointsPosition5_mov;
    VectorXd jointsPosition6_mov;
    VectorXd jointsPosition7_mov;
    VectorXd jointsPosition8_mov;
    VectorXd jointsPosition9_mov;
    VectorXd jointsPosition10_mov;
    VectorXd jointsPosition11_mov;
    // accelerations
    VectorXd jointsAcceleration1_mov;
    VectorXd jointsAcceleration2_mov;
    VectorXd jointsAcceleration3_mov;
    VectorXd jointsAcceleration4_mov;
    VectorXd jointsAcceleration5_mov;
    VectorXd jointsAcceleration6_mov;
    VectorXd jointsAcceleration7_mov;
    VectorXd jointsAcceleration8_mov;
    VectorXd jointsAcceleration9_mov;
    VectorXd jointsAcceleration10_mov;
    VectorXd jointsAcceleration11_mov;
    */
    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
        MatrixXd pos_stage = this->jointsPosition_mov.at(k);
        MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
        MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);
        /*
        int rows = jointsAcceleration1_mov.rows();
        // positions
        jointsPosition1_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition2_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition3_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition4_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition5_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition6_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition7_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition8_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition9_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition10_mov.conservativeResize(rows+pos_stage.rows());
        jointsPosition11_mov.conservativeResize(rows+pos_stage.rows());
        //accelerations
        jointsAcceleration1_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration2_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration3_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration4_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration5_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration6_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration7_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration8_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration9_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration10_mov.conservativeResize(rows+acc_stage.rows());
        jointsAcceleration11_mov.conservativeResize(rows+acc_stage.rows());
        */
        for(int i=0;i<pos_stage.rows();++i){
            // position
            VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
            vector<double> posture; posture.resize(pos_row.size());
            VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
            this->curr_scene->getHumanoid()->getHandPos(arm_code,this->handPosition_mov.at(step),posture);
            // velocity norm
            VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
            vector<double> velocities; velocities.resize(vel_row.size());
            VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
            this->handVelocityNorm_mov.at(step) = this->curr_scene->getHumanoid()->getHandVelNorm(arm_code,posture,velocities);
            vector<double> hand_vel; this->curr_scene->getHumanoid()->getHandVel(arm_code,hand_vel,posture,velocities);
            this->handLinearVelocity_mov.at(step) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
            this->handAngularVelocity_mov.at(step) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
            step++;
            /*
            //positions
            jointsPosition1_mov(i+rows) = pos_stage(i,0);
            jointsPosition2_mov(i+rows) = pos_stage(i,1);
            jointsPosition3_mov(i+rows) = pos_stage(i,2);
            jointsPosition4_mov(i+rows) = pos_stage(i,3);
            jointsPosition5_mov(i+rows) = pos_stage(i,4);
            jointsPosition6_mov(i+rows) = pos_stage(i,5);
            jointsPosition7_mov(i+rows) = pos_stage(i,6);
            jointsPosition8_mov(i+rows) = pos_stage(i,7);
            jointsPosition9_mov(i+rows) = pos_stage(i,8);
            jointsPosition10_mov(i+rows) = pos_stage(i,9);
            jointsPosition11_mov(i+rows) = pos_stage(i,10);
            // accelerations
            jointsAcceleration1_mov(i+rows) = acc_stage(i,0);
            jointsAcceleration2_mov(i+rows) = acc_stage(i,1);
            jointsAcceleration3_mov(i+rows) = acc_stage(i,2);
            jointsAcceleration4_mov(i+rows) = acc_stage(i,3);
            jointsAcceleration5_mov(i+rows) = acc_stage(i,4);
            jointsAcceleration6_mov(i+rows) = acc_stage(i,5);
            jointsAcceleration7_mov(i+rows) = acc_stage(i,6);
            jointsAcceleration8_mov(i+rows) = acc_stage(i,7);
            jointsAcceleration9_mov(i+rows) = acc_stage(i,8);
            jointsAcceleration10_mov(i+rows) = acc_stage(i,9);
            jointsAcceleration11_mov(i+rows) = acc_stage(i,10);
            */

        }
    }    


    // --- compute the cost of the trajectory according to the jerk principle --- //

    /*
    // positions
    // pos1
    vector<double> pos1_mov; pos1_mov.resize(jointsPosition1_mov.size());
    VectorXd::Map(&pos1_mov[0], jointsPosition1_mov.size()) = jointsPosition1_mov;
    QVector<double> pos1 = QVector<double>::fromStdVector(pos1_mov);
    // pos2
    vector<double> pos2_mov; pos2_mov.resize(jointsPosition2_mov.size());
    VectorXd::Map(&pos2_mov[0], jointsPosition2_mov.size()) = jointsPosition2_mov;
    QVector<double> pos2 = QVector<double>::fromStdVector(pos2_mov);
    // pos3
    vector<double> pos3_mov; pos3_mov.resize(jointsPosition3_mov.size());
    VectorXd::Map(&pos3_mov[0], jointsPosition3_mov.size()) = jointsPosition3_mov;
    QVector<double> pos3 = QVector<double>::fromStdVector(pos3_mov);
    // pos4
    vector<double> pos4_mov; pos4_mov.resize(jointsPosition4_mov.size());
    VectorXd::Map(&pos4_mov[0], jointsPosition4_mov.size()) = jointsPosition4_mov;
    QVector<double> pos4 = QVector<double>::fromStdVector(pos4_mov);
    // pos5
    vector<double> pos5_mov; pos5_mov.resize(jointsPosition5_mov.size());
    VectorXd::Map(&pos5_mov[0], jointsPosition5_mov.size()) = jointsPosition5_mov;
    QVector<double> pos5 = QVector<double>::fromStdVector(pos5_mov);
    // pos6
    vector<double> pos6_mov; pos6_mov.resize(jointsPosition6_mov.size());
    VectorXd::Map(&pos6_mov[0], jointsPosition6_mov.size()) = jointsPosition6_mov;
    QVector<double> pos6 = QVector<double>::fromStdVector(pos6_mov);
    // pos7
    vector<double> pos7_mov; pos7_mov.resize(jointsPosition7_mov.size());
    VectorXd::Map(&pos7_mov[0], jointsPosition7_mov.size()) = jointsPosition7_mov;
    QVector<double> pos7 = QVector<double>::fromStdVector(pos7_mov);
    // pos8
    vector<double> pos8_mov; pos8_mov.resize(jointsPosition8_mov.size());
    VectorXd::Map(&pos8_mov[0], jointsPosition8_mov.size()) = jointsPosition8_mov;
    QVector<double> pos8 = QVector<double>::fromStdVector(pos8_mov);
    // pos9
    vector<double> pos9_mov; pos9_mov.resize(jointsPosition9_mov.size());
    VectorXd::Map(&pos9_mov[0], jointsPosition9_mov.size()) = jointsPosition9_mov;
    QVector<double> pos9 = QVector<double>::fromStdVector(pos9_mov);
    // pos10
    vector<double> pos10_mov; pos10_mov.resize(jointsPosition10_mov.size());
    VectorXd::Map(&pos10_mov[0], jointsPosition10_mov.size()) = jointsPosition10_mov;
    QVector<double> pos10 = QVector<double>::fromStdVector(pos10_mov);
    // pos11
    vector<double> pos11_mov; pos11_mov.resize(jointsPosition11_mov.size());
    VectorXd::Map(&pos11_mov[0], jointsPosition11_mov.size()) = jointsPosition11_mov;
    QVector<double> pos11 = QVector<double>::fromStdVector(pos11_mov);

    // accelerations
    // acc1
    vector<double> acc1_mov; acc1_mov.resize(jointsAcceleration1_mov.size());
    VectorXd::Map(&acc1_mov[0], jointsAcceleration1_mov.size()) = jointsAcceleration1_mov;
    QVector<double> acc1 = QVector<double>::fromStdVector(acc1_mov);
    QVector<double> der_acc1;
    // acc2
    vector<double> acc2_mov; acc2_mov.resize(jointsAcceleration2_mov.size());
    VectorXd::Map(&acc2_mov[0], jointsAcceleration2_mov.size()) = jointsAcceleration2_mov;
    QVector<double> acc2 = QVector<double>::fromStdVector(acc2_mov);
    QVector<double> der_acc2;
    // acc3
    vector<double> acc3_mov; acc3_mov.resize(jointsAcceleration3_mov.size());
    VectorXd::Map(&acc3_mov[0], jointsAcceleration3_mov.size()) = jointsAcceleration3_mov;
    QVector<double> acc3 = QVector<double>::fromStdVector(acc3_mov);
    QVector<double> der_acc3;
    // acc4
    vector<double> acc4_mov; acc4_mov.resize(jointsAcceleration4_mov.size());
    VectorXd::Map(&acc4_mov[0], jointsAcceleration4_mov.size()) = jointsAcceleration4_mov;
    QVector<double> acc4 = QVector<double>::fromStdVector(acc4_mov);
    QVector<double> der_acc4;
    // acc5
    vector<double> acc5_mov; acc5_mov.resize(jointsAcceleration5_mov.size());
    VectorXd::Map(&acc5_mov[0], jointsAcceleration5_mov.size()) = jointsAcceleration5_mov;
    QVector<double> acc5 = QVector<double>::fromStdVector(acc5_mov);
    QVector<double> der_acc5;
    // acc6
    vector<double> acc6_mov; acc6_mov.resize(jointsAcceleration6_mov.size());
    VectorXd::Map(&acc6_mov[0], jointsAcceleration6_mov.size()) = jointsAcceleration6_mov;
    QVector<double> acc6 = QVector<double>::fromStdVector(acc6_mov);
    QVector<double> der_acc6;
    // acc7
    vector<double> acc7_mov; acc7_mov.resize(jointsAcceleration7_mov.size());
    VectorXd::Map(&acc7_mov[0], jointsAcceleration7_mov.size()) = jointsAcceleration7_mov;
    QVector<double> acc7 = QVector<double>::fromStdVector(acc7_mov);
    QVector<double> der_acc7;
    // acc8
    vector<double> acc8_mov; acc8_mov.resize(jointsAcceleration8_mov.size());
    VectorXd::Map(&acc8_mov[0], jointsAcceleration8_mov.size()) = jointsAcceleration8_mov;
    QVector<double> acc8 = QVector<double>::fromStdVector(acc8_mov);
    QVector<double> der_acc8;
    // acc9
    vector<double> acc9_mov; acc9_mov.resize(jointsAcceleration9_mov.size());
    VectorXd::Map(&acc9_mov[0], jointsAcceleration9_mov.size()) = jointsAcceleration9_mov;
    QVector<double> acc9 = QVector<double>::fromStdVector(acc9_mov);
    QVector<double> der_acc9;
    // acc10
    vector<double> acc10_mov; acc10_mov.resize(jointsAcceleration10_mov.size());
    VectorXd::Map(&acc10_mov[0], jointsAcceleration10_mov.size()) = jointsAcceleration10_mov;
    QVector<double> acc10 = QVector<double>::fromStdVector(acc10_mov);
    QVector<double> der_acc10;
    // acc11
    vector<double> acc11_mov; acc11_mov.resize(jointsAcceleration11_mov.size());
    VectorXd::Map(&acc11_mov[0], jointsAcceleration11_mov.size()) = jointsAcceleration11_mov;
    QVector<double> acc11 = QVector<double>::fromStdVector(acc11_mov);
    QVector<double> der_acc11;

    // joint legths and duration
    double length_1=0.0; double duration_1=0.0;
    double length_2=0.0; double duration_2=0.0;
    double length_3=0.0; double duration_3=0.0;
    double length_4=0.0; double duration_4=0.0;
    double length_5=0.0; double duration_5=0.0;
    double length_6=0.0; double duration_6=0.0;
    double length_7=0.0; double duration_7=0.0;
    double length_8=0.0; double duration_8=0.0;
    double length_9=0.0; double duration_9=0.0;
    double length_10=0.0; double duration_10=0.0;
    double length_11=0.0; double duration_11=0.0;
    for(size_t i=1;i<pos1.size();++i){
        length_1 += abs(pos1.at(i)-pos1.at(i-1)); duration_1 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_2 += abs(pos2.at(i)-pos2.at(i-1)); duration_2 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_3 += abs(pos3.at(i)-pos3.at(i-1)); duration_3 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_4 += abs(pos4.at(i)-pos4.at(i-1)); duration_4 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_5 += abs(pos5.at(i)-pos5.at(i-1)); duration_5 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_6 += abs(pos6.at(i)-pos6.at(i-1)); duration_6 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_7 += abs(pos7.at(i)-pos7.at(i-1)); duration_7 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_8 += abs(pos8.at(i)-pos8.at(i-1)); duration_8 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_9 += abs(pos9.at(i)-pos9.at(i-1)); duration_9 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_10 += abs(pos10.at(i)-pos10.at(i-1)); duration_10 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
        length_11 += abs(pos11.at(i)-pos11.at(i-1)); duration_11 += abs(this->qtime_mov.at(i)-this->qtime_mov.at(i-1));
    }
    if(length_1==0)
        length_1=0.0001;
    if(length_2==0)
        length_2=0.0001;
    if(length_3==0)
        length_3=0.0001;
    if(length_4==0)
        length_4=0.0001;
    if(length_5==0)
        length_5=0.0001;
    if(length_6==0)
        length_6=0.0001;
    if(length_7==0)
        length_7=0.0001;
    if(length_8==0)
        length_8=0.0001;
    if(length_9==0)
        length_9=0.0001;
    if(length_10==0)
        length_10=0.0001;
    if(length_11==0)
        length_11=0.0001;
    // acceleration derivatives
    this->getDerivative(acc1,tot_timesteps,der_acc1);
    this->getDerivative(acc2,tot_timesteps,der_acc2);
    this->getDerivative(acc3,tot_timesteps,der_acc3);
    this->getDerivative(acc4,tot_timesteps,der_acc4);
    this->getDerivative(acc5,tot_timesteps,der_acc5);
    this->getDerivative(acc6,tot_timesteps,der_acc6);
    this->getDerivative(acc7,tot_timesteps,der_acc7);
    this->getDerivative(acc8,tot_timesteps,der_acc8);
    this->getDerivative(acc9,tot_timesteps,der_acc9);
    this->getDerivative(acc10,tot_timesteps,der_acc10);
    this->getDerivative(acc11,tot_timesteps,der_acc11);

    // squared derivatives
    QVector<double> squared_der_acc1;
    QVector<double> squared_der_acc2;
    QVector<double> squared_der_acc3;
    QVector<double> squared_der_acc4;
    QVector<double> squared_der_acc5;
    QVector<double> squared_der_acc6;
    QVector<double> squared_der_acc7;
    QVector<double> squared_der_acc8;
    QVector<double> squared_der_acc9;
    QVector<double> squared_der_acc10;
    QVector<double> squared_der_acc11;

    for(size_t i=0;i<der_acc1.size();++i){
        squared_der_acc1.push_back(pow(der_acc1.at(i),2));
        squared_der_acc2.push_back(pow(der_acc2.at(i),2));
        squared_der_acc3.push_back(pow(der_acc3.at(i),2));
        squared_der_acc4.push_back(pow(der_acc4.at(i),2));
        squared_der_acc5.push_back(pow(der_acc5.at(i),2));
        squared_der_acc6.push_back(pow(der_acc6.at(i),2));
        squared_der_acc7.push_back(pow(der_acc7.at(i),2));
        squared_der_acc8.push_back(pow(der_acc8.at(i),2));
        squared_der_acc9.push_back(pow(der_acc9.at(i),2));
        squared_der_acc10.push_back(pow(der_acc10.at(i),2));
        squared_der_acc11.push_back(pow(der_acc11.at(i),2));
    }

    // normalized jerk cost of the joints
    double cost_jerk_1=0.0;
    double cost_jerk_2=0.0;
    double cost_jerk_3=0.0;
    double cost_jerk_4=0.0;
    double cost_jerk_5=0.0;
    double cost_jerk_6=0.0;
    double cost_jerk_7=0.0;
    double cost_jerk_8=0.0;
    double cost_jerk_9=0.0;
    double cost_jerk_10=0.0;
    double cost_jerk_11=0.0;
    for(size_t i=1;i<tot_timesteps.size();++i){
        cost_jerk_1 += squared_der_acc1.at(i)*(pow(duration_1,5)/pow(length_1,2))*tot_timesteps.at(i);
        cost_jerk_2 += squared_der_acc2.at(i)*(pow(duration_2,5)/pow(length_2,2))*tot_timesteps.at(i);
        cost_jerk_3 += squared_der_acc3.at(i)*(pow(duration_3,5)/pow(length_3,2))*tot_timesteps.at(i);
        cost_jerk_4 += squared_der_acc4.at(i)*(pow(duration_4,5)/pow(length_4,2))*tot_timesteps.at(i);
        cost_jerk_5 += squared_der_acc5.at(i)*(pow(duration_5,5)/pow(length_5,2))*tot_timesteps.at(i);
        cost_jerk_6 += squared_der_acc6.at(i)*(pow(duration_6,5)/pow(length_6,2))*tot_timesteps.at(i);
        cost_jerk_7 += squared_der_acc7.at(i)*(pow(duration_7,5)/pow(length_7,2))*tot_timesteps.at(i);
        cost_jerk_8 += squared_der_acc8.at(i)*(pow(duration_8,5)/pow(length_8,2))*tot_timesteps.at(i);
        cost_jerk_9 += squared_der_acc9.at(i)*(pow(duration_9,5)/pow(length_9,2))*tot_timesteps.at(i);
        cost_jerk_10 += squared_der_acc10.at(i)*(pow(duration_10,5)/pow(length_10,2))*tot_timesteps.at(i);
        cost_jerk_11 += squared_der_acc11.at(i)*(pow(duration_11,5)/pow(length_11,2))*tot_timesteps.at(i);
    }

    // normalized total jerk cost of the joints
    double total_cost_jerk = sqrt(0.5*cost_jerk_1)+
                            sqrt(0.5*cost_jerk_2)+
                            sqrt(0.5*cost_jerk_3)+
                            sqrt(0.5*cost_jerk_4)+
                            sqrt(0.5*cost_jerk_5)+
                            sqrt(0.5*cost_jerk_6)+
                            sqrt(0.5*cost_jerk_7)+
                            sqrt(0.5*cost_jerk_8)+
                            sqrt(0.5*cost_jerk_9)+
                            sqrt(0.5*cost_jerk_10)+
                            sqrt(0.5*cost_jerk_11);
    ui.label_cost_joints_value->setText(QString::number(total_cost_jerk));
    */

    // normlized jerk cost of the hand
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

} // if the problem has been solved

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


    qnode.execMovement(this->jointsPosition_mov,this->jointsVelocity_mov,this->timesteps_mov, this->tols_stop_mov, this->curr_mov, this->curr_scene);

}

void MainWindow::on_pushButton_execMov_moveit_clicked()
{
    if(this->moveit_mov)
        this->m_planner->execute(m_results);
}

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

void MainWindow::on_pushButton_execTask_pressed(){

    qnode.log(QNode::Info,std::string("Executing the task . . ."));
}


void MainWindow::on_pushButton_execTask_clicked()
{


    qnode.execTask(this->jointsPosition_task,this->jointsVelocity_task,this->timesteps_task, this->tols_stop_task, this->curr_task, this->curr_scene);


}


void MainWindow::on_pushButton_load_task_clicked()
{


    int plan_id; QString plan_type;
    int mov_id; QString mov_type;
    int arm_code; QString arm_type;
    int grip_id; QString grip_type;
    QString obj_str; objectPtr obj;
    QString obj_eng_str; objectPtr obj_eng;
    bool prec;
    int row=0;
    MatrixXd pos_stage;
    MatrixXd vel_stage;
    MatrixXd acc_stage;

    //clear
    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->timesteps_task.clear();
    this->tols_stop_task.clear();
    this->njs_task.clear();
    this->nmu_task.clear();
    this->prob_time_task.clear();
    this->jointsPosition_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsAcceleration_mov.clear();
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
                // the previous movement hs finished
                if(!t_mov.empty()){
                    this->jointsPosition_task.push_back(t_mov);
                    this->jointsVelocity_task.push_back(w_mov);
                    this->jointsAcceleration_task.push_back(a_mov);
                    this->timesteps_task.push_back(timesteps_mov);
                    this->tols_stop_task.push_back(tols_stop_mov);
                }
                // new movement in the task
                t_mov.clear();
                w_mov.clear();
                a_mov.clear();
                timesteps_mov.clear();
                tols_stop_mov.clear();
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
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Object Engaged"),Qt::CaseInsensitive)==0){
                        obj_eng_str=fields1.at(1).simplified();
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Grip Type"),Qt::CaseInsensitive)==0){
                        grip_type=fields1.at(1).simplified();
                    }
                }

                //get the planner id
                if(QString::compare(plan_type,QString("HUML"),Qt::CaseInsensitive)==0){
                    plan_id=0;
                }else if(QString::compare(plan_type,QString("RRT"),Qt::CaseInsensitive)==0){
                    plan_id=1;
                }else if(QString::compare(plan_type,QString("RRTConnect"),Qt::CaseInsensitive)==0){
                    plan_id=2;
                }else if(QString::compare(plan_type,QString("RRTstar"),Qt::CaseInsensitive)==0){
                    plan_id=3;
                }else if(QString::compare(plan_type,QString("PRM"),Qt::CaseInsensitive)==0){
                    plan_id=4;
                }else if(QString::compare(plan_type,QString("PRMstar"),Qt::CaseInsensitive)==0){
                    plan_id=5;
                }

                // get the grip type
                if(QString::compare(grip_type,QString("Precision Side thumb left"),Qt::CaseInsensitive)==0){
                    grip_id=0;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Precision Side thumb right"),Qt::CaseInsensitive)==0){
                    grip_id=1;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Precision Side thumb up"),Qt::CaseInsensitive)==0){
                    grip_id=2;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Precision Side thumb down"),Qt::CaseInsensitive)==0){
                    grip_id=3;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Precision Above"),Qt::CaseInsensitive)==0){
                    grip_id=4;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Precision Below"),Qt::CaseInsensitive)==0){
                    grip_id=5;
                    prec=true;
                }else if(QString::compare(grip_type,QString("Full Side thumb left"),Qt::CaseInsensitive)==0){
                    grip_id=0;
                    prec=false;
                }else if(QString::compare(grip_type,QString("Full Side thumb right"),Qt::CaseInsensitive)==0){
                    grip_id=1;
                    prec=false;
                }else if(QString::compare(grip_type,QString("Full Side thumb up"),Qt::CaseInsensitive)==0){
                    grip_id=2;
                    prec=false;
                }else if(QString::compare(grip_type,QString("Full Side thumb down"),Qt::CaseInsensitive)==0){
                    grip_id=3;
                    prec=false;
                }else if(QString::compare(grip_type,QString("Full Above"),Qt::CaseInsensitive)==0){
                    grip_id=4;
                    prec=false;
                }else if(QString::compare(grip_type,QString("Full Below"),Qt::CaseInsensitive)==0){
                    grip_id=5;
                    prec=false;
                }

                //get the arm
                if(QString::compare(arm_type,QString("both"),Qt::CaseInsensitive)==0){
                    arm_code=0;
                }else if(QString::compare(arm_type,QString("right"),Qt::CaseInsensitive)==0){
                    arm_code=1;
                }else if(QString::compare(arm_type,QString("left"),Qt::CaseInsensitive)==0){
                    arm_code=2;
                }

                // get the movement type
                if(QString::compare(mov_type,QString("Reach-to-grasp"),Qt::CaseInsensitive)==0){
                    mov_id=0;
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());
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
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
                    }else{
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,grip_id,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
                    }
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Reaching"),Qt::CaseInsensitive)==0){
                    mov_id=1;
                }else if(QString::compare(mov_type,QString("Transport"),Qt::CaseInsensitive)==0){
                    mov_id=2;
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
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
                    }else{
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,grip_id,prec),new Scenario(*(this->curr_scene.get())),this->m_planner));
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
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));
                    }else{
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get())),this->m_planner));
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
            }else if(line.at(0)==QChar('n')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("njs"),Qt::CaseInsensitive)==0){
                    this->njs_task.push_back(fields.at(1).toDouble());
                }
            }else if(line.at(0)==QChar('n')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("nmu"),Qt::CaseInsensitive)==0){
                    this->nmu_task.push_back(fields.at(1).toDouble());
                }
            }else if(line.at(0)==QChar('p')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("prob_time"),Qt::CaseInsensitive)==0){
                    this->prob_time_task.push_back(fields.at(1).toDouble());
                }
            }else{
                pos_stage.conservativeResize(pos_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                vel_stage.conservativeResize(vel_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                acc_stage.conservativeResize(acc_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);

                QStringList fields = line.split(",");                               
                for(int i=0; i <fields.size();++i){
                    QStringList fields1 = fields.at(i).split("=");
                    if(QString::compare(fields1.at(0).simplified(),QString("time step"),Qt::CaseInsensitive)==0){
                        timesteps_stage.push_back(fields1.at(1).toDouble());
                    }
                    for(int k=0; k < JOINTS_ARM+JOINTS_HAND; ++k){
                        if(QString::compare(fields1.at(0).simplified(),QString("Joint ")+QString::number(k+1),Qt::CaseInsensitive)==0){
                            QStringList fields2 = fields1.at(1).split("|");
                            pos_stage(row,k) = fields2.at(0).toDouble()*M_PI/180;
                            vel_stage(row,k) = fields2.at(1).toDouble()*M_PI/180;
                            acc_stage(row,k) = fields2.at(2).toDouble()*M_PI/180;
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
        for(size_t h=0; h< this->jointsPosition_task.size();++h){
            pos_mov = this->jointsPosition_task.at(h);
            vel_mov = this->jointsVelocity_task.at(h);
            acc_mov = this->jointsAcceleration_task.at(h);
            tstep_mov = this->timesteps_task.at(h);
            mov_duration = 0.0;

            for (size_t k=0; k< pos_mov.size();++k){
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
                for(int i =0; i< jointPosition_stage.rows(); ++i){
                    tot_steps++;
                    if(i>0){
                        time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                        stage_duration += tstep_stage.at(i);
                    }
                    stage_step.clear();
                    v_headers.push_back(QString("Step ")+QString::number(i));
                    for (int j=0; j<jointPosition_stage.cols();++j){
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
        int step = 0;
        for(size_t j=0;j<this->jointsPosition_task.size();++j){
            vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
            vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);
            for (size_t k=0; k< pos_mov.size();++k){
                MatrixXd pos_stage = pos_mov.at(k);
                MatrixXd vel_stage = vel_mov.at(k);
                for(int i=0;i<pos_stage.rows();++i){
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
                    step++;
                }
            }
        }

        // compute njs, nmu and planning time
        // njs
        double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
        double mean_njs = ((double)sum_njs) / this->njs_task.size();
        string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
        boost::replace_all(mean_njs_str,",",".");
        double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
        double stdev_njs = std::sqrt(((double)sq_sum_njs) / this->njs_task.size() - mean_njs * mean_njs);
        string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
        boost::replace_all(stdev_njs_str,",",".");
        ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));
        // nmu
        double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
        double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
        string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
        boost::replace_all(mean_nmu_str,",",".");
        double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
        double stdev_nmu = std::sqrt(((double)sq_sum_nmu) / this->nmu_task.size() - mean_nmu * mean_nmu);
        string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
        boost::replace_all(stdev_nmu_str,",",".");
        ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));
        // planning time
        double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
        double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
        string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
        boost::replace_all(mean_prob_str,",",".");
        double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
        double stdev_prob = std::sqrt(((double)sq_sum_prob) / this->prob_time_task.size() - mean_prob * mean_prob);
        string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
        boost::replace_all(stdev_prob_str,",",".");
        ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));


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
        for(int i=0; i <ui.listWidget_movs->count(); ++i){
            if((curr_task->getProblem(i)->getSolved()) && curr_task->getProblem(i)->getPartOfTask()){
                stream << "# " << ui.listWidget_movs->item(i)->text().toStdString().c_str()<< endl;
                vector< MatrixXd > traj_mov = this->jointsPosition_task.at(i-h);
                vector< MatrixXd > vel_mov = this->jointsVelocity_task.at(i-h);
                vector< MatrixXd > acc_mov = this->jointsAcceleration_task.at(i-h);
                vector< vector< double > > timesteps_mov = this->timesteps_task.at(i-h);
                vector< double > tols_stop_mov = this->tols_stop_task.at(i-h);
                double njs = this->njs_task.at(i-h);
                int nmu = this->nmu_task.at(i-h);
                double prob_time = this->prob_time_task.at(i-h);
                stream << "njs="<< QString::number(njs).toStdString().c_str()<< endl;
                stream << "nmu="<< QString::number(nmu).toStdString().c_str()<< endl;
                stream << "prob_time="<< QString::number(prob_time).toStdString().c_str()<< endl;

                for(size_t j=0;j < traj_mov.size(); ++j){
                    stream << "Movement stage: "<< QString::number(j+1).toStdString().c_str()<< endl;
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
                                      QString::number(traj(r,c)*180/M_PI,'g',3).toStdString().c_str()<<"|"<<
                                      QString::number(vel(r,c)*180/M_PI,'g',3).toStdString().c_str()<<"|"<<
                                      QString::number(acc(r,c)*180/M_PI,'g',3).toStdString().c_str();
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
    this->m_planner.reset(new moveit_planning::HumanoidPlanner(this->init_scene->getName()));

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

    // Toy vehicle scenario with ARoS
    string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros.ttt");
    string path_rviz_toyscene_aros = PATH_SCENARIOS+string("/rviz/toy_vehicle_aros.scene");

    // Toy vehicle scenario with Jarde
    string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
    string path_rviz_toyscene_jarde = PATH_SCENARIOS+string("/rviz/toy_vehicle_jarde.scene");

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
        success = string("Empty scenario with ARoS with ARoS HAS BEEN LOADED");
        failure = string("Empty scenario with ARoS with ARoS HAS NOT BEEN LOADED");
        break;
    case 3:
        // Assistive scenario: beverages with ARoS
        //TO DO
        break;
    case 4:
        // Assistive scenario: beverages with Avatar
        //TO DO
        break;
    case 5:
        // Organizing scenario: shelfs and objects with ARoS
        //TO DO
        break;
    case 6:
        // Organizing scenario: shelfs ad objects with Avatar
        //TO DO

        break;
    }

    if (qnode.loadScenario(path,1)){
        qnode.log(QNode::Info,success);
        ui.groupBox_getElements->setEnabled(true);
        ui.groupBox_homePosture->setEnabled(true);
        std::vector<objectPtr> objs; this->curr_scene->getObjects(objs);
        qnode.loadRVizScenario(objs);
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

    if(curr_task->getProblem(ui.listWidget_movs->currentRow())->getSolved()){
        this->jointsPosition_task.push_back(this->jointsPosition_mov);
        this->jointsVelocity_task.push_back(this->jointsVelocity_mov);
        this->jointsAcceleration_task.push_back(this->jointsAcceleration_mov);
        this->timesteps_task.push_back(this->timesteps_mov);
        this->tols_stop_task.push_back(this->tols_stop_mov);
        this->njs_task.push_back(this->njs_mov);
        this->nmu_task.push_back(this->nmu_mov);
        this->prob_time_task.push_back(this->prob_time_mov);
         QStringList h_headers; bool h_head=false; QStringList v_headers;
         std::vector<std::vector<QString>> task_steps;
         vector<MatrixXd> pos_mov; vector<MatrixXd> vel_mov; vector<MatrixXd> acc_mov; vector<vector<double>> tstep_mov;
         double task_duration = 0.0; double mov_duration = 0.0; double stage_duration = 0.0;
         vector<double> time_task; uint tot_steps = 0;
         for(size_t h=0; h< this->jointsPosition_task.size();++h){
             pos_mov = this->jointsPosition_task.at(h);
             vel_mov = this->jointsVelocity_task.at(h);
             acc_mov = this->jointsAcceleration_task.at(h);
             tstep_mov = this->timesteps_task.at(h);
             mov_duration = 0.0;
             for (size_t k=0; k< pos_mov.size();++k){
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
                 for(int i =0; i< jointPosition_stage.rows(); ++i){
                     tot_steps++;
                     if(i>0){
                         time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                         stage_duration += tstep_stage.at(i);
                     }
                     stage_step.clear();
                     v_headers.push_back(QString("Step ")+QString::number(i));
                     for (int j=0; j<jointPosition_stage.cols();++j){
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
         int step = 0;
         int arm_code = this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getMovement()->getArm();
         for(size_t j=0;j<this->jointsPosition_task.size();++j){
             vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
             vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);
             for (size_t k=0; k< pos_mov.size();++k){
                 MatrixXd pos_stage = pos_mov.at(k);
                 MatrixXd vel_stage = vel_mov.at(k);
                 for(int i=0;i<pos_stage.rows();++i){
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
                     step++;
                 }
             }
         }

         // compute njs, nmu and planning time
         // njs
         double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
         double mean_njs = ((double)sum_njs) / this->njs_task.size();
         string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
         boost::replace_all(mean_njs_str,",",".");
         double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
         double stdev_njs = std::sqrt(((double)sq_sum_njs) / this->njs_task.size() - mean_njs * mean_njs);
         string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
         boost::replace_all(stdev_njs_str,",",".");
         ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));
         // nmu
         double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
         double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
         string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
         boost::replace_all(mean_nmu_str,",",".");
         double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
         double stdev_nmu = std::sqrt(((double)sq_sum_nmu) / this->nmu_task.size() - mean_nmu * mean_nmu);
         string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
         boost::replace_all(stdev_nmu_str,",",".");
         ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));
         // planning time
         double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
         double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
         string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
         boost::replace_all(mean_prob_str,",",".");
         double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
         double stdev_prob = std::sqrt(((double)sq_sum_prob) / this->prob_time_task.size() - mean_prob * mean_prob);
         string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
         boost::replace_all(stdev_prob_str,",",".");
         ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));




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
    this->handPosition_mov.clear();
    this->handOrientation_mov.clear();
    this->handLinearVelocity_mov.clear();
    this->handAngularVelocity_mov.clear();
    this->handVelocityNorm_mov.clear();
    this->handPosition_task.clear();
    this->handOrientation_task.clear();
    this->handLinearVelocity_task.clear();
    this->handAngularVelocity_task.clear();
    this->handVelocityNorm_task.clear();
    this->nmu_task.clear();
    this->njs_task.clear();
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
           // Single- arm task
       ui.radioButton_right->setEnabled(true);
       ui.radioButton_left->setEnabled(true);

       break;

       case 1:
           //Dual-arm task

        ui.radioButton_right->setEnabled(false);
        ui.radioButton_left->setEnabled(false);


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
            break;
        case 1:
        // Reaching
            ui.comboBox_objects->setEnabled(false);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(false);
            ui.groupBox_grip->setEnabled(false);
            break;
        case 2:
        // Transport
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            break;
        case 3:
        //Engage
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(true);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            break;
        case 4:
        //Disengage
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(true);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
            break;
        case 5:
        // Go park
            ui.comboBox_objects->setEnabled(false);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(false);
            ui.groupBox_grip->setEnabled(false);
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
                //Assistive scenario: beverages with ARoS

                break;

            case 3:
                //Organizing scenario: shelfs and objects with ARoS

                break;

            }

        }

    }
#endif

}


void MainWindow::on_pushButton_plot_mov_clicked()
{

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

void MainWindow::on_pushButton_plot_task_clicked()
{
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

void MainWindow::on_pushButton_joints_results_mov_clicked()
{
    if(!this->jointsPosition_mov.empty())
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_mov,this->jointsVelocity_mov,this->jointsAcceleration_mov,this->timesteps_mov);
    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_joints_results_task_clicked()
{
    if(!this->jointsPosition_task.empty())
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_task,this->jointsVelocity_task,this->jointsAcceleration_task,this->timesteps_task);
    this->mResultsJointsdlg->show();
}

void MainWindow::on_pushButton_power_law_clicked()
{
    if(!this->handPosition_task.empty())
        this->mPowerLawdlg->setupPlots(this->handPosition_task,this->timesteps_task);
    this->mPowerLawdlg->show();
}

void MainWindow::on_pushButton_comp_hand_vel_mov_clicked()
{
    if(!this->handLinearVelocity_mov.empty())
        this->mHandVeldlg->setupPlots(this->handLinearVelocity_mov,this->handAngularVelocity_mov,this->qtime_mov);
    this->mHandVeldlg->show();

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
    ui.plot_hand_vel_task->savePdf(path+QString("hand_vel_task.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));

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

    results.close();


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
           step_value=0.01;
       derFunction.push_back(((-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h))/step_value);

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
           step_value=0.01;
       derFunction.push_back((( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h))/step_value);

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
           derFunction.push_back(((  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h))/step_value);
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
           step_value=0.01;
       derFunction.push_back((( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h))/step_value);

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
           step_value=0.01;
       derFunction.push_back(((  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h))/step_value);

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



}  // namespace motion_manager

