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
    case 2:
        break;
    }

}


void MainWindow::on_pushButton_loadScenario_clicked()
{

    // scenarios
    QVector<QString> scenarios;
        //0 --> Toy vehicle scenario
        //1 --> Beverages scenario
        //2 --> shelfs and objects scenario
#if HAND == 0

    scenarios.push_back(QString("Assembly scenario: Toy vehicle with Jarde"));
    //scenarios.push_back(QString("Assistive scenario: beverages with Jarde"));
    //scenarios.push_back(QString("Organizing scenario: shelfs and objects with Jarde"));

#elif HAND == 1

    scenarios.push_back(QString("Assembly scenario: Toy vehicle with ARoS"));
    //scenarios.push_back(QString("Assistive scenario: beverages with ARoS"));
    //scenarios.push_back(QString("Organizing scenario: shelfs and objects with ARoS"));

#endif

    //this->scenario_id = ui.listWidget_scenario->currentRow();

    QString scenario_text = ui.listWidget_scenario->currentItem()->text();

    int equal;
    for(int i=0; i<scenarios.size();++i){

         equal = scenario_text.compare(scenarios.at(i));
         if(equal==0){

             // Toy vehicle scenario with ARoS
             string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros.ttt");
             //string path_rviz_toyscene_aros = PATH_SCENARIOS+string("/rviz/toy_vehicle_aros.scene");

             // Toy vehicle scenario with Jarde
             string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
             //string path_rviz_toyscene_jarde = PATH_SCENARIOS+string("/rviz/toy_vehicle_jarde.scene");

             switch(i){

             case 0:

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

             case 1:
                 // Assistive scenario: beverages

                 //TO DO

                 break;

             case 2:
                 // Organizing scenario: shelfs ad objects
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

    //if (planner_id==0){
    // Human-like Upper-limbs Motion Planner

        //int rows = ui.listWidget_movs->count();
        bool add = true;
        /*
        if (rows > 0){
            h_problemPtr pre_prob = hum_planner->getProblem(rows-1); //previous problem
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


            // add the movement
            //h_scenarioPtr scene = this->hum_planner->getScenario();

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

                }else{
                    qnode.log(QNode::Error,std::string("The movement requires two objects"));
                }

             }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled()){

                 if(std::strcmp(ui.comboBox_mov->currentText().toStdString().c_str(),"Go-park")!=0){
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

                     }else{
                         qnode.log(QNode::Error,std::string("The movement requires an object"));

                     }

                 }else{

                     // Go park movements
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

                     }else{
                        if(planner_id==0){
                            curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));
                        }else{
                            curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get())),this->m_planner));
                        }
                        success=true;

                     }
                 }


             }else{
                 // reaching movements
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


    //}else{

        //Humanoid MoveIt! Planner




    //} //if/else planner

} // add movement


void MainWindow::on_pushButton_plan_clicked()
{

    ui.tabWidget_sol->setCurrentIndex(0);    
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    double tol_stop; // stop tolerance on the joints when executing the movements
    HUMotion::planning_result_ptr h_results; HUMotion::huml_params  tols;
    moveit_planning::PlanningResultPtr m_results; moveit_planning::moveit_params m_params;

    bool solved = false;
 try{
    switch(planner_id){

    case 0: // HUML
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

        tol_stop = mTolHumldlg->getTolStop(); // stop tolerance on the joints when executing the movements

        h_results = prob->solve(tols); // plan the movement

        ui.pushButton_plan->setCheckable(false);
        if(h_results->status==0){
            qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
            this->curr_mov = prob->getMovement();
            this->timeSteps_mov = h_results->time_steps;
            this->jointsPosition_mov = h_results->trajectory_stages;
            this->jointsVelocity_mov = h_results->velocity_stages;
            this->jointsAcceleration_mov = h_results->acceleration_stages;
            solved=true;
        }else{
            ui.tableWidget_sol_mov->clear();//ui.listWidget_sol_mov->clear();
            qnode.log(QNode::Error,std::string("The planning has failed: ")+h_results->status_msg);
        }

        break;
    case 1: // RRT
        mRRTdlg->setInfo(prob->getInfoLine());
        // configuration
        m_params.config = mRRTdlg->getConfig();
        // pick/place settings
        mRRTdlg->getPreGraspApproach(m_params.pre_grasp_approach); // pick approach
        mRRTdlg->getPostGraspRetreat(m_params.post_grasp_retreat); // pick retreat
        mRRTdlg->getPrePlaceApproach(m_params.pre_place_approach); // place approach
        mRRTdlg->getPostPlaceRetreat(m_params.post_place_retreat); // place retreat

        tol_stop = mRRTdlg->getTolStop(); // stop tolerance on the joints when executing the movements

        m_results = prob->solve(m_params); // plan the movement

        break;


    } // switch planners

if(solved){
    QStringList h_headers; bool h_head=false; QStringList v_headers;
    double mov_duration = 0;
    std::vector<std::vector<QString>> mov_steps;
    for (size_t k=0; k< this->jointsPosition_mov.size();++k){
        MatrixXd jointPosition_stage = this->jointsPosition_mov.at(k);
        MatrixXd jointVelocity_stage = this->jointsVelocity_mov.at(k);
        MatrixXd jointAcceleration_stage = this->jointsAcceleration_mov.at(k);
        mov_duration += this->timeSteps_mov.at(k)*(jointPosition_stage.rows()-1);
        this->tols_stop_mov.push_back(tol_stop);
        std::vector<QString> stage_step;
        for(int i =0; i< jointPosition_stage.rows(); ++i){
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
    }// movements
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
}

}catch (const std::string message){qnode.log(QNode::Error,std::string("Plan failure: ")+message);
}catch(const std::exception exc){qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());}


}




void MainWindow::on_pushButton_execMov_pressed()
{

    qnode.log(QNode::Info,std::string("Executing the movement . . ."));

}


void MainWindow::on_pushButton_execMov_clicked()
{

     qnode.execMovement(this->jointsPosition_mov,this->jointsVelocity_mov,this->timeSteps_mov, this->tols_stop_mov, this->curr_mov, this->curr_scene);

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


    qnode.execTask(this->jointsPosition_task,this->jointsVelocity_task,this->timeSteps_task, this->tols_stop_task, this->curr_task, this->curr_scene);


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
    this->timeSteps_task.clear();
    this->tols_stop_task.clear();
    this->jointsPosition_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsAcceleration_mov.clear();
    this->timeSteps_mov.clear();
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
        vector<double> timesteps_mov;
        vector<double> tols_stop_mov;

        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)==QChar('#')){

                // the previous stage has finished
                if(pos_stage.rows()!=0){
                    t_mov.push_back(pos_stage);
                    w_mov.push_back(vel_stage);
                    a_mov.push_back(acc_stage);
                }
                // the previous movement hs finished
                if(!t_mov.empty()){
                    this->jointsPosition_task.push_back(t_mov);
                    this->jointsVelocity_task.push_back(w_mov);
                    this->jointsAcceleration_task.push_back(a_mov);
                    this->timeSteps_task.push_back(timesteps_mov);
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
                }else if(QString::compare(plan_type,QString("LBKPIECE"),Qt::CaseInsensitive)==0){
                    plan_id=6;
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

                    problemPtr prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
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

                    problemPtr prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }else if(QString::compare(mov_type,QString("Disengage"),Qt::CaseInsensitive)==0){
                    mov_id=4;
                }else if(QString::compare(mov_type,QString("Go park"),Qt::CaseInsensitive)==0){
                    mov_id=5;
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

                    problemPtr prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,grip_id,prec),new Scenario(*(this->curr_scene.get()))));
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
                }
                // new stage in the movement
                pos_stage.resize(0,0);
                vel_stage.resize(0,0);
                acc_stage.resize(0,0);
                row=0;
            }else if(line.at(0)==QChar('t')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("time step"),Qt::CaseInsensitive)==0){
                    timesteps_mov.push_back(fields.at(1).toDouble());
                }else if(QString::compare(fields.at(0).simplified(),QString("tol stop"),Qt::CaseInsensitive)==0){
                    tols_stop_mov.push_back(fields.at(1).toDouble());
                }
            }else{
                pos_stage.conservativeResize(pos_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                vel_stage.conservativeResize(vel_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                acc_stage.conservativeResize(acc_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);

                QStringList fields = line.split(",");                               
                for(int i=0; i <fields.size();++i){
                    QStringList fields1 = fields.at(i).split("=");
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
        vector<MatrixXd> pos_mov; vector<MatrixXd> vel_mov; vector<MatrixXd> acc_mov; vector<double> tstep_mov;
        double task_duration = 0.0; double mov_duration = 0.0;
        for(size_t h=0; h< this->jointsPosition_task.size();++h){
            pos_mov = this->jointsPosition_task.at(h);
            vel_mov = this->jointsVelocity_task.at(h);
            acc_mov = this->jointsAcceleration_task.at(h);
            tstep_mov = this->timeSteps_task.at(h);
            mov_duration = 0;
            for (size_t k=0; k< pos_mov.size();++k){
                MatrixXd jointPosition_stage = pos_mov.at(k);
                MatrixXd jointVelocity_stage = vel_mov.at(k);
                MatrixXd jointAcceleration_stage = acc_mov.at(k);
                mov_duration += tstep_mov.at(k)*(jointPosition_stage.rows()-1);
                std::vector<QString> stage_step;
                for(int i =0; i< jointPosition_stage.rows(); ++i){
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
            }// movements
            task_duration +=mov_duration;
        }//task

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
        for(int i=0; i <ui.listWidget_movs->count(); ++i){
            stream << "# " << ui.listWidget_movs->item(i)->text().toStdString().c_str()<< endl;
            vector< MatrixXd > traj_mov = this->jointsPosition_task.at(i);
            vector< MatrixXd > vel_mov = this->jointsVelocity_task.at(i);
            vector< MatrixXd > acc_mov = this->jointsAcceleration_task.at(i);
            vector< double > timesteps_mov = this->timeSteps_task.at(i);
            vector< double > tols_stop_mov = this->tols_stop_task.at(i);

            for(size_t j=0;j < traj_mov.size(); ++j){
                stream << "Movement stage: "<< QString::number(j+1).toStdString().c_str()<< endl;
                MatrixXd traj = traj_mov.at(j);
                MatrixXd vel = vel_mov.at(j);
                MatrixXd acc = acc_mov.at(j);
                double timestep = timesteps_mov.at(j);
                double tol_stop = tols_stop_mov.at(j);
                stream << "time step="<< QString::number(timestep).toStdString().c_str()<< endl;
                stream << "tol stop="<< QString::number(tol_stop).toStdString().c_str()<< endl;

                for(int r=0; r < traj.rows(); ++r){
                    stream << "step="<< QString::number(r).toStdString().c_str()<<", ";
                    for(int c=0; c < traj.cols(); ++c){
                        stream << "Joint "<<QString::number(c+1).toStdString().c_str()<<"="<<
                                  QString::number(traj(r,c)*180/M_PI,'g',3).toStdString().c_str()<<"|"<<
                                  QString::number(vel(r,c)*180/M_PI,'g',3).toStdString().c_str()<<"|"<<
                                  QString::number(acc(r,c)*180/M_PI,'g',3).toStdString().c_str();
                        if(c==traj.cols()-1){stream << endl;}else{stream<<", ";}
                    }// for loop columns
                }// for loop rows
            }// for loop movement
        }// for loop task
        stream << "#END" <<endl;

    }
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
    this->timeSteps_mov.clear();
    this->tols_stop_mov.clear();

    this->curr_scene = scenarioPtr(new Scenario(*(this->init_scene.get())));
    qnode.resetSimTime();
    qnode.resetGlobals();
    qnode.log(QNode::Info,std::string("Tha scenario has been reset"));

    int scene_id = this->scenario_id;
    string path;
    string title;
    string success;
    string failure;
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
        // Assistive scenario: beverages with ARoS
        //TO DO
        break;
    case 3:
        // Assistive scenario: beverages with Avatar
        //TO DO
        break;
    case 4:
        // Organizing scenario: shelfs and objects with ARoS
        //TO DO
        break;
    case 5:
        // Organizing scenario: shelfs ad objects with Avatar
        //TO DO

        break;
    }

    if (qnode.loadScenario(path,1)){
        qnode.log(QNode::Info,success);
        ui.groupBox_getElements->setEnabled(true);
        ui.groupBox_homePosture->setEnabled(true);

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
        this->timeSteps_task.push_back(this->timeSteps_mov);
        this->tols_stop_task.push_back(this->tols_stop_mov);


         QStringList h_headers; bool h_head=false; QStringList v_headers;
         std::vector<std::vector<QString>> task_steps;
         vector<MatrixXd> pos_mov; vector<MatrixXd> vel_mov; vector<MatrixXd> acc_mov; vector<double> tstep_mov;
         double task_duration = 0.0; double mov_duration = 0.0;
         for(size_t h=0; h< this->jointsPosition_task.size();++h){
             pos_mov = this->jointsPosition_task.at(h);
             vel_mov = this->jointsVelocity_task.at(h);
             acc_mov = this->jointsAcceleration_task.at(h);
             tstep_mov = this->timeSteps_task.at(h);
             mov_duration = 0;
             for (size_t k=0; k< this->jointsPosition_mov.size();++k){
                 MatrixXd jointPosition_stage = pos_mov.at(k);
                 MatrixXd jointVelocity_stage = vel_mov.at(k);
                 MatrixXd jointAcceleration_stage = acc_mov.at(k);
                 mov_duration += tstep_mov.at(k)*(jointPosition_stage.rows()-1);
                 std::vector<QString> stage_step;
                 for(int i =0; i< jointPosition_stage.rows(); ++i){
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
             }// movements
             task_duration +=mov_duration;
         }//task

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

    }



}


void MainWindow::on_pushButton_clear_task_clicked()
{

    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->jointsAcceleration_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsPosition_mov.clear();
    this->timeSteps_task.clear();
    this->timeSteps_mov.clear();
    this->tols_stop_task.clear();
    this->tols_stop_mov.clear();

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
            ui.comboBox_objects->setEnabled(true);
            ui.comboBox_objects_eng->setEnabled(false);
            ui.label_objects->setEnabled(true);
            ui.groupBox_grip->setEnabled(true);
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

            case(0):
                // Assembly scenario: the Toy vehicle with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS has to assemble a toy vehicle on a table in front of him"));
                break;

            case(1):
                //Assistive scenario: beverages with ARoS

                break;

            case(2):
                //Organizing scenario: shelfs and objects with ARoS

                break;

            }

        }

    }
#endif

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





}  // namespace motion_manager

