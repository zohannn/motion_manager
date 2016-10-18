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

namespace motion_manager {

using namespace Qt;
using namespace std;

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

    //create Tuning dialog
    mTolHumldlg = new TolDialogHUML(this);
    mTolHumldlg->setModal(true);


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

    mTolHumldlg->show();
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

             switch(i){

             case 0:

#if HAND == 0
             // Assembly scenario: the Toy vehicle with Jarde
             this->scenario_id = 1;

             if (qnode.loadScenario(MAIN_PATH+string("/scenarios/ToyVehicleTask_jarde.ttt"),this->scenario_id)){
                 qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with Jarde HAS BEEN LOADED"));
                 ui.groupBox_getElements->setEnabled(true);
                 ui.groupBox_homePosture->setEnabled(true);
                 //ui.pushButton_loadScenario->setEnabled(false);
                 string title = string("Assembly scenario: the Toy vehicle with Jarde");
                 this->hum_planner = humplannerPtr(new HUMPlanner(title, new Scenario(title,this->scenario_id+1), new Task()));

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

                 if (qnode.loadScenario(MAIN_PATH+string("/scenarios/ToyVehicleTask_aros.ttt"),this->scenario_id)){
                     qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED"));
                     ui.groupBox_getElements->setEnabled(true);
                     ui.groupBox_homePosture->setEnabled(true);
                     //ui.pushButton_loadScenario->setEnabled(false);
                     string title = string("Assembly scenario: the Toy vehicle with ARoS");
                     this->hum_planner = humplannerPtr(new HUMPlanner(title, new Scenario(title,this->scenario_id+1), new Task()));

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
    ui.listWidget_sol_mov->clear();
    ui.label_totalTime_value_mov->clear();
}



void MainWindow::on_pushButton_getElements_clicked()
{

scenarioPtr scene = this->hum_planner->getScenario();

    try{
        if (qnode.getElements(scene)){

            this->hum_planner->setScenario(scene); //update the scenario
            this->init_scene = scenarioPtr(new Scenario(*scene.get())); // set the initial scene

            ui.pushButton_getElements->setEnabled(false);
            ui.tab_plan->setEnabled(true);
            ui.groupBox_specs->setEnabled(true);
            ui.groupBox_task->setEnabled(false);
            ui.tabWidget_sol->setEnabled(false);
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

    int rows = ui.listWidget_movs->count();
    //problemPtr prob = hum_planner->getProblem(row); //selected problem
    bool add = true;
    if (rows > 0){
        problemPtr pre_prob = hum_planner->getProblem(rows-1); //previous problem
        if(!pre_prob->getMovement()->getExecuted()){
            int reply = QMessageBox::warning(this,tr("Warning"),
                                 tr("<p>The previous movement has not been executed yet. Do you really want to add this movement?</p>"),QMessageBox::Yes,QMessageBox::No);
           add = (reply == QMessageBox::Yes);
        }
    }

    if(add){

        bool success = false;
        int mov_id = ui.comboBox_mov->currentIndex();
        //int plan_id = ui.comboBox_planner->currentIndex();
        int aa;
        if (ui.comboBox_Task->currentIndex()==0){
            //single-arm
            if(ui.radioButton_right->isChecked()){
                aa=1; // right arm
            }else{
                aa=2; // left arm
            }
        }else{
            //dual-arm
            aa=0;
        }


        // add the movement
        scenarioPtr scene = this->hum_planner->getScenario();

         if (ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled()){

             // engage, disengage movements

            //int obj_id = ui.comboBox_objects->currentIndex();
            string obj_name = ui.comboBox_objects->currentText().toStdString();
            //int obj_eng_id = ui.comboBox_objects_eng->currentIndex();
            string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString();

            //objectPtr obj = scene->getObject(obj_id);
            objectPtr obj = scene->getObject(obj_name);
            //objectPtr obj_eng = scene->getObject(obj_eng_id);
            objectPtr obj_eng = scene->getObject(obj_eng_name);

            if(obj!=NULL && obj_eng!=NULL){
                int grip_id = ui.comboBox_grip->currentIndex();
                bool prec = ui.radioButton_prec->isChecked();
                //bool full = ui.radioButton_full->isChecked();
                switch (aa){
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
                hum_planner->addProblem(movementPtr(new Movement(mov_id, aa, obj,obj_eng,grip_id,prec)));
                success=true;

            }else{
                qnode.log(QNode::Error,std::string("The movement requires two objects"));
            }

         }else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled()){

             if(std::strcmp(ui.comboBox_mov->currentText().toStdString().c_str(),"Go-home")!=0){
                  // reach-to- grasp movement, transport movements

                 //int obj_id = ui.comboBox_objects->currentIndex();
                 string obj_name = ui.comboBox_objects->currentText().toStdString();

                 //objectPtr obj = scene->getObject(obj_id);
                 objectPtr obj = scene->getObject(obj_name);

                 if(obj!=NULL){
                     int grip_id = ui.comboBox_grip->currentIndex();
                     bool prec = ui.radioButton_prec->isChecked();
                     //bool full = ui.radioButton_full->isChecked();
                     switch (aa){
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
                     hum_planner->addProblem(movementPtr(new Movement(mov_id, aa, obj,grip_id,prec)));
                     success=true;

                 }else{
                     qnode.log(QNode::Error,std::string("The movement requires an object"));

                 }

             }else{

                 // go home movements
                 //int obj_id = ui.comboBox_objects->currentIndex();
                 string obj_name = ui.comboBox_objects->currentText().toStdString();
                 //objectPtr obj = scene->getObject(obj_id);
                 objectPtr obj = scene->getObject(obj_name);
                 if(obj!=NULL){
                     int grip_id = ui.comboBox_grip->currentIndex();
                     bool prec = ui.radioButton_prec->isChecked();
                     //bool full = ui.radioButton_full->isChecked();
                     switch (aa){
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
                     hum_planner->addProblem(movementPtr(new Movement(mov_id, aa, obj,grip_id,prec)));
                     success=true;

                 }else{
                    hum_planner->addProblem(movementPtr(new Movement(mov_id, aa)));
                    success=true;

                 }
             }


         }else{

             // reaching movements
             hum_planner->addProblem(movementPtr(new Movement(mov_id, aa)));
             success=true;

         }

         if(success){
             qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
             ui.groupBox_task->setEnabled(true);
             ui.listWidget_movs->clear();
             for (int i = 0; i < hum_planner->getProblemNumber();i++ ){
                ui.listWidget_movs->addItem(QString(hum_planner->getProblemInfo(i).c_str()));
             }
             ui.listWidget_movs->setCurrentRow(ui.listWidget_movs->count()-1);
             this->hum_planner->setScenario(scene); // update the scenario
         }


    }


}


void MainWindow::on_pushButton_plan_clicked()
{

    ui.tabWidget_sol->setCurrentIndex(0);    
    problemPtr prob = hum_planner->getProblem(ui.listWidget_movs->currentRow());

        mTolHumldlg->setInfo(prob->getInfoLine());
        try{

            Tols  tols;
            // --- Tolerances for the final posture selection ---- //

            // tolerances of the arm : radius in [mm]
            mTolHumldlg->getTolsArm(tols.tolsArm);
            // tolerances of the hand: radius in [mm]
            mTolHumldlg->getTolsHand(tols.tolsHand);
            // tolerances with the table
            //mTolHumldlg->getTolsTable(tols.tols_table);
            // target avoidance
            tols.target_avoidance = mTolHumldlg->getTargetAvoidance();
            //obstacle avoidance
            tols.obstacle_avoidance = mTolHumldlg->getObstacleAvoidance();
            // joint expense factors
            std::vector<float> lambda;
            mTolHumldlg->getLambda(lambda);
            for (int i = 0; i < JOINTS_ARM; ++i){
                tols.lambda_final.push_back(lambda.at(i));
            }
            // --- Tolerances for the bounce posture selection ---- //

            for (int i=0; i < JOINTS_ARM + JOINTS_HAND; ++i){
                tols.bounds.vel_0.push_back(0);
                tols.bounds.vel_f.push_back(0);
                tols.bounds.acc_0.push_back(0);
                tols.bounds.acc_f.push_back(0);
                tols.lambda_bounce.push_back(lambda.at(i));
                tols.w_max.push_back(mTolHumldlg->getWMax()); // [deg/sec]
            }

            // tolerances for the obstacles
            tols.final_tolsObstacles = MatrixXf::Constant(3,6,1) ;
            mTolHumldlg->getTolsObstacles(tols.final_tolsObstacles);

            tols.singleArm_tolsObstacles.push_back(MatrixXf::Constant(3,6,1));
            tols.singleArm_tolsObstacles.push_back(MatrixXf::Constant(3,6,1));

            mTolHumldlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
            mTolHumldlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));

            // tolerances for the target
            tols.singleArm_tolsTarget.push_back(MatrixXf::Constant(3,6,1));
            tols.singleArm_tolsTarget.push_back(MatrixXf::Constant(3,6,1));
            tols.singleArm_tolsTarget.push_back(MatrixXf::Constant(3,6,1));

            mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
            mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(1));
            mTolHumldlg->getTolsTarget(tols.singleArm_tolsTarget.at(2));


            // number of steps
            tols.steps = mTolHumldlg->getSteps();

            // tol Tar pos and tol tar or
            tols.tolTarPos = mTolHumldlg->getTolTarPos();
            tols.tolTarOr = mTolHumldlg->getTolTarOr();

            // set the approaching target axis
            prob->setApproachingTargetAxis(mTolHumldlg->getApproachAxis());

            //engaging parametrs
            mTolHumldlg->getEngageParams(tols.eng_dist,tols.eng_dir,tols.eng_tols);

            //disengaging parameters
            mTolHumldlg->getDisengageParams(tols.diseng_dist,tols.diseng_dir);

            // tol stop
            tols.tol_stop = mTolHumldlg->getTolStop();

            bool solved = prob->solve(tols); // plan the movement

            if(solved){


                ui.pushButton_plan->setCheckable(false);

                qnode.log(QNode::Info,std::string("The movement has been planned successfully"));


                MatrixXf traj;
                prob->getTrajectory(traj);


                MatrixXf vel;
                float timeStep = prob->getVelocity(vel);
                std::vector<string> velStep = std::vector<string>(vel.rows());
                for (int i=0; i<vel.rows();++i){
                    velStep.at(i) = "Step="+QString::number(i).toStdString()+",";
                    for (int j=0; j<vel.cols();++j){
                        velStep.at(i) = velStep.at(i)+
                                " Joint "+QString::number(j+1).toStdString()+"="+
                                QString::number(traj(i,j)*180/M_PI).toStdString()+"|"+
                                QString::number(vel(i,j)*180/M_PI).toStdString()+", ";
                    }
                    ui.listWidget_sol_mov->addItem(QString(velStep.at(i).c_str()));
                }


                ui.label_totalTime_value_mov->setText(QString::number(timeStep*tols.steps).toStdString().c_str());
                ui.tabWidget_sol->setEnabled(true);

                this->timeStep=timeStep;
                this->jointsVelocity_mov = vel;
                this->mov = prob->getMovement();

                this->jointsPosition_mov = traj;

            }else{

                ui.listWidget_sol_mov->clear();
                ui.pushButton_plan->setCheckable(false);

                int err_log = prob->getErrLog();
                std::string msg;

                switch(err_log){

                case 0:

                    msg="unknown problem";
                    break;

                case 10:

                    msg="the final posture problem of the reach-to-grasp movement has not been solved";
                    break;

                case 20:

                    msg="the bounce posture problem of the reach-to-grasp movement has not been solved";
                    break;

                case 130:

                    msg="the final posture problem of the sub-disengage movement has not been solved";
                    break;

                case 13:

                    msg="the final posture problem of the engage movement has not been solved";
                    break;

                case 131:

                    msg="the final posture problem of the sub-engage movement has not been solved";
                    break;

                case 23:

                    msg="the bounce posture problem of the engage movement has not been solved";
                    break;

                case 25:

                    msg="the bounce posture problem of the go-home movement has not been solved";
                    break;

                default:

                    msg="unknown problem";
                    break;

                }

                qnode.log(QNode::Error,std::string("The planning has failed: ")+msg);


            }

        }
        catch (const std::string message){

            qnode.log(QNode::Error,std::string("Plan failure: ")+message);

        }
        catch(const std::exception exc){

            qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());
        }



}


void MainWindow::on_pushButton_execMov_pressed()
{

    qnode.log(QNode::Info,std::string("Executing the movement . . ."));

}


void MainWindow::on_pushButton_execMov_clicked()
{

    MatrixXf vel = this->jointsVelocity_mov;
    MatrixXf traj = this->jointsPosition_mov;
    float timeStep = this->timeStep;
    float tol_stop = this->mTolHumldlg->getTolStop();

    qnode.execMovement(traj,vel,timeStep, tol_stop, this->mov, this->hum_planner->getScenario());
    //qnode.execMovement(traj,timeStep, this->mov, this->hum_planner->getScenario());


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

    MatrixXf traj = this->jointsPosition_task;
    MatrixXf vel = this->jointsVelocity_task;
    std::vector<float> tSteps = this->timeSteps_task;
    std::vector<int> nSteps = this->nSteps_task;
    std::vector<float> stops = this->tols_stop;

    qnode.execTask(traj,vel,tSteps,nSteps,stops,this->hum_planner->getTask(),this->hum_planner->getScenario());


}


void MainWindow::on_pushButton_load_task_clicked()
{

    int mov_id; QString mov_type;
    int arm_code; QString arm_type;
    int grip_id; QString grip_type;
    QString obj_str; objectPtr obj;
    QString obj_eng_str; objectPtr obj_eng;
    bool prec;
    int steps=0;
    int row=0;
    MatrixXf vel;
    MatrixXf poss;

    //clear
    this->jointsVelocity_task.resize(0,0);
    this->jointsPosition_task.resize(0,0);
    this->jointsVelocity_mov.resize(0,0);
    this->jointsPosition_mov.resize(0,0);
    this->timeStep=0.0;
    this->timeSteps_task.clear();
    this->nSteps_task.clear();
    this->tols_stop.clear();

    ui.listWidget_sol_task->clear();
    ui.listWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->hum_planner->clearProblems();

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the task trajectory"),
                                                    QString(MAIN_PATH)+"/Tasks",
                                                    "All Files (*.*);; Task Files (*.task)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;
        int prev_steps=0;


        while(!stream.atEnd()){

            line = f.readLine();
            if(line.at(0)==QChar('#')){

                if(steps!=0){
                    this->nSteps_task.push_back(steps);
                    prev_steps+=steps+1;
                    steps=0;
                }
                if((line.at(1)==QChar('E')) && (line.at(2)==QChar('N')) && (line.at(3)==QChar('D'))){break;}
                QStringList fields = line.split(",");
                for(int i=0; i< fields.size(); ++i){
                    QStringList fields1 = fields.at(i).split(":");
                    if (QString::compare(fields1.at(0).simplified(),QString("Humanoid"),Qt::CaseInsensitive)==0){
                       // do nothing
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
                    obj = this->hum_planner->getScenario()->getObject(obj_str.toStdString());
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

                    problemPtr prob = problemPtr(new Problem(new Movement(mov_id, arm_code, obj,grip_id,prec),
                                                             new Scenario(*(this->hum_planner->getScenario().get()))));
                    prob->setSolved(true);
                    prob->setPartOfTask(true);

                    this->hum_planner->addProblem(prob);

                }else if(QString::compare(mov_type,QString("Reaching"),Qt::CaseInsensitive)==0){
                    mov_id=1;
                }else if(QString::compare(mov_type,QString("Transport"),Qt::CaseInsensitive)==0){
                    mov_id=2;
                }else if(QString::compare(mov_type,QString("Engage"),Qt::CaseInsensitive)==0){
                    mov_id=3;
                    //get the object
                    obj = this->hum_planner->getScenario()->getObject(obj_str.toStdString());
                    // get the object engaged
                    obj_eng = this->hum_planner->getScenario()->getObject(obj_eng_str.toStdString());
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

                    problemPtr prob = problemPtr(new Problem(new Movement(mov_id, arm_code, obj,obj_eng,grip_id,prec),
                                                             new Scenario(*(this->hum_planner->getScenario().get()))));
                    prob->setSolved(true);
                    prob->setPartOfTask(true);

                    this->hum_planner->addProblem(prob);

                }else if(QString::compare(mov_type,QString("Disengage"),Qt::CaseInsensitive)==0){
                    mov_id=4;
                }else if(QString::compare(mov_type,QString("Go home"),Qt::CaseInsensitive)==0){
                    mov_id=5;
                    //get the object
                    obj = this->hum_planner->getScenario()->getObject(obj_str.toStdString());
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

                    problemPtr prob = problemPtr(new Problem(new Movement(mov_id, arm_code, obj,grip_id,prec),
                                                             new Scenario(*(this->hum_planner->getScenario().get()))));
                    prob->setSolved(true);
                    prob->setPartOfTask(true);

                    this->hum_planner->addProblem(prob);

                }

                //logging
                qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
                ui.groupBox_task->setEnabled(true);
                ui.listWidget_movs->clear();
                for (int i = 0; i < hum_planner->getProblemNumber();i++ ){
                   ui.listWidget_movs->addItem(QString(hum_planner->getProblemInfo(i).c_str()));
                }
                ui.listWidget_movs->setCurrentRow(0);


            }else if(line.at(0)==QChar('t')){

                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("time step"),Qt::CaseInsensitive)==0){
                    this->timeSteps_task.push_back(fields.at(1).toFloat());
                }else if(QString::compare(fields.at(0).simplified(),QString("tol stop"),Qt::CaseInsensitive)==0){
                    this->tols_stop.push_back(fields.at(1).toFloat());
                }
            }else{
                poss.conservativeResize(poss.rows()+1,JOINTS_ARM+JOINTS_HAND);
                vel.conservativeResize(vel.rows()+1,JOINTS_ARM+JOINTS_HAND);

                QStringList fields = line.split(",");                               
                for(int i=0; i <fields.size();++i){
                    QStringList fields1 = fields.at(i).split("=");
                    if(QString::compare(fields1.at(0).simplified(),QString("Step"),Qt::CaseInsensitive)==0){
                        steps=fields1.at(1).toInt()-prev_steps;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 1"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,0)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,0)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 2"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,1)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,1)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 3"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,2)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,2)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 4"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,3)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,3)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 5"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,4)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,4)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 6"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,5)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,5)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 7"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,6)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,6)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 8"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,7)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,7)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 9"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,8)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,8)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 10"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,9)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,9)=fields2.at(1).toFloat()*M_PI/180;
                    }else if(QString::compare(fields1.at(0).simplified(),QString("Joint 11"),Qt::CaseInsensitive)==0){
                        QStringList fields2 = fields1.at(1).split("|");
                        poss(row,10)=fields2.at(0).toFloat()*M_PI/180;
                        vel(row,10)=fields2.at(1).toFloat()*M_PI/180;
                    }
                }
                row++;
            }

        } // while loop

        this->jointsPosition_task=poss;
        this->jointsVelocity_task=vel;

        qnode.log(QNode::Info,std::string("The task has been loaded"));

        this->vel_steps = std::vector<string>(vel.rows());
        for (int i=0; i<vel.rows();++i){
            this->vel_steps.at(i) ="Step="+QString::number(i).toStdString()+",";
            for (int j=0; j<vel.cols();++j){

                this->vel_steps.at(i) = this->vel_steps.at(i)+
                        " Joint "+QString::number(j+1).toStdString()+"="+
                        QString::number(poss(i,j)*180/M_PI).toStdString()+"|"+
                        QString::number(vel(i,j)*180/M_PI).toStdString()+", ";
            }
            ui.listWidget_sol_task->addItem(QString(this->vel_steps.at(i).c_str()));
        }
        float totalTime = 0;
        for (int i = 0; i < this->timeSteps_task.size(); ++i){
            totalTime += this->timeSteps_task.at(i)*this->nSteps_task.at(i);
        }
        ui.label_totalTime_value_task->setText(QString::number(totalTime).toStdString().c_str());

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
        std::vector<float> timeSteps = this->timeSteps_task;
        std::vector<int> steps = this->nSteps_task;
        std::vector<float> tols_stop = this->tols_stop;

        QTextStream stream( &f );
        int prev_steps=0;
        for(int i=0; i <ui.listWidget_movs->count(); ++i){

            if(i!=0){prev_steps += steps.at(i-1)+1;}

            stream << "# " << ui.listWidget_movs->item(i)->text().toStdString().c_str()<< endl;
            stream << "time step="<< QString::number(timeSteps.at(i)).toStdString().c_str()<< endl;
            stream << "tol stop="<< QString::number(tols_stop.at(i)).toStdString().c_str()<< endl;

            for(int j=0; j< steps.at(i)+1; ++j){

                stream << ui.listWidget_sol_task->item(j+prev_steps)->text().toStdString().c_str()<< endl;


            }
        }
        stream << "#END" <<endl;

    }
    f.close();


}


void MainWindow::on_pushButton_scene_reset_clicked()
{

    // reset the movements
    ui.listWidget_sol_mov->clear();
    //ui.listWidget_movs->clear();
    ui.label_totalTime_value_mov->clear();
    this->jointsVelocity_mov.resize(0,0);
    this->jointsPosition_mov.resize(0,0);

    this->hum_planner->setScenario(this->init_scene);
    qnode.resetSimTime();
    qnode.resetGlobals();
    qnode.log(QNode::Info,std::string("Tha scenario has been reset"));

    int scene_id = this->scenario_id;
    string path;
    string title;
    string success;
    string failure;

    switch(scene_id){

    case 0:
        // Assembly scenario: the Toy vehicle with ARoS
        path = string("scenes/ToyVehicleTask_aros.ttt");
        title = string("Assembly scenario: the Toy vehicle with ARoS");
        success = string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with ARoS HAS NOT BEEN LOADED");

        break;
    case 1:
        // Assembly scenario: the Toy vehicle with Avatar

        path = string("scenes/ToyVehicleTask_jarde.ttt");
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

    ui.pushButton_save_task->setEnabled(true);

    if(hum_planner->getProblem(ui.listWidget_movs->currentRow())->getSolved()){

     this->tols_stop.push_back(this->mTolHumldlg->getTolStop());

     MatrixXf vel_mov = this->jointsVelocity_mov;
     MatrixXf vel_task = this->jointsVelocity_task;

     MatrixXf pos_mov = this->jointsPosition_mov;
     MatrixXf pos_task = this->jointsPosition_task;

     this->timeSteps_task.push_back(this->timeStep);
     this->nSteps_task.push_back(vel_mov.rows()-1);

     if (vel_task.rows()==0){
         this->jointsVelocity_task = vel_mov;
         this->jointsPosition_task = pos_mov;
     }else{
        int vel_rows = vel_task.rows();
        int pos_rows = pos_task.rows();

        vel_task.conservativeResize(vel_rows+vel_mov.rows(),vel_task.cols());
        for(int i = 0; i < vel_mov.rows();++i){
            vel_task.row(vel_rows+i) = vel_mov.row(i);
        }
        this->jointsVelocity_task = vel_task;

        pos_task.conservativeResize(pos_rows+pos_mov.rows(),pos_task.cols());
        for(int i = 0; i < pos_mov.rows();++i){
            pos_task.row(pos_rows+i) = pos_mov.row(i);
        }
        this->jointsPosition_task = pos_task;
     }

     ui.listWidget_sol_task->clear();
     MatrixXf poss = this->jointsPosition_task;
     MatrixXf vel = this->jointsVelocity_task;
     this->vel_steps = std::vector<string>(vel.rows());
     for (int i=0; i<vel.rows();++i){
         this->vel_steps.at(i) ="Step="+QString::number(i).toStdString()+",";
         for (int j=0; j<vel.cols();++j){

             this->vel_steps.at(i) = this->vel_steps.at(i)+
                     " Joint "+QString::number(j+1).toStdString()+"="+
                     QString::number(poss(i,j)*180/M_PI).toStdString()+"|"+
                     QString::number(vel(i,j)*180/M_PI).toStdString()+", ";
         }
         ui.listWidget_sol_task->addItem(QString(this->vel_steps.at(i).c_str()));
     }
     float totalTime = 0;
     for (int i = 0; i < this->timeSteps_task.size(); ++i){
         totalTime += this->timeSteps_task.at(i)*this->nSteps_task.at(i);
     }
     ui.label_totalTime_value_task->setText(QString::number(totalTime).toStdString().c_str());
     ui.tabWidget_sol->setCurrentIndex(1);

     // set part of the task
     hum_planner->getProblem(ui.listWidget_movs->currentRow())->setPartOfTask(true);

    }



}

void MainWindow::on_pushButton_clear_task_clicked()
{

    this->jointsVelocity_task.resize(0,0);
    this->jointsPosition_task.resize(0,0);
    this->jointsVelocity_mov.resize(0,0);
    this->jointsPosition_mov.resize(0,0);
    this->timeStep=0.0;
    this->timeSteps_task.clear();
    this->nSteps_task.clear();

    ui.listWidget_sol_task->clear();
    ui.listWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->hum_planner->clearProblems();

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
        // Go home
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

