/**
 * @file /include/motion_manager/main_window.hpp
 *
 * @brief Qt based gui for motion_manager.
 *
 * @date November 2010
 **/
#ifndef motion_manager_MAIN_WINDOW_H
#define motion_manager_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QtGui/QListWidgetItem>
#include <boost/smart_ptr.hpp>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "roscommdialog.hpp"
#include "vrepcommdialog.hpp"
#include "toldialoghuml.hpp"


// *** HUML *** //
#include <humplanner.hpp>
// ************ //

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace motion_manager {

using namespace std;

typedef boost::shared_ptr<HUMotion::HUMPlanner> humplannerPtr;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
        void on_actionAbout_triggered(); // About Dialog
        void on_actionRos_Communication_triggered(); // Ros Comm Dialog
        void on_actionVrep_Communication_triggered(); // Vrep Comm Dialog
        void on_pushButton_loadScenario_clicked(); // load the scenario
        void on_pushButton_getElements_clicked(); // get the elements of the scenario
        void on_pushButton_getElements_pressed(); // get the elements of the scenario
        void on_pushButton_addMov_clicked(); // add a movement to the task
        void on_comboBox_Task_currentIndexChanged(int i); // select the type of task
        void on_comboBox_mov_currentIndexChanged(int i); // select the type of movement
        void on_pushButton_plan_clicked(); // plan the selected movement
        void on_pushButton_plan_pressed(); // plan the selected movement
        void on_pushButton_execMov_clicked(); // execute the movement
        void on_pushButton_execMov_pressed(); // execute the movement
        void on_pushButton_scene_reset_clicked(); // reset the scene
        void on_pushButton_tuning_clicked(); // tuning the optimization problem
        void on_pushButton_append_mov_clicked(); // append the movement to the task
        void on_pushButton_clear_task_clicked(); //clear the current task
        void on_pushButton_execTask_clicked(); // execute the current task
        void on_pushButton_execTask_pressed(); // execute the current task
        void on_pushButton_save_task_clicked(); // save the current task
        void on_pushButton_load_task_clicked();  // load the current task
        void on_pushButton_stop_mov_clicked(); // stop the simulation
        void on_pushButton_stop_task_clicked(); // stop the simulation

        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView(); // no idea why this can't connect automatically
        void updateRosStatus(bool c);
        void updateVrepStatus(bool c);
        void addElement(string value);
        void updateElement(int id,string value);
        void addObject(string value);
        void updateHomePosture(string value);
        void onListScenarioItemClicked(QListWidgetItem* item);



private:
	Ui::MainWindowDesign ui;
        QNode qnode;
        RosCommDialog *mrosCommdlg;
        VrepCommDialog *mvrepCommdlg;
        TolDialogHUML *mTolHumldlg;
        int scenario_id;

        humplannerPtr hum_planner; // human-like upper-limbs planner
        float timeStep; // current tiemStep
        MatrixXf jointsVelocity_mov; // joint velocity of the movement
        MatrixXf jointsPosition_mov; // joint position of the movement
        MatrixXf jointsVelocity_task; // joint velocity of the task
        MatrixXf jointsPosition_task; // joint position of the task
        std::vector<float> timeSteps_task; // vector of time steps of each movement
        std::vector<int> nSteps_task; // vector of number of steps of each movement
        std::vector<string> vel_steps; // steps of the trajectory for the file
        std::vector<float> tols_stop; // vector of the tolerances to stop the movement
        movementPtr mov; // current movement
        scenarioPtr init_scene; // initial scene

        // --- Home postures for ARoS --- //

        // Right arm [deg]
        // Joint 0 = -137.500
        // Joint 1 = -77.94 (12.06 - 90)
        // Joint 2 = 106.5900
        // Joint 3 = -95.42 (-5.4200 -90)
        // Joint 4 = -43.2800
        // Joint 5 = -64.000
        // Joint 6 = 47.970
        // Joint 7 = 0.0
        // Joint 8 = 70.0
        // Joint 9 = 70.0
        // Joint 10 = 70.0

        // Left arm [deg]
        // Joint 0 = 137.500
        // Joint 1 = -77.94 (12.0600 - 90)
        // Joint 2 = -106.5900
        // Joint 3 =  -95.42 (-5.4200 - 90)
        // Joint 4 = 43.2800
        // Joint 5 = -64.000
        // Joint 6 = 132.03
        // Joint 7 = 0.0
        // Joint 8 = 70.0
        // Joint 9 = 70.0
        // Joint 10 = 70.0

        // --- Home postures for Jarde --- //

        // Right arm [deg]
        // Joint 0 = -90
        // Joint 1 = -80
        // Joint 2 =  90
        // Joint 3 =   0
        // Joint 4 =   0
        // Joint 5 =   0
        // Joint 6 =   0
        // Joint 7 =   0
        // Joint 8 = 70.0
        // Joint 9 = 70.0
        // Joint 10 = 70.0

        // Left arm [deg]
        // Joint 0 = -90
        // Joint 1 = -80
        // Joint 2 =  90
        // Joint 3 =   0
        // Joint 4 =   0
        // Joint 5 =   0
        // Joint 6 =   0
        // Joint 7 =   0
        // Joint 8 = 70.0
        // Joint 9 = 70.0
        // Joint 10 = 70.0

};

}  // namespace motion_manager

#endif // motion_manager_MAIN_WINDOW_H
