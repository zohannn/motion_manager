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
#include "rvizcommdialog.hpp"
#include "toldialoghuml.hpp"
#include "config.hpp"

// *** Humanoid MoveIt! planner *** //
#include <aros_moveit_planner/humanoid_planner.hpp>
// ******************************************* //
// *** HUML *** //
#include <humplanner.hpp>
// ************ //


/** This is the main namespace of the program */
namespace motion_manager {

using namespace std;
//using namespace HUMotion;
//using namespace humanoid_planning;
typedef boost::shared_ptr<HUMotion::HUMPlanner> humplannerPtr; /**< shared pointer to a human-like motion planner */
typedef boost::shared_ptr<humanoid_planning::HumanoidPlanner> hplannerPtr; /**< shared point to a humanoid moveit planner */

//! The MainWindow class
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{

    Q_OBJECT

public:
        /**
         * @brief MainWindow, a contructor
         * @param argc
         * @param argv
         * @param parent
         */
	MainWindow(int argc, char** argv, QWidget *parent = 0);

        /**
         * @brief ~MainWindow, a destructor
         */
	~MainWindow();

        /**
         * @brief This method loads up qt program settings at startup
         */
        void ReadSettings();

        /**
         * @brief This method saves qt program settings when closing
         */
        void WriteSettings();

        /**
         * @brief This method menages the close event
         * @param event
         */
        void closeEvent(QCloseEvent *event);

public Q_SLOTS:

        /**
         * @brief This method shows the about dialog
         */
        void on_actionAbout_triggered();

        /**
         * @brief This method shows the ROS communication dialog
         */
        void on_actionRos_Communication_triggered();

        /**
         * @brief This method shows the V-REP communication dialog
         */
        void on_actionVrep_Communication_triggered();

        /**
         * @brief This method shows the RViz communication dialog
         */
        void on_actionRViz_Communication_triggered();

        /**
         * @brief This method loads the selected scenario
         */
        void on_pushButton_loadScenario_clicked();

        /**
         * @brief This method retrievies the information about
         * the elements in the scenario
         */
        void on_pushButton_getElements_clicked();

        /**
         * @brief This method retrievies the information about
         * the elements in the scenario
         */
        void on_pushButton_getElements_pressed();

        /**
         * @brief This method adds a movement to the task
         */
        void on_pushButton_addMov_clicked();

        /**
         * @brief This method selects the type of task
         * @param i
         * index of the task. Single-arm task: i=0. Dual-arm task: i=1;
         */
        void on_comboBox_Task_currentIndexChanged(int i);

        /**
         * @brief This method selects the type of task
         * @param i
         * <table>
         * <caption id="multi_row">Types of the movement</caption>
         * <tr><th>Type      <th>index
         * <tr><td>Reach-to-grasp <td>0
         * <tr><td>Reaching <td>1
         * <tr><td>Transport <td>2
         * <tr><td>Engage <td>3
         * <tr><td>Disengage <td>4
         * <tr><td>Go home <td>5
         * </table>
         */
        void on_comboBox_mov_currentIndexChanged(int i);

        /**
         * @brief This method plans the selected movement
         */
        void on_pushButton_plan_clicked();

        /**
         * @brief This method plans the selected movement
         */
        void on_pushButton_plan_pressed();

        /**
         * @brief This method executes the selected movement
         */
        void on_pushButton_execMov_clicked();

        /**
         * @brief This method executes the selected movement
         */
        void on_pushButton_execMov_pressed();

        /**
         * @brief This method reload the scenario and resets the related variables
         */
        void on_pushButton_scene_reset_clicked();

        /**
         * @brief This method shows the tuning dialog
         */
        void on_pushButton_tuning_clicked();

        /**
         * @brief This method appends the movement to the current task
         */
        void on_pushButton_append_mov_clicked();

        /**
         * @brief This method deletes the current task
         */
        void on_pushButton_clear_task_clicked();

        /**
         * @brief This method executes the current task
         */
        void on_pushButton_execTask_clicked();

        /**
         * @brief This method executes the current task
         */
        void on_pushButton_execTask_pressed();

        /**
         * @brief This method saves the current task to file
         */
        void on_pushButton_save_task_clicked();

        /**
         * @brief This method loads the current task from file
         */
        void on_pushButton_load_task_clicked();

        /**
         * @brief This method stops the execution of the movement
         */
        void on_pushButton_stop_mov_clicked();

        /**
         * @brief This method stops the execution of the task
         */
        void on_pushButton_stop_task_clicked();

        /**
         * This method is signalled by the underlying model. When the model changes,
         * this will drop the cursor down to the last line in the QListview to ensure
         * the user can always see the latest log message.
         */
        void updateLoggingView();

        /**
         * @brief This method updates the ROS status
         * @param c
         * c=true => "connected", c=false => "disconnected"
         */
        void updateRosStatus(bool c);

        /**
         * @brief This method updates the V-REP status
         * @param c
         * c=true => "connected", c=false => "disconnected"
         */
        void updateVrepStatus(bool c);

        /**
         * @brief This method updates the RViz status
         * @param c
         * c=true => "launched", c=false => "not launched"
         */
        void updateRVizStatus(bool c);

        /**
         * @brief This method adds a new element to the widget
         * @param value
         */
        void addElement(string value);

        /**
         * @brief This method updates the info of the element with index id
         * @param id
         * @param value
         */
        void updateElement(int id,string value);

        /**
         * @brief This method add an object to the lists of
         * available objects for manipulation
         * @param value
         */
        void addObject(string value);

        /**
         * @brief This methods updates the home posture of the humanoid
         * @param value
         */
        void updateHomePosture(string value);

        /**
         * @brief This method lists the available scenarios
         * @param item
         */
        void onListScenarioItemClicked(QListWidgetItem* item);



private:
        Ui::MainWindowDesign ui; /**< handles of the main user interface */
        QNode qnode; /**< ROS node handle */
        RosCommDialog *mrosCommdlg; /**< handle of the ROS communication dialog */
        VrepCommDialog *mvrepCommdlg; /**< handle of the V-REP communication dialog */
        RVizCommDialog *mrvizCommdlg; /**< handle of the RViz communication dialog */
        TolDialogHUML *mTolHumldlg; /**< handle of the tuning dialog */
        int scenario_id; /**< id of the current scenario */
        humplannerPtr hum_planner; /**< human-like upper-limbs movement planner */
        hplannerPtr h_planner; /**< humanoid moveit planner */
        float timeStep; /**< current time step of the trajectory */
        MatrixXf jointsVelocity_mov; /**< trajectory of the joint velocity of the movement */
        MatrixXf jointsPosition_mov; /**< trajectory of the joint position of the movement */
        MatrixXf jointsVelocity_task; /**< trajectory of the joint velocity of the task */
        MatrixXf jointsPosition_task; /**< trajectory of the joint position of the task */
        std::vector<float> timeSteps_task; /**< vector of time steps of each movement in the task */
        std::vector<int> nSteps_task; /**< vector of number of steps of each movement in the task */
        std::vector<string> vel_steps; /**< steps of the trajectory for saving/loading file */
        std::vector<float> tols_stop; /**< vector of the tolerances to stop each movement in the task */
        movementPtr mov; /**< current movement */
        scenarioPtr init_scene; /**< initial scenario */

        // --- Park postures for ARoS --- //

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

        // --- Park postures for Jarde --- //

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
