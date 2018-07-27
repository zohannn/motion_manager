
#ifndef motion_manager_MAIN_WINDOW_H
#define motion_manager_MAIN_WINDOW_H


/*****************************************************************************
** Includes
*****************************************************************************/
#include <qcustomplot.h>
#include <qcpdocumentobject.h>
#include <qwt3d_io.h>
#include <qwt3d_io_gl2ps.h>
#include <persistence1d.hpp>
#include <QtGui/QMainWindow>
#include <QtGui/QListWidgetItem>
#include <boost/smart_ptr.hpp>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "roscommdialog.hpp"
#include "vrepcommdialog.hpp"
#include "rvizcommdialog.hpp"
#include "toldialoghump.hpp"
#include "toldialoghump_dual.hpp"
#include "config.hpp"
#include "rrtdialog.hpp"
#include "rrtconnectdialog.hpp"
#include "rrtstardialog.hpp"
#include "prmdialog.hpp"
#include "prmstardialog.hpp"
#include "results_plan_joints_dialog.hpp"
#include "results_warm_start_dialog.hpp"
#include "power_law_dialog.hpp"
#include "powerlaw3ddialog.hpp"
#include "comp_velocity_dialog.hpp"
#include "nat_coll_av_dialog.hpp"
#include "handposplot.hpp"

using namespace std;
using namespace p1d;



/** This is the main namespace of the program */
namespace motion_manager {



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
         * @brief This method saves qt program settings when closing the main_window
         */
        void WriteSettings();

        /**
         * @brief This method menages the close event
         * @param event
         */
        void closeEvent(QCloseEvent *event);

        /**
         * @brief getDerivative
         * @param function
         * @param step_values
         * @param derFunction
         */
        void getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction);

        /**
         * @brief getNumberMovementUnits
         * @param function
         * @param time
         * @return
         */
        int getNumberMovementUnits(vector<double> &function, QVector<double> &time);

        /**
         * @brief getMedian
         * @param v
         * @return
         */
        double getMedian(vector<double> v);

        /**
         * @brief getFirstQuartile
         * @param v
         * @return
         */
        double getFirstQuartile(vector<double> v);

        /**
         * @brief getThirdQuartile
         * @param v
         * @return
         */
        double getThirdQuartile(vector<double> v);

        /**
         * @brief getMedian
         * @param v
         * @return
         */
        double getMedian(vector<int> v);

        /**
         * @brief getFirstQuartile
         * @param v
         * @return
         */
        double getFirstQuartile(vector<int> v);

        /**
         * @brief getThirdQuartile
         * @param v
         * @return
         */
        double getThirdQuartile(vector<int> v);



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

#if MOVEIT==1
        /**
         * @brief This method shows the RViz communication dialog
         */
        void on_actionRViz_Communication_triggered();
#endif

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
        void on_comboBox_mov_left_currentIndexChanged(int i);

        /**
         * @brief This method plans the selected movement
         */
        void on_pushButton_plan_clicked();

        /**
         * @brief on_pushButton_plan_trials_clicked
         */
        void on_pushButton_plan_trials_clicked();

        /**
         * @brief on_pushButton_plan_3d_power_law_clicked
         */
        void on_pushButton_plan_3d_power_law_clicked();

        /**
         * @brief on_pushButton_plan_2d_power_law_clicked
         */
        void on_pushButton_plan_2d_power_law_clicked();

        /**
         * @brief This method plans the selected movement
         */
        void on_pushButton_plan_pressed();

        /**
         * @brief This method executes the selected movement
         */
        void on_pushButton_execMov_clicked();
#if MOVEIT==1
        /**
         * @brief on_pushButton_execMov_moveit_clicked
         */
        void on_pushButton_execMov_moveit_clicked();
#endif

        /**
         * @brief on_pushButton_execMov_moveit_pressed
         */
        void on_pushButton_execMov_moveit_pressed();

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
         * @brief This method saves the end posture data
         */
        void on_pushButton_save_end_posture_clicked();

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
         * @brief addPose
         * @param value
         */
        void addPose(string value);

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

        /**
         * @brief on_pushButton_plot_mov_clicked
         */
        void on_pushButton_plot_mov_clicked();

        /**
         * @brief on_pushButton_plot_mov_dual_clicked
         */
        void on_pushButton_plot_mov_dual_clicked();

        /**
         * @brief on_pushButton_plot_task_clicked
         */
        void on_pushButton_plot_task_clicked();

        /**
         * @brief on_pushButton_plot_task_dual_clicked
         */
        void on_pushButton_plot_task_dual_clicked();

        /**
         * @brief on_pushButton_joints_results_mov_clicked
         */
        void on_pushButton_joints_results_mov_clicked();

        /**
         * @brief on_pushButton_joints_results_mov_right_clicked
         */
        void on_pushButton_joints_results_mov_right_clicked();

        /**
         * @brief on_pushButton_joints_results_mov_left_clicked
         */
        void on_pushButton_joints_results_mov_left_clicked();

        /**
         * @brief on_pushButton_joints_results_task_clicked
         */
        void on_pushButton_joints_results_task_clicked();

        /**
         * @brief on_pushButton_joints_results_task_right_clicked
         */
        void on_pushButton_joints_results_task_right_clicked();

        /**
         * @brief on_pushButton_joints_results_task_left_clicked
         */
        void on_pushButton_joints_results_task_left_clicked();

        /**
         * @brief on_pushButton_power_law_clicked
         */
        void on_pushButton_power_law_clicked();

        /**
         * @brief on_pushButton_power_law_3D_clicked
         */
        void on_pushButton_power_law_3D_clicked();

        /**
         * @brief on_pushButton_nat_coll_av_clicked
         */
        void on_pushButton_nat_coll_av_clicked();


        /**
         * @brief on_pushButton_comp_vel_mov_clicked
         */
        void on_pushButton_comp_vel_mov_clicked();

        /**
         * @brief on_pushButton_warm_start_res_clicked
         */
        void on_pushButton_warm_start_res_clicked();

        /**
         * @brief on_pushButton_comp_vel_mov_right_clicked
         */
        void on_pushButton_comp_vel_mov_right_clicked();

        /**
         * @brief on_pushButton_comp_vel_mov_left_clicked
         */
        void on_pushButton_comp_vel_mov_left_clicked();
        /**
         * @brief on_pushButton_save_res_mov_clicked
         */
        void on_pushButton_save_res_mov_clicked();

        /**
         * @brief on_pushButton_save_res_mov_dual_clicked
         */
        void on_pushButton_save_res_mov_dual_clicked();

        /**
         * @brief on_pushButton_save_res_task_clicked
         */
        void on_pushButton_save_res_task_clicked();

        /**
         * @brief on_pushButton_save_res_task_dual_clicked
         */
        void on_pushButton_save_res_task_dual_clicked();



private:
        Ui::MainWindowDesign ui; /**< handles of the main user interface */
        QNode qnode; /**< ROS node handle */
        RosCommDialog *mrosCommdlg; /**< handle of the ROS communication dialog */
        VrepCommDialog *mvrepCommdlg; /**< handle of the V-REP communication dialog */
#if MOVEIT==1
        RVizCommDialog *mrvizCommdlg; /**< handle of the RViz communication dialog */
#endif
        TolDialogHUMP *mTolHumpdlg; /**< handle of the HUMP tuning dialog */
        TolDialogHUMPDual *mTolHumpDualdlg; /**< handle of the HUMP tuning dialog for bimanual motion*/
        RRTDialog *mRRTdlg; /**< handle of the RRT tuning dialog */
        RRTConnectDialog *mRRTConnectdlg; /**< handle of the RRT Connect tuning dialog */
        RRTstarDialog *mRRTstardlg; /**< handle of the RRT star tuning dialog */
        PRMDialog *mPRMdlg; /**< handle of the PRM tuning dialog */
        PRMstarDialog *mPRMstardlg; /**< handle of the PRM star tuning dlg */
        ResultsJointsDialog *mResultsJointsdlg;/**< handle of the results joints dlg*/
        PowerLawDialog *mPowerLawdlg; /**< handle of the 2/3 power law dialog*/
        PowerLaw3DDialog *mPowerLaw3Ddlg; /**< handle of the 1/6 power law dialog*/
        CompVelocityDialog *mCompVeldlg; /**< handle of the velocity components dlg */
        NatCollAvDialog *mNatCollAvdlg; /**< handle of the natural collision avoidance dlg */
        WarmStartResultsDialog *mWarmdlg; /**< handle of the warm start results dialog */
        int scenario_id; /**< id of the current scenario */
        QVector<QString> scenarios;  /**< list of scenarios */

        vector< vector < double > > timesteps_mov; /**< current time steps of the movement */
        QVector<double> qtime_mov; /**< time of the current movement for plotting */
        vector<double> tols_stop_mov; /**< vector of the tolerances to stop each stage in the movement */
        vector< MatrixXd > jointsAcceleration_mov; /**< trajectory of the joint acceleration of the movement */
        vector< MatrixXd > jointsVelocity_mov; /**< trajectory of the joint velocity of the movement */
        vector< MatrixXd > jointsPosition_mov; /**< trajectory of the joint position of the movement */
        vector< string > traj_descr_mov; /**< description of the trajectories */
        vector<HUMotion::warm_start_params> final_warm_start_res_mov; /**< warm start results of the target posture selections solved for the movement */
        HUMotion::warm_start_params bounce_warm_start_res_mov; /**< warm start results of the bounce posture selection solved for the movement */
        vector<double> jointsEndPosition_mov; /**< end joint position of the movement */
        vector<double> jointsEndVelocity_mov; /**< end joint velocity of the movement */
        vector<double> jointsEndAcceleration_mov; /**< end joint acceleration of the movement */
        vector< vector< MatrixXd > > jointsAcceleration_task; /**< trajectory of the joint acceleration of the task */
        vector< vector< MatrixXd > > jointsVelocity_task; /**< trajectory of the joint velocity of the task */
        vector< vector< MatrixXd > > jointsPosition_task; /**< trajectory of the joint position of the task */
        vector<vector< string >> traj_descr_task; /**< description of the trajectories of the task*/
        vector< vector< vector < double > > > timesteps_task; /**< vector of time steps of each movement in the task */
        QVector<double> qtime_task;/**< time of the current task for plotting */
        vector<vector<double>> tols_stop_task; /**< vector of the tolerances to stop each movement in the task */
        vector<string> vel_steps; /**< steps of the trajectory for saving/loading file */
        double prob_time_mov;/**< time taken to solve the problem */

        vector<vector<double>> handPosition_mov; /**< hand position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_mov; /**< hand orientation during the movement. */
        vector<vector<double>> handLinearVelocity_mov; /**< hand linear velocity during the movement */
        vector<vector<double>> handAngularVelocity_mov;/**< hand angular velocity during the movement */
        vector<vector<double>> wristLinearVelocity_mov; /**< wrist linear velocity during the movement */
        vector<vector<double>> wristAngularVelocity_mov;/**< wrist angular velocity during the movement */
        vector<vector<double>> elbowLinearVelocity_mov; /**< elbow linear velocity during the movement */
        vector<vector<double>> elbowAngularVelocity_mov;/**< elbow angular velocity during the movement */
        vector<vector<double>> shoulderLinearVelocity_mov; /**< shoulder linear velocity during the movement */
        vector<vector<double>> shoulderAngularVelocity_mov;/**< shoulder angular velocity during the movement */
        vector<double> handVelocityNorm_mov; /**< hand velocity norm during the movement */
        vector<double> wristVelocityNorm_mov; /**< wrist velocity norm during the movement */
        vector<double> elbowVelocityNorm_mov; /**< elbow velocity norm during the movement */
        vector<double> shoulderVelocityNorm_mov; /**< shoulder velocity norm during the movement */
        double njs_mov;/**< normalized jerk score of the movement */
        int nmu_mov;/**< number of the movement units */

        vector<vector<double>> handPosition_mov_left; /**< hand position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_mov_left; /**< hand orientation during the movement. */
        vector<vector<double>> handLinearVelocity_mov_left; /**< hand linear velocity during the movement */
        vector<vector<double>> handAngularVelocity_mov_left;/**< hand angular velocity during the movement */
        vector<vector<double>> wristLinearVelocity_mov_left; /**< wrist linear velocity during the movement */
        vector<vector<double>> wristAngularVelocity_mov_left;/**< wrist angular velocity during the movement */
        vector<vector<double>> elbowLinearVelocity_mov_left; /**< elbow linear velocity during the movement */
        vector<vector<double>> elbowAngularVelocity_mov_left;/**< elbow angular velocity during the movement */
        vector<vector<double>> shoulderLinearVelocity_mov_left; /**< shoulder linear velocity during the movement */
        vector<vector<double>> shoulderAngularVelocity_mov_left;/**< shoulder angular velocity during the movement */
        vector<double> handVelocityNorm_mov_left; /**< hand velocity norm during the movement */
        vector<double> wristVelocityNorm_mov_left; /**< wrist velocity norm during the movement */
        vector<double> elbowVelocityNorm_mov_left; /**< elbow velocity norm during the movement */
        vector<double> shoulderVelocityNorm_mov_left; /**< shoulder velocity norm during the movement */
        double njs_mov_left;/**< normalized jerk score of the movement */
        int nmu_mov_left;/**< number of the movement units */

        vector<double> prob_time_task;/**< time taken to solve the problems in the task */

        vector<vector<double>> handPosition_task; /**< hand position during the task. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_task; /**< hand orientation during the task. */
        vector<vector<double>> handLinearVelocity_task; /**< hand linear velocity during the task */
        vector<vector<double>> handAngularVelocity_task;/**< hand angular velocity during the task */
        vector<double> handVelocityNorm_task; /**< hand velocity norm during the task */        
        vector<double> njs_task;/**< normalized jerk scores of the movements in the task */
        vector<int> nmu_task;/**< number of the movement units in the task */

        vector<vector<double>> handPosition_task_left; /**< hand position during the task. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_task_left; /**< hand orientation during the task. */
        vector<vector<double>> handLinearVelocity_task_left; /**< hand linear velocity during the task */
        vector<vector<double>> handAngularVelocity_task_left;/**< hand angular velocity during the task */
        vector<double> handVelocityNorm_task_left; /**< hand velocity norm during the task */
        vector<double> njs_task_left;/**< normalized jerk scores of the movements in the task */
        vector<int> nmu_task_left;/**< number of the movement units in the task */

        bool moveit_mov; /**< true if the movement has been planned by the moveit planner, false otherwise */
        bool moveit_task; /**< true if at least one movement in the task has been planned by the moveit planner, false otherwise */
        HUMotion::planning_result_ptr h_results; /**< single-arm results of the HUMP planner */
        HUMotion::planning_dual_result_ptr h_dual_results; /**< dual-arm results of the HUMP planner */
#if MOVEIT==1
        moveit_plannerPtr m_planner; /**< MoveIt! Libraries planner */
        moveit_planning::PlanningResultPtr m_results; /**< results of the moveit planner */
#endif


        movementPtr curr_mov; /**< current movement */
        taskPtr curr_task;/**< current task */
        scenarioPtr init_scene; /**< initial scenario */
        scenarioPtr curr_scene; /**< current scenario */

        boost::shared_ptr<HandPosPlot> handPosPlot_mov_ptr; /**< pointer to the hand position plot of the movement */
        boost::shared_ptr<HandPosPlot> handPosPlot_mov_left_ptr; /**< pointer to the hand position plot of the movement */
        boost::shared_ptr<HandPosPlot> handPosPlot_task_ptr;/**< pointer to the hand position plot of the task */
        boost::shared_ptr<HandPosPlot> handPosPlot_task_left_ptr;/**< pointer to the hand position plot of the task */

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
