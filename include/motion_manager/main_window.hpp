
#ifndef motion_manager_MAIN_WINDOW_H
#define motion_manager_MAIN_WINDOW_H


/*****************************************************************************
** Includes
*****************************************************************************/
#include <unordered_map>
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
#include "results_control_joints_dialog.hpp"
#include "results_warm_start_dialog.hpp"
#include "power_law_dialog.hpp"
#include "powerlaw3ddialog.hpp"
#include "comp_velocity_dialog.hpp"
#include "comp_control_dialog.hpp"
#include "nat_coll_av_dialog.hpp"
#include "errors_control_dialog.hpp"
#include "handposplot.hpp"
#include "circular_vector_buffer.hpp"
#include "time_map_dialog.hpp"

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace p1d;



/** This is the main namespace of the program */
namespace motion_manager {

typedef boost::shared_ptr< CircularBuffers<double> > bufferPtr;


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
         * @brief getDerivative
         * @param function
         * @param step_values
         * @param derFunction
         */
        void getDerivative(vector<double> &function, vector<double> &step_values, vector<double> &derFunction);

        /**
         * @brief getDerivative
         * @param function
         * @param step_values
         * @param derFunction
         */
        void getDerivative(vector<vector<double>> &function, vector<double> &step_values, vector<vector<double>> &derFunction);

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

        /**
         * @brief init
         */
        void init();

        /**
         * @brief binomialCoeff
         * @param n
         * @param k
         * @return
         */
        int binomialCoeff(int n, int k);

        /**
         * @brief getNoiseRobustDerivate
         * @param N
         * @param h
         * @param buff
         * @return
         */
        double getNoiseRobustDerivate(int N, double h, std::deque<double> &buff);



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
        void on_pushButton_plan_pressed();

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

        // -----------------------------------------------------
        // Learning

        /**
         * @brief on_pushButton_load_learn_prim_duals_clicked
         */
        void on_pushButton_load_learn_prim_duals_clicked();

        /**
         * @brief on_pushButton_plan_collect_clicked
         */
        void on_pushButton_plan_collect_clicked();

        /**
         * @brief on_pushButton_plan_collect_pressed
         */
        void on_pushButton_plan_collect_pressed();

        /**
         * @brief on_pushButton_py_train_file_clicked
         */
        void on_pushButton_py_train_file_clicked();

        /**
         * @brief on_pushButton_train_data_clicked
         */
        void on_pushButton_train_data_clicked();

        /**
         * @brief on_pushButton_models_clicked
         */
        void on_pushButton_models_clicked();

        /**
         * @brief on_pushButton_train_pressed
         */
        void on_pushButton_train_pressed();

        /**
         * @brief on_pushButton_train_clicked
         * @return
         */
        bool on_pushButton_train_clicked();

        /**
         * @brief on_pushButton_py_pred_file_clicked
         */
        void on_pushButton_py_pred_file_clicked();

        /**
         * @brief on_pushButton_predictions_clicked
         */
        void on_pushButton_predictions_clicked();

        /**
         * @brief on_pushButton_collections_clicked
         */
        void on_pushButton_collections_clicked();

        /**
         * @brief on_pushButton_pred_plan_pressed
         */
        void on_pushButton_pred_plan_pressed();

        /**
         * @brief on_pushButton_pred_plan_clicked
         */
        void on_pushButton_pred_plan_clicked();

        /**
         * @brief on_pushButton_save_learning_res_clicked
         */
        void on_pushButton_save_learning_res_clicked();

        /**
         * @brief check_tar_x_pos_var
         * @param state
         */
        void check_tar_x_pos_var(int state);

        /**
         * @brief check_tar_y_pos_var
         * @param state
         */
        void check_tar_y_pos_var(int state);

        /**
         * @brief check_tar_z_pos_var
         * @param state
         */
        void check_tar_z_pos_var(int state);

        /**
         * @brief check_tar_roll_var
         * @param state
         */
        void check_tar_roll_var(int state);

        /**
         * @brief check_tar_pitch_var
         * @param state
         */
        void check_tar_pitch_var(int state);

        /**
         * @brief check_tar_yaw_var
         * @param state
         */
        void check_tar_yaw_var(int state);

        /**
         * @brief check_obsts_x_pos_var
         * @param state
         */
        void check_obsts_x_pos_var(int state);

        /**
         * @brief check_obsts_y_pos_var
         * @param state
         */
        void check_obsts_y_pos_var(int state);

        /**
         * @brief check_obsts_z_pos_var
         * @param state
         */
        void check_obsts_z_pos_var(int state);

        /**
         * @brief check_obsts_roll_var
         * @param state
         */
        void check_obsts_roll_var(int state);

        /**
         * @brief check_obsts_pitch_var
         * @param state
         */
        void check_obsts_pitch_var(int state);

        /**
         * @brief check_obsts_yaw_var
         * @param state
         */
        void check_obsts_yaw_var(int state);

        // -----------------------------------------------------
        // Controlling

        /**
         * @brief on_pushButton_start_control_pressed
         */
        void on_pushButton_start_control_pressed();

        /**
         * @brief on_pushButton_start_control_clicked
         */
        void on_pushButton_start_control_clicked();

        /**
         * @brief on_pushButton_stop_control_pressed
         */
        void on_pushButton_stop_control_pressed();

        /**
         * @brief on_pushButton_stop_control_clicked
         */
        void on_pushButton_stop_control_clicked();

        /**
         * @brief on_pushButton_control_plot_clicked
         */
        void on_pushButton_control_plot_clicked();

        /**
         * @brief on_pushButton_control_save_clicked
         */
        void on_pushButton_control_save_clicked();

        /**
         * @brief check_right_hand_status
         * @param state
         */
        void check_right_hand_status(int state);

        /**
         * @brief check_use_vel_control
         * @param state
         */
        void check_use_vel_control(int state);

        /**
         * @brief check_des_right_hand_pos_x
         * @param state
         */
        void check_des_right_hand_pos_x(int state);

        /**
         * @brief check_des_right_hand_pos_y
         * @param state
         */
        void check_des_right_hand_pos_y(int state);

        /**
         * @brief check_des_right_hand_pos_z
         * @param state
         */
        void check_des_right_hand_pos_z(int state);

        /**
         * @brief check_des_right_hand_q_x
         * @param state
         */
        void check_des_right_hand_q_x(int state);

        /**
         * @brief check_des_right_hand_q_y
         * @param state
         */
        void check_des_right_hand_q_y(int state);

        /**
         * @brief check_des_right_hand_q_z
         * @param state
         */
        void check_des_right_hand_q_z(int state);

        /**
         * @brief check_des_right_hand_q_w
         * @param state
         */
        void check_des_right_hand_q_w(int state);

        /**
         * @brief check_use_plan_hand_pos
         * @param state
         */
        void check_use_plan_hand_pos(int state);

        /**
         * @brief check_des_right_hand_vel_x
         * @param state
         */
        void check_des_right_hand_vel_x(int state);

        /**
         * @brief check_des_right_hand_vel_y
         * @param state
         */
        void check_des_right_hand_vel_y(int state);

        /**
         * @brief check_des_right_hand_vel_z
         * @param state
         */
        void check_des_right_hand_vel_z(int state);

        /**
         * @brief check_des_right_hand_vel_wx
         * @param state
         */
        void check_des_right_hand_vel_wx(int state);

        /**
         * @brief check_des_right_hand_vel_wy
         * @param state
         */
        void check_des_right_hand_vel_wy(int state);

        /**
         * @brief check_des_right_hand_vel_wz
         * @param state
         */
        void check_des_right_hand_vel_wz(int state);

        /**
         * @brief check_ctrl_joints_limits_av
         * @param state
         */
        void check_ctrl_joints_limits_av(int state);

        /**
         * @brief check_ctrl_sing_av
         * @param state
         */
        void check_ctrl_sing_av(int state);

        /**
         * @brief check_ctrl_obsts_av
         * @param state
         */
        void check_ctrl_obsts_av(int state);

        /**
         * @brief check_ctrl_obsts_filter_noise
         * @param state
         */
        void check_ctrl_obsts_filter_noise(int state);

        /**
         * @brief check_ctrl_tar_filter_noise
         * @param state
         */
        void check_ctrl_tar_filter_noise(int state);

        /**
         * @brief check_ctrl_hl_add
         * @param state
         */
        void check_ctrl_hl_add(int state);

        /**
         * @brief check_draw_ellipse
         * @param state
         */
        void check_draw_ellipse(int state);

        /**
         * @brief on_pushButton_control_plot_joints_clicked
         */
        void on_pushButton_control_plot_joints_clicked();

        /**
         * @brief on_pushButton_control_plot_pos_vel_comps_clicked
         */
        void on_pushButton_control_plot_pos_vel_comps_clicked();

        /**
         * @brief on_pushButton_errors_ctrl_clicked
         */
        void on_pushButton_errors_ctrl_clicked();

        /**
         * @brief on_pushButton_save_ctrl_params_clicked
         */
        void on_pushButton_save_ctrl_params_clicked();

        /**
         * @brief on_pushButton_load_ctrl_params_clicked
         */
        void on_pushButton_load_ctrl_params_clicked();

        /**
         * @brief on_radioButton_N_5_clicked
         */
        void on_radioButton_N_5_clicked();

        /**
         * @brief on_radioButton_N_7_clicked
         */
        void on_radioButton_N_7_clicked();

        /**
         * @brief on_radioButton_N_9_clicked
         */
        void on_radioButton_N_9_clicked();

        /**
         * @brief on_radioButton_N_11_clicked
         */
        void on_radioButton_N_11_clicked();

        /**
         * @brief on_radioButton_N_19_clicked
         */
        void on_radioButton_N_19_clicked();

        /**
         * @brief on_radioButton_N_25_clicked
         */
        void on_radioButton_N_25_clicked();

        /**
         * @brief on_radioButton_N_35_clicked
         */
        void on_radioButton_N_35_clicked();

        /**
         * @brief on_radioButton_N_45_clicked
         */
        void on_radioButton_N_45_clicked();

        /**
         * @brief on_radioButton_N_55_clicked
         */
        void on_radioButton_N_55_clicked();

        /**
         * @brief on_pushButton_time_map_clicked
         */
        void on_pushButton_time_map_clicked();


private:
        src::severity_logger< severity_level > lg; /**< logger */

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
        TimeMapDialog *mTimeMapdlg; /**< handle of the time mapping dlg */

        NatCollAvDialog *mNatCollAvdlg; /**< handle of the natural collision avoidance dlg */
        WarmStartResultsDialog *mWarmdlg; /**< handle of the warm start results dialog */
        ResultsCtrlJointsDialog *mResultsCtrlJointsdlg;/**< handle of the results joints dlg during control*/
        CompControlDialog *mCompCtrldlg; /** < handle of the components dlg during control */
        ErrorsControlDialog *mErrCtrldlg; /** < handle of the errors dlg during control */
        int scenario_id; /**< id of the current scenario */
        QVector<QString> scenarios;  /**< list of scenarios */

        vector< vector < double > > timesteps_mov; /**< current time steps of the movement */
        QVector<double> qtime_mov; /**< time of the current movement for plotting */
        vector<double> tols_stop_mov; /**< vector of the tolerances to stop each stage in the movement */
        vector< MatrixXd > jointsAcceleration_mov; /**< trajectory of the joint acceleration of the movement */
        vector< MatrixXd > jointsVelocity_mov; /**< trajectory of the joint velocity of the movement */
        vector< MatrixXd > jointsPosition_mov; /**< trajectory of the joint position of the movement */
        vector< MatrixXd > jacobian_mov; /**< trajectory of the jacobian of the movement */
        vector< MatrixXd > der_jacobian_mov; /**< trajectory of the derivative of the jacobian of the movement */
        vector< MatrixXd > jointsPosition_mov_ctrl; /**< trajectory of the joint position of the movement for controlling */
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
        vector<vector<vector<double>>> handPosition_mov_stages; /**< hand position during the movement divided in stages. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_mov; /**< hand orientation (rpy) during the movement. */
        vector<vector<vector<double>>> handOrientation_mov_stages; /**< hand orientation (rpy) during the movement divided in stages. */
        vector<vector<double>> handOrientation_q_mov; /**< hand orientation (quaternion) during the movement. */
        vector<vector<vector<double>>> handOrientation_q_mov_stages; /**< hand orientation (quaternion) during the movement divided in stages. */
        vector<vector<double>> wristPosition_mov; /**< wrist position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> wristOrientation_mov; /**< wrist orientation (rpy) during the movement. */
        vector<vector<double>> elbowPosition_mov; /**< elbow position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> elbowOrientation_mov; /**< elbow orientation (rpy) during the movement. */
        vector<vector<double>> shoulderPosition_mov; /**< shoulder position during the movement. 0=x,1=y,2=z */
        vector<double> swivel_angle_mov; /**< swivel angle of the trajectory of the movement */
        double swivel_angle_mov_max; /**< max swivel angle of the trajectory of the movement */
        double swivel_angle_mov_min; /**< min swivel angle of the trajectory of the movement */
        double swivel_angle_mov_average; /**< average swivel angle of the trajectory of the movement */
        vector<vector<double>> shoulderOrientation_mov; /**< shoulder orientation (rpy) during the movement. */
        vector<vector<double>> handLinearVelocity_mov; /**< hand linear velocity during the movement */
        vector<vector<double>> handAngularVelocity_mov;/**< hand angular velocity during the movement */
        vector<vector<vector<double>>> handLinearVelocity_mov_stages; /**< hand linear velocity during the movement divided in stages.*/
        vector<vector<vector<double>>> handAngularVelocity_mov_stages; /**< hand angular velocity during the movement divided in stages.*/
        vector<vector<vector<double>>> handLinearAcceleration_mov_stages; /**< hand linear acceleration during the movement divided in stages */
        vector<vector<vector<double>>> handAngularAcceleration_mov_stages;/**< hand angular acceleration during the movement divided in stages */
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
        vector<vector<double>> handLinearAcceleration_mov; /**< hand linear acceleration during the movement */
        vector<vector<double>> handAngularAcceleration_mov;/**< hand angular acceleration during the movement */
        vector<vector<double>> wristLinearAcceleration_mov; /**< wrist linear acceleration during the movement */
        vector<vector<double>> wristAngularAcceleration_mov;/**< wrist angular acceleration during the movement */
        vector<vector<double>> elbowLinearAcceleration_mov; /**< elbow linear acceleration during the movement */
        vector<vector<double>> elbowAngularAcceleration_mov;/**< elbow angular acceleration during the movement */
        vector<vector<double>> shoulderLinearAcceleration_mov; /**< shoulder linear acceleration during the movement */
        vector<vector<double>> shoulderAngularAcceleration_mov;/**< shoulder angular acceleration during the movement */
        double njs_mov;/**< normalized jerk score of the movement */
        int nmu_mov;/**< number of the movement units */

        vector<vector<double>> handPosition_mov_left; /**< hand position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_mov_left; /**< hand orientation (rpy) during the movement. */
        vector<vector<double>> handLinearVelocity_mov_left; /**< hand linear velocity during the movement */
        vector<vector<double>> handAngularVelocity_mov_left;/**< hand angular velocity during the movement */
        vector<vector<double>> handLinearAcceleration_mov_left; /**< hand linear acceleration during the movement */
        vector<vector<double>> handAngularAcceleration_mov_left;/**< hand angular acceleration during the movement */
        vector<vector<double>> wristPosition_mov_left; /**< wrist position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> wristOrientation_mov_left; /**< wrist orientation (rpy) during the movement. */
        vector<vector<double>> wristLinearVelocity_mov_left; /**< wrist linear velocity during the movement */
        vector<vector<double>> wristAngularVelocity_mov_left;/**< wrist angular velocity during the movement */
        vector<vector<double>> wristLinearAcceleration_mov_left; /**< wrist linear acceleration during the movement */
        vector<vector<double>> wristAngularAcceleration_mov_left;/**< wrist angular acceleration during the movement */
        vector<vector<double>> elbowPosition_mov_left; /**< elbow position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> elbowOrientation_mov_left; /**< elbow orientation (rpy) during the movement. */
        vector<vector<double>> elbowLinearVelocity_mov_left; /**< elbow linear velocity during the movement */
        vector<vector<double>> elbowAngularVelocity_mov_left;/**< elbow angular velocity during the movement */
        vector<vector<double>> elbowLinearAcceleration_mov_left; /**< elbow linear acceleration during the movement */
        vector<vector<double>> elbowAngularAcceleration_mov_left;/**< elbow angular acceleration during the movement */
        vector<vector<double>> shoulderPosition_mov_left; /**< shoulder position during the movement. 0=x,1=y,2=z */
        vector<vector<double>> shoulderOrientation_mov_left; /**< shoulder orientation (rpy) during the movement. */
        vector<vector<double>> shoulderLinearVelocity_mov_left; /**< shoulder linear velocity during the movement */
        vector<vector<double>> shoulderAngularVelocity_mov_left;/**< shoulder angular velocity during the movement */
        vector<vector<double>> shoulderLinearAcceleration_mov_left; /**< shoulder linear acceleration during the movement */
        vector<vector<double>> shoulderAngularAcceleration_mov_left;/**< shoulder angular acceleration during the movement */
        vector<double> handVelocityNorm_mov_left; /**< hand velocity norm during the movement */
        vector<double> wristVelocityNorm_mov_left; /**< wrist velocity norm during the movement */
        vector<double> elbowVelocityNorm_mov_left; /**< elbow velocity norm during the movement */
        vector<double> shoulderVelocityNorm_mov_left; /**< shoulder velocity norm during the movement */
        vector<double> handAccelerationNorm_mov_left; /**< hand acceleration norm during the movement */
        vector<double> wristAccelerationNorm_mov_left; /**< wrist acceleration norm during the movement */
        vector<double> elbowAccelerationNorm_mov_left; /**< elbow acceleration norm during the movement */
        vector<double> shoulderAccelerationNorm_mov_left; /**< shoulder velocity norm during the movement */
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
        HUMotion::hump_params  tols; /**< human-like parameters for single-arm planning */
        HUMotion::hump_dual_params  dual_tols;/**< human-like parameters for dual-arm planning */
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

        // ---------------------------- Learning ------------------------------------- //
        /**
         * @brief count_occurrence
         * @param m
         * @param v
         */
        void count_occurrence(std::unordered_map<int,int>& m, std::vector<int>& v);

        /**
         * @brief median
         * @param len
         * @return
         */
        double median(std::vector<double>& len);

        /**
         * @brief add_microsteps
         * @param h_func
         * @param h_micro_func
         * @param microsteps
         */
        void add_microsteps(std::vector<vector<double>>& h_func, std::vector<vector<double>>& h_micro_func, int microsteps);

        /**
         * @brief add_microsteps
         * @param h_func
         * @param h_micro_func
         * @param microsteps
         */
        void add_microsteps(MatrixXd& h_func, MatrixXd& h_micro_func, int microsteps);

        /**
         * @brief getQuaternion
         * @param q
         * @param Rot
         */
        void getQuaternion(std::vector<double>& q, Matrix3d& Rot);

        /**
         * @brief Rot_matrix
         * @param Rot
         * @param rpy
         */
        void Rot_matrix(Matrix3d &Rot,std::vector<double>& rpy);

        // solution of the given problems
        bool sol_loaded;/**< loaded solution flag */
        // plan
        bool sol_plan; /**< true if there are plan target solution */
        vector<double> x_plan; /**< initial guess of the plan target posture selection problem */
        vector<double> zL_plan; /**< lower bounds multipliers of the plan target posture selection problem */
        vector<double> zU_plan; /**< upper bounds multipliers of the plan target posture selection problem */
        vector<double> dual_plan; /**< constraints multipliers of the plan target posture selection problem */
        // approach
        bool sol_approach; /**< true if there are approach target solution */
        vector<double> x_approach; /**< initial guess of the approach target posture selection problem */
        vector<double> zL_approach; /**< lower bounds multipliers of the approach target posture selection problem */
        vector<double> zU_approach; /**< upper bounds multipliers of the approach target posture selection problem */
        vector<double> dual_approach; /**< constraints multipliers of the approach target posture selection problem */
        // retreat
        bool sol_retreat; /**< true if there are retreat target solution */
        vector<double> x_retreat; /**< initial guess of the retreat target posture selection problem */
        vector<double> zL_retreat; /**< lower bounds multipliers of the retreat target posture selection problem */
        vector<double> zU_retreat; /**< upper bounds multipliers of the retreat target posture selection problem */
        vector<double> dual_retreat; /**< constraints multipliers of the retreat target posture selection problem */
        // bounce
        bool sol_bounce; /**< true if there are bounce solution */
        vector<double> x_bounce; /**< initial guess of the bounce target posture selection problem */
        vector<double> zL_bounce; /**< lower bounds multipliers of the bounce target posture selection problem */
        vector<double> zU_bounce; /**< upper bounds multipliers of the bounce target posture selection problem */
        vector<double> dual_bounce; /**< constraints multipliers of the bounce target posture selection problem */

        // results of the predictions
        // plan
        vector<int> success_cold_plan; /**< success flags with cold start */
        vector<double> iter_cold_plan; /**< iterations with cold start */
        vector<double> cpu_cold_plan; /**< cpu time with cold start */
        vector<double> obj_cold_plan; /**< objective function values with cold start */
        vector<int> success_ws_or_plan; /**< success flags with warm start from the original solution */
        vector<double> iter_ws_or_plan; /**< iterations with warm start from the original solution */
        vector<double> cpu_ws_or_plan; /**< cpu time with warm start from the original solution */
        vector<double> obj_ws_or_plan; /**< objective function values with warm start from the original solution */
        vector<int> success_ws_rdm_plan; /**< success flags with warm start from the random solution */
        vector<double> iter_ws_rdm_plan; /**< iterations with warm start from the random solution */
        vector<double> cpu_ws_rdm_plan; /**< cpu time with warm start from the random solution */
        vector<double> obj_ws_rdm_plan; /**< objective function values with warm start from the random solution */
        vector<int> success_ws_nn_plan; /**< success flags with warm start from the neural network solution */
        vector<double> iter_ws_nn_plan; /**< iterations with warm start from the neural network solution */
        vector<double> cpu_ws_nn_plan; /**< cpu time with warm start from the neural network solution */
        vector<double> obj_ws_nn_plan; /**< objective function values with warm start from the neural network solution */
        vector<int> success_ws_svm_plan; /**< success flags with warm start from the support vector machines solution */
        vector<double> iter_ws_svm_plan; /**< iterations with warm start from the support vector machines solution */
        vector<double> cpu_ws_svm_plan; /**< cpu time with warm start from the support vector machines solution */
        vector<double> obj_ws_svm_plan; /**< objective function values with warm start from the support vector machines solution */
        vector<int> success_ws_knn_plan; /**< success flags with warm start from the k-nearest neighbors solution */
        vector<double> iter_ws_knn_plan; /**< iterations with warm start from the k-nearest neighbors solution */
        vector<double> cpu_ws_knn_plan; /**< cpu time with warm start from the k-nearest neighbors solution */
        vector<double> obj_ws_knn_plan; /**< objective function values with warm start from the k-nearest neighbors solution */

        // approach
        vector<int> success_cold_app; /**< success flags with cold start */
        vector<double> iter_cold_app; /**< iterations with cold start */
        vector<double> cpu_cold_app; /**< cpu time with cold start */
        vector<double> obj_cold_app; /**< objective function values with cold start */
        vector<int> success_ws_or_app; /**< success flags with warm start from the original solution */
        vector<double> iter_ws_or_app; /**< iterations with warm start from the original solution */
        vector<double> cpu_ws_or_app; /**< cpu time with warm start from the original solution */
        vector<double> obj_ws_or_app; /**< objective function values with warm start from the original solution */
        vector<int> success_ws_rdm_app; /**< success flags with warm start from the random solution */
        vector<double> iter_ws_rdm_app; /**< iterations with warm start from the random solution */
        vector<double> cpu_ws_rdm_app; /**< cpu time with warm start from the random solution */
        vector<double> obj_ws_rdm_app; /**< objective function values with warm start from the random solution */
        vector<int> success_ws_nn_app; /**< success flags with warm start from the neural network solution */
        vector<double> iter_ws_nn_app; /**< iterations with warm start from the neural network solution */
        vector<double> cpu_ws_nn_app; /**< cpu time with warm start from the neural network solution */
        vector<double> obj_ws_nn_app; /**< objective function values with warm start from the neural network solution */
        vector<int> success_ws_svm_app; /**< success flags with warm start from the support vector machines solution */
        vector<double> iter_ws_svm_app; /**< iterations with warm start from the support vector machines solution */
        vector<double> cpu_ws_svm_app; /**< cpu time with warm start from the support vector machines solution */
        vector<double> obj_ws_svm_app; /**< objective function values with warm start from the support vector machines solution */
        vector<int> success_ws_knn_app; /**< success flags with warm start from the k-nearest neighbors solution */
        vector<double> iter_ws_knn_app; /**< iterations with warm start from the k-nearest neighbors solution */
        vector<double> cpu_ws_knn_app; /**< cpu time with warm start from the k-nearest neighbors solution */
        vector<double> obj_ws_knn_app; /**< objective function values with warm start from the k-nearest neighbors solution */

        // retreat
        vector<int> success_cold_ret; /**< success flags with cold start */
        vector<double> iter_cold_ret; /**< iterations with cold start */
        vector<double> cpu_cold_ret; /**< cpu time with cold start */
        vector<double> obj_cold_ret; /**< objective function values with cold start */
        vector<int> success_ws_or_ret; /**< success flags with warm start from the original solution */
        vector<double> iter_ws_or_ret; /**< iterations with warm start from the original solution */
        vector<double> cpu_ws_or_ret; /**< cpu time with warm start from the original solution */
        vector<double> obj_ws_or_ret; /**< objective function values with warm start from the original solution */
        vector<int> success_ws_rdm_ret; /**< success flags with warm start from the random solution */
        vector<double> iter_ws_rdm_ret; /**< iterations with warm start from the random solution */
        vector<double> cpu_ws_rdm_ret; /**< cpu time with warm start from the random solution */
        vector<double> obj_ws_rdm_ret; /**< objective function values with warm start from the random solution */
        vector<int> success_ws_nn_ret; /**< success flags with warm start from the neural network solution */
        vector<double> iter_ws_nn_ret; /**< iterations with warm start from the neural network solution */
        vector<double> cpu_ws_nn_ret; /**< cpu time with warm start from the neural network solution */
        vector<double> obj_ws_nn_ret; /**< objective function values with warm start from the neural network solution */
        vector<int> success_ws_svm_ret; /**< success flags with warm start from the support vector machines solution */
        vector<double> iter_ws_svm_ret; /**< iterations with warm start from the support vector machines solution */
        vector<double> cpu_ws_svm_ret; /**< cpu time with warm start from the support vector machines solution */
        vector<double> obj_ws_svm_ret; /**< objective function values with warm start from the support vector machines solution */
        vector<int> success_ws_knn_ret; /**< success flags with warm start from the k-nearest neighbors solution */
        vector<double> iter_ws_knn_ret; /**< iterations with warm start from the k-nearest neighbors solution */
        vector<double> cpu_ws_knn_ret; /**< cpu time with warm start from the k-nearest neighbors solution */
        vector<double> obj_ws_knn_ret; /**< objective function values with warm start from the k-nearest neighbors solution */

        // bounce
        vector<int> success_cold_bounce; /**< success flags with cold start */
        vector<double> iter_cold_bounce; /**< iterations with cold start */
        vector<double> cpu_cold_bounce; /**< cpu time with cold start */
        vector<double> obj_cold_bounce; /**< objective function values with cold start */
        vector<int> success_ws_or_bounce; /**< success flags with warm start from the original solution */
        vector<double> iter_ws_or_bounce; /**< iterations with warm start from the original solution */
        vector<double> cpu_ws_or_bounce; /**< cpu time with warm start from the original solution */
        vector<double> obj_ws_or_bounce; /**< objective function values with warm start from the original solution */
        vector<int> success_ws_rdm_bounce; /**< success flags with warm start from the random solution */
        vector<double> iter_ws_rdm_bounce; /**< iterations with warm start from the random solution */
        vector<double> cpu_ws_rdm_bounce; /**< cpu time with warm start from the random solution */
        vector<double> obj_ws_rdm_bounce; /**< objective function values with warm start from the random solution */
        vector<int> success_ws_nn_bounce; /**< success flags with warm start from the neural network solution */
        vector<double> iter_ws_nn_bounce; /**< iterations with warm start from the neural network solution */
        vector<double> cpu_ws_nn_bounce; /**< cpu time with warm start from the neural network solution */
        vector<double> obj_ws_nn_bounce; /**< objective function values with warm start from the neural network solution */
        vector<int> success_ws_svm_bounce; /**< success flags with warm start from the support vector machines solution */
        vector<double> iter_ws_svm_bounce; /**< iterations with warm start from the support vector machines solution */
        vector<double> cpu_ws_svm_bounce; /**< cpu time with warm start from the support vector machines solution */
        vector<double> obj_ws_svm_bounce; /**< objective function values with warm start from the support vector machines solution */
        vector<int> success_ws_knn_bounce; /**< success flags with warm start from the k-nearest neighbors solution */
        vector<double> iter_ws_knn_bounce; /**< iterations with warm start from the k-nearest neighbors solution */
        vector<double> cpu_ws_knn_bounce; /**< cpu time with warm start from the k-nearest neighbors solution */
        vector<double> obj_ws_knn_bounce; /**< objective function values with warm start from the k-nearest neighbors solution */

        // ------------------------------------- Controlling -------------------------------------- //
        boost::atomic<bool> get_right_hand_status;
        boost::mutex hh_control_mtx;
        boost::thread display_r_hand_status_thrd;
        void display_r_hand_status();
        boost::atomic<bool> exec_control;
        boost::atomic<bool> pos_control;
        boost::atomic<bool> vel_control;
        boost::thread execPosControl_thrd;
        void execPosControl();
        boost::thread execVelControl_thrd;
        void execVelControl();
        // derivetion parameters
        int samples_pos; /**< counter to count 5 points for derivation of position */
        int samples_vel; /**< counter to count 5 points for derivation of velocity */
        int samples_h_vel; /**< counter to count 5 points for derivation of hand velocity */
        int samples_w_vel; /**< counter to count 5 points for derivation of wrist velocity */
        int samples_e_vel; /**< counter to count 5 points for derivation of elbow velocity */
        int samples_s_vel; /**< counter to count 5 points for derivation of shoulder velocity */
        int N_filter_length; /**< length of the filter of noise */
        bufferPtr arm_pos_buff; /**< buffer of arm positions for derivation */
        bufferPtr hand_pos_buff; /**< buffer of finger positions for drivation */
        bufferPtr arm_vel_buff; /**< buffer of arm velocities for derivation */
        bufferPtr hand_vel_buff; /**< buffer of finger velocities for drivation */
        bufferPtr r_hand_vel_buff; /**< buffer of task space hand velocities for drivation */
        bufferPtr r_wrist_vel_buff; /**< buffer of task space wrist velocities for drivation */
        bufferPtr r_elbow_vel_buff; /**< buffer of task space elbow velocities for drivation */
        bufferPtr r_shoulder_vel_buff; /**< buffer of task space shoulder velocities for drivation */

        // Jacobians
        MatrixXd Jacobian; /**< current Jacobian matrix */

        MatrixXd jointsPosition_ctrl; /**< trajectory of the joint position during control */
        MatrixXd jointsVelocity_ctrl; /**< trajectory of the joint velocity during control */
        MatrixXd jointsAcceleration_ctrl; /**< trajectory of the joint acceleration during control */
        boost::shared_ptr<HandPosPlot> handPosPlot_ctrl_ptr; /**< pointer to the hand position plot during control */
        // hand
        vector<vector<double>> des_handPosition; /**< vector of desired hand positions during control*/
        vector<vector<double>> des_handOrientation; /**< vector of desired hand orientation (rpy) during control*/
        vector<vector<double>> des_handOrientation_q; /**< vector of desired hand orientation (quaternion) during control*/
        vector<double> bounce_handPosition; /**< bounce hand positions during control*/
        vector<double> bounce_handOrientation; /**< bounce hand orientation (rpy) during control*/
        vector<double> bounce_handOrientation_q; /**< bounce hand orientation (quaternion) during control*/
        int i_ctrl; /**< index of the desired hand pose*/
        double t_past; /**< time past in previous stages during control */
        double t_j_past; /**< time past in previous timestep during control */
        double t_der_past; /**< time past in previous calculus of the derivatives during control */
        int mov_type_ctrl; /**<  movement type during control */
        int dHO_ctrl; /**< distance between the hand and the object being manipulated during control*/
        HUMotion::hump_params tols_ctrl; /**< human-like parameters for single-arm planning during control */
        problemPtr prob_ctrl; /**< problem to solve in re-planning during control */
        std::vector<double> approach_ctrl; /**< approach vector during control */
        std::vector<double> retreat_ctrl; /**< retreat vector during control */
        size_t i_tar_ctrl; /**< index of the object being manipulated */
        bool exec_command_ctrl; /**< true to execute the command control, false otherwise */
        VectorXd hand_j_acc; /**< time derivative Jacobian dependant part of the hand accelearion */
        vector<double> h_hand_pos_end; /**< end hand position during control */
        vector<double> h_hand_or_q_end; /**< end hand orientation (quaternion) during control */
        vector<double> h_hand_pos_init; /**< initial hand position during control */
        vector<double> h_hand_or_init; /**< initial hand orientation (rpy) during control */
        vector<double> h_hand_or_q_init; /**< initial hand orientation (quaternion) during control */
        vector<double> h_hand_lin_vel_end; /**< end hand linear velocity during control */
        vector<double> h_hand_ang_vel_end; /**< end hand angular velocity during control */
        vector<double> h_hand_lin_vel_init; /**< initial hand linear velocity during control */
        vector<double> h_hand_ang_vel_init; /**< initial hand angular velocity during control */
        vector<double> h_hand_lin_acc_end; /**< end hand linear acceleration during control */
        vector<double> h_hand_ang_acc_end; /**< end hand angular acceleration during control */
        vector<double> h_hand_lin_acc_init; /**< initial hand linear acceleration during control */
        vector<double> h_hand_ang_acc_init; /**< initial hand angular acceleration during control */

        vector<vector<double>> handPosition_ctrl; /**< hand position during control. 0=x,1=y,2=z */
        vector<vector<double>> handOrientation_ctrl; /**< hand orientation during control. 0=roll,1=pitch,2=yaw */
        vector<vector<double>> handLinearVelocity_ctrl; /**< hand linear velocity during control */
        vector<vector<double>> handAngularVelocity_ctrl;/**< hand angular velocity during control */
        vector<double> handVelocityNorm_ctrl; /**< hand linear velocity norm during control */
        vector<vector<double>> handLinearAcceleration_ctrl; /**< hand linear acceleration during control */
        vector<vector<double>> handAngularAcceleration_ctrl;/**< hand angular acceleration during control */
        vector<double> handAccelerationNorm_ctrl; /**< hand linear acceleration norm during control */
        // wrist
        vector<vector<double>> wristPosition_ctrl; /**< wrist position during control. 0=x,1=y,2=z */
        vector<vector<double>> wristOrientation_ctrl; /**< wrist orientation during control. 0=roll,1=pitch,2=yaw */
        vector<vector<double>> wristLinearVelocity_ctrl; /**< wrist linear velocity during control */
        vector<vector<double>> wristAngularVelocity_ctrl;/**< wrist angular velocity during control */
        vector<vector<double>> wristLinearAcceleration_ctrl; /**< wrist linear acceleration during control */
        vector<vector<double>> wristAngularAcceleration_ctrl;/**< wrist angular acceleration during control */
        // elbow
        vector<vector<double>> elbowPosition_ctrl; /**< elbow position during control. 0=x,1=y,2=z */
        vector<vector<double>> elbowOrientation_ctrl; /**< elbow orientation during control. 0=roll,1=pitch,2=yaw */
        vector<vector<double>> elbowLinearVelocity_ctrl; /**< elbow linear velocity during control */
        vector<vector<double>> elbowAngularVelocity_ctrl;/**< elbow angular velocity during control */
        vector<vector<double>> elbowLinearAcceleration_ctrl; /**< elbow linear acceleration during control */
        vector<vector<double>> elbowAngularAcceleration_ctrl;/**< elbow angular acceleration during control */
        // shoulder
        vector<vector<double>> shoulderPosition_ctrl; /**< shoulder position during control. 0=x,1=y,2=z */
        vector<vector<double>> shoulderOrientation_ctrl; /**< shoulder orientation during control. 0=roll,1=pitch,2=yaw */
        vector<vector<double>> shoulderLinearVelocity_ctrl; /**< shoulder linear velocity during control */
        vector<vector<double>> shoulderAngularVelocity_ctrl;/**< shoulder angular velocity during control */
        vector<vector<double>> shoulderLinearAcceleration_ctrl; /**< shoulder linear acceleration during control */
        vector<vector<double>> shoulderAngularAcceleration_ctrl;/**< shoulder angular acceleration during control */

        vector<double> error_pos_tot_norm; /**< norm of the total error in position */
        vector<double> error_or_tot_norm; /**< norm of the total error in orientation */
        vector<double> error_pos_or_tot_norm; /**< norm of the total error in position + orientation */
        vector<double> error_lin_vel_tot_norm; /**< norm of the total error in linear velocity */
        vector<double> error_ang_vel_tot_norm; /**< norm of the total error in angular velocity */
        vector<double> error_vel_tot_norm; /**< norm of the total error in velocity */
        vector<double> error_lin_acc_tot_norm; /**< norm of the total error in linear acceleration */
        vector<double> error_ang_acc_tot_norm; /**< norm of the total error in angular acceleration */
        vector<double> error_acc_tot_norm; /**< norm of the total error in acceleration */

        vector<double> sim_time; /**< simulation time [s]*/

        // low pass filter for the target object data
        boost::shared_ptr<LowPassFilter> lpf_tar_pos_x;
        boost::shared_ptr<LowPassFilter> lpf_tar_pos_y;
        boost::shared_ptr<LowPassFilter> lpf_tar_pos_z;
        boost::shared_ptr<LowPassFilter> lpf_tar_or_roll;
        boost::shared_ptr<LowPassFilter> lpf_tar_or_pitch;
        boost::shared_ptr<LowPassFilter> lpf_tar_or_yaw;

        // low pass filter for obstacles data
        boost::shared_ptr<LowPassFilter> lpf_obsts_pos_x;
        boost::shared_ptr<LowPassFilter> lpf_obsts_pos_y;
        boost::shared_ptr<LowPassFilter> lpf_obsts_pos_z;
        boost::shared_ptr<LowPassFilter> lpf_obsts_or_roll;
        boost::shared_ptr<LowPassFilter> lpf_obsts_or_pitch;
        boost::shared_ptr<LowPassFilter> lpf_obsts_or_yaw;

        // low pass filter for the position of the joints
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_1;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_2;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_3;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_4;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_5;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_6;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_7;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_8;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_9;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_10;
        boost::shared_ptr<LowPassFilter> lpf_joint_pos_11;




};

}  // namespace motion_manager

#endif // motion_manager_MAIN_WINDOW_H
