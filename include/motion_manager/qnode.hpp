/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef motion_manager_QNODE_HPP_
#define motion_manager_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <std_msgs/String.h>
#include <QThread>
#include <QStringListModel>
#include <vrep_common/VrepInfo.h>
#include <vrep_common/ProximitySensorData.h>
#include <algorithm>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>



#include <humplanner.hpp>




namespace motion_manager {

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std;
using namespace HUMotion;
using namespace logging::trivial;

//! The QNode class
/**
 * @brief This method defines the ROS node and its functionalities
 */
class QNode : public QThread
{
    Q_OBJECT
public:
        /**
         * @brief QNode, a constructor
         * @param argc
         * @param argv
         */
	QNode(int argc, char** argv );

        /**
         * @brief ~QNode, a destructor
         */
	virtual ~QNode();

        /**
         * @brief This method initializates the node
         * @return
         */
        bool on_init();

        /**
         * @brief This method initializates the node
         * @param master_url
         * @param host_url
         * @return
         */
        bool on_init_url(const string &master_url, const string &host_url);

        /**
         * @brief This method runs ending operations
         */
        void on_end();

        /**
         * @brief This method checks if V-REP is online
         * @return
         */
        bool checkVrep();

        /**
         * @brief This method checks if RViz is online
         * @return
         */
        bool checkRViz();

        /**
         * @brief This method loads the scenario with index id
         * @param path
         * @param id
         * @return
         */
        bool loadScenario(const string &path,int id);

        /**
         * @brief This method gets the elements of the scenario
         * @param scene
         * @return
         */
        bool getElements(scenarioPtr scene);

        /**
         * @brief getArmsHandles
         * @param humanoid
         * @return
         */
        bool getArmsHandles(int humanoid); // get the handles of both arms

        /**
         * @brief This method executes a movement
         * @param traj
         * @param vel
         * @param timeStep
         * @param tol_stop
         * @param mov
         * @param scene
         * @return
         */
        bool execMovement(MatrixXf& traj, MatrixXf& vel, float timeStep, float tol_stop, movementPtr mov, scenarioPtr scene); // execute the movement

        /**
         * @brief This method executes the movements in a task
         * @param traj_task
         * @param vel_task
         * @param timeSteps
         * @param nSteps
         * @param tols_stop
         * @param task
         * @param scene
         * @return
         */
        bool execTask(MatrixXf& traj_task, MatrixXf& vel_task, std::vector<float>& timeSteps, std::vector<int>& nSteps, std::vector<float>& tols_stop, taskPtr task, scenarioPtr scene); // execute the task

        /**
         * @brief This method sets to zero the time of simulation
         */
        void resetSimTime();

        /**
         * @brief This method resets some global variables
         */
        void resetGlobals();

        /**
         * @brief This method stops the simulation in V-REP
         */
        void stopSim();

        /**
         * @brief This is the run() method of the thread
         */
	void run();


        /**
         * @brief This enumerator is used for logging functionalities
         */
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

        /**
         * @brief This method return the list of loggings
         * @return
         */
        QStringListModel* loggingModel() { return &logging_model; }

        /**
         * @brief This method runs logging of the passed message
         * @param level
         * @param msg
         */
        void log( const LogLevel &level, const string &msg);



Q_SIGNALS:
        /**
         * @brief This signal is used to adjust the scrollbar of the logging list
         */
	void loggingUpdated();

        /**
         * @brief This signal is used to close the main window
         */
        void rosShutdown();

        /**
         * @brief This method signals that a new element is part of the scenario
         * @param value
         */
        void newElement(string value);

        /**
         * @brief updateElement
         * @param id
         * @param value
         */
        void updateElement(int id,string value);

        /**
         * @brief This method signals a new object in the scenario
         * @param value
         */
        void newObject(string value);

        /**
         * @brief This method signals that a new joint is part of the humanoid
         * @param value
         */
        void newJoint(string value);


private:

        // members
        int init_argc; /**< initial argc */
        char** init_argv; /**< initial argv */
        ros::ServiceClient add_client;/**<  ROS client */
        ros::Subscriber subInfo; /**< ROS subscriber for information about the simulation */
        ros::Subscriber subJoints_state; /**< ROS subscriber to the topic /vrep/joint_state */
        ros::Subscriber subRightProxSensor;/**< ROS subscriber to the topic /vrep/right_prox_sensor */
        ros::Subscriber subLeftProxSensor; /**< ROS subscriber to the topic /vrep/left_prox_sensor */
        // Toy vehicle scenario --------------------------------------------------------------------------------------------------------------
        ros::Subscriber subBlueColumn; /**< ROS sunscriber to the topic /vrep/BlueColumn_pose (obj_id=0 in the toy vehicle scenario) */
        ros::Subscriber subGreenColumn; /**< ROS sunscriber to the topic /vrep/GreenColumn_pose (obj_id=1 in the toy vehicle scenario) */
        ros::Subscriber subRedColumn; /**< ROS sunscriber to the topic /vrep/RedColumn_pose (obj_id=2 in the toy vehicle scenario) */
        ros::Subscriber subMagentaColumn; /**< ROS sunscriber to the topic /vrep/MagentaColumn_pose (obj_id=3 in the toy vehicle scenario) */
        ros::Subscriber subNut1; /**< ROS sunscriber to the topic /vrep/Nut1_pose (obj_id=4 in the toy vehicle scenario) */
        ros::Subscriber subNut2; /**< ROS sunscriber to the topic /vrep/Nut2_pose (obj_id=5 in the toy vehicle scenario) */
        ros::Subscriber subWheel1; /**< ROS sunscriber to the topic /vrep/Wheel1_pose (obj_id=6 in the toy vehicle scenario) */
        ros::Subscriber subWheel2; /**< ROS sunscriber to the topic /vrep/Wheel2_pose (obj_id=7 in the toy vehicle scenario) */
        ros::Subscriber subBase; /**< ROS sunscriber to the topic /vrep/Base_pose (obj_id=8 in the toy vehicle scenario) */
        //ros::Subscriber subTable; /**< ROS sunscriber to the topic /vrep/Table_pose (obj_id=9 in the toy vehicle scenario) */
        // -----------------------------------------------------------------------------------------------------------------------------------
        //ros::Subscriber subUpdateScene;
        //ros::Subscriber subUpdateTable;
        QStringListModel logging_model; /**< list of loggings */
        bool simulationRunning; /**< true if the simulation in V-REP is running */
        float simulationTime;/**< current time of the simulation */
        float simulationTimeStep;/**< current time step of the simulation */
        string nodeName; /**< name of the ROS node */
        float TotalTime; /**< total time of the movements */
        scenarioPtr curr_scene; /**< current scenario */
        movementPtr curr_mov; /**< current movement that is being executed */
        src::severity_logger< severity_level > lg; /**< logger */
        int right_sensor; /**< handle of the right hand proximity sensor */
        int left_sensor; /**< handle of the left hand proximity sensor */
        int h_detobj; /**< handle of the object that is currently detected by the proximity sensor of the end effector */
        int right_attach; /**< right hand attach point */
        int left_attach; /**< left hand attach point */
        bool got_scene; /**< true if we got all the elements of the scenario */
        bool obj_in_hand; /**< true if the object is in the hand */
        std::vector<int> right_handles; /**< right arm and right hand joints handles */
        std::vector<int> left_handles; /**< left arm and left hand joints handles */
        MatrixXi right_hand_handles; /**< matrix of the handles of the right hand joints */
        MatrixXi left_hand_handles; /**< matrix of the handles of the left hand joints */
        std::vector<float> right_2hand_pos; /**< position of the right hand 2 phalanx */
        std::vector<float> right_2hand_vel; /**< velocity of the right hand 2 phalanx */
        std::vector<float> right_2hand_force; /**< forces of the right hand 2 phalanx */
        std::vector<float> left_2hand_pos; /**< position of the left hand 2 phalanx */
        std::vector<float> left_2hand_vel; /**< velocity of the left hand 2 phalanx */
        std::vector<float> left_2hand_force; /**< forces of the left hand 2 phalanx */

#if HAND ==1
        std::vector<bool> firstPartLocked;
        std::vector<int> needFullOpening;
        std::vector<bool> closed;
#endif


        //methods

        /**
         * @brief This method gets the current date and time already formatted
         * @return
         */
        const string currentDateTime();

        /**
         * @brief This is the callback to retrieve information about the simulation in V-REP
         * @param info
         */
        void infoCallback(const vrep_common::VrepInfoConstPtr& info);

        /**
         * @brief This is the callback to retrieve the state of the joints
         * @param state
         */
        void JointsCallback(const sensor_msgs::JointState& state);

        /**
         * @brief This is the callback to retrieve the state of the proximity sensor on the right end-effector
         * @param data
         */
        void rightProxCallback(const vrep_common::ProximitySensorData& data);

        /**
         * @brief This is the callback to retrieve the state of the proximity sensor on the left end-effector
         * @param data
         */
        void leftProxCallback(const vrep_common::ProximitySensorData& data);

        // Toy vehicle scenario ----------------------------------------------------------------------------------
        /**
         * @brief This is the callback to retrieve the state of the blue column (toy vehicle scenario)
         * @param data
         */
        void BlueColumnCallback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the green column (toy vehicle scenario)
         * @param data
         */
        void GreenColumnCallback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the red column (toy vehicle scenario)
         * @param data
         */
        void RedColumnCallback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the magenta column (toy vehicle scenario)
         * @param data
         */
        void MagentaColumnCallback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the nut1 (toy vehicle scenario)
         * @param data
         */
        void Nut1Callback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the nut2 (toy vehicle scenario)
         * @param data
         */
        void Nut2Callback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the wheel1 (toy vehicle scenario)
         * @param data
         */
        void Wheel1Callback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the wheel2 (toy vehicle scenario)
         * @param data
         */
        void Wheel2Callback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This is the callback to retrieve the state of the base (toy vehicle scenario)
         * @param data
         */
        void BaseCallback(const geometry_msgs::PoseStamped& data);

        //void TableCallback(const geometry_msgs::PoseStamped& data);

        /**
         * @brief This method returns the linear interpolation
         * @param ya
         * @param yb
         * @param m
         * @return
         */
        float interpolate(float ya, float yb, float m);

        /**
         * @brief This method initializate the logging
         */
        void init();

#if HAND == 1
        /**
         * @brief This method closes the Barrett hand
         * @param hand
         * @return
         */
        bool closeBarrettHand(int hand);

        /**
         * @brief This method opens the Barrett hand
         * @param hand
         * @return
         */
        bool openBarrettHand(int hand);
#endif
        /**
         * @brief This method return the RPY values starting from the transformation matrix
         * @param Trans
         * @param rpy
         * @return
         */
        bool getRPY(Matrix4f Trans, std::vector<float>& rpy);

        /**
         * @brief This method update the information of a generic object in V-REP
         * @param obj_id
         * @param name
         * @param data
         */
        void updateObjectInfo(int obj_id,std::string name, const geometry_msgs::PoseStamped &data);


};

}  // namespace motion_manager

#endif /* motion_manager_QNODE_HPP_ */
