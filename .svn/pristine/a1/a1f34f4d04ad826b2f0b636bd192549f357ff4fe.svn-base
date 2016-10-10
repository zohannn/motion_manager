/**
 * @file /include/MotionPlanner/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MotionPlanner_QNODE_HPP_
#define MotionPlanner_QNODE_HPP_

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



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace MotionPlanner {

bool getArmsHandles(int humanoid); // get the handles of both arms

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std;
using namespace HUMotion;
using namespace logging::trivial;




/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        bool on_init();
        bool on_init(const string &master_url, const string &host_url);
        void on_end();
        bool checkVrep();
        bool loadScenario(const string &path,int id);
        bool getElements(scenarioPtr scene); // get the elements of a given scenario
        bool execMovement(MatrixXf& traj, MatrixXf& vel, float timeStep, float tol_stop, movementPtr mov, scenarioPtr scene); // execute the movement
        bool execTask(MatrixXf& traj_task, MatrixXf& vel_task, std::vector<float>& timeSteps, std::vector<int>& nSteps, std::vector<float>& tols_stop, taskPtr task, scenarioPtr scene); // execute the task
        void resetSimTime();
        void resetGlobals();
        void stopSim();

	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
        void log( const LogLevel &level, const string &msg);



Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();
        void newElement(string value);
        void updateElement(int id,string value);
        void newObject(string value);
        void newJoint(string value);


private:
        // members
	int init_argc;
	char** init_argv;
        ros::ServiceClient add_client;
        // subscribers
        ros::Subscriber subInfo;
        ros::Subscriber subJoints_state;
        ros::Subscriber subRightProxSensor;
        ros::Subscriber subLeftProxSensor;
        // Toy vehicle scenario ---------------------------
        ros::Subscriber subBlueColumn; // Bluecolumn (obj_id=0)
        ros::Subscriber subGreenColumn; // GreenColumn (obj_id=1)
        ros::Subscriber subRedColumn; // RedColumn (obj_id=2)
        ros::Subscriber subMagentaColumn; // MagentaColumn (obj_id=3)
        ros::Subscriber subNut1; //Nut1 (obj_id=4)
        ros::Subscriber subNut2; //Nut2 (obj_id=5)
        ros::Subscriber subWheel1; // Wheel1 (obj_id=6)
        ros::Subscriber subWheel2; //Wheel2 (obj_id=7)
        ros::Subscriber subBase; //Base (obj_id=8)
        ros::Subscriber subTable; // Table (obj_id=9)


        ros::Subscriber subUpdateScene;
        //ros::Subscriber subUpdateTable;
        QStringListModel logging_model;
        bool simulationRunning;
        float simulationTime;
        float simulationTimeStep;
        string nodeName;
        float TotalTime; // total time of the movements
        scenarioPtr curr_scene; // current scenario
        movementPtr curr_mov; // current movement being executed
        src::severity_logger< severity_level > lg; // logging


        //methods
        const string currentDateTime();
        void checkVrepCallback(const std_msgs::String::ConstPtr& msg);
        void infoCallback(const vrep_common::VrepInfoConstPtr& info);
        void JointsCallback(const sensor_msgs::JointState& state);
        void rightProxCallback(const vrep_common::ProximitySensorData& data);
        void leftProxCallback(const vrep_common::ProximitySensorData& data);
        // BlueColumn (obj_id=0)
        void BlueColumnCallback(const geometry_msgs::PoseStamped& data);
        // GreenColumn (obj_id=1)
        void GreenColumnCallback(const geometry_msgs::PoseStamped& data);
        // RedColumn (obj_id=2)
        void RedColumnCallback(const geometry_msgs::PoseStamped& data);
        // Magenta Column (obj_id=3)
        void MagentaColumnCallback(const geometry_msgs::PoseStamped& data);
        // Nut 1 (obj_id=4)
        void Nut1Callback(const geometry_msgs::PoseStamped& data);
        // Nut 2 (obj_id=5)
        void Nut2Callback(const geometry_msgs::PoseStamped& data);
        // Wheel 1 (obj_id=6)
        void Wheel1Callback(const geometry_msgs::PoseStamped& data);
        // Wheel 2(obj_id=7)
        void Wheel2Callback(const geometry_msgs::PoseStamped& data);
        // Base (obj_id=8)
        void BaseCallback(const geometry_msgs::PoseStamped& data);
        // Table (obj_id=9)
        void TableCallback(const geometry_msgs::PoseStamped& data);



        //void infoScene(const std_msgs::StringConstPtr& msg);

        float interpolate(float ya, float yb, float m);
        void init(); // init logging
        bool closeARoSHand(int hand);
        bool openARoSHand(int hand);
        bool getRPY(Matrix4f Trans, std::vector<float>& rpy);
        void updateObjectInfo(int obj_id,std::string name, const geometry_msgs::PoseStamped &data);


};

}  // namespace MotionPlanner

#endif /* MotionPlanner_QNODE_HPP_ */
