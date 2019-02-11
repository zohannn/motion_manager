
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>

#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <time.h>
#include "../include/motion_manager/qnode.hpp"
#include <vrep_common/simRosLoadScene.h>
#include <vrep_common/simRosCloseScene.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>
#include <vrep_common/simRosGetFloatSignal.h>
#include <vrep_common/simRosGetIntegerSignal.h>
#include <vrep_common/simRosGetStringSignal.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <vrep_common/simRosEnableSubscriber.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosReadProximitySensor.h>
#include <vrep_common/simRosSetObjectParent.h>
#include <vrep_common/JointSetStateData.h>
#include<vrep_common/simRosSetObjectIntParameter.h>
#include<vrep_common/simRosSetJointForce.h>
#include<vrep_common/simRosSetJointPosition.h>

#include <geometric_shapes/solid_primitive_dims.h>

#include "../include/motion_manager/v_repConst.hpp"



namespace motion_manager {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
    nodeName = "motion_manager";
    TotalTime = 0.0;
    right_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    left_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    right_2hand_pos.assign(3,0.0f);
    right_2hand_vel.assign(3,0.0f);
    right_2hand_force.assign(3,0.0f);
    left_2hand_pos.assign(3,0.0f);
    left_2hand_vel.assign(3,0.0f);
    left_2hand_force.assign(3,0.0f);
    got_scene = false;
    obj_in_hand = false;

#if HAND ==1
    firstPartLocked.assign(3,false);
    needFullOpening.assign(3,0);
    closed.assign(3,false);
#endif


    // logging
    init();
    logging::add_common_attributes();

}

QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown();
    }
    wait();
}

bool QNode::on_init()
{
    ros::init(init_argc,init_argv,"motion_manager");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start();
    start();
	return true;
}

bool QNode::on_init_url(const std::string &master_url, const std::string &host_url)
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
    ros::init(remappings,"motion_manager");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start();
    start();
	return true;
}

void QNode::on_end()
{


}

bool  QNode::loadScenario(const std::string& path,int id)
{

    ros::NodeHandle n;

    // close the old scene
    add_client = n.serviceClient<vrep_common::simRosCloseScene>("/vrep/simRosCloseScene");
    vrep_common::simRosCloseScene srvc;
    add_client.call(srvc);
    // load the new scene
    add_client = n.serviceClient<vrep_common::simRosLoadScene>("/vrep/simRosLoadScene");
    vrep_common::simRosLoadScene srv;
    srv.request.fileName = path;
    add_client.call(srv);
    int res = srv.response.result;

    if (res == 1){

        subInfo = n.subscribe("/vrep/info",1, &QNode::infoCallback,this);

        subJoints_state = n.subscribe("/vrep/joints_state",1, &QNode::JointsCallback, this);
        subRightProxSensor = n.subscribe("/vrep/right_prox_sensor",1,&QNode::rightProxCallback,this);
        subLeftProxSensor = n.subscribe("/vrep/left_prox_sensor",1,&QNode::leftProxCallback,this);
        subRightHandPos = n.subscribe("/vrep/right_hand_pose",1,&QNode::rightHandPosCallback,this);
        subRightHandVel = n.subscribe("/vrep/right_hand_vel",1,&QNode::rightHandVelCallback,this);
        subLeftHandPos = n.subscribe("/vrep/left_hand_pose",1,&QNode::leftHandPosCallback,this);
        subLeftHandVel = n.subscribe("/vrep/left_hand_vel",1,&QNode::leftHandVelCallback,this);

        switch(id){

        case 0: case 1: case 6:
            // Assembly scenario: the Toy vehicle with ARoS
            // Assembly scenario: the Toy vehicle with Jarde
            // Toy vehicle dual arm to swap columns

            // Blue Column (obj_id = 0)
            subBlueColumn = n.subscribe("/vrep/BlueColumn_pose",1,&QNode::BlueColumnCallback,this);
            // Green Column (obj_id = 1)
            subGreenColumn = n.subscribe("/vrep/GreenColumn_pose",1,&QNode::GreenColumnCallback,this);
            // RedColumn (obj_id = 2)
            subRedColumn = n.subscribe("/vrep/RedColumn_pose",1,&QNode::RedColumnCallback,this);
            // MagentaColumn (obj_id = 3)
            subMagentaColumn = n.subscribe("/vrep/MagentaColumn_pose",1,&QNode::MagentaColumnCallback,this);
            // Nut 1 (obj_id = 4)
            subNut1 = n.subscribe("/vrep/Nut1_pose",1,&QNode::Nut1Callback,this);
            // Nut 2 (obj_id = 5)
            subNut2 = n.subscribe("/vrep/Nut2_pose",1,&QNode::Nut2Callback,this);
            // Wheel 1 (obj_id = 6)
            subWheel1 = n.subscribe("/vrep/Wheel1_pose",1,&QNode::Wheel1Callback,this);
            // Wheel 2 (obj_id = 7)
            subWheel2 = n.subscribe("/vrep/Wheel2_pose",1,&QNode::Wheel2Callback,this);
            // Base (obj_id = 8)
            subBase = n.subscribe("/vrep/Base_pose",1,&QNode::BaseCallback,this);
            // Table (obj_id = 9)
            //subTable = n.subscribe("/vrep/Table_pose",1,&QNode::TableCallback,this);

            break;

        case 7: // Human assistance scenario: Moving a tray with ARoS (dual-arm)
            // Bottle Tea (obj_id = 0)
            subBottleTea = n.subscribe("/vrep/BottleTea_pose",1,&QNode::BottleTeaCallback,this);
            // Bottle Coffee (obj_id = 1)
            subBottleCoffee = n.subscribe("/vrep/BottleCoffee_pose",1,&QNode::BottleCoffeeCallback,this);
            // Bottle Juice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose",1,&QNode::BottleJuiceCallback,this);
            // Cup (obj_id = 3)
            //subCup = n.subscribe("/vrep/Cup_pose",1,&QNode::CupCallback,this);
            // Tray (obj_id = 3)
            subTray = n.subscribe("/vrep/Tray_pose",1,&QNode::TrayCallback,this);
            // Cup 1 (obj_id = 4)
            subCup1 = n.subscribe("/vrep/Cup1_pose",1,&QNode::Cup1Callback,this);
            // Box (obj_id = 6)
            subBox = n.subscribe("/vrep/Box_pose",1,&QNode::BoxCallback,this);
            break;

        case 4:
            // Human assistance scenario: Serving a drink with ARoS
            // Bottle Tea (obj_id = 0)
            subBottleTea = n.subscribe("/vrep/BottleTea_pose",1,&QNode::BottleTeaCallback,this);
            // Bottle Coffee (obj_id = 1)
            subBottleCoffee = n.subscribe("/vrep/BottleCoffee_pose",1,&QNode::BottleCoffeeCallback,this);
            // Bottle Juice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose",1,&QNode::BottleJuiceCallback,this);
            // Cup (obj_id = 3)
            subCup = n.subscribe("/vrep/Cup_pose",1,&QNode::CupCallback,this);
            // Cup 1 (obj_id = 4)
            subCup1 = n.subscribe("/vrep/Cup1_pose",1,&QNode::Cup1Callback,this);

            break;

        case 5:
            // Challenging scenario: picking a cup from a shelf with ARoS
            // Cup  (obj_id = 0)
            subCup_shelf = n.subscribe("/vrep/Cup_pose",1,&QNode::Cup_shelfCallback,this);
            // Shelf  (obj_id = 1)
            subShelf = n.subscribe("/vrep/Shelf_pose",1,&QNode::ShelfCallback,this);
            // Shelf_1_b  (obj_id = 2)
            subShelf_1_b = n.subscribe("/vrep/Shelf_1_b_pose",1,&QNode::Shelf_1_bCallback,this);
            // Shelf_2_a  (obj_id = 3)
            subShelf_2_a = n.subscribe("/vrep/Shelf_2_a_pose",1,&QNode::Shelf_2_aCallback,this);
            // Shelf_2_b  (obj_id = 4)
            subShelf_2_b = n.subscribe("/vrep/Shelf_2_b_pose",1,&QNode::Shelf_2_bCallback,this);
            // Shelf_3  (obj_id = 5)
            subShelf_3 = n.subscribe("/vrep/Shelf_3_pose",1,&QNode::Shelf_3Callback,this);
            // Shelf_4_a  (obj_id = 6)
            subShelf_4_a = n.subscribe("/vrep/Shelf_4_a_pose",1,&QNode::Shelf_4_aCallback,this);
            // Shelf_4_b  (obj_id = 7)
            subShelf_4_b = n.subscribe("/vrep/Shelf_4_b_pose",1,&QNode::Shelf_4_bCallback,this);
            // Shelf_4_c  (obj_id = 8)
            subShelf_4_c = n.subscribe("/vrep/Shelf_4_c_pose",1,&QNode::Shelf_4_cCallback,this);
            // Shelf_4_d  (obj_id = 9)
            subShelf_4_d = n.subscribe("/vrep/Shelf_4_d_pose",1,&QNode::Shelf_4_dCallback,this);

            break;

        case 8: // Natural obstacle avoidance with ARoS
            // Cylinder small  (obj_id = 0)
            subCylinderSmall= n.subscribe("/vrep/Cylinder_small_pose",1,&QNode::Cylinder_small_Callback,this);
            // Cylinder tall  (obj_id = 1)
            subCylinderTall = n.subscribe("/vrep/Cylinder_tall_pose",1,&QNode::Cylinder_tall_Callback,this);
            // Bottle Juice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose",1,&QNode::BottleJuiceCallback,this);

            pub_joints = n.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);

            break;

        case 9: // Learning Tasks: reaching with one obstacle
            // Obstacle (obj_id = 0)
            subObstacle = n.subscribe("/vrep/Obstacle_pose",1,&QNode::Obstacle_Callback,this);

        case 10: // Learning Tasks: reaching with many obstacles
            // Cylinder small  (obj_id = 0)
            subCylinderSmall= n.subscribe("/vrep/Cylinder_small_pose",1,&QNode::Cylinder_small_Callback,this);
            // Cylinder tall  (obj_id = 1)
            subCylinderTall = n.subscribe("/vrep/Cylinder_tall_pose",1,&QNode::Cylinder_tall_Callback,this);
            // BottleJuice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose",1,&QNode::BottleJuiceCallback,this);

        case 11: // Learning Tasks: picking the Blue Column
            // Blue Column (obj_id = 0)
            subBlueColumn = n.subscribe("/vrep/BlueColumn_pose",1,&QNode::BlueColumnCallback,this);

            break;

        case 12: case 13: // Controlling scenarios without objects
            pub_joints = n.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);

            break;

        case 14: // Controlling: scenario with one obstacle and draw an allipse on the XY plane
            pub_joints = n.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
            // Cylinder small  (obj_id = 0)
            subCylinderSmall= n.subscribe("/vrep/Cylinder_small_pose",1,&QNode::Cylinder_small_Callback,this);
            break;
        case 15: case 16: // Controlling: pick a red column and Controlling: follow a moving red column
            pub_joints = n.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
            // Cylinder small  (obj_id = 0)
            subCylinderSmall= n.subscribe("/vrep/Cylinder_small_pose",1,&QNode::Cylinder_small_Callback,this);
            // Cylinder tall  (obj_id = 1)
            subCylinderTall = n.subscribe("/vrep/Cylinder_tall_pose",1,&QNode::Cylinder_tall_Callback,this);
            // RedColumn (obj_id = 2)
            subRedColumn = n.subscribe("/vrep/RedColumn_pose",1,&QNode::RedColumnCallback,this);
            break;
        }
#if MOVEIT==1
        // planning scene of RViZ
        planning_scene_interface_ptr.reset(new moveit::planning_interface::PlanningSceneInterface());
#endif
        return true;
    }else{
        return false;
    }


}

#if MOVEIT==1
void QNode::loadRVizScenario(std::vector<objectPtr> &objs)
{
    vector<string> rem_object_ids;
    vector<moveit_msgs::CollisionObject> add_collision_objects;

    for(size_t i=0; i<objs.size();++i){
        objectPtr obj = objs.at(i); string name = obj->getName();
        string mesh_file;
        if(strcmp(name.c_str(),"Table")==0){
            mesh_file = "table/table.dae";
        }else if((strcmp(name.c_str(),"BlueColumn")==0)||
                 (strcmp(name.c_str(),"GreenColumn")==0)||
                 (strcmp(name.c_str(),"RedColumn")==0)||
                 (strcmp(name.c_str(),"MagentaColumn")==0)){
            mesh_file = "column/column.dae";
        }else if((strcmp(name.c_str(),"Nut1")==0)||
                 (strcmp(name.c_str(),"Nut2")==0)){
            mesh_file = "nut/nut.dae";
        }else if((strcmp(name.c_str(),"Wheel1")==0)||
                 (strcmp(name.c_str(),"Wheel2")==0)){
            mesh_file = "wheel/wheel.dae";
        }else if(strcmp(name.c_str(),"Base")==0){
            mesh_file = "base/base.dae";
        }else if((strcmp(name.c_str(),"BottleTea")==0)||
                 (strcmp(name.c_str(),"BottleCoffee")==0)||
                 (strcmp(name.c_str(),"BottleJuice")==0)){
            mesh_file = "bottle/bottle.dae";
        }else if((strcmp(name.c_str(),"Cup")==0)||
                 (strcmp(name.c_str(),"Cup1")==0)){
            mesh_file = "cup/cup.dae";
        }else if((strcmp(name.c_str(),"Shelf")==0)||
                 (strcmp(name.c_str(),"Shelf_1_b")==0)){
            mesh_file = "shelf/shelf_1.dae";
        }else if((strcmp(name.c_str(),"Shelf_2_a")==0)||
                (strcmp(name.c_str(),"Shelf_2_b")==0)){
            mesh_file = "shelf/shelf_2.dae";
        }else if(strcmp(name.c_str(),"Shelf_3")==0){
            mesh_file = "shelf/shelf_3.dae";
        }else if((strcmp(name.c_str(),"Shelf_4_a")==0)||
                 (strcmp(name.c_str(),"Shelf_4_b")==0)||
                 (strcmp(name.c_str(),"Shelf_4_c")==0)||
                 (strcmp(name.c_str(),"Shelf_4_d")==0)){
            mesh_file = "shelf/shelf_4.dae";
        }

        std::vector<double> rpy = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
        Matrix3d Rot; this->RPY_matrix(rpy,Rot); Quaterniond q(Rot);

        string mesh_path = string("package://models/meshes/")+mesh_file;

        moveit_msgs::CollisionObject co;
        co.header.stamp = ros::Time::now();
        co.header.frame_id = FRAME_ID;
        // remove
        co.id = name;
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        rem_object_ids.push_back(co.id);
        // add
        co.operation = moveit_msgs::CollisionObject::ADD;
        shapes::Mesh* table_shape = shapes::createMeshFromResource(mesh_path.c_str());
        shapes::ShapeMsg table_mesh_msg;
        shapes::constructMsgFromShape(table_shape,table_mesh_msg);
        shape_msgs::Mesh table_mesh = boost::get<shape_msgs::Mesh>(table_mesh_msg);
        co.meshes.resize(1);
        co.meshes[0] = table_mesh;
        co.mesh_poses.resize(1);
        co.mesh_poses[0].position.x = obj->getPos().Xpos/1000; // [m]
        co.mesh_poses[0].position.y = obj->getPos().Ypos/1000; // [m]
        co.mesh_poses[0].position.z = obj->getPos().Zpos/1000; // [m]
        co.mesh_poses[0].orientation.w= q.w();
        co.mesh_poses[0].orientation.x= q.x();
        co.mesh_poses[0].orientation.y= q.y();
        co.mesh_poses[0].orientation.z= q.z();
        add_collision_objects.push_back(co);
    }
    // remove
    planning_scene_interface_ptr->removeCollisionObjects(rem_object_ids);
    // add
    planning_scene_interface_ptr->addCollisionObjects(add_collision_objects);

    /* Sleep so we have time to see the object in RViz */
    ros::WallDuration(5.0).sleep();

}
#endif

void QNode::resetSimTime()
{

    this->TotalTime=0.0;
    this->simulationTime = 0.0;
}

void QNode::resetGlobals()
{

#if HAND == 1
    for (int i =0; i < 3; ++i){
        closed.at(i)=false;
        needFullOpening.at(i)=0;
        firstPartLocked.at(i)=false;
    }
#endif

    obj_in_hand = false;
}


bool QNode::getElements(scenarioPtr scene)
{


    ros::NodeHandle n;


    // start the simulation
    this->startSim();
    sleep(1);

    int n_objs; // total number of objects in the scenario
    int n_poses; // total number of poses in the scenario
    int cnt_obj = 0; // index of the object being loaded
    int cnt_pose = 0; // index of the pose being loaded
    std::string signPrefix = ""; // prefix of each object
    std::string infoLine; // info line in the list of elements
    std::string signTarRight = "_targetRight";
    std::string signTarLeft = "_targetLeft";
    std::string signEngage ="_engage";
    bool succ = true;


    pos obj_pos;// position of the object
    orient obj_or;// orientation of the object
    dim obj_size;// size of the object
    pos tarRight_pos;// target right position
    orient tarRight_or;// target right orientation
    pos tarLeft_pos;// target left position
    orient tarLeft_or;// target left orientation
    pos engage_pos; // engage point position
    orient engage_or;// engage point orientation

    pos humanoid_pos; // position of the humanoid
    orient humanoid_or; // orientation of the humanoid
    dim humanoid_size; // size of the humanoid
    arm humanoid_arm_specs; // specs of the arms
    barrett_hand humanoid_hand_specs; // specs of the barret hand

    // **** object info **** //
    std::string obj_info_str;
    std::vector<double> obj_info_vec;
    std::vector<std::string> objs_prefix;
    // **** pose info **** //
    std::string pose_info_str;
    std::vector<double> pose_info_vec;
    pos pose_pos;// position of the pose
    orient pose_or;// orientation of the pose
    std::vector<std::string> poses_prefix; // names of the poses
    std::vector<bool> poses_rel; // relations of the poses
    std::vector<int> poses_obj_id; // id of the related object

    vrep_common::simRosGetIntegerSignal srvi;
    vrep_common::simRosGetFloatSignal srvf;
    vrep_common::simRosGetStringSignal srvs;
    vrep_common::simRosGetObjectHandle srv_get_handle;
    ros::ServiceClient client_getHandle;

    // name of the humanoid
    std::string Hname;

    int floatCount;
    std::vector<double> DH_params_vec;
    string DH_params_str;

    // matrices from world to right and left references
    std::string mat_arms_str;
    std::string mat_right_arm_str;
    std::string mat_left_arm_str;
    std::vector<double> mat_arms_vec;
    std::vector<double> mat_right_arm_vec;
    std::vector<double> mat_left_arm_vec;
    Matrix4d mat_right;
    Matrix4d mat_left;
    // matrices from the last joint of the arm to the palm of the hand
    std::string r_mat_hand_str;
    std::string l_mat_hand_str;
    std::vector<double> r_mat_hand_vec;
    std::vector<double> l_mat_hand_vec;
    Matrix4d mat_r_hand;
    Matrix4d mat_l_hand;

    int rows;

    const string NOBJECTS = string("n_objects");
    const string NPOSES = string("n_poses");

    // parameters of the barrett hand
    double maxAp; //[m]
    double Aw; // [m]
    double A1; // [m]
    double A2; // [m]
    double A3; // [m]
    double D3; // [m]
    double phi2; // [rad]
    double phi3; // [rad]

    // parameters of the human hand
    double max_human_Ap; //[m]
    human_hand jarde_hand;
    jarde_hand.fingers = std::vector<human_finger>(4);
    human_finger fing1 = jarde_hand.fingers.at(0);
    human_finger fing2 = jarde_hand.fingers.at(1);
    human_finger fing3 = jarde_hand.fingers.at(2);
    human_finger fing4 = jarde_hand.fingers.at(3);
    human_thumb thumb = jarde_hand.thumb;

    // torso
    humanoid_part torso;  // parameters of the torso (ARoS and Jarde)
    std::string torso_str;
    std::vector<double> torso_vec;
    /*
    // pelvis
    humanoid_part pelvis; // parameters of the pelvis (Jarde)
    std::string pelvis_str;
    std::vector<double> pelvis_vec;
    //right_upper_leg
    humanoid_part right_upper_leg; // parameters of the right upper leg (Jarde)
    std::string r_upper_leg_str;
    std::vector<double> r_upper_leg_vec;
    humanoid_part left_upper_leg; // parameters of the left upper leg (Jarde)
    std::string l_upper_leg_str;
    std::vector<double> l_upper_leg_vec;
    */
    // postures of the humanoid
    std::vector<double> rposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // right
    std::vector<double> lposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // left

    // joint limits of the humanoid
    std::vector<double> min_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum right limits
    std::vector<double> max_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum right limits
    std::vector<double> min_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum left limits
    std::vector<double> max_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum left limits


    int scenarioID  = scene->getID();
    switch (scenarioID){

    case 0:
        // error, no scenario
        throw string("No scenario");
        break;
    case 1:
        // Assembly scenario: the Toy vehicle with ARoS

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BlueColumn");     // obj_id = 0
        objs_prefix.push_back("GreenColumn");    // obj_id = 1
        objs_prefix.push_back("RedColumn");      // obj_id = 2
        objs_prefix.push_back("MagentaColumn");  // obj_id = 3
        objs_prefix.push_back("Nut1");           // obj_id = 4
        objs_prefix.push_back("Nut2");           // obj_id = 5
        objs_prefix.push_back("Wheel1");         // obj_id = 6
        objs_prefix.push_back("Wheel2");         // obj_id = 7
        objs_prefix.push_back("Base");           // obj_id = 8
        objs_prefix.push_back("Table");          // obj_id = 9


        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;

            }else{succ = false;}

            if (succ){

                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects

        // get the info of the Humanoid

        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);
            //hptr->setMatRightHand(mat_r_hand);  hptr->setMatLeftHand(mat_l_hand);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{

            throw string("Error while retrieving elements from the scenario");
        }

        break;

    case 2: // Assembly toy vehicle scenario with the Avatar

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;

        }else{succ = false; throw string("Communication error");}

        // get the info of the scenario        

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BlueColumn");     // obj_id = 0
        objs_prefix.push_back("GreenColumn");    // obj_id = 1
        objs_prefix.push_back("RedColumn");      // obj_id = 2
        objs_prefix.push_back("MagentaColumn");  // obj_id = 3
        objs_prefix.push_back("Nut1");           // obj_id = 4
        objs_prefix.push_back("Nut2");           // obj_id = 5
        objs_prefix.push_back("Wheel1");         // obj_id = 6
        objs_prefix.push_back("Wheel2");         // obj_id = 7
        objs_prefix.push_back("Base");           // obj_id = 8
        objs_prefix.push_back("Table");          // obj_id = 9


        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;

            }else{succ = false;}

            if (succ){

                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[mm]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[mm]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[mm]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                scene->addObject(objectPtr(ob));

                cnt_obj++;


            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        // get the info of the Humanoid

        // name
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;

        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(1);

        // transformation matrix for both arms
        // from the reference frame to the arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("mat_arms");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_arms_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_arms_vec.empty()){mat_arms_vec.clear();}
        floatCount = mat_arms_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_arms_vec.push_back(static_cast<double>(((float*)mat_arms_str.c_str())[k]));

        // from the last joint of the arm to the palm of the hand
        // right hand
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("r_mat_hand");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             r_mat_hand_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the right hand");}
        if(!r_mat_hand_vec.empty()){r_mat_hand_vec.clear();}
        floatCount = r_mat_hand_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            r_mat_hand_vec.push_back(static_cast<double>(((float*)r_mat_hand_str.c_str())[k]));
        // left hand
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("l_mat_hand");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             l_mat_hand_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the left hand");}
        if(!l_mat_hand_vec.empty()){l_mat_hand_vec.clear();}
        floatCount = l_mat_hand_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            l_mat_hand_vec.push_back(static_cast<double>(((float*)l_mat_hand_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                    mat_r_hand(i,j) = 0;
                    mat_l_hand(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                    mat_r_hand(i,j) = 1;
                    mat_l_hand(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_arms_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_arms_vec.at(j+rows*4)*1000; //[mm]
                    mat_r_hand(i,j) = 0;
                    mat_l_hand(i,j) = 0;
                }else{
                    mat_right(i,j) = mat_arms_vec.at(j+rows*4);
                    mat_left(i,j) = mat_arms_vec.at(j+rows*4);
                    mat_r_hand(i,j) = r_mat_hand_vec.at(j+rows*4);
                    mat_l_hand(i,j) = l_mat_hand_vec.at(j+rows*4);;
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }

#if HAND==0
        // Human Hands
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             max_human_Ap= srvf.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the max aperture of tha hand");}
        jarde_hand.maxAperture=max_human_Ap*1000; // [mm]

        // finger 1 (index)
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_fing1");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the finger 1");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        fing1 = jarde_hand.fingers.at(0);
        fing1.ux = DH_params_vec.at(0)*1000; // [mm]
        fing1.uy = DH_params_vec.at(1)*1000; // [mm]
        fing1.uz = DH_params_vec.at(2)*1000; // [mm]
        fing1.finger_specs.alpha = std::vector<double>(4);
        fing1.finger_specs.a = std::vector<double>(4);
        fing1.finger_specs.d = std::vector<double>(4);
        fing1.finger_specs.theta = std::vector<double>(4);
        for(int i=0;i<4;++i){
            fing1.finger_specs.alpha.at(i) = DH_params_vec.at(i+3)*static_cast<double>(M_PI)/180; // [rad]
            fing1.finger_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            fing1.finger_specs.d.at(i) = DH_params_vec.at(i+11)*1000; // [mm]
            fing1.finger_specs.theta.at(i) = DH_params_vec.at(i+15)*static_cast<double>(M_PI)/180; // [rad]
        }
        jarde_hand.fingers.at(0) = fing1;

        // finger 2 (middle)
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_fing2");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the finger 2");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        fing2 = jarde_hand.fingers.at(1);
        fing2.ux = DH_params_vec.at(0)*1000; // [mm]
        fing2.uy = DH_params_vec.at(1)*1000; // [mm]
        fing2.uz = DH_params_vec.at(2)*1000; // [mm]
        fing2.finger_specs.alpha = std::vector<double>(4);
        fing2.finger_specs.a = std::vector<double>(4);
        fing2.finger_specs.d = std::vector<double>(4);
        fing2.finger_specs.theta = std::vector<double>(4);
        for(int i=0;i<4;++i){
            fing2.finger_specs.alpha.at(i) = DH_params_vec.at(i+3)*static_cast<double>(M_PI)/180; // [rad]
            fing2.finger_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            fing2.finger_specs.d.at(i) = DH_params_vec.at(i+11)*1000; // [mm]
            fing2.finger_specs.theta.at(i) = DH_params_vec.at(i+15)*static_cast<double>(M_PI)/180; // [rad]
        }
        jarde_hand.fingers.at(1) = fing2;


        // finger 3 (ring)
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_fing3");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the finger 3");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        fing3 = jarde_hand.fingers.at(2);
        fing3.ux = DH_params_vec.at(0)*1000; // [mm]
        fing3.uy = DH_params_vec.at(1)*1000; // [mm]
        fing3.uz = DH_params_vec.at(2)*1000; // [mm]
        fing3.finger_specs.alpha = std::vector<double>(4);
        fing3.finger_specs.a = std::vector<double>(4);
        fing3.finger_specs.d = std::vector<double>(4);
        fing3.finger_specs.theta = std::vector<double>(4);
        for(int i=0;i<4;++i){
            fing3.finger_specs.alpha.at(i) = DH_params_vec.at(i+3)*static_cast<double>(M_PI)/180; // [rad]
            fing3.finger_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            fing3.finger_specs.d.at(i) = DH_params_vec.at(i+11)*1000; // [mm]
            fing3.finger_specs.theta.at(i) = DH_params_vec.at(i+15)*static_cast<double>(M_PI)/180; // [rad]
        }
        jarde_hand.fingers.at(2) = fing3;


        // finger 4 (little)
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_fing4");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the finger 4");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        fing4 = jarde_hand.fingers.at(3);
        fing4.ux = DH_params_vec.at(0)*1000; // [mm]
        fing4.uy = DH_params_vec.at(1)*1000; // [mm]
        fing4.uz = DH_params_vec.at(2)*1000; // [mm]
        fing4.finger_specs.alpha = std::vector<double>(4);
        fing4.finger_specs.a = std::vector<double>(4);
        fing4.finger_specs.d = std::vector<double>(4);
        fing4.finger_specs.theta = std::vector<double>(4);
        for(int i=0;i<4;++i){
            fing4.finger_specs.alpha.at(i) = DH_params_vec.at(i+3)*static_cast<double>(M_PI)/180; // [rad]
            fing4.finger_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            fing4.finger_specs.d.at(i) = DH_params_vec.at(i+11)*1000; // [mm]
            fing4.finger_specs.theta.at(i) = DH_params_vec.at(i+15)*static_cast<double>(M_PI)/180; // [rad]
        }
        jarde_hand.fingers.at(3) = fing4;

        // thumb
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_thumb");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the thumb");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        thumb = jarde_hand.thumb;
        thumb.uTx = DH_params_vec.at(0)*1000; // [mm]
        thumb.uTy = DH_params_vec.at(1)*1000; // [mm]
        thumb.uTz = DH_params_vec.at(2)*1000; // [mm]
        thumb.thumb_specs.alpha = std::vector<double>(5);
        thumb.thumb_specs.a = std::vector<double>(5);
        thumb.thumb_specs.d = std::vector<double>(5);
        thumb.thumb_specs.theta = std::vector<double>(5);
        for(int i=0;i<5;++i){
            thumb.thumb_specs.alpha.at(i) = DH_params_vec.at(i+3)*static_cast<double>(M_PI)/180; // [rad]
            thumb.thumb_specs.a.at(i) = DH_params_vec.at(i+8)*1000; // [mm]
            thumb.thumb_specs.d.at(i) = DH_params_vec.at(i+13)*1000; // [mm]
            thumb.thumb_specs.theta.at(i) = DH_params_vec.at(i+18)*static_cast<double>(M_PI)/180; // [rad]
        }
        jarde_hand.thumb = thumb;

#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]

        // Pelvis
        /*
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("PelvisInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             pelvis_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the pelvis");}
        floatCount = pelvis_str.size()/sizeof(float);
        if (!pelvis_vec.empty()){pelvis_vec.clear();}
        for (int k=0;k<floatCount;++k)
            pelvis_vec.push_back(static_cast<double>(((float*)pelvis_str.c_str())[k]));

        pelvis.Xpos = pelvis_vec.at(0)*1000;//[mm]
        pelvis.Ypos = pelvis_vec.at(1)*1000;//[mm]
        pelvis.Zpos = pelvis_vec.at(2)*1000;//[mm]
        pelvis.Roll = pelvis_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        pelvis.Pitch = pelvis_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        pelvis.Yaw = pelvis_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        pelvis.Xsize = pelvis_vec.at(6)*1000;//[mm]
        pelvis.Ysize = pelvis_vec.at(7)*1000;//[mm]
        pelvis.Zsize = pelvis_vec.at(8)*1000;//[mm]
        */

        // right upper leg
        /*
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("UpperLegRightInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             r_upper_leg_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the right upper leg");}
        floatCount = r_upper_leg_str.size()/sizeof(float);
        if (!r_upper_leg_vec.empty()){r_upper_leg_vec.clear();}
        for (int k=0;k<floatCount;++k)
            r_upper_leg_vec.push_back(static_cast<double>(((float*)r_upper_leg_str.c_str())[k]));

        right_upper_leg.Xpos = r_upper_leg_vec.at(0)*1000;//[mm]
        right_upper_leg.Ypos = r_upper_leg_vec.at(1)*1000;//[mm]
        right_upper_leg.Zpos = r_upper_leg_vec.at(2)*1000;//[mm]
        right_upper_leg.Roll = r_upper_leg_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        right_upper_leg.Pitch = r_upper_leg_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        right_upper_leg.Yaw = r_upper_leg_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        right_upper_leg.Xsize = r_upper_leg_vec.at(6)*1000;//[mm]
        right_upper_leg.Ysize = r_upper_leg_vec.at(7)*1000;//[mm]
        right_upper_leg.Zsize = r_upper_leg_vec.at(8)*1000;//[mm]
        */

        // left upper leg
        /*
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("UpperLegLeftInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             l_upper_leg_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the left upper leg");}
        floatCount = l_upper_leg_str.size()/sizeof(float);
        if (!l_upper_leg_vec.empty()){l_upper_leg_vec.clear();}
        for (int k=0;k<floatCount;++k)
            l_upper_leg_vec.push_back(static_cast<double>(((float*)l_upper_leg_str.c_str())[k]));

        left_upper_leg.Xpos = l_upper_leg_vec.at(0)*1000;//[mm]
        left_upper_leg.Ypos = l_upper_leg_vec.at(1)*1000;//[mm]
        left_upper_leg.Zpos = l_upper_leg_vec.at(2)*1000;//[mm]
        left_upper_leg.Roll = l_upper_leg_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        left_upper_leg.Pitch = l_upper_leg_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        left_upper_leg.Yaw = l_upper_leg_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        left_upper_leg.Xsize = l_upper_leg_vec.at(6)*1000;//[mm]
        left_upper_leg.Ysize = l_upper_leg_vec.at(7)*1000;//[mm]
        left_upper_leg.Zsize = l_upper_leg_vec.at(8)*1000;//[mm]
        */

        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");

        // *** right home (park) posture for Jarde [deg] *** //
        //arm
        // Joint 0 = -90
        // Joint 1 = -80
        // Joint 2 = 90
        // Joint 3 = 0
        // Joint 4 = 0
        // Joint 5 = 0
        // Joint 6 = 0
        //hand
        // Joint 7 = 0
        // Joint 8 = 70
        // Joint 9 = 70
        // Joint 10 = 70

        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }

        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }

        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }

        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");

        // *** left home (park) posture for Jarde [deg] *** //
        // arm
        // Joint 0 = -90
        // Joint 1 = -80
        // Joint 2 = 90
        // Joint 3 = 0
        // Joint 4 = 0
        // Joint 5 = 0
        // Joint 6 = 0
        //hand
        // Joint 7 = 0
        // Joint 8 = 70
        // Joint 9 = 70
        // Joint 10 = 70

        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }

        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }

        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;

            }else{succ = false;}
        }



        if(succ){

            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;

#if HAND==0
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, jarde_hand,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);

            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);
            hptr->setMatRightHand(mat_r_hand);
            hptr->setMatLeftHand(mat_l_hand);
            //hptr->setPelvis(pelvis);
            //hptr->setRight_Upper_leg(right_upper_leg);
            //hptr->setLeft_Upper_leg(left_upper_leg);

            scene->addHumanoid(hptr);


            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;

            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);

            std::vector<string> rj = std::vector<string>(rightp.size());
            for (int i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (int i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }

#else
            throw("You have probably chosen the wrong hand type");
#endif


        }else{

            throw string("Error while retrieving elements from the scenario");
        }

        break;

    case 3: case 4:
        // Empty scenario: empty scenario with ARoS
        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}

        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}

        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}

#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9

        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03

        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }


        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;

            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);


            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);
            //hptr->setMatRightHand(mat_r_hand); hptr->setMatLeftHand(mat_l_hand);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;

            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);

            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);

            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;
    case 5:
        // Human assistance scenario: Serving a drink with ARoS

        // get the number of objects and the number of poses in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BottleTea");      // obj_id = 0
        objs_prefix.push_back("BottleCoffee");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");    // obj_id = 2
        objs_prefix.push_back("Cup");            // obj_id = 3
        objs_prefix.push_back("Cup1");           // obj_id = 4
        objs_prefix.push_back("Table");          // obj_id = 5

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects

        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("BottleJuice_pose1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);
        // pose_id = 1
        poses_prefix.push_back("BottleJuice_pose2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);
        // pose_id = 2
        poses_prefix.push_back("BottleJuice_pose3");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses

        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}
        // get the handles of both arms
        succ = getArmsHandles(0);
        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k){
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
        }
        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));
        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }
#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

           // hptr->setMatRightHand(mat_r_hand); hptr->setMatLeftHand(mat_l_hand);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);

            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{

            throw string("Error while retrieving elements from the scenario");
        }
        break;
    case 6:
        // Challenging scenario: picking a cup from a shelf with ARoS

        // get the number of objects and the number of poses in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Cup");        // obj_id = 0
        objs_prefix.push_back("Shelf");      // obj_id = 1
        objs_prefix.push_back("Shelf_1_b");  // obj_id = 2
        objs_prefix.push_back("Shelf_2_a");  // obj_id = 3
        objs_prefix.push_back("Shelf_2_b");  // obj_id = 4
        objs_prefix.push_back("Shelf_3");    // obj_id = 5
        objs_prefix.push_back("Shelf_4_a");  // obj_id = 6
        objs_prefix.push_back("Shelf_4_b");  // obj_id = 7
        objs_prefix.push_back("Shelf_4_c");  // obj_id = 8
        objs_prefix.push_back("Shelf_4_d");  // obj_id = 9
        objs_prefix.push_back("Table");      // obj_id = 10

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects

        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Cup_pose");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }

        }// while loop poses

        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}
        // get the handles of both arms
        succ = getArmsHandles(0);
        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k){
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
        }
        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}
        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));
        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }
#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);
            //hptr->setMatRightHand(mat_r_hand); hptr->setMatLeftHand(mat_l_hand);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);

            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{

            throw string("Error while retrieving elements from the scenario");
        }
        break;
    case 7:
        // Toy vehicle scenario: swap two columns
        // Assembly scenario: the Toy vehicle with ARoS

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BlueColumn");     // obj_id = 0
        objs_prefix.push_back("GreenColumn");    // obj_id = 1
        objs_prefix.push_back("RedColumn");      // obj_id = 2
        objs_prefix.push_back("MagentaColumn");  // obj_id = 3
        objs_prefix.push_back("Nut1");           // obj_id = 4
        objs_prefix.push_back("Nut2");           // obj_id = 5
        objs_prefix.push_back("Wheel1");         // obj_id = 6
        objs_prefix.push_back("Wheel2");         // obj_id = 7
        objs_prefix.push_back("Base");           // obj_id = 8
        objs_prefix.push_back("Table");          // obj_id = 9


        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;

            }else{succ = false;}

            if (succ){

                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects

        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("GreenColumn_pose1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);
        // pose_id = 1
        poses_prefix.push_back("MagentaColumn_pose1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses

        // get the info of the Humanoid

        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<7;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);
            //hptr->setMatRightHand(mat_r_hand); hptr->setMatLeftHand(mat_l_hand);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }


        break;
    case 8:
        // Human assistance scenario: Moving a tray with ARoS

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BottleTea");      // obj_id = 0
        objs_prefix.push_back("BottleCoffee");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");    // obj_id = 2
        objs_prefix.push_back("Tray");           // obj_id = 3
        objs_prefix.push_back("Cup1");           // obj_id = 4
        objs_prefix.push_back("Table");          // obj_id = 5
        objs_prefix.push_back("Box");          // obj_id = 6


        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;

            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                if(signPrefix.compare("Tray")==0)
                {
                    ob->set_dFF(20);
                    ob->set_dFH(10);
                }
                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Tray_poseLeft");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);
        // pose_id = 1
        poses_prefix.push_back("Tray_poseRight");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;

    case 9:
        // Natural obstacle avoidance with ARoS

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Cylinder_small");   // obj_id = 0
        objs_prefix.push_back("Cylinder_tall");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");   // obj_id = 2
        objs_prefix.push_back("Table");          // obj_id = 3



        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;

            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{

                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(2);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;

    case 10:
        // Learning tasks: reaching with one obstacle

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Obstacle");   // obj_id = 0

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }

        break;

    case 11:
        // Learning tasks: reaching with many obstacles

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Cylinder_small");   // obj_id = 0
        objs_prefix.push_back("Cylinder_tall");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");   // obj_id = 2
        objs_prefix.push_back("Table");          // obj_id = 3

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }

        break;

    case 12:
        // Learning tasks: picking the Blue Column

        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("BlueColumn");   // obj_id = 0

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]



        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;

    case 13:
        // Controlling: scenario without objects
        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        //objs_prefix.push_back("BlueColumn");   // obj_id = 0

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]

        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;

    case 14:
        // Controlling: scenario without objects for singularities
        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        //objs_prefix.push_back("BlueColumn");   // obj_id = 0

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]

        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;
    case 15:
        // // Controlling: scenario with one obstacle and drawing an ellipse on the XY plane
        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Cylinder_small");   // obj_id = 0

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]

        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;

    case 16: case 17:
        //  Controlling: pick a red column and Controlling: follow a moving red column
        // get the number of objects in the scenario
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_objs= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}


        /*
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1){
             n_poses= srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}
        */


        // get the info of the scenario

        // get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        // this is the order of the object in this scenario
        objs_prefix.push_back("Cylinder_small");   // obj_id = 0
        objs_prefix.push_back("Cylinder_tall");   // obj_id = 1
        objs_prefix.push_back("RedColumn");   // obj_id = 2

        while(cnt_obj < n_objs){
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

                // position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                // orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
                // size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]


                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                //Pose* ps = new Pose(signPrefix+string("_home"),obj_pos,obj_or,true,cnt_obj);
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);


                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                // get the handles  of the object
                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);
                // handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                // add the object to the scenario
                scene->addObject(objectPtr(ob));
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_obj++;
            }else{
                throw string("Error while retrieving the objects of the scenario");
            }
        } // while loop objects


        /*
        // this is the order of the poses in this scenario
        // pose_id = 0
        poses_prefix.push_back("Target_pose_1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // pose_id = 1
        poses_prefix.push_back("Target_pose_2");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1){
                 pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if (succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }// while loop poses
        */


        // get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             Hname= srvs.response.signalValue;
        }else{succ = false;}

        // get the handles of both arms
        succ = getArmsHandles(0);

        // transformation matrix for both arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
        floatCount = mat_right_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));

        rows=0;
        for(int i=0;i<4;++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        // Arms
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("DH_params_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             DH_params_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the DH parameters of the arm");}

        floatCount = DH_params_str.size()/sizeof(float);
        if (!DH_params_vec.empty()){DH_params_vec.clear();}
        for (int k=0;k<floatCount;++k)
            DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

        humanoid_arm_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_specs.arm_specs.theta = std::vector<double>(7);
        for(int i=0;i<JOINTS_ARM;++i){
            humanoid_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
            humanoid_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+JOINTS_ARM)*1000; // [mm]
            humanoid_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+(2*JOINTS_ARM))*1000; // [mm]
        }


#if HAND==1

        // Barrett Hand
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             maxAp= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             Aw= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A1= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A2= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             A3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             D3= srvf.response.signalValue*1000;
        }else{succ = false;}
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi2= srvf.response.signalValue;
        }else{succ = false;}
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if (srvf.response.result == 1){
             phi3= srvf.response.signalValue;
        }else{succ = false;}
#endif

        // Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if (!torso_vec.empty()){torso_vec.clear();}
        for (int k=0;k<floatCount;++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000;//[mm]
        torso.Ypos = torso_vec.at(1)*1000;//[mm]
        torso.Zpos = torso_vec.at(2)*1000;//[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
        torso.Xsize = torso_vec.at(6)*1000;//[mm]
        torso.Ysize = torso_vec.at(7)*1000;//[mm]
        torso.Zsize = torso_vec.at(8)*1000;//[mm]

        // right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** right home (park) posture for ARoS [deg] *** //
        // Joint 0 = -137.5
        // Joint 1 = -77.94
        // Joint 2 = 106.6
        // Joint 3 = -95.4
        // Joint 4 = -43.28
        // Joint 5 = -64
        // Joint 6 = 47.9
        for (size_t i = 0; i <rposture.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 rposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum right limits
        for (size_t i = 0; i <min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum right limits
        for (size_t i = 0; i <max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_rlimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // *** left home (park) posture for ARoS [deg] *** //
        // Joint 0 = 137.5
        // Joint 1 = -77.94
        // Joint 2 = -106.6
        // Joint 3 = -95.4
        // Joint 4 = 43.28
        // Joint 5 = -64
        // Joint 6 = 132.03
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        if (succ){
            // create the new humanoid and add it to the scenario.
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll =  torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            humanoid_hand_specs.maxAperture = maxAp;
            humanoid_hand_specs.Aw = Aw;
            humanoid_hand_specs.A1 = A1;
            humanoid_hand_specs.A2 = A2;
            humanoid_hand_specs.A3 = A3;
            humanoid_hand_specs.D3 = D3 ;
            humanoid_hand_specs.phi2 = phi2;
            humanoid_hand_specs.phi3 = phi3;

#if HAND==1
            Humanoid *hptr = new Humanoid(Hname,humanoid_pos,humanoid_or,humanoid_size,humanoid_arm_specs, humanoid_hand_specs,
                                          rposture, lposture,
                                          min_rlimits,max_rlimits,
                                          min_llimits,max_llimits);
            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            // get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            for (size_t i=0; i<rightp.size(); i++ ){
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(rj.at(i));
            }
            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ ){
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                       QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                Q_EMIT newJoint(lj.at(i));
            }
            // display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));
#else
            throw("You have probably chosen the wrong hand type");

#endif
        }else{
            throw string("Error while retrieving elements from the scenario");
        }
        break;
    }// switch scenario

    this->curr_scene = scene;

    // stop the simulation
    this->stopSim();
    got_scene = true;

    return succ;

}



void QNode::BlueColumnCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "blue_callback"  ;

    int obj_id = 0;
    string name = string("BlueColumn");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::GreenColumnCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "green_callback"  ;

    int obj_id = 1;
    string name = string("GreenColumn");

    this->updateObjectInfo(obj_id,name,data);

 }


void QNode::RedColumnCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "red_callback"  ;

    int obj_id = 2;
    string name = string("RedColumn");

    this->updateObjectInfo(obj_id,name,data);

}


void QNode::MagentaColumnCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "magenta_callback"  ;

    int obj_id = 3;
    string name = string("MagentaColumn");

    this->updateObjectInfo(obj_id,name,data);

}


void QNode::Nut1Callback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "nut1_callback"  ;


    int obj_id = 4;
    string name = string("Nut1");

    this->updateObjectInfo(obj_id,name,data);

}


void QNode::Nut2Callback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "nut2_callback"  ;


    int obj_id = 5;
    string name = string("Nut2");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Wheel1Callback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "wheel1_callback"  ;

    int obj_id = 6;
    string name = string("Wheel1");

    this->updateObjectInfo(obj_id,name,data);

}


void QNode::Wheel2Callback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "wheel2_callback"  ;

    int obj_id = 7;
    string name = string("Wheel2");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::BaseCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "base_callback"  ;

    int obj_id = 8;
    string name = string("Base");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::TopCallback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "top_callback"  ;

    int obj_id = 8;
    string name = string("Top");

    this->updateObjectInfo(obj_id,name,data);
}

/*
void QNode::TableCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "table_callback"  ;

    int obj_id = 9;
    string name = string("Table");

    this->updateObjectInfo(obj_id,name,data);

}

*/


void QNode::BottleTeaCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "bottle_tea_callback"  ;

    int obj_id = 0;
    string name = string("BottleTea");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::BottleCoffeeCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "bottle_coffee_callback"  ;

    int obj_id = 1;
    string name = string("BottleCoffee");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::BottleJuiceCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "bottle_juice_callback"  ;

    int obj_id = 2;
    string name = string("BottleJuice");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::CupCallback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "cup_callback"  ;

    int obj_id = 3;
    string name = string("Cup");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Cup1Callback(const geometry_msgs::PoseStamped &data)
{

    //BOOST_LOG_SEV(lg, info) << "cup_1_callback"  ;

    int obj_id = 4;
    string name = string("Cup1");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::TrayCallback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "cup_1_callback"  ;

    int obj_id = 3;
    string name = string("Tray");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::BoxCallback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "cup_1_callback"  ;

    int obj_id = 6;
    string name = string("Box");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Cup_shelfCallback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "cup_shelf_callback"  ;

    int obj_id = 3;
    string name = string("Cup");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::ShelfCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_callback"  ;

    int obj_id = 1;
    string name = string("Shelf");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_1_bCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_1_bcallback"  ;

    int obj_id = 2;
    string name = string("Shelf_1_b");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_2_aCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_2_acallback"  ;

    int obj_id = 3;
    string name = string("Shelf_2_a");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_2_bCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_2_bcallback"  ;

    int obj_id = 4;
    string name = string("Shelf_2_b");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_3Callback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_3callback"  ;

    int obj_id = 5;
    string name = string("Shelf_3");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_4_aCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_4_acallback"  ;

    int obj_id = 6;
    string name = string("Shelf_4_a");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_4_bCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_4_bcallback"  ;

    int obj_id = 7;
    string name = string("Shelf_4_b");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_4_cCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_4_ccallback"  ;

    int obj_id = 8;
    string name = string("Shelf_4_c");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Shelf_4_dCallback(const geometry_msgs::PoseStamped &data)
{
    //BOOST_LOG_SEV(lg, info) << "shelf_4_dcallback"  ;

    int obj_id = 9;
    string name = string("Shelf_4_d");

    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Cylinder_small_Callback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "cylinder_small_callback"  ;

    int obj_id = 0;
    string name = string("Cylinder_small");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::Cylinder_tall_Callback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "cylinder_tall_callback"  ;

    int obj_id = 1;
    string name = string("Cylinder_tall");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::Obstacle_Callback(const geometry_msgs::PoseStamped& data)
{
    //BOOST_LOG_SEV(lg, info) << "obstacle_callback"  ;

    int obj_id = 0;
    string name = string("Obstacle");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::updateObjectInfo(int obj_id, string name, const geometry_msgs::PoseStamped &data)
{


    std::vector<double> rpy;
    objectPtr obj = this->curr_scene->getObject(name);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    obj->setPos(poss,true);

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy)){
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
        obj->setOr(orr,true);
    }else{
        // TO DO
        // singularity: leave the previous orientation
    }


    /*

    BOOST_LOG_SEV(lg, info) << "position ";
    BOOST_LOG_SEV(lg, info) << "data pos x = " << data.pose.position.x * 1000;
    BOOST_LOG_SEV(lg, info) << "data pos y = " << data.pose.position.y * 1000;
    BOOST_LOG_SEV(lg, info) << "data pos z = " << data.pose.position.z * 1000;
    BOOST_LOG_SEV(lg, info) << "obj pos x = " << obj->getPos().Xpos;
    BOOST_LOG_SEV(lg, info) << "obj pos y = " << obj->getPos().Ypos;
    BOOST_LOG_SEV(lg, info) << "obj pos z = " << obj->getPos().Zpos;
    BOOST_LOG_SEV(lg, info) << "tar right pos x = " << obj->getTargetRight()->getPos().Xpos;
    BOOST_LOG_SEV(lg, info) << "tar right pos y = " << obj->getTargetRight()->getPos().Ypos;
    BOOST_LOG_SEV(lg, info) << "tar right pos z = " << obj->getTargetRight()->getPos().Zpos;
    BOOST_LOG_SEV(lg, info) << "tar left pos x = " << obj->getTargetLeft()->getPos().Xpos;
    BOOST_LOG_SEV(lg, info) << "tar left pos y = " << obj->getTargetLeft()->getPos().Ypos;
    BOOST_LOG_SEV(lg, info) << "tar left pos z = " << obj->getTargetLeft()->getPos().Zpos;
    BOOST_LOG_SEV(lg, info) << "engage x = " << obj->getEngagePoint()->getPos().Xpos;
    BOOST_LOG_SEV(lg, info) << "engage y = " << obj->getEngagePoint()->getPos().Ypos;
    BOOST_LOG_SEV(lg, info) << "engage z = " << obj->getEngagePoint()->getPos().Zpos;
    BOOST_LOG_SEV(lg, info) << "orientation ";
    BOOST_LOG_SEV(lg, info) << "data or x = " << data.pose.orientation.x;
    BOOST_LOG_SEV(lg, info) << "data or y = " << data.pose.orientation.y;
    BOOST_LOG_SEV(lg, info) << "data or z = " << data.pose.orientation.z;
    BOOST_LOG_SEV(lg, info) << "obj roll  = " << obj->getOr().roll;
    BOOST_LOG_SEV(lg, info) << "obj pitch = " << obj->getOr().pitch;
    BOOST_LOG_SEV(lg, info) << "obj yaw = " << obj->getOr().yaw;
    BOOST_LOG_SEV(lg, info) << "tar right roll = " << obj->getTargetRight()->getOr().roll;
    BOOST_LOG_SEV(lg, info) << "tar right pitch = " << obj->getTargetRight()->getOr().pitch;
    BOOST_LOG_SEV(lg, info) << "tar right yaw = " << obj->getTargetRight()->getOr().yaw;
    BOOST_LOG_SEV(lg, info) << "tar left roll = " << obj->getTargetLeft()->getOr().roll;
    BOOST_LOG_SEV(lg, info) << "tar left pitch = " << obj->getTargetLeft()->getOr().pitch;
    BOOST_LOG_SEV(lg, info) << "tar left yaw = " << obj->getTargetLeft()->getOr().yaw;
    BOOST_LOG_SEV(lg, info) << "engage roll = " << obj->getEngagePoint()->getOr().roll;
    BOOST_LOG_SEV(lg, info) << "engage pitch = " << obj->getEngagePoint()->getOr().pitch;
    BOOST_LOG_SEV(lg, info) << "engage yaw = " << obj->getEngagePoint()->getOr().yaw;
    BOOST_LOG_SEV(lg, info) << "\n";


    */

    string info = obj->getInfoLine();
    Q_EMIT updateElement(obj_id,info);
    if(this->curr_scene){
        this->curr_scene->setObject(obj_id,obj);
    }

}

bool QNode::getRPY(Matrix4d Trans, std::vector<double> &rpy)
{

    rpy = std::vector<double>(3);


    if((abs(Trans(0,0)) < 1e-5) && (abs(Trans(1,0)) < 1e-5)){
        // singularity
        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

        return false;

    }else{

        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        double sp = sin(rpy.at(0));
        double cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

        return true;
    }


}

void QNode::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty()){
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        // Rot = Rot_z * Rot_y * Rot_x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

    }
}


void QNode::infoCallback(const vrep_common::VrepInfoConstPtr& info)
{

    simulationTime=info->simulationTime.data;
    simulationTimeStep=info->timeStep.data;
    simulationRunning=(info->simulatorState.data&1)!=0;
    //printf("simulation time: %f [sec]\n",simulationTime);
}

void QNode::rightProxCallback(const vrep_common::ProximitySensorData& data)
{

    //ros::NodeHandle node;
    //BOOST_LOG_SEV(lg, info) << "right_prox_callback"  ;

    // set the detected object child of the attach point
    //add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
    //vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object

    if (this->curr_mov){
        int arm_code = this->curr_mov->getArm();
        //if (arm_code == 1){
            //right arm
            int h_obj; int h_obj_body;
            int mov_type = this->curr_mov->getType();
            switch (mov_type) {
            case 0: // reach-to-grasp
                h_obj_body = this->curr_mov->getObject()->getHandleBody(); // visible handle of the object we want to grasp
                h_obj = this->curr_mov->getObject()->getHandle(); // non visible handle of the object we want to grasp
                if (arm_code == 1)
                {
                    // right arm
                    h_detobj = data.detectedObject.data; // handle of the object currently detected
                }else if(arm_code == 0){
                    // dual arm
                    r_h_detobj = data.detectedObject.data; // handle of the object currently detected
                }
                //BOOST_LOG_SEV(lg, info) << "h_obj = " << h_obj ;
                //BOOST_LOG_SEV(lg, info) << "\n " ;
                //BOOST_LOG_SEV(lg, info) << "h_detobj = " << h_detobj ;
                //BOOST_LOG_SEV(lg, info) << "\n " ;
                if (arm_code == 1)
                {
                    // right arm
                    obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
                }else if(arm_code == 0){
                    // dual arm
                    obj_in_r_hand = (h_obj == r_h_detobj) || (h_obj_body == r_h_detobj);
                }
                break;
            case 1: // reaching
                break;
            case 2: // transport
                break;
            case 3: // engage
                break;
            case 4: // disengage
                break;
            case 5: // go park
                break;
            }
        //}
    }
}

void QNode::leftProxCallback(const vrep_common::ProximitySensorData& data)
{

    //BOOST_LOG_SEV(lg, info) << "left_prox_callback"  ;
    if (this->curr_mov){
        int arm_code = this->curr_mov->getArm();
        //if (arm_code == 2){
            //left arm
            int h_obj; int h_obj_body;
            int mov_type = this->curr_mov->getType();
            switch (mov_type){
            case 0: // reach-to-grasp
                if (arm_code == 2)
                {
                    // left arm
                    h_obj_body = this->curr_mov->getObject()->getHandleBody(); // visible handle of the object we want to grasp
                    h_obj = this->curr_mov->getObject()->getHandle(); // non visible handle of the object we want to grasp
                    h_detobj = data.detectedObject.data; // handle of the object currently detected
                }else if(arm_code == 0){
                    // dual arm
                    h_obj_body = this->curr_mov->getObjectLeft()->getHandleBody(); // visible handle of the object we want to grasp
                    h_obj = this->curr_mov->getObjectLeft()->getHandle(); // non visible handle of the object we want to grasp
                    l_h_detobj = data.detectedObject.data; // handle of the object currently detected
                }
                //BOOST_LOG_SEV(lg, info) << "h_obj = " << h_obj ;
                //BOOST_LOG_SEV(lg, info) << "\n " ;
                //BOOST_LOG_SEV(lg, info) << "h_detobj = " << h_detobj ;
                //BOOST_LOG_SEV(lg, info) << "\n " ;
                if (arm_code == 2)
                {
                    // left arm
                    obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
                }else if(arm_code == 0){
                    // dual arm
                    obj_in_l_hand = (h_obj == l_h_detobj) || (h_obj_body == l_h_detobj);
                }
                break;
            case 1: // reaching
                break;
            case 2: // transport
                break;
            case 3: // engage
                break;
            case 4: // disengage
                break;
            case 5: // go park
                break;
            }
        //}
    }
}

void QNode::rightHandPosCallback(const geometry_msgs::PoseStamped& data)
{
    vector<double> hand_pos_mes(6);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    vector<double> rpy;
    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy)){
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
    }else{
        // TO DO
        // singularity: leave the previous orientation
    }

    hand_pos_mes.at(0) = poss.Xpos;
    hand_pos_mes.at(1) = poss.Ypos;
    hand_pos_mes.at(2) = poss.Zpos;
    hand_pos_mes.at(3) = orr.roll;
    hand_pos_mes.at(4) = orr.pitch;
    hand_pos_mes.at(5) = orr.yaw;

    this->curr_scene->getHumanoid()->setHandPosMes(1,hand_pos_mes);

}

void QNode::rightHandVelCallback(const geometry_msgs::TwistStamped& data)
{
    vector<double> hand_vel_mes(6);

    hand_vel_mes.at(0) = data.twist.linear.x;
    hand_vel_mes.at(1) = data.twist.linear.y;
    hand_vel_mes.at(2) = data.twist.linear.z;
    hand_vel_mes.at(3) = data.twist.angular.x;
    hand_vel_mes.at(4) = data.twist.angular.y;
    hand_vel_mes.at(5) = data.twist.angular.z;

    this->curr_scene->getHumanoid()->setHandVelMes(1,hand_vel_mes);

}

void QNode::leftHandPosCallback(const geometry_msgs::PoseStamped& data)
{
    vector<double> hand_pos_mes(6);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    vector<double> rpy;
    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy)){
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
    }else{
        // TO DO
        // singularity: leave the previous orientation
    }

    hand_pos_mes.at(0) = poss.Xpos;
    hand_pos_mes.at(1) = poss.Ypos;
    hand_pos_mes.at(2) = poss.Zpos;
    hand_pos_mes.at(3) = orr.roll;
    hand_pos_mes.at(4) = orr.pitch;
    hand_pos_mes.at(5) = orr.yaw;

    this->curr_scene->getHumanoid()->setHandPosMes(2,hand_pos_mes);
}

void QNode::leftHandVelCallback(const geometry_msgs::TwistStamped& data)
{
    vector<double> hand_vel_mes(6);

    hand_vel_mes.at(0) = data.twist.linear.x;
    hand_vel_mes.at(1) = data.twist.linear.y;
    hand_vel_mes.at(2) = data.twist.linear.z;
    hand_vel_mes.at(3) = data.twist.angular.x;
    hand_vel_mes.at(4) = data.twist.angular.y;
    hand_vel_mes.at(5) = data.twist.angular.z;

    this->curr_scene->getHumanoid()->setHandVelMes(2,hand_vel_mes);
}

bool QNode::execMovement(std::vector<MatrixXd>& traj_mov, std::vector<MatrixXd>& vel_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string>& traj_descr,movementPtr mov, scenarioPtr scene)
{

    this->curr_scene = scene;
    //int scenarioID = scene->getID();
    this->curr_mov = mov; int mov_type = mov->getType();  int arm_code = mov->getArm();
    bool plan; bool approach; bool retreat;
    bool hand_closed;

    switch (mov_type){
    case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
        closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
        break;
    case 2: case 3: case 4: // transport, engage, disengage
        closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
        break;
    }


    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    double pre_time= 0.0;

    std::vector<int> handles;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int h_attach; // handle of the attachment point of the hand
    std::vector<int> r_handles;
    MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int r_h_attach; // handle of the attachment point of the hand
    std::vector<int> l_handles;
    MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int l_h_attach; // handle of the attachment point of the hand

    switch (arm_code) {
    case 0: // dual arm
        r_handles = right_handles;
        r_hand_handles = right_hand_handles;
        r_h_attach = right_attach;
        l_handles = left_handles;
        l_hand_handles = left_hand_handles;
        l_h_attach = left_attach;
        break;
    case 1: //right arm
        handles = right_handles;
        hand_handles = right_hand_handles;
        h_attach = right_attach;
        break;
    case 2: // left arm
        handles = left_handles;
        hand_handles = left_hand_handles;
        h_attach = left_attach;
        break;
    }

    // set joints position or velocity (it depends on the settings)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

#if HAND==1
    // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
    ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
#endif
    VectorXd f_posture; // the final posture
    bool f_reached;
    double tol_stop_stage;
    std::vector<double> timesteps_stage;
    MatrixXd traj;
    MatrixXd vel;
    double timeTot = 0.0;

    MatrixXd traj_plan_approach;
    MatrixXd vel_plan_approach;
    std::vector<double> timesteps_plan_approach;
    bool join_plan_approach = false;

    if(mov_type==0 || mov_type==2 || mov_type==3 || mov_type==4){
        // reach-to-grasp, transport, engage, disengage
        if(traj_mov.size() > 1){
            // there is more than one stage
            string mov_descr_1 = traj_descr.at(0);
            string mov_descr_2 = traj_descr.at(1);
            if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"approach")==0)){
                join_plan_approach = true;
                traj_plan_approach.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                vel_plan_approach.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());

            }
        }

    }

    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages


    for (size_t k=0; k< traj_mov.size();++k){  // for loop stages

        string mov_descr = traj_descr.at(k);
        if(strcmp(mov_descr.c_str(),"plan")==0){
            plan=true; approach=false; retreat=false;
            if(join_plan_approach){
                MatrixXd tt = traj_mov.at(k);
                MatrixXd vv = vel_mov.at(k);
                std::vector<double> ttsteps = timesteps.at(k);
                traj_plan_approach.topLeftCorner(tt.rows(),tt.cols()) = tt;
                vel_plan_approach.topLeftCorner(vv.rows(),vv.cols()) = vv;
                timesteps_plan_approach.reserve(ttsteps.size());
                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
                continue;
            }
        }else if(strcmp(mov_descr.c_str(),"approach")==0){
            plan=false; approach=true; retreat=false;
            if(join_plan_approach){
                MatrixXd tt = traj_mov.at(k); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                MatrixXd vv = vel_mov.at(k); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                std::vector<double> ttsteps = timesteps.at(k);
                traj_plan_approach.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                vel_plan_approach.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                //if(moveit_mov){
                timesteps_plan_approach.reserve(ttsteps.size());
                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
                //}else{
                    //timesteps_plan_approach.reserve(ttsteps.size()-1);
                    //std::copy (ttsteps.begin()+1, ttsteps.end(), std::back_inserter(timesteps_plan_approach));
                //}
            }
        }else if(strcmp(mov_descr.c_str(),"retreat")==0){
            plan=false; approach=false; retreat=true;
        }

        switch (mov_type){
        case 0: // reach-to-grasp
            if(retreat){
                if(arm_code!=0){
                    //single-arm
                    if(obj_in_hand){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                    this->closeBarrettHand(arm_code);
#else
                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                    this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
#endif
                    }
                }else{
                    // dual-arm
                    if(obj_in_r_hand){ // right arm
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = r_h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                    this->closeBarrettHand(1);
#else
                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                    this->closeBarrettHand_to_pos(1,hand_init_pos);
#endif
                    }

                    if(obj_in_l_hand){ // left arm
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = l_h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                    this->closeBarrettHand(2);
#else
                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                    this->closeBarrettHand_to_pos(2,hand_init_pos);
#endif
                    }
                }
            }
            break;
        case 1: // reaching
            break;
        case 2: case 3: // transport, engage
            if(retreat){
                if(arm_code!=0){
                    // single-arm
                    //ros::spinOnce();// handle ROS messages
                    if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //if(obj_in_hand){
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = -1; // parentless object
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in releasing the object "));
                            }
                    }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                  //this->openBarrettHand(arm_code);
                  MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                  std::vector<double> hand_init_pos;
                  hand_init_pos.resize(init_h_posture.size());
                  VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                  this->openBarrettHand_to_pos(arm_code,hand_init_pos);
#else
                closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                }else{
                    // dual-arm
                    // right arm
                    //ros::spinOnce();// handle ROS messages
                    if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //if(obj_in_hand){
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = -1; // parentless object
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in releasing the object "));
                            }
                    }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                  //this->openBarrettHand(1);
                  MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                  std::vector<double> hand_init_pos;
                  hand_init_pos.resize(init_h_posture.size());
                  VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                  this->openBarrettHand_to_pos(1,hand_init_pos);
#else
                closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                // left arm
                //ros::spinOnce();// handle ROS messages
                if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    //if(obj_in_hand){
                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = -1; // parentless object
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in releasing the object "));
                        }                    
                }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
              //this->openBarrettHand(2);
              MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
              std::vector<double> hand_init_pos;
              hand_init_pos.resize(init_h_posture.size());
              VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
              this->openBarrettHand_to_pos(2,hand_init_pos);
#else
            closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                }
            }
            break;
        case 4:// disengage
            break;
        case 5: // go-park
            break;
        }


        if(join_plan_approach && (strcmp(mov_descr.c_str(),"approach")==0)){
            traj = traj_plan_approach;
            vel = vel_plan_approach;
            timesteps_stage = timesteps_plan_approach;
        }else{
            traj = traj_mov.at(k);
            vel = vel_mov.at(k);
            timesteps_stage = timesteps.at(k);
        }
        tol_stop_stage = tols_stop.at(k);
        f_posture = traj.row(traj.rows()-1);
        f_reached=false;



#if HAND==0
if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1)){

#elif HAND==1
    if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
         client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1)){
        // ok, the service call was ok, and the subscriber was succesfully started on V-REP side
        // V-REP is now listening to the desired values
        ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
#endif

        // 5. Let's prepare a publisher of those values:
        ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);

        ros::spinOnce(); // handle ROS messages
        pre_time = simulationTime - timeTot; // update the total time of the movement
        BOOST_LOG_SEV(lg, info) << "timeTot = " << timeTot ;
        //BOOST_LOG_SEV(lg, info) << "pre_time = " << pre_time ;

        tb = pre_time;
        //int start = 1;
        //if(moveit_mov){start=0;}
        //for (int i = start; i< vel.rows()-1; ++i){
        //for (int i = 1; i< vel.rows()-1; ++i){
        for (int i = 0; i< vel.rows()-1; ++i){
            //BOOST_LOG_SEV(lg, info) << "Stage = " << k;
            //BOOST_LOG_SEV(lg, info) << "Interval = " << i;
            VectorXd ya = vel.row(i);
            VectorXd yb = vel.row(i+1);
            VectorXd yat = traj.row(i);
            VectorXd ybt = traj.row(i+1);
            //BOOST_LOG_SEV(lg, info) << "timeTot = " << timeTot;
            //BOOST_LOG_SEV(lg, info) << "timesteps_stage = " << timesteps_stage.at(i);
            ta = tb;
            double tt_step = timesteps_stage.at(i);
            if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
            tb = ta + tt_step;
            //tb = ta + timesteps_stage.at(i);

            //BOOST_LOG_SEV(lg, info) << "ta = " << ta;
            //BOOST_LOG_SEV(lg, info) << "tb = " << tb << "\n";
            bool interval = true;
            double tx_prev;
            double yxt_prev;
            //ros::spinOnce(); // get the simulationRunning value
            while (ros::ok() && simulationRunning && interval)
            {// ros is running, simulation is running

                vrep_common::JointSetStateData dataTraj;
#if HAND==1
                vrep_common::JointSetStateData data_hand;
#endif

                tx = simulationTime - timeTot;
                //BOOST_LOG_SEV(lg, info) << "simulationTime = " << simulationTime ;
                if (tx > tb){
                    // go to the next interval
                    interval = false;
                }else{
                    //BOOST_LOG_SEV(lg, info) << "tx = " << tx ;
                    //BOOST_LOG_SEV(lg, info) << "tx_prev = " << tx_prev;

                    double m;
                    if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}

                    std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                    switch (arm_code) {
                    case 0: // dual arm
                        this->curr_scene->getHumanoid()->getRightPosture(r_post);
                        this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                        break;
                    case 1: // right arm
                        this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                        break;
                    case 2: //left arm
                        this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                        break;
                    }
                    double yx; double yxt; double thr = 0.0;
                    if(arm_code!=0){
                        thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                   pow((f_posture(1)-curr_post.at(1)),2)+
                                   pow((f_posture(2)-curr_post.at(2)),2)+
                                   pow((f_posture(3)-curr_post.at(3)),2)+
                                   pow((f_posture(4)-curr_post.at(4)),2)+
                                   pow((f_posture(5)-curr_post.at(5)),2)+
                                   pow((f_posture(6)-curr_post.at(6)),2));
                    }else{
                        thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                   pow((f_posture(1)-r_post.at(1)),2)+
                                   pow((f_posture(2)-r_post.at(2)),2)+
                                   pow((f_posture(3)-r_post.at(3)),2)+
                                   pow((f_posture(4)-r_post.at(4)),2)+
                                   pow((f_posture(5)-r_post.at(5)),2)+
                                   pow((f_posture(6)-r_post.at(6)),2)+
                                   pow((f_posture(11)-l_post.at(0)),2)+
                                   pow((f_posture(12)-l_post.at(1)),2)+
                                   pow((f_posture(13)-l_post.at(2)),2)+
                                   pow((f_posture(14)-l_post.at(3)),2)+
                                   pow((f_posture(15)-l_post.at(4)),2)+
                                   pow((f_posture(16)-l_post.at(5)),2)+
                                   pow((f_posture(17)-l_post.at(6)),2));
                    }
                    if(thr < tol_stop_stage){
                        f_reached=true;
                        //std::cout << "final posture reached" << std::endl;
                        //BOOST_LOG_SEV(lg, info) << "final posture reached" ;
                        break;
                    }else{f_reached=false;}
                    hand_closed = (closed[0] && closed[1] && closed[2]);

                    for (int k = 0; k < vel.cols(); ++k){// for loop joints
                        if(f_reached){
                            yx=0;
                            yxt=yxt_prev;
                        }else{
                            yx = interpolate(ya(k),yb(k),m);
                            yxt = interpolate(yat(k),ybt(k),m);
                            yxt_prev=yxt;
                        }
                        if(arm_code!=0){
                            // single-arm
                            if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                    (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)){ // joints of the hand if the hand is open
                                dataTraj.handles.data.push_back(handles.at(k));
                            }
                        }else{
                            // dual-arm
                            if((k < JOINTS_ARM) || // joints of the right arm OR
                                ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                            {
                                dataTraj.handles.data.push_back(r_handles.at(k));
                            }else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                      ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                            {
                                dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                            }
                        }

                        int exec_mode; double exec_value;
#if VEL==0
                        // position
                        exec_mode = 0; exec_value = yxt;
#elif VEL==1
                        //velocity
                        exec_mode = 2; exec_value = yx;
#endif
                        if(arm_code!=0){
                            // single-arm
                            //ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed) // joints of the hand
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))) // joints of the arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }
                            // Jarde
                            //dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            //dataTraj.values.data.push_back(yxt);
                        }else{
                            //dual-arm
                            if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                      (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }
                        }
                        /*
                        switch(scenarioID){
                        case 0: //error
                            // TO DO
                            break;
                        case 1: case 3: case 4: case 5: case 6: // Toy vehicle scenario with AROS, empty scenario with ARoS, human assistance with ARoS, Challenge scenario with ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))){
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }
                            if((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)){ // joints of the arm
                                dataTraj.values.data.push_back(exec_value);
                            }else if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){ // joint of the hand
                                dataTraj.values.data.push_back(yxt);
                            }
                            break;
                        case 2: // Toy vehicle scenario with Jarde
                            dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            dataTraj.values.data.push_back(yxt);
                            break;
                        }
                        */
#if HAND ==1
                        if(arm_code!=0){
                            //single-arm
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the fingers are being addressed
                                data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }else{
                            // dual-arm
                            if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the right fingers are being addressed
                                data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the left fingers are being addressed
                                data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }
#endif
/*
                      BOOST_LOG_SEV(lg, info) << "joint " << k << " = " << yxt *180/static_cast<double>(M_PI) << ", yat = " << yat(k)*180/static_cast<double>(M_PI) << ", ybt = "<< ybt(k)*180/static_cast<double>(M_PI);
                      if(arm_code!=0){
                        BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  curr_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                  ", error " << k << "=" << (curr_post.at(k)-yxt)*180/static_cast<double>(M_PI);
                      }else{
                          if(k < JOINTS_ARM){
                            BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  r_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                        ", error " << k << "=" << (r_post.at(k)-yxt)*180/static_cast<double>(M_PI);

                          }else if(k >= (JOINTS_ARM + JOINTS_HAND)){
                            BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  l_post.at(k-(JOINTS_ARM+JOINTS_HAND))*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                        ", error " << k << "=" << (l_post.at(k-(JOINTS_ARM+JOINTS_HAND))-yxt)*180/static_cast<double>(M_PI);
                          }
                      }
                      */

                    } // FOR LOOP JOINTS
                    //BOOST_LOG_SEV(lg, info) << "\n";


                    pub.publish(dataTraj);
#if HAND ==1
                    pub_hand.publish(data_hand);
#endif

                    interval = true;
                    tx_prev = tx;

                } // if tx is inside the interval

                // handle ROS messages:
                ros::spinOnce();


            } // while

            if(f_reached){
                log(QNode::Info,string("Final Posture reached."));
                break;}

        }// for loop steps

        // ----- post-movement operations -------- //
        //ros::spinOnce(); // handle ROS messages
        switch (mov_type) {
        case 0: // reach-to grasp
            // grasp the object
            if(approach ||(plan && (traj_mov.size()==1))){
                if(arm_code!=0){
                    //single arm
                    if(obj_in_hand){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
//#if HAND == 1
      //                  this->closeBarrettHand(arm_code);
//#else
      //                  closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
//#endif
                    }
                }else{
                    // dual arm
                    if(obj_in_r_hand){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = r_h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
                    }
                    if(obj_in_l_hand){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        //srvset_parent.request.handle = h_detobj;
                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = l_h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        }
                    }
                }
            }
            break;
        case 1: // reaching
            break;
        case 2: // transport
            break;
        case 3: // engage
            break;
        case 4: // disengage
            break;
        case 5: // go park
            break;
        }
    } // if subscriber
    // handle ROS messages:
    ros::spinOnce();
    timeTot = simulationTime; // update the total time of the movement
} // for loop stages

// pause the simulation
this->pauseSim();

// handle ROS messages:
ros::spinOnce();

TotalTime = simulationTime; // update the total time of the movement

log(QNode::Info,string("Movement completed"));
mov->setExecuted(true);


return true;

}

bool QNode::execTask(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task,taskPtr task, scenarioPtr scene)
{
    bool hand_closed; closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    int arm_code;
    int mov_type;
    double timeTot = 0.0;
    this->curr_scene = scene;
    //int scenarioID = scene->getID();
    std::vector<int> handles;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int h_attach; // handle of the attachment point of the hand
    std::vector<int> r_handles;
    MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int r_h_attach; // handle of the attachment point of the hand
    std::vector<int> l_handles;
    MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int l_h_attach; // handle of the attachment point of the hand
    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan; bool approach; bool retreat;
    double pre_time;


    //BOOST_LOG_SEV(lg, info) << "TotalTime = " << TotalTime;

    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

#if HAND==1

    // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
    ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

#endif


    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task
    for(int kk=0; kk < task->getProblemNumber(); ++kk){ //for loop movements
      if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved()){
          int ii = kk - hh;
          vector<MatrixXd> traj_mov = traj_task.at(ii);
          vector<MatrixXd> vel_mov = vel_task.at(ii);
          vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
          vector<double> tols_stop_mov = tols_stop_task.at(ii);
          double tol_stop_stage;
          MatrixXd traj;
          MatrixXd vel;
          std::vector<double> timesteps_stage;
          movementPtr mov = task->getProblem(kk)->getMovement();
          vector<string> traj_descr_mov = traj_descr_task.at(ii);
          //BOOST_LOG_SEV(lg, info) << "Movement = " << mov->getInfoLine();
          this->curr_mov = mov;
          arm_code = mov->getArm();
          mov_type = mov->getType();

          switch (mov_type){
          case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
              closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
              break;
          case 2: case 3: case 4: // transport, engage, disengage
              closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
              break;
          }
          switch (arm_code) {
          case 0: // dual arm
              r_handles = right_handles;
              r_hand_handles = right_hand_handles;
              r_h_attach = right_attach;
              l_handles = left_handles;
              l_hand_handles = left_hand_handles;
              l_h_attach = left_attach;
              break;
          case 1: //right arm
              handles = right_handles;
              hand_handles = right_hand_handles;
              h_attach = right_attach;
              break;
          case 2: // left arm
              handles = left_handles;
              hand_handles = left_hand_handles;
              h_attach = left_attach;
              break;
          }

          for(size_t j=0; j < traj_mov.size();++j){ //for loop stages
              string mov_descr = traj_descr_mov.at(j);
              if(strcmp(mov_descr.c_str(),"plan")==0){
                  plan=true; approach=false; retreat=false;
              }else if(strcmp(mov_descr.c_str(),"approach")==0){
                  plan=false; approach=true; retreat=false;
              }else if(strcmp(mov_descr.c_str(),"retreat")==0){
                  plan=false; approach=false; retreat=true;
              }

              switch (mov_type){
              case 0: // reach-to-grasp
                  if(retreat){
                      if(arm_code!=0){
                          //single-arm
                          if(obj_in_hand){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(arm_code);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
#endif
                          }
                      }else{
                          // dual-arm
                          if(obj_in_r_hand){ // right arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = r_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(1);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(1,hand_init_pos);
#endif
                          }

                          if(obj_in_l_hand){ // left arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = l_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(2);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(2,hand_init_pos);
#endif
                          }
                      }
                  }
                  break;
              case 1: // reaching
                  break;
              case 2: case 3: // transport, engage
                  if(retreat){
                      if(arm_code!=0){
                          // single-arm
                          //ros::spinOnce();// handle ROS messages
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //if(obj_in_hand){
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = -1; // parentless object
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in releasing the object "));
                                  }
                          }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                      //this->openBarrettHand(arm_code);
                      MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                      std::vector<double> hand_init_pos;
                      hand_init_pos.resize(init_h_posture.size());
                      VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                      this->openBarrettHand_to_pos(arm_code,hand_init_pos);
#else
                      closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                      }else{
                          //dual-arm
                          // right arm
                          //ros::spinOnce();// handle ROS messages
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //if(obj_in_hand){
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = -1; // parentless object
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in releasing the object "));
                                  }
                          }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                        //this->openBarrettHand(1);
                        MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(1,hand_init_pos);
#else
                      closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                      // left arm
                      //ros::spinOnce();// handle ROS messages
                      if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          //if(obj_in_hand){
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = -1; // parentless object
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in releasing the object "));
                              }
                      }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                    //this->openBarrettHand(2);
                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                    this->openBarrettHand_to_pos(2,hand_init_pos);
#else
                  closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                      }
                  }
                  break;
              case 4:// disengage
                  break;
              case 5: // go-park
                  break;
              }
              traj = traj_mov.at(j);
              vel = vel_mov.at(j);
              timesteps_stage = timesteps_mov.at(j);
              f_posture = traj.row(traj.rows()-1);
              f_reached=false;
              tol_stop_stage = tols_stop_mov.at(j);

              ros::spinOnce(); // handle ROS messages
              pre_time = simulationTime - timeTot; // update the total time of the movement

#if HAND==0
        if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1)){

#elif HAND==1
            if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                 client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1)){
                // ok, the service call was ok, and the subscriber was succesfully started on V-REP side
                // V-REP is now listening to the desired values
                ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
#endif

                    // 5. Let's prepare a publisher of those values:
                    ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                    tb = pre_time;
                    for (int i = 0; i< vel.rows()-1; ++i){
                    //for (int i = 1; i< vel.rows()-1; ++i){

                        //BOOST_LOG_SEV(lg, info) << "Interval = " << i;

                        VectorXd ya = vel.row(i);
                        VectorXd yb = vel.row(i+1);
                        VectorXd yat = traj.row(i);
                        VectorXd ybt = traj.row(i+1);

                        ta = tb;
                        double tt_step = timesteps_stage.at(i);
                        if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
                        tb = ta + tt_step;
                        //tb = ta + timesteps_stage.at(i);

                        //BOOST_LOG_SEV(lg, info) << "ta = " << ta;
                        //BOOST_LOG_SEV(lg, info) << "tb = " << tb << "\n";

                        bool interval = true;
                        double tx_prev;
                        double yxt_prev;
                        ros::spinOnce(); // get the simulationRunning value
                        while (ros::ok() && simulationRunning && interval)
                        {// ros is running, simulation is running
                            vrep_common::JointSetStateData dataTraj;
#if HAND==1
                            vrep_common::JointSetStateData data_hand;
#endif

                            tx = simulationTime - timeTot;

                            //BOOST_LOG_SEV(lg, info) << "simulationTime = " << simulationTime ;

                            if (tx >= tb){
                                // go to the next interval
                                interval = false;
                            }else{
                                //BOOST_LOG_SEV(lg, info) << "tx = " << tx ;
                                //BOOST_LOG_SEV(lg, info) << "tx_prev = " << tx_prev;
                                double m;
                                if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}
                                //std::vector<double> r_post;
                                //this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                                switch (arm_code) {
                                case 0: // dual arm
                                    this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                    this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                                    break;
                                case 1: // right arm
                                    this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                                    break;
                                case 2: //left arm
                                    this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                                    break;
                                }
                                double yx; double yxt; double thr = 0.0;
                                if(arm_code!=0){
                                    thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                               pow((f_posture(1)-curr_post.at(1)),2)+
                                               pow((f_posture(2)-curr_post.at(2)),2)+
                                               pow((f_posture(3)-curr_post.at(3)),2)+
                                               pow((f_posture(4)-curr_post.at(4)),2)+
                                               pow((f_posture(5)-curr_post.at(5)),2)+
                                               pow((f_posture(6)-curr_post.at(6)),2));
                                }else{
                                    thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                               pow((f_posture(1)-r_post.at(1)),2)+
                                               pow((f_posture(2)-r_post.at(2)),2)+
                                               pow((f_posture(3)-r_post.at(3)),2)+
                                               pow((f_posture(4)-r_post.at(4)),2)+
                                               pow((f_posture(5)-r_post.at(5)),2)+
                                               pow((f_posture(6)-r_post.at(6)),2)+
                                               pow((f_posture(11)-l_post.at(0)),2)+
                                               pow((f_posture(12)-l_post.at(1)),2)+
                                               pow((f_posture(13)-l_post.at(2)),2)+
                                               pow((f_posture(14)-l_post.at(3)),2)+
                                               pow((f_posture(15)-l_post.at(4)),2)+
                                               pow((f_posture(16)-l_post.at(5)),2)+
                                               pow((f_posture(17)-l_post.at(6)),2));
                                }
                                if(thr < tol_stop_stage){
                                    f_reached=true;
                                    log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                    //std::cout << "final posture reached, movement: " << mov->getStrType() << std::endl;
                                    break;
                                }else{f_reached=false;}
                                hand_closed = (closed[0] && closed[1] && closed[2]);
                                for (int k = 0; k< vel.cols(); ++k){ // for loop joints
                                    if(f_reached){
                                        yx=0;
                                        yxt=yxt_prev;
                                    }else{
                                        yx = interpolate(ya(k),yb(k),m);
                                        yxt = interpolate(yat(k),ybt(k),m);
                                        yxt_prev=yxt;
                                    }
                                    if(arm_code!=0){
                                        // single-arm
                                        if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                                (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)){
                                            dataTraj.handles.data.push_back(handles.at(k));
                                        }
                                    }else{
                                        // dual-arm
                                        if((k < JOINTS_ARM) || // joints of the right arm OR
                                            ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(r_handles.at(k));
                                        }else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                                  ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                                        }
                                    }
                                    int exec_mode; double exec_value;
#if VEL==0
                        // position
                        exec_mode = 0; exec_value = yxt;
#elif VEL==1
                        //velocity
                        exec_mode = 2; exec_value = yx;
#endif
                        if(arm_code!=0){
                            // single-arm
                            //ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed) // joints of the hand
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))) // joints of the arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }
                            // Jarde
                            //dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            //dataTraj.values.data.push_back(yxt);
                        }else{
                            //dual-arm
                            if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                      (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }
                        }

                        /*
                        switch(scenarioID){
                        case 0: //error
                            break;
                        case 1: case 3: case 4: case 5: case 6: // Toy vehicle scenario with AROS, empty scenario with ARoS, human assistance with ARoS, Challenging scenario with ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))){
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }
                            if((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)){ // joints of the arm
                                dataTraj.values.data.push_back(exec_value);
                            }else if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){ // joints of the hand
                                dataTraj.values.data.push_back(yxt);
                            }
                            break;
                        case 2: // Toy vehicle scenario with Jarde
                            dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            dataTraj.values.data.push_back(yxt);
                            break;
                        }
                        */
                        //BOOST_LOG_SEV(lg, info) << "joint " << k << " = " << yx *180/static_cast<double>(M_PI) << ", ya = " << ya(k)*180/static_cast<double>(M_PI) << ", yb = "<< yb(k)*180/static_cast<double>(M_PI);
                        //BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  r_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                  // ", error " << k << "=" << (r_post.at(k)-yxt)*180/static_cast<double>(M_PI);
#if HAND==1

                        if(arm_code!=0){
                            //single-arm
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the fingers are being addressed
                                data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }else{
                            // dual-arm
                            if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the right fingers are being addressed
                                data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the left fingers are being addressed
                                data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }
#endif
                     /*
                      BOOST_LOG_SEV(lg, info) << "joint " << k << " = " << yxt *180/static_cast<double>(M_PI) << ", yat = " << yat(k)*180/static_cast<double>(M_PI) << ", ybt = "<< ybt(k)*180/static_cast<double>(M_PI);
                      if(arm_code!=0){
                        BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  curr_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                  ", error " << k << "=" << (curr_post.at(k)-yxt)*180/static_cast<double>(M_PI);
                      }else{
                          if(k < JOINTS_ARM){
                            BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  r_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                        ", error " << k << "=" << (r_post.at(k)-yxt)*180/static_cast<double>(M_PI);

                          }else if(k >= (JOINTS_ARM + JOINTS_HAND)){
                            BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  l_post.at(k-(JOINTS_ARM+JOINTS_HAND))*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                        ", error " << k << "=" << (l_post.at(k-(JOINTS_ARM+JOINTS_HAND))-yxt)*180/static_cast<double>(M_PI);
                          }
                      }
                      */

                    } // FOR LOOP JOINTS
                    //BOOST_LOG_SEV(lg, info) << "\n";

                    pub.publish(dataTraj);

#if HAND==1
                    pub_hand.publish(data_hand);
#endif

                        interval = true;
                        tx_prev = tx;
                    }

                    // handle ROS messages:
                    ros::spinOnce();

                } // while

                if(f_reached){
                    log(QNode::Info,string("Final Posture reached."));
                    break;}

            } // for loop step

                    // ---- post-movement operations ---- //
                    // set the detected object child of the attach point
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    switch (mov_type) {
                    case 0: // reach-to grasp
                        // grasp the object
                        if(approach ||(plan && (traj_mov.size()==1))){
                            if(arm_code!=0){
                                // single-arm
                                if(obj_in_hand){
                                    //srvset_parent.request.handle = h_detobj;
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
//#if HAND == 1
  //                      this->closeBarrettHand(arm_code);
//#else
  //                      closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
//#endif
                                }
                            }else{
                                // dual-arm
                                if(obj_in_r_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    //srvset_parent.request.handle = h_detobj;
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = r_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                                if(obj_in_l_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    //srvset_parent.request.handle = h_detobj;
                                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                    srvset_parent.request.parentHandle = l_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                            }
                        }
                        break;
                    case 1: // reaching
                        break;
                    case 2: // transport
                        break;
                    case 3: // engage
                        break;
                    case 4: // disengage
                        break;
                    case 5: // go-park
                        break;
                    }

                    // handle ROS messages:
                    ros::spinOnce();
                    timeTot = simulationTime; // update the total time of the movement

                } // if

          }//for loop stages
          // movement complete
          log(QNode::Info,string("Movement completed"));
          task->getProblem(kk)->getMovement()->setExecuted(true);
        }else{
            hh++;
        }// if prob is part of the task

      }// for loop movements
      log(QNode::Info,string("Task completed"));

      // pause the simulation
      this->pauseSim();

      // handle ROS messages:
      ros::spinOnce();

      TotalTime=simulationTime;

      return true;
}

bool QNode::execTask_complete(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task, taskPtr task, scenarioPtr scene)
{
    bool hand_closed; closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    int arm_code;
    int mov_type;
    double timeTot = 0.0;
    this->curr_scene = scene;
    //int scenarioID = scene->getID();
    std::vector<int> handles;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int h_attach; // handle of the attachment point of the hand
    std::vector<int> r_handles;
    MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int r_h_attach; // handle of the attachment point of the hand
    std::vector<int> l_handles;
    MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int l_h_attach; // handle of the attachment point of the hand
    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan; bool approach; bool retreat;
    double pre_time;


    //BOOST_LOG_SEV(lg, info) << "TotalTime = " << TotalTime;



    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

#if HAND==1

    // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
    ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

#endif

    // previous retreat trajectories
    MatrixXd traj_prev_retreat;
    MatrixXd vel_prev_retreat;
    std::vector<double> ttsteps_prev_retreat;

    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task
    for(int kk=0; kk < task->getProblemNumber(); ++kk){ //for loop movements
      if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved()){
          int ii = kk - hh;
          vector<MatrixXd> traj_mov = traj_task.at(ii);
          vector<MatrixXd> vel_mov = vel_task.at(ii);
          vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
          vector<double> tols_stop_mov = tols_stop_task.at(ii);
          double tol_stop_stage;
          MatrixXd traj;
          MatrixXd vel;
          std::vector<double> timesteps_stage;
          movementPtr mov = task->getProblem(kk)->getMovement();
          vector<string> traj_descr_mov = traj_descr_task.at(ii);
          //BOOST_LOG_SEV(lg, info) << "Movement = " << mov->getInfoLine();
          this->curr_mov = mov;
          arm_code = mov->getArm();
          mov_type = mov->getType();


          switch (mov_type){
          case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
              closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
              break;
          case 2: case 3: case 4: // transport, engage, disengage
              closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
              break;
          }


          switch (arm_code) {
          case 0: // dual arm
              r_handles = right_handles;
              r_hand_handles = right_hand_handles;
              r_h_attach = right_attach;
              l_handles = left_handles;
              l_hand_handles = left_hand_handles;
              l_h_attach = left_attach;
              break;
          case 1: //right arm
              handles = right_handles;
              hand_handles = right_hand_handles;
              h_attach = right_attach;
              break;
          case 2: // left arm
              handles = left_handles;
              hand_handles = left_hand_handles;
              h_attach = left_attach;
              break;
          }


          MatrixXd traj_ret_plan_app;
          MatrixXd vel_ret_plan_app;
          std::vector<double> timesteps_ret_plan_app;
          bool join_ret_plan_app = false; bool join_ret_plan = false; bool join_plan_app = false;

          if(mov_type==0 || mov_type==2 || mov_type==3 || mov_type==4){
              // reach-to-grasp, transport, engage, disengage
              if(traj_mov.size() > 1){
                  // there is more than one stage
                  string mov_descr_1 = traj_descr_mov.at(0);
                  string mov_descr_2 = traj_descr_mov.at(1);
                  if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"approach")==0)){

                      if(ii==0 || traj_prev_retreat.rows()==0){
                          // first movement of the task => there is no previous retreat
                          join_plan_app = true;
                          traj_ret_plan_app.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                          vel_ret_plan_app.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                      }else{
                          join_ret_plan_app = true;
                          traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1 + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                          vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1 + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                      }
                  }else if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"retreat")==0)){
                      if(ii!=0 && traj_prev_retreat.rows()!=0){
                          join_ret_plan = true;
                          traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                          vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                      }
                  }
              }else{
                  // there is only the plan stage
                  if(ii!=0 && traj_prev_retreat.rows()!=0){
                      join_ret_plan = true;
                      traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                      vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                  }
              }
          }else{
              // reaching, go-park
              if(ii!=0 && traj_prev_retreat.rows()!=0){
                  join_ret_plan = true;
                  traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                  vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
              }
          }

          for(size_t j=0; j <traj_mov.size();++j){ //for loop stages

              string mov_descr = traj_descr_mov.at(j);
              if(strcmp(mov_descr.c_str(),"plan")==0){
                  plan=true; approach=false; retreat=false;
                  if(join_ret_plan_app){
                      traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                      vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                      timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                      std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                      MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                      MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                      std::vector<double> ttsteps = timesteps_mov.at(j);
                      traj_ret_plan_app.block(traj_prev_retreat.rows(),0,tt_red.rows(),tt_red.cols()) = tt_red;
                      vel_ret_plan_app.block(vel_prev_retreat.rows(),0,vv_red.rows(),vv_red.cols()) = vv_red;
                      //if(moveit_task){
                      timesteps_ret_plan_app.reserve(ttsteps.size());
                      //}else{
                        //timesteps_ret_plan_app.reserve(ttsteps.size()-1);
                      //}
                      std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                      traj_prev_retreat.resize(0,0);
                      continue;
                  }else if(join_ret_plan){
                      traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                      vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                      timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                      std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                      MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                      MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                      std::vector<double> ttsteps = timesteps_mov.at(j);
                      traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                      vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                      //if(moveit_task){
                      timesteps_ret_plan_app.reserve(ttsteps.size());
                      //}else{
                        //timesteps_ret_plan_app.reserve(ttsteps.size()-1);
                      //}
                      std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                      traj_prev_retreat.resize(0,0);
                  }else if(join_plan_app){
                      MatrixXd tt = traj_mov.at(j);
                      MatrixXd vv = vel_mov.at(j);
                      std::vector<double> ttsteps = timesteps_mov.at(j);
                      traj_ret_plan_app.topLeftCorner(tt.rows(),tt.cols()) = tt;
                      vel_ret_plan_app.topLeftCorner(vv.rows(),vv.cols()) = vv;
                      timesteps_ret_plan_app.reserve(ttsteps.size());
                      std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                      continue;
                  }
              }else if(strcmp(mov_descr.c_str(),"approach")==0){
                  plan=false; approach=true; retreat=false;
                  if(join_ret_plan_app || join_plan_app){
                      MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                      MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                      std::vector<double> ttsteps = timesteps_mov.at(j);
                      traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                      vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                      //if(moveit_task){
                      timesteps_ret_plan_app.reserve(ttsteps.size());
                      std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                      //}else{
                       // timesteps_ret_plan_app.reserve(ttsteps.size()-1);
                       // std::copy (ttsteps.begin()+1, ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                      //}
                  }else if(join_ret_plan){
                      // ERROR
                  }
              }else if(strcmp(mov_descr.c_str(),"retreat")==0){
                  plan=false; approach=false; retreat=true;
                  traj_prev_retreat = traj_mov.at(j);
                  vel_prev_retreat = vel_mov.at(j);
                  ttsteps_prev_retreat = timesteps_mov.at(j);

              }

              switch (mov_type){
              case 0: // reach-to-grasp
                  if(retreat){
                      if(arm_code!=0){
                          // single-arm
                          if(obj_in_hand){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(arm_code);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
#endif
                          }
                          continue;
                      }else{
                          // dual-arm
                          if(obj_in_r_hand){ // right arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = r_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(1);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(1,hand_init_pos);
#endif
                          }
                          if(obj_in_l_hand){ // left arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = l_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
#if HAND == 1 && OPEN_CLOSE_HAND ==1
                          this->closeBarrettHand(2);
#else
                          MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                          std::vector<double> hand_init_pos;
                          hand_init_pos.resize(init_h_posture.size());
                          VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                          this->closeBarrettHand_to_pos(2,hand_init_pos);
#endif
                          }
                          continue;
                      }
                  }
                  break;
              case 1: // reaching
                  break;
              case 2: case 3: // transport, engage
                  if(retreat){
                      if(arm_code!=0){
                      // single-arm
                          //ros::spinOnce();// handle ROS messages
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //if(obj_in_hand){
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = -1; // parentless object
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in releasing the object "));
                                  }
                          }
#if HAND ==1 && OPEN_CLOSE_HAND ==1
                      //this->openBarrettHand(arm_code);
                      MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                      std::vector<double> hand_init_pos;
                      hand_init_pos.resize(init_h_posture.size());
                      VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                      this->openBarrettHand_to_pos(arm_code,hand_init_pos);
#else
                      closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
#endif
                          continue;
                      }else{
                          //dual-arm
                          // right arm
                          //ros::spinOnce();// handle ROS messages
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //if(obj_in_hand){
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = -1; // parentless object
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in releasing the object "));
                                  }
                          }
      #if HAND ==1 && OPEN_CLOSE_HAND ==1
                        //this->openBarrettHand(1);
                        MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(1,hand_init_pos);
      #else
                      closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
      #endif
                      // left arm
                      //ros::spinOnce();// handle ROS messages
                      if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          //if(obj_in_hand){
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = -1; // parentless object
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in releasing the object "));
                              }
                      }
      #if HAND ==1 && OPEN_CLOSE_HAND ==1
                    //this->openBarrettHand(2);
                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                    this->openBarrettHand_to_pos(2,hand_init_pos);
      #else
                  closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
      #endif
                      }
                      continue;
                  }
                  break;
              case 4:// disengage
                  break;
              case 5: // go-park
                  break;
              }

              if((join_ret_plan && (strcmp(mov_descr.c_str(),"plan")==0)) || ((join_ret_plan_app || join_plan_app) && (strcmp(mov_descr.c_str(),"approach")==0))){
                  traj = traj_ret_plan_app;
                  vel = vel_ret_plan_app;
                  timesteps_stage = timesteps_ret_plan_app;
              }else{
                  traj = traj_mov.at(j);
                  vel = vel_mov.at(j);
                  timesteps_stage = timesteps_mov.at(j);
              }

              f_posture = traj.row(traj.rows()-1);
              f_reached=false;              
              tol_stop_stage = tols_stop_mov.at(j);

              ros::spinOnce(); // handle ROS messages
              pre_time = simulationTime - timeTot; // update the total time of the movement

#if HAND==0
        if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1)){

#elif HAND==1
            if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                 client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1)){
                // ok, the service call was ok, and the subscriber was succesfully started on V-REP side
                // V-REP is now listening to the desired values
                ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
#endif

                    // 5. Let's prepare a publisher of those values:
                    ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                    tb = pre_time;
                    //int start = 1;
                    //if(moveit_task){start=0;}
                    //for (int i = start; i< vel.rows()-1; ++i){
                    //for (int i = 1; i< vel.rows()-1; ++i){
                    for (int i = 0; i< vel.rows()-1; ++i){

                        //BOOST_LOG_SEV(lg, info) << "Interval = " << i;

                        VectorXd ya = vel.row(i);
                        VectorXd yb = vel.row(i+1);
                        VectorXd yat = traj.row(i);
                        VectorXd ybt = traj.row(i+1);

                        ta = tb;
                        double tt_step = timesteps_stage.at(i);
                        if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
                        tb = ta + tt_step;
                        //tb = ta + timesteps_stage.at(i);

                        //BOOST_LOG_SEV(lg, info) << "ta = " << ta;
                        //BOOST_LOG_SEV(lg, info) << "tb = " << tb << "\n";

                        bool interval = true;
                        double tx_prev;
                        double yxt_prev;
                        ros::spinOnce(); // get the simulationRunning value
                        while (ros::ok() && simulationRunning && interval)
                        {// ros is running, simulation is running
                            vrep_common::JointSetStateData dataTraj;
#if HAND==1
                            vrep_common::JointSetStateData data_hand;
#endif

                            tx = simulationTime - timeTot;

                            //BOOST_LOG_SEV(lg, info) << "simulationTime = " << simulationTime ;

                            if (tx >= tb){
                                // go to the next interval
                                interval = false;
                            }else{

                                //BOOST_LOG_SEV(lg, info) << "tx = " << tx ;
                                //BOOST_LOG_SEV(lg, info) << "tx_prev = " << tx_prev;

                                double m;
                                if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}

                                std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                                switch (arm_code) {
                                case 0: // dual arm
                                    this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                    this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                                    break;
                                case 1: // right arm
                                    this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                                    break;
                                case 2: //left arm
                                    this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                                    break;
                                }
                                double yx; double yxt; double thr = 0.0;
                                if(arm_code!=0){
                                    thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                               pow((f_posture(1)-curr_post.at(1)),2)+
                                               pow((f_posture(2)-curr_post.at(2)),2)+
                                               pow((f_posture(3)-curr_post.at(3)),2)+
                                               pow((f_posture(4)-curr_post.at(4)),2)+
                                               pow((f_posture(5)-curr_post.at(5)),2)+
                                               pow((f_posture(6)-curr_post.at(6)),2));
                                }else{
                                    thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                               pow((f_posture(1)-r_post.at(1)),2)+
                                               pow((f_posture(2)-r_post.at(2)),2)+
                                               pow((f_posture(3)-r_post.at(3)),2)+
                                               pow((f_posture(4)-r_post.at(4)),2)+
                                               pow((f_posture(5)-r_post.at(5)),2)+
                                               pow((f_posture(6)-r_post.at(6)),2)+
                                               pow((f_posture(11)-l_post.at(0)),2)+
                                               pow((f_posture(12)-l_post.at(1)),2)+
                                               pow((f_posture(13)-l_post.at(2)),2)+
                                               pow((f_posture(14)-l_post.at(3)),2)+
                                               pow((f_posture(15)-l_post.at(4)),2)+
                                               pow((f_posture(16)-l_post.at(5)),2)+
                                               pow((f_posture(17)-l_post.at(6)),2));
                                }
                                if(thr < tol_stop_stage){
                                    f_reached=true;
                                    log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                    //std::cout << "final posture reached, movement: " << mov->getStrType() << std::endl;
                                    break;
                                }else{f_reached=false;}
                                hand_closed = (closed[0] && closed[1] && closed[2]);

                                for (int k = 0; k< vel.cols(); ++k){
                                    if(f_reached){
                                        yx=0;
                                        yxt=yxt_prev;
                                    }else{
                                        yx = interpolate(ya(k),yb(k),m);
                                        yxt = interpolate(yat(k),ybt(k),m);
                                        yxt_prev=yxt;
                                    }
                                    if(arm_code!=0){
                                        // single-arm
                                        if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                                (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)){ // joints of the hand if the hand is open
                                            dataTraj.handles.data.push_back(handles.at(k));
                                        }
                                    }else{
                                        // dual-arm
                                        if((k < JOINTS_ARM) || // joints of the right arm OR
                                            ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(r_handles.at(k));
                                        }else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                                  ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                                        }
                                    }
                                    int exec_mode; double exec_value;
#if VEL==0
                        // position
                        exec_mode = 0; exec_value = yxt;
#elif VEL==1
                        //velocity
                        exec_mode = 2; exec_value = yx;
#endif
                        if(arm_code!=0){
                            // single-arm
                            //ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed) // joints of the hand
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))) // joints of the arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }
                            // Jarde
                            //dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            //dataTraj.values.data.push_back(yxt);
                        }else{
                            //dual-arm
                            if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                      (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }
                        }

                        /*
                        switch(scenarioID){
                        case 0: //error
                            break;
                        case 1: case 3: case 4: case 5: case 6: // Toy vehicle scenario with AROS, empty scenario with ARoS, human assistance with ARoS, Challenging scenario with ARoS
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))){
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            }
                            if((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)){ // joints of the arm
                                dataTraj.values.data.push_back(exec_value);
                            }else if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed){ // joints of the hand
                                dataTraj.values.data.push_back(yxt);
                            }
                            break;
                        case 2: // Toy vehicle scenario with Jarde
                            dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                            dataTraj.values.data.push_back(yxt);
                            break;
                        }
                        */

#if HAND==1

                        if(arm_code!=0){
                            //single-arm
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the fingers are being addressed
                                data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }else{
                            // dual-arm
                            if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the right fingers are being addressed
                                data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the left fingers are being addressed
                                data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }
#endif
                        /*
                          BOOST_LOG_SEV(lg, info) << "joint " << k << " = " << yxt *180/static_cast<double>(M_PI) << ", yat = " << yat(k)*180/static_cast<double>(M_PI) << ", ybt = "<< ybt(k)*180/static_cast<double>(M_PI);
                          if(arm_code!=0){
                            BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  curr_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                      ", error " << k << "=" << (curr_post.at(k)-yxt)*180/static_cast<double>(M_PI);
                          }else{
                              if(k < JOINTS_ARM){
                                BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  r_post.at(k)*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                            ", error " << k << "=" << (r_post.at(k)-yxt)*180/static_cast<double>(M_PI);

                              }else if(k >= (JOINTS_ARM + JOINTS_HAND)){
                                BOOST_LOG_SEV(lg, info) << "real joint " << k << " = " <<  l_post.at(k-(JOINTS_ARM+JOINTS_HAND))*180/static_cast<double>(M_PI) << ",  traj joint " << k << " = " << yxt *180/static_cast<double>(M_PI) <<
                                                            ", error " << k << "=" << (l_post.at(k-(JOINTS_ARM+JOINTS_HAND))-yxt)*180/static_cast<double>(M_PI);
                              }
                          }
                          */

                        } // FOR LOOP JOINTS
                        //BOOST_LOG_SEV(lg, info) << "\n";

                        pub.publish(dataTraj);

#if HAND==1
                        pub_hand.publish(data_hand);
#endif

                        interval = true;
                        tx_prev = tx;
                    }

                    // handle ROS messages:
                    ros::spinOnce();

                } // while

                if(f_reached){
                    log(QNode::Info,string("Final Posture reached."));
                    break;}

            } // for loop step

            // ---- post-movement operations ---- //
            // set the detected object child of the attach point
            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
            switch (mov_type) {
            case 0: // reach-to grasp
                // grasp the object
                if(approach ||(plan && (traj_mov.size()==1))){
                    if(arm_code!=0){
                    //single-arm
                        if(obj_in_hand){
                            //srvset_parent.request.handle = h_detobj;
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in grasping the object "));
                            }
//#if HAND == 1
//                      this->closeBarrettHand(arm_code);
//#else
//                      closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
//#endif
                        }
                    }else{
                        //dual-arm
                        if(obj_in_r_hand){
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            //srvset_parent.request.handle = h_detobj;
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = r_h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in grasping the object "));
                            }
                        }
                        if(obj_in_l_hand){
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            //srvset_parent.request.handle = h_detobj;
                            srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                            srvset_parent.request.parentHandle = l_h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in grasping the object "));
                            }
                        }
                    }
                }
                break;
            case 1: // reaching
                break;
            case 2: // transport
                break;
            case 3: // engage
                break;
            case 4: // disengage
                break;
            case 5: // go-park
                break;
            }

            // handle ROS messages:
            ros::spinOnce();
            timeTot = simulationTime; // update the total time of the movement

        } // if

      }//for loop stages
      // movement complete
      log(QNode::Info,string("Movement completed"));
      task->getProblem(kk)->getMovement()->setExecuted(true);

    }else{
        hh++;
    }// if prob is part of the task

  }// for loop movements
  log(QNode::Info,string("Task completed"));

  // pause the simulation
  this->pauseSim();

  // handle ROS messages:
  ros::spinOnce();

  TotalTime=simulationTime;

  return true;


}

bool QNode::execKinControl(int arm, vector<double> &r_posture, vector<double> &r_velocities)
{

    std::vector<int> handles;
    switch (arm) {
    case 0: // dual arm
        // TO DO
        break;
    case 1: //right arm
        handles = right_handles;
        break;
    case 2: // left arm
        handles = left_handles;
        break;
    }


    if(!simulationRunning)
    {
        ros::NodeHandle node;
        // set joints position or velocity (it depends on the settings)
        ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
        vrep_common::simRosEnableSubscriber srv_enableSubscriber;
        srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
        srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
        srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
        client_enableSubscriber.call(srv_enableSubscriber);

        // start the simulation
        add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
        vrep_common::simRosStartSimulation srvstart;
        add_client.call(srvstart);

        ros::spinOnce(); // first handle ROS messages
    }

    if(ros::ok() && simulationRunning)
    {// ros is running, simulation is running


        // handle ROS messages:
        ros::spinOnce();

        vrep_common::JointSetStateData data;
        int exec_mode = 0;
        //int exec_mode = 2;
        double exec_value;
        //double time_step = simulationTime - timetot;
        for (int i = 0; i < r_velocities.size(); ++i)
        {
            exec_value = r_posture.at(i) + (r_velocities.at(i)) * simulationTimeStep;
            //exec_value = r_velocities.at(i);

            if(arm!=0){
                // single-arm
                data.handles.data.push_back(handles.at(i));
                data.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop joints
        pub_joints.publish(data);
    }
}

bool QNode::execKinControl(int arm, vector<double> &r_arm_posture, vector<double> &r_arm_velocities, vector<double> &r_hand_posture, vector<double> &r_hand_velocities,bool joints_arm_vel_ctrl)
{

    std::vector<int> handles;
    //MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    switch (arm) {
    case 0: // dual arm
        // TO DO
        break;
    case 1: //right arm
        handles = right_handles;
        //hand_handles = right_hand_handles;
        break;
    case 2: // left arm
        handles = left_handles;
        //hand_handles = left_hand_handles;
        break;
    }


    if(ros::ok() && simulationRunning)
    {// ros is running, simulation is running


        // handle ROS messages:
        ros::spinOnce();

        vrep_common::JointSetStateData data;
        int exec_arm_mode; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        if(joints_arm_vel_ctrl){
            exec_arm_mode=2;
        }else{
            exec_arm_mode=0;
        }
        int exec_hand_mode = 1; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        double exec_value;

        for (size_t i = 0; i < r_arm_velocities.size(); ++i)
        {
            if(joints_arm_vel_ctrl){
                exec_value =  r_arm_velocities.at(i); // vel
            }else{
               exec_value = r_arm_posture.at(i) +  r_arm_velocities.at(i)*simulationTimeStep; // pos
            }

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_arm_mode);
                data.handles.data.push_back(handles.at(i));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop arm joints

        for (size_t i = 0; i < r_hand_velocities.size(); ++i)
        {
            exec_value = r_hand_posture.at(i) + (r_hand_velocities.at(i)) * simulationTimeStep;

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_hand_mode);
                data.handles.data.push_back(handles.at(i+r_arm_velocities.size()));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop hand joints
        pub_joints.publish(data);
    }
}

bool QNode::execKinControlAcc(int arm, vector<double> &r_arm_posture, vector<double> &r_arm_velocities, vector<double> &r_arm_accelerations, vector<double> &r_hand_posture, vector<double> &r_hand_velocities)
{


    std::vector<int> handles;
    //MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    switch (arm) {
    case 0: // dual arm
        // TO DO
        break;
    case 1: //right arm
        handles = right_handles;
        //hand_handles = right_hand_handles;
        break;
    case 2: // left arm
        handles = left_handles;
        //hand_handles = left_hand_handles;
        break;
    }


//    if(!simulationRunning)
//    {
//        ros::NodeHandle node;
//        // set joints position or velocity (it depends on the settings)
//        ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
//        vrep_common::simRosEnableSubscriber srv_enableSubscriber;
//        srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
//        srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
//        srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
//        client_enableSubscriber.call(srv_enableSubscriber);

//        // start the simulation
//        this->startSim();

//        ros::spinOnce(); // first handle ROS messages
//    }

    if(ros::ok() && simulationRunning)
    {// ros is running, simulation is running


        // handle ROS messages:
        ros::spinOnce();

        vrep_common::JointSetStateData data;
        int exec_arm_mode = 2; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        int exec_hand_mode = 1; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        double exec_value;

        for (size_t i = 0; i < r_arm_accelerations.size(); ++i)
        {
            //exec_value = r_arm_posture.at(i) +  0.5 * r_arm_accelerations.at(i))* pow(simulationTimeStep,2); // pos
            exec_value =  r_arm_accelerations.at(i) * simulationTimeStep; // vel

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_arm_mode);
                data.handles.data.push_back(handles.at(i));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop arm joints

        for (size_t i = 0; i < r_hand_velocities.size(); ++i)
        {
            exec_value = r_hand_posture.at(i) + (r_hand_velocities.at(i)) * simulationTimeStep;

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_hand_mode);
                data.handles.data.push_back(handles.at(i+r_arm_accelerations.size()));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop hand joints
        pub_joints.publish(data);
    }
}


double QNode::interpolate(double ya, double yb, double m)
{

    // linear interpolation
    return ya+(yb-ya)*m;

}

void QNode::startSim()
{
    ros::NodeHandle node;

    // start the simulation
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);
    this->simulationRunning=true;
    this->simulationPaused=false;

}

void QNode::stopSim()
{

    ros::NodeHandle node;

    // stop the simulation
    add_client = node.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);
    this->simulationTime=0.0;
    this->simulationRunning=false;
    this->simulationPaused=false;
}

void QNode::pauseSim()
{
    ros::NodeHandle node;

    // pause the simulation
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);
    this->simulationPaused=true;
    this->simulationRunning=true;
    this->simulationTimePaused = this->simulationTime;
}

double QNode::getSimTime()
{
    return this->simulationTime;
}

double QNode::getSimTimePaused()
{
    return this->simulationTimePaused;
}

double QNode::getSimTimeStep()
{
    return this->simulationTimeStep;
}

string QNode::getNodeName()
{
    return this->nodeName;
}

bool QNode::isSimulationRunning()
{
    return this->simulationRunning;
}

bool QNode::isSimulationPaused()
{
    return this->simulationPaused;
}

void QNode::enableSetJoints()
{
    ros::NodeHandle node;
    // set joints position or velocity (it depends on the settings)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+this->nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
    client_enableSubscriber.call(srv_enableSubscriber);
}


bool QNode::checkRViz()
{

    FILE *fp;
    const int length=1000;
    char result[length]; // line to read
    std::string s2("unknown node");
    bool online=false;

    fp = popen("rosnode ping -c 1 /move_group", "r");

    int cnt=0;
    while (fgets(result, length, fp) != NULL){
     //   printf("%s", result)
        if (cnt==1){
            // second line
            std::string s1(result);

            if (s1.find(s2) != std::string::npos){
                // V-REP is off-line
                online=false;
            }else{
                // V-REP is on-line
                online=true;
            }

        }
        cnt++;

    }
    return online;

}

bool QNode::checkVrep()
{

    FILE *fp;
    const int length=1000;
    char result[length]; // line to read
    std::string s2("unknown node");
    bool online=false;

    fp = popen("rosnode ping -c 1 /vrep", "r");

    int cnt=0;
    while (fgets(result, length, fp) != NULL){
     //   printf("%s", result)
        if (cnt==1){
            // second line
            std::string s1(result);

            if (s1.find(s2) != std::string::npos){
                // V-REP is off-line
                online=false;
            }else{
                // V-REP is on-line
                online=true;
            }

        }
        cnt++;

    }
    return online;

}




void QNode::run()
{
    //ros::Rate loop_rate(0.5);
    //ros::NodeHandle node;

    while ( ros::ok() ) {} // infinite loop while ros is running

    ros::spinOnce(); // handles ROS messages
    //std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::JointsCallback(const sensor_msgs::JointState &state)
{

std::vector<std::string> joints_names = state.name;
std::vector<double> joints_pos(state.position.begin(),state.position.end());
std::vector<double> joints_vel(state.velocity.begin(),state.velocity.end());
std::vector<double> joints_force(state.effort.begin(),state.effort.end());

std::vector<double> right_posture;
std::vector<double> right_vel;
std::vector<double> right_forces;
std::vector<double> left_posture;
std::vector<double> left_vel;
std::vector<double> left_forces;


//BOOST_LOG_SEV(lg, info) << "joints_callback";

#if HAND == 0
const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                         "right_joint_thumb_TMC_aa","right_joint_fing1_MCP","right_joint_fing3_MCP","right_joint_thumb_TMC_fe"};
const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                         "left_joint_thumb_TMC_aa","left_joint_fing1_MCP","left_joint_fing3_MCP","left_joint_thumb_TMC_fe"};
const char *r_2hand[] = {"right_joint_fing1_PIP","right_joint_fing3_PIP","right_joint_thumb_MCP"};
const char *l_2hand[] = {"left_joint_fing1_PIP","left_joint_fing3_PIP","left_joint_thumb_MCP"};

#elif HAND == 1
const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                         "right_BarrettHand_jointA_0","right_BarrettHand_jointB_0","right_BarrettHand_jointB_2","right_BarrettHand_jointB_1"};
const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                         "left_BarrettHand_jointA_0","left_BarrettHand_jointB_0","left_BarrettHand_jointB_2","left_BarrettHand_jointB_1"};
const char *r_2hand[]={"right_BarrettHand_jointC_0","right_BarrettHand_jointC_2","right_BarrettHand_jointC_1"};
const char *l_2hand[]={"left_BarrettHand_jointC_0","left_BarrettHand_jointC_2","left_BarrettHand_jointC_1"};
#endif

for (int i = 0; i < JOINTS_ARM+JOINTS_HAND; ++i){

    size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();
    size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_names[i]) - joints_names.begin();
    if (r_index >= joints_names.size() || l_index >= joints_names.size()){
        std::cout << "element not found in state.name\n";
    }else{

        /*
        std::cout << "real joint 0: " << joints_pos.at(0)*180/static_cast<double>(M_PI) << "real joint 1: " << joints_pos.at(1)*180/static_cast<double>(M_PI)  << "real joint 2: " << joints_pos.at(2)*180/static_cast<double>(M_PI) << "real joint 3: " << joints_pos.at(3)*180/static_cast<double>(M_PI)
                  << "real joint 4: " << joints_pos.at(4)*180/static_cast<double>(M_PI) << "real joint 5: " << joints_pos.at(5)*180/static_cast<double>(M_PI) << "real joint 6: " << joints_pos.at(6)*180/static_cast<double>(M_PI) << '\n';
        std::cout << "traj joint 0: " << r_traj.at(0)*180/static_cast<double>(M_PI) << "traj joint 1: " << r_traj.at(1)*180/static_cast<double>(M_PI)  << "traj joint 2: " << r_traj.at(2)*180/static_cast<double>(M_PI) << "traj joint 3: " << r_traj.at(3)*180/static_cast<double>(M_PI)
                  << "traj joint 4: " << r_traj.at(4)*180/static_cast<double>(M_PI) << "traj joint 5: " << r_traj.at(5)*180/static_cast<double>(M_PI) << "traj joint 6: " << r_traj.at(6)*180/static_cast<double>(M_PI) << '\n';
                  */

        right_posture.push_back(joints_pos.at(r_index));
        right_vel.push_back(joints_vel.at(r_index));
        right_forces.push_back(joints_force.at(r_index));

        left_posture.push_back(joints_pos.at(l_index));
        left_vel.push_back(joints_vel.at(l_index));
        left_forces.push_back(joints_force.at(l_index));
    }

}

for(int i = 0; i < HAND_FINGERS; ++i){

    size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_2hand[i]) - joints_names.begin();
    size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_2hand[i]) - joints_names.begin();
    if (r_index >= joints_names.size() || l_index >= joints_names.size()){
        std::cout << "element not found in state.name\n";
    }else{

        right_2hand_pos.at(i)=joints_pos.at(r_index);
        right_2hand_vel.at(i)=joints_vel.at(r_index);
        right_2hand_force.at(i)=joints_force.at(r_index);


        left_2hand_pos.at(i)=joints_pos.at(l_index);
        left_2hand_vel.at(i)=joints_vel.at(l_index);
        left_2hand_force.at(i)=joints_force.at(l_index);

    }


}


/*

BOOST_LOG_SEV(lg, info) << "right forces: "
                        << right_forces.at(0)<< " "
                        << right_forces.at(1)<< " "
                        << right_forces.at(2)<< " "
                        << right_forces.at(3)<< " "
                        << right_forces.at(4)<< " "
                        << right_forces.at(5)<< " "
                        << right_forces.at(6)<< " "
                        << right_forces.at(7)<< " "
                        << right_forces.at(8)<< " "
                        << right_forces.at(9)<< " "
                        << right_forces.at(10)<< " ";

                        */

if (this->curr_scene){

    this->curr_scene->getHumanoid()->setRightPosture(right_posture);
    this->curr_scene->getHumanoid()->setLeftPosture(left_posture);
    this->curr_scene->getHumanoid()->setRightVelocities(right_vel);
    this->curr_scene->getHumanoid()->setLeftVelocities(left_vel);
    this->curr_scene->getHumanoid()->setRightForces(right_forces);
    this->curr_scene->getHumanoid()->setLeftForces(left_forces);

}



}


void QNode::log( const LogLevel &level, const std::string &msg)
{
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    //ros::Time::init();// bug fixed
	switch ( level ) {
		case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                //logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[DEBUG] [" << currentDateTime() << "]: " << msg;
				break;
		}
		case(Info) : {
                ROS_INFO_STREAM(msg);
                //logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
				break;
		}
		case(Warn) : {
                ROS_WARN_STREAM(msg);
                //logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
				break;
		}
		case(Error) : {
                ROS_ERROR_STREAM(msg);
                //logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[ERROR] [" << currentDateTime() << "]: " << msg;
				break;
		}
		case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                //logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[FATAL] [" << currentDateTime() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated();
}

void QNode::checkProximityObject(movementPtr mov,string stage)
{
    this->curr_mov = mov;
    ros::NodeHandle node;
    int h_attach;
    int arm = this->curr_mov->getArm();
    int mov_type = this->curr_mov->getType();
    switch (arm) {
    case 0: // dual arm (TO DO)
        break;
    case 1: //right arm
        //handles = right_handles;
        //hand_handles = right_hand_handles;
        h_attach = right_attach;
        break;
    case 2: // left arm
        //handles = left_handles;
        //hand_handles = left_hand_handles;
        h_attach = left_attach;
        break;
    }

    switch (mov_type){
    case 0: // reach-to-grasp
        if(stage.compare("retreat")==0){
            if(obj_in_hand){
                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                //srvset_parent.request.handle = h_detobj;
                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                srvset_parent.request.parentHandle = h_attach;
                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                add_client.call(srvset_parent);
                if (srvset_parent.response.result != 1){
                    log(QNode::Error,string("Error in grasping the object "));
                }
//#if HAND == 1 && OPEN_CLOSE_HAND ==1
//                this->closeBarrettHand(arm_code);
//#else
//                MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
//                std::vector<double> hand_init_pos;
//                hand_init_pos.resize(init_h_posture.size());
//                VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
//                this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
//#endif
            }
        break;
        }
    }
}

const std::string QNode::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


void QNode::init()
{

    logging::add_file_log
    (
        keywords::file_name = "QNode_%N.log",                                        /*< file name pattern >*/
        keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
        keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0,0,0), /*< ...or at midnight >*/
        keywords::format = "[%TimeStamp%]: %Message%",                                 /*< log record format >*/
        keywords::target = "Boost_logs"
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
}



bool QNode::getArmsHandles(int humanoid)
{

    bool succ = true;

     ros::NodeHandle node;
     ros::ServiceClient add_client;

    // get the joint arm + hand handles
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    vrep_common::simRosGetObjectHandle srvgetHandle;

    for (int k = 0; k < JOINTS_ARM + JOINTS_HAND; ++k){

        if (k < 7){
            srvgetHandle.request.objectName = string("right_joint")+QString::number(k).toStdString();
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                right_handles.push_back(srvgetHandle.response.handle);
            }else{succ=false;}

            srvgetHandle.request.objectName = string("left_joint")+QString::number(k).toStdString();
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                left_handles.push_back(srvgetHandle.response.handle);
            }else{succ=false;}

        }else if (k == 7){

            switch(humanoid){

            case 0: // ARoS

                srvgetHandle.request.objectName = string("right_BarrettHand_jointA_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointA_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            case 1: // Jarde

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            }


        }else if(k == 8){

            switch(humanoid){

            case 0: // ARoS

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            case 1: // Jarde

                srvgetHandle.request.objectName = string("right_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            }


        }else if(k == 9){


            switch(humanoid){

            case 0: // ARoS

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            case 1: // Jarde

                srvgetHandle.request.objectName = string("right_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            }


        }else if(k == 10){

            switch(humanoid){

            case 0: // ARoS

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            case 1: // Jarde

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_handles.push_back(srvgetHandle.response.handle);
                }else{succ=false;}

                break;

            }


        }

    }

    // get the complete hand handles
    switch(humanoid){

    case 0: // ARoS

        for (int k = 0; k < HAND_FINGERS; ++k){

            if (k!=1){

                srvgetHandle.request.objectName = string("right_BarrettHand_jointA_")+QString::number(k).toStdString();
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointA_")+QString::number(k).toStdString();
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

            }

            if (k == 0){

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_BarrettHand_jointC_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

            }

            if (k == 1){

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_BarrettHand_jointC_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

            }

            if (k == 2){

                srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_BarrettHand_jointC_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}


            }
        }
            break;

    case 1: // Jarde

        for (int k = 0; k < HAND_FINGERS; ++k){

            if (k==2){// Thumb

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_IP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,3)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_IP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,3)=(srvgetHandle.response.handle);
                }else{succ=false;}




            }else if(k==0){ // Index

                srvgetHandle.request.objectName = string("right_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing1_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing1_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

            }else if(k==1){ // Ring

                srvgetHandle.request.objectName = string("right_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing3_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing3_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}
            }
        }

        break;
    }


switch(humanoid){

case 0: // ARos

    // get the object handle of the sensors
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_BarrettHand_attachProxSensor");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        right_sensor=srvgetHandle.response.handle;
    }else{succ=false;}

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("left_BarrettHand_attachProxSensor");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        left_sensor=srvgetHandle.response.handle;
    }else{succ=false;}


    // get the object handle of the attach points
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_BarrettHand_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        right_attach=srvgetHandle.response.handle;
    }else{succ=false;}

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("left_BarrettHand_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        left_attach=srvgetHandle.response.handle;
    }else{succ=false;}

    break;


case 1: // Jarde

    // get the object handle of the sensors
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_hand_attachProxSensor");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        right_sensor=srvgetHandle.response.handle;
    }else{succ=false;}

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("left_hand_attachProxSensor");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        left_sensor=srvgetHandle.response.handle;
    }else{succ=false;}


    // get the object handle of the attach points
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_hand_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        right_attach=srvgetHandle.response.handle;
    }else{succ=false;}

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("left_hand_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1){
        left_attach=srvgetHandle.response.handle;
    }else{succ=false;}

    break;


    break;

}




    return succ;
}

#if HAND == 1

bool QNode::closeBarrettHand(int hand)
{


    int cnt = 0;
    std::vector<int> firstPartTorqueOvershootCount(3, 0);
    firstPartLocked.at(0)=false;
    firstPartLocked.at(1)=false;
    firstPartLocked.at(2)=false;

    needFullOpening.at(0)=0;
    needFullOpening.at(1)=0;
    needFullOpening.at(2)=0;

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;


    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    //set the force
    ros::ServiceClient client_setForce = node.serviceClient<vrep_common::simRosSetJointForce>("/vrep/simRosSetJointForce");
    vrep_common::simRosSetJointForce srv_setForce;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


while (ros::ok() && simulationRunning && (!closed[0] || !closed[1] || !closed[2]) && cnt<1000){

    cnt++;

    switch (hand) {

    case 1: // right hand

        hand_handles = right_hand_handles;
        this->curr_scene->getHumanoid()->getRightHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getRightHandPosture(hand_posture);

        break;

    case 2: // left hand

        hand_handles = left_hand_handles;
        this->curr_scene->getHumanoid()->getLeftHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getLeftHandPosture(hand_posture);

        break;
    }

/*

    BOOST_LOG_SEV(lg, info) << "cnt: " << cnt << ", "<< "Hand forces: "
                            << hand_forces.at(0)<< " "
                            << hand_forces.at(1)<< " "
                            << hand_forces.at(2)<< " "
                            << hand_forces.at(3)<< " "   ;

                            */




        for (size_t i = 0; i < HAND_FINGERS; i++)
        {

            if (firstPartLocked.at(i)){

                closed[i] = true;


                // set the velocity of the second phalanx (1/3 of the velocity of the first phalanx)
                srv_setObjInt.request.handle = hand_handles(i,2);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                srv_setTarVel.request.handle = hand_handles(i,2);
                srv_setTarVel.request.targetVelocity = closingVel/3.0f;
                client_setTarVel.call(srv_setTarVel);
                //srv_setTarVel.response.results;
                //simSetJointTargetVelocity(handHandles[i][2], closingVel / 3.0f);
                //if joints locked, do nothing.



            }else if (!firstPartLocked.at(i)){

                double t = 0.0f;

                //int res = simJointGetForce(handfirstpartTorqueHandles[i], &t);
                t = hand_forces.at(i+1);

                if (abs(t) > firstPartMaxTorque)
                    firstPartTorqueOvershootCount[i] ++;
                else
                    firstPartTorqueOvershootCount[i] = 0;

                if (firstPartTorqueOvershootCount[i] >= firstPartTorqueOvershootCountRequired)
                {
                    needFullOpening[i] = 1;
                    firstPartLocked[i] = true;
                    //First part is now locked and holding the position

                    // lock the first part
                    // set the position control
                    srv_setObjInt.request.handle = hand_handles(i,1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 1;
                    client_setIntParam.call(srv_setObjInt);
                    // set the force
                    srv_setForce.request.handle = hand_handles(i,1);
                    srv_setForce.request.forceOrTorque = closingOpeningTorque*100.0f;
                    client_setForce.call(srv_setForce);
                    // set the target position
                    srv_setTarPos.request.handle = hand_handles(i,1);
                    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
                    client_setTarPos.call(srv_setTarPos);


                    // go on with the second part
                    srv_setObjInt.request.handle = hand_handles(i,2);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i,2);
                    srv_setTarVel.request.targetVelocity = closingVel / 3.0f;
                    client_setTarVel.call(srv_setTarVel);

                }
                else
                {
                    //make first joint to close with a predefined velocity
                    srv_setObjInt.request.handle = hand_handles(i,1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i,1);
                    srv_setTarVel.request.targetVelocity = closingVel;
                    client_setTarVel.call(srv_setTarVel);

                    //second joint position is 1/3 of the first  
                    srv_setTarPos.request.handle = hand_handles(i,2);
                    srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1) / 3.0f;
                    client_setTarPos.call(srv_setTarPos);

                }
            }
        }

        // handle ROS messages:
        ros::spinOnce();

    } // while loop

for (size_t i = 0; i < HAND_FINGERS; i++){
    // set the position control
    srv_setObjInt.request.handle = hand_handles(i,1);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
    // set the target position
    srv_setTarPos.request.handle = hand_handles(i,1);
    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
    client_setTarPos.call(srv_setTarPos);

    srv_setObjInt.request.handle = hand_handles(i,2);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
}

    log(QNode::Info,string("Hand closed."));
    return (closed[0] && closed[1] && closed[2]);


}

bool QNode::openBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand2_pos;

    switch (hand) {
    case 1: // right hand
        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        break;
    case 2: // left hand
        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        break;
    }

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;

    for (size_t i = 0; i < HAND_FINGERS; ++i){
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,2);
        srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = false;
        needFullOpening[i] = 0;
        closed[i]=false;

    }
    log(QNode::Info,string("Hand open."));
    return true;
}

bool QNode::closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand2_pos;

    switch (hand) {
    case 1: // right hand
        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        break;
    case 2: // left hand
        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        break;
    }

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;

    for (size_t i = 0; i < HAND_FINGERS; ++i){
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,2);
        srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = true;
        needFullOpening[i] = 1;
        closed[i]=true;

    }
    log(QNode::Info,string("Hand closed."));
    return true;
}

bool QNode::openBarrettHand(int hand)
{

    int cnt = 0;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;
    std::vector<double> hand2_pos;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    //set the force
    ros::ServiceClient client_setForce = node.serviceClient<vrep_common::simRosSetJointForce>("/vrep/simRosSetJointForce");
    vrep_common::simRosSetJointForce srv_setForce;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


while (ros::ok() && simulationRunning && (closed[0] || closed[1] || closed[2]) && cnt<1000){

    cnt++;

    switch (hand) {

    case 1: // right hand

        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        this->curr_scene->getHumanoid()->getRightHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getRightHandPosture(hand_posture);

        break;

    case 2: // left hand

        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        this->curr_scene->getHumanoid()->getLeftHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getLeftHandPosture(hand_posture);

        break;
    }



    //BOOST_LOG_SEV(lg, info) << "cnt: " << cnt << ", "<< "Hand posture: "
      //                      << hand_posture.at(0)<< " "
       //                     << hand_posture.at(1)<< " "
        //                    << hand_posture.at(2)<< " "
        //                    << hand_posture.at(3)<< " "   ;



    for (size_t i = 0; i < HAND_FINGERS; i++){

        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 0;
        client_setIntParam.call(srv_setObjInt);

        srv_setTarVel.request.handle = hand_handles(i,2);
        srv_setTarVel.request.targetVelocity = openingVel/3.0f;
        client_setTarVel.call(srv_setTarVel);

        if (firstPartLocked[i]){

            if(hand2_pos.at(i) < 45.5*static_cast<double>(M_PI) / 180.0){
                // unlock the first part

                // set the velocity control
                srv_setObjInt.request.handle = hand_handles(i,1);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                //make first joints to open with a predefined velocity
                srv_setTarVel.request.handle = hand_handles(i,1);
                srv_setTarVel.request.targetVelocity = openingVel;
                client_setTarVel.call(srv_setTarVel);



                firstPartLocked[i] = false;

                //BOOST_LOG_SEV(lg, info) << " First part unlocked: " << i;
            }

        }else{

            if (needFullOpening[i] != 0){

                // full opening is needed

                if (hand2_pos.at(i) < 45.5f*static_cast<double>(M_PI) / 180.0f && hand_posture.at(i+1) < 0.5f*static_cast<double>(M_PI) / 180.0f){

                    needFullOpening[i] = 0;
                    // second joint in position control
                    // set the position control
                    srv_setObjInt.request.handle = hand_handles(i,2);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 1;
                    client_setIntParam.call(srv_setObjInt);

                    // set the target position
                    srv_setTarPos.request.handle = hand_handles(i,2);
                    srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
                    client_setTarPos.call(srv_setTarPos);

                    //BOOST_LOG_SEV(lg, info) << " Full opening needed: " << i;

                }


            }else{

                // full opening is NOT needed

                //make first joint to open with a predefined velocity
                srv_setObjInt.request.handle = hand_handles(i,1);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                srv_setTarVel.request.handle = hand_handles(i,1);
                srv_setTarVel.request.targetVelocity = openingVel;
                client_setTarVel.call(srv_setTarVel);



                //BOOST_LOG_SEV(lg, info) << "hand posture " << i << hand_posture.at(i+1) <<" Full opening NOT needed ";

                if ( hand_posture.at(i+1) <= 36.0f*static_cast<double>(M_PI) / 180.0f){

                    closed[i]=false;

                    //BOOST_LOG_SEV(lg, info) << " closed false: " << i;
                }



            }



        }



    }// for loop


    // handle ROS messages:
    ros::spinOnce();

}// while loop




for (size_t i = 0; i < HAND_FINGERS; i++){
    // set the position control
    srv_setObjInt.request.handle = hand_handles(i,1);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
    // set the target position
    srv_setTarPos.request.handle = hand_handles(i,1);
    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
    client_setTarPos.call(srv_setTarPos);

    srv_setObjInt.request.handle = hand_handles(i,2);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);

}

log(QNode::Info,string("Hand open."));
return true;

}

#endif

}  // namespace motion_manager
