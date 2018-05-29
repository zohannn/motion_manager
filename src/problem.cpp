#include "../include/motion_manager/problem.hpp"

namespace motion_manager {

Problem::Problem():
    mov(nullptr),scene(nullptr)
{

    this->rightFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<double>(JOINTS_HAND);
    this->leftFinalHand = std::vector<double>(JOINTS_HAND);
    //this->rightBouncePosture = std::vector<double>(JOINTS_ARM);
    //this->leftBouncePosture = std::vector<double>(JOINTS_ARM);

    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;


}

Problem::Problem(int planner_id,Movement* mov,Scenario* scene)
{
    this->rightFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<double>(JOINTS_HAND);
    this->leftFinalHand = std::vector<double>(JOINTS_HAND);

    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;

    this->mov = movementPtr(mov); int arm_sel = this->mov->getArm();
    this->scene = scenarioPtr(scene);
    this->planner_id=planner_id;

    bool hump = false;

    if (planner_id==0){hump = true;this->planner_name = "HUMP";}

    string scene_name = this->scene->getName();
    //int scene_id = this->scene->getID();

    if (hump){
        // --- Human-like movement planner settings --- //
        HUMotion::HUMPlanner::hand_fingers = HAND_FINGERS;
        HUMotion::HUMPlanner::joints_arm = JOINTS_ARM;
        HUMotion::HUMPlanner::joints_hand = JOINTS_HAND;
        HUMotion::HUMPlanner::n_phalange = N_PHALANGE;
#if MOVEIT==1
        this->m_planner = nullptr;
#endif
        this->h_planner.reset(new HUMotion::HUMPlanner(scene_name));
        // set the current obstacles and targets of the scenario
        vector<objectPtr> scene_objects;
        if(this->scene->getObjects(scene_objects)){
            HUMotion::objectPtr hump_obj; // object of the planner
            objectPtr obj;
            for(size_t i=0; i < scene_objects.size(); ++i){
                obj = scene_objects.at(i);
                std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
                std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
                std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
                hump_obj.reset(new HUMotion::Object(obj->getName()));
                hump_obj->setParams(position,orientation,dimension);
                if(arm_sel!=0)
                {
                    // single-arm movement
                    if(!obj->isTargetRightEnabled() && !obj->isTargetLeftEnabled()){
                        this->h_planner->addObstacle(hump_obj); // the object is an obstacle for the planner
                    }
                }else{
                    // dual-arm movement
                    if(!obj->isTargetRightEnabled()){
                        this->h_planner->addObstacleRight(hump_obj); // the object is an obstacle for the planner
                    }
                    if(!obj->isTargetLeftEnabled()){
                        this->h_planner->addObstacleLeft(hump_obj); // the object is an obstacle for the planner
                    }
                }
            }
        }else{
            // the scene is empty of objects
        }

        // set the humanoid
        Matrix4d mat_right_arm; Matrix4d mat_right_hand; vector<double> min_rlimits; vector<double> max_rlimits;
        Matrix4d mat_left_arm; Matrix4d mat_left_hand; vector<double> min_llimits; vector<double> max_llimits;
        this->scene->getHumanoid()->getMatRight(mat_right_arm);
        this->scene->getHumanoid()->getMatRightHand(mat_right_hand);
        this->scene->getHumanoid()->getMatLeft(mat_left_arm);
        this->scene->getHumanoid()->getMatLeftHand(mat_left_hand);
        this->scene->getHumanoid()->getRightMinLimits(min_rlimits);
        this->scene->getHumanoid()->getRightMaxLimits(max_rlimits);
        this->scene->getHumanoid()->getLeftMinLimits(min_llimits);
        this->scene->getHumanoid()->getLeftMaxLimits(max_llimits);
        h_planner->setMatRightArm(mat_right_arm);
        h_planner->setMatRightHand(mat_right_hand);
        h_planner->setRightMaxLimits(max_rlimits);
        h_planner->setRightMinLimits(min_rlimits);
        h_planner->setMatLeftArm(mat_left_arm);
        h_planner->setMatLeftHand(mat_left_hand);
        h_planner->setLeftMaxLimits(max_llimits);
        h_planner->setLeftMinLimits(min_llimits);

        dim torso_dim = this->scene->getHumanoid()->getSize();
        std::vector<double> tsize = {torso_dim.Xsize,torso_dim.Ysize,torso_dim.Zsize};
        h_planner->setTorsoSize(tsize);

        DHparams rDH = this->scene->getHumanoid()->getDH_rightArm();
        DHparams lDH = this->scene->getHumanoid()->getDH_leftArm();
        HUMotion::DHparameters right_arm_DH;
        HUMotion::DHparameters left_arm_DH;
        right_arm_DH.a = rDH.a; right_arm_DH.alpha = rDH.alpha; right_arm_DH.d = rDH.d; right_arm_DH.theta = rDH.theta;
        left_arm_DH.a = lDH.a; left_arm_DH.alpha = lDH.alpha; left_arm_DH.d = lDH.d; left_arm_DH.theta = lDH.theta;
        h_planner->setDH_rightArm(right_arm_DH);
        h_planner->setDH_leftArm(left_arm_DH);


#if HAND==0
        human_hand hhand = this->scene->getHumanoid()->getHumanHand();
        HUMotion::HumanHand hump_hhand;
        hump_hhand.maxAperture = hhand.maxAperture;
        hump_hhand.thumb.uTx = hhand.thumb.uTx;
        hump_hhand.thumb.uTy = hhand.thumb.uTy;
        hump_hhand.thumb.uTz = hhand.thumb.uTz;
        hump_hhand.thumb.thumb_specs.a = hhand.thumb.thumb_specs.a;
        hump_hhand.thumb.thumb_specs.alpha = hhand.thumb.thumb_specs.alpha;
        hump_hhand.thumb.thumb_specs.d = hhand.thumb.thumb_specs.d;
        hump_hhand.thumb.thumb_specs.theta = hhand.thumb.thumb_specs.theta;
        vector<HUMotion::HumanFinger> hump_fings = hump_hhand.fingers;
        vector<human_finger> fings = hhand.fingers;
        for(size_t i=0; i<fings.size();++i){
            human_finger fing = fings.at(i);
            HUMotion::HumanFinger hump_fing = hump_fings.at(i);
            hump_fing.ux = fing.ux; hump_fing.uy = fing.uy; hump_fing.uz = fing.uz;
            hump_fing.finger_specs.a = fing.finger_specs.a;
            hump_fing.finger_specs.alpha = fing.finger_specs.alpha;
            hump_fing.finger_specs.d = fing.finger_specs.d;
            hump_fing.finger_specs.theta = fing.finger_specs.theta;
            hump_fings.at(i) = hump_fing;
        }
        hump_hhand.fingers = hump_fings;
#elif HAND==1
        barrett_hand b_hand = this->scene->getHumanoid()->getBarrettHand();
        std::vector<int> rk; this->scene->getHumanoid()->getRK(rk);
        std::vector<int> jk; this->scene->getHumanoid()->getRK(jk);
        HUMotion::BarrettHand hump_bhand;
        hump_bhand.A1 = b_hand.A1;
        hump_bhand.A2 = b_hand.A2;
        hump_bhand.A3 = b_hand.A3;
        hump_bhand.Aw = b_hand.Aw;
        hump_bhand.D3 = b_hand.D3;
        hump_bhand.maxAperture = b_hand.maxAperture;
        hump_bhand.phi2 = b_hand.phi2;
        hump_bhand.phi3 = b_hand.phi3;
        hump_bhand.rk = rk;
        hump_bhand.jk = jk;
        h_planner->setBarrettHand(hump_bhand);
#endif

    }

}

#if MOVEIT==1
Problem::Problem(int planner_id, Movement *mov, Scenario *scene, moveit_plannerPtr m_plannerPtr)
{
    this->rightFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<double>(JOINTS_HAND);
    this->leftFinalHand = std::vector<double>(JOINTS_HAND);

    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;

    this->mov = movementPtr(mov);
    this->scene = scenarioPtr(scene);
    this->planner_id=planner_id;

    switch(planner_id){
    case 0:
        this->planner_name = "HUMP";
        break;
    case 1:
        this->planner_name = "RRT";
        break;
    case 2:
        this->planner_name = "RRTConnect";
        break;
    case 3:
        this->planner_name = "RRTstar";
        break;
    case 4:
        this->planner_name = "PRM";
        break;
    case 5:
        this->planner_name = "PRMstar";
        break;
    }

    this->m_planner=m_plannerPtr;
    this->h_planner=nullptr;


}
#endif

Problem::Problem(const Problem& s)
{

    this->rightFinalPosture = s.rightFinalPosture;
    this->rightFinalPosture_diseng = s.rightFinalPosture_diseng;
    this->rightFinalPosture_eng = s.rightFinalPosture_eng;
    this->rightFinalHand = s.rightFinalHand;
    this->leftFinalPosture = s.leftFinalPosture;
    this->leftFinalPosture_eng = s.leftFinalPosture_eng;
    this->leftFinalPosture_diseng = s.leftFinalPosture_diseng;
    this->leftFinalHand = s.leftFinalHand;

    this->dFF = s.dFF;
    this->dFH = s.dFH;
    this->dHOl = s.dHOl;
    this->dHOr = s.dHOr;
    this->solved=s.solved;
    this->part_of_task=s.part_of_task;
    this->err_log=s.err_log;

    this->targetAxis = s.targetAxis;
    this->mov = movementPtr(new Movement(*s.mov.get()));
    this->scene = scenarioPtr(new Scenario(*s.scene.get()));
#if MOVEIT==1
    if(s.m_planner!=nullptr){
        this->m_planner = moveit_plannerPtr(new moveit_planning::HumanoidPlanner(*s.m_planner.get()));
    }
#endif
    if(s.h_planner!=nullptr){
        this->h_planner = h_plannerPtr(new HUMotion::HUMPlanner(*s.h_planner.get()));
    }

    this->planner_id=s.planner_id;
    this->planner_name=s.planner_name;
}


Problem::~Problem()
{

}

void Problem::setPlannerID(int id)
{
    this->planner_id=id;

    switch(id){

    case 0:
        this->planner_name = "HUMP";
        break;
    case 1:
        this->planner_name = "RRT";
        break;
    case 2:
        this->planner_name = "RRTConnect";
        break;
    case 3:
        this->planner_name = "RRTstar";
        break;
    case 4:
        this->planner_name = "PRM";
        break;
    case 5:
        this->planner_name = "PRMstar";
        break;

    }

}

int Problem::getPlannerID()
{
    return this->planner_id;
}

string Problem::getPlannerName()
{
    return this->planner_name;
}

int Problem::getErrLog()
{
    return this->err_log;
}


bool Problem::getSolved()
{

    return this->solved;
}

bool Problem::getPartOfTask()
{

    return this->part_of_task;
}

string Problem::getInfoLine()
{

    return string("Planner: ")+this->getPlannerName()+string(" ,Movement: ")+this->mov->getInfoLine();
}

objectPtr Problem::getObjectEngaged()
{

    return this->obj_eng;
}

void Problem::setSolved(bool s)
{

    this->solved=s;
}

void Problem::setPartOfTask(bool p)
{

    this->part_of_task=p;
}

void Problem::setMoveSettings(std::vector<double> &tar, std::vector<double> &final_hand, std::vector<double> &final_arm, bool use_posture)
{
    this->move_target=tar;
    this->move_final_hand=final_hand;
    this->move_final_arm=final_arm;
    this->use_posture=use_posture;
}

void Problem::setMoveSettings(std::vector<double> &tar_right, std::vector<double> &tar_left,
                     std::vector<double> &final_hand_right, std::vector<double> &final_hand_left,
                     std::vector<double> &final_arm_right, std::vector<double> &final_arm_left,
                     bool use_posture_right,bool use_posture_left)
{
    this->move_final_hand_right=final_hand_right;
    this->move_final_hand_left=final_hand_left;
    this->move_final_arm_right=final_arm_right;
    this->move_final_arm_left=final_arm_left;
    this->move_target_right=tar_right;
    this->move_target_left=tar_left;
    this->use_posture_right=use_posture_right;
    this->use_posture_left=use_posture_left;

}

bool Problem::finalPostureFingers(int hand_id)
{

    bool success=false;
    humanoidPtr hh = this->scene->getHumanoid();


    // get the object(s) involved in this movement
    objectPtr obj;
    // get the type of grip for this movement
    bool prec;
    switch(hand_id)
    {
        case 0:// error
            break;
        case 1: // right hand
            obj = this->mov->getObject();
            prec = this->mov->getGrip();
            break;
        case 2: //left hand
            obj = this->mov->getObjectLeft();
            prec = this->mov->getGripLeft();
            break;
    }
    // get the type of grip for this movement
    //int grip_code = this->mov->getGrip();
    //bool prec = this->mov->getGrip();

    // compute the diameter (plus tolerance) of the object to be grasped
    double d_obj;

    if(prec){
        d_obj = obj->getRadius()*2.0+TOL_GRIP;
#if HAND==0
        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1

        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif
    }else{
#if HAND==0

        d_obj = min(hh->getHumanHand().maxAperture,double(1.2)*obj->getRadius()*2+TOL_GRIP);

        if(obj->getRadius()*2+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");

        }

#elif HAND==1
        d_obj = min(hh->getBarrettHand().maxAperture,double(1.2)*obj->getRadius()*2+TOL_GRIP);

        if (obj->getRadius()*2+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif
    }
/*
    switch (grip_code){

    case 111: case 112:
        // Precision Side thumb left and Precision Side thumb right

        d_obj = obj->getRadius()*2.0+TOL_GRIP;
#if HAND==0
        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1

        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif


        break;

    case 113: case 114:
        // Precision Side thumb up and Precision Side thumb down
        d_obj=obj->getSize().Zsize+TOL_GRIP;
#if HAND==0

        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1
        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif



        break;

    case 121: case 122:
        // Precision Above and Precision Below
        d_obj = obj->getRadius()*2+TOL_GRIP;
#if HAND==0

        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");

        }
#elif HAND==1

        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif

        break;

    case 211: case 212:
        // Full Side thumb left and Full Side thumb right

#if HAND==0

        d_obj = min(hh->getHumanHand().maxAperture,double(1.2)*obj->getRadius()*2+TOL_GRIP);

        if(obj->getRadius()*2+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");

        }

#elif HAND==1
        d_obj = min(hh->getBarrettHand().maxAperture,double(1.2)*obj->getRadius()*2+TOL_GRIP);

        if (obj->getRadius()*2+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif
        break;

    case 213: case 214:
        // Full Side thumb up and Full Side thumb down

#if HAND==0
        d_obj = min(hh->getHumanHand().maxAperture,double(1.2)*(obj->getSize().Zsize+TOL_GRIP));

        if(obj->getSize().Zsize+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1
        d_obj = min(hh->getBarrettHand().maxAperture,double(1.2)*(obj->getSize().Zsize+TOL_GRIP));

        if (obj->getSize().Zsize+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif




        break;

    case 221: case 222:
        // Full Above and Full Below

#if HAND==0

        d_obj = min(hh->getHumanHand().maxAperture,double(1.2)*(obj->getRadius()*2+TOL_GRIP));

        if(obj->getRadius()*2+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1

        d_obj = min(hh->getBarrettHand().maxAperture,double(1.2)*(obj->getRadius()*2+TOL_GRIP));

        if (obj->getRadius()*2+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif
        break;

    }// switch grip code
    */

    // compute the inverse kinematics of the hand
    std::vector<double> sols;
    bool inv_succ;
    double theta;
    double thetaT;

    try{

        inv_succ = this->invKinHand(d_obj,sols);
        if (inv_succ){
            theta = sols.at(0);
            thetaT = sols.at(1);
        }else{
            throw string("Error: hand inverse kinematic not solved");
        }
    }
    catch(const string str){

        success=false;
        throw str;

    }
    success=true;

    //switch(this->mov->getArm()){
    switch(hand_id){

    case 0: // both arms

        //TO DO;
        break;

    case 1: // right arm

        this->rightFinalHand.at(0) = THETA8_FINAL;
        this->rightFinalPosture.at(JOINTS_ARM) = THETA8_FINAL;
        this->rightFinalPosture_diseng.at(JOINTS_ARM) = THETA8_FINAL;
        this->rightFinalPosture_eng.at(JOINTS_ARM) = THETA8_FINAL;
#if HAND == 0

        this->rightFinalHand.at(1) = theta;
        this->rightFinalHand.at(2) = theta;
        this->rightFinalHand.at(3) = thetaT;

        this->rightFinalPosture.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture.at(JOINTS_ARM+3)=thetaT;

        this->rightFinalPosture_diseng.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture_diseng.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture_diseng.at(JOINTS_ARM+3)=thetaT;

        this->rightFinalPosture_eng.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture_eng.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture_eng.at(JOINTS_ARM+3)=thetaT;


#elif HAND == 1
        // we consider:
        // 1. the spread of the fingers F1 and F2 always at home ( theta8 = THETA8_home)
        // 2. the displacement of the other joints is equal (theta9=theta10=theta11)

        for (int i=0; i< HAND_FINGERS; ++i){
             this->rightFinalHand.at(i+1) = theta;
             this->rightFinalPosture.at(JOINTS_ARM+i+1)=theta;
        }

#endif

        if(prec){
            this->dHOr = this->dFH;
        }else{
            this->dHOr = obj->getRadius()+TOL_GRIP;
        }

        /*
        switch (grip_code) {
        case 111: case 112: case 113: case 114: case 121: case 122:
            //Precision grip

             this->dHOr = this->dFH;

            break;
        case 211: case 212: case 213: case 214:
            // Full Side grip

            this->dHOr = obj->getRadius()+TOL_GRIP;

            break;
        case 221: case 222:
            // Full Above and Full Below

            this->dHOr = obj->getSize().Zsize/2+TOL_GRIP;

            break;
        }
        */

        break;


    case 2: // left arm



        this->leftFinalHand.at(0) = THETA8_FINAL; // spread of F1 and F2
        this->leftFinalPosture.at(JOINTS_ARM)=THETA8_FINAL;

#if HAND==0

        this->leftFinalHand.at(1) = theta;
        this->leftFinalHand.at(2) = theta;
        this->leftFinalHand.at(3) = thetaT;

        this->leftFinalPosture.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture.at(JOINTS_ARM+3)=thetaT;

        this->leftFinalPosture_diseng.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture_diseng.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture_diseng.at(JOINTS_ARM+3)=thetaT;

        this->leftFinalPosture_eng.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture_eng.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture_eng.at(JOINTS_ARM+3)=thetaT;


#elif HAND==1

        // we consider:
        // 1. the spread of the fingers F1 and F2 always at home ( theta8 = THETA8_final)
        // 2. the displacement of the other joints is equal (theta9=theta10=theta11)

        for (int i = 0; i<HAND_FINGERS; ++i){
             this->leftFinalHand.at(i+1) = theta;
             this->leftFinalPosture.at(JOINTS_ARM+i+1)=theta;
        }

#endif

        if(prec){
            this->dHOl = this->dFH;
        }else{
            this->dHOl = obj->getRadius()+TOL_GRIP;
        }
        /*
        switch (grip_code) {
        case 111: case 112: case 113: case 114: case 121: case 122:
            //Precision grip

             this->dHOl = this->dFH;

            break;
        case 211: case 212: case 213: case 214:
            // Full Side grip

            this->dHOl = obj->getRadius()+TOL_GRIP;

            break;
        case 221: case 222:
            // Full Above and Full Below

            this->dHOl = obj->getSize().Zsize/2+TOL_GRIP;

            break;

        }
        */

        break;

    }



    return success;

}

bool Problem::invKinHand(double d_obj,std::vector<double>& sols)
{

    humanoidPtr hh = this->scene->getHumanoid();

    bool success = false;
    sols = std::vector<double>(2);
    double theta;
    double thetaT;


#if HAND==0

    human_hand hand = hh->getHumanHand();
    human_finger middle = hand.fingers.at(1);
    human_thumb thumb = hand.thumb;
    double maxAp = hand.maxAperture;
    int k;

    // middle finger
    //double ux = middle.ux;
    double uy;
    double uz = middle.uz;
    double Lp = middle.finger_specs.a.at(1);
    double Lmi = middle.finger_specs.a.at(2);
    double Ld = middle.finger_specs.a.at(3);
    double alpha0;
    double theta0;

    // thumb
    //double uTx = thumb.uTx;
    double uTy;
    //double uTz = thumb.uTz;
    double LTm = thumb.thumb_specs.a.at(2);
    double LTp = thumb.thumb_specs.a.at(3);
    double LTd = thumb.thumb_specs.a.at(4);
    double alpha0T;
    double alpha1T = thumb.thumb_specs.alpha.at(1);
    double theta0T = thumb.thumb_specs.theta.at(0);
    double theta1T = THETA8_FINAL;

    switch(hand_id){

    case 1: // right hand
        k=1;
        uy = middle.uy;
        alpha0 = middle.finger_specs.alpha.at(0);
        theta0 = middle.finger_specs.theta.at(0);
        uTy = thumb.uTy;
        alpha0T = thumb.thumb_specs.alpha.at(0);
        break;
    case 2: // left hand
        k=-1;
        uy = -middle.uy;
        alpha0 = -middle.finger_specs.alpha.at(0);
        theta0 = -middle.finger_specs.theta.at(0);
        uTy = -thumb.uTy;
        alpha0T = thumb.thumb_specs.alpha.at(0)-90*M_PI/180;
        break;
    }


#elif HAND==1

    double A1 = hh->getBarrettHand().A1;
    double A2 = hh->getBarrettHand().A2;
    double A3 = hh->getBarrettHand().A3;
    double D3 = hh->getBarrettHand().D3;
    double phi2 = hh->getBarrettHand().phi2;
    double phi3 = hh->getBarrettHand().phi3;
    double maxAp = hh->getBarrettHand().maxAperture;
    double fnew; // dFF
    double dfnew;// ddFF

    double x0 = 60.0* M_PI/180.0; // initial approximation
    double xold = x0;
    theta = xold;
    double xnew = 140.0* M_PI/180.0;

#endif

    if (d_obj > maxAp){ throw string(" the object is too big to be grasped");}
    int cnt=0;


#if HAND==0

    // initial approximation
    double xold = 30.0* M_PI/180.0;
    double xoldT = k*30.0* M_PI/180.0;
    theta = xold;
    thetaT = xoldT;
    double xnew = 140.0* M_PI/180.0;
    double xnewT = k*140.0* M_PI/180.0;
    double fnew;
    double dfnew;
    double fnewT;
    double dfnewT;
    double dMH;
    double dTH;

    while((abs(xnew-xold)>1e-4 || abs(xnewT-xoldT)>1e-4) && cnt <100){
        cnt++;

        xold=theta;
        xoldT=thetaT;

        // fnew = k*P_middle(2) - d_obj/2;
        fnew = k*(uy
                 -Ld*cos(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))
                 +Lmi*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))
                 -Ld*sin(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))
                 -Lmi*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))
                 +Lp*cos(theta)*sin(theta0) +Lp*cos(alpha0)*cos(theta0)*sin(theta))
                  -d_obj/2;

        dfnew = -k*((Ld*cos(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))))/3
                    +Ld*cos(theta/3)*((5*sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3 + (5*cos((2*theta)/3)*(sin(theta0)*sin(theta)-cos(alpha0)*cos(theta0)*cos(theta)))/3)
                    +(5*Lmi*cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))/3
                    -(Ld*sin(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))))/3
                    -Ld*sin(theta/3)*((5*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))/3 - (5*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3)
                    +(5*Lmi*sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3
                    +Lp*sin(theta0)*sin(theta) -Lp*cos(alpha0)*cos(theta0)*cos(theta));

        // fnewT = k*P_thumb(2) + d_obj/2;
        fnewT = -k*(LTp*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T)+cos(alpha1T)*sin(theta0T)*sin(theta1T)-cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))+sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                 -uTy
                 +LTm*sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
                 +LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) + sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 +LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) - sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 -LTm*cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))
                 +LTp*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 +d_obj/2;

        dfnewT = -k*(LTm*cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
                     -(21*LTp*sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                     +(11*LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                                                   -sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))))/12
                     +LTd*cos((11*thetaT)/12)*((21*cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                                               -(21*sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10)
                     -(11*LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                                                   +sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))))/12
                     -LTd*sin((11*thetaT)/12)*((21*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                                               +(21*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10)
                     +(21*LTp*cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                     +LTm*sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)));



        xnew=xold-fnew/dfnew;
        theta = xnew;

        xnewT=xoldT-fnewT/dfnewT;
        thetaT = xnewT;

    }
    if (cnt < 100){success = true;}

    theta=xnew;
    thetaT=abs(xnewT);


    dMH = k*(uy
             -Ld*cos(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))
             +Lmi*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))
             -Ld*sin(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))
             -Lmi*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))
             +Lp*cos(theta)*sin(theta0) +Lp*cos(alpha0)*cos(theta0)*sin(theta));

    dTH =k*(LTp*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T)+cos(alpha1T)*sin(theta0T)*sin(theta1T)-cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))+sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
             -uTy
             +LTm*sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
             +LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) + sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
             +LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) - sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
             -LTm*cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))
             +LTp*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))));


    this->dFF = dMH + dTH;

    this->dFH =uz+Lp*sin(alpha0)*sin(theta) + Ld*cos(theta/3)*(cos((2*theta)/3)*sin(alpha0)*sin(theta) + sin((2*theta)/3)*sin(alpha0)*cos(theta))
              +Ld*sin(theta/3)*(cos((2*theta)/3)*sin(alpha0)*cos(theta) - sin((2*theta)/3)*sin(alpha0)*sin(theta))
              +Lmi*cos((2*theta)/3)*sin(alpha0)*sin(theta)
              +Lmi*sin((2*theta)/3)*sin(alpha0)*cos(theta);

    sols.at(0)=theta;
    sols.at(1)=thetaT;


#elif HAND==1
    while(abs(xnew-xold)>1e-4 && cnt <100){

        cnt++;
        xold=theta;
        fnew=2*cos((4/3)*theta+phi2+phi3)*A3-2*sin((4/3)*theta+phi2+phi3)*D3+
                2*cos(theta+phi2)*A2+2*A1-d_obj;
        dfnew=-(8/3)*sin((4/3)*theta+phi2+phi3)*A3-(8/3)*cos((4/3)*theta+phi2+phi3)*D3-
                2*sin(theta+phi2)*A2;
        xnew=xold-fnew/dfnew;
        theta = xnew;


    }
    if (cnt < 100){success = true;}

    theta=xnew;
    this->dFF = 2*cos((4/3)*theta+phi2+phi3)*A3-2*sin((4/3)*theta+phi2+phi3)*D3+
            2*cos(theta+phi2)*A2+2*A1;
    this->dFH = sin((4/3)*theta+phi2+phi3)*A3+cos((4/3)*theta+phi2+phi3)*D3+sin(theta+phi2)*A2;

    sols.at(0)=theta;
    sols.at(1)=theta;

#endif


    return success;

}

bool Problem::getRPY(std::vector<double>& rpy, Matrix3d& Rot)
{
    if((Rot.cols()==3) && (Rot.rows()==3))
    {// the matrix is not empy
        rpy.resize(3,0);
        if((Rot(0,0)<1e-10) && (Rot(1,0)<1e-10))
        {// singularity
            rpy.at(0) = 0; // roll
            rpy.at(1) = std::atan2(-Rot(2,0),Rot(0,0)); // pitch
            rpy.at(2) = std::atan2(-Rot(1,2),Rot(1,1)); // yaw
            return false;
        }else{
            rpy.at(0) = std::atan2(Rot(1,0),Rot(0,0)); // roll
            double sp = std::sin(rpy.at(0)); double cp = std::cos(rpy.at(0));
            rpy.at(1) = std::atan2(-Rot(2,0),cp*Rot(0,0)+sp*Rot(1,0)); // pitch
            rpy.at(2) = std::atan2(sp*Rot(0,2)-cp*Rot(1,2),cp*Rot(1,1)-sp*Rot(0,1)); // yaw
            return true;
        }
    }else{
        return false;
    }
}

bool Problem::RPY_matrix(std::vector<double>& rpy, Matrix3d& Rot)
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

        return true;
    }else{
        return false;
    }
}



movementPtr Problem::getMovement()
{
    return this->mov;
}

HUMotion::planning_result_ptr Problem::solve(HUMotion::hump_params &params)
{

    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type = this->mov->getType();
    int sceneID = this->scene->getID();
    params.mov_specs.support_obj = "Table";
#if HAND==0
    // Human Hand
    int hand_code = 0;
#elif HAND == 1
    // Barrett Hand
    int hand_code = 1;
#endif
    double dHO;
    std::vector<double> finalHand;
    std::vector<double> homePosture;
    std::vector<double> initPosture;
    targetPtr tar;
    objectPtr obj; engagePtr eng;
    objectPtr obj_eng; engagePtr eng1;
    posePtr pose;
    if(mov_type!=1 && mov_type!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(arm_code);
        }catch(const string message){throw message;}
        obj = this->mov->getObject();
        pose = this->mov->getPose();
        // engaging info
        obj_eng = this->mov->getObjectEng();
        eng = obj->getEngagePoint();
        eng1 = obj_eng->getEngagePoint();
    }

    std::vector<double> eng_to_tar;
    switch(arm_code){
    case 0: // both arms
        break;
    case 1://right arm
        if(obj!=nullptr){obj->getEngTarRight(eng_to_tar);}
        this->scene->getHumanoid()->getRightPosture(initPosture);
        this->scene->getHumanoid()->getRightArmHomePosture(homePosture);
        if(mov_type==5){
          this->scene->getHumanoid()->getRightHandHomePosture(finalHand);
        }else if(mov_type==1){
            finalHand=this->move_final_hand;
        }else{
            dHO=this->dHOr;
            tar = obj->getTargetRight();
            finalHand = this->rightFinalHand;
        }
        break;
    case 2:// left arm
        if(obj!=nullptr){obj->getEngTarLeft(eng_to_tar);}
        this->scene->getHumanoid()->getLeftPosture(initPosture);
        this->scene->getHumanoid()->getLeftArmHomePosture(homePosture);
        if(mov_type==5){
          this->scene->getHumanoid()->getLeftHandHomePosture(finalHand);
        }else if(mov_type==1){
            finalHand=this->move_final_hand;
        }else{
            dHO=this->dHOl;
            finalHand = this->leftFinalHand;
            tar = obj->getTargetLeft();
        }
        break;
    }
    std::vector<double> target;
    std::vector<double> tar_pose;
    std::vector<double> place_location;
    if(mov_type!=1 && mov_type!=5){
        // compute the position of the target when the object will be engaged
        pos eng1_pos = eng1->getPos(); // position related to the world of the engage point of the other object
        orient eng1_or = eng1->getOr(); // orientation of the engage point of the other object
        std::vector<double> rpy_eng1 = {eng1_or.roll,eng1_or.pitch,eng1_or.yaw};
        Matrix3d Rot_eng1; this->RPY_matrix(rpy_eng1,Rot_eng1);
        pos new_tar;
        std::vector<double> rpy = {tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};
        Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
        Vector3d v(eng_to_tar.at(0),eng_to_tar.at(1),eng_to_tar.at(2));
        Vector3d eng_to_tar_w = Rot_tar*v;
        new_tar.Xpos=eng1_pos.Xpos - eng_to_tar_w(0);
        new_tar.Ypos=eng1_pos.Ypos - eng_to_tar_w(1);
        new_tar.Zpos=eng1_pos.Zpos - eng_to_tar_w(2);

        orient eng_or = eng->getOr(); // orientation of the engage point of the object to engage
        std::vector<double> rpy_eng = {eng_or.roll,eng_or.pitch,eng_or.yaw};
        Matrix3d Rot_eng; this->RPY_matrix(rpy_eng,Rot_eng);
        Matrix3d Rot_eng_inv = Rot_eng.transpose();
        Matrix3d Rot_eng_tar = Rot_eng_inv * Rot_tar;
        Matrix3d Rot_new_tar = Rot_eng1 * Rot_eng_tar;
        std::vector<double> rpy_new_tar; this->getRPY(rpy_new_tar,Rot_new_tar);

        place_location.push_back(new_tar.Xpos);
        place_location.push_back(new_tar.Ypos);
        place_location.push_back(new_tar.Zpos);
        place_location.push_back(rpy_new_tar.at(0));
        place_location.push_back(rpy_new_tar.at(1));
        place_location.push_back(rpy_new_tar.at(2));

        HUMotion::objectPtr hump_obj;
        target = {tar->getPos().Xpos, tar->getPos().Ypos, tar->getPos().Zpos,tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};
        tar_pose = {pose->getPos().Xpos, pose->getPos().Ypos, pose->getPos().Zpos, pose->getOr().roll, pose->getOr().pitch, pose->getOr().yaw};
        std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
        std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
        std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
        hump_obj.reset(new HUMotion::Object(obj->getName()));
        hump_obj->setParams(position,orientation,dimension);

        // movement settings
        //params.mov_specs.griptype = this->mov->getGrip();
        params.mov_specs.dHO = dHO;
        params.mov_specs.obj = hump_obj;        

    }

    // movement settings
    params.mov_specs.arm_code = arm_code;
    params.mov_specs.hand_code = hand_code;
    params.mov_specs.mov_infoline = this->mov->getInfoLine();
    params.mov_specs.finalHand = finalHand;

    HUMotion::planning_result_ptr res;
    long long curr_time;
    switch(mov_type){
    case 0:// reach-to-grasp
        params.mov_specs.target = target;
        curr_time = this->GetTimeMs64();
        res =  this->h_planner->plan_pick(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 1:// reaching
        if(this->use_posture){
          curr_time = this->GetTimeMs64();
          res = this->h_planner->plan_move(params,initPosture,this->move_final_arm);
          this->exec_time = double(this->GetTimeMs64()-curr_time);
        }else{
         params.mov_specs.target=this->move_target;
         curr_time = this->GetTimeMs64();
         res = this->h_planner->plan_move(params,initPosture);
         this->exec_time = double(this->GetTimeMs64()-curr_time);
        }
        break;
    case 2://transport
        if (sceneID==6){
            params.mov_specs.support_obj = "Shelf_2_a";
        }
        params.mov_specs.target = tar_pose;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 3://engage
        params.mov_specs.support_obj = obj_eng->getName();
        params.mov_specs.target = place_location;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 4:// disengage
        // TO DO
        //params.mov_specs.target = pose;
        //curr_time = this->GetTimeMs64();
        //res = this->h_planner->plan_place(params,initPosture);
        //this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 5:// go-park
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_move(params,initPosture,homePosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    }
    this->h_params = params;
    if(res!=nullptr){if(res->status==0){this->solved=true;}}

    return res;
}

HUMotion::planning_dual_result_ptr Problem::solve(HUMotion::hump_dual_params &params)
{
    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type_right = this->mov->getType(); int mov_type_left = this->mov->getTypeLeft();
    int sceneID = this->scene->getID();
#if HAND==0
    // Human Hand
    int hand_code = 0;
#elif HAND == 1
    // Barrett Hand
    int hand_code = 1;
#endif
    double dHO_right; double dHO_left;
    std::vector<double> finalHand_right; std::vector<double> finalHand_left;
    std::vector<double> homePosture_right; std::vector<double> homePosture_left;
    std::vector<double> initPosture_right; std::vector<double> initPosture_left;

    targetPtr tar_right; targetPtr tar_left;
    objectPtr obj_right; engagePtr eng_right;
    objectPtr obj_left; engagePtr eng_left;
    objectPtr obj_eng_right; engagePtr eng1_right;
    objectPtr obj_eng_left; engagePtr eng1_left;
    posePtr pose_right; posePtr pose_left;
    if(mov_type_right!=1 && mov_type_right!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(1); // right hand
        }catch(const string message){throw message;}
        obj_right = this->mov->getObject();
        pose_right = this->mov->getPose();
        // engaging info
        obj_eng_right = this->mov->getObjectEng();
        eng_right = obj_right->getEngagePoint();
        eng1_right = obj_eng_right->getEngagePoint();
    }    
    if(mov_type_left!=1 && mov_type_left!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(2); // left hand
        }catch(const string message){throw message;}
        obj_left = this->mov->getObjectLeft();
        pose_left = this->mov->getPoseLeft();
        // engaging info
        obj_eng_left = this->mov->getObjectEngLeft();
        eng_left = obj_left->getEngagePoint();
        eng1_left = obj_eng_left->getEngagePoint();
    }

    std::vector<double> eng_to_tar_right; std::vector<double> eng_to_tar_left;
    if(obj_right!=nullptr){obj_right->getEngTarRight(eng_to_tar_right);}
    if(obj_left!=nullptr){obj_left->getEngTarLeft(eng_to_tar_left);}
    this->scene->getHumanoid()->getRightPosture(initPosture_right);
    this->scene->getHumanoid()->getLeftPosture(initPosture_left);
    this->scene->getHumanoid()->getRightArmHomePosture(homePosture_right);
    this->scene->getHumanoid()->getLeftArmHomePosture(homePosture_left);
    if(mov_type_right==5){
      this->scene->getHumanoid()->getRightHandHomePosture(finalHand_right);
    }else if(mov_type_right==1){
        finalHand_right=this->move_final_hand_right;
    }else{
        dHO_right=this->dHOr;
        tar_right = obj_right->getTargetRight();
        finalHand_right = this->rightFinalHand;
    }
    if(mov_type_left==5){
      this->scene->getHumanoid()->getLeftHandHomePosture(finalHand_left);
    }else if(mov_type_left==1){
        finalHand_left=this->move_final_hand_left;
    }else{
        dHO_left=this->dHOl;
        tar_left = obj_left->getTargetLeft();
        finalHand_left = this->leftFinalHand;
    }

    std::vector<double> target_right; std::vector<double> target_left;
    std::vector<double> tar_pose_right; std::vector<double> tar_pose_left;
    std::vector<double> place_location_right; std::vector<double> place_location_left;
    if(mov_type_right!=1 && mov_type_right!=5){
        // compute the position of the target when the object will be engaged

        // position related to the world of the engage point of the other object
        pos eng1_right_pos = eng1_right->getPos();
        // orientation of the engage point of the other object
        orient eng1_right_or = eng1_right->getOr();
        std::vector<double> rpy_eng1_right = {eng1_right_or.roll,eng1_right_or.pitch,eng1_right_or.yaw};
        Matrix3d Rot_eng1_right; this->RPY_matrix(rpy_eng1_right,Rot_eng1_right);

        std::vector<double> rpy_right = {tar_right->getOr().roll,tar_right->getOr().pitch,tar_right->getOr().yaw};
        Matrix3d Rot_tar_right; this->RPY_matrix(rpy_right,Rot_tar_right);

        pos new_tar_right;
        Vector3d v_right(eng_to_tar_right.at(0),eng_to_tar_right.at(1),eng_to_tar_right.at(2));
        Vector3d eng_to_tar_right_w = Rot_tar_right*v_right;
        new_tar_right.Xpos=eng1_right_pos.Xpos - eng_to_tar_right_w(0);
        new_tar_right.Ypos=eng1_right_pos.Ypos - eng_to_tar_right_w(1);
        new_tar_right.Zpos=eng1_right_pos.Zpos - eng_to_tar_right_w(2);

        // orientation of the engage point of the object to engage
        orient eng_right_or = eng_right->getOr();
        std::vector<double> rpy_eng_right = {eng_right_or.roll,eng_right_or.pitch,eng_right_or.yaw};
        Matrix3d Rot_eng_right; this->RPY_matrix(rpy_eng_right,Rot_eng_right);
        Matrix3d Rot_eng_right_inv = Rot_eng_right.transpose();
        Matrix3d Rot_eng_tar_right = Rot_eng_right_inv * Rot_tar_right;
        Matrix3d Rot_new_tar_right = Rot_eng1_right * Rot_eng_tar_right;
        std::vector<double> rpy_new_tar_right; this->getRPY(rpy_new_tar_right,Rot_new_tar_right);

        place_location_right.push_back(new_tar_right.Xpos);
        place_location_right.push_back(new_tar_right.Ypos);
        place_location_right.push_back(new_tar_right.Zpos);
        place_location_right.push_back(rpy_new_tar_right.at(0));
        place_location_right.push_back(rpy_new_tar_right.at(1));
        place_location_right.push_back(rpy_new_tar_right.at(2));

        HUMotion::objectPtr hump_obj_right;
        target_right = {tar_right->getPos().Xpos, tar_right->getPos().Ypos, tar_right->getPos().Zpos,tar_right->getOr().roll,tar_right->getOr().pitch,tar_right->getOr().yaw};
        tar_pose_right = {pose_right->getPos().Xpos, pose_right->getPos().Ypos, pose_right->getPos().Zpos, pose_right->getOr().roll, pose_right->getOr().pitch, pose_right->getOr().yaw};
        std::vector<double> position_right = {obj_right->getPos().Xpos,obj_right->getPos().Ypos,obj_right->getPos().Zpos};
        std::vector<double> orientation_right = {obj_right->getOr().roll,obj_right->getOr().pitch,obj_right->getOr().yaw};
        std::vector<double> dimension_right = {obj_right->getSize().Xsize,obj_right->getSize().Ysize,obj_right->getSize().Zsize};
        hump_obj_right.reset(new HUMotion::Object(obj_right->getName()));
        hump_obj_right->setParams(position_right,orientation_right,dimension_right);

        // movement settings
        //params.mov_specs.griptype = this->mov->getGrip();
        params.mov_specs_right.dHO = dHO_right;
        params.mov_specs_right.obj = hump_obj_right;

    }

    if(mov_type_left!=1 && mov_type_left!=5){
        // compute the position of the target when the object will be engaged

        // position related to the world of the engage point of the other object
        pos eng1_left_pos = eng1_left->getPos();
        // orientation of the engage point of the other object
        orient eng1_left_or = eng1_left->getOr();
        std::vector<double> rpy_eng1_left = {eng1_left_or.roll,eng1_left_or.pitch,eng1_left_or.yaw};
        Matrix3d Rot_eng1_left; this->RPY_matrix(rpy_eng1_left,Rot_eng1_left);

        std::vector<double> rpy_left = {tar_left->getOr().roll,tar_left->getOr().pitch,tar_left->getOr().yaw};
        Matrix3d Rot_tar_left; this->RPY_matrix(rpy_left,Rot_tar_left);

        pos new_tar_left;
        Vector3d v_left(eng_to_tar_left.at(0),eng_to_tar_left.at(1),eng_to_tar_left.at(2));
        Vector3d eng_to_tar_left_w = Rot_tar_left*v_left;
        new_tar_left.Xpos=eng1_left_pos.Xpos - eng_to_tar_left_w(0);
        new_tar_left.Ypos=eng1_left_pos.Ypos - eng_to_tar_left_w(1);
        new_tar_left.Zpos=eng1_left_pos.Zpos - eng_to_tar_left_w(2);

        // orientation of the engage point of the object to engage
        orient eng_left_or = eng_left->getOr();
        std::vector<double> rpy_eng_left = {eng_left_or.roll,eng_left_or.pitch,eng_left_or.yaw};
        Matrix3d Rot_eng_left; this->RPY_matrix(rpy_eng_left,Rot_eng_left);
        Matrix3d Rot_eng_left_inv = Rot_eng_left.transpose();
        Matrix3d Rot_eng_tar_left = Rot_eng_left_inv * Rot_tar_left;
        Matrix3d Rot_new_tar_left = Rot_eng1_left * Rot_eng_tar_left;
        std::vector<double> rpy_new_tar_left; this->getRPY(rpy_new_tar_left,Rot_new_tar_left);

        place_location_left.push_back(new_tar_left.Xpos);
        place_location_left.push_back(new_tar_left.Ypos);
        place_location_left.push_back(new_tar_left.Zpos);
        place_location_left.push_back(rpy_new_tar_left.at(0));
        place_location_left.push_back(rpy_new_tar_left.at(1));
        place_location_left.push_back(rpy_new_tar_left.at(2));

        HUMotion::objectPtr hump_obj_left;
        target_left = {tar_left->getPos().Xpos, tar_left->getPos().Ypos, tar_left->getPos().Zpos,tar_left->getOr().roll,tar_left->getOr().pitch,tar_left->getOr().yaw};
        tar_pose_left = {pose_left->getPos().Xpos, pose_left->getPos().Ypos, pose_left->getPos().Zpos, pose_left->getOr().roll, pose_left->getOr().pitch, pose_left->getOr().yaw};
        std::vector<double> position_left = {obj_left->getPos().Xpos,obj_left->getPos().Ypos,obj_left->getPos().Zpos};
        std::vector<double> orientation_left = {obj_left->getOr().roll,obj_left->getOr().pitch,obj_left->getOr().yaw};
        std::vector<double> dimension_left = {obj_left->getSize().Xsize,obj_left->getSize().Ysize,obj_left->getSize().Zsize};
        hump_obj_left.reset(new HUMotion::Object(obj_left->getName()));
        hump_obj_left->setParams(position_left,orientation_left,dimension_left);

        // movement settings
        //params.mov_specs.griptype = this->mov->getGrip();
        params.mov_specs_left.dHO = dHO_left;
        params.mov_specs_left.obj = hump_obj_left;

    }

    // movement settings (right)
    params.mov_specs_right.arm_code = arm_code;
    params.mov_specs_right.hand_code = hand_code;
    params.mov_specs_right.mov_infoline = this->mov->getInfoLine();
    params.mov_specs_right.finalHand = finalHand_right;

    // movement settings (left)
    params.mov_specs_left.arm_code = arm_code;
    params.mov_specs_left.hand_code = hand_code;
    params.mov_specs_left.mov_infoline = this->mov->getInfoLine();
    params.mov_specs_left.finalHand = finalHand_left;

    HUMotion::planning_dual_result_ptr res;
    long long curr_time;

    if (mov_type_right==0 && mov_type_left==0)
    { // dual-arm reach-to-grasp right and reach-to-grasp left

        params.mov_specs_right.target = target_right;
        params.mov_specs_left.target = target_left;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_dual_pick_pick(params,initPosture_right,initPosture_left);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
    }

    /*
    switch(mov_type_right){
    case 0:// reach-to-grasp
        params.mov_specs_right.target = target_right;
        curr_time = this->GetTimeMs64();
        res =  this->h_planner->plan_pick(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 1:// reaching
        if(this->use_posture){
          curr_time = this->GetTimeMs64();
          res = this->h_planner->plan_move(params,initPosture,this->move_final_arm);
          this->exec_time = double(this->GetTimeMs64()-curr_time);
        }else{
         params.mov_specs.target=this->move_target;
         curr_time = this->GetTimeMs64();
         res = this->h_planner->plan_move(params,initPosture);
         this->exec_time = double(this->GetTimeMs64()-curr_time);
        }
        break;
    case 2://transport
        if (sceneID==6){
            params.mov_specs.support_obj = "Shelf_2_a";
        }
        params.mov_specs.target = tar_pose;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 3://engage
        params.mov_specs.support_obj = obj_eng->getName();
        params.mov_specs.target = place_location;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 4:// disengage
        // TO DO
        //params.mov_specs.target = pose;
        //curr_time = this->GetTimeMs64();
        //res = this->h_planner->plan_place(params,initPosture);
        //this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 5:// go-park
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_move(params,initPosture,homePosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    }
    */

    this->h_dual_params = params;
    if(res!=nullptr){if(res->status==0){this->solved=true;}}

    return res;

}

#if MOVEIT==1
moveit_planning::PlanningResultPtr Problem::solve(moveit_planning::moveit_params &params)
{
    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type = this->mov->getType();
    int sceneID = this->scene->getID();
    params.support_surface = "Table";

#if HAND==0
    // Human Hand
    int hand_code = 0;
#elif HAND == 1
    // Barrett Hand
    int hand_code = 1;
#endif
    double dHO;
    std::vector<double> finalHand;
    std::vector<double> homePosture;
    targetPtr tar;
    objectPtr obj; engagePtr eng;
    objectPtr obj_eng; engagePtr eng1;
    posePtr pose;
    if(mov_type!=1 && mov_type!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(arm_code);
        }catch(const string message){throw message;}
        obj = this->mov->getObject();
        pose = this->mov->getPose();
        // engaging info
        obj_eng = this->mov->getObjectEng();
        eng = obj->getEngagePoint();
        eng1 = obj_eng->getEngagePoint();
    }

    std::vector<double> eng_to_obj;
    if(obj!=nullptr){obj->getEngObj(eng_to_obj);}
    std::vector<double> tar_to_obj;
    Matrix3d Rot_tar_or;
    switch(arm_code){
    case 0: // both arms
        break;
    case 1://right arm
        if(obj!=nullptr){
            obj->getTarRightObj(tar_to_obj);
            obj->getTar_right_RPY_matrix(Rot_tar_or);
        }
        this->scene->getHumanoid()->getRightArmHomePosture(homePosture);
        if(mov_type==5){
            this->scene->getHumanoid()->getRightHandHomePosture(finalHand);
        }else if(mov_type==1){
            finalHand=this->move_final_hand;
        }else{
            dHO=this->dHOr;
            finalHand = this->rightFinalHand;
            tar = obj->getTargetRight();
        }
        break;
    case 2:// left arm
        if(obj!=nullptr){
            obj->getTarLeftObj(tar_to_obj);
            obj->getTar_left_RPY_matrix(Rot_tar_or);
        }
        this->scene->getHumanoid()->getLeftArmHomePosture(homePosture);
        if(mov_type==5){
            this->scene->getHumanoid()->getLeftHandHomePosture(finalHand);
        }else if(mov_type==1){
            finalHand=this->move_final_hand;
        }else{
            dHO=this->dHOl;
            finalHand = this->leftFinalHand;
            tar = obj->getTargetLeft();
        }
        break;
    }
    std::vector<double> target;
    std::vector<double> tar_pose;
    std::vector<double> place_location;
    if(mov_type!=1 && mov_type!=5){
        // compute the position of the target when the object will be engaged
        pos eng1_pos = eng1->getPos(); // position of the engage point of the other object
        orient eng1_or = eng1->getOr(); // orientation of the engage point of the other object
        std::vector<double> rpy_eng1 = {eng1_or.roll,eng1_or.pitch,eng1_or.yaw};
        Matrix3d Rot_eng1; this->RPY_matrix(rpy_eng1,Rot_eng1);
        pos new_obj_pos;
        orient eng_or = eng->getOr(); // orientation of the engage point of the object to engage
        std::vector<double> rpy_eng = {eng_or.roll,eng_or.pitch,eng_or.yaw};
        Matrix3d Rot_eng; this->RPY_matrix(rpy_eng,Rot_eng);
        Matrix3d Rot_eng_inv = Rot_eng.transpose();
        orient obj_or = obj->getOr(); // orientation of the objetc to engage
        std::vector<double> rpy_obj = {obj_or.roll,obj_or.pitch,obj_or.yaw};
        Matrix3d Rot_obj; this->RPY_matrix(rpy_obj,Rot_obj);
        Matrix3d Rot_eng_obj = Rot_eng_inv * Rot_obj;
        Matrix3d Rot_obj1 = Rot_eng1 * Rot_eng_obj;
        std::vector<double> rpy_obj1; this->getRPY(rpy_obj1,Rot_obj1);

        Vector3d v(eng_to_obj.at(0),eng_to_obj.at(1),eng_to_obj.at(2));
        Vector3d eng_to_obj_w = Rot_obj*v;
        new_obj_pos.Xpos=eng1_pos.Xpos - eng_to_obj_w(0);
        new_obj_pos.Ypos=eng1_pos.Ypos - eng_to_obj_w(1);
        new_obj_pos.Zpos=eng1_pos.Zpos - eng_to_obj_w(2);

        place_location.push_back(new_obj_pos.Xpos/1000);
        place_location.push_back(new_obj_pos.Ypos/1000);
        place_location.push_back(new_obj_pos.Zpos/1000);
        place_location.push_back(rpy_obj1.at(0));
        place_location.push_back(rpy_obj1.at(1));
        place_location.push_back(rpy_obj1.at(2));

        target = {tar->getPos().Xpos/1000, tar->getPos().Ypos/1000, tar->getPos().Zpos/1000,
                  tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};

        pos new_tar_pose;
        Vector3d vp(tar_to_obj.at(0),tar_to_obj.at(1),tar_to_obj.at(2));
        Vector3d tar_to_obj_w = Rot_obj*vp;
        new_tar_pose.Xpos=pose->getPos().Xpos - tar_to_obj_w(0);
        new_tar_pose.Ypos=pose->getPos().Ypos - tar_to_obj_w(1);
        new_tar_pose.Zpos=pose->getPos().Zpos - tar_to_obj_w(2);
        Matrix3d Rot_tar_or_inv = Rot_tar_or.transpose();
        Matrix3d Rot_tar_obj = Rot_tar_or_inv * Rot_obj;
        std::vector<double> rpy_pose = {pose->getOr().roll,pose->getOr().pitch,pose->getOr().yaw};
        Matrix3d Rot_pose; this->RPY_matrix(rpy_pose,Rot_pose);
        Matrix3d Rot_new_obj_pose = Rot_pose * Rot_tar_obj;
        std::vector<double> rpy_obj_pose; this->getRPY(rpy_obj_pose,Rot_new_obj_pose);

        tar_pose = {new_tar_pose.Xpos/1000, new_tar_pose.Ypos/1000, new_tar_pose.Zpos/1000,
                    rpy_obj_pose.at(0),rpy_obj_pose.at(1),rpy_obj_pose.at(2)};

        // movement settings
        params.dHO = dHO/1000;
        //params.griptype = this->mov->getGrip();
        params.obj_name = this->mov->getObject()->getName();
    }

    // movement settings
    params.arm_code = arm_code;
    params.hand_code = hand_code;
    params.mov_infoline = this->mov->getInfoLine();
    params.finalHand = finalHand;

    moveit_planning::PlanningResultPtr res;
    long long curr_time;
    switch(mov_type){
    case 0:// reach-to-grasp
        if (sceneID==6){
            params.support_surface = "Shelf_4_a";
            params.allowed_touch_objects = {"Shelf","Shelf_4_d","Shelf_3"};
        }
        params.target = target;
        curr_time = this->GetTimeMs64();
        res =  this->m_planner->pick(params);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 1:// reaching
        if(this->use_posture){
          curr_time = this->GetTimeMs64();
          res = this->m_planner->move(params,this->move_final_arm);
          this->exec_time = double(this->GetTimeMs64()-curr_time);
        }else{
          params.target=this->move_target;
          curr_time = this->GetTimeMs64();
          res = this->m_planner->move(params);
          this->exec_time = double(this->GetTimeMs64()-curr_time);
        }
        break;
    case 2://transport
        if (sceneID==6){
            params.support_surface = "Shelf_2_a";
            params.allowed_touch_objects = {"Shelf_4_a","Shelf_3","Shelf"};
        }
        params.target = tar_pose;
        curr_time = this->GetTimeMs64();
        res =  this->m_planner->place(params);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 3://engage
        params.support_surface = obj_eng->getName();
        params.target = place_location;
        curr_time = this->GetTimeMs64();
        res =  this->m_planner->place(params);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    case 4:// disengage
        // TO DO
        break;
    case 5:// go-park
        curr_time = this->GetTimeMs64();
        res = this->m_planner->move(params,homePosture);
        this->exec_time = double(this->GetTimeMs64()-curr_time);
        break;
    }
    this->m_params = params;
    if(res!=nullptr){if(res->status==1){this->solved=true;}}

    return res;


}
#endif

long long Problem::GetTimeMs64()
{
#ifdef WIN32
 /* Windows */
 FILETIME ft;
 LARGE_INTEGER li;
 /* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
  * to a LARGE_INTEGER structure. */
 GetSystemTimeAsFileTime(&ft);
 li.LowPart = ft.dwLowDateTime;
 li.HighPart = ft.dwHighDateTime;
 unsigned long long ret = li.QuadPart;
 ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
 ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */
 return ret;
#else
 /* Linux */
 struct timeval tv;
 gettimeofday(&tv, NULL);
 uint64_t ret = tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000;
 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 ret += (tv.tv_sec * 1000);
 return ret;
#endif
}

double Problem::getTime()
{
    return this->exec_time;
}


}// motion_manager
