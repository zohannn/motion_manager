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

    this->mov = movementPtr(mov);
    this->scene = scenarioPtr(scene);
    this->planner_id=planner_id;

    bool huml = false;

    if (planner_id==0){huml = true;this->planner_name = "HUML";}

    string scene_name = this->scene->getName();
    //int scene_id = this->scene->getID();

    if (huml){
        // --- Human-like movement planner settings --- //
        HUMotion::HUMPlanner::hand_fingers = HAND_FINGERS;
        HUMotion::HUMPlanner::joints_arm = JOINTS_ARM;
        HUMotion::HUMPlanner::joints_hand = JOINTS_HAND;
        HUMotion::HUMPlanner::n_phalange = N_PHALANGE;
        this->m_planner = nullptr;
        this->h_planner.reset(new HUMotion::HUMPlanner(scene_name));
        // set the current obstacles and targets of the scenario
        vector<objectPtr> scene_objects;
        if(this->scene->getObjects(scene_objects)){
            HUMotion::objectPtr huml_obj; // object of the planner
            objectPtr obj;
            for(size_t i=0; i < scene_objects.size(); ++i){
                obj = scene_objects.at(i);
                std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
                std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
                std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
                huml_obj.reset(new HUMotion::Object(obj->getName()));
                huml_obj->setParams(position,orientation,dimension);
                if(!obj->isTargetRightEnabled() && !obj->isTargetLeftEnabled()){
                    this->h_planner->addObstacle(huml_obj); // the object is an obstacle for the planner
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
        HUMotion::HumanHand huml_hhand;
        huml_hhand.maxAperture = hhand.maxAperture;
        huml_hhand.thumb.uTx = hhand.thumb.uTx;
        huml_hhand.thumb.uTy = hhand.thumb.uTy;
        huml_hhand.thumb.uTz = hhand.thumb.uTz;
        huml_hhand.thumb.thumb_specs.a = hhand.thumb.thumb_specs.a;
        huml_hhand.thumb.thumb_specs.alpha = hhand.thumb.thumb_specs.alpha;
        huml_hhand.thumb.thumb_specs.d = hhand.thumb.thumb_specs.d;
        huml_hhand.thumb.thumb_specs.theta = hhand.thumb.thumb_specs.theta;
        vector<HUMotion::HumanFinger> huml_fings = huml_hhand.fingers;
        vector<human_finger> fings = hhand.fingers;
        for(size_t i=0; i<fings.size();++i){
            human_finger fing = fings.at(i);
            HUMotion::HumanFinger huml_fing = huml_fings.at(i);
            huml_fing.ux = fing.ux; huml_fing.uy = fing.uy; huml_fing.uz = fing.uz;
            huml_fing.finger_specs.a = fing.finger_specs.a;
            huml_fing.finger_specs.alpha = fing.finger_specs.alpha;
            huml_fing.finger_specs.d = fing.finger_specs.d;
            huml_fing.finger_specs.theta = fing.finger_specs.theta;
            huml_fings.at(i) = huml_fing;
        }
        huml_hhand.fingers = huml_fings;
#elif HAND==1
        barrett_hand b_hand = this->scene->getHumanoid()->getBarrettHand();
        std::vector<int> rk; this->scene->getHumanoid()->getRK(rk);
        std::vector<int> jk; this->scene->getHumanoid()->getRK(jk);
        HUMotion::BarrettHand huml_bhand;
        huml_bhand.A1 = b_hand.A1;
        huml_bhand.A2 = b_hand.A2;
        huml_bhand.A3 = b_hand.A3;
        huml_bhand.Aw = b_hand.Aw;
        huml_bhand.D3 = b_hand.D3;
        huml_bhand.maxAperture = b_hand.maxAperture;
        huml_bhand.phi2 = b_hand.phi2;
        huml_bhand.phi3 = b_hand.phi3;
        huml_bhand.rk = rk;
        huml_bhand.jk = jk;
        h_planner->setBarrettHand(huml_bhand);
#endif

    }

}

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
        this->planner_name = "HUML";
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
    if(s.m_planner!=nullptr){
        this->m_planner = moveit_plannerPtr(new moveit_planning::HumanoidPlanner(*s.m_planner.get()));
    }
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
        this->planner_name = "HUML";
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

bool Problem::finalPostureFingers(int hand_id)
{

    bool success=false;
    humanoidPtr hh = this->scene->getHumanoid();


    // get the object(s) involved in this movement
    objectPtr obj = this->mov->getObject();
    // get the type of grip for this movement
    int grip_code = this->mov->getGrip();

    // compute the diameter (plus tolerance) of the object to be grasped
    double d_obj;

    switch (grip_code) {

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

    // compute the inverse kinematics of the hand
    std::vector<double> sols;
    bool inv_succ;
    double theta;
    double thetaT;

    try{

        inv_succ = this->invKinHand(d_obj,hand_id,sols);
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

    switch(this->mov->getArm()){

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

        break;

    }



    return success;

}

bool Problem::invKinHand(double d_obj,int hand_id,std::vector<double>& sols)
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



movementPtr Problem::getMovement()
{
    return this->mov;
}

HUMotion::planning_result_ptr Problem::solve(HUMotion::huml_params &params)
{

    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type = this->mov->getType();
#if HAND==0
    // Human Hand
    int hand_code = 0;
#elif HAND == 1
    // Barrett Hand
    int hand_code = 1;
#endif
    std::vector<double> shPos;
    double dHO;
    std::vector<double> finalHand;
    std::vector<double> homePosture;
    std::vector<double> initPosture;
    targetPtr tar;
    objectPtr obj; engagePtr eng;
    objectPtr obj_eng; engagePtr eng1;
    if(mov_type!=1 && mov_type!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(arm_code);
        }catch(const string message){throw message;}
        obj = this->mov->getObject();
        // engaging info
        obj_eng = this->mov->getObjectEng();
        eng = obj->getEngagePoint();
        eng1 = obj_eng->getEngagePoint();
    }

    switch(arm_code){
    case 0: // both arms
        break;
    case 1://right arm
        this->scene->getHumanoid()->getRightShoulderPos(shPos);
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
        this->scene->getHumanoid()->getLeftShoulderPos(shPos);
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
    // shoulder position (center of the reacheble workspace )
    this->h_planner->setShpos(shPos);
    std::vector<double> target;
    std::vector<double> place_location;
    if(mov_type!=1 && mov_type!=5){
        // compute the position of the engage point relative to the target frame
        pos tar_pos = tar->getPos();
        pos eng_pos = eng->getPos();
        Matrix3d Rot_tar; tar->RPY_matrix(Rot_tar);
        Matrix3d Rot_tar_inv = Rot_tar.inverse();
        Vector3d diff;
        diff(0) = eng_pos.Xpos - tar_pos.Xpos;
        diff(1) = eng_pos.Ypos - tar_pos.Ypos;
        diff(2) = eng_pos.Zpos - tar_pos.Zpos;
        Vector3d eng_to_tar; eng_to_tar = Rot_tar_inv * diff;
        // compute the position of the target when the object will be engaged
        pos eng1_pos = eng1->getPos(); // position of the engage point of the other object
        pos new_tar;
        new_tar.Xpos=eng1_pos.Xpos - eng_to_tar(0);
        new_tar.Ypos=eng1_pos.Ypos - eng_to_tar(1);
        new_tar.Zpos=eng1_pos.Zpos - eng_to_tar(2);

        place_location.push_back(new_tar.Xpos);
        place_location.push_back(new_tar.Ypos);
        place_location.push_back(new_tar.Zpos);
        place_location.push_back(eng1->getOr().roll);
        place_location.push_back(eng1->getOr().pitch);
        place_location.push_back(eng1->getOr().yaw);

        HUMotion::objectPtr huml_obj;
        target = {tar->getPos().Xpos, tar->getPos().Ypos, tar->getPos().Zpos,tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};
        std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
        std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
        std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
        huml_obj.reset(new HUMotion::Object(obj->getName()));
        huml_obj->setParams(position,orientation,dimension);

        // movement settings
        params.mov_specs.griptype = this->mov->getGrip();
        params.mov_specs.dHO = dHO;
        params.mov_specs.obj = huml_obj;

    }

    // movement settings
    params.mov_specs.arm_code = arm_code;
    params.mov_specs.hand_code = hand_code;
    params.mov_specs.mov_infoline = this->mov->getInfoLine();
    params.mov_specs.finalHand = finalHand;

    HUMotion::planning_result_ptr res;
    switch(mov_type){
    case 0:// reach-to-grasp
        params.mov_specs.target = target;
        res =  this->h_planner->plan_pick(params,initPosture);
        break;
    case 1:// reaching
        if(this->use_posture){
          res = this->h_planner->plan_move(params,initPosture,this->move_final_arm);
        }else{
         params.mov_specs.target=this->move_target;
         res = this->h_planner->plan_move(params,initPosture);
        }
        break;
    case 2://transport
        break;
    case 3://engage
        params.mov_specs.target = place_location;
        res = this->h_planner->plan_place(params,initPosture);
        break;
    case 4:// disengage
        break;
    case 5:// go-park
        res = this->h_planner->plan_move(params,initPosture,homePosture);
        break;
    }
    this->h_params = params;
    if(res->status==0){this->solved=true;}

    return res;
}


moveit_planning::PlanningResultPtr Problem::solve(moveit_planning::moveit_params &params)
{
    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type = this->mov->getType();
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
    if(mov_type!=1 && mov_type!=5){
        try{ // compute the final posture of the fingers according to the object involved in the movement
            this->finalPostureFingers(arm_code);
        }catch(const string message){throw message;}
        obj = this->mov->getObject();
        // engaging info
        obj_eng = this->mov->getObjectEng();
        eng = obj->getEngagePoint();
        eng1 = obj_eng->getEngagePoint();
    }

    switch(arm_code){
    case 0: // both arms
        break;
    case 1://right arm
        this->scene->getHumanoid()->getRightArmHomePosture(homePosture);
        if(mov_type==5){
            this->scene->getHumanoid()->getRightHandHomePosture(finalHand);
        }else{
            dHO=this->dHOr;
            finalHand = this->rightFinalHand;
            tar = obj->getTargetRight();
        }
        break;
    case 2:// left arm
        this->scene->getHumanoid()->getLeftArmHomePosture(homePosture);
        if(mov_type==5){
            this->scene->getHumanoid()->getLeftHandHomePosture(finalHand);
        }else{
            dHO=this->dHOl;
            finalHand = this->leftFinalHand;
            tar = obj->getTargetLeft();
        }
        break;
    }
    std::vector<double> target;
    std::vector<double> place_location;
    if(mov_type!=1 && mov_type!=5){
        // compute the position of the engage point relative to the target frame
        pos tar_pos = tar->getPos();
        pos eng_pos = eng->getPos();
        Matrix3d Rot_tar; tar->RPY_matrix(Rot_tar);
        Matrix3d Rot_tar_inv = Rot_tar.inverse();
        Vector3d diff;
        diff(0) = eng_pos.Xpos - tar_pos.Xpos;
        diff(1) = eng_pos.Ypos - tar_pos.Ypos;
        diff(2) = eng_pos.Zpos - tar_pos.Zpos;
        Vector3d eng_to_tar; eng_to_tar = Rot_tar_inv * diff;
        // compute the position of the target when the object will be engaged
        pos eng1_pos = eng1->getPos(); // position of the engage point of the other object
        pos new_tar;
        new_tar.Xpos=eng1_pos.Xpos - eng_to_tar(0);
        new_tar.Ypos=eng1_pos.Ypos - eng_to_tar(1);
        new_tar.Zpos=eng1_pos.Zpos - eng_to_tar(2);

        place_location.push_back(new_tar.Xpos/1000);
        place_location.push_back(new_tar.Ypos/1000);
        place_location.push_back(new_tar.Zpos/1000);
        place_location.push_back(eng1->getOr().roll);
        place_location.push_back(eng1->getOr().pitch);
        place_location.push_back(eng1->getOr().yaw);

        target = {tar->getPos().Xpos/1000, tar->getPos().Ypos/1000, tar->getPos().Zpos/1000,
                  tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};

        // movement settings
        params.dHO = dHO/1000;
        params.griptype = this->mov->getGrip();
        params.obj_name = this->mov->getObject()->getName();
    }

    // movement settings
    params.arm_code = arm_code;
    params.hand_code = hand_code;
    params.mov_infoline = this->mov->getInfoLine();
    params.finalHand = finalHand;

    moveit_planning::PlanningResultPtr res;
    switch(mov_type){
    case 0:// reach-to-grasp
        params.target = target;
        res =  this->m_planner->pick(params);
        break;
    case 1:// reaching
        break;
    case 2://transport
        break;
    case 3://engage
        params.support_surface = obj_eng->getName();
        params.target = place_location;
        res =  this->m_planner->place(params);
        break;
    case 4:// disengage
        break;
    case 5:// go-park
        res = this->m_planner->move(params,homePosture);
        break;
    }
    this->m_params = params;
    if(res->status==1){this->solved=true;}

    return res;


}


}// motion_manager
