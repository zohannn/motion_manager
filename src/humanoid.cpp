#include "../include/motion_manager/humanoid.hpp"


namespace motion_manager{

#if HAND==0

Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs)
{

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    this->m_human_hand_specs = hspecs;

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);


    this->mat_right = Matrix4d::Constant(1);
    this->mat_left = Matrix4d::Constant(1);
    this->mat_r_hand = Matrix4d::Constant(1);
    this->mat_l_hand = Matrix4d::Constant(1);



}


Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
                   vector<double> &r, vector<double> &l)
{

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    this->m_human_hand_specs = hspecs;

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Constant(1);
    this->mat_left = Matrix4d::Constant(1);
    this->mat_r_hand = Matrix4d::Constant(1);
    this->mat_l_hand = Matrix4d::Constant(1);


}


Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
                   vector<double> &r, vector<double> &l,
                   vector<double> &min_rl, vector<double> &max_rl,
                   vector<double> &min_ll, vector<double> &max_ll)
{

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    this->m_human_hand_specs = hspecs;

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Constant(1);
    this->mat_left = Matrix4d::Constant(1);
    this->mat_r_hand = Matrix4d::Constant(1);
    this->mat_l_hand = Matrix4d::Constant(1);

}

#elif HAND==1

Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs)
{

        this->m_name = name;
        this->m_torso_pos = ppos;
        this->m_torso_or = oor;
        this->m_torso_size = ssize;
        this->m_arm_specs = aspecs;

        this->m_barrett_hand_specs = hspecs;

        this->rk.push_back(-1.0);
        this->rk.push_back(1.0);
        this->rk.push_back(0.0);

        this->jk.push_back(-1.0);
        this->jk.push_back(-1.0);
        this->jk.push_back(1.0);

        this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

        this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

        this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

        this->mat_right = Matrix4d::Identity(4,4);
        this->mat_left = Matrix4d::Identity(4,4);
        this->mat_r_hand = Matrix4d::Identity(4,4);
        this->mat_l_hand = Matrix4d::Identity(4,4);

        // logging
        init();
        logging::add_common_attributes();

}

Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
                   vector<double>& r, vector<double>& l)
{

        this->m_name = name;
        this->m_torso_pos = ppos;
        this->m_torso_or = oor;
        this->m_torso_size = ssize;
        this->m_arm_specs = aspecs;

        this->m_barrett_hand_specs = hspecs;

        this->rk.push_back(-1.0);
        this->rk.push_back(1.0);
        this->rk.push_back(0.0);

        this->jk.push_back(-1.0);
        this->jk.push_back(-1.0);
        this->jk.push_back(1.0);

        this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

        std::copy(r.begin(),r.end(),this->rightPosture.begin());
        std::copy(l.begin(),l.end(),this->leftPosture.begin());
        std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
        std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

        this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

        this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
        this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

        this->mat_right = Matrix4d::Identity(4,4);
        this->mat_left = Matrix4d::Identity(4,4);
        this->mat_r_hand = Matrix4d::Identity(4,4);
        this->mat_l_hand = Matrix4d::Identity(4,4);

        // logging
        init();
        logging::add_common_attributes();

}

Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
                   vector<double> &r, vector<double> &l,
                   vector<double> &min_rl, vector<double> &max_rl,
                   vector<double> &min_ll, vector<double> &max_ll)
{


    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;


    this->m_barrett_hand_specs = hspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);


    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);

    // logging
    init();
    logging::add_common_attributes();

}
#endif

Humanoid::Humanoid(const Humanoid &hh)
{

    this->m_name = hh.m_name;
    this->m_torso_pos = hh.m_torso_pos;
    this->m_torso_or = hh.m_torso_or;
    this->m_torso_size = hh.m_torso_size;
    this->m_arm_specs = hh.m_arm_specs;
#if HAND==0
    this->m_human_hand_specs = hh.m_human_hand_specs;
#elif HAND==1
    this->m_barrett_hand_specs = hh.m_barrett_hand_specs;
    this->rk = hh.rk;
    this->jk = hh.jk;
#endif
    this->m_DH_rightArm=hh.m_DH_rightArm;
    this->m_DH_leftArm=hh.m_DH_leftArm;
    this->m_DH_rightHand = hh.m_DH_rightHand;
    this->m_DH_leftHand = hh.m_DH_leftHand;

//#if HEAD==1
 //this->head=hh.head;
//#endif
//#if NECK==1
  //  this->neck=hh.neck;
//#endif
//#if PELVIS==1
  //  this->pelvis=hh.pelvis;
//#endif
//#if RIGHT_UPPER_LEG==1
  //  this->right_upper_leg=hh.right_upper_leg;
//#endif
//#if RIGHT_LOWER_LEG==1
  //  this->right_lower_leg=hh.right_lower_leg;
//#endif
//#if RIGHT_FOOT==1
  //  this->right_foot=hh.right_foot;
//#endif
//#if LEFT_UPPER_LEG==1
  //  this->left_upper_leg=hh.left_upper_leg;
//#endif
//#if LEFT_LOWER_LEG==1
  //  this->left_lower_leg=hh.left_lower_leg;
//#endif
//#if LEFT_FOOT==1
  //  this->left_foot=hh.left_foot;
//#endif

    this->mat_right = hh.mat_right;
    this->mat_left = hh.mat_left;
    this->mat_r_hand = hh.mat_r_hand;
    this->mat_l_hand = hh.mat_l_hand;

    this->max_rightLimits = hh.max_rightLimits;
    this->min_rightLimits = hh.min_rightLimits;
    this->max_leftLimits = hh.max_leftLimits;
    this->min_leftLimits=hh.min_leftLimits;
    this->rightPosture = hh.rightPosture;
    this->leftPosture = hh.leftPosture;
    this->rightVelocities = hh.rightVelocities;
    this->leftVelocities = hh.leftVelocities;
    this->rightForces = hh.rightForces;
    this->leftForces = hh.leftForces;
    this->rightHomePosture = hh.rightHomePosture;
    this->leftHomePosture = hh.leftHomePosture;

    this->rightShoulderPos=hh.rightShoulderPos;
    this->rightShoulderOr=hh.rightShoulderOr;
    this->rightElbowPos=hh.rightElbowPos;
    this->rightElbowOr=hh.rightElbowOr;
    this->rightWristPos=hh.rightWristPos;
    this->rightWristOr=hh.rightWristOr;
    this->rightHandPos=hh.rightHandPos;
    this->rightHandOr=hh.rightHandOr;
    this->rightFingers=hh.rightFingers;

    this->leftShoulderPos=hh.leftShoulderPos;
    this->leftShoulderOr=hh.leftShoulderOr;
    this->leftElbowPos=hh.leftElbowPos;
    this->leftElbowOr=hh.leftElbowOr;
    this->leftWristPos=hh.leftWristPos;
    this->leftWristOr=hh.leftWristOr;
    this->leftHandPos=hh.leftHandPos;
    this->leftHandOr=hh.leftHandOr;
    this->leftFingers=hh.leftFingers;

    this->rightHandPos_mes=hh.rightHandPos_mes;
    this->rightHandVel_mes=hh.rightHandVel_mes;
    this->leftHandPos_mes=hh.leftHandPos_mes;
    this->leftHandVel_mes=hh.leftHandVel_mes;

}

void Humanoid::init()
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


Humanoid::~Humanoid()
{


}

void Humanoid::setName(string& name)
{

    this->m_name = name;

}

void Humanoid::setPos(pos& ppos)
{

    this->m_torso_pos = ppos;
}


void Humanoid::setOr(orient& oor)
{

    this->m_torso_or = oor;
}

void Humanoid::setSize(dim& ssize)
{

    this->m_torso_size = ssize;
}

void Humanoid::setArm(arm& specs)
{

    this->m_arm_specs = specs;
}

#if HAND==0

void Humanoid::setHumanHand(human_hand &specs)
{

    this->m_human_hand_specs=specs;
}
#elif HAND==1


void Humanoid::setBarrettHand(barrett_hand& specs)
{

    this->m_barrett_hand_specs=specs;
}

#endif


void Humanoid::setRightPosture(vector<double> &r)
{

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
}

void Humanoid::setLeftPosture(vector<double> &l)
{

    std::copy(l.begin(),l.end(),this->leftPosture.begin());
}


void Humanoid::setRightHomePosture(vector<double> &r)
{

    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
}

void Humanoid::setLeftHomePosture(vector<double> &l)
{

    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());
}

void Humanoid::setRightMinLimits(vector<double> &min_rl)
{

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
}

void Humanoid::setRightMaxLimits(vector<double> &max_rl)
{

    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
}

void Humanoid::setLeftMinLimits(vector<double> &min_ll)
{

    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
}

void Humanoid::setLeftMaxLimits(vector<double> &max_ll)
{

    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());
}

void Humanoid::setRightVelocities(vector<double> &r)
{

    std::copy(r.begin(),r.end(),this->rightVelocities.begin());
}

void Humanoid::setLeftVelocities(vector<double> &l)
{

    std::copy(l.begin(),l.end(),this->leftVelocities.begin());
}


void Humanoid::setRightForces(vector<double> &r)
{

    std::copy(r.begin(),r.end(),this->rightForces.begin());
}


void Humanoid::setLeftForces(vector<double> &l)
{

    std::copy(l.begin(),l.end(),this->leftForces.begin());

}

void Humanoid::setMatRight(Matrix4d &m)
{

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_right(i, j) = m(i,j);
        }
    }

}

void Humanoid::setMatLeft(Matrix4d &m)
{


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_left(i, j) = m(i,j);
        }
    }
}


void Humanoid::setMatRightHand(Matrix4d &m)
{

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_r_hand(i, j) = m(i,j);
        }
    }
}


void Humanoid::setMatLeftHand(Matrix4d &m)
{

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_l_hand(i, j) = m(i,j);
        }
    }
}
// humanoid parts

//#if HEAD==1

//void Humanoid::setHead(humanoid_part& head){

  //  this->head=head;
//}
//#endif

//#if NECK==1

//void Humanoid::setNeck(humanoid_part& neck){

  //  this->neck=neck;
//}
//#endif

//#if PELVIS==1

//void Humanoid::setPelvis(humanoid_part& pelvis){

  //  this->pelvis=pelvis;

//}
//#endif

//#if RIGHT_UPPER_LEG==1

//void Humanoid::setRight_Upper_leg(humanoid_part& right_upper_leg){

  //  this->right_upper_leg=right_upper_leg;
//}
//#endif

//#if RIGHT_LOWER_LEG==1

//void Humanoid::setRight_Lower_leg(humanoid_part& right_lower_leg){

  //  this->right_lower_leg=right_lower_leg;
//}
//#endif

//#if RIGHT_FOOT==1

//void Humanoid::setRight_foot(humanoid_part& right_foot){

  //  this->right_foot=right_foot;
//}
//#endif

//#if LEFT_UPPER_LEG==1

//void Humanoid::setLeft_Upper_leg(humanoid_part& left_upper_leg){

  //  this->left_upper_leg=left_upper_leg;
//}
//#endif

//#if LEFT_LOWER_LEG==1

//void Humanoid::setLeft_Lower_leg(humanoid_part& left_lower_leg){

  //  this->left_lower_leg=left_lower_leg;
//}
//#endif

//#if LEFT_FOOT==1

//void Humanoid::setLeft_foot(humanoid_part& left_foot){

  //  this->left_foot=left_foot;
//}
//#endif


string Humanoid::getName()
{

    return this->m_name;
}

pos Humanoid::getPos()
{

    return this->m_torso_pos;
}


orient Humanoid::getOr()
{

    return this->m_torso_or;
}

dim Humanoid::getSize()
{

    return this->m_torso_size;
}

#if HAND ==1

void Humanoid::getRK(vector<int> &rkk)
{

    rkk = this->rk;
    /*
    for (std::size_t i=0; i < this->rk.size(); ++i){
        rkk.push_back(this->rk.at(i));
    }
    */

}

void Humanoid::getJK(vector<int> &jkk){

    jkk = jk;
    /*
    for(std::size_t i=0; i <this->jk.size(); ++i){
        jkk.push_back(this->jk.at(i));
    }
    */

}

#endif


arm Humanoid::getArm()
{

    return this->m_arm_specs;
}

#if HAND==0
human_hand Humanoid::getHumanHand()
{

    return this->m_human_hand_specs;
}
#elif HAND==1

barrett_hand Humanoid::getBarrettHand()
{

    return this->m_barrett_hand_specs;
}
#endif


void Humanoid::getRightPosture(vector<double>& p)
{

    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightPosture.begin(),this->rightPosture.end(),p.begin());

}

void Humanoid::getRightArmPosture(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);

    std::copy(this->rightPosture.begin(),this->rightPosture.end()-JOINTS_HAND,p.begin());

}


void Humanoid::getRightHandPosture(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

    std::copy(this->rightPosture.begin()+JOINTS_ARM,this->rightPosture.end(),p.begin());

}


void Humanoid::getLeftPosture(vector<double>& p)
{

    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftPosture.begin(),this->leftPosture.end(),p.begin());

}

void Humanoid::getLeftArmPosture(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);

    std::copy(this->leftPosture.begin(),this->leftPosture.end()-JOINTS_HAND,p.begin());

}

void Humanoid::getLeftHandPosture(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

    std::copy(this->leftPosture.begin()+JOINTS_ARM,this->leftPosture.end(),p.begin());

}


void Humanoid::getRightHomePosture(vector<double>& p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightHomePosture.begin(),this->rightHomePosture.end(),p.begin());

}

void Humanoid::getRightHandHomePosture(vector<double> &p)
{
   p = vector<double>(JOINTS_HAND);
   std::copy(this->rightHomePosture.begin()+JOINTS_ARM,this->rightHomePosture.end(),p.begin());
}

void Humanoid::getRightArmHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);
    std::copy(this->rightHomePosture.begin(),this->rightHomePosture.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHandHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);
    std::copy(this->leftHomePosture.begin()+JOINTS_ARM,this->leftHomePosture.end(),p.begin());
}

void Humanoid::getLeftArmHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);
    std::copy(this->leftHomePosture.begin(),this->leftHomePosture.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHomePosture(vector<double>& p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftHomePosture.begin(),this->leftHomePosture.end(),p.begin());

}


void Humanoid::getRightMinLimits(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_rightLimits.begin(),this->min_rightLimits.end(),p.begin());

}

void Humanoid::getRightMaxLimits(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_rightLimits.begin(),this->max_rightLimits.end(),p.begin());


}

void Humanoid::getLeftMinLimits(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_leftLimits.begin(),this->min_leftLimits.end(),p.begin());


}

void Humanoid::getLeftMaxLimits(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_leftLimits.begin(),this->max_leftLimits.end(),p.begin());

}

void Humanoid::getRightVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightVelocities.begin(),this->rightVelocities.end(),p.begin());
}

void Humanoid::getRightArmVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);

     std::copy(this->rightVelocities.begin(),this->rightVelocities.end()-JOINTS_HAND,p.begin());

}

void Humanoid::getRightHandVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

     std::copy(this->rightVelocities.begin()+JOINTS_ARM,this->rightVelocities.end(),p.begin());
}

void Humanoid::getLeftVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftVelocities.begin(),this->leftVelocities.end(),p.begin());
}

void Humanoid::getLeftArmVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);

     std::copy(this->leftVelocities.begin(),this->leftVelocities.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHandVelocities(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

     std::copy(this->leftVelocities.begin()+JOINTS_ARM,this->leftVelocities.end(),p.begin());
}

void Humanoid::getRightForces(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightForces.begin(),this->rightForces.end(),p.begin());
}

void Humanoid::getRightArmForces(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);

     std::copy(this->rightForces.begin(),this->rightForces.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getRightHandForces(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

     std::copy(this->rightForces.begin()+JOINTS_ARM,this->rightForces.end(),p.begin());
}

void Humanoid::getLeftForces(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM+JOINTS_HAND);
    std::copy(this->leftForces.begin(),this->leftForces.end(),p.begin());
}

void Humanoid::getLeftArmForces(vector<double> &p)
{

     p = vector<double>(JOINTS_ARM);
     std::copy(this->leftForces.begin(),this->leftForces.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHandForces(vector<double> &p)
{

     p = vector<double>(JOINTS_HAND);

     std::copy(this->leftForces.begin()+JOINTS_ARM,this->leftForces.end(),p.begin());
}

void Humanoid::getMatRight(Matrix4d &m)
{


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_right(i, j);
        }
    }
}

void Humanoid::getMatLeft(Matrix4d &m)
{

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_left(i, j);
        }
    }
}

void Humanoid::getMatRightHand(Matrix4d &m)
{


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_r_hand(i, j);
        }
    }
}

void Humanoid::getMatLeftHand(Matrix4d &m)
{

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_l_hand(i, j);
        }
    }
}


string Humanoid::getInfoLine()
{

    return  this->m_name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % this->m_torso_pos.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % this->m_torso_pos.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % this->m_torso_pos.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % this->m_torso_or.roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % this->m_torso_or.pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % this->m_torso_or.yaw) + RAD + SEP+
            XsizeSTR + str(boost::format("%d") % this->m_torso_size.Xsize) + MILLIMETERS + SEP+
            YsizeSTR + str(boost::format("%d") % this->m_torso_size.Ysize) + MILLIMETERS + SEP+
            ZsizeSTR + str(boost::format("%d") % this->m_torso_size.Zsize)+ MILLIMETERS;
            //L1STR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(0))+ MILLIMETERS + SEP+
            //LuSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(2))+ MILLIMETERS + SEP+
            //LlSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(4)) + MILLIMETERS + SEP+
            //LhSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(6)) + MILLIMETERS;

}

DHparams Humanoid::getDH_rightArm()
{
    this->computeRightArmDHparams();
    return this->m_DH_rightArm;
}

DHparams Humanoid::getDH_leftArm()
{
    this->computeLeftArmDHparams();
    return this->m_DH_leftArm;
}


void Humanoid::getRightShoulderPos(vector<double> &pos)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightShoulderPos;
}

double Humanoid::getRightShoulderNorm()
{

    vector<double> pos;

    this->getRightShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));

}


void Humanoid::getRightShoulderOr(Matrix3d &orr)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr=rightShoulderOr;

}

void Humanoid::getRightElbowPos(vector<double> &pos)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightElbowPos;
}

double Humanoid::getRightElbowNorm()
{

    vector<double> pos;

    this->getRightElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));


}


void Humanoid::getRightElbowOr(Matrix3d &orr)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightElbowOr;
}


void Humanoid::getRightWristPos(vector<double> &pos)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightWristPos;
}

double Humanoid::getRightWristNorm()
{

    vector<double> pos;

    this->getRightWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getRightWristOr(Matrix3d &orr)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightWristOr;
}

void Humanoid::getRightHandPos(vector<double> &pos)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightHandPos;
}



double Humanoid::getRightHandNorm()
{

    vector<double> pos;

    this->getRightHandPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getRightHandOr(Matrix3d &orr)
{

    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightHandOr;
}

void Humanoid::getRightHandVel(vector<double> &vel)
{
    std::vector<double> posture; std::vector<double> velocities;
    this->getRightArmPosture(posture); this->getRightArmVelocities(velocities);
    directDiffKinematicsSingleArm(1,posture,velocities,vel,3);
}

double Humanoid::getRightHandVelNorm()
{
    std::vector<double> hand_vel;

    this->getRightHandVel(hand_vel);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}

void Humanoid::getLeftShoulderPos(vector<double> &pos)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftShoulderPos;
}


double Humanoid::getLeftShoulderNorm()
{

    vector<double> pos;

    this->getLeftShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getLeftShoulderOr(Matrix3d &orr)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftShoulderOr;
}


void Humanoid::getLeftElbowPos(vector<double> &pos)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftElbowPos;

}

double Humanoid::getLeftElbowNorm()
{

    vector<double> pos;

    this->getLeftElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getLeftElbowOr(Matrix3d &orr)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftElbowOr;
}


void Humanoid::getLeftWristPos(vector<double> &pos)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftWristPos;
}

double Humanoid::getLeftWristNorm()
{

    vector<double> pos;

    this->getLeftWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getLeftWristOr(Matrix3d &orr)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftWristOr;
}


void Humanoid::getLeftHandPos(vector<double> &pos)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftHandPos;
}

double Humanoid::getLeftHandNorm()
{

    vector<double> pos;

    this->getLeftHandPos(pos);

    pos = leftHandPos;

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Humanoid::getLeftHandOr(Matrix3d &orr)
{

    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftHandOr;
}

void Humanoid::getLeftHandVel(vector<double> &vel)
{
    std::vector<double> posture; std::vector<double> velocities;
    this->getLeftArmPosture(posture); this->getLeftArmVelocities(velocities);
    directDiffKinematicsSingleArm(2,posture,velocities,vel,3);
}

double Humanoid::getLeftHandVelNorm()
{
    std::vector<double> hand_vel;

    this->getLeftHandVel(hand_vel);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}

void Humanoid::getAllPos(int arm, vector<double> &hand_pos, vector<double> &wrist_pos, vector<double> &elbow_pos, vector<double> &shoulder_pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    shoulder_pos.clear();
    Matrix3d shoulderOr; vector<double> s_rpy;
    elbow_pos.clear();
    Matrix3d elbowOr; vector<double> e_rpy;
    wrist_pos.clear();
    Matrix3d wristOr; vector<double> w_rpy;
    hand_pos.clear();
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulder_pos.push_back(v[0]);
            shoulder_pos.push_back(v[1]);
            shoulder_pos.push_back(v[2]);
            shoulder_pos.push_back(s_rpy.at(0));
            shoulder_pos.push_back(s_rpy.at(1));
            shoulder_pos.push_back(s_rpy.at(2));
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbow_pos.push_back(v[0]);
            elbow_pos.push_back(v[1]);
            elbow_pos.push_back(v[2]);
            elbow_pos.push_back(e_rpy.at(0));
            elbow_pos.push_back(e_rpy.at(1));
            elbow_pos.push_back(e_rpy.at(2));
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wrist_pos.push_back(v[0]);
            wrist_pos.push_back(v[1]);
            wrist_pos.push_back(v[2]);
            wrist_pos.push_back(w_rpy.at(0));
            wrist_pos.push_back(w_rpy.at(1));
            wrist_pos.push_back(w_rpy.at(2));
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            hand_pos.push_back(v[0]);
            hand_pos.push_back(v[1]);
            hand_pos.push_back(v[2]);
            hand_pos.push_back(h_rpy.at(0));
            hand_pos.push_back(h_rpy.at(1));
            hand_pos.push_back(h_rpy.at(2));
        }
    }
}

void Humanoid::getAllPos_q(int arm, vector<double> &hand_pos, vector<double> &wrist_pos, vector<double> &elbow_pos, vector<double> &shoulder_pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    shoulder_pos.clear();
    Matrix3d shoulderOr; vector<double> s_q;
    elbow_pos.clear();
    Matrix3d elbowOr; vector<double> e_q;
    wrist_pos.clear();
    Matrix3d wristOr; vector<double> w_q;
    hand_pos.clear();
    Matrix3d handOr; vector<double> h_q;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            //this->getRPY(s_rpy,shoulderOr);
            this->getQuaternion(s_q,shoulderOr);
            v = T.block(0,3,3,1);
            shoulder_pos.push_back(v[0]);
            shoulder_pos.push_back(v[1]);
            shoulder_pos.push_back(v[2]);
            shoulder_pos.push_back(s_q.at(0));
            shoulder_pos.push_back(s_q.at(1));
            shoulder_pos.push_back(s_q.at(2));
            shoulder_pos.push_back(s_q.at(3));
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            //this->getRPY(e_rpy,elbowOr);
            this->getQuaternion(e_q,elbowOr);
            v = T.block(0,3,3,1);
            elbow_pos.push_back(v[0]);
            elbow_pos.push_back(v[1]);
            elbow_pos.push_back(v[2]);
            elbow_pos.push_back(e_q.at(0));
            elbow_pos.push_back(e_q.at(1));
            elbow_pos.push_back(e_q.at(2));
            elbow_pos.push_back(e_q.at(3));
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            //this->getRPY(w_rpy,wristOr);
            this->getQuaternion(w_q,wristOr);
            v = T.block(0,3,3,1);
            wrist_pos.push_back(v[0]);
            wrist_pos.push_back(v[1]);
            wrist_pos.push_back(v[2]);
            wrist_pos.push_back(w_q.at(0));
            wrist_pos.push_back(w_q.at(1));
            wrist_pos.push_back(w_q.at(2));
            wrist_pos.push_back(w_q.at(3));
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            //this->getRPY(h_rpy,handOr);
            this->getQuaternion(h_q,handOr);
            v = T.block(0,3,3,1);
            hand_pos.push_back(v[0]);
            hand_pos.push_back(v[1]);
            hand_pos.push_back(v[2]);
            hand_pos.push_back(h_q.at(0));
            hand_pos.push_back(h_q.at(1));
            hand_pos.push_back(h_q.at(2));
            hand_pos.push_back(h_q.at(3));
        }
    }
}

void Humanoid::getHandPos(int arm, vector<double> &pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    pos.clear();
    pos.push_back(handPos[0]);
    pos.push_back(handPos[1]);
    pos.push_back(handPos[2]);

}

void Humanoid::getHandOr(int arm, vector<double>& orr, vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    orr.clear();
    orr.push_back(handPos[3]);
    orr.push_back(handPos[4]);
    orr.push_back(handPos[5]);
}

void Humanoid::getHandOr_q(int arm, vector<double>& orr_q, vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(7);
    Matrix3d shoulderOr; vector<double> s_q;
    vector<double> elbowPos = vector<double>(7);
    Matrix3d elbowOr; vector<double> e_q;
    vector<double> wristPos = vector<double>(7);
    Matrix3d wristOr; vector<double> w_q;
    vector<double> handPos = vector<double>(7);
    Matrix3d handOr; vector<double> h_q;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            //this->getRPY(s_rpy,shoulderOr);
            this->getQuaternion(s_q,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_q.at(0);
            shoulderPos[4] = s_q.at(1);
            shoulderPos[5] = s_q.at(2);
            shoulderPos[6] = s_q.at(3);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            //this->getRPY(e_rpy,elbowOr);
            this->getQuaternion(e_q,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_q.at(0);
            elbowPos[4] = e_q.at(1);
            elbowPos[5] = e_q.at(2);
            elbowPos[6] = e_q.at(3);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            //this->getRPY(w_rpy,wristOr);
            this->getQuaternion(w_q,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_q.at(0);
            wristPos[4] = w_q.at(1);
            wristPos[5] = w_q.at(2);
            wristPos[6] = w_q.at(3);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            //this->getRPY(h_rpy,handOr);
            this->getQuaternion(h_q,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_q.at(0);
            handPos[4] = h_q.at(1);
            handPos[5] = h_q.at(2);
            handPos[6] = h_q.at(3);
        }

    }

    orr_q.clear();
    orr_q.push_back(handPos[3]);
    orr_q.push_back(handPos[4]);
    orr_q.push_back(handPos[5]);
    orr_q.push_back(handPos[6]);
}

void Humanoid::getHandPosMes(int arm, vector<double>& ppos)
{
   switch(arm){
   case 1: // right arm
       ppos = this->rightHandPos_mes;
       break;
   case 2: // left arm
       ppos = this->leftHandPos_mes;
       break;
   }
}

double Humanoid::getSwivelAngle(int arm, vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    Vector3d shoulderPos;
    Vector3d elbowPos;
    Vector3d wristPos;
    Vector3d handPos;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            v = T.block(0,3,3,1);
            shoulderPos << v[0], v[1], v[2];
        }else if (i==2){
            // get the elbow
            v = T.block(0,3,3,1);
            elbowPos << v[0], v[1], v[2];
        }else if (i==4){
            // get the wrist
            v = T.block(0,3,3,1);
            wristPos << v[0], v[1], v[2];
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            v = T.block(0,3,3,1);
            handPos << v[0], v[1], v[2];
        }
    }

    double Lu = m_DH_arm.d.at(2); // length of the upper arm
    Vector3d v_SE = (shoulderPos-elbowPos)/((shoulderPos-elbowPos).norm());
    Vector3d v_SW = (shoulderPos-wristPos)/((shoulderPos-wristPos).norm());
    Vector3d C = shoulderPos + Lu*(v_SW.dot(v_SE))*v_SW;
    Vector3d v_CE = (C-elbowPos)/((C-elbowPos).norm());
    Vector3d u; u << v_SW(1), -v_SW(0), 0.0; u = u/u.norm();
    Vector3d v; v = u.cross(v_SW);
    double alpha = atan2(v_CE.dot(v),v_CE.dot(u));

    return alpha;
}

double Humanoid::getSwivelAngle(int arm)
{

    std::vector<double> posture;
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    Vector3d shoulderPos;
    Vector3d elbowPos;
    Vector3d wristPos;
    Vector3d handPos;

    switch(arm){
    case 1: // right arm
        this->getRightArmPosture(posture);
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: // left arm
        this->getLeftArmPosture(posture);
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            v = T.block(0,3,3,1);
            shoulderPos << v[0], v[1], v[2];
        }else if (i==2){
            // get the elbow
            v = T.block(0,3,3,1);
            elbowPos << v[0], v[1], v[2];
        }else if (i==4){
            // get the wrist
            v = T.block(0,3,3,1);
            wristPos << v[0], v[1], v[2];
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            v = T.block(0,3,3,1);
            handPos << v[0], v[1], v[2];
        }
    }

    double Lu = m_DH_arm.d.at(2); // length of the upper arm
    Vector3d v_SE = (shoulderPos-elbowPos)/((shoulderPos-elbowPos).norm());
    Vector3d v_SW = (shoulderPos-wristPos)/((shoulderPos-wristPos).norm());
    Vector3d C = shoulderPos + Lu*(v_SW.dot(v_SE))*v_SW;
    Vector3d v_CE = (C-elbowPos)/((C-elbowPos).norm());
    Vector3d u; u << v_SW(1), -v_SW(0), 0.0; u = u/u.norm();
    Vector3d v; v = u.cross(v_SW);
    double alpha = atan2(v_CE.dot(v),v_CE.dot(u));

    return alpha;
}

void Humanoid::setHandPosMes(int arm, vector<double> &ppos)
{
    switch(arm){
    case 1: // right arm
        this->rightHandPos_mes = ppos;
        break;
    case 2: // left arm
        this->leftHandPos_mes = ppos;
        break;
    }
}

void Humanoid::getHandVelMes(int arm, vector<double> &vel)
{
    switch(arm){
    case 1: // right hand
        vel = this->rightHandVel_mes;
        break;
    case 2: // left hand
        vel = this->leftHandVel_mes;
        break;
    }
}

void Humanoid::setHandVelMes(int arm, vector<double> &vel)
{
    switch(arm){
    case 1: // right arm
        this->rightHandVel_mes = vel;
        break;
    case 2: // left arm
        this->leftHandVel_mes = vel;
        break;
    }
}

void Humanoid::getAllVel(int arm, vector<double> &hand_vel, vector<double> &wrist_vel, vector<double> &elbow_vel, vector<double> &shoulder_vel, vector<double> &posture, vector<double> &velocities)
{
    VectorXd joint_velocities;
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);
    joint_velocities.resize(velocities.size());

    for (int i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
        joint_velocities(i) = velocities.at(i);
    }

    MatrixXd Jac_tmp; VectorXd joint_vel_tmp;
    // shoulder velocity
    Jac_tmp = JacobianArm.block<6,2>(0,0);
    joint_vel_tmp = joint_velocities.block<2,1>(0,0);
    VectorXd shoulder_vel_xd = Jac_tmp*joint_vel_tmp;
    // elbow velocity
    Jac_tmp = JacobianArm.block<6,4>(0,0);
    joint_vel_tmp = joint_velocities.block<4,1>(0,0);
    VectorXd elbow_vel_xd = Jac_tmp*joint_vel_tmp;
    // wrist velocity
    Jac_tmp = JacobianArm.block<6,6>(0,0);
    joint_vel_tmp = joint_velocities.block<6,1>(0,0);
    VectorXd wrist_vel_xd = Jac_tmp*joint_vel_tmp;
    // hand velocity
    VectorXd hand_vel_xd = JacobianArm*joint_velocities;

    // shoulder
    shoulder_vel.clear();
    shoulder_vel.resize(shoulder_vel_xd.size());
    VectorXd::Map(&shoulder_vel[0], shoulder_vel_xd.size()) = shoulder_vel_xd;

    // elbow
    elbow_vel.clear();
    elbow_vel.resize(elbow_vel_xd.size());
    VectorXd::Map(&elbow_vel[0], elbow_vel_xd.size()) = elbow_vel_xd;

    // wrist
    wrist_vel.clear();
    wrist_vel.resize(wrist_vel_xd.size());
    VectorXd::Map(&wrist_vel[0], wrist_vel_xd.size()) = wrist_vel_xd;

    // hand
    hand_vel.clear();
    hand_vel.resize(hand_vel_xd.size());
    VectorXd::Map(&hand_vel[0], hand_vel_xd.size()) = hand_vel_xd;
}

void Humanoid::getHandVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,3);
}

double Humanoid::getHandVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> hand_vel;

    this->getHandVel(arm,hand_vel,posture,velocities);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}

void Humanoid::getWristPos(int arm, vector<double> &pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    pos.clear();
    pos.push_back(wristPos[0]);
    pos.push_back(wristPos[1]);
    pos.push_back(wristPos[2]);
}

void Humanoid::getWristOr(int arm, vector<double> &orr, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    orr.clear();
    orr.push_back(wristPos[3]);
    orr.push_back(wristPos[4]);
    orr.push_back(wristPos[5]);
}

void Humanoid::getWristOr_q(int arm,vector<double>& orr_q,vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(7);
    Matrix3d shoulderOr; vector<double> s_q;
    vector<double> elbowPos = vector<double>(7);
    Matrix3d elbowOr; vector<double> e_q;
    vector<double> wristPos = vector<double>(7);
    Matrix3d wristOr; vector<double> w_q;
    vector<double> handPos = vector<double>(7);
    Matrix3d handOr; vector<double> h_q;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            //this->getRPY(s_rpy,shoulderOr);
            this->getQuaternion(s_q,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_q.at(0);
            shoulderPos[4] = s_q.at(1);
            shoulderPos[5] = s_q.at(2);
            shoulderPos[6] = s_q.at(3);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            //this->getRPY(e_rpy,elbowOr);
            this->getQuaternion(e_q,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_q.at(0);
            elbowPos[4] = e_q.at(1);
            elbowPos[5] = e_q.at(2);
            elbowPos[6] = e_q.at(3);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            //this->getRPY(w_rpy,wristOr);
            this->getQuaternion(w_q,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_q.at(0);
            wristPos[4] = w_q.at(1);
            wristPos[5] = w_q.at(2);
            wristPos[6] = w_q.at(3);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            //this->getRPY(h_rpy,handOr);
            this->getQuaternion(h_q,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_q.at(0);
            handPos[4] = h_q.at(1);
            handPos[5] = h_q.at(2);
            handPos[6] = h_q.at(3);
        }

    }

    orr_q.clear();
    orr_q.push_back(wristPos[3]);
    orr_q.push_back(wristPos[4]);
    orr_q.push_back(wristPos[5]);
    orr_q.push_back(wristPos[6]);

}

void Humanoid::getWristVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,2);
}

double Humanoid::getWristVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> wrist_vel;

    this->getWristVel(arm,wrist_vel,posture,velocities);

    return sqrt(pow(wrist_vel.at(0),2)+pow(wrist_vel.at(1),2)+pow(wrist_vel.at(2),2));
}

void Humanoid::getElbowPos(int arm, vector<double> &pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    pos.clear();
    pos.push_back(elbowPos[0]);
    pos.push_back(elbowPos[1]);
    pos.push_back(elbowPos[2]);
}

void Humanoid::getElbowOr(int arm, vector<double> &orr, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    orr.clear();
    orr.push_back(elbowPos[3]);
    orr.push_back(elbowPos[4]);
    orr.push_back(elbowPos[5]);
}

void Humanoid::getElbowOr_q(int arm,vector<double>& orr_q,vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(7);
    Matrix3d shoulderOr; vector<double> s_q;
    vector<double> elbowPos = vector<double>(7);
    Matrix3d elbowOr; vector<double> e_q;
    vector<double> wristPos = vector<double>(7);
    Matrix3d wristOr; vector<double> w_q;
    vector<double> handPos = vector<double>(7);
    Matrix3d handOr; vector<double> h_q;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            //this->getRPY(s_rpy,shoulderOr);
            this->getQuaternion(s_q,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_q.at(0);
            shoulderPos[4] = s_q.at(1);
            shoulderPos[5] = s_q.at(2);
            shoulderPos[6] = s_q.at(3);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            //this->getRPY(e_rpy,elbowOr);
            this->getQuaternion(e_q,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_q.at(0);
            elbowPos[4] = e_q.at(1);
            elbowPos[5] = e_q.at(2);
            elbowPos[6] = e_q.at(3);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            //this->getRPY(w_rpy,wristOr);
            this->getQuaternion(w_q,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_q.at(0);
            wristPos[4] = w_q.at(1);
            wristPos[5] = w_q.at(2);
            wristPos[6] = w_q.at(3);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            //this->getRPY(h_rpy,handOr);
            this->getQuaternion(h_q,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_q.at(0);
            handPos[4] = h_q.at(1);
            handPos[5] = h_q.at(2);
            handPos[6] = h_q.at(3);
        }

    }

    orr_q.clear();
    orr_q.push_back(elbowPos[3]);
    orr_q.push_back(elbowPos[4]);
    orr_q.push_back(elbowPos[5]);
    orr_q.push_back(elbowPos[6]);
}

void Humanoid::getElbowVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,1);
}

double Humanoid::getElbowVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> elbow_vel;

    this->getElbowVel(arm,elbow_vel,posture,velocities);

    return sqrt(pow(elbow_vel.at(0),2)+pow(elbow_vel.at(1),2)+pow(elbow_vel.at(2),2));
}


void Humanoid::getShoulderPos(int arm, vector<double> &pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    pos.clear();
    pos.push_back(shoulderPos[0]);
    pos.push_back(shoulderPos[1]);
    pos.push_back(shoulderPos[2]);
}

void Humanoid::getShoulderOr(int arm, vector<double> &orr, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(6);
    Matrix3d shoulderOr; vector<double> s_rpy;
    vector<double> elbowPos = vector<double>(6);
    Matrix3d elbowOr; vector<double> e_rpy;
    vector<double> wristPos = vector<double>(6);
    Matrix3d wristOr; vector<double> w_rpy;
    vector<double> handPos = vector<double>(6);
    Matrix3d handOr; vector<double> h_rpy;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            this->getRPY(s_rpy,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_rpy.at(0);
            shoulderPos[4] = s_rpy.at(1);
            shoulderPos[5] = s_rpy.at(2);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            this->getRPY(e_rpy,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_rpy.at(0);
            elbowPos[4] = e_rpy.at(1);
            elbowPos[5] = e_rpy.at(2);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            this->getRPY(w_rpy,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_rpy.at(0);
            wristPos[4] = w_rpy.at(1);
            wristPos[5] = w_rpy.at(2);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            this->getRPY(h_rpy,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_rpy.at(0);
            handPos[4] = h_rpy.at(1);
            handPos[5] = h_rpy.at(2);
        }

    }

    orr.clear();
    orr.push_back(shoulderPos[3]);
    orr.push_back(shoulderPos[4]);
    orr.push_back(shoulderPos[5]);
}

void Humanoid::getShoulderOr_q(int arm,vector<double>& orr_q,vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(7);
    Matrix3d shoulderOr; vector<double> s_q;
    vector<double> elbowPos = vector<double>(7);
    Matrix3d elbowOr; vector<double> e_q;
    vector<double> wristPos = vector<double>(7);
    Matrix3d wristOr; vector<double> w_q;
    vector<double> handPos = vector<double>(7);
    Matrix3d handOr; vector<double> h_q;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            //this->getRPY(s_rpy,shoulderOr);
            this->getQuaternion(s_q,shoulderOr);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            shoulderPos[3] = s_q.at(0);
            shoulderPos[4] = s_q.at(1);
            shoulderPos[5] = s_q.at(2);
            shoulderPos[6] = s_q.at(3);
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            //this->getRPY(e_rpy,elbowOr);
            this->getQuaternion(e_q,elbowOr);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            elbowPos[3] = e_q.at(0);
            elbowPos[4] = e_q.at(1);
            elbowPos[5] = e_q.at(2);
            elbowPos[6] = e_q.at(3);
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            //this->getRPY(w_rpy,wristOr);
            this->getQuaternion(w_q,wristOr);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            wristPos[3] = w_q.at(0);
            wristPos[4] = w_q.at(1);
            wristPos[5] = w_q.at(2);
            wristPos[6] = w_q.at(3);
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            //this->getRPY(h_rpy,handOr);
            this->getQuaternion(h_q,handOr);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            handPos[3] = h_q.at(0);
            handPos[4] = h_q.at(1);
            handPos[5] = h_q.at(2);
            handPos[6] = h_q.at(3);
        }

    }

    orr_q.clear();
    orr_q.push_back(shoulderPos[3]);
    orr_q.push_back(shoulderPos[4]);
    orr_q.push_back(shoulderPos[5]);
    orr_q.push_back(shoulderPos[6]);

}

void Humanoid::getShoulderVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,0);
}

double Humanoid::getShoulderVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> shoulder_vel;

    this->getShoulderVel(arm,shoulder_vel,posture,velocities);

    return sqrt(pow(shoulder_vel.at(0),2)+pow(shoulder_vel.at(1),2)+pow(shoulder_vel.at(2),2));
}

//#if HEAD==1

//humanoid_part Humanoid::getHead(){
  //  return this->head;
//}
//#endif

//#if NECK==1

//#endif

//#if PELVIS==1

//humanoid_part Humanoid::getPelvis(){

  //  return this->pelvis;
//}
//#endif

//#if RIGHT_UPPER_LEG==1

//humanoid_part Humanoid::getRight_Upper_leg(){

  //  return this->right_upper_leg;
//}
//#endif

//#if RIGHT_LOWER_LEG==1

//humanoid_part Humanoid::getRight_Lower_leg(){

  //  return this->right_lower_leg;
//}
//#endif

//#if RIGHT_FOOT==1

//humanoid_part Humanoid::getRight_foot(){

  //  return this->right_foot;
//}
//#endif

//#if LEFT_UPPER_LEG==1

//humanoid_part Humanoid::getLeft_Upper_leg(){

  //  return this->left_upper_leg;

//}
//#endif

//#if LEFT_LOWER_LEG==1

//humanoid_part Humanoid::getLeft_Lower_leg(){

  //  return this->left_lower_leg;

//}
//#endif

//#if LEFT_FOOT==1

//humanoid_part Humanoid::getLeft_foot(){

  //  return this->left_foot;

//}
//#endif


void Humanoid::directKinematicsDualArm()
{


    // direct kinematics of the right arm
    std::vector<double> rposture;
    this->getRightArmPosture(rposture);
    directKinematicsSingleArm(1,rposture);
    // direct kinematics of the left arm
    std::vector<double> lposture;
    this->getLeftArmPosture(lposture);
    directKinematicsSingleArm(2,lposture);

}

void Humanoid::computeRightArmDHparams()
{

    this->m_DH_rightArm.a.clear();
    this->m_DH_rightArm.d.clear();
    this->m_DH_rightArm.alpha.clear();
    this->m_DH_rightArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i){

    // d [mm]
    m_DH_rightArm.d.push_back(m_arm_specs.arm_specs.d.at(i));

    //a [mm]
    m_DH_rightArm.a.push_back(m_arm_specs.arm_specs.a.at(i));

    //alpha [rad]
    m_DH_rightArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));

    //theta [rad]
    m_DH_rightArm.theta.push_back(rightPosture.at(i));

    }


}

void Humanoid::computeLeftArmDHparams()
{

    this->m_DH_leftArm.a.clear();
    this->m_DH_leftArm.d.clear();
    this->m_DH_leftArm.alpha.clear();
    this->m_DH_leftArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i){

    // d [mm]
    //m_DH_leftArm.d.push_back(-m_arm_specs.arm_specs.d.at(i));
    m_DH_leftArm.d.push_back(m_arm_specs.arm_specs.d.at(i));

    //a [mm]
    m_DH_leftArm.a.push_back(m_arm_specs.arm_specs.a.at(i));

    //alpha [rad]
    /*
    if ((i == 0)){
        m_DH_leftArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));
    }else{
        m_DH_leftArm.alpha.push_back(-m_arm_specs.arm_specs.alpha.at(i));
    }
    */
    m_DH_leftArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));

    //theta [rad]
    m_DH_leftArm.theta.push_back(leftPosture.at(i));
    }


}

void Humanoid::computeRightHandDHparams()
{

    this->m_DH_rightHand.clear();

    for (int i = 0; i< HAND_FINGERS; ++i){

        vector<double> t;
        this->getRightHandPosture(t);

        DHparams f;
        vector<double> fing_pos;


#if HAND==0
        if (i==0){


            f.a = vector<double>(4);
            f.d = vector<double>(4);
            f.alpha = vector<double>(4);
            f.theta = vector<double>(4);
            human_finger fing = m_human_hand_specs.fingers.at(0); // index
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);


        }else if(i==1){
            f.a = vector<double>(4);
            f.d = vector<double>(4);
            f.alpha = vector<double>(4);
            f.theta = vector<double>(4);

            human_finger fing = m_human_hand_specs.fingers.at(2); // ring
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);

        }else if(i==2){

            f.a = vector<double>(5);
            f.d = vector<double>(5);
            f.alpha = vector<double>(5);
            f.theta = vector<double>(5);

            human_thumb thumb = m_human_hand_specs.thumb; // thumb
            // finger positions [mm]
            fing_pos.push_back(thumb.uTx);
            fing_pos.push_back(thumb.uTy);
            fing_pos.push_back(thumb.uTz);
            //a [mm]
            f.a.at(0) = thumb.thumb_specs.a.at(0);
            f.a.at(1) = thumb.thumb_specs.a.at(1);
            f.a.at(2) = thumb.thumb_specs.a.at(2);
            f.a.at(3) = thumb.thumb_specs.a.at(3);
            f.a.at(4) = thumb.thumb_specs.a.at(4);
            // d [mm]
            f.d.at(0) = thumb.thumb_specs.d.at(0);
            f.d.at(1) = thumb.thumb_specs.d.at(1);
            f.d.at(2) = thumb.thumb_specs.d.at(2);
            f.d.at(3) = thumb.thumb_specs.d.at(3);
            f.d.at(4) = thumb.thumb_specs.d.at(4);
            // alpha [rad]
            f.alpha.at(0) = thumb.thumb_specs.alpha.at(0);
            f.alpha.at(1) = thumb.thumb_specs.alpha.at(1);
            f.alpha.at(2) = thumb.thumb_specs.alpha.at(2);
            f.alpha.at(3) = thumb.thumb_specs.alpha.at(3);
            f.alpha.at(4) = thumb.thumb_specs.alpha.at(4);
            // theta [rad]
            f.theta.at(0) = thumb.thumb_specs.theta.at(0);
            f.theta.at(1) = thumb.thumb_specs.theta.at(1);
            f.theta.at(2) = thumb.thumb_specs.theta.at(2);
            f.theta.at(3) = thumb.thumb_specs.theta.at(3);
            f.theta.at(4) = thumb.thumb_specs.theta.at(4);



        }

#elif HAND==1

        f.a = vector<double>(4);
        f.d = vector<double>(4);
        f.alpha = vector<double>(4);
        f.theta = vector<double>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i)*(m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i)*t.at(0))-1.57*jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2+t.at(i+1);
        f.theta.at(2) = m_barrett_hand_specs.phi3+(1/3)*t.at(i+1);
        f.theta.at(3) = 0.0;

#endif

          m_DH_rightHand.push_back(f);
          right_fing_pos.push_back(fing_pos);

    }


}

void Humanoid::computeLeftHandDHparams()
{


    this->m_DH_leftHand.clear();


    for (int i = 0; i< HAND_FINGERS; ++i){

        DHparams f;
        vector<double> fing_pos;

        vector<double> t;
        this->getLeftHandPosture(t);


#if HAND==0
        if (i==0){

            f.a = vector<double>(4);
            f.d = vector<double>(4);
            f.alpha = vector<double>(4);
            f.theta = vector<double>(4);
            human_finger fing = m_human_hand_specs.fingers.at(0); // index
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);


        }else if(i==1){
            f.a = vector<double>(4);
            f.d = vector<double>(4);
            f.alpha = vector<double>(4);
            f.theta = vector<double>(4);

            human_finger fing = m_human_hand_specs.fingers.at(2); // ring
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);

        }else if(i==2){

            f.a = vector<double>(5);
            f.d = vector<double>(5);
            f.alpha = vector<double>(5);
            f.theta = vector<double>(5);

            human_thumb thumb = m_human_hand_specs.thumb; // thumb
            // finger positions [mm]
            fing_pos.push_back(thumb.uTx);
            fing_pos.push_back(thumb.uTy);
            fing_pos.push_back(thumb.uTz);
            //a [mm]
            f.a.at(0) = thumb.thumb_specs.a.at(0);
            f.a.at(1) = thumb.thumb_specs.a.at(1);
            f.a.at(2) = thumb.thumb_specs.a.at(2);
            f.a.at(3) = thumb.thumb_specs.a.at(3);
            f.a.at(4) = thumb.thumb_specs.a.at(4);
            // d [mm]
            f.d.at(0) = thumb.thumb_specs.d.at(0);
            f.d.at(1) = thumb.thumb_specs.d.at(1);
            f.d.at(2) = thumb.thumb_specs.d.at(2);
            f.d.at(3) = thumb.thumb_specs.d.at(3);
            f.d.at(4) = thumb.thumb_specs.d.at(4);
            // alpha [rad]
            f.alpha.at(0) = thumb.thumb_specs.alpha.at(0);
            f.alpha.at(1) = thumb.thumb_specs.alpha.at(1);
            f.alpha.at(2) = thumb.thumb_specs.alpha.at(2);
            f.alpha.at(3) = thumb.thumb_specs.alpha.at(3);
            f.alpha.at(4) = thumb.thumb_specs.alpha.at(4);
            // theta [rad]
            f.theta.at(0) = thumb.thumb_specs.theta.at(0);
            f.theta.at(1) = thumb.thumb_specs.theta.at(1);
            f.theta.at(2) = thumb.thumb_specs.theta.at(2);
            f.theta.at(3) = thumb.thumb_specs.theta.at(3);
            f.theta.at(4) = thumb.thumb_specs.theta.at(4);



        }

#elif HAND==1

        f.a = vector<double>(4);
        f.d = vector<double>(4);
        f.alpha = vector<double>(4);
        f.theta = vector<double>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i)*(m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i)*t.at(0))-1.57*jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2+t.at(i+1);
        f.theta.at(2) = m_barrett_hand_specs.phi3+(1/3)*t.at(i+1);
        f.theta.at(3) = 0.0;
#endif

        m_DH_leftHand.push_back(f);
        left_fing_pos.push_back(fing_pos);


    }


}

void Humanoid::getJacobian(int arm,std::vector<double>& posture,MatrixXd& Jacobian)
{

    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    Jacobian.resize(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i),posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        Jacobian.col(i) = column;
    }
}

void Humanoid::getTimeDerivativeJacobian(int arm,std::vector<double>& posture,std::vector<double>& velocities,MatrixXd& TimeDerivativeJacobian)
{

    TimeDerivativeJacobian.resize(6,JOINTS_ARM);
    VectorXd joint_velocities(7);joint_velocities << velocities.at(0),velocities.at(1),velocities.at(2),velocities.at(3),velocities.at(4),velocities.at(5),velocities.at(6);
    MatrixXd Jacobian; this->getJacobian(arm,posture,Jacobian);
    vector<double> handPos; Vector3d pos_hand; VectorXd pos_hand_vel(6);
    this->getHandPos(arm,handPos,posture);
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);
    pos_hand_vel = Jacobian*joint_velocities;
    Vector3d pos_hand_lin_vel = pos_hand_vel.block<3,1>(0,0);

    MatrixXd Jac_tmp; VectorXd joint_vel_tmp;
    // p0 velocity
    Jac_tmp = Jacobian.block<6,1>(0,0);
    joint_vel_tmp = joint_velocities.block<1,1>(0,0);
    VectorXd pos_0_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_0_lin_vel = pos_0_vel.block<3,1>(0,0);
    VectorXd pos_0_ang_vel = pos_0_vel.block<3,1>(3,0);
    Matrix3d S_0;
    S_0(0,0) = 0.0;                 S_0(0,1) = -pos_0_ang_vel(2); S_0(0,2) = pos_0_ang_vel(1);
    S_0(1,0) = pos_0_ang_vel(2);    S_0(1,1) = 0.0;               S_0(1,2) = -pos_0_ang_vel(0);
    S_0(2,0) = -pos_0_ang_vel(1);   S_0(2,1) = pos_0_ang_vel(0);  S_0(2,2) = 0.0;
    // p1 velocity
    Jac_tmp = Jacobian.block<6,2>(0,0);
    joint_vel_tmp = joint_velocities.block<2,1>(0,0);
    VectorXd pos_1_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_1_lin_vel = pos_1_vel.block<3,1>(0,0);
    VectorXd pos_1_ang_vel = pos_1_vel.block<3,1>(3,0);
    Matrix3d S_1;
    S_1(0,0) = 0.0;                 S_1(0,1) = -pos_1_ang_vel(2); S_1(0,2) = pos_1_ang_vel(1);
    S_1(1,0) = pos_1_ang_vel(2);    S_1(1,1) = 0.0;               S_1(1,2) = -pos_1_ang_vel(0);
    S_1(2,0) = -pos_1_ang_vel(1);   S_1(2,1) = pos_1_ang_vel(0);  S_1(2,2) = 0.0;
    // p2 velocity
    Jac_tmp = Jacobian.block<6,3>(0,0);
    joint_vel_tmp = joint_velocities.block<3,1>(0,0);
    VectorXd pos_2_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_2_lin_vel = pos_2_vel.block<3,1>(0,0);
    VectorXd pos_2_ang_vel = pos_2_vel.block<3,1>(3,0);
    Matrix3d S_2;
    S_2(0,0) = 0.0;                 S_2(0,1) = -pos_2_ang_vel(2); S_2(0,2) = pos_2_ang_vel(1);
    S_2(1,0) = pos_2_ang_vel(2);    S_2(1,1) = 0.0;               S_2(1,2) = -pos_2_ang_vel(0);
    S_2(2,0) = -pos_2_ang_vel(1);   S_2(2,1) = pos_2_ang_vel(0);  S_2(2,2) = 0.0;
    // p3 velocity
    Jac_tmp = Jacobian.block<6,4>(0,0);
    joint_vel_tmp = joint_velocities.block<4,1>(0,0);
    VectorXd pos_3_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_3_lin_vel = pos_3_vel.block<3,1>(0,0);
    VectorXd pos_3_ang_vel = pos_3_vel.block<3,1>(3,0);
    Matrix3d S_3;
    S_3(0,0) = 0.0;                 S_0(0,1) = -pos_3_ang_vel(2); S_3(0,2) = pos_3_ang_vel(1);
    S_3(1,0) = pos_3_ang_vel(2);    S_0(1,1) = 0.0;               S_3(1,2) = -pos_3_ang_vel(0);
    S_3(2,0) = -pos_3_ang_vel(1);   S_0(2,1) = pos_3_ang_vel(0);  S_3(2,2) = 0.0;
    // p4 velocity
    Jac_tmp = Jacobian.block<6,5>(0,0);
    joint_vel_tmp = joint_velocities.block<5,1>(0,0);
    VectorXd pos_4_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_4_lin_vel = pos_4_vel.block<3,1>(0,0);
    VectorXd pos_4_ang_vel = pos_4_vel.block<3,1>(3,0);
    Matrix3d S_4;
    S_4(0,0) = 0.0;                 S_4(0,1) = -pos_4_ang_vel(2); S_4(0,2) = pos_4_ang_vel(1);
    S_4(1,0) = pos_4_ang_vel(2);    S_4(1,1) = 0.0;               S_4(1,2) = -pos_4_ang_vel(0);
    S_4(2,0) = -pos_4_ang_vel(1);   S_4(2,1) = pos_4_ang_vel(0);  S_4(2,2) = 0.0;
    // p5 velocity
    Jac_tmp = Jacobian.block<6,6>(0,0);
    joint_vel_tmp = joint_velocities.block<6,1>(0,0);
    VectorXd pos_5_vel = Jac_tmp*joint_vel_tmp;
    VectorXd pos_5_lin_vel = pos_5_vel.block<3,1>(0,0);
    VectorXd pos_5_ang_vel = pos_5_vel.block<3,1>(3,0);
    Matrix3d S_5;
    S_5(0,0) = 0.0;                 S_5(0,1) = -pos_5_ang_vel(2); S_5(0,2) = pos_5_ang_vel(1);
    S_5(1,0) = pos_5_ang_vel(2);    S_5(1,1) = 0.0;               S_5(1,2) = -pos_5_ang_vel(0);
    S_5(2,0) = -pos_5_ang_vel(1);   S_5(2,1) = pos_5_ang_vel(0);  S_5(2,2) = 0.0;
    // p6 velocity
    VectorXd pos_6_vel = Jacobian*joint_velocities;
    VectorXd pos_6_lin_vel = pos_6_vel.block<3,1>(0,0);
    VectorXd pos_6_ang_vel = pos_6_vel.block<3,1>(3,0);
    Matrix3d S_6;
    S_6(0,0) = 0.0;                 S_6(0,1) = -pos_6_ang_vel(2); S_6(0,2) = pos_6_ang_vel(1);
    S_6(1,0) = pos_6_ang_vel(2);    S_6(1,1) = 0.0;               S_6(1,2) = -pos_6_ang_vel(0);
    S_6(2,0) = -pos_6_ang_vel(1);   S_6(2,1) = pos_6_ang_vel(0);  S_6(2,2) = 0.0;

    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    Vector3d pos0;
    Vector3d z0;
    Vector3d z0_der;
    Matrix3d Rot_0;
    Matrix3d Rot_der_0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d z1_der;
    Matrix3d Rot_1;
    Matrix3d Rot_der_1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d z2_der;
    Matrix3d Rot_2;
    Matrix3d Rot_der_2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d z3_der;
    Matrix3d Rot_3;
    Matrix3d Rot_der_3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d z4_der;
    Matrix3d Rot_4;
    Matrix3d Rot_der_4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d z5_der;
    Matrix3d Rot_5;
    Matrix3d Rot_der_5;
    Vector3d pos6;
    Vector3d z6;
    Vector3d z6_der;
    Matrix3d Rot_6;
    Matrix3d Rot_der_6;

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i),posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d diff_der;
        Vector3d cross;
        Vector3d zi_der;
        switch(i){
        case 0:
            Rot_0 = T.block(0,0,3,3);
            Rot_der_0 = S_0*Rot_0;
            z0_der = Rot_der_0.block(0,2,3,1);
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            diff_der = pos_hand_lin_vel - pos_0_lin_vel;
            cross = z0_der.cross(diff) + z0.cross(diff_der);
            zi_der=z0_der;
            break;
        case 1:
            Rot_1 = T.block(0,0,3,3);
            Rot_der_1 = S_1*Rot_1;
            z1_der = Rot_der_1.block(0,2,3,1);
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            diff_der = pos_hand_lin_vel - pos_1_lin_vel;
            cross = z1_der.cross(diff) + z1.cross(diff_der);
            zi_der=z1_der;
            break;
        case 2:
            Rot_2 = T.block(0,0,3,3);
            Rot_der_2 = S_2*Rot_2;
            z2_der = Rot_der_2.block(0,2,3,1);
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            diff_der = pos_hand_lin_vel - pos_2_lin_vel;
            cross = z2_der.cross(diff) + z2.cross(diff_der);
            zi_der=z2_der;
            break;
        case 3:
            Rot_3 = T.block(0,0,3,3);
            Rot_der_3 = S_3*Rot_3;
            z3_der = Rot_der_3.block(0,2,3,1);
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            diff_der = pos_hand_lin_vel - pos_3_lin_vel;
            cross = z3_der.cross(diff) + z3.cross(diff_der);
            zi_der=z3_der;
            break;
        case 4:
            Rot_4 = T.block(0,0,3,3);
            Rot_der_4 = S_4*Rot_4;
            z4_der = Rot_der_4.block(0,2,3,1);
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            diff_der = pos_hand_lin_vel - pos_4_lin_vel;
            cross = z4_der.cross(diff) + z4.cross(diff_der);
            zi_der=z4_der;
            break;
        case 5:
            Rot_5 = T.block(0,0,3,3);
            Rot_der_5 = S_5*Rot_5;
            z5_der = Rot_der_5.block(0,2,3,1);
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            diff_der = pos_hand_lin_vel - pos_5_lin_vel;
            cross = z5_der.cross(diff) + z5.cross(diff_der);
            zi_der=z5_der;
            break;
        case 6:
            Rot_6 = T.block(0,0,3,3);
            Rot_der_6 = S_0*Rot_0;
            z6_der = Rot_der_6.block(0,2,3,1);
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            diff_der = pos_hand_lin_vel - pos_6_lin_vel;
            cross = z6_der.cross(diff) + z6.cross(diff_der);
            zi_der=z6_der;
            break;
        }
        VectorXd column(6); column << cross, zi_der;
        TimeDerivativeJacobian.col(i) = column;
    }


}

void Humanoid::getTimeDerivativeJacobian(MatrixXd &currJacobian, MatrixXd &pastJacobian, double timestep, MatrixXd &TimeDerivativeJacobian)
{
    TimeDerivativeJacobian.resize(currJacobian.rows(),currJacobian.cols());

    for(int i=0; i<currJacobian.rows();++i)
    {
        for(int j=0; j<currJacobian.cols();++j)
        {
            TimeDerivativeJacobian(i,j) = (currJacobian(i,j) - pastJacobian(i,j)) / timestep;
        }
    }
}

void Humanoid::getDerivative(vector<MatrixXd> &matrix, vector<double> &step_values, vector<MatrixXd> &der_matrix)
{
    der_matrix.resize(matrix.size());
    vector<vector<vector<double>>> elements(matrix.at(0).rows());
    for(size_t h=0; h<elements.size();++h)
    {
        elements.at(h).resize(matrix.at(0).cols());
    }

    for(size_t i=0;i<matrix.size();++i)
    {
        MatrixXd mat = matrix.at(i);
        der_matrix.at(i).resize(mat.rows(),mat.cols());
        for(int r=0; r<mat.rows();++r){
            for(int c=0; c< mat.cols();++c){
                elements.at(r).at(c).push_back(mat(r,c));
            }

        }
    }
    //
    for(size_t h=0; h<elements.size();++h)
    { // rows
        for(size_t k=0; k<elements.at(h).size();++k)
        { // columns
            vector<double> el = elements.at(h).at(k);
            vector<double> der_el;
            this->getDerivative(el,step_values,der_el);
            for(size_t ii=0;ii<der_el.size();++ii)
            {
                der_matrix.at(ii)(h,k) = der_el.at(ii);
            }
        }
    }
}

void Humanoid::getDerivative(vector<double> &function, vector<double> &step_values, vector<double> &derFunction)
{
       const double MIN_STEP_VALUE = 0.1;

       // Formula of the numarical differentiation with 5 points
          // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
          // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
          // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
          // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
          // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


          int h = 1;
          int tnsample;
          double f0;
          double f1;
          double f2;
          double f3;
          double f4;
          double step_value;

          // 1st point
          // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
          tnsample = 0;
          f0 = function.at(tnsample);
          f1 = function.at(tnsample+1);
          f2 = function.at(tnsample+2);
          f3 = function.at(tnsample+3);
          f4 = function.at(tnsample+4);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));

          // 2nd point
          // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
          tnsample = 1;
          f0 = function.at(tnsample-1);
          f1 = function.at(tnsample);
          f2 = function.at(tnsample+1);
          f3 = function.at(tnsample+2);
          f4 = function.at(tnsample+3);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));

          // 3rd point
          // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
          for (size_t i=2; i< function.size() -2;++i){     // centered
              f0 = function.at(i-2);
              f1 = function.at(i-1);
              f2 = function.at(i);
              f3 = function.at(i+1);
              f4 = function.at(i+2);
              step_value = step_values.at(i);
              if(step_value==0)
                  step_value=0.01;
              derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
          }

          // 4th point
          // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
          tnsample = function.size()-2;
          f0 = function.at(tnsample-3);
          f1 = function.at(tnsample-2);
          f2 = function.at(tnsample-1);
          f3 = function.at(tnsample);
          f4 = function.at(tnsample+1);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));

          // 5th point
          // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
          tnsample = function.size()-1;
          f0 = function.at(tnsample-4);
          f1 = function.at(tnsample-3);
          f2 = function.at(tnsample-2);
          f3 = function.at(tnsample-1);
          f4 = function.at(tnsample);
          step_value = step_values.at(tnsample);
          if(step_value==0)
              step_value=MIN_STEP_VALUE;
          derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));



}



void Humanoid::directKinematicsSingleArm(int arm, std::vector<double>& posture)
{


    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(3);
    Matrix3d shoulderOr;
    vector<double> elbowPos = vector<double>(3);
    Matrix3d elbowOr;
    vector<double> wristPos = vector<double>(3);
    Matrix3d wristOr;
    vector<double> handPos = vector<double>(3);
    Matrix3d handOr;

    switch (arm) {
    case 1: // right arm        
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (int i = 0; i < posture.size(); ++i){

        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);

        T = T * T_aux;
        Vector3d v;

        if (i==0){
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];

            switch (arm){
            case 1: // right arm
                this->rightShoulderPos = shoulderPos;
                this->rightShoulderOr = shoulderOr;
                break;
            case 2: // left arm
                this->leftShoulderPos = shoulderPos;
                this->leftShoulderOr = shoulderOr;
                break;
            }

        }else if (i==2){

            // get the elbow
            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];

            switch(arm){
            case 1: // right arm
                this->rightElbowPos = elbowPos;
                this->rightElbowOr = elbowOr;
                break;
            case 2: // left arm
                this->leftElbowPos = elbowPos;
                this->leftElbowOr = elbowOr;
                break;
            }

        }else if (i==4){

            // get the wrist
            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];

            switch(arm){
            case 1: // right arm
                this->rightWristPos = wristPos;
                this->rightWristOr = wristOr;
                break;
            case 2: // left arm
                this->leftWristPos = wristPos;
                this->leftWristOr = wristOr;
                break;
            }


        } else if (i==6){

            //get the hand
            T = T * mat_hand;

            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];

            switch(arm){
            case 1: // right arm
                this->rightHandPos = handPos;
                this->rightHandOr = handOr;
                break;
            case 2: // left arm
                this->leftHandPos = handPos;
                this->leftHandOr = handOr;
                break;
            }


        }

    }

    // Direct kinematics of the fingers

    this->rightFingers.resize(HAND_FINGERS,12);
    this->leftFingers.resize(HAND_FINGERS,12);

    Matrix4d T_H_0_pos;
    vector<double> fing_pos;

    for (int i=0; i< HAND_FINGERS; ++i){
        DHparams p = m_DH_hand.at(i);
        switch (arm) {
        case 1: // right arm
            fing_pos=this->right_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,rightFingers);
            break;
        case 2: // left arm
            fing_pos=this->left_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,leftFingers);
            break;
        }
    }

}

void Humanoid::directDiffKinematicsSingleArm(int arm,vector<double> posture, vector<double> velocities, vector<double>& vel, int mod)
{
    VectorXd joint_velocities;
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);
    joint_velocities.resize(velocities.size());

    for (int i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
        joint_velocities(i) = velocities.at(i);
    }
    MatrixXd Jac_tmp; VectorXd joint_vel_tmp;
    // shoulder velocity
    Jac_tmp = JacobianArm.block<6,2>(0,0);
    joint_vel_tmp = joint_velocities.block<2,1>(0,0);
    VectorXd shoulder_vel_xd = Jac_tmp*joint_vel_tmp;
    // elbow velocity
    Jac_tmp = JacobianArm.block<6,4>(0,0);
    joint_vel_tmp = joint_velocities.block<4,1>(0,0);
    VectorXd elbow_vel_xd = Jac_tmp*joint_vel_tmp;
    // wrist velocity
    Jac_tmp = JacobianArm.block<6,6>(0,0);
    joint_vel_tmp = joint_velocities.block<6,1>(0,0);
    VectorXd wrist_vel_xd = Jac_tmp*joint_vel_tmp;
    // hand velocity
    VectorXd hand_vel_xd = JacobianArm*joint_velocities;

    switch(mod){
    case 0: // shoulder
        vel.clear();
        vel.resize(shoulder_vel_xd.size());
        VectorXd::Map(&vel[0], shoulder_vel_xd.size()) = shoulder_vel_xd;
        break;
    case 1:// elbow
        vel.clear();
        vel.resize(elbow_vel_xd.size());
        VectorXd::Map(&vel[0], elbow_vel_xd.size()) = elbow_vel_xd;
        break;
    case 2: // wrist
        vel.clear();
        vel.resize(wrist_vel_xd.size());
        VectorXd::Map(&vel[0], wrist_vel_xd.size()) = wrist_vel_xd;
        break;
    case 3: // hand
        vel.clear();
        vel.resize(hand_vel_xd.size());
        VectorXd::Map(&vel[0], hand_vel_xd.size()) = hand_vel_xd;
        break;
    default: // hand
        vel.clear();
        vel.resize(hand_vel_xd.size());
        VectorXd::Map(&vel[0], hand_vel_xd.size()) = hand_vel_xd;
        break;
    }
}

void Humanoid::inverseDiffKinematicsSingleArm(int arm, vector<double> posture, vector<double> hand_vel, vector<double> &velocities)
{

    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);


    for (int i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
    }

    double k; // damping factor
    MatrixXd I = MatrixXd::Identity(6,6);
    MatrixXd JacobianArmT = JacobianArm.transpose();
    MatrixXd JJ = JacobianArm*JacobianArmT;
    if(abs(JJ.determinant())<0.001){
        k = 0.01;
    }else{
        k=0.0;
    }
    MatrixXd JJk = (JJ+pow(k,2)*I);
    MatrixXd JJk_inv = JJk.inverse();
    MatrixXd J_plus = JacobianArmT*JJk_inv; // pseudo-inverse of the Jacobian
    VectorXd hand_vel_xd(6);
    hand_vel_xd << hand_vel.at(0),hand_vel.at(1),hand_vel.at(2),hand_vel.at(3),hand_vel.at(4),hand_vel.at(5);
    VectorXd joint_velocities = J_plus*hand_vel_xd;
    velocities.clear();
    velocities.resize(joint_velocities.size());
    VectorXd::Map(&velocities[0], joint_velocities.size()) = joint_velocities;

}

void Humanoid::inverseDiffKinematicsSingleArm(int arm, vector<double> posture, vector<double> hand_vel, vector<double> &velocities, bool jlim_en, bool sing_en, bool obsts_en,
                                              double vel_max, double sing_coeff, double sing_damping, double obst_coeff, double obst_damping,
                                              double jlim_th, double jlim_rate, double jlim_coeff, double jlim_damping, vector<objectPtr>& obsts)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    VectorXd vel_max_vec(JOINTS_ARM);
    for(size_t i=0; i < vel_max_vec.size(); ++i){
        vel_max_vec(i) = vel_max;
    }
    double vel_max_norm = vel_max_vec.norm();

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    vector<double> max_limits;
    vector<double> min_limits;
    vector<double> mid_limits(JOINTS_ARM+JOINTS_HAND);    

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        max_limits = this->max_rightLimits;
        min_limits = this->min_rightLimits;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        max_limits = this->max_leftLimits;
        min_limits = this->min_leftLimits;
        break;
    }


    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);

    // current Jacobian
    T = mat_world;
    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
    }

    //double k; // damping factor
    //MatrixXd I = MatrixXd::Identity(6,6);
    MatrixXd JacobianArmT = JacobianArm.transpose();
    MatrixXd JJ = JacobianArm*JacobianArmT;
    //if(abs(JJ.determinant()) < 0.01){
    //    k = 0.01;
    //}else{
    //    k=0.0;
    //}
    //MatrixXd JJk = (JJ+pow(k,2)*I);
    //MatrixXd JJk_inv = JJk.inverse();
    MatrixXd JJk_inv = JJ.inverse();
    MatrixXd J_plus = JacobianArmT*JJk_inv; // pseudo-inverse of the Jacobian
    VectorXd hand_vel_xd(6);
    hand_vel_xd << hand_vel.at(0),hand_vel.at(1),hand_vel.at(2),hand_vel.at(3),hand_vel.at(4),hand_vel.at(5);
    VectorXd joint_velocities = J_plus*hand_vel_xd;

    double null_th = 0.001;
    // Joints limits minimization function
    if(jlim_en){        
        VectorXd delta_H_jlim(posture.size());
        VectorXd delta_H_jlim_mod(posture.size());
        std::transform(max_limits.begin(),max_limits.end(),min_limits.begin(),mid_limits.begin(),plus<double>());
        std::transform(mid_limits.begin(), mid_limits.end(), mid_limits.begin(), std::bind1st(std::multiplies<double>(),0.5));
        for (size_t i = 0; i < posture.size(); ++i){
            delta_H_jlim(i) = (posture.at(i) - mid_limits.at(i))/(max_limits.at(i) - min_limits.at(i));
            delta_H_jlim_mod(i) = delta_H_jlim(i)*(1+exp((delta_H_jlim(i) - jlim_th)/jlim_rate));
        }
        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
        MatrixXd Jpp = J_plus*JacobianArm;
        MatrixXd J_Null = Id - Jpp;
        //VectorXd J_jlim= J_Null*delta_H_jlim;
        VectorXd J_jlim= J_Null*delta_H_jlim_mod;
        double k_jlim = 0;
        if(J_Null.norm() > null_th){
            k_jlim = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_jlim.norm())));
        }
        double fd = jlim_coeff * (1 - exp(-jlim_damping*(J_jlim.norm())));

        joint_velocities +=  k_jlim*fd*J_jlim;

        //BOOST_LOG_SEV(lg, info) << "# ----------------------------------- # ";
        //BOOST_LOG_SEV(lg, info) << "k_jlim = " << k_jlim;
        //BOOST_LOG_SEV(lg, info) << "joint 2 = " << J_Null(1);
        //BOOST_LOG_SEV(lg, info) << "joint 3 = " << J_Null(2);
        //BOOST_LOG_SEV(lg, info) << "joint 4 = " << J_Null(3);
        //BOOST_LOG_SEV(lg, info) << "joint 5 = " << J_Null(4);
        //BOOST_LOG_SEV(lg, info) << "joint 6 = " << J_Null(5);
        //BOOST_LOG_SEV(lg, info) << "joint 7 = " << J_Null(6);
    }

    // Manipulability measure minimization function (Avoidance of singularities)
    if(sing_en){
        double delta_theta = 0.001; // rad = 0.57 deg
        vector<double> posture_delta(posture.size());
        MatrixXd JacobianArm_delta(6,JOINTS_ARM);
        std::transform(posture.begin(), posture.end(), posture_delta.begin(), std::bind1st(std::plus<double>(),delta_theta));
        T = mat_world;
        for (size_t i = 0; i < posture_delta.size(); ++i){
            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture_delta.at(i),T_aux);
            T = T * T_aux;
            Vector3d diff;
            Vector3d cross;
            Vector3d zi;
            switch(i){
            case 0:
                z0 = T.block(0,2,3,1);
                pos0 = T.block(0,3,3,1);
                diff = pos_hand - pos0;
                cross = z0.cross(diff);
                zi=z0;
                break;
            case 1:
                z1 = T.block(0,2,3,1);
                pos1 = T.block(0,3,3,1);
                diff = pos_hand - pos1;
                cross = z1.cross(diff);
                zi=z1;
                break;
            case 2:
                z2 = T.block(0,2,3,1);
                pos2 = T.block(0,3,3,1);
                diff = pos_hand - pos2;
                cross = z2.cross(diff);
                zi=z2;
                break;
            case 3:
                z3 = T.block(0,2,3,1);
                pos3 = T.block(0,3,3,1);
                diff = pos_hand - pos3;
                cross = z3.cross(diff);
                zi=z3;
                break;
            case 4:
                z4 = T.block(0,2,3,1);
                pos4 = T.block(0,3,3,1);
                diff = pos_hand - pos4;
                cross = z4.cross(diff);
                zi=z4;
                break;
            case 5:
                z5 = T.block(0,2,3,1);
                pos5 = T.block(0,3,3,1);
                diff = pos_hand - pos5;
                cross = z5.cross(diff);
                zi=z5;
                break;
            case 6:
                z6 = T.block(0,2,3,1);
                pos6 = T.block(0,3,3,1);
                diff = pos_hand - pos6;
                cross = z6.cross(diff);
                zi=z6;
                break;
            }
            VectorXd column(6); column << cross, zi;
            JacobianArm_delta.col(i) = column;
        }

        MatrixXd JacobianArm_deltaT = JacobianArm_delta.transpose();
        MatrixXd JJ_delta = JacobianArm_delta*JacobianArm_deltaT;
        VectorXd delta_H_sing(posture_delta.size());
        double H_sing_av = -sqrt(JJ.determinant()); // minimize H to maximize the manipulability measure
        double H_sing_av_delta = -sqrt(JJ_delta.determinant()); // minimize H to maximize the manipulability measure
        for (int i = 0; i < delta_H_sing.size(); ++i){
            delta_H_sing(i) = (H_sing_av_delta - H_sing_av)/delta_theta;
        }
        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
        MatrixXd Jpp = J_plus*JacobianArm;
        MatrixXd J_Null = Id - Jpp;
        VectorXd J_sing= J_Null*delta_H_sing;
        double k_sing = 0;
        if(J_sing.norm() > null_th){
            k_sing = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_sing.norm())));
        }
        double fd = sing_coeff * (1 - exp(-sing_damping*(J_sing.norm())));

        joint_velocities +=  (k_sing*fd*J_sing);

        //BOOST_LOG_SEV(lg, info) << "# ----------------------------------- # ";
        //BOOST_LOG_SEV(lg, info) << "k_sing = " << k_sing;
        //BOOST_LOG_SEV(lg, info) << "joint 2 = " << J_Null(1);
        //BOOST_LOG_SEV(lg, info) << "joint 3 = " << J_Null(2);
        //BOOST_LOG_SEV(lg, info) << "joint 4 = " << J_Null(3);
        //BOOST_LOG_SEV(lg, info) << "joint 5 = " << J_Null(4);
        //BOOST_LOG_SEV(lg, info) << "joint 6 = " << J_Null(5);
        //BOOST_LOG_SEV(lg, info) << "joint 7 = " << J_Null(6);
    }

    // Obstacles avoidance
    if(obsts_en){

        // current positions of the arm
        vector<vector<double>> points_arm = vector<vector<double>>(5);
        vector<double> shoulderPos = vector<double>(3);
        vector<double> shoulder_elbowPos = vector<double>(3);
        vector<double> elbowPos = vector<double>(3);
        vector<double> elbow_wristPos = vector<double>(3);
        vector<double> wristPos = vector<double>(3);
        vector<double> wrist_handPos = vector<double>(3);
        vector<double> handPos = vector<double>(3);
        T = mat_world;
        for (size_t i = 0; i < posture.size(); ++i){
            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
            T = T * T_aux;
            Vector3d v;
            switch(i){
            case 0:
                v = T.block(0,3,3,1);
                shoulderPos[0] = v[0];
                shoulderPos[1] = v[1];
                shoulderPos[2] = v[2];
                break;
            case 2:
                v = T.block(0,3,3,1);
                elbowPos[0] = v[0];
                elbowPos[1] = v[1];
                elbowPos[2] = v[2];
                break;
            case 4:
                v = T.block(0,3,3,1);
                wristPos[0] = v[0];
                wristPos[1] = v[1];
                wristPos[2] = v[2];
                break;
            case 6:
                v = T.block(0,3,3,1);
                handPos[0] = v[0];
                handPos[1] = v[1];
                handPos[2] = v[2];
                break;
            default:
                // do nothing
                break;
            }
        }
        //middle point between the shoulder and the elbow
        std::transform(shoulderPos.begin(),shoulderPos.end(),elbowPos.begin(),shoulder_elbowPos.begin(),plus<double>());
        std::transform(shoulder_elbowPos.begin(), shoulder_elbowPos.end(), shoulder_elbowPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
        //middle point between the elbow and the wrist
        std::transform(elbowPos.begin(),elbowPos.end(),wristPos.begin(),elbow_wristPos.begin(),plus<double>());
        std::transform(elbow_wristPos.begin(), elbow_wristPos.end(), elbow_wristPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
        //middle point between the wrist and the hand
        std::transform(wristPos.begin(),wristPos.end(),handPos.begin(),wrist_handPos.begin(),plus<double>());
        std::transform(wrist_handPos.begin(), wrist_handPos.end(), wrist_handPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
        // points on the arm
        points_arm.at(0) = shoulder_elbowPos;
        points_arm.at(1) = elbowPos;
        points_arm.at(2) = elbow_wristPos;
        points_arm.at(3) = wristPos;
        points_arm.at(4) = wrist_handPos;

        // perturbated positions of the arm
        double delta_theta = 0.001; // rad = 0.57 deg
        vector<double> posture_delta(posture.size());
        std::transform(posture.begin(), posture.end(), posture_delta.begin(), std::bind1st(std::plus<double>(),delta_theta));
        vector<vector<double>> points_arm_delta = vector<vector<double>>(5);
        vector<double> shoulderPos_delta = vector<double>(3);
        vector<double> shoulder_elbowPos_delta = vector<double>(3);
        vector<double> elbowPos_delta = vector<double>(3);
        vector<double> elbow_wristPos_delta = vector<double>(3);
        vector<double> wristPos_delta = vector<double>(3);
        vector<double> wrist_handPos_delta = vector<double>(3);
        vector<double> handPos_delta = vector<double>(3);
        T = mat_world;
        for (size_t i = 0; i < posture_delta.size(); ++i){
            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture_delta.at(i),T_aux);
            T = T * T_aux;
            Vector3d v;
            switch(i){
            case 0:
                v = T.block(0,3,3,1);
                shoulderPos_delta[0] = v[0];
                shoulderPos_delta[1] = v[1];
                shoulderPos_delta[2] = v[2];
                break;
            case 2:
                v = T.block(0,3,3,1);
                elbowPos_delta[0] = v[0];
                elbowPos_delta[1] = v[1];
                elbowPos_delta[2] = v[2];
                break;
            case 4:
                v = T.block(0,3,3,1);
                wristPos_delta[0] = v[0];
                wristPos_delta[1] = v[1];
                wristPos_delta[2] = v[2];
                break;
            case 6:
                v = T.block(0,3,3,1);
                handPos_delta[0] = v[0];
                handPos_delta[1] = v[1];
                handPos_delta[2] = v[2];
                break;
            default:
                // do nothing
                break;
            }
        }
        //middle point between the shoulder and the elbow
        std::transform(shoulderPos_delta.begin(),shoulderPos_delta.end(),elbowPos_delta.begin(),shoulder_elbowPos_delta.begin(),plus<double>());
        std::transform(shoulder_elbowPos_delta.begin(), shoulder_elbowPos_delta.end(), shoulder_elbowPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
        //middle point between the elbow and the wrist
        std::transform(elbowPos_delta.begin(),elbowPos_delta.end(),wristPos_delta.begin(),elbow_wristPos_delta.begin(),plus<double>());
        std::transform(elbow_wristPos_delta.begin(), elbow_wristPos_delta.end(), elbow_wristPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
        //middle point between the wrist and the hand
        std::transform(wristPos_delta.begin(),wristPos_delta.end(),handPos_delta.begin(),wrist_handPos_delta.begin(),plus<double>());
        std::transform(wrist_handPos_delta.begin(), wrist_handPos_delta.end(), wrist_handPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
        // points on the arm
        points_arm_delta.at(0) = shoulder_elbowPos_delta;
        points_arm_delta.at(1) = elbowPos_delta;
        points_arm_delta.at(2) = elbow_wristPos_delta;
        points_arm_delta.at(3) = wristPos_delta;
        points_arm_delta.at(4) = wrist_handPos_delta;


        vector<double> e_arm = vector<double>(5,0.0);
        vector<double> e_arm_delta = vector<double>(5,0.0);
        vector<double> e_arm_torso = vector<double>(5,0.0);
        vector<double> e_arm_torso_delta = vector<double>(5,0.0);
        // current obstacles in the scenario
        for(size_t i=0;i<obsts.size();++i){
            objectPtr obst = obsts.at(i);
            vector<double> obst_pos = vector<double>(3);
            vector<double> obst_or = vector<double>(3);
            obst_pos.at(0) = obst->getPos().Xpos;
            obst_pos.at(1) = obst->getPos().Ypos;
            obst_pos.at(2) = obst->getPos().Zpos;
            obst_or.at(0) = obst->getOr().roll;
            obst_or.at(1) = obst->getOr().pitch;
            obst_or.at(2) = obst->getOr().yaw;
            Matrix3d Rot_obst; this->Rot_matrix(Rot_obst,obst_or);
            Matrix3d Rot_obst_t = Rot_obst.transpose();
            double x_size = obst->getSize().Xsize;
            double y_size = obst->getSize().Ysize;
            double z_size = obst->getSize().Zsize;
            Matrix3d A_obst;
            A_obst(0,0) = pow(x_size,-2); A_obst(0,1) = 0.0; A_obst(0,2) = 0.0;
            A_obst(1,0) = 0.0; A_obst(1,1) = pow(y_size,-2); A_obst(1,2) = 0.0;
            A_obst(2,0) = 0.0; A_obst(2,1) = 0.0; A_obst(2,2) = pow(z_size,-2);
            Matrix3d L_obst = Rot_obst_t*A_obst*Rot_obst;

            // distances between the arm and the obstacles
            vector<double> dist_arm = vector<double>(5); vector<double> e_dist_arm = vector<double>(5);
            vector<double> dist_arm_delta = vector<double>(5); vector<double> e_dist_arm_delta = vector<double>(5);
            // get the distances between the current obstacle and the points on the arm            
            this->get_distances_arm_obstacles(points_arm,obst_pos,L_obst,dist_arm);
            // get the distances between the current obstacle and the points on the arm with perturbated posture
            this->get_distances_arm_obstacles(points_arm_delta,obst_pos,L_obst,dist_arm_delta);

            // distances between the arm and the torso of the humanoid
            vector<double> dist_arm_torso = vector<double>(5); vector<double> e_dist_arm_torso = vector<double>(5);
            vector<double> dist_arm_torso_delta = vector<double>(5); vector<double> e_dist_arm_torso_delta = vector<double>(5);
            this->get_distances_arm_torso(points_arm,dist_arm_torso);
            this->get_distances_arm_torso(points_arm_delta,dist_arm_torso_delta);

            double k_off = 0.0001;// offset constant
            for (size_t i=0; i<dist_arm.size();++i){
                e_dist_arm.at(i) = 1/(dist_arm.at(i)+k_off);
                e_dist_arm_delta.at(i) = 1/(dist_arm_delta.at(i)+k_off);
                e_dist_arm_torso.at(i) = 1/(dist_arm_torso.at(i)+k_off);
                e_dist_arm_torso_delta.at(i) = 1/(dist_arm_torso_delta.at(i)+k_off);
            }

            for(size_t i=0;i<e_arm.size();++i){
                e_arm.at(i) += e_dist_arm.at(i);
                e_arm_delta.at(i) += e_dist_arm_delta.at(i);
                e_arm_torso.at(i) += e_dist_arm_torso.at(i);
                e_arm_torso_delta.at(i) += e_dist_arm_torso_delta.at(i);
            }
        } //for loop obstacles

        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
        MatrixXd Jpp = J_plus*JacobianArm;
        MatrixXd J_Null = Id - Jpp;

        // ---------- torso ----------------------------------------

        // shoulder - elbow
        VectorXd delta_H_torso_SE(posture.size());
        for(size_t i=0;i<delta_H_torso_SE.size();++i){
            delta_H_torso_SE(i) = (e_arm_torso_delta.at(0) - e_arm_torso.at(0))/delta_theta;
        }
        VectorXd J_torso_SE= J_Null*delta_H_torso_SE;
        double k_torso_SE = 0;
        if(J_torso_SE.norm() > null_th){
            k_torso_SE = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_SE.norm())));
        }
        double fd_torso_SE = obst_coeff * (1 - exp(-obst_damping*(J_torso_SE.norm())));
        joint_velocities +=  (k_torso_SE*fd_torso_SE*J_torso_SE);

        // elbow
        VectorXd delta_H_torso_E(posture.size());
        for(size_t i=0;i<delta_H_torso_E.size();++i){
            delta_H_torso_E(i) = (e_arm_torso_delta.at(1) - e_arm_torso.at(1))/delta_theta;
        }
        VectorXd J_torso_E= J_Null*delta_H_torso_E;
        double k_torso_E = 0;
        if(J_torso_E.norm() > null_th){
            k_torso_E = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_E.norm())));
        }
        double fd_torso_E = obst_coeff * (1 - exp(-obst_damping*(J_torso_E.norm())));
        joint_velocities +=  (k_torso_E*fd_torso_E*J_torso_E);

        // elbow - wrist
        VectorXd delta_H_torso_EW(posture.size());
        for(size_t i=0;i<delta_H_torso_EW.size();++i){
            delta_H_torso_EW(i) = (e_arm_torso_delta.at(2) - e_arm_torso.at(2))/delta_theta;
        }
        VectorXd J_torso_EW= J_Null*delta_H_torso_EW;
        double k_torso_EW = 0;
        if(J_torso_EW.norm() > null_th){
            k_torso_EW = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_EW.norm())));
        }
        double fd_torso_EW = obst_coeff * (1 - exp(-obst_damping*(J_torso_EW.norm())));
        joint_velocities +=  (k_torso_EW*fd_torso_EW*J_torso_EW);

        // wrist
        VectorXd delta_H_torso_W(posture.size());
        for(size_t i=0;i<delta_H_torso_W.size();++i){
            delta_H_torso_W(i) = (e_arm_torso_delta.at(3) - e_arm_torso.at(3))/delta_theta;
        }
        VectorXd J_torso_W= J_Null*delta_H_torso_W;
        double k_torso_W = 0;
        if(J_torso_W.norm() > null_th){
            k_torso_W = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_W.norm())));
        }
        double fd_torso_W = obst_coeff * (1 - exp(-obst_damping*(J_torso_W.norm())));
        joint_velocities +=  (k_torso_W*fd_torso_W*J_torso_W);

        // wrist - hand
        VectorXd delta_H_torso_WH(posture.size());
        for(size_t i=0;i<delta_H_torso_WH.size();++i){
            delta_H_torso_WH(i) = (e_arm_torso_delta.at(4) - e_arm_torso.at(4))/delta_theta;
        }
        VectorXd J_torso_WH= J_Null*delta_H_torso_WH;
        double k_torso_WH = 0;
        if(J_torso_WH.norm() > null_th){
            k_torso_WH = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_WH.norm())));
        }
        double fd_torso_WH = obst_coeff * (1 - exp(-obst_damping*(J_torso_WH.norm())));
        joint_velocities +=  (k_torso_WH*fd_torso_WH*J_torso_WH);

        // ---------- obstacles --------------------------------

        // shoulder - elbow
        VectorXd delta_H_obst_SE(posture.size());
        for(size_t i=0;i<delta_H_obst_SE.size();++i){
            delta_H_obst_SE(i) = (e_arm_delta.at(0) - e_arm.at(0))/delta_theta;
        }
        VectorXd J_obst_SE= J_Null*delta_H_obst_SE;
        double k_obst_SE = 0;
        if(J_obst_SE.norm() > null_th){
            k_obst_SE = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_SE.norm())));
        }
        double fd_SE = obst_coeff * (1 - exp(-obst_damping*(J_obst_SE.norm())));
        joint_velocities +=  (k_obst_SE*fd_SE*J_obst_SE);

        // elbow
        VectorXd delta_H_obst_E(posture.size());
        for(size_t i=0;i<delta_H_obst_E.size();++i){
            delta_H_obst_E(i) = (e_arm_delta.at(1) - e_arm.at(1))/delta_theta;
        }
        VectorXd J_obst_E= J_Null*delta_H_obst_E;
        double k_obst_E = 0;
        if(J_obst_E.norm() > null_th){
            k_obst_E = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_E.norm())));
        }
        double fd_E = obst_coeff * (1 - exp(-obst_damping*(J_obst_E.norm())));
        joint_velocities +=  (k_obst_E*fd_E*J_obst_E);

        // elbow - wrist
        VectorXd delta_H_obst_EW(posture.size());
        for(size_t i=0;i<delta_H_obst_EW.size();++i){
            delta_H_obst_EW(i) = (e_arm_delta.at(2) - e_arm.at(2))/delta_theta;
        }
        VectorXd J_obst_EW= J_Null*delta_H_obst_EW;
        double k_obst_EW = 0;
        if(J_obst_EW.norm() > null_th){
            k_obst_EW = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_EW.norm())));
        }
        double fd_EW = obst_coeff * (1 - exp(-obst_damping*(J_obst_EW.norm())));
        joint_velocities +=  (k_obst_EW*fd_EW*J_obst_EW);

        // wrist
        VectorXd delta_H_obst_W(posture.size());
        for(size_t i=0;i<delta_H_obst_W.size();++i){
            delta_H_obst_W(i) = (e_arm_delta.at(3) - e_arm.at(3))/delta_theta;
        }
        VectorXd J_obst_W= J_Null*delta_H_obst_W;
        double k_obst_W = 0;
        if(J_obst_W.norm() > null_th){
            k_obst_W = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_W.norm())));
        }
        double fd_W = obst_coeff * (1 - exp(-obst_damping*(J_obst_W.norm())));
        joint_velocities +=  (k_obst_W*fd_W*J_obst_W);

        // wrist - hand
        VectorXd delta_H_obst_WH(posture.size());
        for(size_t i=0;i<delta_H_obst_WH.size();++i){
            delta_H_obst_WH(i) = (e_arm_delta.at(4) - e_arm.at(4))/delta_theta;
        }
        VectorXd J_obst_WH= J_Null*delta_H_obst_WH;
        double k_obst_WH = 0;
        if(J_obst_WH.norm() > null_th){
            k_obst_WH = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_WH.norm())));
        }
        double fd_WH = obst_coeff * (1 - exp(-obst_damping*(J_obst_WH.norm())));
        joint_velocities +=  (k_obst_WH*fd_WH*J_obst_WH);

    }


    velocities.clear();
    velocities.resize(joint_velocities.size());
    VectorXd::Map(&velocities[0], joint_velocities.size()) = joint_velocities;

}


void Humanoid::inverseDiffKinematicsSingleArm2(int arm, vector<double> posture, vector<double> hand_acc, vector<double> &accelerations, bool jlim_en, bool sing_en, bool obsts_en,
                                              double vel_max, double sing_coeff, double sing_damping, double obst_coeff, double obst_damping,
                                              double jlim_th, double jlim_rate, double jlim_coeff, double jlim_damping, vector<objectPtr>& obsts)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    VectorXd vel_max_vec(JOINTS_ARM);
    for(size_t i=0; i < vel_max_vec.size(); ++i){
        vel_max_vec(i) = vel_max;
    }
    double vel_max_norm = vel_max_vec.norm();

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    vector<double> max_limits;
    vector<double> min_limits;
    vector<double> mid_limits(JOINTS_ARM+JOINTS_HAND);

    switch (arm) {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        max_limits = this->max_rightLimits;
        min_limits = this->min_rightLimits;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        max_limits = this->max_leftLimits;
        min_limits = this->min_leftLimits;
        break;
    }


    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);

    // current Jacobian
    T = mat_world;
    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
    }

    //double k; // damping factor
    //MatrixXd I = MatrixXd::Identity(6,6);
    MatrixXd JacobianArmT = JacobianArm.transpose();
    MatrixXd JJ = JacobianArm*JacobianArmT;
    //if(abs(JJ.determinant()) < 0.01){
    //    k = 0.01;
    //}else{
    //    k=0.0;
    //}
    //MatrixXd JJk = (JJ+pow(k,2)*I);
    //MatrixXd JJk_inv = JJk.inverse();
    MatrixXd JJk_inv = JJ.inverse();
    MatrixXd J_plus = JacobianArmT*JJk_inv; // pseudo-inverse of the Jacobian
    VectorXd hand_acc_xd(6);
    hand_acc_xd << hand_acc.at(0),hand_acc.at(1),hand_acc.at(2),hand_acc.at(3),hand_acc.at(4),hand_acc.at(5);
    VectorXd joint_accelerations = J_plus*hand_acc_xd;

    double null_th = 0.001;
    // Joints limits minimization function
    if(jlim_en){
//        VectorXd delta_H_jlim(posture.size());
//        VectorXd delta_H_jlim_mod(posture.size());
//        std::transform(max_limits.begin(),max_limits.end(),min_limits.begin(),mid_limits.begin(),plus<double>());
//        std::transform(mid_limits.begin(), mid_limits.end(), mid_limits.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        for (size_t i = 0; i < posture.size(); ++i){
//            delta_H_jlim(i) = (posture.at(i) - mid_limits.at(i))/(max_limits.at(i) - min_limits.at(i));
//            delta_H_jlim_mod(i) = delta_H_jlim(i)*(1+exp((delta_H_jlim(i) - jlim_th)/jlim_rate));
//        }
//        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
//        MatrixXd Jpp = J_plus*JacobianArm;
//        MatrixXd J_Null = Id - Jpp;
//        //VectorXd J_jlim= J_Null*delta_H_jlim;
//        VectorXd J_jlim= J_Null*delta_H_jlim_mod;
//        double k_jlim = 0;
//        if(J_Null.norm() > null_th){
//            k_jlim = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_jlim.norm())));
//        }
//        double fd = jlim_coeff * (1 - exp(-jlim_damping*(J_jlim.norm())));

//        joint_velocities +=  k_jlim*fd*J_jlim;

//        //BOOST_LOG_SEV(lg, info) << "# ----------------------------------- # ";
//        //BOOST_LOG_SEV(lg, info) << "k_jlim = " << k_jlim;
//        //BOOST_LOG_SEV(lg, info) << "joint 2 = " << J_Null(1);
//        //BOOST_LOG_SEV(lg, info) << "joint 3 = " << J_Null(2);
//        //BOOST_LOG_SEV(lg, info) << "joint 4 = " << J_Null(3);
//        //BOOST_LOG_SEV(lg, info) << "joint 5 = " << J_Null(4);
//        //BOOST_LOG_SEV(lg, info) << "joint 6 = " << J_Null(5);
//        //BOOST_LOG_SEV(lg, info) << "joint 7 = " << J_Null(6);
    }

    // Manipulability measure minimization function (Avoidance of singularities)
    if(sing_en){
//        double delta_theta = 0.001; // rad = 0.57 deg
//        vector<double> posture_delta(posture.size());
//        MatrixXd JacobianArm_delta(6,JOINTS_ARM);
//        std::transform(posture.begin(), posture.end(), posture_delta.begin(), std::bind1st(std::plus<double>(),delta_theta));
//        T = mat_world;
//        for (size_t i = 0; i < posture_delta.size(); ++i){
//            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture_delta.at(i),T_aux);
//            T = T * T_aux;
//            Vector3d diff;
//            Vector3d cross;
//            Vector3d zi;
//            switch(i){
//            case 0:
//                z0 = T.block(0,2,3,1);
//                pos0 = T.block(0,3,3,1);
//                diff = pos_hand - pos0;
//                cross = z0.cross(diff);
//                zi=z0;
//                break;
//            case 1:
//                z1 = T.block(0,2,3,1);
//                pos1 = T.block(0,3,3,1);
//                diff = pos_hand - pos1;
//                cross = z1.cross(diff);
//                zi=z1;
//                break;
//            case 2:
//                z2 = T.block(0,2,3,1);
//                pos2 = T.block(0,3,3,1);
//                diff = pos_hand - pos2;
//                cross = z2.cross(diff);
//                zi=z2;
//                break;
//            case 3:
//                z3 = T.block(0,2,3,1);
//                pos3 = T.block(0,3,3,1);
//                diff = pos_hand - pos3;
//                cross = z3.cross(diff);
//                zi=z3;
//                break;
//            case 4:
//                z4 = T.block(0,2,3,1);
//                pos4 = T.block(0,3,3,1);
//                diff = pos_hand - pos4;
//                cross = z4.cross(diff);
//                zi=z4;
//                break;
//            case 5:
//                z5 = T.block(0,2,3,1);
//                pos5 = T.block(0,3,3,1);
//                diff = pos_hand - pos5;
//                cross = z5.cross(diff);
//                zi=z5;
//                break;
//            case 6:
//                z6 = T.block(0,2,3,1);
//                pos6 = T.block(0,3,3,1);
//                diff = pos_hand - pos6;
//                cross = z6.cross(diff);
//                zi=z6;
//                break;
//            }
//            VectorXd column(6); column << cross, zi;
//            JacobianArm_delta.col(i) = column;
//        }

//        MatrixXd JacobianArm_deltaT = JacobianArm_delta.transpose();
//        MatrixXd JJ_delta = JacobianArm_delta*JacobianArm_deltaT;
//        VectorXd delta_H_sing(posture_delta.size());
//        double H_sing_av = -sqrt(JJ.determinant()); // minimize H to maximize the manipulability measure
//        double H_sing_av_delta = -sqrt(JJ_delta.determinant()); // minimize H to maximize the manipulability measure
//        for (int i = 0; i < delta_H_sing.size(); ++i){
//            delta_H_sing(i) = (H_sing_av_delta - H_sing_av)/delta_theta;
//        }
//        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
//        MatrixXd Jpp = J_plus*JacobianArm;
//        MatrixXd J_Null = Id - Jpp;
//        VectorXd J_sing= J_Null*delta_H_sing;
//        double k_sing = 0;
//        if(J_sing.norm() > null_th){
//            k_sing = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_sing.norm())));
//        }
//        double fd = sing_coeff * (1 - exp(-sing_damping*(J_sing.norm())));

//        joint_velocities +=  (k_sing*fd*J_sing);

//        //BOOST_LOG_SEV(lg, info) << "# ----------------------------------- # ";
//        //BOOST_LOG_SEV(lg, info) << "k_sing = " << k_sing;
//        //BOOST_LOG_SEV(lg, info) << "joint 2 = " << J_Null(1);
//        //BOOST_LOG_SEV(lg, info) << "joint 3 = " << J_Null(2);
//        //BOOST_LOG_SEV(lg, info) << "joint 4 = " << J_Null(3);
//        //BOOST_LOG_SEV(lg, info) << "joint 5 = " << J_Null(4);
//        //BOOST_LOG_SEV(lg, info) << "joint 6 = " << J_Null(5);
//        //BOOST_LOG_SEV(lg, info) << "joint 7 = " << J_Null(6);
    }

    // Obstacles avoidance
    if(obsts_en){

//        // current positions of the arm
//        vector<vector<double>> points_arm = vector<vector<double>>(5);
//        vector<double> shoulderPos = vector<double>(3);
//        vector<double> shoulder_elbowPos = vector<double>(3);
//        vector<double> elbowPos = vector<double>(3);
//        vector<double> elbow_wristPos = vector<double>(3);
//        vector<double> wristPos = vector<double>(3);
//        vector<double> wrist_handPos = vector<double>(3);
//        vector<double> handPos = vector<double>(3);
//        T = mat_world;
//        for (size_t i = 0; i < posture.size(); ++i){
//            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
//            T = T * T_aux;
//            Vector3d v;
//            switch(i){
//            case 0:
//                v = T.block(0,3,3,1);
//                shoulderPos[0] = v[0];
//                shoulderPos[1] = v[1];
//                shoulderPos[2] = v[2];
//                break;
//            case 2:
//                v = T.block(0,3,3,1);
//                elbowPos[0] = v[0];
//                elbowPos[1] = v[1];
//                elbowPos[2] = v[2];
//                break;
//            case 4:
//                v = T.block(0,3,3,1);
//                wristPos[0] = v[0];
//                wristPos[1] = v[1];
//                wristPos[2] = v[2];
//                break;
//            case 6:
//                v = T.block(0,3,3,1);
//                handPos[0] = v[0];
//                handPos[1] = v[1];
//                handPos[2] = v[2];
//                break;
//            default:
//                // do nothing
//                break;
//            }
//        }
//        //middle point between the shoulder and the elbow
//        std::transform(shoulderPos.begin(),shoulderPos.end(),elbowPos.begin(),shoulder_elbowPos.begin(),plus<double>());
//        std::transform(shoulder_elbowPos.begin(), shoulder_elbowPos.end(), shoulder_elbowPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        //middle point between the elbow and the wrist
//        std::transform(elbowPos.begin(),elbowPos.end(),wristPos.begin(),elbow_wristPos.begin(),plus<double>());
//        std::transform(elbow_wristPos.begin(), elbow_wristPos.end(), elbow_wristPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        //middle point between the wrist and the hand
//        std::transform(wristPos.begin(),wristPos.end(),handPos.begin(),wrist_handPos.begin(),plus<double>());
//        std::transform(wrist_handPos.begin(), wrist_handPos.end(), wrist_handPos.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        // points on the arm
//        points_arm.at(0) = shoulder_elbowPos;
//        points_arm.at(1) = elbowPos;
//        points_arm.at(2) = elbow_wristPos;
//        points_arm.at(3) = wristPos;
//        points_arm.at(4) = wrist_handPos;

//        // perturbated positions of the arm
//        double delta_theta = 0.001; // rad = 0.57 deg
//        vector<double> posture_delta(posture.size());
//        std::transform(posture.begin(), posture.end(), posture_delta.begin(), std::bind1st(std::plus<double>(),delta_theta));
//        vector<vector<double>> points_arm_delta = vector<vector<double>>(5);
//        vector<double> shoulderPos_delta = vector<double>(3);
//        vector<double> shoulder_elbowPos_delta = vector<double>(3);
//        vector<double> elbowPos_delta = vector<double>(3);
//        vector<double> elbow_wristPos_delta = vector<double>(3);
//        vector<double> wristPos_delta = vector<double>(3);
//        vector<double> wrist_handPos_delta = vector<double>(3);
//        vector<double> handPos_delta = vector<double>(3);
//        T = mat_world;
//        for (size_t i = 0; i < posture_delta.size(); ++i){
//            this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture_delta.at(i),T_aux);
//            T = T * T_aux;
//            Vector3d v;
//            switch(i){
//            case 0:
//                v = T.block(0,3,3,1);
//                shoulderPos_delta[0] = v[0];
//                shoulderPos_delta[1] = v[1];
//                shoulderPos_delta[2] = v[2];
//                break;
//            case 2:
//                v = T.block(0,3,3,1);
//                elbowPos_delta[0] = v[0];
//                elbowPos_delta[1] = v[1];
//                elbowPos_delta[2] = v[2];
//                break;
//            case 4:
//                v = T.block(0,3,3,1);
//                wristPos_delta[0] = v[0];
//                wristPos_delta[1] = v[1];
//                wristPos_delta[2] = v[2];
//                break;
//            case 6:
//                v = T.block(0,3,3,1);
//                handPos_delta[0] = v[0];
//                handPos_delta[1] = v[1];
//                handPos_delta[2] = v[2];
//                break;
//            default:
//                // do nothing
//                break;
//            }
//        }
//        //middle point between the shoulder and the elbow
//        std::transform(shoulderPos_delta.begin(),shoulderPos_delta.end(),elbowPos_delta.begin(),shoulder_elbowPos_delta.begin(),plus<double>());
//        std::transform(shoulder_elbowPos_delta.begin(), shoulder_elbowPos_delta.end(), shoulder_elbowPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        //middle point between the elbow and the wrist
//        std::transform(elbowPos_delta.begin(),elbowPos_delta.end(),wristPos_delta.begin(),elbow_wristPos_delta.begin(),plus<double>());
//        std::transform(elbow_wristPos_delta.begin(), elbow_wristPos_delta.end(), elbow_wristPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        //middle point between the wrist and the hand
//        std::transform(wristPos_delta.begin(),wristPos_delta.end(),handPos_delta.begin(),wrist_handPos_delta.begin(),plus<double>());
//        std::transform(wrist_handPos_delta.begin(), wrist_handPos_delta.end(), wrist_handPos_delta.begin(), std::bind1st(std::multiplies<double>(),0.5));
//        // points on the arm
//        points_arm_delta.at(0) = shoulder_elbowPos_delta;
//        points_arm_delta.at(1) = elbowPos_delta;
//        points_arm_delta.at(2) = elbow_wristPos_delta;
//        points_arm_delta.at(3) = wristPos_delta;
//        points_arm_delta.at(4) = wrist_handPos_delta;


//        vector<double> e_arm = vector<double>(5,0.0);
//        vector<double> e_arm_delta = vector<double>(5,0.0);
//        vector<double> e_arm_torso = vector<double>(5,0.0);
//        vector<double> e_arm_torso_delta = vector<double>(5,0.0);
//        // current obstacles in the scenario
//        for(size_t i=0;i<obsts.size();++i){
//            objectPtr obst = obsts.at(i);
//            vector<double> obst_pos = vector<double>(3);
//            vector<double> obst_or = vector<double>(3);
//            obst_pos.at(0) = obst->getPos().Xpos;
//            obst_pos.at(1) = obst->getPos().Ypos;
//            obst_pos.at(2) = obst->getPos().Zpos;
//            obst_or.at(0) = obst->getOr().roll;
//            obst_or.at(1) = obst->getOr().pitch;
//            obst_or.at(2) = obst->getOr().yaw;
//            Matrix3d Rot_obst; this->Rot_matrix(Rot_obst,obst_or);
//            Matrix3d Rot_obst_t = Rot_obst.transpose();
//            double x_size = obst->getSize().Xsize;
//            double y_size = obst->getSize().Ysize;
//            double z_size = obst->getSize().Zsize;
//            Matrix3d A_obst;
//            A_obst(0,0) = pow(x_size,-2); A_obst(0,1) = 0.0; A_obst(0,2) = 0.0;
//            A_obst(1,0) = 0.0; A_obst(1,1) = pow(y_size,-2); A_obst(1,2) = 0.0;
//            A_obst(2,0) = 0.0; A_obst(2,1) = 0.0; A_obst(2,2) = pow(z_size,-2);
//            Matrix3d L_obst = Rot_obst_t*A_obst*Rot_obst;

//            // distances between the arm and the obstacles
//            vector<double> dist_arm = vector<double>(5); vector<double> e_dist_arm = vector<double>(5);
//            vector<double> dist_arm_delta = vector<double>(5); vector<double> e_dist_arm_delta = vector<double>(5);
//            // get the distances between the current obstacle and the points on the arm
//            this->get_distances_arm_obstacles(points_arm,obst_pos,L_obst,dist_arm);
//            // get the distances between the current obstacle and the points on the arm with perturbated posture
//            this->get_distances_arm_obstacles(points_arm_delta,obst_pos,L_obst,dist_arm_delta);

//            // distances between the arm and the torso of the humanoid
//            vector<double> dist_arm_torso = vector<double>(5); vector<double> e_dist_arm_torso = vector<double>(5);
//            vector<double> dist_arm_torso_delta = vector<double>(5); vector<double> e_dist_arm_torso_delta = vector<double>(5);
//            this->get_distances_arm_torso(points_arm,dist_arm_torso);
//            this->get_distances_arm_torso(points_arm_delta,dist_arm_torso_delta);

//            double k_off = 0.0001;// offset constant
//            for (size_t i=0; i<dist_arm.size();++i){
//                e_dist_arm.at(i) = 1/(dist_arm.at(i)+k_off);
//                e_dist_arm_delta.at(i) = 1/(dist_arm_delta.at(i)+k_off);
//                e_dist_arm_torso.at(i) = 1/(dist_arm_torso.at(i)+k_off);
//                e_dist_arm_torso_delta.at(i) = 1/(dist_arm_torso_delta.at(i)+k_off);
//            }

//            for(size_t i=0;i<e_arm.size();++i){
//                e_arm.at(i) += e_dist_arm.at(i);
//                e_arm_delta.at(i) += e_dist_arm_delta.at(i);
//                e_arm_torso.at(i) += e_dist_arm_torso.at(i);
//                e_arm_torso_delta.at(i) += e_dist_arm_torso_delta.at(i);
//            }
//        } //for loop obstacles

//        MatrixXd Id = MatrixXd::Identity(JOINTS_ARM,JOINTS_ARM);
//        MatrixXd Jpp = J_plus*JacobianArm;
//        MatrixXd J_Null = Id - Jpp;

//        // ---------- torso ----------------------------------------

//        // shoulder - elbow
//        VectorXd delta_H_torso_SE(posture.size());
//        for(size_t i=0;i<delta_H_torso_SE.size();++i){
//            delta_H_torso_SE(i) = (e_arm_torso_delta.at(0) - e_arm_torso.at(0))/delta_theta;
//        }
//        VectorXd J_torso_SE= J_Null*delta_H_torso_SE;
//        double k_torso_SE = 0;
//        if(J_torso_SE.norm() > null_th){
//            k_torso_SE = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_SE.norm())));
//        }
//        double fd_torso_SE = obst_coeff * (1 - exp(-obst_damping*(J_torso_SE.norm())));
//        joint_velocities +=  (k_torso_SE*fd_torso_SE*J_torso_SE);

//        // elbow
//        VectorXd delta_H_torso_E(posture.size());
//        for(size_t i=0;i<delta_H_torso_E.size();++i){
//            delta_H_torso_E(i) = (e_arm_torso_delta.at(1) - e_arm_torso.at(1))/delta_theta;
//        }
//        VectorXd J_torso_E= J_Null*delta_H_torso_E;
//        double k_torso_E = 0;
//        if(J_torso_E.norm() > null_th){
//            k_torso_E = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_E.norm())));
//        }
//        double fd_torso_E = obst_coeff * (1 - exp(-obst_damping*(J_torso_E.norm())));
//        joint_velocities +=  (k_torso_E*fd_torso_E*J_torso_E);

//        // elbow - wrist
//        VectorXd delta_H_torso_EW(posture.size());
//        for(size_t i=0;i<delta_H_torso_EW.size();++i){
//            delta_H_torso_EW(i) = (e_arm_torso_delta.at(2) - e_arm_torso.at(2))/delta_theta;
//        }
//        VectorXd J_torso_EW= J_Null*delta_H_torso_EW;
//        double k_torso_EW = 0;
//        if(J_torso_EW.norm() > null_th){
//            k_torso_EW = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_EW.norm())));
//        }
//        double fd_torso_EW = obst_coeff * (1 - exp(-obst_damping*(J_torso_EW.norm())));
//        joint_velocities +=  (k_torso_EW*fd_torso_EW*J_torso_EW);

//        // wrist
//        VectorXd delta_H_torso_W(posture.size());
//        for(size_t i=0;i<delta_H_torso_W.size();++i){
//            delta_H_torso_W(i) = (e_arm_torso_delta.at(3) - e_arm_torso.at(3))/delta_theta;
//        }
//        VectorXd J_torso_W= J_Null*delta_H_torso_W;
//        double k_torso_W = 0;
//        if(J_torso_W.norm() > null_th){
//            k_torso_W = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_W.norm())));
//        }
//        double fd_torso_W = obst_coeff * (1 - exp(-obst_damping*(J_torso_W.norm())));
//        joint_velocities +=  (k_torso_W*fd_torso_W*J_torso_W);

//        // wrist - hand
//        VectorXd delta_H_torso_WH(posture.size());
//        for(size_t i=0;i<delta_H_torso_WH.size();++i){
//            delta_H_torso_WH(i) = (e_arm_torso_delta.at(4) - e_arm_torso.at(4))/delta_theta;
//        }
//        VectorXd J_torso_WH= J_Null*delta_H_torso_WH;
//        double k_torso_WH = 0;
//        if(J_torso_WH.norm() > null_th){
//            k_torso_WH = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_torso_WH.norm())));
//        }
//        double fd_torso_WH = obst_coeff * (1 - exp(-obst_damping*(J_torso_WH.norm())));
//        joint_velocities +=  (k_torso_WH*fd_torso_WH*J_torso_WH);

//        // ---------- obstacles --------------------------------

//        // shoulder - elbow
//        VectorXd delta_H_obst_SE(posture.size());
//        for(size_t i=0;i<delta_H_obst_SE.size();++i){
//            delta_H_obst_SE(i) = (e_arm_delta.at(0) - e_arm.at(0))/delta_theta;
//        }
//        VectorXd J_obst_SE= J_Null*delta_H_obst_SE;
//        double k_obst_SE = 0;
//        if(J_obst_SE.norm() > null_th){
//            k_obst_SE = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_SE.norm())));
//        }
//        double fd_SE = obst_coeff * (1 - exp(-obst_damping*(J_obst_SE.norm())));
//        joint_velocities +=  (k_obst_SE*fd_SE*J_obst_SE);

//        // elbow
//        VectorXd delta_H_obst_E(posture.size());
//        for(size_t i=0;i<delta_H_obst_E.size();++i){
//            delta_H_obst_E(i) = (e_arm_delta.at(1) - e_arm.at(1))/delta_theta;
//        }
//        VectorXd J_obst_E= J_Null*delta_H_obst_E;
//        double k_obst_E = 0;
//        if(J_obst_E.norm() > null_th){
//            k_obst_E = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_E.norm())));
//        }
//        double fd_E = obst_coeff * (1 - exp(-obst_damping*(J_obst_E.norm())));
//        joint_velocities +=  (k_obst_E*fd_E*J_obst_E);

//        // elbow - wrist
//        VectorXd delta_H_obst_EW(posture.size());
//        for(size_t i=0;i<delta_H_obst_EW.size();++i){
//            delta_H_obst_EW(i) = (e_arm_delta.at(2) - e_arm.at(2))/delta_theta;
//        }
//        VectorXd J_obst_EW= J_Null*delta_H_obst_EW;
//        double k_obst_EW = 0;
//        if(J_obst_EW.norm() > null_th){
//            k_obst_EW = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_EW.norm())));
//        }
//        double fd_EW = obst_coeff * (1 - exp(-obst_damping*(J_obst_EW.norm())));
//        joint_velocities +=  (k_obst_EW*fd_EW*J_obst_EW);

//        // wrist
//        VectorXd delta_H_obst_W(posture.size());
//        for(size_t i=0;i<delta_H_obst_W.size();++i){
//            delta_H_obst_W(i) = (e_arm_delta.at(3) - e_arm.at(3))/delta_theta;
//        }
//        VectorXd J_obst_W= J_Null*delta_H_obst_W;
//        double k_obst_W = 0;
//        if(J_obst_W.norm() > null_th){
//            k_obst_W = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_W.norm())));
//        }
//        double fd_W = obst_coeff * (1 - exp(-obst_damping*(J_obst_W.norm())));
//        joint_velocities +=  (k_obst_W*fd_W*J_obst_W);

//        // wrist - hand
//        VectorXd delta_H_obst_WH(posture.size());
//        for(size_t i=0;i<delta_H_obst_WH.size();++i){
//            delta_H_obst_WH(i) = (e_arm_delta.at(4) - e_arm.at(4))/delta_theta;
//        }
//        VectorXd J_obst_WH= J_Null*delta_H_obst_WH;
//        double k_obst_WH = 0;
//        if(J_obst_WH.norm() > null_th){
//            k_obst_WH = - ((vel_max_norm - (J_plus*hand_vel_xd).norm())/((J_Null.norm())*(J_obst_WH.norm())));
//        }
//        double fd_WH = obst_coeff * (1 - exp(-obst_damping*(J_obst_WH.norm())));
//        joint_velocities +=  (k_obst_WH*fd_WH*J_obst_WH);

    }


    accelerations.clear();
    accelerations.resize(joint_accelerations.size());
    VectorXd::Map(&accelerations[0], joint_accelerations.size()) = joint_accelerations;
}


void Humanoid::getHandAcceleration(int arm,MatrixXd &joint_traj_pos,MatrixXd &joint_traj_vel,MatrixXd &joint_traj_acc,vector<double> timesteps,vector<vector<double>> &hand_lin_acc,vector<vector<double>> &hand_ang_acc)
{
    vector<MatrixXd> jac_vec(joint_traj_pos.rows());
    vector<MatrixXd> der_jac_vec(joint_traj_pos.rows());
    for(int r=0;r<joint_traj_pos.rows();++r)
    {
        VectorXd posture_tot = joint_traj_pos.row(r);
        VectorXd arm_posture_vec = posture_tot.head<JOINTS_ARM>();
        vector<double> arm_posture;  arm_posture.resize(arm_posture_vec.size());
        VectorXd::Map(&arm_posture[0], arm_posture_vec.size()) = arm_posture_vec;
        this->getJacobian(arm,arm_posture,jac_vec.at(r));
    }
    this->getDerivative(jac_vec,timesteps,der_jac_vec);

    //TO DO

}

void Humanoid::get_distances_arm_torso(vector<vector<double>>& points_arm,vector<double>& dist_arm)
{
    // size of the torso
    double torso_x_size = this->m_torso_size.Xsize;
    double torso_y_size = this->m_torso_size.Ysize;

    for(size_t i=0;i<points_arm.size();++i){
        vector<double> pt = points_arm.at(i);
        double x = pt.at(0); double y = pt.at(1);
        dist_arm.at(i) = sqrt(pow(x-torso_x_size,2)+pow(y-torso_y_size,2));
    }
}

void Humanoid::get_distances_arm_obstacles(vector<vector<double>>& points_arm,vector<double>& obst_pos,Matrix3d& L_obst,vector<double>& dist_arm)
{
    // see octave workspace obstacle_avoidance
    Vector3d sol_1; Vector3d sol_2;


    double xc,yc,zc; // coordinates of the center of the obstacles
    double l11,l12,l13,l21,l22,l23,l31,l32,l33; // elements of the matrix R^t*A*R
    double xp,yp,zp; // coordinates of the point on the arm

    xc = obst_pos.at(0); yc = obst_pos.at(1); zc = obst_pos.at(2);
    l11 = L_obst(0,0); l12 = L_obst(0,1); l13 = L_obst(0,2);
    l21 = L_obst(1,0); l22 = L_obst(1,1); l23 = L_obst(1,2);
    l31 = L_obst(2,0); l32 = L_obst(2,1); l33 = L_obst(2,2);


    for(size_t i=0;i<points_arm.size();++i){
        vector<double> pt = points_arm.at(i);
        xp = pt.at(0); yp = pt.at(1); zp = pt.at(2);
        Vector3d pt_arm(xp,yp,zp);

        // sol 1: x
        //        __________________________________________________________________________________________________________________________________________________________________________________________________________
        //       ╱       2                       2                                                                                                                                                         2
        //  xc⋅╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅yp +
        //  ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //            ______________________________________________________________________________________________________________________________________________________________________________________________________
        //           ╱       2                       2                                                                                                                                                         2
        //         ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅

        //  ___________________________________________________________________________________________________________________________________________________________________________________________
        //         2                                                                                                                                                         2                       2
        //   l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp   - xc + xp
        //  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //  _______________________________________________________________________________________________________________________________________________________________________________________________
        //             2                                                                                                                                                         2                       2
        //  yp + l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp
        sol_1(0) = (xc*sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2))-xc+xp)/
                (sqrt(l11*pow(xc,2)-2*l11*xc*xp +l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)));

        // sol 1: y
        //                           ⎛
        //-yc⋅zp + yp⋅zc + (yc - yp)⋅⎜zc - ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //                           ⎜        ____________________________________________________________________________________________________________________________________________________________________________
        //                           ⎜       ╱       2                       2
        //                           ⎝     ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅x
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


        //zc - zp
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //________________________________________________________________________________________________________________________________________________________________________________________________________________
        //             2                       2                                                                                                                                                         2
        //p⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅yp + l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp +
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //zc - zp

        //         ⎞
        //─────────⎟
        //_________⎟
        //       2 ⎟
        //l₃₃⋅zp   ⎠
        //──────────
        sol_1(1) = (-yc*zp+yp*zc+(yc-yp)*(zc - (zc-zp)/(sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+
                                                             l22*pow(yc,2)-2*l22*yc*yp+l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)))))/(zc-zp);

        // sol 1: z
        //        zc - zp
        //zc - ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //     ________________________________________________________________________________________________________________________________________________________________________________________________________
        //    ╱       2                       2                                                                                                                                                         2
        //  ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅yp


        //─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //_____________________________________________________________________________________________________________________________________________________________________________________________
        //        2                                                                                                                                                         2                       2
        //+ l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp
        sol_1(2) = zc-(zc-zp)/(sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+
                                    l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)));

        // sol 2: x
        //        __________________________________________________________________________________________________________________________________________________________________________________________________________
        //       ╱       2                       2                                                                                                                                                         2
        //  xc⋅╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅yp +
        //  ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //            ______________________________________________________________________________________________________________________________________________________________________________________________________
        //           ╱       2                       2                                                                                                                                                         2
        //         ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅

        //  ___________________________________________________________________________________________________________________________________________________________________________________________
        //         2                                                                                                                                                         2                       2
        //   l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp   + xc - xp
        //  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //  _______________________________________________________________________________________________________________________________________________________________________________________________
        //             2                                                                                                                                                         2                       2
        //  yp + l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp
        sol_2(0) = (xc*sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*pow(yc,2)+l12*xp*yp*l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+
                            l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2))+xc-xp)
                /(sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+
                       l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)));

        // sol 2: y
        //                           ⎛
        //-yc⋅zp + yp⋅zc + (yc - yp)⋅⎜zc + ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //                           ⎜        ____________________________________________________________________________________________________________________________________________________________________________
        //                           ⎜       ╱       2                       2
        //                           ⎝     ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅x
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


        //zc - zp
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //________________________________________________________________________________________________________________________________________________________________________________________________________________
        //             2                       2                                                                                                                                                         2
        //p⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅yp + l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp +
        //────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //zc - zp

        //         ⎞
        //─────────⎟
        //_________⎟
        //       2 ⎟
        //l₃₃⋅zp   ⎠
        //──────────
        sol_2(1) = (-yc*zp+yp*zc+(yc-yp)*(zc+(zc-zp)/(sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+
                                                           l22*pow(yc,2)-2*l22*yc*yp+l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)))))/(zc-zp);

        // sol 2: z
        //        (zc - zp)
        //zc + ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //         ______________________________________________________________________________________________________________________________________________________________________________________________________
        //        ╱       2                       2                                                                                                                                                         2
        //      ╲╱  l₁₁⋅xc  - 2⋅l₁₁⋅xc⋅xp + l₁₁⋅xp  + l₁₂⋅xc⋅yc - l₁₂⋅xc⋅yp - l₁₂⋅xp⋅yc + l₁₂⋅xp⋅yp + l₁₃⋅xc⋅zc - l₁₃⋅xc⋅zp - l₁₃⋅xp⋅zc + l₁₃⋅xp⋅zp + l₂₁⋅xc⋅yc - l₂₁⋅xc⋅yp - l₂₁⋅xp⋅yc + l₂₁⋅xp⋅yp + l₂₂⋅yc  - 2⋅l₂₂⋅yc⋅


        //───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
        //_______________________________________________________________________________________________________________________________________________________________________________________________
        //           2                                                                                                                                                         2                       2
        //yp + l₂₂⋅yp  + l₂₃⋅yc⋅zc - l₂₃⋅yc⋅zp - l₂₃⋅yp⋅zc + l₂₃⋅yp⋅zp + l₃₁⋅xc⋅zc - l₃₁⋅xc⋅zp - l₃₁⋅xp⋅zc + l₃₁⋅xp⋅zp + l₃₂⋅yc⋅zc - l₃₂⋅yc⋅zp - l₃₂⋅yp⋅zc + l₃₂⋅yp⋅zp + l₃₃⋅zc  - 2⋅l₃₃⋅zc⋅zp + l₃₃⋅zp
        sol_2(2) = zc + (zc-zp)/(sqrt(l11*pow(xc,2)-2*l11*xc*xp+l11*pow(xp,2)+l12*xc*yc-l12*xc*yp-l12*xp*yc+l12*xp*yp+l13*xc*zc-l13*xc*zp-l13*xp*zc+l13*xp*zp+l21*xc*yc-l21*xc*yp-l21*xp*yc+l21*xp*yp+l22*pow(yc,2)-2*l22*yc*yp+
                                      l22*pow(yp,2)+l23*yc*zc-l23*yc*zp-l23*yp*zc+l23*yp*zp+l31*xc*zc-l31*xc*zp-l31*xp*zc+l31*xp*zp+l32*yc*zc-l32*yc*zp-l32*yp*zc+l32*yp*zp+l33*pow(zc,2)-2*l33*zc*zp+l33*pow(zp,2)));


        Vector3d dist_vec_1 = pt_arm - sol_1; double dist_1 = dist_vec_1.norm();
        Vector3d dist_vec_2 = pt_arm - sol_2; double dist_2 = dist_vec_2.norm();
        double dist_obst = std::min(dist_1,dist_2);

        dist_arm.at(i) = dist_obst;

    }// for loop points on the arm

}

void Humanoid::transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T)
{
    T = Matrix4d::Zero();

    T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha); T(1,1) = cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
    T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;

}

bool Humanoid::getRPY(std::vector<double>& rpy, Matrix3d& Rot)
{
    if((Rot.cols()==3) && (Rot.rows()==3))
    {// the matrix is not empy
        rpy.resize(3,0);

//        Vector3d rpy_vec = Rot.eulerAngles(0,1,2);
//        rpy.at(0) = rpy_vec(0); // roll
//        rpy.at(1) = rpy_vec(1); // pitch
//        rpy.at(2) = rpy_vec(2); // yaw
//        return true;

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


void Humanoid::getQuaternion(std::vector<double>& q, Matrix3d& Rot)
{
    Quaterniond qq(Rot);
    q.clear();
    q.push_back(qq.x());
    q.push_back(qq.y());
    q.push_back(qq.z());
    q.push_back(qq.w());

}

void Humanoid::Rot_matrix(Matrix3d &Rot,std::vector<double>& rpy)
{
    Rot = Matrix3d::Zero();

    double roll = rpy.at(0); // around z
    double pitch = rpy.at(1); // around y
    double yaw = rpy.at(2); // around x

    // Rot = Rot_z * Rot_y * Rot_x

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

}

void Humanoid::Rot_matrix_q(Matrix3d &Rot,std::vector<double>& qq)
{
    Quaterniond q;
    q.x() = qq.at(0);
    q.y() = qq.at(1);
    q.z() = qq.at(2);
    q.w() = qq.at(3);

    Rot = q.normalized().toRotationMatrix();
}


void Humanoid::directKinematicsFinger(DHparams& p, Matrix4d& T_ext, Matrix4d& T_H_0_pos, int id_fing, MatrixXd& Fingers)
{



     Matrix4d T;
     vector<double> pos = vector<double>(3);
     Matrix4d T_aux;

     for(int i =0; i<T_aux.rows();++i){
         for(int j=0; j<T_aux.cols();++j){
             T_aux(i,j)=T_ext(i,j);
         }
     }

     // translate to the begenning of each finger
     T_aux = T_aux * T_H_0_pos;

#if HAND == 0
     int cnt;
     if (id_fing == 3){
         // thumb
         cnt = N_PHALANGE+2;
     }else{
         cnt = N_PHALANGE+1;
     }
     for (int i=0; i< cnt; ++i){
#elif HAND == 1

     for (int i=0; i< N_PHALANGE+1; ++i){
#endif

         double a = p.a.at(i);
         double d = p.d.at(i);
         double alpha = p.alpha.at(i);
         double theta = p.theta.at(i);

#if HAND == 0

         T(0,0) = cos(theta); T(0,1) = -sin(theta)*cos(alpha);  T(0,2) = sin(theta)*cos(alpha);  T(0,3) = a*cos(theta);
         T(1,0) = sin(theta); T(1,1) = cos(theta)*cos(alpha);   T(1,2) = -cos(theta)*sin(alpha); T(1,3) = a*sin(theta);
         T(2,0) = 0.0;        T(2,1) = sin(alpha);              T(2,2) = cos(alpha);             T(2,3) = d;
         T(3,0) = 0.0;        T(3,1) = 0.0;                     T(3,2) = 0.0;                    T(3,3) = 1.0;

#elif HAND == 1

         T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
         T(1,0) = sin(theta)*cos(alpha); T(1,1) = -cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
         T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
         T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;

#endif


         T_aux = T_aux * T;

         pos[0] = T_aux(0,3);
         pos[1] = T_aux(1,3);
         pos[2] = T_aux(2,3);

         Fingers(id_fing,3*i) = pos[0]; Fingers(id_fing,3*i+1) = pos[1]; Fingers(id_fing,3*i+2) = pos[2];

     }




}



} // namespace motion_manager
