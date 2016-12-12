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
    directDiffKinematicsSingleArm(1,posture,velocities,vel);
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
    directDiffKinematicsSingleArm(2,posture,velocities,vel);
}

double Humanoid::getLeftHandVelNorm()
{
    std::vector<double> hand_vel;

    this->getLeftHandVel(hand_vel);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}


void Humanoid::getHandPos(int arm, vector<double> &pos, vector<double> &posture)
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

    for (size_t i = 0; i < posture.size(); ++i){
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
        }else if (i==2){
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
        }else if (i==4){
            // get the wrist
            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
        } else if (i==6){
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
        }

    }

    pos.clear();
    pos.push_back(handPos[0]);
    pos.push_back(handPos[1]);
    pos.push_back(handPos[2]);

}

void Humanoid::getHandVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel);
}

double Humanoid::getHandVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> hand_vel;

    this->getHandVel(arm,hand_vel,posture,velocities);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
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
    m_DH_leftArm.d.push_back(-m_arm_specs.arm_specs.d.at(i));

    //a [mm]
    m_DH_leftArm.a.push_back(m_arm_specs.arm_specs.a.at(i));

    //alpha [rad]
    if ((i == 0)){
        m_DH_leftArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));
    }else{
        m_DH_leftArm.alpha.push_back(-m_arm_specs.arm_specs.alpha.at(i));
    }

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

void Humanoid::directDiffKinematicsSingleArm(int arm,vector<double> posture, vector<double> velocities, vector<double>& hand_vel)
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
    VectorXd hand_vel_xd = JacobianArm*joint_velocities;
    hand_vel.clear();
    hand_vel.resize(hand_vel_xd.size());
    VectorXd::Map(&hand_vel[0], hand_vel_xd.size()) = hand_vel_xd;

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
    MatrixXd JT = JacobianArmT*(JJ+pow(k,2)*I);
    VectorXd hand_vel_xd(6);
    hand_vel_xd << hand_vel.at(0),hand_vel.at(1),hand_vel.at(2),hand_vel.at(3),hand_vel.at(4),hand_vel.at(5);
    VectorXd joint_velocities = JT*hand_vel_xd;
    velocities.clear();
    velocities.resize(joint_velocities.size());
    VectorXd::Map(&velocities[0], joint_velocities.size()) = joint_velocities;

}

void Humanoid::transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T)
{
    T = Matrix4d::Zero();

    T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha); T(1,1) = -cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
    T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;



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
