#ifndef HUMANOID_HPP
#define HUMANOID_HPP

#include "common.hpp"

namespace motion_manager{

//! The Humanoid class
/**
 * @brief This class defines the concept of a humanoid robot
 */
class Humanoid
{

public:

#if HAND==0
    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             vector<double>& r, vector<double>& l);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     * @param min_rl
     * @param max_rl
     * @param min_ll
     * @param max_ll
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             vector<double>& r, vector<double>& l,
             vector<double>& min_rl, vector<double>& max_rl,
             vector<double>& min_ll, vector<double>& max_ll);
#elif HAND==1
    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             vector<double>& r, vector<double>& l);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     * @param min_rl
     * @param max_rl
     * @param min_ll
     * @param max_ll
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             vector<double>& r, vector<double>& l,
             vector<double>& min_rl, vector<double>& max_rl,
             vector<double>& min_ll, vector<double>& max_ll);
#endif

    /**
     * @brief Humanoid, a copy constructor
     * @param hh
     */
    Humanoid(const Humanoid& hh);

    /**
     * @brief ~Humanoid, a destructor
     */
    ~Humanoid();

    /**
     * @brief This method sets the name of the humanoid
     * @param name
     */
    void setName(string& name);

    /**
     * @brief This method sets the position of the humanoid as the position of its torso
     * @param ppos
     */
    void setPos(pos& ppos);

    /**
     * @brief This method sets the orientation of the humanoid as the orientation of its torso
     * @param oor
     */
    void setOr(orient& oor);

    /**
     * @brief This method sets the size of the humanoid as the size of its torso
     * @param ssize
     */
    void setSize(dim& ssize);

    /**
     * @brief This method sets the specifications of the arms
     * @param specs
     */
    void setArm(arm& specs);
#if HAND==0

    /**
     * @brief This method sets the specifications of the hand
     * @param specs
     */
    void setHumanHand(human_hand& specs);
#elif HAND==1

    /**
     * @brief This method sets the specifications of the hand
     * @param specs
     */
    void setBarrettHand(barrett_hand& specs);
#endif

    /**
     * @brief This method sets the posture of the right arm
     * @param r
     */
    void setRightPosture(vector<double>& r);

    /**
     * @brief This method sets the posture of the left arm
     * @param l
     */
    void setLeftPosture(vector<double>& l);

    /**
     * @brief This method sets the home posture of the right arm
     * @param r
     */
    void setRightHomePosture(vector<double>& r);

    /**
     * @brief This method sets the home posture of the left arm
     * @param l
     */
    void setLeftHomePosture(vector<double>& l);

    /**
     * @brief This method sets the joint velocities of the right arm
     * @param r
     */
    void setRightVelocities(vector<double>& r);

    /**
     * @brief This method sets the joint velocities of the left arm
     * @param l
     */
    void setLeftVelocities(vector<double>& l);

    /**
     * @brief This method sets the joint forces of the right arm
     * @param r
     */
    void setRightForces(vector<double>& r);

    /**
     * @brief This method sets the joint forces of the left arm
     * @param l
     */
    void setLeftForces(vector<double>& l);

    /**
     * @brief This method sets the minimum joint limits of the right arm
     * @param min_rl
     */
    void setRightMinLimits(vector<double>& min_rl);

    /**
     * @brief This method sets the maximum joint limits of the right arm
     * @param max_rl
     */
    void setRightMaxLimits(vector<double>& max_rl);

    /**
     * @brief This method sets the minimum joint limits of the left arm
     * @param min_ll
     */
    void setLeftMinLimits(vector<double>& min_ll);

    /**
     * @brief This method sets the maximum joint limits of the left arm
     * @param max_ll
     */
    void setLeftMaxLimits(vector<double>& max_ll);

    /**
     * @brief This method sets the attribute mat_right
     * @param m
     */
    void setMatRight(Matrix4d& m);

    /**
     * @brief This method sets the attribute mat_left
     * @param m
     */
    void setMatLeft(Matrix4d& m);

    /**
     * @brief This method sets the attribute mat_r_hand
     * @param m
     */
    void setMatRightHand(Matrix4d& m);

    /**
     * @brief This method sets the attribute mat_l_hand
     * @param m
     */
    void setMatLeftHand(Matrix4d& m);

    // humanoid parts
//#if HEAD==1
  //  void setHead(humanoid_part& head);
//#endif
//#if NECK==1
  //  void setNeck(humanoid_part& neck);
//#endif
//#if PELVIS==1
  //  void setPelvis(humanoid_part& pelvis);
//#endif
//#if RIGHT_UPPER_LEG==1
  //  void setRight_Upper_leg(humanoid_part& right_upper_leg);
//#endif
//#if RIGHT_LOWER_LEG==1
  //  void setRight_Lower_leg(humanoid_part& right_lower_leg);
//#endif
//#if RIGHT_FOOT==1
  //  void setRight_foot(humanoid_part& right_foot);
//#endif
//#if LEFT_UPPER_LEG==1
  //  void setLeft_Upper_leg(humanoid_part& left_upper_leg);
//#endif
//#if LEFT_LOWER_LEG==1
  //  void setLeft_Lower_leg(humanoid_part& left_lower_leg);
//#endif
//#if LEFT_FOOT==1
  //  void setLeft_foot(humanoid_part& left_foot);
//#endif

    /**
     * @brief This method gets the name of the humanoid
     * @return
     */
    string getName();

    /**
     * @brief This method gets the position of the humanoid
     * @return
     */
    pos getPos();

    /**
     * @brief This method gets the orientation of the humanoid
     * @return
     */
    orient getOr();

    /**
     * @brief This method gets the size of the humanoid
     * @return
     */
    dim getSize();

    /**
     * @brief This method gets the arm specifications of the humanoid
     * @return
     */
    arm getArm();
#if HAND==0

    /**
     * @brief This method gets hand specifications of the humanoid
     * @return
     */
    human_hand getHumanHand();
#elif HAND==1
    /**
     * @brief This method gets the hand specifications of the humanoid
     * @return
     */
    barrett_hand getBarrettHand();

    /**
     * @brief This method gets the r parameters of the hand of the humanoid
     * @param rk
     */
    void getRK(vector<int>& rk);

    /**
     * @brief This method gets the j parameters of the hand of the humanoid
     * @param jk
     */
    void getJK(vector<int>& jk);
#endif

    /**
     * @brief This method gets the current posture of the right arm+hand
     * @param p
     */
    void getRightPosture(vector<double>& p);

    /**
     * @brief This method gets the current posture of the right arm
     * @param p
     */
    void getRightArmPosture(vector<double>& p);

    /**
     * @brief This method gets the current posture of the right hand
     * @param p
     */
    void getRightHandPosture(vector<double>& p);

    /**
     * @brief This method gets the current posture of the left arm+hand
     * @param p
     */
    void getLeftPosture(vector<double>& p);

    /**
     * @brief This method gets the current posture of the left arm
     * @param p
     */
    void getLeftArmPosture(vector<double>& p);

    /**
     * @brief This method gets the current posture of the left hand
     * @param p
     */
    void getLeftHandPosture(vector<double>& p);

    /**
     * @brief This method gets the home posture of the right arm+hand
     * @param p
     */
    void getRightHomePosture(vector<double>& p);

    /**
     * @brief getRightHandHomePosture
     * @param p
     */
    void getRightHandHomePosture(vector<double>& p);

    /**
     * @brief getRightArmHomePosture
     * @param p
     */
    void getRightArmHomePosture(vector<double>& p);

    /**
     * @brief This method gets the home posture of the left arm+hand
     * @param p
     */
    void getLeftHomePosture(vector<double>& p);

    /**
     * @brief getLeftHandHomePosture
     * @param p
     */
    void getLeftHandHomePosture(vector<double>& p);

    /**
     * @brief getLeftArmHomePosture
     * @param p
     */
    void getLeftArmHomePosture(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the right arm+hand
     * @param p
     */
    void getRightVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the right arm
     * @param p
     */
    void getRightArmVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the right hand
     * @param p
     */
    void getRightHandVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the left arm+hand
     * @param p
     */
    void getLeftVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the left arm
     * @param p
     */
    void getLeftArmVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint velocities of the left hand
     * @param p
     */
    void getLeftHandVelocities(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the right arm+hand
     * @param p
     */
    void getRightForces(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the right arm+hand
     * @param p
     */
    void getRightArmForces(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the right hand
     * @param p
     */
    void getRightHandForces(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the left arm+hand
     * @param p
     */
    void getLeftForces(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the left hand
     * @param p
     */
    void getLeftArmForces(vector<double>& p);

    /**
     * @brief This method gets the joint forces of the left hand
     * @param p
     */
    void getLeftHandForces(vector<double>& p);

    /**
     * @brief This method gets the minimum joint limits of the right arm+hand
     * @param p
     */
    void getRightMinLimits(vector<double>& p);

    /**
     * @brief This method gets the maximum joint limits of the right arm+hand
     * @param p
     */
    void getRightMaxLimits(vector<double>& p);

    /**
     * @brief This method gets the minimum joint limits of the left arm+hand
     * @param p
     */
    void getLeftMinLimits(vector<double>& p);

    /**
     * @brief This method gets the maximum joint limits of the left arm+hand
     * @param p
     */
    void getLeftMaxLimits(vector<double>& p);

    /**
     * @brief This method gets the attribute mat_right
     * @param m
     */
    void getMatRight(Matrix4d& m);

    /**
     * @brief This method gets the attribute mat_left
     * @param m
     */
    void getMatLeft(Matrix4d& m);

    /**
     * @brief This method gets the attribute mat_r_hand
     * @param m
     */
    void getMatRightHand(Matrix4d& m);

    /**
     * @brief This method gets the attribute mat_l_hand
     * @param m
     */
    void getMatLeftHand(Matrix4d& m);


    /**
     * @brief This method gets the position of the right shoulder
     * @param pos
     */
    void getRightShoulderPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right shoulder
     * @return
     */
    double getRightShoulderNorm();

    /**
     * @brief This method gets the orientation of the right shoulder
     * @param orr
     */
    void getRightShoulderOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the right elbow
     * @param pos
     */
    void getRightElbowPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right elbow
     * @return
     */
    double getRightElbowNorm();

    /**
     * @brief This method gets the orientation of the right elbow
     * @param orr
     */
    void getRightElbowOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the right wrist
     * @param pos
     */
    void getRightWristPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right wrist
     * @return
     */
    double getRightWristNorm();

    /**
     * @brief This method gets the orientation of the right wrist
     * @param orr
     */
    void getRightWristOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the right hand
     * @param pos
     */
    void getRightHandPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right hand
     * @return
     */
    double getRightHandNorm();

    /**
     * @brief This method gets the orientation of the right hand
     * @param orr
     */
    void getRightHandOr(Matrix3d& orr);

    /**
     * @brief getRightHandVel
     * @param vel
     */
    void getRightHandVel(vector<double>& vel);

    /**
     * @brief getRightHandVelNorm
     * @return
     */
    double getRightHandVelNorm();


    /**
     * @brief This method gets the position of the left shoulder
     * @param pos
     */
    void getLeftShoulderPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left shoulder
     * @return
     */
    double getLeftShoulderNorm();

    /**
     * @brief This method gets the orientation of the left shoulder
     * @param orr
     */
    void getLeftShoulderOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the left elbow
     * @param pos
     */
    void getLeftElbowPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left elbow
     * @return
     */
    double getLeftElbowNorm();

    /**
     * @brief This method gets the orientation of the left elbow
     * @param orr
     */
    void getLeftElbowOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the left wrist
     * @param pos
     */
    void getLeftWristPos(vector<double>& pos);

    /**
     * @brief This method gets the norm pointing to the left wrist
     * @return
     */
    double getLeftWristNorm();
    /**
     * @brief This method gets the orientation of the left wrist
     * @param orr
     */
    void getLeftWristOr(Matrix3d& orr);

    /**
     * @brief This method gets the position of the left hand
     * @param pos
     */
    void getLeftHandPos(vector<double>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left hand
     * @return
     */
    double getLeftHandNorm();

    /**
     * @brief This method gets the orientation of the left hand
     * @param orr
     */
    void getLeftHandOr(Matrix3d& orr);

    /**
     * @brief getLeftHandVel
     * @param vel
     */
    void getLeftHandVel(vector<double>& vel);

    /**
     * @brief getLeftHandVelNorm
     * @return
     */
    double getLeftHandVelNorm();


    /**
     * @brief getHandPos
     * @param arm
     * @param pos
     * @param posture
     */
    void getHandPos(int arm, vector<double>& pos, vector<double>& posture);

    /**
     * @brief getHandVel
     * @param arm
     * @param vel
     * @param posture
     * @param velocities
     */
    void getHandVel(int arm, vector<double>& vel, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getHandVelNorm
     * @param arm
     * @param posture
     * @param velocities
     * @return
     */
    double getHandVelNorm(int arm, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getWristVel
     * @param arm
     * @param vel
     * @param posture
     * @param velocities
     */
    void getWristVel(int arm, vector<double>& vel, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getWristVelNorm
     * @param arm
     * @param posture
     * @param velocities
     * @return
     */
    double getWristVelNorm(int arm, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getElbowVel
     * @param arm
     * @param vel
     * @param posture
     * @param velocities
     */
    void getElbowVel(int arm, vector<double>& vel, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getElbowVelNorm
     * @param arm
     * @param posture
     * @param velocities
     * @return
     */
    double getElbowVelNorm(int arm,vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getShoulderVel
     * @param arm
     * @param vel
     * @param posture
     * @param velocities
     */
    void getShoulderVel(int arm, vector<double>& vel, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief getShoulderVelNorm
     * @param arm
     * @param posture
     * @param velocities
     * @return
     */
    double getShoulderVelNorm(int arm, vector<double>& posture,vector<double>& velocities);

    /**
     * @brief This method gets information about the humanoid
     * @return
     */
    string getInfoLine();

    /**
     * @brief getDH_rightArm
     * @return
     */
    DHparams getDH_rightArm();

    /**
     * @brief getDH_leftArm
     * @return
     */
    DHparams getDH_leftArm();

//#if HEAD==1
  //  humanoid_part getHead();
//#endif
//#if NECK==1
  //  humanoid_part getNeck();
//#endif
//#if PELVIS==1
  //  humanoid_part getPelvis();
//#endif
//#if RIGHT_UPPER_LEG==1
  //  humanoid_part getRight_Upper_leg();
//#endif
//#if RIGHT_LOWER_LEG==1
  //  humanoid_part getRight_Lower_leg();
//#endif
//#if RIGHT_FOOT==1
  //  humanoid_part getRight_foot();
//#endif
//#if LEFT_UPPER_LEG==1
  //  humanoid_part getLeft_Upper_leg();
//#endif
//#if LEFT_LOWER_LEG==1
  //  humanoid_part getLeft_Lower_leg();
//#endif
//#if LEFT_FOOT==1
  //  humanoid_part getLeft_foot();
//#endif


private:

    string m_name; /**< name of the humanoid */
    pos m_torso_pos;/**< position of the torso */
    orient m_torso_or; /**< orientation of the torso */
    dim m_torso_size; /**< size of the torso */
//#if HEAD==1
  //  humanoid_part head;
//#endif
//#if NECK==1
  //  humanoid_part neck;
//#endif
//#if PELVIS==1
  //  humanoid_part pelvis;
//#endif
//#if RIGHT_UPPER_LEG==1
  //  humanoid_part right_upper_leg;
//#endif
//#if RIGHT_LOWER_LEG==1
  //  humanoid_part right_lower_leg;
//#endif
//#if RIGHT_FOOT==1
  //  humanoid_part right_foot;
//#endif
//#if LEFT_UPPER_LEG==1
  //  humanoid_part left_upper_leg;
//#endif
//#if LEFT_LOWER_LEG==1
  //  humanoid_part left_lower_leg;
//#endif
//#if LEFT_FOOT==1
  //  humanoid_part left_foot;
//#endif
    arm m_arm_specs; /**< specifications of the arm */
#if HAND==0
    human_hand m_human_hand_specs; /**< specifications of the hand */
#elif HAND==1
    barrett_hand m_barrett_hand_specs; /**< specifications of the hand */
    vector<int> rk; /**< r parameters of the barrett hand */
    vector<int> jk; /**< j parameters of the barrett hand */
#endif

    DHparams m_DH_rightArm; /**< current D-H parameters of the right arm */
    DHparams m_DH_leftArm; /**< current D-H parameters of the left arm */
    vector<DHparams> m_DH_rightHand; /**< current D-H parameters of the fingers on the right hand */
    vector< vector<double> > right_fing_pos; /**< current positions of the phalanges of the fingers on the right hand */
    vector<DHparams> m_DH_leftHand; /**< current D-H parameters of the fingers on the left hand */
    vector< vector<double> > left_fing_pos; /**< current positions of the phalanges of the fingers on the left hand */

    Matrix4d mat_right; /**< transformation matrix from the fixed world frame and the reference frame of the right arm (positions are in [mm]) */
    Matrix4d mat_left; /**< transformation matrix from the fixed world frame and the reference frame of the left arm (positions are in [mm]) */
    Matrix4d mat_r_hand; /**< trabsformation matrix from the last joint of the right arm and the palm of the right hand (positions are in [mm]) */
    Matrix4d mat_l_hand; /**< trabsformation matrix from the last joint of the left arm and the palm of the left hand (positions are in [mm]) */

    // joints [rad]: 7 joints + 4 joints for each arm (total: 22 joints)
    vector<double> rightPosture; /**< right arm+hand current posture */
    vector<double> leftPosture; /**< left arm+hand current posture */
    vector<double> rightHomePosture; /**< right arm+hand home posture */
    vector<double> leftHomePosture; /**< left arm+hand home posture */
    vector<double> min_rightLimits; /**< minimum right limits */
    vector<double> max_rightLimits; /**< maximum right limits */
    vector<double> min_leftLimits; /**< minimum left limits */
    vector<double> max_leftLimits; /**< maximum left limits */

    // joints velocities
    vector<double> rightVelocities; /**< right arm+hand current velocities */
    vector<double> leftVelocities; /**< left arm+hand current velocities */

    // joints forces
    vector<double> rightForces; /**< right arm+hand current forces */
    vector<double> leftForces; /**< left arm+hand current forces */

    // positions on the right arm
    vector<double> rightShoulderPos; /**< position of the right shoulder */
    vector<double> rightElbowPos; /**< position of the right elbow */
    vector<double> rightWristPos; /**< position of the right wrist */
    vector<double> rightHandPos; /**< position of the right hand */

    // orientations on the right arm
    Matrix3d rightShoulderOr; /**< orientation of the right shoulder */
    Matrix3d rightElbowOr; /**< orientation of the right elbow */
    Matrix3d rightWristOr; /**< orientation of the right wrist */
    Matrix3d rightHandOr; /**< orientation of the right hand */

    // positions on the right hand
    MatrixXd rightFingers; /**< positions of the phalanges of the fingers on the right hand */

    // positions on the left arm
    vector<double> leftShoulderPos; /**< position of the left shoulder */
    vector<double> leftElbowPos; /**< position of the left elbow */
    vector<double> leftWristPos; /**< position of the left wrist */
    vector<double> leftHandPos; /**< position of the left hand */

    // orientations on the left arm
    Matrix3d leftShoulderOr; /**< orientation of the left shoulder */
    Matrix3d leftElbowOr; /**< orientation of the left elbow */
    Matrix3d leftWristOr; /**< orientation of the left wrist */
    Matrix3d leftHandOr; /**< orientation of the left hand */

    //positions on the left hand
    MatrixXd leftFingers; /**< positions of the phalanges of the fingers on the left hand */


    /**
     * @brief This method computes the current D-H parameters of the right arm
     */
    void computeRightArmDHparams();

    /**
     * @brief This method computes the current D-H parameters of the left arm
     */
    void computeLeftArmDHparams();

    /**
     * @brief This method computes the current D-H parameters of the right hand
     */
    void computeRightHandDHparams();

    /**
     * @brief This method computes the current D-H parameters of the left hand
     */
    void computeLeftHandDHparams();

    /**
     * @brief This method computes the direct kinematic of the arm
     * @param arm
     * @param posture
     */
    void directKinematicsSingleArm(int arm,std::vector<double>& posture);

    /**
     * @brief directDiffKinematicsSingleArm
     * @param arm
     * @param posture
     * @param velocities
     * @param vel
     * @param mod: 0=shoulder, 1=elbow, 2=wrist, 3=hand
     */
    void directDiffKinematicsSingleArm(int arm, vector<double> posture, vector<double> velocities, vector<double> &vel, int mod);

    /**
     * @brief This method computes the direct kinematic of both arms
     */
    void directKinematicsDualArm();

    /**
     * @brief This method computes the direct kinematics of the fingers
     * @param p
     * @param T_ext
     * @param T_H_0_pos
     * @param id_fing
     * @param Fingers
     */
    void directKinematicsFinger(DHparams& p,Matrix4d& T_ext,Matrix4d& T_H_0_pos,int id_fing, MatrixXd& Fingers);

    /**
     * @brief inverseDiffKinematicsSingleArm
     * @param arm
     * @param posture
     * @param hand_vel
     * @param velocities
     */
    void inverseDiffKinematicsSingleArm(int arm, vector<double> posture, vector<double> hand_vel, vector<double>& velocities);

    /**
     * @brief This method computes the transformation matrix from the D-H parameters
     * It performes the homogeneus transformation matrix given the D-H parameters: \n
     * - translate by d_i along the z_i axis \n
     * - rotate counterclockwise  by theta around the z_i axis \n
     * - translate by a_(i-1) along the x_(i-1) \n
     * - rotate counterclockwise by alpha_(i-1) around the x_(i-1) axis \n
     * @param alpha
     * @param a
     * @param d
     * @param theta
     * @param T
     */
    void transfMatrix(double alpha, double a, double d, double theta, Matrix4d& T);

};

}// namespace motion_manager

#endif // HUMANOID_HPP
