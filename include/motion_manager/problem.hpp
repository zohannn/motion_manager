#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <sys/time.h>

#include "movement.hpp"
#include "scenario.hpp"

#if MOVEIT==1
    // *** Humanoid MoveIt! Planner *** //
    #include <aros_moveit_planner/humanoid_moveit_planner.hpp>
#endif
// ******************************************* //
// *** Human-like Upper-limbs Motion Library (HUMP) *** //
#include <humplanner.hpp>
// ************ //


namespace motion_manager {
#if MOVEIT==1
typedef boost::shared_ptr<moveit_planning::HumanoidPlanner> moveit_plannerPtr; /**< shared pointer to a moveit humanoid planner */
#endif
typedef boost::shared_ptr<HUMotion::HUMPlanner> h_plannerPtr; /**< shared pointer to a human-like motion planner */

typedef boost::shared_ptr<Scenario> scenarioPtr; /**< shared pointer to a scenario */
typedef boost::shared_ptr<Movement> movementPtr; /**< shared pointer to a movement */

//! The Problem class
/**
 * @brief This class defines the concept of the motion planning problem
 */
class Problem
{
public:
    /**
     * @brief Problem, a constructor
     */
    Problem();

    /**
     * @brief Problem, a constructor
     * @param planner_id;
     * @param mov
     * @param scene
     */
    Problem(int planner_id,Movement* mov,Scenario* scene);

#if MOVEIT==1
    /**
     * @brief Problem
     * @param planner_id
     * @param mov
     * @param scene
     * @param m_plannerPtr
     */
    Problem(int planner_id,Movement* mov,Scenario* scene, moveit_plannerPtr m_plannerPtr);
#endif

    /**
     * @brief Problem, a copy constructor
     * @param s
     */
    Problem(const Problem& s);

    /**
     * @brief ~Problem, a desctructor
     */
    ~Problem();

    /**
     * @brief setPlannerID
     * It sets the id (and the name) of the the planner
     * @param id
     */
    void setPlannerID(int id);

    /**
     * @brief getPlannerID
     * It gets the id of the current planner
     * @return
     */
    int getPlannerID();

    /**
     * @brief getPlannerName
     * It gets the name of the current planner
     * @return
     */
    string getPlannerName();

    /**
     * @brief This method sets the axis of the target that during the movement has to be approached
     * @param a
     */
    void setApproachingTargetAxis(int a);

    /**
     * @brief This method sets the problem as solved or unsolved
     * @param s
     */
    void setSolved(bool s);

    /**
     * @brief If p is true, then this method sets the problem as part of a task, otherwise the problem is not part of the task
     * @param p
     */
    void setPartOfTask(bool p);

    /**
     * @brief setMoveSettings
     * @param tar
     * @param final_hand
     * @param final_arm
     * @param use_posture
     */
    void setMoveSettings(std::vector<double> &tar, std::vector<double> &final_hand, std::vector<double> &final_arm, bool use_posture);

    /**
     * @brief This method solves the problem given the tolerances and the parameters of the planner HUMP
     * @param tols
     * @return
     */
    HUMotion::planning_result_ptr solve(HUMotion::hump_params& params);

#if MOVEIT==1
    /**
     * @brief This method solves the problem given the tolerances and the parameters
     * humanoid planner
     * @param params
     * @return
     */
    moveit_planning::PlanningResultPtr solve(moveit_planning::moveit_params& params);
#endif

    /**
     * @brief This method gets the information of the problem
     * @return
     */
    string getInfoLine();

    /**
     * @brief This method gets the trajectory planned
     * @param traj
     * @return
     */
    double getTrajectory(MatrixXd& traj);

    /**
     * @brief This method gets the velocity of the trajectory planned
     * @param vel
     * @return
     */
    double getVelocity(MatrixXd& vel);

    /**
     * @brief This method gets the movement that is related to the problem
     * @return
     */
    movementPtr getMovement();

    /**
     * @brief This method gets the engaged object given in engaging/disengaging movements
     * @return
     */
    objectPtr getObjectEngaged();

    /**
     * @brief This method returns true if the problem has been solved, false otherwise
     * @return
     */
    bool getSolved();

    /**
     * @brief This method returns true if the problem is part of a task, false otherwise
     * @return
     */
    bool getPartOfTask();

    /**
     * @brief This method gets the error log of the problem
     * @return
     */
    int getErrLog();

    /* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
     * windows and linux.

      Thanks to stackoverflow.com: http://stackoverflow.com/questions/1861294/how-to-calculate-execution-time-of-a-code-snippet-in-c
      */
    long long GetTimeMs64();

    /**
     * @brief getTime
     * @return
     */
    double getTime();

private:

    bool solved; /**< true if the problem has been solved */
    double exec_time;/**< time taken by the functions of the planning libraries [ms]*/

   /**
     * @brief error log of the problem (TO DO!!!!!!!!!!!!!!!!!!!!)
     * <table>
     * <caption id="multi_row">Types of the errors</caption>
     * <tr><th>Type      <th>Code <th>Description
     * <tr><td>Any <td>0 <td>the problems have been successfully solved
     * <tr><td rowspan="2">Reach-to-grasp <td>10 <td>final posture reach-to-grasp not solved
     * <tr><td>20 <td>bounce posture reach-to-grasp not solved
     * <tr><td rowspan="4">Engage <td>130 <td>final posture engage sub-disengage not solved
     * <tr><td>13 <td>final posture engage not solved
     * <tr><td>131 <td>final posture engage sub-engage not solved
     * <tr><td>23 <td>bounce posture engage not solved
     * <tr><td>Go Park <td>25 <td> bounce posture Go Park not solved
     * </table>
     */
    int err_log;
    bool part_of_task; /**< true if the problem is part of a task */
    double dHOr; /**< distance between the right hand and the center of the object that is being manipulated */
    double dHOl; /**< distance between the left hand and the center of the object that is being manipulated */
    double dFF; /**< distance between the fingertip F3 and the fingertips F1 and F2 */
    double dFH; /**< distance between the fingers and the palm of the hand */
    std::vector<double> rightFinalPosture; /**< final posture of the right arm+hand */
    std::vector<double> rightFinalHand; /**< final posture of the right hand */
    std::vector<double> leftFinalPosture; /**< final posture of the left arm+hand */
    std::vector<double> leftFinalHand; /**< final posture of the left hand */
    std::vector<double> rightFinalPosture_diseng; /**< final posture of the right arm+hand for disengaging movements*/
    std::vector<double> rightFinalPosture_eng; /**< final posture of the right arm+hand for engaging movements*/
    std::vector<double> leftFinalPosture_diseng; /**< final posture of the left arm+hand for disengaging movements*/
    std::vector<double> leftFinalPosture_eng; /**< final posture of the left arm+hand for engaging movements*/
    MatrixXd optimalTraj; /**< human-like optimized trajectory */
#if MOVEIT==1
    moveit_planning::moveit_params m_params; /**< parameters of the moveit Humanoid planner */
#endif
    HUMotion::hump_params h_params; /**< parameters of the HUMP planner */
    movementPtr mov; /**< movement to be planned */
    scenarioPtr scene; /**< current scene */
    int targetAxis; /**< approaching direction towards the target: 0 = none , 1 = x axis , 2 = y axis, 3 = z axis*/
    objectPtr obj_curr; /**< current object being manipulated */
    targetPtr tar_eng; /**< target of the engaged object */
    objectPtr obj_eng; /**< engaged object */

    int planner_id; /**<  planner id of the selected planner */
    string planner_name; /**< name of the selected planner */

    // move movements settings
    std::vector<double> move_final_hand;/**< goal hand posture in move movements */
    std::vector<double> move_final_arm;/**< goal arm posture in move movements */
    std::vector<double> move_target;/**< goal target pose of the end-effector in move movements */
    bool use_posture;/**< true to use the move_final_arm, false to use move_target in move movements */
#if MOVEIT==1
    moveit_plannerPtr m_planner; /**< MoveIt! Libraries planner */
#endif
    h_plannerPtr h_planner; /**< Human-like Upper-limbs Motion Planner */

    /**
     * @brief This method computes the final posture of the fingers.\n
     * It takes into account the type of grip and the size of the object
     * @param hand_id: it is 1 for right hand and 2 for the left hand
     * @return
     */
    bool finalPostureFingers(int hand_id);

    /**
     * @brief This method computes the inverse kinematics of the hand.
     * @param d_obj: diameter of the object
     * @param hand_id: it is 1 for right hand and 2 for the left hand
     * @param sols: solution
     * @return
     */
    bool invKinHand(double d_obj,int hand_id,std::vector<double>& sols);

    /**
     * @brief getRPY
     * @param rpy
     * @param Rot
     * @return
     */
    bool getRPY(std::vector<double>& rpy, Matrix3d& Rot);

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     * @return
     */
    bool RPY_matrix(std::vector<double>& rpy, Matrix3d& Rot);

};

}// namespace motion_manager

#endif // PROBLEM_HPP
