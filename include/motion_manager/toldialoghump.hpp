#ifndef TOLDIALOGHUMP_H
#define TOLDIALOGHUMP_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <ui_toldialoghump.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

//! The TolDialogHUMP class
/**
 * @brief This class defines the tuning process of the human-like motion planner
 */
class TolDialogHUMP : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief This method saves the tuning parameters to a file
     */
    void on_pushButton_save_clicked();

    /**
     * @brief This method loads the tuning parameters from a file
     */
    void on_pushButton_load_clicked();

    /**
     * @brief on_pushButton_load_warm_start_settings_clicked
     */
    void on_pushButton_load_warm_start_settings_clicked();

    /**
     * @brief checkApproach
     * @param state
     */
    void checkApproach(int state);

    /**
     * @brief checkRetreat
     * @param state
     */
    void checkRetreat(int state);

    /**
     * @brief checkFinalPosture
     * @param state
     */
    void checkFinalPosture(int state);

    /**
     * @brief checkAddPlane
     * @param state
     */
    void checkAddPlane(int state);

    /**
     * @brief checkWarmStart
     * @param state
     */
    void checkWarmStart(int state);

    /**
     * @brief checkMaxIterPlan
     * @param state
     */
    void checkMaxIterPlan(int state);

    /**
     * @brief checkMaxIterApp
     * @param state
     */
    void checkMaxIterApp(int state);

    /**
     * @brief checkMaxIterRet
     * @param state
     */
    void checkMaxIterRet(int state);

    /**
     * @brief checkMaxIterBounce
     * @param state
     */
    void checkMaxIterBounce(int state);

    /**
     * @brief checkSetHandCond
     * @param state
     */
    //void checkSetHandCond(int state);

    /**
     * @brief checkSetHandCondApproach
     * @param state
     */
    //void checkSetHandCondApproach(int state);


public:

    /**
     * @brief TolDialogHUMP, a constructor
     * @param parent
     */
    explicit TolDialogHUMP(QWidget *parent = 0);

    /**
     * @brief TolDialogHUMP, a destructor
     */
    ~TolDialogHUMP();

    /**
     * @brief This method gets the tolerances of the arm
     * @param tols
     */
    void getTolsArm(vector<double>& tols);

    /**
     * @brief This method gets the tolerances of the hand
     * @param tols
     */
    void getTolsHand(MatrixXd& tols);


    /**
     * @brief This method gets the weights of the objective function
     * @param lambda
     */
    void getLambda(std::vector<double>& lambda);

    /**
     * @brief This method gets the tolerances of the obstacles
     * @param tols
     */
    void getTolsObstacles(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the target
     * @param tols
     */
    void getTolsTarget(MatrixXd& tols);


    /**
     * @brief This method gets the maximum angular velocity allowed for each joint [deg/sec]
     * @return
     */
    double getWMax();

    /**
     * @brief This method gets the maximum angular acceleration allowed for each joint [deg/sec²]
     * @return
     */
    double getAlphaMax();

    /**
     * @brief This method gets the maximum angula velocity allowed for each joint [deg/sec]
     * @param w
     */
    void setWMax(double w);

    /**
     * @brief setAlphaMax
     * @param a
     */
    void setAlphaMax(double a);


    /**
     * @brief This method gets the tolerances in positioning the end-effector
     * @return
     */
    double getTolTarPos();

    /**
     * @brief This method gets the tolerances in orienting the end-effector
     * @return
     */
    double getTolTarOr();

    /**
     * @brief This method gets the target avoidance flag
     * @return
     */
    bool getTargetAvoidance();

    /**
     * @brief This method gets the obstacle avoidance flag
     * @return
     */
    bool getObstacleAvoidance();

    /**
     * @brief getApproach
     * @return
     */
    bool getApproach();

    /**
     * @brief getRetreat
     * @return
     */
    bool getRetreat();

    /**
     * @brief getInitVel
     * @param init_vel
     */
    void getInitVel(std::vector<double>& init_vel);

    /**
     * @brief getFinalVel
     * @param final_vel
     */
    void getFinalVel(std::vector<double>& final_vel);

    /**
     * @brief getInitAcc
     * @param init_acc
     */
    void getInitAcc(std::vector<double>& init_acc);

    /**
     * @brief getFinalAcc
     * @param final_acc
     */
    void getFinalAcc(std::vector<double>& final_acc);

    /**
     * @brief getVelApproach
     * @param vel_approach
     */
    //void getVelApproach(std::vector<double>& vel_approach);

    /**
     * @brief getAccApproach
     * @param acc_approach
     */
    //void getAccApproach(std::vector<double>& acc_approach);

    /**
     * @brief getPreGraspApproach
     * @param pre_grasp
     */
    void getPreGraspApproach(std::vector<double>& pre_grasp);

    /**
     * @brief getPostGraspRetreat
     * @param post_grasp
     */
    void getPostGraspRetreat(std::vector<double>& post_grasp);

    /**
     * @brief getPrePlaceApproach
     * @param pre_place
     */
    void getPrePlaceApproach(std::vector<double>& pre_place);

    /**
     * @brief getPostPlaceRetreat
     * @param post_place
     */
    void getPostPlaceRetreat(std::vector<double>& post_place);

    /**
     * @brief getW_red_app
     * @return
     */
    double getW_red_app();

    /**
     * @brief getW_red_ret
     * @return
     */
    double getW_red_ret();

    /**
     * @brief setInitJointsVel
     * @param init_vel
     */
    void setInitJointsVel(std::vector<double>& init_vel);

    /**
     * @brief setInitJointsAcc
     * @param init_acc
     */
    void setInitJointsAcc(std::vector<double>& init_acc);

    /**
     * @brief This method sets the information about the tuning of the planner
     * @param info
     */
    void setInfo(string info);

    /**
     * @brief getRandInit
     * @return
     */
    bool getRandInit();

    /**
     * @brief setRandInit
     * @param rand
     */
    void setRandInit(bool rand);

    /**
     * @brief getColl
     * @return  true if collisions are enabled
     */
    bool getColl();

    /**
     * @brief getCollBody
     * @return
     */
    bool getCollBody();

    /**
     * @brief setColl
     * @param coll, true to enable them
     */
    void setColl(bool coll=true);

    /**
     * @brief getTargetMove
     * @param target
     */
    void getTargetMove(std::vector<double> &target);

    /**
     * @brief setTargetMove
     * @param target
     */
    void setTargetMove(std::vector<double> &target);

    /**
     * @brief getFinalHand
     * @param finalHand
     */
    void getFinalHand(std::vector<double> &finalHand);

    /**
     * @brief getFinalArm
     * @param finalArm
     */
    void getFinalArm(std::vector<double> &finalArm);

    /**
     * @brief get_use_final_posture
     * @return
     */
    bool get_use_final_posture();

    /**
     * @brief get_add_plane
     * @return
     */
    bool get_add_plane();

    /**
     * @brief get_straight_line
     * @return
     */
    bool get_straight_line();

    /**
     * @brief setStraightLine
     * @param straight
     */
    void setStraightLine(bool straight=true);

    /**
     * @brief set_add_plane
     * @param plane
     */
    void set_add_plane(bool plane);

    /**
     * @brief getPlaneParameters
     * a*x+b*y+c*z+d=0
     * @param params
     * a=params(0), b=params(1), c=params(2), d=params(3)
     */
    void getPlaneParameters(std::vector<double> &params);

    /**
     * @brief setPlaneParameters
     * @param point1
     * @param point2
     * @param point3
     */
    void setPlaneParameters(std::vector<double> &point1,std::vector<double> &point2,std::vector<double> &point3);

    /**
     * @brief getWarmStartOption
     * @return
     */
    bool getWarmStartOption();

    /**
     * @brief getWarmStartPlanOption
     * @return
     */
    bool getWarmStartPlanOption();

    /**
     * @brief getWarmStartApproachOption
     * @return
     */
    bool getWarmStartApproachOption();

    /**
     * @brief getWarmStartRetreatOption
     * @return
     */
    bool getWarmStartRetreatOption();

    /**
     * @brief getWarmStartBounceOption
     * @return
     */
    bool getWarmStartBounceOption();

    /**
     * @brief getPlanData
     * @param x
     * @param zL
     * @param zU
     * @param dual
     * @param steps
     */
    void getPlanData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual, int& steps);

    /**
     * @brief getApproachData
     * @param x
     * @param zL
     * @param zU
     * @param dual
     * @param steps
     */
    void getApproachData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual,int &steps);

    /**
     * @brief getRetreatData
     * @param x
     * @param zL
     * @param zU
     * @param dual
     * @param steps
     */
    void getRetreatData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual,int &steps);

    /**
     * @brief getBounceData
     * @param x
     * @param zL
     * @param zU
     * @param dual
     */
    void getBounceData(vector<double> &x,vector<double> &zL,vector<double> &zU,vector<double> &dual);

    /**
     * @brief getMaxIterPlanOption
     * @return
     */
    bool getMaxIterPlanOption();

    /**
     * @brief getMaxIterAppOption
     * @return
     */
    bool getMaxIterAppOption();

    /**
     * @brief getMaxIterRetOption
     * @return
     */
    bool getMaxIterRetOption();

    /**
     * @brief getMaxIterBounceOption
     * @return
     */
    bool getMaxIterBounceOption();

    /**
     * @brief getMaxIterPlan
     * @return
     */
    int getMaxIterPlan();

    /**
     * @brief getMaxIterApp
     * @return
     */
    int getMaxIterApp();

    /**
     * @brief getMaxIterRet
     * @return
     */
    int getMaxIterRet();

    /**
     * @brief getMaxIterBounce
     * @return
     */
    int getMaxIterBounce();


private:
    Ui::TolDialogHUMP *ui; /**< handle of the user interface */
    string infoLine; /**< information about the tuning of the planner */
    bool rand_init;/**< random initialization */

    // max iterations
    bool set_max_iter_plan; /**< true if the max iterations for plan must be set */
    bool set_max_iter_app; /**< true if the max iterations for approach must be set */
    bool set_max_iter_ret; /**< true if the max iterations for retreat must be set */
    bool set_max_iter_bounce; /**< true if the max iterations for bounce must be set */
    int max_iter_plan;/**< max iterations for the plan target posture selection */
    int max_iter_app;/**< max iterations for the approach target posture selection */
    int max_iter_ret;/**< max iterations for the retreat target posture selection */
    int max_iter_bounce;/**< max iterations for the bounce posture selection */

    // warm start settings
    bool warm_start;/**< warm start option */
    // plan
    bool warm_start_plan; /**< true if there are plan target warm start parameters */
    int warm_n_steps_plan; /**< steps of the plan stage when warming start */
    vector<double> x_plan; /**< initial guess of the plan target posture selection problem */
    vector<double> zL_plan; /**< lower bounds multipliers of the plan target posture selection problem */
    vector<double> zU_plan; /**< upper bounds multipliers of the plan target posture selection problem */
    vector<double> dual_plan; /**< constraints multipliers of the plan target posture selection problem */
    // approach
    bool warm_start_approach; /**< true if there are approach target warm start parameters */
    int warm_n_steps_approach; /**< steps of the approach stage when warming start */
    vector<double> x_approach; /**< initial guess of the approach target posture selection problem */
    vector<double> zL_approach; /**< lower bounds multipliers of the approach target posture selection problem */
    vector<double> zU_approach; /**< upper bounds multipliers of the approach target posture selection problem */
    vector<double> dual_approach; /**< constraints multipliers of the approach target posture selection problem */
    // retreat
    bool warm_start_retreat; /**< true if there are retreat target warm start parameters */
    int warm_n_steps_retreat; /**< steps of the retreat stage when warming start */
    vector<double> x_retreat; /**< initial guess of the retreat target posture selection problem */
    vector<double> zL_retreat; /**< lower bounds multipliers of the retreat target posture selection problem */
    vector<double> zU_retreat; /**< upper bounds multipliers of the retreat target posture selection problem */
    vector<double> dual_retreat; /**< constraints multipliers of the retreat target posture selection problem */
    // bounce
    bool warm_start_bounce; /**< true if there are bounce warm start parameters */
    vector<double> x_bounce; /**< initial guess of the bounce target posture selection problem */
    vector<double> zL_bounce; /**< lower bounds multipliers of the bounce target posture selection problem */
    vector<double> zU_bounce; /**< upper bounds multipliers of the bounce target posture selection problem */
    vector<double> dual_bounce; /**< constraints multipliers of the bounce target posture selection problem */

};

} // namespace motion_manager

#endif // TOLDIALOGHUMP_H
