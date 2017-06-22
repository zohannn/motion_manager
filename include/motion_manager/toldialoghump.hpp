#ifndef TOLDIALOGHUMP_H
#define TOLDIALOGHUMP_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include<QMessageBox>
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





private:
    Ui::TolDialogHUMP *ui; /**< handle of the user interface */
    string infoLine; /**< information about the tuning of the planner */
    bool rand_init;
};

} // namespace motion_manager

#endif // TOLDIALOGHUMP_H
