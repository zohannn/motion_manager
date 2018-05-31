#ifndef TOLDIALOGHUMPDUAL_H
#define TOLDIALOGHUMPDUAL_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include<QMessageBox>
#include <ui_toldialoghump_dual.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

//! The TolDialogHUMPDual class
/**
 * @brief This class defines the tuning process of the human-like motion planner for dual-arm motion
 */
class TolDialogHUMPDual : public QDialog
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
     * @brief checkFinalPostureRight
     * @param state
     */
    void checkFinalPostureRight(int state);

    /**
     * @brief checkFinalPostureLeft
     * @param state
     */
    void checkFinalPostureLeft(int state);

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
     * @brief TolDialogHUMPDual, a constructor
     * @param parent
     */
    explicit TolDialogHUMPDual(QWidget *parent = 0);

    /**
     * @brief TolDialogHUMPDual, a destructor
     */
    ~TolDialogHUMPDual();

    /**
     * @brief This method gets the tolerances of the right arm
     * @param tols
     */
    void getTolsArmRight(vector<double>& tols);

    /**
     * @brief This method gets the tolerances of the left arm
     * @param tols
     */
    void getTolsArmLeft(vector<double>& tols);

    /**
     * @brief This method gets the tolerances of the right hand
     * @param tols
     */
    void getTolsHandRight(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the left hand
     * @param tols
     */
    void getTolsHandLeft(MatrixXd& tols);

    /**
     * @brief This method gets the weights of the objective function (right)
     * @param lambda
     */
    void getLambdaRight(std::vector<double>& lambda);

    /**
     * @brief This method gets the weights of the objective function (left)
     * @param lambda
     */
    void getLambdaLeft(std::vector<double>& lambda);

    /**
     * @brief This method gets the tolerances of the obstacles (right)
     * @param tols
     */
    void getTolsObstaclesRight(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the obstacles (left)
     * @param tols
     */
    void getTolsObstaclesLeft(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the target (right)
     * @param tols
     */
    void getTolsTargetRight(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the target (left)
     * @param tols
     */
    void getTolsTargetLeft(MatrixXd& tols);


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
     * @brief This method gets the tolerances in positioning the right end-effector
     * @return
     */
    double getTolTarPosRight();

    /**
     * @brief This method gets the tolerances in positioning the left end-effector
     * @return
     */
    double getTolTarPosLeft();

    /**
     * @brief This method gets the tolerances in orienting the right end-effector
     * @return
     */
    double getTolTarOrRight();

    /**
     * @brief This method gets the tolerances in orienting the left end-effector
     * @return
     */
    double getTolTarOrLeft();

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
     * @brief getInitVelRight
     * @param init_vel
     */
    void getInitVelRight(std::vector<double>& init_vel);

    /**
     * @brief setInitVelRight
     * @param init_vel
     */
    void setInitVelRight(std::vector<double>& init_vel);

    /**
     * @brief getInitVelLeft
     * @param init_vel
     */
    void getInitVelLeft(std::vector<double>& init_vel);

    /**
     * @brief setInitVelLeft
     * @param init_vel
     */
    void setInitVelLeft(std::vector<double>& init_vel);

    /**
     * @brief getFinalVelRight
     * @param final_vel
     */
    void getFinalVelRight(std::vector<double>& final_vel);

    /**
     * @brief getFinalVelLeft
     * @param final_vel
     */
    void getFinalVelLeft(std::vector<double>& final_vel);

    /**
     * @brief getInitAccRight
     * @param init_acc
     */
    void getInitAccRight(std::vector<double>& init_acc);

    /**
     * @brief setInitAccRight
     * @param init_acc
     */
    void setInitAccRight(std::vector<double>& init_acc);

    /**
     * @brief getInitAccLeft
     * @param init_acc
     */
    void getInitAccLeft(std::vector<double>& init_acc);

    /**
     * @brief setInitAccLeft
     * @param init_acc
     */
    void setInitAccLeft(std::vector<double>& init_acc);

    /**
     * @brief getFinalAccRight
     * @param final_acc
     */
    void getFinalAccRight(std::vector<double>& final_acc);

    /**
     * @brief getFinalAccLeft
     * @param final_acc
     */
    void getFinalAccLeft(std::vector<double>& final_acc);

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
     * @brief getPreGraspApproachRight
     * @param pre_grasp
     */
    void getPreGraspApproachRight(std::vector<double>& pre_grasp);

    /**
     * @brief getPreGraspApproachLeft
     * @param pre_grasp
     */
    void getPreGraspApproachLeft(std::vector<double>& pre_grasp);

    /**
     * @brief getPostGraspRetreatRight
     * @param post_grasp
     */
    void getPostGraspRetreatRight(std::vector<double>& post_grasp);

    /**
     * @brief getPostGraspRetreatLeft
     * @param post_grasp
     */
    void getPostGraspRetreatLeft(std::vector<double>& post_grasp);

    /**
     * @brief getPrePlaceApproachRight
     * @param pre_place
     */
    void getPrePlaceApproachRight(std::vector<double>& pre_place);

    /**
     * @brief getPrePlaceApproachLeft
     * @param pre_place
     */
    void getPrePlaceApproachLeft(std::vector<double>& pre_place);

    /**
     * @brief getPostPlaceRetreatRight
     * @param post_place
     */
    void getPostPlaceRetreatRight(std::vector<double>& post_place);

    /**
     * @brief getPostPlaceRetreatLeft
     * @param post_place
     */
    void getPostPlaceRetreatLeft(std::vector<double>& post_place);

    /**
     * @brief getW_red_app_right
     * @return
     */
    double getW_red_app_right();

    /**
     * @brief getW_red_app_left
     * @return
     */
    double getW_red_app_left();

    /**
     * @brief getW_red_ret_right
     * @return
     */
    double getW_red_ret_right();

    /**
     * @brief getW_red_ret_left
     * @return
     */
    double getW_red_ret_left();

    /**
     * @brief setInitJointsVelRight
     * @param init_vel
     */
    void setInitJointsVelRight(std::vector<double>& init_vel);

    /**
     * @brief setInitJointsVelLeft
     * @param init_vel
     */
    void setInitJointsVelLeft(std::vector<double>& init_vel);

    /**
     * @brief setInitJointsAccRight
     * @param init_acc
     */
    void setInitJointsAccRight(std::vector<double>& init_acc);

    /**
     * @brief setInitJointsAccLeft
     * @param init_acc
     */
    void setInitJointsAccLeft(std::vector<double>& init_acc);

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
     * @brief getCollArms
     * @return
     */
    bool getCollArms();

    /**
     * @brief setColl
     * @param coll, true to enable them
     */
    void setColl(bool coll=true);

    /**
     * @brief getTargetMoveRight
     * @param target
     */
    void getTargetMoveRight(std::vector<double> &target);

    /**
     * @brief getTargetMoveLeft
     * @param target
     */
    void getTargetMoveLeft(std::vector<double> &target);

    /**
     * @brief setTargetMoveRight
     * @param target
     */
    void setTargetMoveRight(std::vector<double> &target);

    /**
     * @brief setTargetMoveLeft
     * @param target
     */
    void setTargetMoveLeft(std::vector<double> &target);

    /**
     * @brief getFinalHandRight
     * @param finalHand
     */
    void getFinalHandRight(std::vector<double> &finalHand);

    /**
     * @brief getFinalHandLeft
     * @param finalHand
     */
    void getFinalHandLeft(std::vector<double> &finalHand);

    /**
     * @brief getFinalArmRight
     * @param finalArm
     */
    void getFinalArmRight(std::vector<double> &finalArm);

    /**
     * @brief getFinalArmLeft
     * @param finalArm
     */
    void getFinalArmLeft(std::vector<double> &finalArm);

    /**
     * @brief get_use_final_posture_right
     * @return
     */
    bool get_use_final_posture_right();

    /**
     * @brief get_use_final_posture_left
     * @return
     */
    bool get_use_final_posture_left();

    /**
     * @brief get_add_plane
     * @return
     */
    bool get_add_plane();

    /**
     * @brief get_straight_line_right
     * @return
     */
    bool get_straight_line_right();

    /**
     * @brief get_straight_line_left
     * @return
     */
    bool get_straight_line_left();

    /**
     * @brief setStraightLineRight
     * @param straight
     */
    void setStraightLineRight(bool straight=true);

    /**
     * @brief setStraightLineLeft
     * @param straight
     */
    void setStraightLineLeft(bool straight=true);

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
    Ui::TolDialogHUMPDual *ui; /**< handle of the user interface */
    string infoLine; /**< information about the tuning of the planner */
    bool rand_init;
};

} // namespace motion_manager

#endif // TOLDIALOGHUMPDUAL_H
