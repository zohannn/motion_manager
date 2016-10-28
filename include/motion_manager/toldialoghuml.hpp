#ifndef TOLDIALOGHUML_H
#define TOLDIALOGHUML_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include<QMessageBox>
#include <ui_toldialoghuml.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

//! The TolDialogHUML class
/**
 * @brief This class defines the tuning process of the human-like motion planner
 */
class TolDialogHUML : public QDialog
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

public:

    /**
     * @brief TolDialogHUML, a constructor
     * @param parent
     */
    explicit TolDialogHUML(QWidget *parent = 0);

    /**
     * @brief TolDialogHUML, a destructor
     */
    ~TolDialogHUML();

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


    //void getTolsTable(std::vector<double>& tols);

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
     * @brief This method gets the number of steps in a movement
     * @return
     */
    int getSteps();

    /**
     * @brief This method gets the maximum angula velocity allowed for each joint
     * @return
     */
    double getWMax();

    /**
     * @brief This method gets the approaching axis in reach-to-grasp movements
     * @return
     */
    int getApproachAxis();

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
     * @brief This method gets the tolerance to stop the execution the movement
     * @return
     */
    double getTolStop();

    /**
     * @brief This method gets the parameters for engaging movements
     * @param dist
     * @param dir
     * @param tols
     */
    void getEngageParams(double& dist, int& dir, std::vector<double>& tols);

    /**
     * @brief This method gets the parameters for disengaging movements
     * @param dist
     * @param dir
     */
    void getDisengageParams(double& dist, int& dir);

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
     * @brief This method sets the information about the tuning of the planner
     * @param info
     */
    void setInfo(string info);


private:
    Ui::TolDialogHUML *ui; /**< handle of the user interface */
    string infoLine; /**< information about the tuning of the planner */
};

} // namespace motion_manager

#endif // TOLDIALOGHUML_H
