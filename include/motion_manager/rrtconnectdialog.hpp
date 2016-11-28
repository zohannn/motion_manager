#ifndef RRTCONNECTDIALOG_HPP
#define RRTCONNECTDIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_rrtconnectdialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class RRTConnectDialog : public QDialog
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
     * @brief checkFinalPosture
     * @param state
     */
    void checkFinalPosture(int state);

    /**
     * @brief checkAddPlane
     * @param state
     */
    void checkAddPlane(int state);

public:
    /**
     * @brief RRTConnectDialog, a constructor
     * @param parent
     */
    explicit RRTConnectDialog(QWidget *parent = 0);

    /**
     * @brief ~RRTConnectDialog, a destructor
     */
    ~RRTConnectDialog();

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
     * @brief getConfig
     * @return
     */
    std::string getConfig();

    /**
     * @brief This method sets the information about the tuning of the planner
     * @param info
     */
    void setInfo(std::string info);

    /**
     * @brief getTargetMove
     * @param target
     */
    void getTargetMove(std::vector<double> &target);

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
     * @brief getPlaneParameters
     * a*x+b*y+c*z+d=0
     * @param params
     * @param point1
     * @param point2
     * @param point3
     * a=params(0), b=params(1), c=params(2), d=params(3)
     */
    void getPlaneParameters(std::vector<double> &params, std::vector<double> &point1, std::vector<double> &point2, std::vector<double> &point3);


private:
    Ui::RRTConnectDialog *ui; /**< handle of the user interface */
    std::string config; /**< current configuration */
    std::string infoLine; /**< information about the tuning of the planner */
};

} // namespace motion_manager
#endif // RRTCONNECTDIALOG_HPP
