#ifndef COMP_VELOCITY_DIALOG_HPP
#define COMP_VELOCITY_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_comp_velocitydialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CompVelocityDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_save_shoulder_clicked
     */
    void on_pushButton_save_shoulder_clicked();

    /**
     * @brief on_pushButton_save_elbow_clicked
     */
    void on_pushButton_save_elbow_clicked();

    /**
     * @brief on_pushButton_save_wrist_clicked
     */
    void on_pushButton_save_wrist_clicked();

    /**
     * @brief on_pushButton_save_hand_clicked
     */
    void on_pushButton_save_hand_clicked();

    /**
     * @brief setDual
     * @param d
     */
    void setDual(bool d);

    /**
     * @brief setRight
     * @param r
     */
    void setRight(bool r);

public:
    /**
     * @brief CompVelocityDialog, a constructor
     * @param parent
     */
    explicit CompVelocityDialog(QWidget *parent = 0);

    /**
     * @brief ~CompVelocityDialog, a destructor
     */
    ~CompVelocityDialog();

    /**
     * @brief setupPlots
     * @param position
     * @param orientation
     * @param hand_linear_velocity
     * @param hand_angular_velocity
     * @param linear_acceleration
     * @param angular_acceleration
     * @param time
     * @param mod
     */
    void setupPlots(vector<vector<double> > &position, vector<vector<double> > &orientation, vector<vector<double>> &hand_linear_velocity, vector<vector<double>> &hand_angular_velocity, vector<vector<double> > &linear_acceleration, vector<vector<double> > &angular_acceleration, QVector<double> &time, int mod);


private:
    Ui::CompVelocityDialog *ui; /**< handle of the user interface */

    QVector<double> qtime;

    // hand
    QVector<double> linear_pos_x_hand, linear_pos_y_hand, linear_pos_z_hand;
    QVector<double> angular_pos_x_hand, angular_pos_y_hand, angular_pos_z_hand;
    QVector<double> linear_vel_x_hand, linear_vel_y_hand, linear_vel_z_hand;
    QVector<double> angular_vel_x_hand, angular_vel_y_hand, angular_vel_z_hand;
    QVector<double> linear_acc_x_hand, linear_acc_y_hand, linear_acc_z_hand;
    QVector<double> angular_acc_x_hand, angular_acc_y_hand, angular_acc_z_hand;
    QVector<double> linear_pos_norm_hand, angular_pos_norm_hand;
    QVector<double> linear_vel_norm_hand,angular_vel_norm_hand;
    QVector<double> linear_acc_norm_hand,angular_acc_norm_hand;
    // wrist
    QVector<double> linear_pos_x_wrist, linear_pos_y_wrist, linear_pos_z_wrist;
    QVector<double> angular_pos_x_wrist, angular_pos_y_wrist, angular_pos_z_wrist;
    QVector<double> linear_vel_x_wrist, linear_vel_y_wrist, linear_vel_z_wrist;
    QVector<double> angular_vel_x_wrist, angular_vel_y_wrist, angular_vel_z_wrist;
    QVector<double> linear_acc_x_wrist, linear_acc_y_wrist, linear_acc_z_wrist;
    QVector<double> angular_acc_x_wrist, angular_acc_y_wrist, angular_acc_z_wrist;
    QVector<double> linear_pos_norm_wrist, angular_pos_norm_wrist;
    QVector<double> linear_vel_norm_wrist,angular_vel_norm_wrist;
    QVector<double> linear_acc_norm_wrist,angular_acc_norm_wrist;
    // elbow
    QVector<double> linear_pos_x_elbow, linear_pos_y_elbow, linear_pos_z_elbow;
    QVector<double> angular_pos_x_elbow, angular_pos_y_elbow, angular_pos_z_elbow;
    QVector<double> linear_vel_x_elbow, linear_vel_y_elbow, linear_vel_z_elbow;
    QVector<double> angular_vel_x_elbow, angular_vel_y_elbow, angular_vel_z_elbow;
    QVector<double> linear_acc_x_elbow, linear_acc_y_elbow, linear_acc_z_elbow;
    QVector<double> angular_acc_x_elbow, angular_acc_y_elbow, angular_acc_z_elbow;
    QVector<double> linear_pos_norm_elbow, angular_pos_norm_elbow;
    QVector<double> linear_vel_norm_elbow,angular_vel_norm_elbow;
    QVector<double> linear_acc_norm_elbow,angular_acc_norm_elbow;
    // shoulder
    QVector<double> linear_pos_x_shoulder, linear_pos_y_shoulder, linear_pos_z_shoulder;
    QVector<double> angular_pos_x_shoulder, angular_pos_y_shoulder, angular_pos_z_shoulder;
    QVector<double> linear_vel_x_shoulder, linear_vel_y_shoulder, linear_vel_z_shoulder;
    QVector<double> angular_vel_x_shoulder, angular_vel_y_shoulder, angular_vel_z_shoulder;
    QVector<double> linear_acc_x_shoulder, linear_acc_y_shoulder, linear_acc_z_shoulder;
    QVector<double> angular_acc_x_shoulder, angular_acc_y_shoulder, angular_acc_z_shoulder;
    QVector<double> linear_pos_norm_shoulder, angular_pos_norm_shoulder;
    QVector<double> linear_vel_norm_shoulder,angular_vel_norm_shoulder;
    QVector<double> linear_acc_norm_shoulder,angular_acc_norm_shoulder;


    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     * @param lin
     */
    void plotComp(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double> &vel, QVector<double> &acc, bool lin);


    bool dual;
    bool right;
};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG_HPP
