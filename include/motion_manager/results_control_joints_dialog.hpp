#ifndef RESULTS_CONTROL_JOINTS_DIALOG_HPP
#define RESULTS_CONTROL_JOINTS_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_control_joints_dialog.h>
#include <eigen3/Eigen/Dense>
#include <LowPassFilter.hpp>
#include "config.hpp"
#include "results_control_null_joints_dialog.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

class ResultsCtrlJointsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_plot_clicked
     */
    void on_pushButton_plot_clicked();

    /**
     * @brief on_pushButton_save_joints_plots_clicked
     */
    void on_pushButton_save_joints_plots_clicked();

public:
    /**
     * @brief ResultsCtrlJointsDialog, a constructor
     * @param parent
     */
    explicit ResultsCtrlJointsDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsCtrlJointsDialog, a destructor
     */
    ~ResultsCtrlJointsDialog();

    /**
     * @brief setupPlots
     * @param positions
     * @param velocities
     * @param accelerations
     * @param null_velocities
     * @param max_limits
     * @param min_limits
     * @param time
     */
    void setupPlots(MatrixXd &positions, MatrixXd &velocities, MatrixXd &accelerations, MatrixXd &null_velocities, vector<double> &max_limits, vector<double> &min_limits, vector<double> &time);


public Q_SLOTS:

    /**
     * @brief check_jlim
     * @param state
     */
    void check_jlim(int state);

    /**
     * @brief on_pushButton_null_space_vel_clicked
     */
    void on_pushButton_null_space_vel_clicked();

private:
    Ui::ResultsCtrlJointsDialog *ui; /**< handle of the user interface */
    ResultsCtrlNullJointsDialog *mResultsNullJointsdlg; /**< handle of the dialog to show the null space components of the joints velocities */

    MatrixXd positions; /**< positions of the joints */
    MatrixXd velocities; /**< velocities of the joints */
    MatrixXd null_space_velocities; /**< null space velocities of the joints */
    MatrixXd accelerations; /**< accelerations of the joints */
    vector<double> max_pos_limits; /**< maximum limits of the joints */
    vector<double> min_pos_limits; /**< minimum limits of the joints */
    vector<double> time; /**< elapsed time */

    QVector<double> pos_joint1, vel_joint1,acc_joint1,max_limits_joint1,min_limits_joint1;
    QVector<double> pos_joint2, vel_joint2,acc_joint2,max_limits_joint2,min_limits_joint2;
    QVector<double> pos_joint3, vel_joint3,acc_joint3,max_limits_joint3,min_limits_joint3;
    QVector<double> pos_joint4, vel_joint4,acc_joint4,max_limits_joint4,min_limits_joint4;
    QVector<double> pos_joint5, vel_joint5,acc_joint5,max_limits_joint5,min_limits_joint5;
    QVector<double> pos_joint6, vel_joint6,acc_joint6,max_limits_joint6,min_limits_joint6;
    QVector<double> pos_joint7, vel_joint7,acc_joint7,max_limits_joint7,min_limits_joint7;
    QVector<double> pos_joint8, vel_joint8,acc_joint8,max_limits_joint8,min_limits_joint8;
    QVector<double> pos_joint9, vel_joint9,acc_joint9,max_limits_joint9,min_limits_joint9;
    QVector<double> pos_joint10, vel_joint10,acc_joint10,max_limits_joint10,min_limits_joint10;
    QVector<double> pos_joint11, vel_joint11,acc_joint11,max_limits_joint11,min_limits_joint11;

    QVector<double> qtime;


    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     * @param max_pos
     * @param min_pos
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel, QVector<double> &acc, QVector<double> &max_pos, QVector<double> &min_pos);
};

} // namespace motion_manager
#endif // RESULTS_CONTROL_JOINTS_DIALOG_HPP
