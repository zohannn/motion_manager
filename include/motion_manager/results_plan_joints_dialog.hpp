#ifndef RESULTS_PLAN_JOINTS_DIALOG_HPP
#define RESULTS_PLAN_JOINTS_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_plan_joints_dialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ResultsJointsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_save_joints_plots_clicked
     */
    void on_pushButton_save_joints_plots_clicked();

public:
    /**
     * @brief ResultsJointsDialog, a constructor
     * @param parent
     */
    explicit ResultsJointsDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsJointsDialog, a destructor
     */
    ~ResultsJointsDialog();

    /**
     * @brief setupPlots
     * @param pos
     * @param vel
     * @param acc
     * @param timesteps
     */
    void setupPlots(vector<MatrixXd> &pos,vector<MatrixXd> &vel,vector<MatrixXd> &acc,vector<vector<double>> &timesteps);

    /**
     * @brief setupPlots
     * @param pos
     * @param vel
     * @param acc
     * @param timesteps
     */
    void setupPlots(vector<vector<MatrixXd>> &pos, vector<vector<MatrixXd>> &vel,vector<vector<MatrixXd>> &acc,vector<vector<vector<double>>> &timesteps);

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
private:
    Ui::ResultsJointsDialog *ui; /**< handle of the user interface */
    QVector<double> qtime;
    QVector<double> pos_joint1, vel_joint1, acc_joint1;
    QVector<double> pos_joint2, vel_joint2, acc_joint2;
    QVector<double> pos_joint3, vel_joint3, acc_joint3;
    QVector<double> pos_joint4, vel_joint4, acc_joint4;
    QVector<double> pos_joint5, vel_joint5, acc_joint5;
    QVector<double> pos_joint6, vel_joint6, acc_joint6;
    QVector<double> pos_joint7, vel_joint7, acc_joint7;
    QVector<double> pos_joint8, vel_joint8, acc_joint8;
    QVector<double> pos_joint9, vel_joint9, acc_joint9;
    QVector<double> pos_joint10, vel_joint10, acc_joint10;
    QVector<double> pos_joint11, vel_joint11, acc_joint11;

    bool dual;

    bool right;

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel, QVector<double>& acc);
};

} // namespace motion_manager
#endif // RESULTS_PLAN_JOINTS_DIALOG_HPP
