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
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ResultsCtrlJointsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

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
    Ui::ResultsCtrlJointsDialog *ui; /**< handle of the user interface */

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
#endif // RESULTS_CONTROL_JOINTS_DIALOG_HPP
