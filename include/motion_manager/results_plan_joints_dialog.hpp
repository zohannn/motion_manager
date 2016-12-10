#ifndef RESULTS_PLAN_JOINTS_DIALOG_HPP
#define RESULTS_PLAN_JOINTS_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_plan_joints_dialog.h>
#include <eigen3/Eigen/Dense>
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

private:
    Ui::ResultsJointsDialog *ui; /**< handle of the user interface */

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
