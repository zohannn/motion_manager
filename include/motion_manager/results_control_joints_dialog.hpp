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
     * @param positions
     * @param velocities
     * @param time
     */
    void setupPlots(MatrixXd &positions,MatrixXd &velocities,vector<double> &time);


private:
    Ui::ResultsCtrlJointsDialog *ui; /**< handle of the user interface */

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel);
};

} // namespace motion_manager
#endif // RESULTS_CONTROL_JOINTS_DIALOG_HPP
