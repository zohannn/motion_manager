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
     * @param max_limits
     * @param min_limits
     * @param time
     */
    void setupPlots(MatrixXd &positions, MatrixXd &velocities, vector<double> &max_limits, vector<double> &min_limits, vector<double> &time);


public Q_SLOTS:

    /**
     * @brief check_jlim
     * @param state
     */
    void check_jlim(int state);

private:
    Ui::ResultsCtrlJointsDialog *ui; /**< handle of the user interface */

    MatrixXd positions; /**< positions of the joints */
    MatrixXd velocities; /**< velocities of the joints */
    vector<double> max_pos_limits; /**< maximum limits of the joints */
    vector<double> min_pos_limits; /**< minimum limits of the joints */
    vector<double> time; /**< elapsed time */

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param max_pos
     * @param min_pos
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel, QVector<double> &max_pos, QVector<double> &min_pos);
};

} // namespace motion_manager
#endif // RESULTS_CONTROL_JOINTS_DIALOG_HPP
