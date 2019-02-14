#ifndef RESULTS_CONTROL_NULL_JOINTS_DIALOG_HPP
#define RESULTS_CONTROL_NULL_JOINTS_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_control_null_joints_dialog.h>
#include <eigen3/Eigen/Dense>
#include <LowPassFilter.hpp>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

class ResultsCtrlNullJointsDialog : public QDialog
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
     * @brief ResultsCtrlNullJointsDialog, a constructor
     * @param parent
     */
    explicit ResultsCtrlNullJointsDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsCtrlNullJointsDialog, a destructor
     */
    ~ResultsCtrlNullJointsDialog();

    /**
     * @brief setupPlots
     * @param velocities
     * @param time
     */
    void setupPlots(MatrixXd &velocities, vector<double> &time);




private:
    Ui::ResultsCtrlNullJointsDialog *ui; /**< handle of the user interface */

    MatrixXd velocities; /**< velocities of the joints */
    vector<double> time; /**< elapsed time */

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param vel
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& vel);
};

} // namespace motion_manager
#endif // RESULTS_CONTROL_NULL_JOINTS_DIALOG_HPP
