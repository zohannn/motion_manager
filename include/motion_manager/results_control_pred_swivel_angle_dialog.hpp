#ifndef RESULTS_CONTROL_PRED_SWIVEL_ANGLE_DIALOG_HPP
#define RESULTS_CONTROL_PRED_SWIVEL_ANGLE_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_control_pred_swivel_angle_dialog.h>
#include <eigen3/Eigen/Dense>
#include <LowPassFilter.hpp>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

class ResultsCtrlPredSwivelAngleDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_plot_clicked
     */
    void on_pushButton_plot_clicked();

    /**
     * @brief on_pushButton_save_clicked
     */
    void on_pushButton_save_clicked();

public:
    /**
     * @brief ResultsCtrlPredSwivelAngleDialog, a constructor
     * @param parent
     */
    explicit ResultsCtrlPredSwivelAngleDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsCtrlPredSwivelAngleDialog, a destructor
     */
    ~ResultsCtrlPredSwivelAngleDialog();

    /**
     * @brief setupPlots
     * @param swivel_angle_pos
     * @param swivel_angle_vel
     * @param time
     */
    void setupPlots(vector<double> &swivel_angle_pos, vector<double> &swivel_angle_vel, vector<double> &time);



private:
    Ui::ResultsCtrlPredSwivelAngleDialog *ui; /**< handle of the user interface */

    vector<double> swivel_angle_pos; /**< position of the swivel angle */
    vector<double> swivel_angle_vel; /**< velocity of the swivel angle */
    vector<double> time; /**< elapsed time */

    /**
     * @brief plotSwivel
     * @param plot
     * @param title
     * @param time
     * @param data
     * @param vel
     */
    void plotSwivel(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& data, bool vel);
};

} // namespace motion_manager
#endif // RESULTS_CONTROL_PRED_SWIVEL_ANGLE_DIALOG_HPP
