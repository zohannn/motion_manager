#ifndef ERRORS_VELOCITY_DIALOG_HPP
#define ERRORS_VELOCITY_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_errors_controldialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <LowPassFilter.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ErrorsControlDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:


    /**
     * @brief on_pushButton_plot_hand_clicked
     */
    void on_pushButton_plot_hand_clicked();

    /**
     * @brief on_pushButton_save_hand_clicked
     */
    void on_pushButton_save_hand_clicked();

    /**
     * @brief on_pushButton_plot_fing_clicked
     */
    void on_pushButton_plot_fing_clicked();

    /**
     * @brief on_pushButton_save_fing_clicked
     */
    void on_pushButton_save_fing_clicked();

public:

    /**
     * @brief ErrorsControlDialog
     * @param parent
     */
    explicit ErrorsControlDialog(QWidget *parent = 0);

    /**
     * @brief ErrorsControlDialog
     */
    ~ErrorsControlDialog();

    /**
     * @brief setupHandPlots
     * @param errors_pos
     * @param errors_or
     * @param errors_pos_or_tot
     * @param errors_lin_vel
     * @param errors_ang_vel
     * @param errors_vel_tot
     * @param errors_lin_acc
     * @param errors_ang_acc
     * @param errors_acc_tot
     * @param time
     */
    void setupHandPlots(vector<double> &errors_pos, vector<double> &errors_or, vector<double> &errors_pos_or_tot, vector<double> &errors_lin_vel, vector<double> &errors_ang_vel, vector<double> &errors_vel_tot, vector<double> &errors_lin_acc, vector<double> &errors_ang_acc, vector<double> &errors_acc_tot, vector<double> &time);

    /**
     * @brief setupFingersPlots
     * @param errors_pos
     * @param errors_vel
     * @param errors_acc
     * @param time
     */
    void setupFingersPlots(vector<vector<double>> &errors_pos,vector<vector<double>> &errors_vel,vector<vector<double>> &errors_acc, vector<double> &time);

private:
    Ui::ErrorsControlDialog *ui; /**< handle of the user interface */
    QVector<double> qtime; /**< time */
    // hand
    QVector<double> qerrors_pos; /**< error in position */
    QVector<double> qerrors_pos_plot; /**< error in position plot*/
    QVector<double> qerrors_or; /**< error in orientation */
    QVector<double> qerrors_or_plot; /**< error in orientation plot */
    QVector<double> qerrors_pos_or_tot; /**< error in position and orientation */
    QVector<double> qerrors_pos_or_tot_plot; /**< error in position and orientation plot*/
    QVector<double> qerrors_lin_vel; /**< error in linear velocity */
    QVector<double> qerrors_lin_vel_plot; /**< error in linear velocity plot*/
    QVector<double> qerrors_ang_vel; /**< error in angular velocity */
    QVector<double> qerrors_ang_vel_plot; /**< error in angular velocity plot */
    QVector<double> qerrors_vel_tot; /**< error in velocity */
    QVector<double> qerrors_vel_tot_plot; /**< error in velocity plot*/
    QVector<double> qerrors_lin_acc; /**< error in linear acceleration */
    QVector<double> qerrors_lin_acc_plot; /**< error in linear acceleration plot*/
    QVector<double> qerrors_ang_acc; /**< error in angular acceleration */
    QVector<double> qerrors_ang_acc_plot; /**< error in angular acceleration plot*/
    QVector<double> qerrors_acc_tot; /**< error in acceleration */
    QVector<double> qerrors_acc_tot_plot; /**< error in acceleration plot*/

    //finger 0
    QVector<double> qerrors_fing_pos_0;
    QVector<double> qerrors_fing_pos_0_plot;
    QVector<double> qerrors_fing_vel_0;
    QVector<double> qerrors_fing_vel_0_plot;
    QVector<double> qerrors_fing_acc_0;
    QVector<double> qerrors_fing_acc_0_plot;

    //finger 1
    QVector<double> qerrors_fing_pos_1;
    QVector<double> qerrors_fing_pos_1_plot;
    QVector<double> qerrors_fing_vel_1;
    QVector<double> qerrors_fing_vel_1_plot;
    QVector<double> qerrors_fing_acc_1;
    QVector<double> qerrors_fing_acc_1_plot;

    //finger 2
    QVector<double> qerrors_fing_pos_2;
    QVector<double> qerrors_fing_pos_2_plot;
    QVector<double> qerrors_fing_vel_2;
    QVector<double> qerrors_fing_vel_2_plot;
    QVector<double> qerrors_fing_acc_2;
    QVector<double> qerrors_fing_acc_2_plot;

    //finger 3
    QVector<double> qerrors_fing_pos_3;
    QVector<double> qerrors_fing_pos_3_plot;
    QVector<double> qerrors_fing_vel_3;
    QVector<double> qerrors_fing_vel_3_plot;
    QVector<double> qerrors_fing_acc_3;
    QVector<double> qerrors_fing_acc_3_plot;

    /**
     * @brief plotError
     * @param plot
     * @param title
     * @param time
     * @param error
     * @param name
     * @param color
     */
    void plotError(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& error, QString name, Qt::GlobalColor color);

};

} // namespace motion_manager
#endif // ERRORS_VELOCITY_DIALOG
