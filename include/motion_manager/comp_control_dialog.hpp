#ifndef COMP_VELOCITY_DIALOG_HPP
#define COMP_VELOCITY_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_comp_controldialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <LowPassFilter.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CompControlDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_plot_shoulder_clicked
     */
    void on_pushButton_plot_shoulder_clicked();

    /**
     * @brief on_pushButton_plot_elbow_clicked
     */
    void on_pushButton_plot_elbow_clicked();

    /**
     * @brief on_pushButton_plot_wrist_clicked
     */
    void on_pushButton_plot_wrist_clicked();

    /**
     * @brief on_pushButton_plot_hand_clicked
     */
    void on_pushButton_plot_hand_clicked();

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

public:
    /**
     * @brief CompControlDialog, a constructor
     * @param parent
     */
    explicit CompControlDialog(QWidget *parent = 0);

    /**
     * @brief ~CompcontrolDialog, a destructor
     */
    ~CompControlDialog();

    /**
     * @brief setupPlots
     * @param positions
     * @param orientations
     * @param linear_velocity
     * @param angular_velocity
     * @param linear_acceleration
     * @param angular_acceleration
     * @param time
     * @param mod
     */
    void setupPlots(vector<vector<double>> &positions, vector<vector<double>> &orientations, vector<vector<double>> &linear_velocity, vector<vector<double>> &angular_velocity, vector<vector<double> > &linear_acceleration, vector<vector<double> > &angular_acceleration, vector<double> &time, int mod);


private:
    Ui::CompControlDialog *ui; /**< handle of the user interface */
    QVector<double> qtime;
    // hand
    vector<vector<double>> positions_hand; vector<vector<double>> orientations_hand;
    vector<vector<double>> lin_vel_hand; vector<vector<double>> ang_vel_hand;
    vector<vector<double>> lin_acc_hand; vector<vector<double>> ang_acc_hand;
    QVector<double> linear_pos_x_hand, linear_pos_y_hand, linear_pos_z_hand;
    QVector<double> angular_pos_x_hand, angular_pos_y_hand, angular_pos_z_hand;
    QVector<double> linear_vel_x_hand, linear_vel_y_hand, linear_vel_z_hand;
    QVector<double> angular_vel_x_hand, angular_vel_y_hand, angular_vel_z_hand;
    QVector<double> linear_acc_x_hand, linear_acc_y_hand, linear_acc_z_hand;
    QVector<double> angular_acc_x_hand, angular_acc_y_hand, angular_acc_z_hand;
    // wrist
    vector<vector<double>> positions_wrist; vector<vector<double>> orientations_wrist;
    vector<vector<double>> lin_vel_wrist; vector<vector<double>> ang_vel_wrist;
    vector<vector<double>> lin_acc_wrist; vector<vector<double>> ang_acc_wrist;
    QVector<double> linear_pos_x_wrist, linear_pos_y_wrist, linear_pos_z_wrist;
    QVector<double> angular_pos_x_wrist, angular_pos_y_wrist, angular_pos_z_wrist;
    QVector<double> linear_vel_x_wrist, linear_vel_y_wrist, linear_vel_z_wrist;
    QVector<double> angular_vel_x_wrist, angular_vel_y_wrist, angular_vel_z_wrist;
    QVector<double> linear_acc_x_wrist, linear_acc_y_wrist, linear_acc_z_wrist;
    QVector<double> angular_acc_x_wrist, angular_acc_y_wrist, angular_acc_z_wrist;
    // elbow
    vector<vector<double>> positions_elbow; vector<vector<double>> orientations_elbow;
    vector<vector<double>> lin_vel_elbow; vector<vector<double>> ang_vel_elbow;
    vector<vector<double>> lin_acc_elbow; vector<vector<double>> ang_acc_elbow;
    QVector<double> linear_pos_x_elbow, linear_pos_y_elbow, linear_pos_z_elbow;
    QVector<double> angular_pos_x_elbow, angular_pos_y_elbow, angular_pos_z_elbow;
    QVector<double> linear_vel_x_elbow, linear_vel_y_elbow, linear_vel_z_elbow;
    QVector<double> angular_vel_x_elbow, angular_vel_y_elbow, angular_vel_z_elbow;
    QVector<double> linear_acc_x_elbow, linear_acc_y_elbow, linear_acc_z_elbow;
    QVector<double> angular_acc_x_elbow, angular_acc_y_elbow, angular_acc_z_elbow;
    // shoulder
    vector<vector<double>> positions_shoulder; vector<vector<double>> orientations_shoulder;
    vector<vector<double>> lin_vel_shoulder; vector<vector<double>> ang_vel_shoulder;
    vector<vector<double>> lin_acc_shoulder; vector<vector<double>> ang_acc_shoulder;
    QVector<double> linear_pos_x_shoulder, linear_pos_y_shoulder, linear_pos_z_shoulder;
    QVector<double> angular_pos_x_shoulder, angular_pos_y_shoulder, angular_pos_z_shoulder;
    QVector<double> linear_vel_x_shoulder, linear_vel_y_shoulder, linear_vel_z_shoulder;
    QVector<double> angular_vel_x_shoulder, angular_vel_y_shoulder, angular_vel_z_shoulder;
    QVector<double> linear_acc_x_shoulder, linear_acc_y_shoulder, linear_acc_z_shoulder;
    QVector<double> angular_acc_x_shoulder, angular_acc_y_shoulder, angular_acc_z_shoulder;


    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param var_pos
     * @param var_vel
     * @param var_acc
     * @param lin
     */
    void plotComp(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& var_pos, QVector<double> &var_vel, QVector<double> &var_acc, bool lin);

};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
