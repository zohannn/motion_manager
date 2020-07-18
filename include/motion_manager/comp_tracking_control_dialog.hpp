#ifndef COMP_TRACKING_CONTROL_DIALOG_HPP
#define COMP_TRACKING_CONTROL_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_comp_tracking_controldialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <LowPassFilter.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CompTrackingControlDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:


    /**
     * @brief on_pushButton_plot_hand_pos_clicked
     */
    void on_pushButton_plot_hand_pos_clicked();

    /**
     * @brief on_pushButton_plot_hand_vel_clicked
     */
    void on_pushButton_plot_hand_vel_clicked();

    /**
     * @brief on_pushButton_plot_hand_acc_clicked
     */
    void on_pushButton_plot_hand_acc_clicked();

    /**
     * @brief on_pushButton_plot_fing_pos_clicked
     */
    void on_pushButton_plot_fing_pos_clicked();

    /**
     * @brief on_pushButton_plot_fing_vel_clicked
     */
    void on_pushButton_plot_fing_vel_clicked();

    /**
     * @brief on_pushButton_plot_fing_acc_clicked
     */
    void on_pushButton_plot_fing_acc_clicked();

    /**
     * @brief on_pushButton_plot_alpha_clicked
     */
    void on_pushButton_plot_alpha_clicked();

    /**
     * @brief on_pushButton_save_hand_pos_clicked
     */
    void on_pushButton_save_hand_pos_clicked();

    /**
     * @brief on_pushButton_save_hand_vel_clicked
     */
    void on_pushButton_save_hand_vel_clicked();

    /**
     * @brief on_pushButton_save_hand_acc_clicked
     */
    void on_pushButton_save_hand_acc_clicked();

    /**
     * @brief on_pushButton_save_fing_pos_clicked
     */
    void on_pushButton_save_fing_pos_clicked();

    /**
     * @brief on_pushButton_save_fing_vel_clicked
     */
    void on_pushButton_save_fing_vel_clicked();

    /**
     * @brief on_pushButton_save_fing_acc_clicked
     */
    void on_pushButton_save_fing_acc_clicked();

    /**
     * @brief on_pushButton_save_alpha_clicked
     */
    void on_pushButton_save_alpha_clicked();

public:
    /**
     * @brief CompControlDialog, a constructor
     * @param parent
     */
    explicit CompTrackingControlDialog(QWidget *parent = 0);

    /**
     * @brief ~CompcontrolDialog, a destructor
     */
    ~CompTrackingControlDialog();

    /**
     * @brief setupPlots
     * @param pos_hand
     * @param or_hand
     * @param des_pos_hand
     * @param lin_vel_hand
     * @param ang_vel_hand
     * @param des_vel_hand
     * @param lin_acc_hand
     * @param ang_acc_hand
     * @param des_acc_hand
     * @param pos_fing
     * @param des_pos_fing
     * @param vel_fing
     * @param des_vel_fing
     * @param acc_fing
     * @param des_acc_fing
     * @param pos_alpha
     * @param des_pos_alpha
     * @param vel_alpha
     * @param des_vel_alpha
     * @param acc_alpha
     * @param des_acc_alpha
     * @param time
     */
    void setupPlots(vector<vector<double>> &pos_hand, vector<vector<double> > &or_hand, vector<vector<double>> &des_pos_hand, vector<vector<double> > &lin_vel_hand, vector<vector<double> > &ang_vel_hand, vector<vector<double> > &des_vel_hand, vector<vector<double> > &lin_acc_hand, vector<vector<double> > &ang_acc_hand, vector<vector<double> > &des_acc_hand,
                    vector<vector<double>> &pos_fing, vector<vector<double>> &des_pos_fing, vector<vector<double> > &vel_fing, vector<vector<double> > &des_vel_fing, vector<vector<double> > &acc_fing, vector<vector<double> > &des_acc_fing, vector<double> &pos_alpha, vector<double> &des_pos_alpha, vector<double> &vel_alpha, vector<double> &des_vel_alpha, vector<double> &acc_alpha, vector<double> &des_acc_alpha, vector<double> &time);


private:
    Ui::CompTrackingControlDialog *ui; /**< handle of the user interface */
    QVector<double> qtime;
    // hand
    vector<vector<double>> positions_hand,orientations_hand,des_positions_hand;
    QVector<double> hand_pos_x; QVector<double> des_hand_pos_x;
    QVector<double> hand_pos_y; QVector<double> des_hand_pos_y;
    QVector<double> hand_pos_z; QVector<double> des_hand_pos_z;
    QVector<double> hand_or_qx; QVector<double> des_hand_or_qx;
    QVector<double> hand_or_qy; QVector<double> des_hand_or_qy;
    QVector<double> hand_or_qz; QVector<double> des_hand_or_qz;
    QVector<double> hand_or_qw; QVector<double> des_hand_or_qw;
    vector<vector<double>> lin_vel_hand,ang_vel_hand,des_vel_hand;
    QVector<double> hand_lin_vel_x; QVector<double> des_hand_lin_vel_x;
    QVector<double> hand_lin_vel_y; QVector<double> des_hand_lin_vel_y;
    QVector<double> hand_lin_vel_z; QVector<double> des_hand_lin_vel_z;
    QVector<double> hand_ang_vel_x; QVector<double> des_hand_ang_vel_x;
    QVector<double> hand_ang_vel_y; QVector<double> des_hand_ang_vel_y;
    QVector<double> hand_ang_vel_z; QVector<double> des_hand_ang_vel_z;
    vector<vector<double>> lin_acc_hand,ang_acc_hand,des_acc_hand;
    QVector<double> hand_lin_acc_x; QVector<double> des_hand_lin_acc_x;
    QVector<double> hand_lin_acc_y; QVector<double> des_hand_lin_acc_y;
    QVector<double> hand_lin_acc_z; QVector<double> des_hand_lin_acc_z;
    QVector<double> hand_ang_acc_x; QVector<double> des_hand_ang_acc_x;
    QVector<double> hand_ang_acc_y; QVector<double> des_hand_ang_acc_y;
    QVector<double> hand_ang_acc_z; QVector<double> des_hand_ang_acc_z;
    // fingers
    vector<vector<double>> positions_fing, des_positions_fing;
    QVector<double> pos_fing_0; QVector<double> des_pos_fing_0;
    QVector<double> pos_fing_1; QVector<double> des_pos_fing_1;
    QVector<double> pos_fing_2; QVector<double> des_pos_fing_2;
    QVector<double> pos_fing_3; QVector<double> des_pos_fing_3;
    vector<vector<double>> velocities_fing, des_velocities_fing;
    QVector<double> vel_fing_0; QVector<double> des_vel_fing_0;
    QVector<double> vel_fing_1; QVector<double> des_vel_fing_1;
    QVector<double> vel_fing_2; QVector<double> des_vel_fing_2;
    QVector<double> vel_fing_3; QVector<double> des_vel_fing_3;
    vector<vector<double>> accelerations_fing, des_accelerations_fing;
    QVector<double> acc_fing_0; QVector<double> des_acc_fing_0;
    QVector<double> acc_fing_1; QVector<double> des_acc_fing_1;
    QVector<double> acc_fing_2; QVector<double> des_acc_fing_2;
    QVector<double> acc_fing_3; QVector<double> des_acc_fing_3;
    // swivel angle
    QVector<double> qalpha_pos, qdes_alpha_pos;
    QVector<double> qalpha_vel, qdes_alpha_vel;
    QVector<double> qalpha_acc, qdes_alpha_acc;


    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param var_real
     * @param var_des
     */
    void plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var_real, QVector<double> &var_des);

};

} // namespace motion_manager
#endif // COMP_TRACKING_CONTROL_DIALOG_HPP
