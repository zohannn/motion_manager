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
     * @brief on_pushButton_plot_hand_clicked
     */
    void on_pushButton_plot_hand_clicked();

    /**
     * @brief on_pushButton_plot_fing_clicked
     */
    void on_pushButton_plot_fing_clicked();

    /**
     * @brief on_pushButton_save_hand_clicked
     */
    void on_pushButton_save_hand_clicked();

    /**
     * @brief on_pushButton_save_fing_clicked
     */
    void on_pushButton_save_fing_clicked();

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
     * @param pos_fing
     * @param des_pos_fing
     * @param time
     */
    void setupPlots(vector<vector<double>> &pos_hand, vector<vector<double> > &or_hand, vector<vector<double>> &des_pos_hand,
                    vector<vector<double>> &pos_fing, vector<vector<double>> &des_pos_fing, vector<double> &time);


private:
    Ui::CompTrackingControlDialog *ui; /**< handle of the user interface */
    QVector<double> qtime;
    // hand
    vector<vector<double>> positions_hand, des_positions_hand;
    vector<vector<double>> orientations_hand;
    QVector<double> hand_pos_x; QVector<double> des_hand_pos_x;
    QVector<double> hand_pos_y; QVector<double> des_hand_pos_y;
    QVector<double> hand_pos_z; QVector<double> des_hand_pos_z;
    QVector<double> hand_or_qx; QVector<double> des_hand_or_qx;
    QVector<double> hand_or_qy; QVector<double> des_hand_or_qy;
    QVector<double> hand_or_qz; QVector<double> des_hand_or_qz;
    QVector<double> hand_or_qw; QVector<double> des_hand_or_qw;
    // fingers
    vector<vector<double>> positions_fing, des_positions_fing;
    QVector<double> pos_fing_0; QVector<double> des_pos_fing_0;
    QVector<double> pos_fing_1; QVector<double> des_pos_fing_1;
    QVector<double> pos_fing_2; QVector<double> des_pos_fing_2;
    QVector<double> pos_fing_3; QVector<double> des_pos_fing_3;

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
