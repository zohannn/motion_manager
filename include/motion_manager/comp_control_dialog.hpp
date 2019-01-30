#ifndef COMP_VELOCITY_DIALOG_HPP
#define COMP_VELOCITY_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_comp_controldialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CompControlDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

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
