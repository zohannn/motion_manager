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
#include <ui_comp_velocitydialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CompVelocityDialog : public QDialog
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

    /**
     * @brief setDual
     * @param d
     */
    void setDual(bool d);

    /**
     * @brief setRight
     * @param r
     */
    void setRight(bool r);

public:
    /**
     * @brief CompVelocityDialog, a constructor
     * @param parent
     */
    explicit CompVelocityDialog(QWidget *parent = 0);

    /**
     * @brief ~CompVelocityDialog, a destructor
     */
    ~CompVelocityDialog();

    /**
     * @brief setupPlots
     * @param position
     * @param orientation
     * @param hand_linear_velocity
     * @param hand_angular_velocity
     * @param linear_acceleration
     * @param angular_acceleration
     * @param time
     * @param mod
     */
    void setupPlots(vector<vector<double> > &position, vector<vector<double> > &orientation, vector<vector<double>> &hand_linear_velocity, vector<vector<double>> &hand_angular_velocity, vector<vector<double> > &linear_acceleration, vector<vector<double> > &angular_acceleration, QVector<double> &time, int mod);


private:
    Ui::CompVelocityDialog *ui; /**< handle of the user interface */

    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     * @param lin
     */
    void plotComp(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double> &vel, QVector<double> &acc, bool lin);


    bool dual;
    bool right;
};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
