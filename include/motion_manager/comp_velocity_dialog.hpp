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
     * @param hand_linear_velocity
     * @param hand_angular_velocity
     * @param time
     * @param mod: 0=shoulder, 1=elbow, 2=wrist, 3=hand
     */
    void setupPlots(vector<vector<double>> &hand_linear_velocity,vector<vector<double>> &hand_angular_velocity,QVector<double> &time, int mod);


private:
    Ui::CompVelocityDialog *ui; /**< handle of the user interface */

    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param var
     * @param lin
     */
    void plotComp(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& var,bool lin);
};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
