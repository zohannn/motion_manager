#ifndef HAND_VELOCITY_DIALOG_HPP
#define HAND_VELOCITY_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_hand_velocitydialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class HandVelocityDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_save_clicked
     */
    void on_pushButton_save_clicked();

public:
    /**
     * @brief HandVelocityDialog, a constructor
     * @param parent
     */
    explicit HandVelocityDialog(QWidget *parent = 0);

    /**
     * @brief ~HandVelocityDialog, a destructor
     */
    ~HandVelocityDialog();

    /**
     * @brief setupPlots
     * @param hand_linear_velocity
     * @param hand_angular_velocity
     * @param time
     */
    void setupPlots(vector<vector<double>> &hand_linear_velocity,vector<vector<double>> &hand_angular_velocity,QVector<double> &time);


private:
    Ui::HandVelocityDialog *ui; /**< handle of the user interface */

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param var
     * @param lin
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& var,bool lin);
};

} // namespace motion_manager
#endif // HAND_VELOCITY_DIALOG
