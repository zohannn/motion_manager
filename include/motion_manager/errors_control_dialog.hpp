#ifndef ERRORS_VELOCITY_DIALOG_HPP
#define ERRORS_VELOCITY_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_errors_controldialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ErrorsControlDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:


    /**
     * @brief on_pushButton_save_clicked
     */
    void on_pushButton_save_clicked();

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
     * @brief setupPlots
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
    void setupPlots(vector<double> &errors_pos, vector<double> &errors_or, vector<double> &errors_pos_or_tot, vector<double> &errors_lin_vel, vector<double> &errors_ang_vel, vector<double> &errors_vel_tot, vector<double> &errors_lin_acc, vector<double> &errors_ang_acc, vector<double> &errors_acc_tot, vector<double> &time);


private:
    Ui::ErrorsControlDialog *ui; /**< handle of the user interface */

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
