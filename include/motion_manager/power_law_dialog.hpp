#ifndef POWER_LAW_DIALOG_HPP
#define POWER_LAW_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_power_lawdialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class powerLawDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:



public:
    /**
     * @brief powerLawDialog, a constructor
     * @param parent
     */
    explicit powerLawDialog(QWidget *parent = 0);

    /**
     * @brief ~powerLawDialog, a destructor
     */
    ~powerLawDialog();

    /**
     * @brief setupPlots
     * @param pos
     * @param vel
     * @param acc
     * @param timesteps
     */
    void setupPlots(vector<MatrixXd> &pos,vector<MatrixXd> &vel,vector<MatrixXd> &acc,vector<vector<double>> &timesteps);


private:
    Ui::powerLawDialog *ui; /**< handle of the user interface */

    /**
     * @brief plotJoint
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     */
    void plotJoint(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel, QVector<double>& acc);
};

} // namespace motion_manager
#endif // POWER_LAW_DIALOG_HPP
