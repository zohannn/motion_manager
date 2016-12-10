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
#include <iostream>
#include <ui_power_lawdialog.h>
#include <eigen3/Eigen/Dense>
#include <pca.hpp>
#include <fstream>
#include <iomanip>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class PowerLawDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:



public:
    /**
     * @brief PowerLawDialog, a constructor
     * @param parent
     */
    explicit PowerLawDialog(QWidget *parent = 0);

    /**
     * @brief ~PowerLawDialog, a destructor
     */
    ~PowerLawDialog();

    /**
     * @brief setupPlots
     * @param hand_position
     * @param timesteps
     */
    void setupPlots(vector<vector<double>>& hand_position,vector<vector<vector<double>>> &timesteps);

private:
    Ui::powerLawDialog *ui; /**< handle of the user interface */

    /**
     * @brief doPCA
     * @param data
     * @param data_red
     * @return -1 in case of error, 0 otherwise
     */
    int doPCA(vector<vector<double>>& data, vector<vector<double> > &data_red);

    /**
     * @brief getDerivative
     * @param function
     * @param step_values
     * @param derFunction
     */
    void getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction);

    /**
     * @brief linreg
     * @param x array of data
     * @param y array of data
     * @param b output intercept
     * @param m output slope
     * @param r output correlation coefficient (can be NULL if you don't want it)
     * @return
     */
     int linreg(const QVector<double> &x, const QVector<double> &y, double* b, double* m, double* r);

};

} // namespace motion_manager
#endif // POWER_LAW_DIALOG_HPP
