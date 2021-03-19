#ifndef POWERLAW3DDIALOG_HPP
#define POWERLAW3DDIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <ui_power_law_3ddialog.h>
#include <eigen3/Eigen/Dense>
#include <pca.hpp>
#include <fstream>
#include <iomanip>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

class PowerLaw3DDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_save_clicked
     */
    void on_pushButton_save_clicked();

public:
    /**
     * @brief PowerLaw3DDialog, a constructor
     * @param parent
     */
    explicit PowerLaw3DDialog(QWidget *parent = 0);

    /**
     * @brief ~PowerLaw3DDialog, a destructor
     */
    ~PowerLaw3DDialog();

    /**
     * @brief setupPlots
     * @param hand_position
     * @param timesteps
     */
    void setupPlots(vector<vector<double>>& hand_position,vector<vector<vector<double>>> &timesteps);

private:
    Ui::powerLaw3DDialog *ui; /**< handle of the user interface */
    std::vector<double> slopes; /**< slopes of the linear regression to be printed */
    std::vector<double> r_squared; /**< R^2 of the linear regression to be printed */
    std::vector<double> n_points; /**< number of points of each regression to be printed */
    std::vector<std::vector<double>> vel_task; /**< hand velocity of the task for each movement */
    std::vector<std::vector<double>> acc_task; /**< hand acceleration of the task for each movement */
    std::vector<std::vector<double>> K_task; /**< hand curvature of the task for each movement */
    std::vector<std::vector<double>> T_task; /**< hand torsion of the task for each movement */

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

#endif // POWERLAW3DDIALOG_HPP
