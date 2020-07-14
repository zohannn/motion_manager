#ifndef COEFFSDIALOG_HPP
#define COEFFSDIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_coeffs_dialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class CoeffsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief This method saves the tuning parameters to a file
     */
    void on_pushButton_save_clicked();

    /**
     * @brief This method loads the tuning parameters from a file
     */
    void on_pushButton_load_clicked();

    /**
     * @brief accept
     */
    void accept();

    /**
     * @brief reject
     */
    void reject();


public:
    /**
     * @brief CoeffsDialog, a constructor
     * @param parent
     */
    explicit CoeffsDialog(QWidget *parent = 0);

    /**
     * @brief ~CoeffsDialog, a destructor
     */
    ~CoeffsDialog();

    /**
     * @brief getPositionCoeffs
     * @param kpx
     * @param kpy
     * @param kpz
     * @param kox
     * @param koy
     * @param koz
     */
    void getPositionCoeffs(double &kpx, double &kpy, double &kpz, double &kox, double &koy, double &koz);

    /**
     * @brief getVelocityCoeffs
     * @param kdx
     * @param kdy
     * @param kdz
     * @param kwx
     * @param kwy
     * @param kwz
     */
    void getVelocityCoeffs(double &kdx, double &kdy, double &kdz, double &kwx, double &kwy, double &kwz);

private:
    Ui::CoeffsDialog *ui; /**< handle of the user interface */
    // position coefficients
    double kpx,kpy,kpz; /**< kp position **/
    double kox,koy,koz; /**< ko orientation **/
    // velocity coefficients
    double kdx,kdy,kdz; /**< kd linear velocity **/
    double kwx,kwy,kwz; /**< kw angular velocity **/
};

} // namespace motion_manager
#endif // COEFFSDIALOG_HPP
