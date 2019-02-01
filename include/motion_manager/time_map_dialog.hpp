#ifndef TIMEMAPDIALOG_HPP
#define TIMEMAPDIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_time_map_dialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class TimeMapDialog : public QDialog
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
     * @brief TimeMapDialog, a constructor
     * @param parent
     */
    explicit TimeMapDialog(QWidget *parent = 0);

    /**
     * @brief ~TimeMapDialog, a destructor
     */
    ~TimeMapDialog();

    /**
     * @brief getPlanTimeMapping
     * @param tau
     * @param dec_rate
     * @param diff_w
     */
    void getPlanTimeMapping(double &tau, double &dec_rate, double &diff_w);

    /**
     * @brief getApproachTimeMapping
     * @param tau
     * @param dec_rate
     * @param diff_w
     */
    void getApproachTimeMapping(double &tau, double &dec_rate, double &diff_w);

    /**
     * @brief getRetreatTimeMapping
     * @param tau
     * @param dec_rate
     * @param diff_w
     */
    void getRetreatTimeMapping(double &tau, double &dec_rate, double &diff_w);

private:
    Ui::TimeMapDialog *ui; /**< handle of the user interface */
    double tau_plan; /**< time constant pf the plan stage */
    double dec_rate_plan; /**< decreasing rate of the plan stage */
    double diff_w_plan; /**< difference weight of the plan stage */
    double tau_app; /**< time constant pf the approach stage */
    double dec_rate_app; /**< decreasing rate of the approach stage */
    double diff_w_app; /**< difference weight of the approach stage */
    double tau_ret; /**< time constant pf the retreat stage */
    double dec_rate_ret; /**< decreasing rate of the retreat stage */
    double diff_w_ret; /**< difference weight of the retreat stage */

};

} // namespace motion_manager
#endif // TIMEMAPDIALOG_HPP
