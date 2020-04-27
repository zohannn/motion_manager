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
#include <ui_results_warm_start_dialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class WarmStartResultsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:


    /**
     * @brief on_pushButton_save_warm_start_res_clicked
     */
    void on_pushButton_save_warm_start_res_clicked();

    /**
     * @brief on_pushButton_save_warm_start_plots_clicked
     */
    void on_pushButton_save_warm_start_plots_clicked();


public:

    /**
     * @brief WarmStartResultsDialog
     * @param parent
     */
    explicit WarmStartResultsDialog(QWidget *parent = 0);

    /**
     * @brief ~WarmStartResultsDialog, a destructor
     */
    ~WarmStartResultsDialog();

    /**
     * @brief WarmStartResultsDialog::plotIterStats
     * @param plot
     * @param title
     * @param iter
     * @param obj
     * @param dual_inf
     * @param constr_viol
     * @param error
     */
    void plotIterStats(QCustomPlot *plot, QString title, QVector<double> &iter, QVector<double> &obj, QVector<double> &dual_inf, QVector<double> &constr_viol, QVector<double> &error);

    /**
     * @brief setPlanData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param overall_error
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     * @param obj_values
     * @param dual_inf_values
     * @param constr_viol_values
     * @param error_values
     */
    void setPlanData(int iter, double cpu_time, double obj, double overall_error, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps, vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values);

    /**
     * @brief setApproachData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param overall_error
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     * @param obj_values
     * @param dual_inf_values
     * @param constr_viol_values
     * @param error_values
     */
    void setApproachData(int iter, double cpu_time, double obj, double overall_error, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps, vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values);

    /**
     * @brief setRetreatData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param overall_error
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     * @param obj_values
     * @param dual_inf_values
     * @param constr_viol_values
     * @param error_values
     */
    void setRetreatData(int iter, double cpu_time, double obj, double overall_error, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps, vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values);

    /**
     * @brief setBounceData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param overall_error
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param obj_values
     * @param dual_inf_values
     * @param constr_viol_values
     * @param error_values
     */
    void setBounceData(int iter, double cpu_time, double obj, double overall_error, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values);

    /**
     * @brief enablePlanData
     * @param en
     */
    void enablePlanData(bool en);

    /**
     * @brief enableApproachData
     * @param en
     */
    void enableApproachData(bool en);

    /**
     * @brief enableRetreatData
     * @param en
     */
    void enableRetreatData(bool en);

    /**
     * @brief enableBounceData
     * @param en
     */
    void enableBounceData(bool en);


private:
    Ui::WarmStartResultsDialog *ui; /**< handle of the user interface */

    // plan data
    bool en_plan; /**< true if the plan tab is enabled, false otherwise */
    int iterations_plan; /**< number of iterations of the plan data */
    double cpu_time_plan; /**< cpu time of the plan data */
    double obj_plan; /**< obj value of the plan data */
    double error_plan; /**< overall error value of the plan data */
    vector<double> x_plan; /**< solution of the plan data */
    vector<double> zL_plan; /**< lower bounds multipliers of the plan data */
    vector<double> zU_plan; /**< upper bounds multipliers of the plan data */
    vector<double> dual_plan; /**< constraints multipliers of the plan data */
    int n_plan_steps; /**< steps in the plan stage */
    vector<double> obj_values_plan; /**< obj values of the plan data */
    vector<double> dual_inf_values_plan; /**< dual infeasibility values of the plan data */
    vector<double> constr_viol_values_plan; /**< constraint violation values of the plan data */
    vector<double> error_values_plan; /**< overall error values of the plan data */

    // approach data
    bool en_approach; /**< true if the approach tab is enabled, false otherwise */
    int iterations_approach; /**< number of iterations of the approach data */
    double cpu_time_approach; /**< cpu time of the approach data */
    double obj_approach; /**< obj value of the approach data */
    double error_approach; /**< overall error value of the approach data */
    vector<double> x_approach; /**< solution of the approach data */
    vector<double> zL_approach; /**< lower bounds multipliers of the approach data */
    vector<double> zU_approach; /**< upper bounds multipliers of the approach data */
    vector<double> dual_approach; /**< constraints multipliers of the approach data */
    int n_app_steps; /**< steps in the approach stage */
    vector<double> obj_values_app; /**< obj values of the approach data */
    vector<double> dual_inf_values_app; /**< dual infeasibility values of the approach data */
    vector<double> constr_viol_values_app; /**< constraint violation values of the approach data */
    vector<double> error_values_app; /**< overall error values of the approach data */

    // retreat data
    bool en_retreat; /**< true if the retreat tab is enabled, false otherwise */
    int iterations_retreat; /**< number of iterations of the retreat data */
    double cpu_time_retreat; /**< cpu time of the retreat data */
    double obj_retreat; /**< obj value of the retreat data */
    double error_retreat; /**< overall error value of the retreat data */
    vector<double> x_retreat; /**< solution of the retreat data */
    vector<double> zL_retreat; /**< lower bounds multipliers of the retreat data */
    vector<double> zU_retreat; /**< upper bounds multipliers of the retreat data */
    vector<double> dual_retreat; /**< constraints multipliers of the retreat data */
    int n_ret_steps; /**< steps in the retreat stage */
    vector<double> obj_values_ret; /**< obj values of the retreat data */
    vector<double> dual_inf_values_ret; /**< dual infeasibility values of the retreat data */
    vector<double> constr_viol_values_ret; /**< constraint violation values of the retreat data */
    vector<double> error_values_ret; /**< overall error values of the retreat data */

    //bounce data
    bool en_bounce; /**< true if the bounce tab is enabled, false otherwise */
    int iterations_bounce; /**< number of iterations of the bounce data */
    double cpu_time_bounce; /**< cpu time of the bounce data */
    double obj_bounce; /**< obj value of the bounce data */
    double error_bounce; /**< overall error value of the bounce data */
    vector<double> x_bounce; /**< solution of the bounce data */
    vector<double> zL_bounce; /**< lower bounds multipliers of the bounce data */
    vector<double> zU_bounce; /**< upper bounds multipliers of the bounce data */
    vector<double> dual_bounce; /**< constraints multipliers of the bounce data */
    vector<double> obj_values_bounce; /**< obj values of the bounce data */
    vector<double> dual_inf_values_bounce; /**< dual infeasibility values of the bounce data */
    vector<double> constr_viol_values_bounce; /**< constraint violation values of the bounce data */
    vector<double> error_values_bounce; /**< overall error values of the bounce data */




};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
