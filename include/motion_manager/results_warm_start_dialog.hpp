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
     * @brief setPlanData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     */
    void setPlanData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps);

    /**
     * @brief setApproachData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     */
    void setApproachData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps);

    /**
     * @brief setRetreatData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     * @param warm_n_steps
     */
    void setRetreatData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps);

    /**
     * @brief setBounceData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     */
    void setBounceData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars);

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
    vector<double> x_plan; /**< solution of the plan data */
    vector<double> zL_plan; /**< lower bounds multipliers of the plan data */
    vector<double> zU_plan; /**< upper bounds multipliers of the plan data */
    vector<double> dual_plan; /**< constraints multipliers of the plan data */
    int n_plan_steps; /**< steps in the plan stage */

    // approach data
    bool en_approach; /**< true if the approach tab is enabled, false otherwise */
    int iterations_approach; /**< number of iterations of the approach data */
    double cpu_time_approach; /**< cpu time of the approach data */
    double obj_approach; /**< obj value of the approach data */
    vector<double> x_approach; /**< solution of the approach data */
    vector<double> zL_approach; /**< lower bounds multipliers of the approach data */
    vector<double> zU_approach; /**< upper bounds multipliers of the approach data */
    vector<double> dual_approach; /**< constraints multipliers of the approach data */
    int n_app_steps; /**< steps in the approach stage */

    // retreat data
    bool en_retreat; /**< true if the retreat tab is enabled, false otherwise */
    int iterations_retreat; /**< number of iterations of the retreat data */
    double cpu_time_retreat; /**< cpu time of the retreat data */
    double obj_retreat; /**< obj value of the retreat data */
    vector<double> x_retreat; /**< solution of the retreat data */
    vector<double> zL_retreat; /**< lower bounds multipliers of the retreat data */
    vector<double> zU_retreat; /**< upper bounds multipliers of the retreat data */
    vector<double> dual_retreat; /**< constraints multipliers of the retreat data */
    int n_ret_steps; /**< steps in the retreat stage */

    //bounce data
    bool en_bounce; /**< true if the bounce tab is enabled, false otherwise */
    int iterations_bounce; /**< number of iterations of the bounce data */
    double cpu_time_bounce; /**< cpu time of the bounce data */
    double obj_bounce; /**< obj value of the bounce data */
    vector<double> x_bounce; /**< solution of the bounce data */
    vector<double> zL_bounce; /**< lower bounds multipliers of the bounce data */
    vector<double> zU_bounce; /**< upper bounds multipliers of the bounce data */
    vector<double> dual_bounce; /**< constraints multipliers of the bounce data */




};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
