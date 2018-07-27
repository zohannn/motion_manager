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
     */
    void setPlanData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars);

    /**
     * @brief setApproachData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     */
    void setApproachData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars);

    /**
     * @brief setRetreatData
     * @param iter
     * @param cpu_time
     * @param obj
     * @param x
     * @param zL
     * @param zU
     * @param dual_vars
     */
    void setRetreatData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars);

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


};

} // namespace motion_manager
#endif // COMP_VELOCITY_DIALOG
