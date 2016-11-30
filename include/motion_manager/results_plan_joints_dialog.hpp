#ifndef RESULTS_PLAN_JOINTS_DIALOG_HPP
#define RESULTS_PLAN_JOINTS_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_results_plan_joints_dialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ResultsJointsDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief This method saves the tuning parameters to a file
     */
    void on_pushButton_save_clicked();

public:
    /**
     * @brief ResultsJointsDialog, a constructor
     * @param parent
     */
    explicit ResultsJointsDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsJointsDialog, a destructor
     */
    ~ResultsJointsDialog();

    /**
     * @brief setupPlots
     * @param pos
     * @param vel
     * @param acc
     * @param timesteps
     */
    void setupPlots(vector<MatrixXd> &pos,vector<MatrixXd> &vel,vector<MatrixXd> &acc,vector<vector<double>> &timesteps);


private:
    Ui::ResultsJointsDialog *ui; /**< handle of the user interface */
};

} // namespace motion_manager
#endif // RESULTS_PLAN_JOINTS_DIALOG_HPP
