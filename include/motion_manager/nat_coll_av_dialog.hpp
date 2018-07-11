#ifndef NAT_COLL_AV_DIALOG_HPP
#define NAT_COLL_AV_DIALOG

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_nat_coll_avdialog.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class NatCollAvDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:


    /**
     * @brief on_pushButton_save_nat_coll_av_clicked
     */
    void on_pushButton_save_nat_coll_av_clicked();


public:
    /**
     * @brief NatCollAvDialog, a constructor
     * @param parent
     */
    explicit NatCollAvDialog(QWidget *parent = 0);

    /**
     * @brief ~NatCollAvDialog, a destructor
     */
    ~NatCollAvDialog();

    /**
     * @brief setupPlots
     * @param hand_linear_velocity
     * @param hand_position
     * @param timesteps
     */
    void setupPlots(vector<vector<double>> &hand_linear_velocity,vector<vector<double> > &hand_position, vector<vector<vector<double> > > &timesteps);

private:
    Ui::Nat_coll_av_Dialog *ui; /**< handle of the user interface */

    /**
     * @brief plotComp
     * @param plot
     * @param title
     * @param time
     * @param var
     * @param lin
     */
    void plotComp(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& var,bool lin);

    /**
     * @brief getDerivative
     * @param function
     * @param step_values
     * @param derFunction
     */
    void getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction);

};

} // namespace motion_manager
#endif // NAT_COLL_AV_DIALOG
