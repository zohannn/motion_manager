#ifndef RESULTS_PLAN_ALPHA_DIALOG_HPP
#define RESULTS_PLAN_ALPHA_DIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ui_results_plan_alpha_dialog.h>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "config.hpp"

namespace motion_manager {

using namespace std;
using namespace Eigen;

class ResultsAlphaDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief on_pushButton_save_clicked
     */
    void on_pushButton_save_clicked();

public:
    /**
     * @brief ResultsAlphaDialog
     * @param parent
     */
    explicit ResultsAlphaDialog(QWidget *parent = 0);

    /**
     * @brief ~ResultsAlphaDialog
     * @param parent
     */
    ~ResultsAlphaDialog();

    /**
     * @brief setupPlots
     * @param pos
     * @param vel
     * @param acc
     * @param timesteps
     */
    void setupPlots(vector<vector<double>> &pos,vector<vector<double>> &vel,vector<vector<double>> &acc,vector<vector<double>> &timesteps);

    /**
     * @brief setDual
     * @param d
     */
    void setDual(bool d);

    /**
     * @brief setRight
     * @param r
     */
    void setRight(bool r);
private:
    Ui::ResultsAlphaDialog *ui; /**< handle of the user interface */
    QVector<double> qtime;
    QVector<double> pos_alpha, vel_alpha, acc_alpha;

    bool dual;

    bool right;

    /**
     * @brief plotAlpha
     * @param plot
     * @param title
     * @param time
     * @param pos
     * @param vel
     * @param acc
     */
    void plotAlpha(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& pos, QVector<double>& vel, QVector<double>& acc);
};

} // namespace motion_manager
#endif // RESULTS_PLAN_ALPHA_DIALOG_HPP
