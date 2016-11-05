#ifndef RRTDIALOG_HPP
#define RRTDIALOG_HPP

#include <QDialog>
#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_rrtdialog.h>
#include "config.hpp"

namespace motion_manager {

class RRTDialog : public QDialog
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

public:
    /**
     * @brief RRTDialog, a constructor
     * @param parent
     */
    explicit RRTDialog(QWidget *parent = 0);

    /**
     * @brief ~RRTDialog, a destructor
     */
    ~RRTDialog();

    /**
     * @brief getPreGraspApproach
     * @param pre_grasp
     */
    void getPreGraspApproach(std::vector<double>& pre_grasp);

    /**
     * @brief getPostGraspRetreat
     * @param post_grasp
     */
    void getPostGraspRetreat(std::vector<double>& post_grasp);

    /**
     * @brief getPrePlaceApproach
     * @param pre_place
     */
    void getPrePlaceApproach(std::vector<double>& pre_place);

    /**
     * @brief getPostPlaceRetreat
     * @param post_place
     */
    void getPostPlaceRetreat(std::vector<double>& post_place);

    /**
     * @brief getConfig
     * @return
     */
    std::string getConfig();


private:
    Ui::RRTDialog *ui; /**< handle of the user interface */
    std::string config; /**< current configuration */
};

} // namespace motion_manager
#endif // RRTDIALOG_HPP
