#ifndef RVIZCOMMDIALOG_HPP
#define RVIZCOMMDIALOG_HPP

#include <QDialog>
#include <ui_rvizcommdialog.h>
#include "qnode.hpp"


namespace motion_manager{

//! The RVizCommDialog class
/**
 * @brief This class defines the dialog of the RViz communication functionalities
 */
class RVizCommDialog : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief RVizCommDialog, a constructor
     * @param q
     * @param parent
     */
    RVizCommDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~RVizCommDialog, a destructor
     */
    ~RVizCommDialog();

public Q_SLOTS:

    /**
     * @brief This method checks if RViz has started
     * @param check
     */
    void on_button_check_clicked(bool check);

Q_SIGNALS:

    /**
     * @brief This method signals if RViz is on-line (c=true)
     * @param c
     */
    void rvizConnected(bool c);


private:
    Ui::RVizCommDialogDesign *ui;/**< handle of the user interface */
    QNode *qnode; /**< pointer of the ROS node */

};




} // namespace motion_manager

#endif // RVIZCOMMDIALOG_HPP
