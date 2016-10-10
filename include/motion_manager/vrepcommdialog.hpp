#ifndef VREPCOMMDIALOG_HPP
#define VREPCOMMDIALOG_HPP

#include <QDialog>
#include <ui_vrepcommdialog.h>
#include "qnode.hpp"

namespace motion_manager{

//! The VrepCommDialog class
/**
 * @brief This class defines the dialog of the V-REP communication functionalities
 */
class VrepCommDialog : public QDialog
{
    Q_OBJECT

public:

    /**
     * @brief VrepCommDialog, a constructor
     * @param q
     * @param parent
     */
    VrepCommDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~VrepCommDialog, a destructor
     */
    ~VrepCommDialog();

public Q_SLOTS:

    /**
     * @brief This method checks if V-REP has started
     * @param check
     */
    void on_button_check_clicked(bool check);

Q_SIGNALS:

    /**
     * @brief This method signals if vrep is on-line (c=true)
     * @param c
     */
    void vrepConnected(bool c);

private:
    Ui::VrepCommDialogDesign *ui;/**< handle of the user interface */
    QNode *qnode; /**< pointer of the ROS node */
};

} // namespace motion_manager

#endif // VREPCOMMDIALOG_HPP
