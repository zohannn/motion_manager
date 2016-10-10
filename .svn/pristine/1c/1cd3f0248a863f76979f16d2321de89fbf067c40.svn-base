#ifndef VREPCOMMDIALOG_HPP
#define VREPCOMMDIALOG_HPP

#include <QDialog>
#include <ui_vrepcommdialog.h>
#include "qnode.hpp"

namespace MotionPlanner{

class VrepCommDialog : public QDialog
{
    Q_OBJECT

public:
    explicit VrepCommDialog(QNode* q, QWidget *parent = 0);
    ~VrepCommDialog();

public Q_SLOTS:
    void on_button_check_clicked(bool check); // check VREP button

Q_SIGNALS:
    void vrepConnected(bool c);

private:
    Ui::VrepCommDialogDesign *ui;
    QNode *qnode;
};

} // namespace MotionPlanner

#endif // VREPCOMMDIALOG_HPP
