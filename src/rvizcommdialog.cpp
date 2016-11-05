#include "../include/motion_manager/rvizcommdialog.hpp"

namespace motion_manager {

using namespace Qt;

RVizCommDialog::RVizCommDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RVizCommDialogDesign)
{
    ui->setupUi(this);
    qnode = q;
}


RVizCommDialog::~RVizCommDialog()
{
    delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void RVizCommDialog::on_button_check_clicked(bool check)
{

    bool bstatus = false;

    if (qnode->checkRViz()){

        ui->labelStatusRViz->setText("on-line");
        qnode->log(QNode::Info,"RViz is on-line");
        bstatus = true;

    }else{

        ui->labelStatusRViz->setText("off-line");
        qnode->log(QNode::Info,"RViz is off-line");
        bstatus = false;

    }
    Q_EMIT rvizConnected(bstatus);
}


} // namespace motion_manager
