#include "../include/motion_manager/vrepcommdialog.hpp"


namespace motion_manager {

using namespace Qt;


VrepCommDialog::VrepCommDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VrepCommDialogDesign)
{
    ui->setupUi(this);
    qnode = q;
}


VrepCommDialog::~VrepCommDialog()
{
    delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void VrepCommDialog::on_button_check_clicked(bool check)
{

    bool bstatus = false;

    if (qnode->checkVrep()){

        ui->labelStatusVrep->setText("on-line");
        qnode->log(QNode::Info,"V-REP is on-line");
        bstatus = true;

    }else{

        ui->labelStatusVrep->setText("off-line");
        qnode->log(QNode::Info,"V-REP is off-line");
        bstatus = false;

    }
    Q_EMIT vrepConnected(bstatus);
}

} // namespace motion_manager
