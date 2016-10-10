#include "../include/MotionPlanner/vrepcommdialog.hpp"


namespace MotionPlanner {

using namespace Qt;


// constructors

VrepCommDialog::VrepCommDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VrepCommDialogDesign)
{
    ui->setupUi(this);
    qnode = q;
}


// descructors
VrepCommDialog::~VrepCommDialog()
{
    delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/**
 * @brief VrepCommDialog::on_button_check_clicked
 * @param check
 */
void VrepCommDialog::on_button_check_clicked(bool check){

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

} // namespace MotionPlanner
