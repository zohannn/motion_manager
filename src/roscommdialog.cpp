#include <QMessageBox>
#include "../include/motion_manager/roscommdialog.hpp"



namespace motion_manager {

using namespace Qt;

RosCommDialog::RosCommDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RosCommDialogDesign)
{
    ui->setupUi(this);
    qnode = q;

}


RosCommDialog::~RosCommDialog()
{
    delete ui;

}

void RosCommDialog::setMasterUrl(QString& master_url)
{

    ui->line_edit_master->setText(master_url);
}

void RosCommDialog::setHostUrl(QString& host_url)
{

    ui->line_edit_host->setText(host_url);
}

void RosCommDialog::setRememberCheckbox(bool r)
{

    ui->checkbox_remember_settings->setChecked(r);
}

void RosCommDialog::setUseEnvCheckbox(bool r)
{

    ui->checkbox_use_environment->setChecked(r);

}



void RosCommDialog::enableMasterUrl(bool e)
{

    ui->line_edit_master->setEnabled(e);
}


void RosCommDialog::enableHostUrl(bool e){

    ui->line_edit_host->setEnabled(e);
}


QString RosCommDialog::getMasterUrl()
{


   return ui->line_edit_master->text();

}

QString RosCommDialog::getHostUrl()
{


   return ui->line_edit_host->text();

}

bool RosCommDialog::getRememberCheckbox()
{


    return ui->checkbox_remember_settings->isChecked();
}

bool RosCommDialog::getUseEnvCheckbox()
{

    return ui->checkbox_use_environment->isChecked();
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void RosCommDialog::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    //qnode->on_end();
    //close();
}

void RosCommDialog::on_button_connect_clicked(bool check)
{

    std::string status;
    bool bstatus;

    if (ui->button_connect->isChecked()){
// I want to connect
        if ( ui->checkbox_use_environment->isChecked() ) {
            // use environment is checked
            if ( !qnode->on_init() ) {
                showNoMasterMessage();

                ui->button_connect->setText("Connect");
                ui->button_connect->setChecked(false);
                ui->line_edit_master->setEnabled(true);
                ui->line_edit_host->setEnabled(true);
                ui->line_edit_master->setReadOnly(false);
                ui->line_edit_host->setReadOnly(false);
                ui->checkbox_remember_settings->setEnabled(true);
                ui->checkbox_use_environment->setEnabled(true);
                qnode->on_end();

                //status = "Node disconnected from ROS" ;
                //bstatus=false;

            } else {

                ui->button_connect->setChecked(true);
                ui->line_edit_master->setEnabled(false);
                ui->line_edit_host->setEnabled(false);
                ui->checkbox_remember_settings->setEnabled(false);
                ui->checkbox_use_environment->setEnabled(false);

                ui->button_connect->setText("Disconnect");
                status = "Node connected to ROS";
                bstatus=true;

                qnode->log(QNode::Info,status);
                Q_EMIT rosConnected(bstatus);
            }
        } else {
            if ( ! qnode->on_init_url(ui->line_edit_master->text().toStdString(),
                       ui->line_edit_host->text().toStdString()) ) {
                showNoMasterMessage();

                ui->button_connect->setText("Connect");
                ui->button_connect->setChecked(false);
                ui->line_edit_master->setEnabled(true);
                ui->line_edit_host->setEnabled(true);
                ui->line_edit_master->setReadOnly(false);
                ui->line_edit_host->setReadOnly(false);
                ui->checkbox_remember_settings->setEnabled(true);
                ui->checkbox_use_environment->setEnabled(true);
                qnode->on_end();

                //status = "Node disconnected from ROS" ;
                //bstatus=false;

            } else {

                ui->button_connect->setChecked(true);
                ui->line_edit_master->setEnabled(false);
                ui->line_edit_host->setEnabled(false);
                ui->checkbox_remember_settings->setEnabled(false);
                ui->checkbox_use_environment->setEnabled(false);

                ui->button_connect->setText("Disconnect");
                status = "Node connected to ROS";
                bstatus=true;

                qnode->log(QNode::Info,status);
                Q_EMIT rosConnected(bstatus);

            }
        }




    }else{
// I want to disconnect

        ui->button_connect->setText("Connect");
        ui->button_connect->setChecked(false);

        ui->line_edit_master->setEnabled(true);
        ui->line_edit_host->setEnabled(true);
        ui->line_edit_master->setReadOnly(false);
        ui->line_edit_host->setReadOnly(false);
        ui->checkbox_remember_settings->setEnabled(true);
        ui->checkbox_use_environment->setEnabled(true);
        qnode->on_end();

        status = "Node disconnected from ROS" ;
        bstatus=false;

        qnode->log(QNode::Info,status);
        Q_EMIT rosConnected(bstatus);

    }



}


} // namespace motion_manager
