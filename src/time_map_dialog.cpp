#include "../include/motion_manager/time_map_dialog.hpp"

namespace motion_manager {

TimeMapDialog::TimeMapDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TimeMapDialog)
{
    ui->setupUi(this);

    QObject::connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    QObject::connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}

TimeMapDialog::~TimeMapDialog()
{
    delete ui;
}



void TimeMapDialog::getPlanTimeMapping(double& tau, double& dec_rate, double& diff_w)
{
    tau = this->tau_plan;
    dec_rate = this->dec_rate_plan;
    diff_w = this->diff_w_plan;
}

void TimeMapDialog::getApproachTimeMapping(double& tau, double& dec_rate, double& diff_w)
{
    tau = this->tau_app;
    dec_rate = this->dec_rate_app;
    diff_w = this->diff_w_app;
}

void TimeMapDialog::getRetreatTimeMapping(double& tau, double& dec_rate, double& diff_w)
{
    tau = this->tau_ret;
    dec_rate = this->dec_rate_ret;
    diff_w = this->diff_w_ret;

}


// Q_SLOTS


void TimeMapDialog::accept()
{
    this->tau_plan = this->ui->lineEdit_tau_plan->text().toDouble();
    this->dec_rate_plan = this->ui->lineEdit_dec_rate_plan->text().toDouble();
    this->diff_w_plan = this->ui->lineEdit_diff_w_plan->text().toDouble();

    this->tau_app = this->ui->lineEdit_tau_app->text().toDouble();
    this->dec_rate_app = this->ui->lineEdit_dec_rate_app->text().toDouble();
    this->diff_w_app = this->ui->lineEdit_diff_w_app->text().toDouble();

    this->tau_ret = this->ui->lineEdit_tau_ret->text().toDouble();
    this->dec_rate_ret = this->ui->lineEdit_dec_rate_ret->text().toDouble();
    this->diff_w_ret = this->ui->lineEdit_diff_w_ret->text().toDouble();

    this->hide();
}

void TimeMapDialog::reject()
{
    this->hide();
}

void TimeMapDialog::on_pushButton_save_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "tmap Files (*.tmap)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        stream << "### Parameters of the Time mapping ###" << endl;
        stream << "# Plan stage #" << endl;
        stream << "tau_plan="<< ui->lineEdit_tau_plan->text().toStdString().c_str() << endl;
        stream << "dec_rate_plan="<< ui->lineEdit_dec_rate_plan->text().toStdString().c_str() << endl;
        stream << "diff_w_plan="<< ui->lineEdit_diff_w_plan->text().toStdString().c_str() << endl;
        stream << "# Approach stage #" << endl;
        stream << "tau_app="<< ui->lineEdit_tau_app->text().toStdString().c_str() << endl;
        stream << "dec_rate_app="<< ui->lineEdit_dec_rate_app->text().toStdString().c_str() << endl;
        stream << "diff_w_app="<< ui->lineEdit_diff_w_app->text().toStdString().c_str() << endl;
        stream << "# Retreat stage #" << endl;
        stream << "tau_ret="<< ui->lineEdit_tau_ret->text().toStdString().c_str() << endl;
        stream << "dec_rate_ret="<< ui->lineEdit_dec_rate_ret->text().toStdString().c_str() << endl;
        stream << "diff_w_ret="<< ui->lineEdit_diff_w_ret->text().toStdString().c_str() << endl;
    }
    f.close();
}

void TimeMapDialog::on_pushButton_load_clicked()
{

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "tmap Files (*.tmap)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){
        QTextStream stream( &f );
        QString line;
        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)!=QChar('#')){
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0),QString("tau_plan"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_tau_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("dec_rate_plan"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_dec_rate_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("diff_w_plan"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_diff_w_plan->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tau_app"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_tau_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("dec_rate_app"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_dec_rate_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("diff_w_app"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_diff_w_app->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tau_ret"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_tau_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("dec_rate_ret"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_dec_rate_ret->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("diff_w_ret"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_diff_w_ret->setText(fields.at(1));
                }
            } // line
        }// while
     f.close();
    }
}
} // namespace motion_manager
