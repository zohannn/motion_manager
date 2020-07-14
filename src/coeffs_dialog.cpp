#include "../include/motion_manager/coeffs_dialog.hpp"

namespace motion_manager {

CoeffsDialog::CoeffsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CoeffsDialog)
{
    ui->setupUi(this);

    QObject::connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    QObject::connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}

CoeffsDialog::~CoeffsDialog()
{
    delete ui;
}



void CoeffsDialog::getPositionCoeffs(double &kpx, double &kpy, double &kpz, double &kox, double &koy, double &koz)
{
    kpx = this->kpx;
    kpy = this->kpy;
    kpz = this->kpz;
    kox = this->kox;
    koy = this->koy;
    koz = this->koz;
}

void CoeffsDialog::getVelocityCoeffs(double &kdx, double &kdy, double &kdz, double &kwx, double &kwy, double &kwz)
{
    kdx = this->kdx;
    kdy = this->kdy;
    kdz = this->kdz;
    kwx = this->kwx;
    kwy = this->kwy;
    kwz = this->kwz;
}



// Q_SLOTS


void CoeffsDialog::accept()
{
    this->kpx = this->ui->lineEdit_kp_x_pos->text().toDouble();
    this->kpy = this->ui->lineEdit_kp_y_pos->text().toDouble();
    this->kpz = this->ui->lineEdit_kp_z_pos->text().toDouble();
    this->kox = this->ui->lineEdit_kp_x_or->text().toDouble();
    this->koy = this->ui->lineEdit_kp_y_or->text().toDouble();
    this->koz = this->ui->lineEdit_kp_z_or->text().toDouble();

    this->kdx = this->ui->lineEdit_kd_x_vel->text().toDouble();
    this->kdy = this->ui->lineEdit_kd_y_vel->text().toDouble();
    this->kdz = this->ui->lineEdit_kd_z_vel->text().toDouble();
    this->kwx = this->ui->lineEdit_kd_x_omega->text().toDouble();
    this->kwy = this->ui->lineEdit_kd_y_omega->text().toDouble();
    this->kwz = this->ui->lineEdit_kd_z_omega->text().toDouble();

    this->hide();
}

void CoeffsDialog::reject()
{
    this->hide();
}

void CoeffsDialog::on_pushButton_save_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of control coefficients"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "coeff Files (*.coeff)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        stream << "### Coefficients of the control ###" << endl;
        stream << "# Position #" << endl;
        stream << "kpx="<< ui->lineEdit_kp_x_pos->text().toStdString().c_str() << endl;
        stream << "kpy="<< ui->lineEdit_kp_y_pos->text().toStdString().c_str() << endl;
        stream << "kpz="<< ui->lineEdit_kp_z_pos->text().toStdString().c_str() << endl;
        stream << "kox="<< ui->lineEdit_kp_x_or->text().toStdString().c_str() << endl;
        stream << "koy="<< ui->lineEdit_kp_y_or->text().toStdString().c_str() << endl;
        stream << "koz="<< ui->lineEdit_kp_z_or->text().toStdString().c_str() << endl;
        stream << "# Velocity #" << endl;
        stream << "kdx="<< ui->lineEdit_kd_x_vel->text().toStdString().c_str() << endl;
        stream << "kdy="<< ui->lineEdit_kd_y_vel->text().toStdString().c_str() << endl;
        stream << "kdz="<< ui->lineEdit_kd_z_vel->text().toStdString().c_str() << endl;
        stream << "kwx="<< ui->lineEdit_kd_x_omega->text().toStdString().c_str() << endl;
        stream << "kwy="<< ui->lineEdit_kd_y_omega->text().toStdString().c_str() << endl;
        stream << "kwz="<< ui->lineEdit_kd_z_omega->text().toStdString().c_str() << endl;

    }
    f.close();
}

void CoeffsDialog::on_pushButton_load_clicked()
{

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of coefficients"),
                                                    QString(MAIN_PATH)+"/Control",
                                                    "coeff Files (*.coeff)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){
        QTextStream stream( &f );
        QString line;
        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)!=QChar('#')){
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0),QString("kpx"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_x_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kpy"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_y_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kpz"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_z_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kox"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_x_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("koy"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_y_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("koz"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kp_z_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kdx"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_x_vel->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kdy"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_y_vel->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kdz"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_z_vel->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kwx"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_x_omega->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kwy"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_y_omega->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("kwz"),Qt::CaseInsensitive)==0){
                    this->ui->lineEdit_kd_z_omega->setText(fields.at(1));
                }
            } // line
        }// while
     f.close();
    }
}
} // namespace motion_manager
