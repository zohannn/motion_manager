#include "../include/motion_manager/rrtconnectdialog.hpp"

namespace motion_manager {

RRTConnectDialog::RRTConnectDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RRTConnectDialog)
{
    ui->setupUi(this);
    this->config = std::string("RRTConnectkConfigDefault");
}

RRTConnectDialog::~RRTConnectDialog()
{
    delete ui;
}

void RRTConnectDialog::getPreGraspApproach(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist->text().toDouble()/1000);
}

void RRTConnectDialog::getPostGraspRetreat(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist->text().toDouble()/1000);
}

void RRTConnectDialog::getPrePlaceApproach(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist->text().toDouble()/1000);
}

void RRTConnectDialog::getPostPlaceRetreat(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist->text().toDouble()/1000);
}

std::string RRTConnectDialog::getConfig()
{

    if(ui->radioButton_default->isChecked()){
        this->config = std::string("RRTConnectkConfigDefault");
    }
    if(ui->radioButton_1->isChecked()){
        this->config = std::string("RRTConnectkConfig1");
    }
    if(ui->radioButton_2->isChecked()){
        this->config = std::string("RRTConnectkConfig2");
    }

    return this->config;

}

// Q_SLOTS

void RRTConnectDialog::on_pushButton_save_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Tols",
                                                    "All Files (*.*);;Tol Files (*.tol)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        stream << "### Parameters of the RRT planner in MoveIt! ###" << endl;
        stream << "# "<< this->infoLine.c_str() << endl;
        stream << "# Configuration settings" << endl;
        stream << "config=" << this->config.c_str() << endl;
        stream << "# Pick settings" << endl;
        stream << "pre_grasp_x="<< ui->lineEdit_pre_grasp_x->text().toStdString().c_str() << endl;
        stream << "pre_grasp_y="<< ui->lineEdit_pre_grasp_y->text().toStdString().c_str() << endl;
        stream << "pre_grasp_z="<< ui->lineEdit_pre_grasp_z->text().toStdString().c_str() << endl;
        stream << "pre_grasp_dist="<< ui->lineEdit_pre_grasp_dist->text().toStdString().c_str() << endl;
        stream << "post_grasp_x="<< ui->lineEdit_post_grasp_x->text().toStdString().c_str() << endl;
        stream << "post_grasp_y="<< ui->lineEdit_post_grasp_y->text().toStdString().c_str() << endl;
        stream << "post_grasp_z="<< ui->lineEdit_post_grasp_z->text().toStdString().c_str() << endl;
        stream << "post_grasp_dist="<< ui->lineEdit_post_grasp_dist->text().toStdString().c_str() << endl;
        stream << "# Place settings" << endl;
        stream << "pre_place_x="<< ui->lineEdit_pre_place_x->text().toStdString().c_str() << endl;
        stream << "pre_place_y="<< ui->lineEdit_pre_place_y->text().toStdString().c_str() << endl;
        stream << "pre_place_z="<< ui->lineEdit_pre_place_z->text().toStdString().c_str() << endl;
        stream << "pre_place_dist="<< ui->lineEdit_pre_place_dist->text().toStdString().c_str() << endl;
        stream << "post_place_x="<< ui->lineEdit_post_place_x->text().toStdString().c_str() << endl;
        stream << "post_place_y="<< ui->lineEdit_post_place_y->text().toStdString().c_str() << endl;
        stream << "post_place_z="<< ui->lineEdit_post_place_z->text().toStdString().c_str() << endl;
        stream << "post_place_dist="<< ui->lineEdit_post_place_dist->text().toStdString().c_str() << endl;
        stream << "# Move settings" << endl;
        stream << "# Others" << endl;

    }
}

void RRTConnectDialog::on_pushButton_load_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Tols",
                                                    "All Files (*.*);; Tol Files (*.tol)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;
        while(!stream.atEnd()){
            line = f.readLine();
            if(line.at(0)!=QChar('#')){
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0),QString("config"),Qt::CaseInsensitive)==0){
                    this->config = fields.at(1).simplified().toStdString();
                }else if(QString::compare(fields.at(0),QString("pre_grasp_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_grasp_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_grasp_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_grasp_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_grasp_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("pre_place_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_pre_place_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("post_place_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_post_place_dist->setText(fields.at(1));
                }
            }

        }

        if(strcmp(this->config.c_str(),"RRTConnectkConfigDefault")==0){
            ui->radioButton_default->setChecked(true);
        }else if(strcmp(this->config.c_str(),"RRTConnectkConfig1")==0){
            ui->radioButton_1->setChecked(true);
        }else if(strcmp(this->config.c_str(),"RRTConnectkConfig2")==0){
            ui->radioButton_2->setChecked(true);
        }

    }
}

void RRTConnectDialog::setInfo(std::string info)
{

    this->infoLine = info;
}




} // namespace motion_manager
