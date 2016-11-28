#include "../include/motion_manager/prmstardialog.hpp"

namespace motion_manager {

PRMstarDialog::PRMstarDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PRMstarDialog)
{
    ui->setupUi(this);

    QObject::connect(ui->checkBox_sel_final_posture, SIGNAL(stateChanged(int)), this, SLOT(checkFinalPosture(int)));
    QObject::connect(ui->checkBox_add_plane, SIGNAL(stateChanged(int)), this, SLOT(checkAddPlane(int)));
    this->config = std::string("PRMstarkConfigDefault");
}

PRMstarDialog::~PRMstarDialog()
{
    delete ui;
}

void PRMstarDialog::getPreGraspApproach(std::vector<double> &pre_grasp)
{
    pre_grasp.clear();
    pre_grasp.push_back(ui->lineEdit_pre_grasp_x->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_y->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_z->text().toDouble());
    pre_grasp.push_back(ui->lineEdit_pre_grasp_dist->text().toDouble()/1000);
}

void PRMstarDialog::getPostGraspRetreat(std::vector<double> &post_grasp)
{
    post_grasp.clear();
    post_grasp.push_back(ui->lineEdit_post_grasp_x->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_y->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_z->text().toDouble());
    post_grasp.push_back(ui->lineEdit_post_grasp_dist->text().toDouble()/1000);
}

void PRMstarDialog::getPrePlaceApproach(std::vector<double> &pre_place)
{
    pre_place.clear();
    pre_place.push_back(ui->lineEdit_pre_place_x->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_y->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_z->text().toDouble());
    pre_place.push_back(ui->lineEdit_pre_place_dist->text().toDouble()/1000);
}

void PRMstarDialog::getPostPlaceRetreat(std::vector<double> &post_place)
{
    post_place.clear();
    post_place.push_back(ui->lineEdit_post_place_x->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_y->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_z->text().toDouble());
    post_place.push_back(ui->lineEdit_post_place_dist->text().toDouble()/1000);
}

std::string PRMstarDialog::getConfig()
{

    if(ui->radioButton_default->isChecked()){
        this->config = std::string("PRMstarkConfigDefault");
    }
    if(ui->radioButton_1->isChecked()){
        this->config = std::string("PRMstarkConfig1");
    }
    if(ui->radioButton_2->isChecked()){
        this->config = std::string("PRMstarkConfig2");
    }
    if(ui->radioButton_3->isChecked()){
        this->config = std::string("PRMstarkConfig3");
    }

    return this->config;

}

void PRMstarDialog::getTargetMove(std::vector<double> &target)
{
    target.clear();
    target.push_back(ui->lineEdit_target_x->text().toDouble()/1000);
    target.push_back(ui->lineEdit_target_y->text().toDouble()/1000);
    target.push_back(ui->lineEdit_target_z->text().toDouble()/1000);
    target.push_back(ui->lineEdit_target_roll->text().toDouble());
    target.push_back(ui->lineEdit_target_pitch->text().toDouble());
    target.push_back(ui->lineEdit_target_yaw->text().toDouble());
}

void PRMstarDialog::getFinalArm(std::vector<double> &finalArm)
{
    finalArm.clear();
    finalArm.push_back(ui->lineEdit_final_arm_1->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_2->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_3->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_4->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_5->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_6->text().toDouble());
    finalArm.push_back(ui->lineEdit_final_arm_7->text().toDouble());
}

void PRMstarDialog::getFinalHand(std::vector<double> &finalHand)
{
    finalHand.clear();
    finalHand.push_back(ui->lineEdit_final_hand_1->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_2->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_3->text().toDouble());
    finalHand.push_back(ui->lineEdit_final_hand_4->text().toDouble());
}


void PRMstarDialog::getPlaneParameters(std::vector<double> &params,
                                   std::vector<double> &point1,
                                   std::vector<double> &point2,
                                   std::vector<double> &point3)
{
    params.clear(); double a,b,c,d;
    point1.clear(); point2.clear(); point3.clear();

    point1.push_back(ui->lineEdit_point_1_x->text().toDouble()/1000);
    point1.push_back(ui->lineEdit_point_1_y->text().toDouble()/1000);
    point1.push_back(ui->lineEdit_point_1_z->text().toDouble()/1000);

    point2.push_back(ui->lineEdit_point_2_x->text().toDouble()/1000);
    point2.push_back(ui->lineEdit_point_2_y->text().toDouble()/1000);
    point2.push_back(ui->lineEdit_point_2_z->text().toDouble()/1000);

    point3.push_back(ui->lineEdit_point_3_x->text().toDouble()/1000);
    point3.push_back(ui->lineEdit_point_3_y->text().toDouble()/1000);
    point3.push_back(ui->lineEdit_point_3_z->text().toDouble()/1000);


    Matrix3d D; double det;
    D << point1.at(0),point1.at(1),point1.at(2),
        point2.at(0),point2.at(1),point2.at(2),
        point3.at(0),point3.at(1),point3.at(2);

    det = D.determinant();

    if(det!=0){
        d=1;

        Matrix3d A;
        A << 1,point1.at(1),point1.at(2),
            1,point2.at(1),point2.at(2),
            1,point3.at(1),point3.at(2);
        a = (-d/det)*A.determinant();

        Matrix3d B;
        B << point1.at(0),1,point1.at(2),
            point2.at(0),1,point2.at(2),
            point3.at(0),1,point3.at(2);
        b = (-d/det)*B.determinant();

        Matrix3d C;
        C << point1.at(0),point1.at(1),1,
            point2.at(0),point2.at(1),1,
            point3.at(0),point3.at(1),1;
        c = (-d/det)*C.determinant();

        params.push_back(a);
        params.push_back(b);
        params.push_back(c);
        params.push_back(d);
    }
}

// Q_SLOTS

void PRMstarDialog::on_pushButton_save_clicked()
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
        stream << "target_x=" << ui->lineEdit_target_x->text().toStdString().c_str() << endl;
        stream << "target_y=" << ui->lineEdit_target_y->text().toStdString().c_str() << endl;
        stream << "target_z=" << ui->lineEdit_target_z->text().toStdString().c_str() << endl;
        stream << "target_roll=" << ui->lineEdit_target_roll->text().toStdString().c_str() << endl;
        stream << "target_pitch=" << ui->lineEdit_target_pitch->text().toStdString().c_str() << endl;
        stream << "target_yaw=" << ui->lineEdit_target_yaw->text().toStdString().c_str() << endl;
        stream << "final_arm_1=" << ui->lineEdit_final_arm_1->text().toStdString().c_str() << endl;
        stream << "final_arm_2=" << ui->lineEdit_final_arm_2->text().toStdString().c_str() << endl;
        stream << "final_arm_3=" << ui->lineEdit_final_arm_3->text().toStdString().c_str() << endl;
        stream << "final_arm_4=" << ui->lineEdit_final_arm_4->text().toStdString().c_str() << endl;
        stream << "final_arm_5=" << ui->lineEdit_final_arm_5->text().toStdString().c_str() << endl;
        stream << "final_arm_6=" << ui->lineEdit_final_arm_6->text().toStdString().c_str() << endl;
        stream << "final_arm_7=" << ui->lineEdit_final_arm_7->text().toStdString().c_str() << endl;
        stream << "final_hand_1=" << ui->lineEdit_final_hand_1->text().toStdString().c_str() << endl;
        stream << "final_hand_2=" << ui->lineEdit_final_hand_2->text().toStdString().c_str() << endl;
        stream << "final_hand_3=" << ui->lineEdit_final_hand_3->text().toStdString().c_str() << endl;
        stream << "final_hand_4=" << ui->lineEdit_final_hand_4->text().toStdString().c_str() << endl;
        if (ui->checkBox_sel_final_posture->isChecked()){ stream << "sel_final=true"<< endl;}else{stream << "sel_final=false"<< endl;}
        stream << "plane_point1_x=" << ui->lineEdit_point_1_x->text().toStdString().c_str() << endl;
        stream << "plane_point1_y=" << ui->lineEdit_point_1_y->text().toStdString().c_str() << endl;
        stream << "plane_point1_z=" << ui->lineEdit_point_1_z->text().toStdString().c_str() << endl;
        stream << "plane_point2_x=" << ui->lineEdit_point_2_x->text().toStdString().c_str() << endl;
        stream << "plane_point2_y=" << ui->lineEdit_point_2_y->text().toStdString().c_str() << endl;
        stream << "plane_point2_z=" << ui->lineEdit_point_2_z->text().toStdString().c_str() << endl;
        stream << "plane_point3_x=" << ui->lineEdit_point_3_x->text().toStdString().c_str() << endl;
        stream << "plane_point3_y=" << ui->lineEdit_point_3_y->text().toStdString().c_str() << endl;
        stream << "plane_point3_z=" << ui->lineEdit_point_3_z->text().toStdString().c_str() << endl;
        if (ui->checkBox_add_plane->isChecked()){ stream << "add_plane=true"<< endl;}else{stream << "add_plane=false"<< endl;}
        stream << "# Others" << endl;

    }
}

void PRMstarDialog::on_pushButton_load_clicked()
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
                }else if(QString::compare(fields.at(0),QString("target_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_roll"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_roll->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_pitch"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_pitch->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("target_yaw"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_target_yaw->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_arm_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_arm_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("final_hand_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_final_hand_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("sel_final"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_sel_final_posture->setChecked(false);
                    }else{
                        ui->checkBox_sel_final_posture->setChecked(true);
                    }
                }else if(QString::compare(fields.at(0),QString("plane_point1_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point1_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point1_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_1_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point2_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_2_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("plane_point3_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_point_3_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("add_plane"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_add_plane->setChecked(false);
                    }else{
                        ui->checkBox_add_plane->setChecked(true);
                    }
                }
            }

        }

        if(strcmp(this->config.c_str(),"PRMstarkConfigDefault")==0){
            ui->radioButton_default->setChecked(true);
        }else if(strcmp(this->config.c_str(),"PRMstarkConfig1")==0){
            ui->radioButton_1->setChecked(true);
        }else if(strcmp(this->config.c_str(),"PRMstarkConfig2")==0){
            ui->radioButton_2->setChecked(true);
        }else if(strcmp(this->config.c_str(),"PRMstarkConfig3")==0){
            ui->radioButton_3->setChecked(true);
        }

    }// while loop
}

void PRMstarDialog::setInfo(std::string info)
{

    this->infoLine = info;
}

void PRMstarDialog::checkFinalPosture(int state)
{
    if(state==0){
        // unchecked
        ui->groupBox_target->setEnabled(true);
        ui->groupBox_final_arm->setEnabled(false);
    }else{
        //checked
        ui->groupBox_target->setEnabled(false);
        ui->groupBox_final_arm->setEnabled(true);
    }
}

void PRMstarDialog::checkAddPlane(int state)
{
    if(state==0){
        //unchecked
        ui->groupBox_plane->setEnabled(false);
    }else{
        //checked
        ui->groupBox_plane->setEnabled(true);
    }
}

bool PRMstarDialog::get_use_final_posture()
{
    return ui->checkBox_sel_final_posture->isChecked();
}

bool PRMstarDialog::get_add_plane()
{
    return ui->checkBox_add_plane->isChecked();
}




} // namespace motion_manager
