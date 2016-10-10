#include "../include/motion_manager/toldialoghuml.hpp"


namespace motion_manager {

using namespace Qt;

TolDialogHUML::TolDialogHUML(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TolDialogHUML)
{
    ui->setupUi(this);
}

TolDialogHUML::~TolDialogHUML()
{
    delete ui;
}

//getters

/**
 * @brief TolDialogHUML::getEngageParams
 * @param dist
 * @param dir
 * @param tols
 */
void TolDialogHUML::getEngageParams(float& dist, int& dir, std::vector<float> &tols){

    dist = ui->lineEdit_eng_dist->text().toFloat();
    dir = ui->comboBox_eng_dir->currentIndex();
    tols = std::vector<float>(3);
    tols.at(0) = ui->lineEdit_eng_x->text().toFloat();
    tols.at(1) = ui->lineEdit_eng_y->text().toFloat();
    tols.at(2) = ui->lineEdit_eng_z->text().toFloat();

}

/**
 * @brief TolDialogHUML::getDisengageParams
 * @param dist
 * @param dir
 */
void TolDialogHUML::getDisengageParams(float &dist, int &dir){

    dist = ui->lineEdit_diseng_dist->text().toFloat();
    dir = ui->comboBox_diseng_dir->currentIndex();

}

/**
 * @brief TolDialogHUML::getTolStop
 */
float TolDialogHUML::getTolStop(){

    return ui->lineEdit_tol_stop->text().toFloat();
}

/**
 * @brief TolDialogHUML::getTolsArm
 * @param tols
 */
void TolDialogHUML::getTolsArm(std::vector<float> &tols){

    tols.push_back(ui->lineEdit_shoulder_r->text().toFloat());
    tols.push_back(ui->lineEdit_elbow_r->text().toFloat());
    tols.push_back(ui->lineEdit_wrist_r->text().toFloat());
    tols.push_back(ui->lineEdit_hand_r->text().toFloat());
}

/**
 * @brief TolDialogHUML::getTolsHand
 * @param tols
 */
void TolDialogHUML::getTolsHand(MatrixXf &tols){

   tols = MatrixXf::Constant(4,3,1);

    tols(0,0) = ui->lineEdit_hand_1_1->text().toFloat(); tols(0,1) = ui->lineEdit_hand_1_2->text().toFloat(); tols(0,2) = ui->lineEdit_hand_1_3->text().toFloat();
    tols(1,0) = ui->lineEdit_hand_2_1->text().toFloat(); tols(1,1) = ui->lineEdit_hand_2_2->text().toFloat(); tols(1,2) = ui->lineEdit_hand_2_3->text().toFloat();
    tols(2,0) = ui->lineEdit_hand_3_1->text().toFloat(); tols(2,1) = ui->lineEdit_hand_3_2->text().toFloat(); tols(2,2) = ui->lineEdit_hand_3_3->text().toFloat();
    tols(3,0) = ui->lineEdit_hand_tip_1->text().toFloat(); tols(3,1) = ui->lineEdit_hand_tip_2->text().toFloat(); tols(3,2) = ui->lineEdit_hand_tip_3->text().toFloat();
}

/**
 * @brief TolDialogHUML::getTolsTable
 * @param tols
 */
void TolDialogHUML::getTolsTable(std::vector<float> &tols){


    tols.push_back(ui->lineEdit_table_1->text().toFloat());
    tols.push_back(ui->lineEdit_table_2->text().toFloat());
    tols.push_back(ui->lineEdit_table_3->text().toFloat());

}

/**
 * @brief TolDialogHUML::getLambda
 * @param lambda
 */
void TolDialogHUML::getLambda(std::vector<float> &lambda){

   lambda.push_back(ui->lineEdit_lambda_1->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_2->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_3->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_4->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_5->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_6->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_7->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_8->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_9->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_10->text().toFloat());
   lambda.push_back(ui->lineEdit_lambda_11->text().toFloat());

}

/**
 * @brief TolDialogHUML::getTolsObstacles
 * @param tols
 */
void TolDialogHUML::getTolsObstacles(MatrixXf &tols){

    tols = MatrixXf::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_obs_xx_1->text().toFloat(); tols(0,1)=ui->lineEdit_obs_yy_1->text().toFloat(); tols(0,2)=ui->lineEdit_obs_zz_1->text().toFloat(); tols(0,3)=ui->lineEdit_obs_xy_1->text().toFloat(); tols(0,4)=ui->lineEdit_obs_xz_1->text().toFloat(); tols(0,5)=ui->lineEdit_obs_yz_1->text().toFloat();
    tols(1,0)=ui->lineEdit_obs_xx_2->text().toFloat(); tols(1,1)=ui->lineEdit_obs_yy_2->text().toFloat(); tols(1,2)=ui->lineEdit_obs_zz_2->text().toFloat(); tols(1,3)=ui->lineEdit_obs_xy_2->text().toFloat(); tols(1,4)=ui->lineEdit_obs_xz_2->text().toFloat(); tols(1,5)=ui->lineEdit_obs_yz_2->text().toFloat();
    tols(2,0)=ui->lineEdit_obs_xx_3->text().toFloat(); tols(2,1)=ui->lineEdit_obs_yy_3->text().toFloat(); tols(2,2)=ui->lineEdit_obs_zz_3->text().toFloat(); tols(2,3)=ui->lineEdit_obs_xy_3->text().toFloat(); tols(2,4)=ui->lineEdit_obs_xz_3->text().toFloat(); tols(2,5)=ui->lineEdit_obs_yz_3->text().toFloat();

}

/**
 * @brief TolDialogHUML::getTolsTarget
 * @param tols
 */
void TolDialogHUML::getTolsTarget(MatrixXf &tols){

    tols = MatrixXf::Constant(3,6,1);

    tols(0,0)=ui->lineEdit_tar_xx_1->text().toFloat(); tols(0,1)=ui->lineEdit_tar_yy_1->text().toFloat(); tols(0,2)=ui->lineEdit_tar_zz_1->text().toFloat(); tols(0,3)=ui->lineEdit_tar_xy_1->text().toFloat(); tols(0,4)=ui->lineEdit_tar_xz_1->text().toFloat(); tols(0,5)=ui->lineEdit_tar_yz_1->text().toFloat();
    tols(1,0)=ui->lineEdit_tar_xx_2->text().toFloat(); tols(1,1)=ui->lineEdit_tar_yy_2->text().toFloat(); tols(1,2)=ui->lineEdit_tar_zz_2->text().toFloat(); tols(1,3)=ui->lineEdit_tar_xy_2->text().toFloat(); tols(1,4)=ui->lineEdit_tar_xz_2->text().toFloat(); tols(1,5)=ui->lineEdit_tar_yz_2->text().toFloat();
    tols(2,0)=ui->lineEdit_tar_xx_3->text().toFloat(); tols(2,1)=ui->lineEdit_tar_yy_3->text().toFloat(); tols(2,2)=ui->lineEdit_tar_zz_3->text().toFloat(); tols(2,3)=ui->lineEdit_tar_xy_3->text().toFloat(); tols(2,4)=ui->lineEdit_tar_xz_3->text().toFloat(); tols(2,5)=ui->lineEdit_tar_yz_3->text().toFloat();


}

/**
 * @brief TolDialogHUML::getSteps
 * @return
 */
int TolDialogHUML::getSteps(){

    return ui->lineEdit_steps->text().toInt();
}

/**
 * @brief TolDialogHUML::getWMax
 * @return
 */
float TolDialogHUML::getWMax(){

    return ui->lineEdit_w_max->text().toFloat();
}

/**
 * @brief TolDialogHUML::getApproachAxis
 * @return
 */
int TolDialogHUML::getApproachAxis(){

    return ui->comboBox_approach_axis->currentIndex();


}

/**
 * @brief TolDialogHUML::getTolTarPos
 * @return
 */
float TolDialogHUML::getTolTarPos(){

    return ui->lineEdit_tar_pos->text().toFloat();
}

/**
 * @brief TolDialogHUML::getTolTarOr
 * @return
 */
float TolDialogHUML::getTolTarOr(){

    return ui->lineEdit_tar_or->text().toFloat();
}
/**
 * @brief TolDialogHUML::setInfo
 * @param info
 */
void TolDialogHUML::setInfo(string info){

    this->infoLine = info;
}

/**
 * @brief TolDialogHUML::getTargetAvoidance
 * @return
 */
bool TolDialogHUML::getTargetAvoidance(){

   return !ui->checkBox_tar_av->isChecked();
}


/**
 * @brief TolDialogHUML::getObstacleAvoidance
 * @return
 */
bool TolDialogHUML::getObstacleAvoidance(){

    return !ui->checkBox_ob_av->isChecked();
}

// Q_SLOTS
/**
 * @brief TolDialogHUML::on_pushButton_save_clicked
 */
void TolDialogHUML::on_pushButton_save_clicked(){

   QString filename = QFileDialog::getSaveFileName(this,
                                                   tr("Save the file of tolerances"),
                                                   QDir::currentPath()+"/Tols",
                                                   "All Files (*.*);;Tol Files (*.tol)");
   QFile f( filename );
   if(f.open( QIODevice::WriteOnly )){

       QTextStream stream( &f );
       stream << "### Tolerances for the Human-like Upper-limbs Motion Library ###" << endl;
       stream << "# "<< this->infoLine.c_str() << endl;
       stream << "# Geometric Dimensions of the arms and of the fingers" << endl;
       stream << "Shoulder_radius=" << ui->lineEdit_shoulder_r->text().toStdString().c_str() << endl;
       stream << "Elbow_radius=" << ui->lineEdit_elbow_r->text().toStdString().c_str() << endl;
       stream << "Wrist_radius=" << ui->lineEdit_wrist_r->text().toStdString().c_str() << endl;
       stream << "Hand_radius=" << ui->lineEdit_hand_r->text().toStdString().c_str() << endl;
       stream << "Hand_1_1=" << ui->lineEdit_hand_1_1->text().toStdString().c_str() << endl;
       stream << "Hand_1_2=" << ui->lineEdit_hand_1_2->text().toStdString().c_str() << endl;
       stream << "Hand_1_3=" << ui->lineEdit_hand_1_3->text().toStdString().c_str() << endl;
       stream << "Hand_2_1=" << ui->lineEdit_hand_2_1->text().toStdString().c_str() << endl;
       stream << "Hand_2_2=" << ui->lineEdit_hand_2_2->text().toStdString().c_str() << endl;
       stream << "Hand_2_3=" << ui->lineEdit_hand_2_3->text().toStdString().c_str() << endl;
       stream << "Hand_3_1=" << ui->lineEdit_hand_3_1->text().toStdString().c_str() << endl;
       stream << "Hand_3_2=" << ui->lineEdit_hand_3_2->text().toStdString().c_str() << endl;
       stream << "Hand_3_3=" << ui->lineEdit_hand_3_3->text().toStdString().c_str() << endl;
       stream << "Hand_tip_1=" << ui->lineEdit_hand_tip_1->text().toStdString().c_str() << endl;
       stream << "Hand_tip_2=" << ui->lineEdit_hand_tip_2->text().toStdString().c_str() << endl;
       stream << "Hand_tip_3=" << ui->lineEdit_hand_tip_3->text().toStdString().c_str() << endl;
       stream << "# Tolerance with the table [mm]" << endl;
       stream << "against_wrist="<< ui->lineEdit_table_1->text().toStdString().c_str() << endl;
       stream << "against_hand="<< ui->lineEdit_table_2->text().toStdString().c_str() << endl;
       stream << "against_fingers="<< ui->lineEdit_table_3->text().toStdString().c_str() << endl;
       stream << "# Joint Expanse factors" << endl;
       stream << "lambda_1=" << ui->lineEdit_lambda_1->text().toStdString().c_str() << endl;
       stream << "lambda_2=" << ui->lineEdit_lambda_2->text().toStdString().c_str() << endl;
       stream << "lambda_3=" << ui->lineEdit_lambda_3->text().toStdString().c_str() << endl;
       stream << "lambda_4=" << ui->lineEdit_lambda_4->text().toStdString().c_str() << endl;
       stream << "lambda_5=" << ui->lineEdit_lambda_5->text().toStdString().c_str() << endl;
       stream << "lambda_6=" << ui->lineEdit_lambda_6->text().toStdString().c_str() << endl;
       stream << "lambda_7=" << ui->lineEdit_lambda_7->text().toStdString().c_str() << endl;
       stream << "lambda_8=" << ui->lineEdit_lambda_8->text().toStdString().c_str() << endl;
       stream << "lambda_9=" << ui->lineEdit_lambda_9->text().toStdString().c_str() << endl;
       stream << "lambda_10=" << ui->lineEdit_lambda_10->text().toStdString().c_str() << endl;
       stream << "lambda_11=" << ui->lineEdit_lambda_11->text().toStdString().c_str() << endl;
       stream << "# Tolerances with the target [mm]" << endl;
       stream << "tar_xx_1=" << ui->lineEdit_tar_xx_1->text().toStdString().c_str()<< endl;
       stream << "tar_xx_2=" << ui->lineEdit_tar_xx_2->text().toStdString().c_str()<< endl;
       stream << "tar_xx_3=" << ui->lineEdit_tar_xx_3->text().toStdString().c_str()<< endl;
       stream << "tar_yy_1=" << ui->lineEdit_tar_yy_1->text().toStdString().c_str()<< endl;
       stream << "tar_yy_2=" << ui->lineEdit_tar_yy_2->text().toStdString().c_str()<< endl;
       stream << "tar_yy_3=" << ui->lineEdit_tar_yy_3->text().toStdString().c_str()<< endl;
       stream << "tar_zz_1=" << ui->lineEdit_tar_zz_1->text().toStdString().c_str()<< endl;
       stream << "tar_zz_2=" << ui->lineEdit_tar_zz_2->text().toStdString().c_str()<< endl;
       stream << "tar_zz_3=" << ui->lineEdit_tar_zz_3->text().toStdString().c_str()<< endl;
       stream << "tar_xy_1=" << ui->lineEdit_tar_xy_1->text().toStdString().c_str()<< endl;
       stream << "tar_xy_2=" << ui->lineEdit_tar_xy_2->text().toStdString().c_str()<< endl;
       stream << "tar_xy_3=" << ui->lineEdit_tar_xy_3->text().toStdString().c_str()<< endl;
       stream << "tar_xz_1=" << ui->lineEdit_tar_xz_1->text().toStdString().c_str()<< endl;
       stream << "tar_xz_2=" << ui->lineEdit_tar_xz_2->text().toStdString().c_str()<< endl;
       stream << "tar_xz_3=" << ui->lineEdit_tar_xz_3->text().toStdString().c_str()<< endl;
       stream << "tar_yz_1=" << ui->lineEdit_tar_yz_1->text().toStdString().c_str()<< endl;
       stream << "tar_yz_2=" << ui->lineEdit_tar_yz_2->text().toStdString().c_str()<< endl;
       stream << "tar_yz_3=" << ui->lineEdit_tar_yz_3->text().toStdString().c_str()<< endl;
       stream << "# Tolerances with the obstacles [mm]" << endl;
       stream << "obs_xx_1=" << ui->lineEdit_obs_xx_1->text().toStdString().c_str()<< endl;
       stream << "obs_xx_2=" << ui->lineEdit_obs_xx_2->text().toStdString().c_str()<< endl;
       stream << "obs_xx_3=" << ui->lineEdit_obs_xx_3->text().toStdString().c_str()<< endl;
       stream << "obs_yy_1=" << ui->lineEdit_obs_yy_1->text().toStdString().c_str()<< endl;
       stream << "obs_yy_2=" << ui->lineEdit_obs_yy_2->text().toStdString().c_str()<< endl;
       stream << "obs_yy_3=" << ui->lineEdit_obs_yy_3->text().toStdString().c_str()<< endl;
       stream << "obs_zz_1=" << ui->lineEdit_obs_zz_1->text().toStdString().c_str()<< endl;
       stream << "obs_zz_2=" << ui->lineEdit_obs_zz_2->text().toStdString().c_str()<< endl;
       stream << "obs_zz_3=" << ui->lineEdit_obs_zz_3->text().toStdString().c_str()<< endl;
       stream << "obs_xy_1=" << ui->lineEdit_obs_xy_1->text().toStdString().c_str()<< endl;
       stream << "obs_xy_2=" << ui->lineEdit_obs_xy_2->text().toStdString().c_str()<< endl;
       stream << "obs_xy_3=" << ui->lineEdit_obs_xy_3->text().toStdString().c_str()<< endl;
       stream << "obs_xz_1=" << ui->lineEdit_obs_xz_1->text().toStdString().c_str()<< endl;
       stream << "obs_xz_2=" << ui->lineEdit_obs_xz_2->text().toStdString().c_str()<< endl;
       stream << "obs_xz_3=" << ui->lineEdit_obs_xz_3->text().toStdString().c_str()<< endl;
       stream << "obs_yz_1=" << ui->lineEdit_obs_yz_1->text().toStdString().c_str()<< endl;
       stream << "obs_yz_2=" << ui->lineEdit_obs_yz_2->text().toStdString().c_str()<< endl;
       stream << "obs_yz_3=" << ui->lineEdit_obs_yz_3->text().toStdString().c_str()<< endl;
       stream << "# Tolerances for the final posture" << endl;
       stream << "tar_pos=" << ui->lineEdit_tar_pos->text().toStdString().c_str()<< endl;
       stream << "tar_or="<< ui->lineEdit_tar_or->text().toStdString().c_str() << endl;
       stream << "# Engaging parameters [mm]" << endl;
       stream << "eng_dist=" << ui->lineEdit_eng_dist->text().toStdString().c_str()<< endl;
       stream << "eng_dir=" << ui->comboBox_eng_dir->currentIndex()<< endl;
       stream << "eng_x=" << ui->lineEdit_eng_x->text().toStdString().c_str()<< endl;
       stream << "eng_y=" << ui->lineEdit_eng_y->text().toStdString().c_str()<< endl;
       stream << "eng_z=" << ui->lineEdit_eng_z->text().toStdString().c_str()<< endl;
       stream << "# Disengaging parameters [mm]" << endl;
       stream << "diseng_dist=" << ui->lineEdit_diseng_dist->text().toStdString().c_str()<< endl;
       stream << "diseng_dir=" << ui->comboBox_diseng_dir->currentIndex()<< endl;
       stream << "# Others" << endl;
       stream << "max_velocity="<< ui->lineEdit_w_max->text().toStdString().c_str() <<endl;
       stream << "steps=" << ui->lineEdit_steps->text().toStdString().c_str()<< endl;
       stream << "tar_axis=" << ui->comboBox_approach_axis->currentIndex()<< endl;
       stream << "tol_stop=" << ui->lineEdit_tol_stop->text().toStdString().c_str()<< endl;
       if (ui->checkBox_tar_av->isChecked()){
           stream << "tar_av=false"<< endl;
       }else{
           stream << "tar_av=true"<< endl;
       }
       if (ui->checkBox_ob_av->isChecked()){
           stream << "ob_av=false"<< endl;
       }else{
           stream << "ob_av=true"<< endl;
       }
       //stream << "# END" << endl;


       f.close();
   }




}
/**
 * @brief TolDialogHUML::on_pushButton_load_clicked
 */
void TolDialogHUML::on_pushButton_load_clicked(){

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QDir::currentPath()+"/Tols",
                                                    "All Files (*.*);; Tol Files (*.tol)");
    QFile f( filename );
    if(f.open( QIODevice::ReadOnly )){

        QTextStream stream( &f );
        QString line;

        while(!stream.atEnd()){

            line = f.readLine();
            if(line.at(0)!=QChar('#')){

                 //std::cout << line.toStdString().c_str();
                QStringList fields = line.split("=");

                //std::cout << fields.at(0).toStdString().c_str() << "\n";
                //std::cout << fields.at(1).toStdString().c_str() << "\n";

                if (QString::compare(fields.at(0),QString("Shoulder_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_shoulder_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Elbow_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_elbow_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Wrist_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_wrist_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_radius"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_r->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_1_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_1_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_2_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_2_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_3_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_3_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("Hand_tip_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_hand_tip_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("against_wrist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_table_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("against_hand"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_table_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("against_fingers"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_table_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_4"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_4->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_5"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_5->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_6"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_6->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_7"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_7->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_8"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_8->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_9"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_9->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_10"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_10->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("lambda_11"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_lambda_11->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xx_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xx_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_zz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_zz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_xz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_xz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_yz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_yz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xx_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xx_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_zz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_zz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xy_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xy_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_xz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_xz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_1"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_1->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_2"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_2->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("obs_yz_3"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_obs_yz_3->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_pos"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_pos->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_or"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tar_or->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("eng_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_eng_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("eng_dir"),Qt::CaseInsensitive)==0){
                    ui->comboBox_eng_dir->setCurrentIndex(fields.at(1).toInt());
                }else if(QString::compare(fields.at(0),QString("eng_x"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_eng_x->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("eng_y"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_eng_y->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("eng_z"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_eng_z->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("diseng_dist"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_diseng_dist->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("diseng_dir"),Qt::CaseInsensitive)==0){
                    ui->comboBox_diseng_dir->setCurrentIndex(fields.at(1).toInt());
                }else if(QString::compare(fields.at(0),QString("max_velocity"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_w_max->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("steps"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_steps->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_axis"),Qt::CaseInsensitive)==0){
                    ui->comboBox_approach_axis->setCurrentIndex(fields.at(1).toInt());
                }else if(QString::compare(fields.at(0),QString("tol_stop"),Qt::CaseInsensitive)==0){
                    ui->lineEdit_tol_stop->setText(fields.at(1));
                }else if(QString::compare(fields.at(0),QString("tar_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_tar_av->setChecked(true);
                    }else{
                        ui->checkBox_tar_av->setChecked(false);
                    }
                }else if(QString::compare(fields.at(0),QString("ob_av"),Qt::CaseInsensitive)==0){
                    if(QString::compare(fields.at(1),QString("false\n"),Qt::CaseInsensitive)==0){
                        ui->checkBox_ob_av->setChecked(true);
                    }else{
                        ui->checkBox_ob_av->setChecked(false);
                    }
                }

            }

        }

        f.close();

    }

}




} // namespace motion_manager
