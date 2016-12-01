#include "../include/motion_manager/results_plan_joints_dialog.hpp"

namespace motion_manager {

ResultsJointsDialog::ResultsJointsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ResultsJointsDialog)
{
    ui->setupUi(this);
}

ResultsJointsDialog::~ResultsJointsDialog()
{
    delete ui;
}


void ResultsJointsDialog::setupPlots(vector<MatrixXd> &pos, vector<MatrixXd> &vel, vector<MatrixXd> &acc, vector<vector<double> > &timesteps)
{
    const double radtodeg = 180.0/static_cast<double>(M_PI);

    vector<double> time;
    QVector<double> pos_joint1, vel_joint1, acc_joint1;
    QVector<double> pos_joint2, vel_joint2, acc_joint2;
    QVector<double> pos_joint3, vel_joint3, acc_joint3;
    QVector<double> pos_joint4, vel_joint4, acc_joint4;
    QVector<double> pos_joint5, vel_joint5, acc_joint5;
    QVector<double> pos_joint6, vel_joint6, acc_joint6;
    QVector<double> pos_joint7, vel_joint7, acc_joint7;
    QVector<double> pos_joint8, vel_joint8, acc_joint8;
    QVector<double> pos_joint9, vel_joint9, acc_joint9;
    QVector<double> pos_joint10, vel_joint10, acc_joint10;
    QVector<double> pos_joint11, vel_joint11, acc_joint11;

    for(size_t i=0; i<pos.size();++i){
        MatrixXd pos_stage = pos.at(i);
        MatrixXd vel_stage = vel.at(i);
        MatrixXd acc_stage = acc.at(i);
        vector<double> tsteps_stage = timesteps.at(i);
        double time_init;
        if(time.empty()){
            time_init=0.0;
        }else{
            time_init=time.at(time.size()-1);
        }
        vector<double> time_stage(tsteps_stage.size());
        time_stage.at(0) = time_init;

        for(int k=0;k<pos_stage.rows();++k){
            if(k>0){time_stage.at(k) = time_stage.at(k-1) + tsteps_stage.at(k-1);}
            for(int j=0;j<pos_stage.cols();++j){
                switch(j){
                case 0:// joint 1
                    pos_joint1.push_back(radtodeg*pos_stage(k,j));
                    vel_joint1.push_back(radtodeg*vel_stage(k,j));
                    acc_joint1.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 1://joint 2
                    pos_joint2.push_back(radtodeg*pos_stage(k,j));
                    vel_joint2.push_back(radtodeg*vel_stage(k,j));
                    acc_joint2.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 2://joint 3
                    pos_joint3.push_back(radtodeg*pos_stage(k,j));
                    vel_joint3.push_back(radtodeg*vel_stage(k,j));
                    acc_joint3.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 3://joint 4
                    pos_joint4.push_back(radtodeg*pos_stage(k,j));
                    vel_joint4.push_back(radtodeg*vel_stage(k,j));
                    acc_joint4.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 4://joint 5
                    pos_joint5.push_back(radtodeg*pos_stage(k,j));
                    vel_joint5.push_back(radtodeg*vel_stage(k,j));
                    acc_joint5.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 5://joint 6
                    pos_joint6.push_back(radtodeg*pos_stage(k,j));
                    vel_joint6.push_back(radtodeg*vel_stage(k,j));
                    acc_joint6.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 6://joint 7
                    pos_joint7.push_back(radtodeg*pos_stage(k,j));
                    vel_joint7.push_back(radtodeg*vel_stage(k,j));
                    acc_joint7.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 7://joint 8
                    pos_joint8.push_back(radtodeg*pos_stage(k,j));
                    vel_joint8.push_back(radtodeg*vel_stage(k,j));
                    acc_joint8.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 8://joint 9
                    pos_joint9.push_back(radtodeg*pos_stage(k,j));
                    vel_joint9.push_back(radtodeg*vel_stage(k,j));
                    acc_joint9.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 9://joint 10
                    pos_joint10.push_back(radtodeg*pos_stage(k,j));
                    vel_joint10.push_back(radtodeg*vel_stage(k,j));
                    acc_joint10.push_back(radtodeg*acc_stage(k,j));
                    break;
                case 10://joint 11
                    pos_joint11.push_back(radtodeg*pos_stage(k,j));
                    vel_joint11.push_back(radtodeg*vel_stage(k,j));
                    acc_joint11.push_back(radtodeg*acc_stage(k,j));
                    break;
                }
            }
        }
        time.reserve(time_stage.size());
        std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
    }
    QVector<double> qtime = QVector<double>::fromStdVector(time);
    // joint 1
    //if(ui->plot_joint_1->graphCount()>0){ui->plot_joint_1->clearGraphs();};
    //if(ui->plot_joint_1->plotLayout()->rowCount()>0){ui->plot_joint_1->plotLayout()->clear();}
    ui->plot_joint_1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->plot_joint_1->legend->setVisible(false);
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->plot_joint_1->legend->setFont(legendFont);
    ui->plot_joint_1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    ui->plot_joint_1->addGraph();
    ui->plot_joint_1->xAxis2->setVisible(false);
    ui->plot_joint_1->yAxis2->setVisible(false);
    ui->plot_joint_1->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_joint_1->graph(0)->setData(qtime, pos_joint1);
    ui->plot_joint_1->graph(0)->rescaleAxes();
    ui->plot_joint_1->plotLayout()->insertRow(0);
    ui->plot_joint_1->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_joint_1,"Joint 1"));
    ui->plot_joint_1->xAxis->setLabel("time [s]");
    ui->plot_joint_1->yAxis->setLabel("position [deg]");
    ui->plot_joint_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_joint_1->replot();


}


// Q_SLOTS

void ResultsJointsDialog::on_pushButton_save_clicked()
{
    /*
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Tols",
                                                    "All Files (*.*);;Tol Files (*.tol)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){
        QTextStream stream( &f );
        stream << "### Parameters of the RRT planner in MoveIt! ###" << endl;


    }
    */
}







} // namespace motion_manager
