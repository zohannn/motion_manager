#include "../include/motion_manager/results_control_joints_dialog.hpp"

namespace motion_manager {

ResultsCtrlJointsDialog::ResultsCtrlJointsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ResultsCtrlJointsDialog)
{
    ui->setupUi(this);
}

ResultsCtrlJointsDialog::~ResultsCtrlJointsDialog()
{
    delete ui;
}

/**
void ResultsCtrlJointsDialog::setupPlots(vector<MatrixXd> &pos, vector<MatrixXd> &vel, vector<double> &time)
{
    const double radtodeg = 180.0/static_cast<double>(M_PI);

    QVector<double> pos_joint1, vel_joint1;
    QVector<double> pos_joint2, vel_joint2;
    QVector<double> pos_joint3, vel_joint3;
    QVector<double> pos_joint4, vel_joint4;
    QVector<double> pos_joint5, vel_joint5;
    QVector<double> pos_joint6, vel_joint6;
    QVector<double> pos_joint7, vel_joint7;
    QVector<double> pos_joint8, vel_joint8;
    QVector<double> pos_joint9, vel_joint9;
    QVector<double> pos_joint10, vel_joint10;
    QVector<double> pos_joint11, vel_joint11;

    for(size_t i=0; i<pos.size();++i){
        MatrixXd positions = pos.at(i);
        MatrixXd velocities = vel.at(i);

        for(int k=0;k<positions.rows();++k){

            for(int j=0;j < positions.cols();++j){
                if(j==0){// joint 1
                    pos_joint1.push_back(radtodeg*positions(k,j));
                    vel_joint1.push_back(radtodeg*vel_stage(k,j));                 
                }else if(j==1){//joint 2
                    pos_joint2.push_back(radtodeg*positions(k,j));
                    vel_joint2.push_back(radtodeg*vel_stage(k,j));
                    acc_joint2.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==2){//joint 3
                    pos_joint3.push_back(radtodeg*positions(k,j));
                    vel_joint3.push_back(radtodeg*vel_stage(k,j));
                    acc_joint3.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==3){//joint 4
                    pos_joint4.push_back(radtodeg*positions(k,j));
                    vel_joint4.push_back(radtodeg*vel_stage(k,j));
                    acc_joint4.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==4){//joint 5
                    pos_joint5.push_back(radtodeg*positions(k,j));
                    vel_joint5.push_back(radtodeg*vel_stage(k,j));
                    acc_joint5.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==5){//joint 6
                    pos_joint6.push_back(radtodeg*positions(k,j));
                    vel_joint6.push_back(radtodeg*vel_stage(k,j));
                    acc_joint6.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==6){//joint 7
                    pos_joint7.push_back(radtodeg*positions(k,j));
                    vel_joint7.push_back(radtodeg*vel_stage(k,j));
                    acc_joint7.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==7){//joint 8
                    pos_joint8.push_back(radtodeg*positions(k,j));
                    vel_joint8.push_back(radtodeg*vel_stage(k,j));
                    acc_joint8.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==8){//joint 9
                    pos_joint9.push_back(radtodeg*positions(k,j));
                    vel_joint9.push_back(radtodeg*vel_stage(k,j));
                    acc_joint9.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==9){//joint 10
                    pos_joint10.push_back(radtodeg*positions(k,j));
                    vel_joint10.push_back(radtodeg*vel_stage(k,j));
                    acc_joint10.push_back(radtodeg*acc_stage(k,j));                    
                }else if(j==10){//joint 11
                    pos_joint11.push_back(radtodeg*pos_stage(k,j));
                    vel_joint11.push_back(radtodeg*vel_stage(k,j));
                    acc_joint11.push_back(radtodeg*acc_stage(k,j));                    
                }
            }
        }
        time.reserve(time_stage.size());
        std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
    }
    QVector<double> qtime = QVector<double>::fromStdVector(time);

    if (dual)
    {
        if(right){
            plotJoint(ui->plot_joint_1,QString("Right Joint 1"),qtime,pos_joint1,vel_joint1,acc_joint1); // plot joint 1
            plotJoint(ui->plot_joint_2,QString("Right Joint 2"),qtime,pos_joint2,vel_joint2,acc_joint2); // plot joint 2
            plotJoint(ui->plot_joint_3,QString("Right Joint 3"),qtime,pos_joint3,vel_joint3,acc_joint3); // plot joint 3
            plotJoint(ui->plot_joint_4,QString("Right Joint 4"),qtime,pos_joint4,vel_joint4,acc_joint4); // plot joint 4
            plotJoint(ui->plot_joint_5,QString("Right Joint 5"),qtime,pos_joint5,vel_joint5,acc_joint5); // plot joint 5
            plotJoint(ui->plot_joint_6,QString("Right Joint 6"),qtime,pos_joint6,vel_joint6,acc_joint6); // plot joint 6
            plotJoint(ui->plot_joint_7,QString("Right Joint 7"),qtime,pos_joint7,vel_joint7,acc_joint7); // plot joint 7
            plotJoint(ui->plot_joint_8,QString("Right Joint 8"),qtime,pos_joint8,vel_joint8,acc_joint8); // plot joint 8
            plotJoint(ui->plot_joint_9,QString("Right Joint 9"),qtime,pos_joint9,vel_joint9,acc_joint9); // plot joint 9
            plotJoint(ui->plot_joint_10,QString("Right Joint 10"),qtime,pos_joint10,vel_joint10,acc_joint10); // plot joint 10
            plotJoint(ui->plot_joint_11,QString("Right Joint 11"),qtime,pos_joint11,vel_joint11,acc_joint11); // plot joint 11
        }else{
            plotJoint(ui->plot_joint_1,QString("Left Joint 1"),qtime,pos_joint1,vel_joint1,acc_joint1); // plot joint 1
            plotJoint(ui->plot_joint_2,QString("Left Joint 2"),qtime,pos_joint2,vel_joint2,acc_joint2); // plot joint 2
            plotJoint(ui->plot_joint_3,QString("Left Joint 3"),qtime,pos_joint3,vel_joint3,acc_joint3); // plot joint 3
            plotJoint(ui->plot_joint_4,QString("Left Joint 4"),qtime,pos_joint4,vel_joint4,acc_joint4); // plot joint 4
            plotJoint(ui->plot_joint_5,QString("Left Joint 5"),qtime,pos_joint5,vel_joint5,acc_joint5); // plot joint 5
            plotJoint(ui->plot_joint_6,QString("Left Joint 6"),qtime,pos_joint6,vel_joint6,acc_joint6); // plot joint 6
            plotJoint(ui->plot_joint_7,QString("Left Joint 7"),qtime,pos_joint7,vel_joint7,acc_joint7); // plot joint 7
            plotJoint(ui->plot_joint_8,QString("Left Joint 8"),qtime,pos_joint8,vel_joint8,acc_joint8); // plot joint 8
            plotJoint(ui->plot_joint_9,QString("Left Joint 9"),qtime,pos_joint9,vel_joint9,acc_joint9); // plot joint 9
            plotJoint(ui->plot_joint_10,QString("Left Joint 10"),qtime,pos_joint10,vel_joint10,acc_joint10); // plot joint 10
            plotJoint(ui->plot_joint_11,QString("Left Joint 11"),qtime,pos_joint11,vel_joint11,acc_joint11); // plot joint 11
        }

    }else{
        plotJoint(ui->plot_joint_1,QString("Joint 1"),qtime,pos_joint1,vel_joint1,acc_joint1); // plot joint 1
        plotJoint(ui->plot_joint_2,QString("Joint 2"),qtime,pos_joint2,vel_joint2,acc_joint2); // plot joint 2
        plotJoint(ui->plot_joint_3,QString("Joint 3"),qtime,pos_joint3,vel_joint3,acc_joint3); // plot joint 3
        plotJoint(ui->plot_joint_4,QString("Joint 4"),qtime,pos_joint4,vel_joint4,acc_joint4); // plot joint 4
        plotJoint(ui->plot_joint_5,QString("Joint 5"),qtime,pos_joint5,vel_joint5,acc_joint5); // plot joint 5
        plotJoint(ui->plot_joint_6,QString("Joint 6"),qtime,pos_joint6,vel_joint6,acc_joint6); // plot joint 6
        plotJoint(ui->plot_joint_7,QString("Joint 7"),qtime,pos_joint7,vel_joint7,acc_joint7); // plot joint 7
        plotJoint(ui->plot_joint_8,QString("Joint 8"),qtime,pos_joint8,vel_joint8,acc_joint8); // plot joint 8
        plotJoint(ui->plot_joint_9,QString("Joint 9"),qtime,pos_joint9,vel_joint9,acc_joint9); // plot joint 9
        plotJoint(ui->plot_joint_10,QString("Joint 10"),qtime,pos_joint10,vel_joint10,acc_joint10); // plot joint 10
        plotJoint(ui->plot_joint_11,QString("Joint 11"),qtime,pos_joint11,vel_joint11,acc_joint11); // plot joint 11
    }


}
**/
void ResultsCtrlJointsDialog::plotJoint(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &pos, QVector<double> &vel, QVector<double> &acc)
{
    plot->plotLayout()->clear();
    plot->clearGraphs();
    plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(plot);
    wideAxisRect->setupFullAxesBox(true);
    wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::red));
    wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::darkGreen));
    QCPMarginGroup *marginGroup = new QCPMarginGroup(plot);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : plot->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }

    plot->plotLayout()->addElement(0,0, new QCPPlotTitle(plot,title));
    plot->plotLayout()->addElement(1, 0, wideAxisRect);

    // position
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(0)->setName("[deg]");
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, pos);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(pos.begin(), pos.end()),
                                                      *std::max_element(pos.begin(), pos.end()));
    plot->graph(0)->rescaleAxes();

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName("[deg/s]");
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("time [s]");
    plot->graph(1)->setData(time, vel);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(vel.begin(), vel.end()),
                                                      *std::max_element(vel.begin(), vel.end()));
    plot->graph(1)->rescaleAxes();

    // acceleration
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    plot->graph(2)->setPen(QPen(Qt::darkGreen));
    plot->graph(2)->setName("[deg/s^2]");
    plot->graph(2)->valueAxis()->setTickLabelColor(Qt::darkGreen);
    plot->graph(2)->keyAxis()->setLabel("time [s]");
    plot->graph(2)->setData(time, acc);
    plot->graph(2)->valueAxis()->setRange(*std::min_element(acc.begin(), acc.end()),
                                                      *std::max_element(acc.begin(), acc.end()));
    plot->graph(2)->rescaleAxes();

    // legend
    QCPLegend *legend = new QCPLegend();
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    plot->plotLayout()->addElement(2, 0, subLayout);
    subLayout->setMargins(QMargins(5, 0, 5, 5));
    subLayout->addElement(0, 0, legend);
    // set legend's row stretch factor very small so it ends up with minimum height:
    plot->plotLayout()->setRowStretchFactor(2, 0.001);
    legend->setLayer("legend");
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    legend->setFont(legendFont);
    legend->addElement(0,0,new QCPPlottableLegendItem(legend,plot->graph(0)));
    legend->addElement(0,1,new QCPPlottableLegendItem(legend,plot->graph(1)));
    legend->addElement(0,2,new QCPPlottableLegendItem(legend,plot->graph(2)));

    // interactions
    connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(1)->valueAxis(), SLOT(setRange(QCPRange)));
    connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(2)->valueAxis(), SLOT(setRange(QCPRange)));
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


    plot->replot();
}


// Q_SLOTS


void ResultsCtrlJointsDialog::on_pushButton_save_joints_plots_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/joints", &st) == -1) {
        mkdir("results/planning/joints", 0700);
    }
    path = QString("results/planning/joints/");

    ui->plot_joint_1->savePdf(path+QString("joint1.pdf"),true,0,0,QString(),QString("Kinematics of the joint 1"));
    ui->plot_joint_2->savePdf(path+QString("joint2.pdf"),true,0,0,QString(),QString("Kinematics of the joint 2"));
    ui->plot_joint_3->savePdf(path+QString("joint3.pdf"),true,0,0,QString(),QString("Kinematics of the joint 3"));
    ui->plot_joint_4->savePdf(path+QString("joint4.pdf"),true,0,0,QString(),QString("Kinematics of the joint 4"));
    ui->plot_joint_5->savePdf(path+QString("joint5.pdf"),true,0,0,QString(),QString("Kinematics of the joint 5"));
    ui->plot_joint_6->savePdf(path+QString("joint6.pdf"),true,0,0,QString(),QString("Kinematics of the joint 6"));
    ui->plot_joint_7->savePdf(path+QString("joint7.pdf"),true,0,0,QString(),QString("Kinematics of the joint 7"));
    ui->plot_joint_8->savePdf(path+QString("joint8.pdf"),true,0,0,QString(),QString("Kinematics of the joint 8"));
    ui->plot_joint_9->savePdf(path+QString("joint9.pdf"),true,0,0,QString(),QString("Kinematics of the joint 9"));
    ui->plot_joint_10->savePdf(path+QString("joint10.pdf"),true,0,0,QString(),QString("Kinematics of the joint 10"));
    ui->plot_joint_11->savePdf(path+QString("joint11.pdf"),true,0,0,QString(),QString("Kinematics of the joint 11"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("joint1.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint1.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint2.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint2.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint3.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint3.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint4.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint4.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint5.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint5.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint6.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint6.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint7.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint7.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint8.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint8.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint9.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint9.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint10.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint10.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("joint11.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("joint11.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

}







} // namespace motion_manager
