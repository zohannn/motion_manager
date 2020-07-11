#include "../include/motion_manager/results_control_null_joints_dialog.hpp"

namespace motion_manager {

ResultsCtrlNullJointsDialog::ResultsCtrlNullJointsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ResultsCtrlNullJointsDialog)
{
    ui->setupUi(this);

}

ResultsCtrlNullJointsDialog::~ResultsCtrlNullJointsDialog()
{
    delete ui;
}


void ResultsCtrlNullJointsDialog::setupPlots(MatrixXd &velocities, vector<double> &time)
{
    this->velocities = velocities.replicate(1,1);
    this->time = time;
}

void ResultsCtrlNullJointsDialog::plotJoint(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &vel)
{
    plot->plotLayout()->clear();
    plot->clearGraphs();
    plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(plot);
    wideAxisRect->setupFullAxesBox(true);
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

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::red));
    plot->graph(0)->valueAxis()->setLabel("[deg/s]");
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, vel);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(vel.begin(), vel.end()),*std::max_element(vel.begin(), vel.end()));
    plot->graph(0)->rescaleAxes();
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    plot->replot();
}


// Q_SLOTS

void ResultsCtrlNullJointsDialog::on_pushButton_plot_clicked()
{
    const double radtodeg = 180.0/static_cast<double>(M_PI);

    double f_th_vel = this->ui->lineEdit_f_cutoff_vel->text().toDouble();
    double timestep_vel = this->ui->lineEdit_time_step_vel->text().toDouble();

    LowPassFilter lpf_vel_1(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_2(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_3(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_4(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_5(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_6(f_th_vel, timestep_vel);
    LowPassFilter lpf_vel_7(f_th_vel, timestep_vel);

    this->vel_joint1.clear();
    this->vel_joint2.clear();
    this->vel_joint3.clear();
    this->vel_joint4.clear();
    this->vel_joint5.clear();
    this->vel_joint6.clear();
    this->vel_joint7.clear();

    for(int k=0;k<velocities.rows();++k){
        for(int j=0;j < velocities.cols();++j){
            if(j==0){// joint 1
                vel_joint1.push_back(lpf_vel_1.update(radtodeg*velocities(k,j)));
            }else if(j==1){//joint 2
                vel_joint2.push_back(lpf_vel_2.update(radtodeg*velocities(k,j)));
            }else if(j==2){//joint 3
                vel_joint3.push_back(lpf_vel_3.update(radtodeg*velocities(k,j)));
            }else if(j==3){//joint 4
                vel_joint4.push_back(lpf_vel_4.update(radtodeg*velocities(k,j)));
            }else if(j==4){//joint 5
                vel_joint5.push_back(lpf_vel_5.update(radtodeg*velocities(k,j)));
            }else if(j==5){//joint 6
                vel_joint6.push_back(lpf_vel_6.update(radtodeg*velocities(k,j)));
            }else if(j==6){//joint 7
                vel_joint7.push_back(lpf_vel_7.update(radtodeg*velocities(k,j)));            
            }
        }
    }


    this->qtime = QVector<double>::fromStdVector(time);

    plotJoint(ui->plot_joint_1,QString("Joint 1"),qtime,vel_joint1); // plot joint 1
    plotJoint(ui->plot_joint_2,QString("Joint 2"),qtime,vel_joint2); // plot joint 2
    plotJoint(ui->plot_joint_3,QString("Joint 3"),qtime,vel_joint3); // plot joint 3
    plotJoint(ui->plot_joint_4,QString("Joint 4"),qtime,vel_joint4); // plot joint 4
    plotJoint(ui->plot_joint_5,QString("Joint 5"),qtime,vel_joint5); // plot joint 5
    plotJoint(ui->plot_joint_6,QString("Joint 6"),qtime,vel_joint6); // plot joint 6
    plotJoint(ui->plot_joint_7,QString("Joint 7"),qtime,vel_joint7); // plot joint 7

}

void ResultsCtrlNullJointsDialog::on_pushButton_save_joints_plots_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/joints", &st) == -1) {
        mkdir("results/controlling/joints", 0700);
    }
    if (stat("results/controlling/joints/null_space", &st) == -1) {
        mkdir("results/controlling/joints/null_space", 0700);
    }
    path = QString("results/controlling/joints/null_space/");

    ui->plot_joint_1->savePdf(path+QString("joint1.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 1"));
    ui->plot_joint_2->savePdf(path+QString("joint2.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 2"));
    ui->plot_joint_3->savePdf(path+QString("joint3.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 3"));
    ui->plot_joint_4->savePdf(path+QString("joint4.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 4"));
    ui->plot_joint_5->savePdf(path+QString("joint5.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 5"));
    ui->plot_joint_6->savePdf(path+QString("joint6.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 6"));
    ui->plot_joint_7->savePdf(path+QString("joint7.pdf"),true,0,0,QString(),QString("Null space velocity of the joint 7"));

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

    // null velocity joint 1
    if(!this->vel_joint1.empty()){
        string filename("null_vel_joint1.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 1 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint1.size();++i){
            double vel = this->vel_joint1.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 2
    if(!this->vel_joint2.empty()){
        string filename("null_vel_joint2.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 2 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint2.size();++i){
            double vel = this->vel_joint2.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 3
    if(!this->vel_joint3.empty()){
        string filename("null_vel_joint3.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 3 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint3.size();++i){
            double vel = this->vel_joint3.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 4
    if(!this->vel_joint4.empty()){
        string filename("null_vel_joint4.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 4 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint4.size();++i){
            double vel = this->vel_joint4.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 5
    if(!this->vel_joint5.empty()){
        string filename("null_vel_joint5.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 5 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint5.size();++i){
            double vel = this->vel_joint5.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 6
    if(!this->vel_joint6.empty()){
        string filename("null_vel_joint6.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 6 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint6.size();++i){
            double vel = this->vel_joint6.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }

    // null velocity joint 7
    if(!this->vel_joint7.empty()){
        string filename("null_vel_joint7.txt");
        ofstream vel_stream;
        vel_stream.open(path.toStdString()+filename);

        vel_stream << string("# NULL SPACE VELOCITY JOINT 7 \n");
        vel_stream << string("# velocity [deg/s], time [s] \n");

        for(size_t i=0;i<this->vel_joint7.size();++i){
            double vel = this->vel_joint7.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            vel_stream << vel_str+string(", ")+t_str+string("\n");
        }
        vel_stream.close();
    }


}


} // namespace motion_manager
