#include "../include/motion_manager/errors_control_dialog.hpp"

namespace motion_manager {

ErrorsControlDialog::ErrorsControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ErrorsControlDialog)
{
    ui->setupUi(this);
}

ErrorsControlDialog::~ErrorsControlDialog()
{
    delete ui;
}

void ErrorsControlDialog::setupHandPlots(vector<double> &errors_pos, vector<double> &errors_or, vector<double> &errors_pos_or_tot,
                                     vector<double> &errors_lin_vel, vector<double> &errors_ang_vel, vector<double> &errors_vel_tot,
                                     vector<double> &errors_lin_acc, vector<double> &errors_ang_acc, vector<double> &errors_acc_tot,
                                     vector<double> &time)
{

    this->qtime = QVector<double>::fromStdVector(time);
    this->qerrors_pos = QVector<double>::fromStdVector(errors_pos);
    this->qerrors_or = QVector<double>::fromStdVector(errors_or);
    this->qerrors_pos_or_tot = QVector<double>::fromStdVector(errors_pos_or_tot);
    this->qerrors_lin_vel = QVector<double>::fromStdVector(errors_lin_vel);
    this->qerrors_ang_vel = QVector<double>::fromStdVector(errors_ang_vel);
    this->qerrors_vel_tot = QVector<double>::fromStdVector(errors_vel_tot);
    this->qerrors_lin_acc = QVector<double>::fromStdVector(errors_lin_acc);
    this->qerrors_ang_acc = QVector<double>::fromStdVector(errors_ang_acc);
    this->qerrors_acc_tot = QVector<double>::fromStdVector(errors_acc_tot);

}

void ErrorsControlDialog::setupFingersPlots(vector<vector<double>> &errors_pos,vector<vector<double>> &errors_vel,vector<vector<double>> &errors_acc, vector<double> &time)
{
    this->qtime = QVector<double>::fromStdVector(time);
    this->qerrors_fing_pos_0.clear(); this->qerrors_fing_vel_0.clear(); this->qerrors_fing_acc_0.clear();
    this->qerrors_fing_pos_1.clear(); this->qerrors_fing_vel_1.clear(); this->qerrors_fing_acc_1.clear();
    this->qerrors_fing_pos_2.clear(); this->qerrors_fing_vel_2.clear(); this->qerrors_fing_acc_2.clear();
    this->qerrors_fing_pos_3.clear(); this->qerrors_fing_vel_3.clear(); this->qerrors_fing_acc_3.clear();

    for(size_t i=0;i<errors_pos.size();++i){
        vector<double> pos = errors_pos.at(i);
        vector<double> vel = errors_vel.at(i);
        vector<double> acc = errors_acc.at(i);
        // finger 0
        this->qerrors_fing_pos_0.push_back(pos.at(0));
        this->qerrors_fing_vel_0.push_back(vel.at(0));
        this->qerrors_fing_acc_0.push_back(acc.at(0));
        // finger 1
        this->qerrors_fing_pos_1.push_back(pos.at(1));
        this->qerrors_fing_vel_1.push_back(vel.at(1));
        this->qerrors_fing_acc_1.push_back(acc.at(1));
        // finger 2
        this->qerrors_fing_pos_2.push_back(pos.at(2));
        this->qerrors_fing_vel_2.push_back(vel.at(2));
        this->qerrors_fing_acc_2.push_back(acc.at(2));
        // finger 3
        this->qerrors_fing_pos_3.push_back(pos.at(3));
        this->qerrors_fing_vel_3.push_back(vel.at(3));
        this->qerrors_fing_acc_3.push_back(acc.at(3));
    }
}

void ErrorsControlDialog::setupAlphaPlots(vector<double> &error_alpha_pos,vector<double> &error_alpha_vel,vector<double> &error_alpha_acc,vector<double> &time)
{

    this->qtime = QVector<double>::fromStdVector(time);
    this->qerror_alpha_pos = QVector<double>::fromStdVector(error_alpha_pos);
    this->qerror_alpha_vel = QVector<double>::fromStdVector(error_alpha_vel);
    this->qerror_alpha_acc = QVector<double>::fromStdVector(error_alpha_acc);

}


void ErrorsControlDialog::plotError(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &error,QString name,Qt::GlobalColor color)
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

//    QString name;
//    if(lin && pos && !tot){
//        name="[mm]";
//    }else if(lin && !pos && !tot){
//        name="[mm/s]";
//    }else if(!lin && pos && !tot){
//        name = "q error";
//    }else if(!lin && !pos && !tot){
//        name = "q prog error";
//    }else{
//        name = "total error";
//    }


    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(color));
    plot->graph(0)->setName(name);
    plot->graph(0)->valueAxis()->setTickLabelColor(color);
    plot->graph(0)->valueAxis()->setLabel(name);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, error);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(error.begin(), error.end()),
                                                      *std::max_element(error.begin(), error.end()));
    plot->graph(0)->rescaleAxes();



    // interactions
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


    plot->replot();
}


// Q_SLOTS

void ErrorsControlDialog::on_pushButton_plot_hand_clicked()
{
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();
    double f_th_vel = this->ui->lineEdit_f_cutoff_vel->text().toDouble();
    double timestep_vel = this->ui->lineEdit_time_step_vel->text().toDouble();
    double f_th_acc = this->ui->lineEdit_f_cutoff_acc->text().toDouble();
    double timestep_acc = this->ui->lineEdit_time_step_acc->text().toDouble();

    LowPassFilter lpf_err_pos(f_th_pos, timestep_pos); LowPassFilter lpf_err_or(f_th_pos, timestep_pos); LowPassFilter lpf_err_pos_or_tot(f_th_pos, timestep_pos);
    LowPassFilter lpf_err_lin_vel(f_th_vel, timestep_vel); LowPassFilter lpf_err_ang_vel(f_th_vel, timestep_vel); LowPassFilter lpf_err_vel_tot(f_th_vel, timestep_vel);
    LowPassFilter lpf_err_lin_acc(f_th_acc, timestep_acc); LowPassFilter lpf_err_ang_acc(f_th_acc, timestep_acc); LowPassFilter lpf_err_acc_tot(f_th_acc, timestep_acc);

    this->qerrors_pos_plot.clear();
    this->qerrors_or_plot.clear();
    this->qerrors_pos_or_tot_plot.clear();
    this->qerrors_lin_vel_plot.clear();
    this->qerrors_ang_vel_plot.clear();
    this->qerrors_vel_tot_plot.clear();
    this->qerrors_lin_acc_plot.clear();
    this->qerrors_ang_acc_plot.clear();
    this->qerrors_acc_tot_plot.clear();

    for(int k=0;k<this->qerrors_pos.size();++k){
        // pos
        this->qerrors_pos_plot.push_back(lpf_err_pos.update(this->qerrors_pos.at(k)));
        this->qerrors_or_plot.push_back(lpf_err_or.update(this->qerrors_or.at(k)));
        this->qerrors_pos_or_tot_plot.push_back(lpf_err_pos_or_tot.update(this->qerrors_pos_or_tot.at(k)));
        //vel
        this->qerrors_lin_vel_plot.push_back(lpf_err_lin_vel.update(this->qerrors_lin_vel.at(k)));
        this->qerrors_ang_vel_plot.push_back(lpf_err_ang_vel.update(this->qerrors_ang_vel.at(k)));
        this->qerrors_vel_tot_plot.push_back(lpf_err_vel_tot.update(this->qerrors_vel_tot.at(k)));
        // acc
        this->qerrors_lin_acc_plot.push_back(lpf_err_lin_acc.update(this->qerrors_lin_acc.at(k)));
        this->qerrors_ang_acc_plot.push_back(lpf_err_ang_acc.update(this->qerrors_ang_acc.at(k)));
        this->qerrors_acc_tot_plot.push_back(lpf_err_acc_tot.update(this->qerrors_acc_tot.at(k)));

    }


    plotError(ui->plot_error_pos_hand,QString("Error in position"),qtime,qerrors_pos_plot,"[mm]",Qt::blue);
    plotError(ui->plot_error_or_hand,QString("Error in orientation"),qtime,qerrors_or_plot,"q error",Qt::blue);
    plotError(ui->plot_error_pos_or_tot_hand,QString("Error in position and in orientation"),qtime,qerrors_pos_or_tot_plot,"total error",Qt::blue);
    plotError(ui->plot_error_lin_vel_hand,QString("Error in linear velocity"),qtime,qerrors_lin_vel_plot,"[mm/s]",Qt::red);
    plotError(ui->plot_error_ang_vel_hand,QString("Error in angular velocity"),qtime,qerrors_ang_vel_plot,"[rad/s]",Qt::red);
    plotError(ui->plot_error_vel_tot_hand,QString("Error in velocity"),qtime,qerrors_vel_tot_plot,"total error",Qt::red);
    plotError(ui->plot_error_lin_acc_hand,QString("Error in linear acceleration"),qtime,qerrors_lin_acc_plot,"[mm/s^2]",Qt::darkGreen);
    plotError(ui->plot_error_ang_acc_hand,QString("Error in angular acceleration"),qtime,qerrors_ang_acc_plot,"[rad/s^2]",Qt::darkGreen);
    plotError(ui->plot_error_acc_tot_hand,QString("Error in acceleration"),qtime,qerrors_acc_tot_plot,"total error",Qt::darkGreen);
}

void ErrorsControlDialog::on_pushButton_plot_fing_clicked()
{
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();
    double f_th_vel = this->ui->lineEdit_f_cutoff_vel->text().toDouble();
    double timestep_vel = this->ui->lineEdit_time_step_vel->text().toDouble();
    double f_th_acc = this->ui->lineEdit_f_cutoff_acc->text().toDouble();
    double timestep_acc = this->ui->lineEdit_time_step_acc->text().toDouble();

    LowPassFilter lpf_err_pos_fing_0(f_th_pos, timestep_pos); LowPassFilter lpf_err_vel_fing_0(f_th_vel, timestep_vel); LowPassFilter lpf_err_acc_fing_0(f_th_acc, timestep_acc);
    LowPassFilter lpf_err_pos_fing_1(f_th_pos, timestep_pos); LowPassFilter lpf_err_vel_fing_1(f_th_vel, timestep_vel); LowPassFilter lpf_err_acc_fing_1(f_th_acc, timestep_acc);
    LowPassFilter lpf_err_pos_fing_2(f_th_pos, timestep_pos); LowPassFilter lpf_err_vel_fing_2(f_th_vel, timestep_vel); LowPassFilter lpf_err_acc_fing_2(f_th_acc, timestep_acc);
    LowPassFilter lpf_err_pos_fing_3(f_th_pos, timestep_pos); LowPassFilter lpf_err_vel_fing_3(f_th_vel, timestep_vel); LowPassFilter lpf_err_acc_fing_3(f_th_acc, timestep_acc);

    qerrors_fing_pos_0_plot.clear(); qerrors_fing_vel_0_plot.clear(); qerrors_fing_acc_0_plot.clear();
    qerrors_fing_pos_1_plot.clear(); qerrors_fing_vel_1_plot.clear(); qerrors_fing_acc_1_plot.clear();
    qerrors_fing_pos_2_plot.clear(); qerrors_fing_vel_2_plot.clear(); qerrors_fing_acc_2_plot.clear();
    qerrors_fing_pos_3_plot.clear(); qerrors_fing_vel_3_plot.clear(); qerrors_fing_acc_3_plot.clear();

    for(int k=0;k<this->qerrors_fing_pos_0.size();++k){
        // finger 0
        this->qerrors_fing_pos_0_plot.push_back(lpf_err_pos_fing_0.update(this->qerrors_fing_pos_0.at(k)));
        this->qerrors_fing_vel_0_plot.push_back(lpf_err_vel_fing_0.update(this->qerrors_fing_vel_0.at(k)));
        this->qerrors_fing_acc_0_plot.push_back(lpf_err_acc_fing_0.update(this->qerrors_fing_acc_0.at(k)));
        // finger 1
        this->qerrors_fing_pos_1_plot.push_back(lpf_err_pos_fing_1.update(this->qerrors_fing_pos_1.at(k)));
        this->qerrors_fing_vel_1_plot.push_back(lpf_err_vel_fing_1.update(this->qerrors_fing_vel_1.at(k)));
        this->qerrors_fing_acc_1_plot.push_back(lpf_err_acc_fing_1.update(this->qerrors_fing_acc_1.at(k)));
        // finger 2
        this->qerrors_fing_pos_2_plot.push_back(lpf_err_pos_fing_2.update(this->qerrors_fing_pos_2.at(k)));
        this->qerrors_fing_vel_2_plot.push_back(lpf_err_vel_fing_2.update(this->qerrors_fing_vel_2.at(k)));
        this->qerrors_fing_acc_2_plot.push_back(lpf_err_acc_fing_2.update(this->qerrors_fing_acc_2.at(k)));
        // finger 3
        this->qerrors_fing_pos_3_plot.push_back(lpf_err_pos_fing_3.update(this->qerrors_fing_pos_3.at(k)));
        this->qerrors_fing_vel_3_plot.push_back(lpf_err_vel_fing_3.update(this->qerrors_fing_vel_3.at(k)));
        this->qerrors_fing_acc_3_plot.push_back(lpf_err_acc_fing_3.update(this->qerrors_fing_acc_3.at(k)));
    }

    // finger 0
    plotError(ui->plot_error_pos_fing_0,QString("Error in position Finger 0"),qtime,qerrors_fing_pos_0_plot,"[deg]",Qt::blue);
    plotError(ui->plot_error_vel_fing_0,QString("Error in velocity Finger 0"),qtime,qerrors_fing_vel_0_plot,"[deg/s]",Qt::red);
    plotError(ui->plot_error_acc_fing_0,QString("Error in acceleration Finger 0"),qtime,qerrors_fing_acc_0_plot,"[deg/s^2]",Qt::darkGreen);
    // finger 1
    plotError(ui->plot_error_pos_fing_1,QString("Error in position Finger 1"),qtime,qerrors_fing_pos_1_plot,"[deg]",Qt::blue);
    plotError(ui->plot_error_vel_fing_1,QString("Error in velocity Finger 1"),qtime,qerrors_fing_vel_1_plot,"[deg/s]",Qt::red);
    plotError(ui->plot_error_acc_fing_1,QString("Error in acceleration Finger 1"),qtime,qerrors_fing_acc_1_plot,"[deg/s^2]",Qt::darkGreen);
    // finger 2
    plotError(ui->plot_error_pos_fing_2,QString("Error in position Finger 2"),qtime,qerrors_fing_pos_2_plot,"[deg]",Qt::blue);
    plotError(ui->plot_error_vel_fing_2,QString("Error in velocity Finger 2"),qtime,qerrors_fing_vel_2_plot,"[deg/s]",Qt::red);
    plotError(ui->plot_error_acc_fing_2,QString("Error in acceleration Finger 2"),qtime,qerrors_fing_acc_2_plot,"[deg/s^2]",Qt::darkGreen);
    // finger 3
    plotError(ui->plot_error_pos_fing_3,QString("Error in position Finger 3"),qtime,qerrors_fing_pos_3_plot,"[deg]",Qt::blue);
    plotError(ui->plot_error_vel_fing_3,QString("Error in velocity Finger 3"),qtime,qerrors_fing_vel_3_plot,"[deg/s]",Qt::red);
    plotError(ui->plot_error_acc_fing_3,QString("Error in acceleration Finger 3"),qtime,qerrors_fing_acc_3_plot,"[deg/s^2]",Qt::darkGreen);


}

void ErrorsControlDialog::on_pushButton_plot_alpha_clicked()
{
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();
    double f_th_vel = this->ui->lineEdit_f_cutoff_vel->text().toDouble();
    double timestep_vel = this->ui->lineEdit_time_step_vel->text().toDouble();
    double f_th_acc = this->ui->lineEdit_f_cutoff_acc->text().toDouble();
    double timestep_acc = this->ui->lineEdit_time_step_acc->text().toDouble();

    LowPassFilter lpf_err_pos(f_th_pos, timestep_pos); LowPassFilter lpf_err_vel(f_th_vel, timestep_vel); LowPassFilter lpf_err_acc(f_th_acc, timestep_acc);

    this->qerror_alpha_pos_plot.clear();
    this->qerror_alpha_vel_plot.clear();
    this->qerror_alpha_acc_plot.clear();

    for(int k=0;k<this->qerror_alpha_pos.size();++k){
        // pos
        this->qerror_alpha_pos_plot.push_back(lpf_err_pos.update(this->qerror_alpha_pos.at(k)));
        // vel
        this->qerror_alpha_vel_plot.push_back(lpf_err_vel.update(this->qerror_alpha_vel.at(k)));
        // acc
        this->qerror_alpha_acc_plot.push_back(lpf_err_acc.update(this->qerror_alpha_acc.at(k)));
    }

    plotError(ui->plot_error_alpha_pos,QString("Error in position"),qtime,qerror_alpha_pos_plot,"[rad]",Qt::blue);
    plotError(ui->plot_error_alpha_vel,QString("Error in velocity"),qtime,qerror_alpha_vel_plot,"[rad/s]",Qt::red);
    plotError(ui->plot_error_alpha_acc,QString("Error in acceleration"),qtime,qerror_alpha_acc_plot,"[rad/s^2]",Qt::darkGreen);
}

void ErrorsControlDialog::on_pushButton_save_hand_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/errors", &st) == -1) {
        mkdir("results/controlling/errors", 0700);
    }
    if (stat("results/controlling/errors/hand", &st) == -1) {
        mkdir("results/controlling/errors/hand", 0700);
    }
    path = QString("results/controlling/errors/hand/");

    ui->plot_error_pos_hand->savePdf(path+QString("error_pos_hand.pdf"),true,0,0,QString(),QString("Error in position [mm]"));
    ui->plot_error_or_hand->savePdf(path+QString("error_or_hand.pdf"),true,0,0,QString(),QString("Error in orientation"));
    ui->plot_error_pos_or_tot_hand->savePdf(path+QString("error_pos_or_hand.pdf"),true,0,0,QString(),QString("Error in position and in orientation"));
    ui->plot_error_lin_vel_hand->savePdf(path+QString("error_lin_vel_hand.pdf"),true,0,0,QString(),QString("Error in linear velocity [mm/s]"));
    ui->plot_error_ang_vel_hand->savePdf(path+QString("error_ang_vel_hand.pdf"),true,0,0,QString(),QString("Error in angular velocity [rad/s]"));
    ui->plot_error_vel_tot_hand->savePdf(path+QString("error_vel_hand.pdf"),true,0,0,QString(),QString("Error in velocity"));
    ui->plot_error_lin_acc_hand->savePdf(path+QString("error_lin_acc_hand.pdf"),true,0,0,QString(),QString("Error in linear acceleration [mm/s^2]"));
    ui->plot_error_ang_acc_hand->savePdf(path+QString("error_ang_acc_hand.pdf"),true,0,0,QString(),QString("Error in angular acceleration [rad/s^2]"));
    ui->plot_error_acc_tot_hand->savePdf(path+QString("error_acc_hand.pdf"),true,0,0,QString(),QString("Error in acceleration"));

    // save text data files

    // error in position
    if(!this->qerrors_pos_plot.empty()){
        string filename("error_pos_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND POSITION \n");
        error << string("# error [mm], time [s] \n");

        for(size_t i=0;i<this->qerrors_pos_plot.size();++i){
            double err = this->qerrors_pos_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in orientation
    if(!this->qerrors_or_plot.empty()){
        string filename("error_or_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND ORIENTATION (QUATERNIONS)\n");
        error << string("# error, time [s] \n");

        for(size_t i=0;i<this->qerrors_or_plot.size();++i){
            double err = this->qerrors_or_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in position and orientation
    if(!this->qerrors_pos_or_tot_plot.empty()){
        string filename("error_pos_or_tot_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND POSITION AND ORIENTATION (QUATERNIONS)\n");
        error << string("# error, time [s] \n");

        for(size_t i=0;i<this->qerrors_pos_or_tot_plot.size();++i){
            double err = this->qerrors_pos_or_tot_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in linear velocity
    if(!this->qerrors_lin_vel_plot.empty()){
        string filename("error_lin_vel_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND LINEAR VELOCITY\n");
        error << string("# error [mm/s], time [s] \n");

        for(size_t i=0;i<this->qerrors_lin_vel_plot.size();++i){
            double err = this->qerrors_lin_vel_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in angular velocity
    if(!this->qerrors_ang_vel_plot.empty()){
        string filename("error_ang_vel_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND ANGULAR VELOCITY\n");
        error << string("# error [rad/s], time [s] \n");

        for(size_t i=0;i<this->qerrors_ang_vel_plot.size();++i){
            double err = this->qerrors_ang_vel_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in velocity
    if(!this->qerrors_vel_tot_plot.empty()){
        string filename("error_vel_tot_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND TOTAL VELOCITY\n");
        error << string("# error, time [s] \n");

        for(size_t i=0;i<this->qerrors_vel_tot_plot.size();++i){
            double err = this->qerrors_vel_tot_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in linear acceleration
    if(!this->qerrors_lin_acc_plot.empty()){
        string filename("error_lin_acc_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND LINEAR ACCELERATION\n");
        error << string("# error [mm/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_lin_acc_plot.size();++i){
            double err = this->qerrors_lin_acc_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in angular acceleration
    if(!this->qerrors_ang_acc_plot.empty()){
        string filename("error_ang_acc_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND ANGULAR ACCELERATION\n");
        error << string("# error [rad/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_ang_acc_plot.size();++i){
            double err = this->qerrors_ang_acc_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in total acceleration
    if(!this->qerrors_acc_tot_plot.empty()){
        string filename("error_acc_tot_hand.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND TOTAL ACCELERATION\n");
        error << string("# error, time [s] \n");

        for(size_t i=0;i<this->qerrors_acc_tot_plot.size();++i){
            double err = this->qerrors_acc_tot_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

}

void ErrorsControlDialog::on_pushButton_save_fing_clicked()
{
    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/errors", &st) == -1) {
        mkdir("results/controlling/errors", 0700);
    }
    if (stat("results/controlling/errors/fingers", &st) == -1) {
        mkdir("results/controlling/errors/fingers", 0700);
    }
    path = QString("results/controlling/errors/fingers/");

    // finger 0
    ui->plot_error_pos_fing_0->savePdf(path+QString("error_pos_fing_0.pdf"),true,0,0,QString(),QString("Error in position Finger 0 [deg]"));
    ui->plot_error_vel_fing_0->savePdf(path+QString("error_vel_fing_0.pdf"),true,0,0,QString(),QString("Error in velocity Finger 0 [deg/s]"));
    ui->plot_error_acc_fing_0->savePdf(path+QString("error_acc_fing_0.pdf"),true,0,0,QString(),QString("Error in acceleration Finger 0 [deg/s^2]"));
    // finger 1
    ui->plot_error_pos_fing_1->savePdf(path+QString("error_pos_fing_1.pdf"),true,0,0,QString(),QString("Error in position Finger 1 [deg]"));
    ui->plot_error_vel_fing_1->savePdf(path+QString("error_vel_fing_1.pdf"),true,0,0,QString(),QString("Error in velocity Finger 1 [deg/s]"));
    ui->plot_error_acc_fing_1->savePdf(path+QString("error_acc_fing_1.pdf"),true,0,0,QString(),QString("Error in acceleration Finger 1 [deg/s^2]"));
    // finger 2
    ui->plot_error_pos_fing_2->savePdf(path+QString("error_pos_fing_2.pdf"),true,0,0,QString(),QString("Error in position Finger 2 [deg]"));
    ui->plot_error_vel_fing_2->savePdf(path+QString("error_vel_fing_2.pdf"),true,0,0,QString(),QString("Error in velocity Finger 2 [deg/s]"));
    ui->plot_error_acc_fing_2->savePdf(path+QString("error_acc_fing_2.pdf"),true,0,0,QString(),QString("Error in acceleration Finger 2 [deg/s^2]"));
    // finger 3
    ui->plot_error_pos_fing_3->savePdf(path+QString("error_pos_fing_3.pdf"),true,0,0,QString(),QString("Error in position Finger 3 [deg]"));
    ui->plot_error_vel_fing_3->savePdf(path+QString("error_vel_fing_3.pdf"),true,0,0,QString(),QString("Error in velocity Finger 3 [deg/s]"));
    ui->plot_error_acc_fing_3->savePdf(path+QString("error_acc_fing_3.pdf"),true,0,0,QString(),QString("Error in acceleration Finger 3 [deg/s^2]"));

    // save text data files

    // finger 0
    if(!this->qerrors_fing_pos_0_plot.empty()){
        string filename("error_fing_0.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERRORS IN FINGER 0 \n");
        error << string("# error pos [deg], error vel [deg/s], error acc [deg/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_fing_pos_0_plot.size();++i){
            double err_pos = this->qerrors_fing_pos_0_plot.at(i);
            double err_vel = this->qerrors_fing_vel_0_plot.at(i);
            double err_acc = this->qerrors_fing_acc_0_plot.at(i);
            double time = this->qtime.at(i);
            string err_pos_str =  boost::str(boost::format("%.2f") % (err_pos)); boost::replace_all(err_pos_str,",",".");
            string err_vel_str =  boost::str(boost::format("%.2f") % (err_vel)); boost::replace_all(err_vel_str,",",".");
            string err_acc_str =  boost::str(boost::format("%.2f") % (err_acc)); boost::replace_all(err_acc_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");
            error << err_pos_str+string(", ")+err_vel_str+string(", ")+err_acc_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // finger 1
    if(!this->qerrors_fing_pos_1_plot.empty()){
        string filename("error_fing_1.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERRORS IN FINGER 1 \n");
        error << string("# error pos [deg], error vel [deg/s], error acc [deg/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_fing_pos_1_plot.size();++i){
            double err_pos = this->qerrors_fing_pos_1_plot.at(i);
            double err_vel = this->qerrors_fing_vel_1_plot.at(i);
            double err_acc = this->qerrors_fing_acc_1_plot.at(i);
            double time = this->qtime.at(i);
            string err_pos_str =  boost::str(boost::format("%.2f") % (err_pos)); boost::replace_all(err_pos_str,",",".");
            string err_vel_str =  boost::str(boost::format("%.2f") % (err_vel)); boost::replace_all(err_vel_str,",",".");
            string err_acc_str =  boost::str(boost::format("%.2f") % (err_acc)); boost::replace_all(err_acc_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");
            error << err_pos_str+string(", ")+err_vel_str+string(", ")+err_acc_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // finger 2
    if(!this->qerrors_fing_pos_2_plot.empty()){
        string filename("error_fing_2.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERRORS IN FINGER 2 \n");
        error << string("# error pos [deg], error vel [deg/s], error acc [deg/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_fing_pos_2_plot.size();++i){
            double err_pos = this->qerrors_fing_pos_2_plot.at(i);
            double err_vel = this->qerrors_fing_vel_2_plot.at(i);
            double err_acc = this->qerrors_fing_acc_2_plot.at(i);
            double time = this->qtime.at(i);
            string err_pos_str =  boost::str(boost::format("%.2f") % (err_pos)); boost::replace_all(err_pos_str,",",".");
            string err_vel_str =  boost::str(boost::format("%.2f") % (err_vel)); boost::replace_all(err_vel_str,",",".");
            string err_acc_str =  boost::str(boost::format("%.2f") % (err_acc)); boost::replace_all(err_acc_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");
            error << err_pos_str+string(", ")+err_vel_str+string(", ")+err_acc_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // finger 3
    if(!this->qerrors_fing_pos_3_plot.empty()){
        string filename("error_fing_3.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERRORS IN FINGER 3 \n");
        error << string("# error pos [deg], error vel [deg/s], error acc [deg/s^2], time [s] \n");

        for(size_t i=0;i<this->qerrors_fing_pos_3_plot.size();++i){
            double err_pos = this->qerrors_fing_pos_3_plot.at(i);
            double err_vel = this->qerrors_fing_vel_3_plot.at(i);
            double err_acc = this->qerrors_fing_acc_3_plot.at(i);
            double time = this->qtime.at(i);
            string err_pos_str =  boost::str(boost::format("%.2f") % (err_pos)); boost::replace_all(err_pos_str,",",".");
            string err_vel_str =  boost::str(boost::format("%.2f") % (err_vel)); boost::replace_all(err_vel_str,",",".");
            string err_acc_str =  boost::str(boost::format("%.2f") % (err_acc)); boost::replace_all(err_acc_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");
            error << err_pos_str+string(", ")+err_vel_str+string(", ")+err_acc_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }
}

void ErrorsControlDialog::on_pushButton_save_alpha_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/errors", &st) == -1) {
        mkdir("results/controlling/errors", 0700);
    }
    if (stat("results/controlling/errors/alpha", &st) == -1) {
        mkdir("results/controlling/errors/alpha", 0700);
    }
    path = QString("results/controlling/errors/alpha/");

    ui->plot_error_alpha_pos->savePdf(path+QString("error_pos_alpha.pdf"),true,0,0,QString(),QString("Error in position [rad]"));
    ui->plot_error_alpha_vel->savePdf(path+QString("error_vel_alpha.pdf"),true,0,0,QString(),QString("Error in velocity [rad/s]"));
    ui->plot_error_alpha_acc->savePdf(path+QString("error_acc_alpha.pdf"),true,0,0,QString(),QString("Error in acceleration [rad/s^2]"));

    // save text data files

    // error in position
    if(!this->qerror_alpha_pos_plot.empty()){
        string filename("error_pos_alpha.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ALPHA ERROR IN POSITION \n");
        error << string("# error [rad], time [s] \n");

        for(size_t i=0;i<this->qerror_alpha_pos_plot.size();++i){
            double err = this->qerror_alpha_pos_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in velocity
    if(!this->qerror_alpha_vel_plot.empty()){
        string filename("error_vel_alpha.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ALPHA ERROR IN VELOCITY \n");
        error << string("# error [rad/s], time [s] \n");

        for(size_t i=0;i<this->qerror_alpha_vel_plot.size();++i){
            double err = this->qerror_alpha_vel_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }

    // error in acceleration
    if(!this->qerror_alpha_acc_plot.empty()){
        string filename("error_acc_alpha.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ALPHA ERROR IN ACCELERATION \n");
        error << string("# error [rad/s^2], time [s] \n");

        for(size_t i=0;i<this->qerror_alpha_acc_plot.size();++i){
            double err = this->qerror_alpha_acc_plot.at(i);
            double time = this->qtime.at(i);
            string err_str =  boost::str(boost::format("%.2f") % (err));
            boost::replace_all(err_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            error << err_str+string(", ")+t_str+string("\n");
        }
        error.close();
    }


}







} // namespace motion_manager
