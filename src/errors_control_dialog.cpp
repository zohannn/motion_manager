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

void ErrorsControlDialog::setupPlots(vector<double> &errors_pos, vector<double> &errors_or, vector<double> &errors_pos_or_tot,
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

void ErrorsControlDialog::on_pushButton_plot_clicked()
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


    plotError(ui->plot_error_pos,QString("Error in position"),qtime,qerrors_pos_plot,"[mm]",Qt::blue);
    plotError(ui->plot_error_or,QString("Error in orientation"),qtime,qerrors_or_plot,"q error",Qt::blue);
    plotError(ui->plot_error_pos_or_tot,QString("Error in position and in orientation"),qtime,qerrors_pos_or_tot_plot,"total error",Qt::blue);
    plotError(ui->plot_error_lin_vel,QString("Error in linear velocity"),qtime,qerrors_lin_vel_plot,"[mm/s]",Qt::red);
    plotError(ui->plot_error_ang_vel,QString("Error in angular velocity"),qtime,qerrors_ang_vel_plot,"q propag error",Qt::red);
    plotError(ui->plot_error_vel_tot,QString("Error in velocity"),qtime,qerrors_vel_tot_plot,"total error",Qt::red);
    plotError(ui->plot_error_lin_acc,QString("Error in linear acceleration"),qtime,qerrors_lin_acc_plot,"[mm/s^2]",Qt::darkGreen);
    plotError(ui->plot_error_ang_acc,QString("Error in angular acceleration"),qtime,qerrors_ang_acc_plot,"q acc error",Qt::darkGreen);
    plotError(ui->plot_error_acc_tot,QString("Error in acceleration"),qtime,qerrors_acc_tot_plot,"total error",Qt::darkGreen);
}

void ErrorsControlDialog::on_pushButton_save_clicked()
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
    path = QString("results/controlling/errors/");

    ui->plot_error_pos->savePdf(path+QString("error_pos.pdf"),true,0,0,QString(),QString("Error in position [mm]"));
    ui->plot_error_or->savePdf(path+QString("error_or.pdf"),true,0,0,QString(),QString("Error in orientation"));
    ui->plot_error_pos_or_tot->savePdf(path+QString("error_pos_or.pdf"),true,0,0,QString(),QString("Error in position and in orientation"));
    ui->plot_error_lin_vel->savePdf(path+QString("error_lin_vel.pdf"),true,0,0,QString(),QString("Error in linear velocity [mm/s]"));
    ui->plot_error_ang_vel->savePdf(path+QString("error_ang_vel.pdf"),true,0,0,QString(),QString("Error in angular velocity"));
    ui->plot_error_vel_tot->savePdf(path+QString("error_vel.pdf"),true,0,0,QString(),QString("Error in velocity"));
    ui->plot_error_lin_acc->savePdf(path+QString("error_lin_acc.pdf"),true,0,0,QString(),QString("Error in linear acceleration [mm/s^2]"));
    ui->plot_error_ang_acc->savePdf(path+QString("error_ang_acc.pdf"),true,0,0,QString(),QString("Error in angular acceleration"));
    ui->plot_error_acc_tot->savePdf(path+QString("error_acc.pdf"),true,0,0,QString(),QString("Error in acceleration"));

    // save text data files

    // error in position
    if(!this->qerrors_pos_plot.empty()){
        string filename("error_pos.txt");
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
        string filename("error_or.txt");
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
        string filename("error_pos_or_tot.txt");
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
        string filename("error_lin_vel.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND LINEAR VELOCITY \n");
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
        string filename("error_ang_vel.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND ANGULAR VELOCITY (QUATERNIONS)\n");
        error << string("# error, time [s] \n");

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
        string filename("error_vel_tot.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND TOTAL VELOCITY (QUATERNIONS)\n");
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
        string filename("error_lin_acc.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND LINEAR ACCELERATION \n");
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
        string filename("error_ang_acc.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND ANGULAR ACCELERATION (QUATERNIONS)\n");
        error << string("# error, time [s] \n");

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
        string filename("error_acc_tot.txt");
        ofstream error;
        error.open(path.toStdString()+filename);

        error << string("# ERROR IN HAND TOTAL ACCELERATION (QUATERNIONS)\n");
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







} // namespace motion_manager
