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

    QVector<double> qtime = QVector<double>::fromStdVector(time);
    QVector<double> qerrors_pos = QVector<double>::fromStdVector(errors_pos);
    QVector<double> qerrors_or = QVector<double>::fromStdVector(errors_or);
    QVector<double> qerrors_pos_or_tot = QVector<double>::fromStdVector(errors_pos_or_tot);
    QVector<double> qerrors_lin_vel = QVector<double>::fromStdVector(errors_lin_vel);
    QVector<double> qerrors_ang_vel = QVector<double>::fromStdVector(errors_ang_vel);
    QVector<double> qerrors_vel_tot = QVector<double>::fromStdVector(errors_vel_tot);
    QVector<double> qerrors_lin_acc = QVector<double>::fromStdVector(errors_lin_acc);
    QVector<double> qerrors_ang_acc = QVector<double>::fromStdVector(errors_ang_acc);
    QVector<double> qerrors_acc_tot = QVector<double>::fromStdVector(errors_acc_tot);

    plotError(ui->plot_error_pos,QString("Error in position"),qtime,qerrors_pos,"[mm]",Qt::blue);
    plotError(ui->plot_error_or,QString("Error in orientation"),qtime,qerrors_or,"q error",Qt::blue);
    plotError(ui->plot_error_pos_or_tot,QString("Error in position and in orientation"),qtime,qerrors_pos_or_tot,"total error",Qt::blue);
    plotError(ui->plot_error_lin_vel,QString("Error in linear velocity"),qtime,qerrors_lin_vel,"[mm/s]",Qt::red);
    plotError(ui->plot_error_ang_vel,QString("Error in angular velocity"),qtime,qerrors_ang_vel,"q propag error",Qt::red);
    plotError(ui->plot_error_vel_tot,QString("Error in velocity"),qtime,qerrors_vel_tot,"total error",Qt::red);
    plotError(ui->plot_error_lin_acc,QString("Error in linear acceleration"),qtime,qerrors_lin_acc,"[mm/s^2]",Qt::darkGreen);
    plotError(ui->plot_error_ang_acc,QString("Error in angular acceleration"),qtime,qerrors_ang_acc,"q acc error",Qt::darkGreen);
    plotError(ui->plot_error_acc_tot,QString("Error in acceleration"),qtime,qerrors_acc_tot,"total error",Qt::darkGreen);

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
}







} // namespace motion_manager
