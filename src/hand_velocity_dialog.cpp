#include "../include/motion_manager/hand_velocity_dialog.hpp"

namespace motion_manager {

HandVelocityDialog::HandVelocityDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HandVelocityDialog)
{
    ui->setupUi(this);
}

HandVelocityDialog::~HandVelocityDialog()
{
    delete ui;
}


void HandVelocityDialog::setupPlots(vector<vector<double>> &hand_linear_velocity,vector<vector<double>> &hand_angular_velocity,QVector<double> &time)
{
    //const double radtodeg = 180.0/static_cast<double>(M_PI);

    QVector<double> hand_linear_vel_x; QVector<double> hand_linear_vel_y; QVector<double> hand_linear_vel_z;
    QVector<double> hand_angular_vel_x; QVector<double> hand_angular_vel_y; QVector<double> hand_angular_vel_z;


    for(size_t i=0;i<hand_linear_velocity.size();++i){
        vector<double> linear = hand_linear_velocity.at(i);
        vector<double> angular = hand_angular_velocity.at(i);
        hand_linear_vel_x.push_back(linear.at(0));
        hand_linear_vel_y.push_back(linear.at(1));
        hand_linear_vel_z.push_back(linear.at(2));
        hand_angular_vel_x.push_back(angular.at(0));
        hand_angular_vel_y.push_back(angular.at(1));
        hand_angular_vel_z.push_back(angular.at(2));
    }


    plotJoint(ui->plot_hand_x,QString("Hand linear velocity x"),time,hand_linear_vel_x,true); // plot hand vel x
    plotJoint(ui->plot_hand_y,QString("Hand linear velocity y"),time,hand_linear_vel_y,true); // plot hand vel y
    plotJoint(ui->plot_hand_z,QString("Hand linear velocity z"),time,hand_linear_vel_z,true); // plot hand vel z
    plotJoint(ui->plot_hand_wx,QString("Hand angular velocity x"),time,hand_angular_vel_x,false); // plot hand vel wx
    plotJoint(ui->plot_hand_wy,QString("Hand angular velocity y"),time,hand_angular_vel_y,false); // plot hand vel wy
    plotJoint(ui->plot_hand_wz,QString("Hand angular velocity z"),time,hand_angular_vel_z,false); // plot hand vel wz



}


void HandVelocityDialog::plotJoint(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var,bool lin)
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

    QString name;
    if(lin){
        name="[mm/s]";
    }else{
        name="[rad/s]";
    }
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(0)->setName(name);
    plot->graph(0)->valueAxis()->setLabel(name);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, var);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(var.begin(), var.end()),
                                          *std::max_element(var.begin(), var.end()));
    plot->graph(0)->rescaleAxes();

    // interactions
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    plot->replot();
}


// Q_SLOTS

void HandVelocityDialog::on_pushButton_save_clicked()
{

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/hand", &st) == -1) {
        mkdir("results/planning/hand", 0700);
    }
    QString path("results/planning/hand/");

    ui->plot_hand_x->savePdf(path+QString("hand_vel_x.pdf"),true,0,0,QString(),QString("Hand Linear Velocity x"));
    ui->plot_hand_y->savePdf(path+QString("hand_vel_y.pdf"),true,0,0,QString(),QString("Hand Linear Velocity y"));
    ui->plot_hand_z->savePdf(path+QString("hand_vel_z.pdf"),true,0,0,QString(),QString("Hand Linear Velocity z"));
    ui->plot_hand_wx->savePdf(path+QString("hand_vel_wx.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));
    ui->plot_hand_wy->savePdf(path+QString("hand_vel_wy.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));
    ui->plot_hand_wz->savePdf(path+QString("hand_vel_wz.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));

}







} // namespace motion_manager
