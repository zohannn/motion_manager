#include "../include/motion_manager/comp_control_dialog.hpp"

namespace motion_manager {

CompControlDialog::CompControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CompControlDialog)
{
    ui->setupUi(this);
}

CompControlDialog::~CompControlDialog()
{
    delete ui;
}

void CompControlDialog::setupPlots(vector<vector<double>> &positions,vector<vector<double>> &orientations,vector<vector<double>> &linear_velocity,vector<vector<double>> &angular_velocity,vector<vector<double>> &linear_acceleration,vector<vector<double>> &angular_acceleration,vector<double> &time, int mod)
{
    //const double radtodeg = 180.0/static_cast<double>(M_PI);

    QVector<double> qtime = QVector<double>::fromStdVector(time);

    QVector<double> linear_pos_x; QVector<double> linear_pos_y; QVector<double> linear_pos_z;
    QVector<double> angular_pos_x; QVector<double> angular_pos_y; QVector<double> angular_pos_z;
    QVector<double> linear_vel_x; QVector<double> linear_vel_y; QVector<double> linear_vel_z;
    QVector<double> angular_vel_x; QVector<double> angular_vel_y; QVector<double> angular_vel_z;
    QVector<double> linear_acc_x; QVector<double> linear_acc_y; QVector<double> linear_acc_z;
    QVector<double> angular_acc_x; QVector<double> angular_acc_y; QVector<double> angular_acc_z;

    for(size_t i=0;i<linear_velocity.size();++i){
        vector<double> linear_pos = positions.at(i);
        vector<double> angular_pos = orientations.at(i);
        vector<double> linear_vel = linear_velocity.at(i);
        vector<double> angular_vel = angular_velocity.at(i);
        vector<double> linear_acc = linear_acceleration.at(i);
        vector<double> angular_acc = angular_acceleration.at(i);

        linear_pos_x.push_back(linear_pos.at(0));
        linear_pos_y.push_back(linear_pos.at(1));
        linear_pos_z.push_back(linear_pos.at(2));
        angular_pos_x.push_back(angular_pos.at(0));
        angular_pos_y.push_back(angular_pos.at(1));
        angular_pos_z.push_back(angular_pos.at(2));

        linear_vel_x.push_back(linear_vel.at(0));
        linear_vel_y.push_back(linear_vel.at(1));
        linear_vel_z.push_back(linear_vel.at(2));
        angular_vel_x.push_back(angular_vel.at(0));
        angular_vel_y.push_back(angular_vel.at(1));
        angular_vel_z.push_back(angular_vel.at(2));

        linear_acc_x.push_back(linear_acc.at(0));
        linear_acc_y.push_back(linear_acc.at(1));
        linear_acc_z.push_back(linear_acc.at(2));
        angular_acc_x.push_back(angular_acc.at(0));
        angular_acc_y.push_back(angular_acc.at(1));
        angular_acc_z.push_back(angular_acc.at(2));
    }

    switch(mod){
    case 0: // shoulder
        plotComp(ui->plot_shoulder_x,QString("Shoulder linear component x"),qtime,linear_pos_x,linear_vel_x,linear_acc_x,true);
        plotComp(ui->plot_shoulder_y,QString("Shoulder linear component y"),qtime,linear_pos_y,linear_vel_y,linear_acc_y,true);
        plotComp(ui->plot_shoulder_z,QString("Shoulder linear component z"),qtime,linear_pos_z,linear_vel_z,linear_acc_z,true);
        plotComp(ui->plot_shoulder_wx,QString("Shoulder angular component x"),qtime,angular_pos_x,angular_vel_x,angular_acc_x,false);
        plotComp(ui->plot_shoulder_wy,QString("Shoulder angular component y"),qtime,angular_pos_y,angular_vel_y,angular_acc_y,false);
        plotComp(ui->plot_shoulder_wz,QString("Shoulder angular component z"),qtime,angular_pos_z,angular_vel_z,angular_acc_z,false);
        break;
    case 1:// elbow
        plotComp(ui->plot_elbow_x,QString("Elbow linear component x"),qtime,linear_pos_x,linear_vel_x,linear_acc_x,true);
        plotComp(ui->plot_elbow_y,QString("Elbow linear component y"),qtime,linear_pos_y,linear_vel_y,linear_acc_y,true);
        plotComp(ui->plot_elbow_z,QString("Elbow linear component z"),qtime,linear_pos_z,linear_vel_z,linear_acc_z,true);
        plotComp(ui->plot_elbow_wx,QString("Elbow angular component x"),qtime,angular_pos_x,angular_vel_x,angular_acc_x,false);
        plotComp(ui->plot_elbow_wy,QString("Elbow angular component y"),qtime,angular_pos_y,angular_vel_y,angular_acc_y,false);
        plotComp(ui->plot_elbow_wz,QString("Elbow angular component z"),qtime,angular_pos_z,angular_vel_z,angular_acc_z,false);
        break;
    case 2: // wrist
        plotComp(ui->plot_wrist_x,QString("Wrist linear component x"),qtime,linear_pos_x,linear_vel_x,linear_acc_x,true);
        plotComp(ui->plot_wrist_y,QString("Wrist linear component y"),qtime,linear_pos_y,linear_vel_y,linear_acc_y,true);
        plotComp(ui->plot_wrist_z,QString("Wrist linear component z"),qtime,linear_pos_z,linear_vel_z,linear_acc_z,true);
        plotComp(ui->plot_wrist_wx,QString("Wrist angular component x"),qtime,angular_pos_x,angular_vel_x,angular_acc_x,false);
        plotComp(ui->plot_wrist_wy,QString("Wrist angular component y"),qtime,angular_pos_y,angular_vel_y,angular_acc_y,false);
        plotComp(ui->plot_wrist_wz,QString("Wrist angular component z"),qtime,angular_pos_z,angular_vel_z,angular_acc_z,false);
        break;
    case 3: default: // hand
        plotComp(ui->plot_hand_x,QString("Hand linear component x"),qtime,linear_pos_x,linear_vel_x,linear_acc_x,true);
        plotComp(ui->plot_hand_y,QString("Hand linear component y"),qtime,linear_pos_y,linear_vel_y,linear_acc_y,true);
        plotComp(ui->plot_hand_z,QString("Hand linear component z"),qtime,linear_pos_z,linear_vel_z,linear_acc_z,true);
        plotComp(ui->plot_hand_wx,QString("Hand angular component x"),qtime,angular_pos_x,angular_vel_x,angular_acc_x,false);
        plotComp(ui->plot_hand_wy,QString("Hand angular component y"),qtime,angular_pos_y,angular_vel_y,angular_acc_y,false);
        plotComp(ui->plot_hand_wz,QString("Hand angular component z"),qtime,angular_pos_z,angular_vel_z,angular_acc_z,false);
        break;        
    }
}


void CompControlDialog::plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var_pos, QVector<double> &var_vel, QVector<double> &var_acc, bool lin)
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

    QString name_pos; QString name_vel; QString name_acc;
    if(lin){
        name_pos="[mm]";
        name_vel="[mm/s]";
        name_acc="[mm/s^2]";
    }else{
        name_pos="[rad]";
        name_vel="[rad/s]";
        name_acc="[rad/s^2]";
    }

    // position
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(0)->setName(name_pos);
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, var_pos);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(var_pos.begin(), var_pos.end()),
                                                      *std::max_element(var_pos.begin(), var_pos.end()));
    plot->graph(0)->rescaleAxes();

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName(name_vel);
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("time [s]");
    plot->graph(1)->setData(time, var_vel);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(var_vel.begin(), var_vel.end()),
                                                      *std::max_element(var_vel.begin(), var_vel.end()));
    plot->graph(1)->rescaleAxes();

    // acceleration
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    plot->graph(2)->setPen(QPen(Qt::darkGreen));
    plot->graph(2)->setName(name_acc);
    plot->graph(2)->valueAxis()->setTickLabelColor(Qt::darkGreen);
    plot->graph(2)->keyAxis()->setLabel("time [s]");
    plot->graph(2)->setData(time, var_acc);
    plot->graph(2)->valueAxis()->setRange(*std::min_element(var_acc.begin(), var_acc.end()),
                                                      *std::max_element(var_acc.begin(), var_acc.end()));
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

void CompControlDialog::on_pushButton_save_shoulder_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/shoulder", &st) == -1) {
        mkdir("results/controlling/shoulder", 0700);
    }
    path = QString("results/controlling/shoulder/");

    ui->plot_shoulder_x->savePdf(path+QString("shoulder_lin_x.pdf"),true,0,0,QString(),QString("Shoulder Linear Component x"));
    ui->plot_shoulder_y->savePdf(path+QString("shoulder_lin_y.pdf"),true,0,0,QString(),QString("Shoulder Linear Component y"));
    ui->plot_shoulder_z->savePdf(path+QString("shoulder_lin_z.pdf"),true,0,0,QString(),QString("Shoulder Linear Component z"));
    ui->plot_shoulder_wx->savePdf(path+QString("shoulder_ang_x.pdf"),true,0,0,QString(),QString("Shoulder Angular Component x"));
    ui->plot_shoulder_wy->savePdf(path+QString("shoulder_ang_y.pdf"),true,0,0,QString(),QString("Shoulder Angular Component y"));
    ui->plot_shoulder_wz->savePdf(path+QString("shoulder_ang_z.pdf"),true,0,0,QString(),QString("Shoulder Angular Component z"));

}

void CompControlDialog::on_pushButton_save_elbow_clicked()
{
    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/elbow", &st) == -1) {
        mkdir("results/controlling/elbow", 0700);
    }
    path = QString("results/controlling/elbow/");

    ui->plot_elbow_x->savePdf(path+QString("elbow_lin_x.pdf"),true,0,0,QString(),QString("Elbow Linear Component x"));
    ui->plot_elbow_y->savePdf(path+QString("elbow_lin_y.pdf"),true,0,0,QString(),QString("Elbow Linear Component y"));
    ui->plot_elbow_z->savePdf(path+QString("elbow_lin_z.pdf"),true,0,0,QString(),QString("Elbow Linear Component z"));
    ui->plot_elbow_wx->savePdf(path+QString("elbow_ang_x.pdf"),true,0,0,QString(),QString("Elbow Angular Component x"));
    ui->plot_elbow_wy->savePdf(path+QString("elbow_ang_y.pdf"),true,0,0,QString(),QString("Elbow Angular Component y"));
    ui->plot_elbow_wz->savePdf(path+QString("elbow_ang_z.pdf"),true,0,0,QString(),QString("Elbow Angular Component z"));
}

void CompControlDialog::on_pushButton_save_wrist_clicked()
{
    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/wrist", &st) == -1) {
        mkdir("results/controlling/wrist", 0700);
    }
    path = QString("results/controlling/wrist/");

    ui->plot_wrist_x->savePdf(path+QString("wrist_lin_x.pdf"),true,0,0,QString(),QString("Wrist Linear Component x"));
    ui->plot_wrist_y->savePdf(path+QString("wrist_lin_y.pdf"),true,0,0,QString(),QString("Wrist Linear Component y"));
    ui->plot_wrist_z->savePdf(path+QString("wrist_lin_z.pdf"),true,0,0,QString(),QString("Wrist Linear Component z"));
    ui->plot_wrist_wx->savePdf(path+QString("wrist_ang_x.pdf"),true,0,0,QString(),QString("Wrist Angular Component x"));
    ui->plot_wrist_wy->savePdf(path+QString("wrist_ang_y.pdf"),true,0,0,QString(),QString("Wrist Angular Component y"));
    ui->plot_wrist_wz->savePdf(path+QString("wrist_ang_z.pdf"),true,0,0,QString(),QString("Wrist Angular Component z"));
}

void CompControlDialog::on_pushButton_save_hand_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/hand", &st) == -1) {
        mkdir("results/controlling/hand", 0700);
    }
    path = QString("results/controlling/hand/");

    ui->plot_hand_x->savePdf(path+QString("hand_lin_x.pdf"),true,0,0,QString(),QString("Hand Linear Component x"));
    ui->plot_hand_y->savePdf(path+QString("hand_lin_y.pdf"),true,0,0,QString(),QString("Hand Linear Component y"));
    ui->plot_hand_z->savePdf(path+QString("hand_lin_z.pdf"),true,0,0,QString(),QString("Hand Linear Component z"));
    ui->plot_hand_wx->savePdf(path+QString("hand_ang_x.pdf"),true,0,0,QString(),QString("Hand Angular Component x"));
    ui->plot_hand_wy->savePdf(path+QString("hand_ang_y.pdf"),true,0,0,QString(),QString("Hand Angular Component y"));
    ui->plot_hand_wz->savePdf(path+QString("hand_ang_z.pdf"),true,0,0,QString(),QString("Hand Angular Component z"));

}







} // namespace motion_manager
