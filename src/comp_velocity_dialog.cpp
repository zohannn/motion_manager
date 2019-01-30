#include "../include/motion_manager/comp_velocity_dialog.hpp"

namespace motion_manager {

CompVelocityDialog::CompVelocityDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CompVelocityDialog)
{
    ui->setupUi(this);
    dual = false;
    right = true;
}

CompVelocityDialog::~CompVelocityDialog()
{
    delete ui;
}

void CompVelocityDialog::setDual(bool d)
{
    this->dual = d;
}

void CompVelocityDialog::setRight(bool r)
{
    this->right = r;
}

void CompVelocityDialog::setupPlots(vector<vector<double>> &position,vector<vector<double>> &orientation,
                                    vector<vector<double>> &linear_velocity,vector<vector<double>> &angular_velocity,
                                    vector<vector<double>> &linear_acceleration,vector<vector<double>> &angular_acceleration,
                                    QVector<double> &time, int mod)
{
    //const double radtodeg = 180.0/static_cast<double>(M_PI);

    QVector<double> linear_pos_x; QVector<double> linear_pos_y; QVector<double> linear_pos_z;
    QVector<double> angular_pos_x; QVector<double> angular_pos_y; QVector<double> angular_pos_z;
    QVector<double> linear_pos_norm; QVector<double> angular_pos_norm;

    QVector<double> linear_vel_x; QVector<double> linear_vel_y; QVector<double> linear_vel_z;
    QVector<double> angular_vel_x; QVector<double> angular_vel_y; QVector<double> angular_vel_z;
    QVector<double> linear_vel_norm; QVector<double> angular_vel_norm;

    QVector<double> linear_acc_x; QVector<double> linear_acc_y; QVector<double> linear_acc_z;
    QVector<double> angular_acc_x; QVector<double> angular_acc_y; QVector<double> angular_acc_z;
    QVector<double> linear_acc_norm; QVector<double> angular_acc_norm;



    for(size_t i=0;i<linear_velocity.size();++i){
        vector<double> linear_acc = linear_acceleration.at(i);
        vector<double> angular_acc = angular_acceleration.at(i);
        vector<double> linear_vel = linear_velocity.at(i);
        vector<double> angular_vel = angular_velocity.at(i);
        vector<double> linear_pos = position.at(i);
        vector<double> angular_pos = orientation.at(i);
        linear_pos_x.push_back(linear_pos.at(0));
        linear_pos_y.push_back(linear_pos.at(1));
        linear_pos_z.push_back(linear_pos.at(2));
        linear_pos_norm.push_back(sqrt(pow(linear_pos.at(0),2)+pow(linear_pos.at(1),2)+pow(linear_pos.at(2),2)));
        angular_pos_x.push_back(angular_pos.at(0));
        angular_pos_y.push_back(angular_pos.at(1));
        angular_pos_z.push_back(angular_pos.at(2));
        angular_pos_norm.push_back(sqrt(pow(angular_pos.at(0),2)+pow(angular_pos.at(1),2)+pow(angular_pos.at(2),2)));
        linear_vel_x.push_back(linear_vel.at(0));
        linear_vel_y.push_back(linear_vel.at(1));
        linear_vel_z.push_back(linear_vel.at(2));
        linear_vel_norm.push_back(sqrt(pow(linear_vel.at(0),2)+pow(linear_vel.at(1),2)+pow(linear_vel.at(2),2)));
        angular_vel_x.push_back(angular_vel.at(0));
        angular_vel_y.push_back(angular_vel.at(1));
        angular_vel_z.push_back(angular_vel.at(2));
        angular_vel_norm.push_back(sqrt(pow(angular_vel.at(0),2)+pow(angular_vel.at(1),2)+pow(angular_vel.at(2),2)));
        linear_acc_x.push_back(linear_acc.at(0));
        linear_acc_y.push_back(linear_acc.at(1));
        linear_acc_z.push_back(linear_acc.at(2));
        linear_acc_norm.push_back(sqrt(pow(linear_acc.at(0),2)+pow(linear_acc.at(1),2)+pow(linear_acc.at(2),2)));
        angular_acc_x.push_back(angular_acc.at(0));
        angular_acc_y.push_back(angular_acc.at(1));
        angular_acc_z.push_back(angular_acc.at(2));
        angular_acc_norm.push_back(sqrt(pow(angular_acc.at(0),2)+pow(angular_acc.at(1),2)+pow(angular_vel.at(2),2)));
    }

    switch(mod){
    case 0: // shoulder
        plotComp(ui->plot_shoulder_x,QString("Shoulder linear component x"),time,linear_pos_x,linear_vel_x,linear_acc_x,true); // plot linear  x
        plotComp(ui->plot_shoulder_y,QString("Shoulder linear component y"),time,linear_pos_y,linear_vel_y,linear_acc_y,true); // plot linear y
        plotComp(ui->plot_shoulder_z,QString("Shoulder linear component z"),time,linear_pos_z,linear_vel_z,linear_acc_z,true); // plot linear z
        plotComp(ui->plot_shoulder_lin_vel,QString("Shoulder linear component norm"),time,linear_pos_norm,linear_vel_norm,linear_acc_norm,true); // plot linear norm
        plotComp(ui->plot_shoulder_wx,QString("Shoulder angular component x"),time,angular_pos_x,angular_vel_x,angular_acc_x,false); // plot wx
        plotComp(ui->plot_shoulder_wy,QString("Shoulder angular component y"),time,angular_pos_y,angular_vel_y,angular_acc_y,false); // plot  wy
        plotComp(ui->plot_shoulder_wz,QString("Shoulder angular component z"),time,angular_pos_z,angular_vel_z,angular_acc_z,false); // plot  wz
        plotComp(ui->plot_shoulder_ang_vel,QString("Shoulder angular component norm"),time,angular_pos_norm,angular_vel_norm,angular_acc_norm,false); // plot angular  norm
        break;
    case 1:// elbow
        plotComp(ui->plot_elbow_x,QString("Elbow linear component x"),time,linear_pos_x,linear_vel_x,linear_acc_x,true); // plot linear  x
        plotComp(ui->plot_elbow_y,QString("Elbow linear component y"),time,linear_pos_y,linear_vel_y,linear_acc_y,true); // plot linear  y
        plotComp(ui->plot_elbow_z,QString("Elbow linear component z"),time,linear_pos_z,linear_vel_z,linear_acc_z,true); // plot linear  z
        plotComp(ui->plot_elbow_lin_vel,QString("Elbow linear component norm"),time,linear_pos_norm,linear_vel_norm,linear_acc_norm,true); // plot linear  norm
        plotComp(ui->plot_elbow_wx,QString("Elbow angular component x"),time,angular_pos_x,angular_vel_x,angular_acc_x,false); // plot  wx
        plotComp(ui->plot_elbow_wy,QString("Elbow angular component y"),time,angular_pos_y,angular_vel_y,angular_acc_y,false); // plot wy
        plotComp(ui->plot_elbow_wz,QString("Elbow angular component z"),time,angular_pos_z,angular_vel_z,angular_acc_z,false); // plot  wz
        plotComp(ui->plot_elbow_ang_vel,QString("Elbow angular component norm"),time,angular_pos_norm,angular_vel_norm,angular_acc_norm,false); // plot angular  norm
        break;
    case 2: // wrist
        plotComp(ui->plot_wrist_x,QString("Wrist linear component x"),time,linear_pos_x,linear_vel_x,linear_acc_x,true); // plot linear x
        plotComp(ui->plot_wrist_y,QString("Wrist linear component y"),time,linear_pos_y,linear_vel_y,linear_acc_y,true); // plot linear  y
        plotComp(ui->plot_wrist_z,QString("Wrist linear component z"),time,linear_pos_z,linear_vel_z,linear_acc_z,true); // plot linear  z
        plotComp(ui->plot_wrist_lin_vel,QString("Wrist linear component norm"),time,linear_pos_norm,linear_vel_norm,linear_acc_norm,true); // plot linear norm
        plotComp(ui->plot_wrist_wx,QString("Wrist angular component x"),time,angular_acc_x,angular_vel_x,angular_acc_x,false); // plot  wx
        plotComp(ui->plot_wrist_wy,QString("Wrist angular component y"),time,angular_pos_y,angular_vel_y,angular_acc_y,false); // plot  wy
        plotComp(ui->plot_wrist_wz,QString("Wrist angular component z"),time,angular_pos_z,angular_vel_z,angular_acc_z,false); // plot  wz
        plotComp(ui->plot_wrist_ang_vel,QString("Wrist angular component norm"),time,angular_pos_norm,angular_vel_norm,angular_acc_norm,false); // plot angular norm
        break;
    case 3: default: // hand
        plotComp(ui->plot_hand_x,QString("Hand linear component x"),time,linear_pos_x,linear_vel_x,linear_acc_x,true); // plot linear vel x
        plotComp(ui->plot_hand_y,QString("Hand linear component y"),time,linear_pos_y,linear_vel_y,linear_acc_y,true); // plot linear vel y
        plotComp(ui->plot_hand_z,QString("Hand linear component z"),time,linear_pos_z,linear_vel_z,linear_acc_z,true); // plot linear vel z
        plotComp(ui->plot_hand_lin_vel,QString("Hand linear component norm"),time,linear_pos_norm,linear_vel_norm,linear_acc_norm,true); // plot linear vel norm
        plotComp(ui->plot_hand_wx,QString("Hand angular component x"),time,angular_pos_x,angular_vel_x,angular_acc_x,false); // plot vel wx
        plotComp(ui->plot_hand_wy,QString("Hand angular component y"),time,angular_pos_y,angular_vel_y,angular_acc_y,false); // plot vel wy
        plotComp(ui->plot_hand_wz,QString("Hand angular component z"),time,angular_pos_z,angular_vel_z,angular_acc_z,false); // plot vel wz
        plotComp(ui->plot_hand_ang_vel,QString("Hand angular component norm"),time,angular_pos_norm,angular_vel_norm,angular_acc_norm,false); // plot angular vel norm
        break;        
    }
}


void CompVelocityDialog::plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &pos,QVector<double> &vel,QVector<double> &acc,bool lin)
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

    QFont *title_font = new QFont(); title_font->setPointSize(12);
    QCPPlotTitle *plot_title = new QCPPlotTitle(plot,title); plot_title->setFont(*title_font);
    plot->plotLayout()->addElement(0,0, plot_title);
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
    plot->graph(0)->setData(time, pos);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(pos.begin(), pos.end()),
                                          *std::max_element(pos.begin(), pos.end()));
    plot->graph(0)->rescaleAxes();

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName(name_vel);
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("time [s]");
    plot->graph(1)->setData(time, vel);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(vel.begin(), vel.end()),
                                                      *std::max_element(vel.begin(), vel.end()));
    plot->graph(1)->rescaleAxes();

    // acceleration
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    plot->graph(2)->setPen(QPen(Qt::darkGreen));
    plot->graph(2)->setName(name_acc);
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

void CompVelocityDialog::on_pushButton_save_shoulder_clicked()
{

    QString path;

    if(dual)
    {
        if(right){
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/shoulder_right", &st) == -1) {
                mkdir("results/planning/shoulder_right", 0700);
            }
            path = QString("results/planning/shoulder_right/");
        }else{
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/shoulder_left", &st) == -1) {
                mkdir("results/planning/shoulder_left", 0700);
            }
            path = QString("results/planning/shoulder_left/");
        }
    }else{
        struct stat st = {0};
        if (stat("results", &st) == -1) {
            mkdir("results", 0700);
        }
        if (stat("results/planning", &st) == -1) {
            mkdir("results/planning", 0700);
        }
        if (stat("results/planning/shoulder", &st) == -1) {
            mkdir("results/planning/shoulder", 0700);
        }
        path = QString("results/planning/shoulder/");
    }

    ui->plot_shoulder_x->savePdf(path+QString("shoulder_x.pdf"),true,0,0,QString(),QString("Shoulder Linear Component x"));
    ui->plot_shoulder_y->savePdf(path+QString("shoulder_y.pdf"),true,0,0,QString(),QString("Shoulder Linear Component y"));
    ui->plot_shoulder_z->savePdf(path+QString("shoulder_z.pdf"),true,0,0,QString(),QString("Shoulder Linear Component z"));
    ui->plot_shoulder_lin_vel->savePdf(path+QString("shoulder_lin.pdf"),true,0,0,QString(),QString("Shoulder Linear Component Norm"));
    ui->plot_shoulder_wx->savePdf(path+QString("shoulder_wx.pdf"),true,0,0,QString(),QString("Shoulder Angular Component x"));
    ui->plot_shoulder_wy->savePdf(path+QString("shoulder_wy.pdf"),true,0,0,QString(),QString("Shoulder Angular Component y"));
    ui->plot_shoulder_wz->savePdf(path+QString("shoulder_wz.pdf"),true,0,0,QString(),QString("Shoulder Angular Component z"));
    ui->plot_shoulder_ang_vel->savePdf(path+QString("shoulder_ang.pdf"),true,0,0,QString(),QString("Shoulder Angular Component Norm"));

}

void CompVelocityDialog::on_pushButton_save_elbow_clicked()
{
    QString path;

    if(dual)
    {
        if(right){
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/elbow_right", &st) == -1) {
                mkdir("results/planning/elbow_right", 0700);
            }
            path = QString("results/planning/elbow_right/");
        }else{
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/elbow_left", &st) == -1) {
                mkdir("results/planning/elbow_left", 0700);
            }
            path = QString("results/planning/elbow_left/");
        }
    }else{
        struct stat st = {0};
        if (stat("results", &st) == -1) {
            mkdir("results", 0700);
        }
        if (stat("results/planning", &st) == -1) {
            mkdir("results/planning", 0700);
        }
        if (stat("results/planning/elbow", &st) == -1) {
            mkdir("results/planning/elbow", 0700);
        }
        path = QString("results/planning/elbow/");
    }

    ui->plot_elbow_x->savePdf(path+QString("elbow_x.pdf"),true,0,0,QString(),QString("Elbow Linear Component x"));
    ui->plot_elbow_y->savePdf(path+QString("elbow_y.pdf"),true,0,0,QString(),QString("Elbow Linear Component y"));
    ui->plot_elbow_z->savePdf(path+QString("elbow_z.pdf"),true,0,0,QString(),QString("Elbow Linear Component z"));
    ui->plot_elbow_lin_vel->savePdf(path+QString("elbow_lin.pdf"),true,0,0,QString(),QString("Elbow Linear Component Norm"));
    ui->plot_elbow_wx->savePdf(path+QString("elbow_wx.pdf"),true,0,0,QString(),QString("Elbow Angular Component x"));
    ui->plot_elbow_wy->savePdf(path+QString("elbow_wy.pdf"),true,0,0,QString(),QString("Elbow Angular Component y"));
    ui->plot_elbow_wz->savePdf(path+QString("elbow_wz.pdf"),true,0,0,QString(),QString("Elbow Angular Component z"));
    ui->plot_elbow_ang_vel->savePdf(path+QString("elbow_ang.pdf"),true,0,0,QString(),QString("Elbow Angular Component Norm"));
}

void CompVelocityDialog::on_pushButton_save_wrist_clicked()
{
    QString path;

    if(dual)
    {
        if(right){
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/wrist_right", &st) == -1) {
                mkdir("results/planning/wrist_right", 0700);
            }
            path = QString("results/planning/wrist_right/");
        }else{
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/wrist_left", &st) == -1) {
                mkdir("results/planning/wrist_left", 0700);
            }
            path = QString("results/planning/wrist_left/");
        }
    }else{
        struct stat st = {0};
        if (stat("results", &st) == -1) {
            mkdir("results", 0700);
        }
        if (stat("results/planning", &st) == -1) {
            mkdir("results/planning", 0700);
        }
        if (stat("results/planning/wrist", &st) == -1) {
            mkdir("results/planning/wrist", 0700);
        }
        path = QString("results/planning/wrist/");
    }

    ui->plot_wrist_x->savePdf(path+QString("wrist_x.pdf"),true,0,0,QString(),QString("Wrist Linear Component x"));
    ui->plot_wrist_y->savePdf(path+QString("wrist_y.pdf"),true,0,0,QString(),QString("Wrist Linear Component y"));
    ui->plot_wrist_z->savePdf(path+QString("wrist_z.pdf"),true,0,0,QString(),QString("Wrist Linear Component z"));
    ui->plot_wrist_lin_vel->savePdf(path+QString("wrist_lin.pdf"),true,0,0,QString(),QString("Wrist Linear Component Norm"));
    ui->plot_wrist_wx->savePdf(path+QString("wrist_wx.pdf"),true,0,0,QString(),QString("Wrist Angular Component x"));
    ui->plot_wrist_wy->savePdf(path+QString("wrist_wy.pdf"),true,0,0,QString(),QString("Wrist Angular Component y"));
    ui->plot_wrist_wz->savePdf(path+QString("wrist_wz.pdf"),true,0,0,QString(),QString("Wrist Angular Component z"));
    ui->plot_wrist_ang_vel->savePdf(path+QString("wrist_ang.pdf"),true,0,0,QString(),QString("Wrist Angular Component Norm"));
}

void CompVelocityDialog::on_pushButton_save_hand_clicked()
{

    QString path;

    if(dual)
    {
        if(right)
        {
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/hand_right", &st) == -1) {
                mkdir("results/planning/hand_right", 0700);
            }
            path = QString("results/planning/hand_right/");
        }else{
            struct stat st = {0};
            if (stat("results", &st) == -1) {
                mkdir("results", 0700);
            }
            if (stat("results/planning", &st) == -1) {
                mkdir("results/planning", 0700);
            }
            if (stat("results/planning/hand_left", &st) == -1) {
                mkdir("results/planning/hand_left", 0700);
            }
            path = QString("results/planning/hand_left/");
        }
    }else{
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
        path = QString("results/planning/hand/");
    }

    ui->plot_hand_x->savePdf(path+QString("hand_x.pdf"),true,0,0,QString(),QString("Hand Linear Component x"));
    ui->plot_hand_y->savePdf(path+QString("hand_y.pdf"),true,0,0,QString(),QString("Hand Linear Component y"));
    ui->plot_hand_z->savePdf(path+QString("hand_z.pdf"),true,0,0,QString(),QString("Hand Linear Component z"));
    ui->plot_hand_lin_vel->savePdf(path+QString("hand_lin.pdf"),true,0,0,QString(),QString("Hand Linear Component Norm"));
    ui->plot_hand_wx->savePdf(path+QString("hand_wx.pdf"),true,0,0,QString(),QString("Hand Angular Component x"));
    ui->plot_hand_wy->savePdf(path+QString("hand_wy.pdf"),true,0,0,QString(),QString("Hand Angular Component y"));
    ui->plot_hand_wz->savePdf(path+QString("hand_wz.pdf"),true,0,0,QString(),QString("Hand Angular Component z"));
    ui->plot_hand_ang_vel->savePdf(path+QString("hand_ang.pdf"),true,0,0,QString(),QString("Hand Angular Component Norm"));

}







} // namespace motion_manager
