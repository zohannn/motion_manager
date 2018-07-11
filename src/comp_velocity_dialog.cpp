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

void CompVelocityDialog::setupPlots(vector<vector<double>> &linear_velocity,vector<vector<double>> &angular_velocity,QVector<double> &time, int mod)
{
    //const double radtodeg = 180.0/static_cast<double>(M_PI);

    QVector<double> linear_vel_x; QVector<double> linear_vel_y; QVector<double> linear_vel_z;
    QVector<double> angular_vel_x; QVector<double> angular_vel_y; QVector<double> angular_vel_z;
    QVector<double> linear_vel_norm; QVector<double> angular_vel_norm;



    for(size_t i=0;i<linear_velocity.size();++i){
        vector<double> linear = linear_velocity.at(i);
        vector<double> angular = angular_velocity.at(i);
        linear_vel_x.push_back(linear.at(0));
        linear_vel_y.push_back(linear.at(1));
        linear_vel_z.push_back(linear.at(2));
        linear_vel_norm.push_back(sqrt(pow(linear.at(0),2)+pow(linear.at(1),2)+pow(linear.at(2),2)));
        angular_vel_x.push_back(angular.at(0));
        angular_vel_y.push_back(angular.at(1));
        angular_vel_z.push_back(angular.at(2));
        angular_vel_norm.push_back(sqrt(pow(angular.at(0),2)+pow(angular.at(1),2)+pow(angular.at(2),2)));
    }

    switch(mod){
    case 0: // shoulder
        plotComp(ui->plot_shoulder_x,QString("Shoulder linear velocity x"),time,linear_vel_x,true); // plot linear vel x
        plotComp(ui->plot_shoulder_y,QString("Shoulder linear velocity y"),time,linear_vel_y,true); // plot linear vel y
        plotComp(ui->plot_shoulder_z,QString("Shoulder linear velocity z"),time,linear_vel_z,true); // plot linear vel z
        plotComp(ui->plot_shoulder_lin_vel,QString("Shoulder linear velocity norm"),time,linear_vel_norm,true); // plot linear vel norm
        plotComp(ui->plot_shoulder_wx,QString("Shoulder angular velocity x"),time,angular_vel_x,false); // plot vel wx
        plotComp(ui->plot_shoulder_wy,QString("Shoulder angular velocity y"),time,angular_vel_y,false); // plot vel wy
        plotComp(ui->plot_shoulder_wz,QString("Shoulder angular velocity z"),time,angular_vel_z,false); // plot vel wz
        plotComp(ui->plot_shoulder_ang_vel,QString("Shoulder angular velocity norm"),time,angular_vel_norm,false); // plot angular vel norm
        break;
    case 1:// elbow
        plotComp(ui->plot_elbow_x,QString("Elbow linear velocity x"),time,linear_vel_x,true); // plot linear vel x
        plotComp(ui->plot_elbow_y,QString("Elbow linear velocity y"),time,linear_vel_y,true); // plot linear vel y
        plotComp(ui->plot_elbow_z,QString("Elbow linear velocity z"),time,linear_vel_z,true); // plot linear vel z
        plotComp(ui->plot_elbow_lin_vel,QString("Elbow linear velocity norm"),time,linear_vel_norm,true); // plot linear vel norm
        plotComp(ui->plot_elbow_wx,QString("Elbow angular velocity x"),time,angular_vel_x,false); // plot vel wx
        plotComp(ui->plot_elbow_wy,QString("Elbow angular velocity y"),time,angular_vel_y,false); // plot vel wy
        plotComp(ui->plot_elbow_wz,QString("Elbow angular velocity z"),time,angular_vel_z,false); // plot vel wz
        plotComp(ui->plot_elbow_ang_vel,QString("Elbow angular velocity norm"),time,angular_vel_norm,false); // plot angular vel norm
        break;
    case 2: // wrist
        plotComp(ui->plot_wrist_x,QString("Wrist linear velocity x"),time,linear_vel_x,true); // plot linear vel x
        plotComp(ui->plot_wrist_y,QString("Wrist linear velocity y"),time,linear_vel_y,true); // plot linear vel y
        plotComp(ui->plot_wrist_z,QString("Wrist linear velocity z"),time,linear_vel_z,true); // plot linear vel z
        plotComp(ui->plot_wrist_lin_vel,QString("Wrist linear velocity norm"),time,linear_vel_norm,true); // plot linear vel norm
        plotComp(ui->plot_wrist_wx,QString("Wrist angular velocity x"),time,angular_vel_x,false); // plot vel wx
        plotComp(ui->plot_wrist_wy,QString("Wrist angular velocity y"),time,angular_vel_y,false); // plot vel wy
        plotComp(ui->plot_wrist_wz,QString("Wrist angular velocity z"),time,angular_vel_z,false); // plot vel wz
        plotComp(ui->plot_wrist_ang_vel,QString("Wrist angular velocity norm"),time,angular_vel_norm,false); // plot angular vel norm
        break;
    case 3: default: // hand
        plotComp(ui->plot_hand_x,QString("Hand linear velocity x"),time,linear_vel_x,true); // plot linear vel x
        plotComp(ui->plot_hand_y,QString("Hand linear velocity y"),time,linear_vel_y,true); // plot linear vel y
        plotComp(ui->plot_hand_z,QString("Hand linear velocity z"),time,linear_vel_z,true); // plot linear vel z
        plotComp(ui->plot_hand_lin_vel,QString("Hand linear velocity norm"),time,linear_vel_norm,true); // plot linear vel norm
        plotComp(ui->plot_hand_wx,QString("Hand angular velocity x"),time,angular_vel_x,false); // plot vel wx
        plotComp(ui->plot_hand_wy,QString("Hand angular velocity y"),time,angular_vel_y,false); // plot vel wy
        plotComp(ui->plot_hand_wz,QString("Hand angular velocity z"),time,angular_vel_z,false); // plot vel wz
        plotComp(ui->plot_hand_ang_vel,QString("Hand angular velocity norm"),time,angular_vel_norm,false); // plot angular vel norm
        break;        
    }
}


void CompVelocityDialog::plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var,bool lin)
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

    QFont *title_font = new QFont(); title_font->setPointSize(12);
    QCPPlotTitle *plot_title = new QCPPlotTitle(plot,title); plot_title->setFont(*title_font);
    plot->plotLayout()->addElement(0,0, plot_title);
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

    ui->plot_shoulder_x->savePdf(path+QString("shoulder_vel_x.pdf"),true,0,0,QString(),QString("Shoulder Linear Velocity x"));
    ui->plot_shoulder_y->savePdf(path+QString("shoulder_vel_y.pdf"),true,0,0,QString(),QString("Shoulder Linear Velocity y"));
    ui->plot_shoulder_z->savePdf(path+QString("shoulder_vel_z.pdf"),true,0,0,QString(),QString("Shoulder Linear Velocity z"));
    ui->plot_shoulder_lin_vel->savePdf(path+QString("shoulder_lin_vel.pdf"),true,0,0,QString(),QString("Shoulder Linear Velocity Norm"));
    ui->plot_shoulder_wx->savePdf(path+QString("shoulder_vel_wx.pdf"),true,0,0,QString(),QString("Shoulder Angular Velocity x"));
    ui->plot_shoulder_wy->savePdf(path+QString("shoulder_vel_wy.pdf"),true,0,0,QString(),QString("Shoulder Angular Velocity x"));
    ui->plot_shoulder_wz->savePdf(path+QString("shoulder_vel_wz.pdf"),true,0,0,QString(),QString("Shoulder Angular Velocity x"));
    ui->plot_shoulder_ang_vel->savePdf(path+QString("shoulder_ang_vel.pdf"),true,0,0,QString(),QString("Shoulder Angular Velocity Norm"));

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

    ui->plot_elbow_x->savePdf(path+QString("elbow_vel_x.pdf"),true,0,0,QString(),QString("Elbow Linear Velocity x"));
    ui->plot_elbow_y->savePdf(path+QString("elbow_vel_y.pdf"),true,0,0,QString(),QString("Elbow Linear Velocity y"));
    ui->plot_elbow_z->savePdf(path+QString("elbow_vel_z.pdf"),true,0,0,QString(),QString("Elbow Linear Velocity z"));
    ui->plot_elbow_lin_vel->savePdf(path+QString("elbow_lin_vel.pdf"),true,0,0,QString(),QString("Elbow Linear Velocity Norm"));
    ui->plot_elbow_wx->savePdf(path+QString("elbow_vel_wx.pdf"),true,0,0,QString(),QString("Elbow Angular Velocity x"));
    ui->plot_elbow_wy->savePdf(path+QString("elbow_vel_wy.pdf"),true,0,0,QString(),QString("Elbow Angular Velocity x"));
    ui->plot_elbow_wz->savePdf(path+QString("elbow_vel_wz.pdf"),true,0,0,QString(),QString("Elbow Angular Velocity x"));
    ui->plot_elbow_ang_vel->savePdf(path+QString("elbow_ang_vel.pdf"),true,0,0,QString(),QString("Elbow Angular Velocity Norm"));
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

    ui->plot_wrist_x->savePdf(path+QString("wrist_vel_x.pdf"),true,0,0,QString(),QString("Wrist Linear Velocity x"));
    ui->plot_wrist_y->savePdf(path+QString("wrist_vel_y.pdf"),true,0,0,QString(),QString("Wrist Linear Velocity y"));
    ui->plot_wrist_z->savePdf(path+QString("wrist_vel_z.pdf"),true,0,0,QString(),QString("Wrist Linear Velocity z"));
    ui->plot_wrist_lin_vel->savePdf(path+QString("wrist_lin_vel.pdf"),true,0,0,QString(),QString("Wrist Linear Velocity Norm"));
    ui->plot_wrist_wx->savePdf(path+QString("wrist_vel_wx.pdf"),true,0,0,QString(),QString("Wrist Angular Velocity x"));
    ui->plot_wrist_wy->savePdf(path+QString("wrist_vel_wy.pdf"),true,0,0,QString(),QString("Wrist Angular Velocity x"));
    ui->plot_wrist_wz->savePdf(path+QString("wrist_vel_wz.pdf"),true,0,0,QString(),QString("Wrist Angular Velocity x"));
    ui->plot_wrist_ang_vel->savePdf(path+QString("wrist_ang_vel.pdf"),true,0,0,QString(),QString("Wrist Angular Velocity Norm"));
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

    ui->plot_hand_x->savePdf(path+QString("hand_vel_x.pdf"),true,0,0,QString(),QString("Hand Linear Velocity x"));
    ui->plot_hand_y->savePdf(path+QString("hand_vel_y.pdf"),true,0,0,QString(),QString("Hand Linear Velocity y"));
    ui->plot_hand_z->savePdf(path+QString("hand_vel_z.pdf"),true,0,0,QString(),QString("Hand Linear Velocity z"));
    ui->plot_hand_lin_vel->savePdf(path+QString("hand_lin_vel.pdf"),true,0,0,QString(),QString("Hand Linear Velocity Norm"));
    ui->plot_hand_wx->savePdf(path+QString("hand_vel_wx.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));
    ui->plot_hand_wy->savePdf(path+QString("hand_vel_wy.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));
    ui->plot_hand_wz->savePdf(path+QString("hand_vel_wz.pdf"),true,0,0,QString(),QString("Hand Angular Velocity x"));
    ui->plot_hand_ang_vel->savePdf(path+QString("hand_ang_vel.pdf"),true,0,0,QString(),QString("Hand Angular Velocity Norm"));

}







} // namespace motion_manager
