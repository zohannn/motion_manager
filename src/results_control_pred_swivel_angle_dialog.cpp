#include "../include/motion_manager/results_control_pred_swivel_angle_dialog.hpp"

namespace motion_manager {

ResultsCtrlPredSwivelAngleDialog::ResultsCtrlPredSwivelAngleDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ResultsCtrlPredSwivelAngleDialog)
{
    ui->setupUi(this);

}

ResultsCtrlPredSwivelAngleDialog::~ResultsCtrlPredSwivelAngleDialog()
{
    delete ui;
}


void ResultsCtrlPredSwivelAngleDialog::setupPlots(vector<double> &swivel_angle_pos, vector<double> &swivel_angle_vel, vector<double> &time)
{
    this->swivel_angle_pos = swivel_angle_pos;
    this->swivel_angle_vel = swivel_angle_vel;
    this->time = time;
}

void ResultsCtrlPredSwivelAngleDialog::plotSwivel(QCustomPlot* plot, QString title, QVector<double>& time, QVector<double>& data, bool vel)
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
    if(vel){
        name = "[deg/s]";
    }else{
        name = "[deg]";
    }
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    if(vel){
        plot->graph(0)->setPen(QPen(Qt::red));
        plot->graph(0)->valueAxis()->setTickLabelColor(Qt::red);
    }else{
        plot->graph(0)->setPen(QPen(Qt::blue));
        plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    }
    plot->graph(0)->setName(name);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, data);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(data.begin(),data.end()),*std::max_element(data.begin(),data.end()));
    plot->graph(0)->rescaleAxes();
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    plot->replot();
}


// Q_SLOTS

void ResultsCtrlPredSwivelAngleDialog::on_pushButton_plot_clicked()
{
//    const double radtodeg = 180.0/static_cast<double>(M_PI);

//    vector<double> swivel_angle_pos_deg(this->swivel_angle_pos.size());
//    vector<double> swivel_angle_vel_deg(this->swivel_angle_vel.size());
//    std::transform (this->swivel_angle_pos.begin(),this->swivel_angle_pos.end(),swivel_angle_pos_deg.begin(),std::bind1st (std::multiplies<double>(),radtodeg));
//    std::transform (this->swivel_angle_vel.begin(),this->swivel_angle_vel.end(),swivel_angle_vel_deg.begin(),std::bind1st (std::multiplies<double>(),radtodeg));

    QVector<double> swivel_angle_pos_deg_q = QVector<double>::fromStdVector(this->swivel_angle_pos);
    QVector<double> swivel_angle_vel_deg_q = QVector<double>::fromStdVector(this->swivel_angle_vel);
    QVector<double> qtime = QVector<double>::fromStdVector(this->time);

    plotSwivel(ui->plot_pred_swivel_angle_pos,QString("Predicted Swivel Angle"),qtime,swivel_angle_pos_deg_q,false); // plot swivel_angle_pos_deg
    plotSwivel(ui->plot_pred_swivel_angle_vel,QString("Time Derivative of the predicted Swivel Angle"),qtime,swivel_angle_vel_deg_q,true); // plot swivel_angle_vel_deg

}

void ResultsCtrlPredSwivelAngleDialog::on_pushButton_save_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/pred_swivel_angle", &st) == -1) {
        mkdir("results/controlling/pred_swivel_angle", 0700);
    }
    path = QString("results/controlling/pred_swivel_angle/");

    ui->plot_pred_swivel_angle_pos->savePdf(path+QString("pred_swivel_angle_pos.pdf"),true,0,0,QString(),QString("Predicted Swivel Angle"));
    ui->plot_pred_swivel_angle_vel->savePdf(path+QString("plot_pred_swivel_angle_vel.pdf"),true,0,0,QString(),QString("Time Derivative of the predicted Swivel Angle"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("pred_swivel_angle_pos.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("pred_swivel_angle_pos.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("plot_pred_swivel_angle_vel.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("plot_pred_swivel_angle_vel.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    // swivel angle position
    if(!this->swivel_angle_pos.empty()){
        string filename("swivel_angle_pos.txt");
        ofstream swivel_stream;
        swivel_stream.open(path.toStdString()+filename);

        swivel_stream << string("# SWIVEL ANGLE POSITION \n");
        swivel_stream << string("# swivel angle [deg], time [s] \n");

        for(size_t i=0;i<this->swivel_angle_pos.size();++i){
            double sw = this->swivel_angle_pos.at(i);
            double time = this->time.at(i);
            string sw_str =  boost::str(boost::format("%.15f") % (sw)); boost::replace_all(sw_str,",",".");
            string t_str =  boost::str(boost::format("%.15f") % (time)); boost::replace_all(t_str,",",".");
            swivel_stream << sw_str+string(", ")+t_str+string("\n");
        }
        swivel_stream.close();
    }

    // swivel angle velocity
    if(!this->swivel_angle_vel.empty()){
        string filename("swivel_angle_vel.txt");
        ofstream swivel_stream;
        swivel_stream.open(path.toStdString()+filename);

        swivel_stream << string("# SWIVEL ANGLE VELOCITY \n");
        swivel_stream << string("# swivel angle [deg/s], time [s] \n");

        for(size_t i=0;i<this->swivel_angle_vel.size();++i){
            double sw = this->swivel_angle_vel.at(i);
            double time = this->time.at(i);
            string sw_str =  boost::str(boost::format("%.15f") % (sw)); boost::replace_all(sw_str,",",".");
            string t_str =  boost::str(boost::format("%.15f") % (time)); boost::replace_all(t_str,",",".");
            swivel_stream << sw_str+string(", ")+t_str+string("\n");
        }
        swivel_stream.close();
    }

}


} // namespace motion_manager
