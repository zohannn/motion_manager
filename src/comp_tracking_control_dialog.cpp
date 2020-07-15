#include "../include/motion_manager/comp_tracking_control_dialog.hpp"

namespace motion_manager {

CompTrackingControlDialog::CompTrackingControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CompTrackingControlDialog)
{
    ui->setupUi(this);
}

CompTrackingControlDialog::~CompTrackingControlDialog()
{
    delete ui;
}

void CompTrackingControlDialog::setupPlots(vector<vector<double>> &pos_hand, vector<vector<double>> &or_hand, vector<vector<double>> &des_pos_hand,
                                           vector<vector<double>> &pos_fing, vector<vector<double>> &des_pos_fing,
                                           vector<double> &pos_alpha, vector<double> &des_pos_alpha,
                                           vector<double> &time)
{

    this->qtime = QVector<double>::fromStdVector(time);

    this->positions_hand = pos_hand; this->des_positions_hand = des_pos_hand;
    this->orientations_hand = or_hand;
    this->positions_fing = pos_fing; this->des_positions_fing = des_pos_fing;

    this->qalpha_pos = QVector<double>::fromStdVector(pos_alpha);
    this->qdes_alpha_pos = QVector<double>::fromStdVector(des_pos_alpha);

}


void CompTrackingControlDialog::plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var_real, QVector<double> &var_des)
{
    plot->plotLayout()->clear();
    plot->clearGraphs();
    plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(plot);
    wideAxisRect->setupFullAxesBox(true);
    wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::red));
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
    plot->graph(0)->setName("real");
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, var_real);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(var_real.begin(), var_real.end()),
                                                      *std::max_element(var_real.begin(), var_real.end()));
    plot->graph(0)->rescaleAxes();

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName("desired");
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("time [s]");
    plot->graph(1)->setData(time, var_des);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(var_des.begin(), var_des.end()),
                                                      *std::max_element(var_des.begin(), var_des.end()));
    plot->graph(1)->rescaleAxes();


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

    // interactions
    connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(1)->valueAxis(), SLOT(setRange(QCPRange)));
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


    plot->replot();
}


// Q_SLOTS


void CompTrackingControlDialog::on_pushButton_plot_hand_clicked()
{
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();


    LowPassFilter lpf_pos_x(f_th_pos, timestep_pos);
    LowPassFilter lpf_pos_y(f_th_pos, timestep_pos);
    LowPassFilter lpf_pos_z(f_th_pos, timestep_pos);
    LowPassFilter lpf_or_qx(f_th_pos, timestep_pos);
    LowPassFilter lpf_or_qy(f_th_pos, timestep_pos);
    LowPassFilter lpf_or_qz(f_th_pos, timestep_pos);
    LowPassFilter lpf_or_qw(f_th_pos, timestep_pos);


    // clear
    hand_pos_x.clear(); des_hand_pos_x.clear();
    hand_pos_y.clear(); des_hand_pos_y.clear();
    hand_pos_z.clear(); des_hand_pos_z.clear();
    hand_or_qx.clear(); des_hand_or_qx.clear();
    hand_or_qy.clear(); des_hand_or_qy.clear();
    hand_or_qz.clear(); des_hand_or_qz.clear();
    hand_or_qw.clear(); des_hand_or_qw.clear();

    for(size_t i=0;i<qtime.size();++i){
        vector<double> pos_hand = positions_hand.at(i);
        vector<double> or_hand = orientations_hand.at(i);
        vector<double> des_pos_hand = des_positions_hand.at(i);

        hand_pos_x.push_back(pos_hand.at(0));
        hand_pos_y.push_back(pos_hand.at(1));
        hand_pos_z.push_back(pos_hand.at(2));
        hand_or_qx.push_back(or_hand.at(0));
        hand_or_qy.push_back(or_hand.at(1));
        hand_or_qz.push_back(or_hand.at(2));
        hand_or_qw.push_back(or_hand.at(3));

        des_hand_pos_x.push_back(lpf_pos_x.update(des_pos_hand.at(0)));
        des_hand_pos_y.push_back(lpf_pos_y.update(des_pos_hand.at(1)));
        des_hand_pos_z.push_back(lpf_pos_z.update(des_pos_hand.at(2)));
        des_hand_or_qx.push_back(lpf_or_qx.update(des_pos_hand.at(3)));
        des_hand_or_qy.push_back(lpf_or_qy.update(des_pos_hand.at(4)));
        des_hand_or_qz.push_back(lpf_or_qz.update(des_pos_hand.at(5)));
        des_hand_or_qw.push_back(lpf_or_qw.update(des_pos_hand.at(6)));
    }


    plotComp(ui->plot_hand_x,QString("Hand tracking position x [mm]"),qtime,hand_pos_x,des_hand_pos_x);
    plotComp(ui->plot_hand_y,QString("Hand tracking position y [mm]"),qtime,hand_pos_y,des_hand_pos_y);
    plotComp(ui->plot_hand_z,QString("Hand tracking position z [mm]"),qtime,hand_pos_z,des_hand_pos_z);
    plotComp(ui->plot_hand_qx,QString("Hand tracking orientation qx"),qtime,hand_or_qx,des_hand_or_qx);
    plotComp(ui->plot_hand_qy,QString("Hand tracking orientation qy"),qtime,hand_or_qy,des_hand_or_qy);
    plotComp(ui->plot_hand_qz,QString("Hand tracking orientation qz"),qtime,hand_or_qz,des_hand_or_qz);
    plotComp(ui->plot_hand_qw,QString("Hand tracking orientation qw"),qtime,hand_or_qw,des_hand_or_qw);

}

void CompTrackingControlDialog::on_pushButton_plot_fing_clicked()
{
    const double radtodeg = 180.0/static_cast<double>(M_PI);
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();

    LowPassFilter lpf_fing_0(f_th_pos, timestep_pos);
    LowPassFilter lpf_fing_1(f_th_pos, timestep_pos);
    LowPassFilter lpf_fing_2(f_th_pos, timestep_pos);
    LowPassFilter lpf_fing_3(f_th_pos, timestep_pos);

    // clear
    pos_fing_0.clear(); des_pos_fing_0.clear();
    pos_fing_1.clear(); des_pos_fing_1.clear();
    pos_fing_2.clear(); des_pos_fing_2.clear();
    pos_fing_3.clear(); des_pos_fing_3.clear();

    for(size_t i=0;i<qtime.size();++i){
        vector<double> pos_fing = positions_fing.at(i);
        vector<double> des_pos_fing = des_positions_fing.at(i);

        pos_fing_0.push_back(lpf_fing_0.update(radtodeg*pos_fing.at(0)));
        pos_fing_1.push_back(lpf_fing_1.update(radtodeg*pos_fing.at(1)));
        pos_fing_2.push_back(lpf_fing_2.update(radtodeg*pos_fing.at(2)));
        pos_fing_3.push_back(lpf_fing_3.update(radtodeg*pos_fing.at(3)));

        des_pos_fing_0.push_back(radtodeg*des_pos_fing.at(0));
        des_pos_fing_1.push_back(radtodeg*des_pos_fing.at(1));
        des_pos_fing_2.push_back(radtodeg*des_pos_fing.at(2));
        des_pos_fing_3.push_back(radtodeg*des_pos_fing.at(3));

    }

    plotComp(ui->plot_fing_0,QString("Finger 0 position tracking [deg]"),qtime,pos_fing_0,des_pos_fing_0);
    plotComp(ui->plot_fing_1,QString("Finger 1 position tracking [deg]"),qtime,pos_fing_1,des_pos_fing_1);
    plotComp(ui->plot_fing_2,QString("Finger 2 position tracking [deg]"),qtime,pos_fing_2,des_pos_fing_2);
    plotComp(ui->plot_fing_3,QString("Finger 3 position tracking [deg]"),qtime,pos_fing_3,des_pos_fing_3);
}

void CompTrackingControlDialog::on_pushButton_plot_alpha_clicked()
{
    const double radtodeg = 180.0/static_cast<double>(M_PI);
    double f_th_pos = this->ui->lineEdit_f_cutoff_pos->text().toDouble();
    double timestep_pos = this->ui->lineEdit_time_step_pos->text().toDouble();

    LowPassFilter lpf_alpha(f_th_pos, timestep_pos);

    QVector<double> pos_a;
    QVector<double> des_pos_a;
    for(size_t i=0;i<qtime.size();++i){
        pos_a.push_back(lpf_alpha.update(radtodeg*this->qalpha_pos.at(i)));
        des_pos_a.push_back(lpf_alpha.update(radtodeg*this->qdes_alpha_pos.at(i)));
    }
    plotComp(ui->plot_alpha_pos,QString("Swivel angle position tracking [deg]"),qtime,pos_a,des_pos_a);
}

void CompTrackingControlDialog::on_pushButton_save_hand_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/tracking", &st) == -1) {
        mkdir("results/controlling/tracking", 0700);
    }
    if (stat("results/controlling/tracking/hand", &st) == -1) {
        mkdir("results/controlling/tracking/hand", 0700);
    }
    path = QString("results/controlling/tracking/hand/");

    ui->plot_hand_x->savePdf(path+QString("hand_track_pos_x.pdf"),true,0,0,QString(),QString("Hand tracking position x [mm]"));
    ui->plot_hand_y->savePdf(path+QString("hand_track_pos_y.pdf"),true,0,0,QString(),QString("Hand tracking position y [mm]"));
    ui->plot_hand_z->savePdf(path+QString("hand_track_pos_z.pdf"),true,0,0,QString(),QString("Hand tracking position z [mm]"));
    ui->plot_hand_qx->savePdf(path+QString("hand_track_or_qx.pdf"),true,0,0,QString(),QString("Hand tracking orientation qx"));
    ui->plot_hand_qy->savePdf(path+QString("hand_track_or_qy.pdf"),true,0,0,QString(),QString("Hand tracking orientation qy"));
    ui->plot_hand_qz->savePdf(path+QString("hand_track_or_qz.pdf"),true,0,0,QString(),QString("Hand tracking orientation qz"));
    ui->plot_hand_qw->savePdf(path+QString("hand_track_or_qw.pdf"),true,0,0,QString(),QString("Hand tracking orientation qw"));

    // save data
    if(!this->hand_pos_x.empty()){
        string filename("hand_tracking.txt");
        ofstream hand_stream;
        hand_stream.open(path.toStdString()+filename);

        hand_stream << string("# HAND TRACKING \n");
        hand_stream << string("# position x [mm], position y [mm], position z [mm], orientation qx, orientation qy, orientation qz, orientation qw, "
                              " des position x [mm], des position y [mm], des position z [mm], des orientation qx, des orientation qy, des orientation qz, des orientation qw,"
                              "time [s] \n");

        for(size_t i=0;i<this->hand_pos_x.size();++i){

            double pos_x = this->hand_pos_x.at(i);
            double pos_y = this->hand_pos_y.at(i);
            double pos_z = this->hand_pos_z.at(i);
            double or_qx = this->hand_or_qx.at(i);
            double or_qy = this->hand_or_qy.at(i);
            double or_qz = this->hand_or_qz.at(i);
            double or_qw = this->hand_or_qw.at(i);

            string pos_x_str =  boost::str(boost::format("%.2f") % (pos_x)); boost::replace_all(pos_x_str,",",".");
            string pos_y_str =  boost::str(boost::format("%.2f") % (pos_y)); boost::replace_all(pos_y_str,",",".");
            string pos_z_str =  boost::str(boost::format("%.2f") % (pos_z)); boost::replace_all(pos_z_str,",",".");
            string or_qx_str =  boost::str(boost::format("%.2f") % (or_qx)); boost::replace_all(or_qx_str,",",".");
            string or_qy_str =  boost::str(boost::format("%.2f") % (or_qy)); boost::replace_all(or_qy_str,",",".");
            string or_qz_str =  boost::str(boost::format("%.2f") % (or_qz)); boost::replace_all(or_qz_str,",",".");
            string or_qw_str =  boost::str(boost::format("%.2f") % (or_qw)); boost::replace_all(or_qw_str,",",".");


            double des_pos_x = this->des_hand_pos_x.at(i);
            double des_pos_y = this->des_hand_pos_y.at(i);
            double des_pos_z = this->des_hand_pos_z.at(i);
            double des_or_qx = this->des_hand_or_qx.at(i);
            double des_or_qy = this->des_hand_or_qy.at(i);
            double des_or_qz = this->des_hand_or_qz.at(i);
            double des_or_qw = this->des_hand_or_qw.at(i);

            string des_pos_x_str =  boost::str(boost::format("%.2f") % (des_pos_x)); boost::replace_all(des_pos_x_str,",",".");
            string des_pos_y_str =  boost::str(boost::format("%.2f") % (des_pos_y)); boost::replace_all(des_pos_y_str,",",".");
            string des_pos_z_str =  boost::str(boost::format("%.2f") % (des_pos_z)); boost::replace_all(des_pos_z_str,",",".");
            string des_or_qx_str =  boost::str(boost::format("%.2f") % (des_or_qx)); boost::replace_all(des_or_qx_str,",",".");
            string des_or_qy_str =  boost::str(boost::format("%.2f") % (des_or_qy)); boost::replace_all(des_or_qy_str,",",".");
            string des_or_qz_str =  boost::str(boost::format("%.2f") % (des_or_qz)); boost::replace_all(des_or_qz_str,",",".");
            string des_or_qw_str =  boost::str(boost::format("%.2f") % (des_or_qw)); boost::replace_all(des_or_qw_str,",",".");


            // time
            double time = this->qtime.at(i);
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");

            hand_stream << pos_x_str+string(", ")+pos_y_str+string(", ")+pos_z_str+string(", ")+or_qx_str+string(", ")+or_qy_str+string(", ")+or_qz_str+string(", ")+or_qw_str+string(", ")
                           +des_pos_x_str+string(", ")+des_pos_y_str+string(", ")+des_pos_z_str+string(", ")+des_or_qx_str+string(", ")+des_or_qy_str+string(", ")+des_or_qz_str+string(", ")+des_or_qw_str+string(", ")
                           +t_str+string("\n");
        }
        hand_stream.close();
    }

}

void CompTrackingControlDialog::on_pushButton_save_fing_clicked()
{
    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/tracking", &st) == -1) {
        mkdir("results/controlling/tracking", 0700);
    }
    if (stat("results/controlling/tracking/fingers", &st) == -1) {
        mkdir("results/controlling/tracking/fingers", 0700);
    }
    path = QString("results/controlling/tracking/fingers/");

    ui->plot_fing_0->savePdf(path+QString("fing_0_pos_track.pdf"),true,0,0,QString(),QString("Finger 0 position tracking [deg]"));
    ui->plot_fing_1->savePdf(path+QString("fing_1_pos_track.pdf"),true,0,0,QString(),QString("Finger 1 position tracking [deg]"));
    ui->plot_fing_2->savePdf(path+QString("fing_2_pos_track.pdf"),true,0,0,QString(),QString("Finger 2 position tracking [deg]"));
    ui->plot_fing_3->savePdf(path+QString("fing_3_pos_track.pdf"),true,0,0,QString(),QString("Finger 3 position tracking [deg]"));

    // save data
    if(!this->pos_fing_0.empty()){
        string filename("fingers_tracking.txt");
        ofstream hand_stream;
        hand_stream.open(path.toStdString()+filename);

        hand_stream << string("# FINGERS TRACKING \n");
        hand_stream << string("# finger 0 [deg], finger 1 [deg], finger 2 [deg], finger 3 [deg],"
                              " des finger 0 [deg], des finger 1 [deg], des finger 2 [deg], des finger 3 [deg],"
                              "time [s] \n");

        for(size_t i=0;i<this->pos_fing_0.size();++i){

            double fing_pos_0 = this->pos_fing_0.at(i);
            double fing_pos_1 = this->pos_fing_1.at(i);
            double fing_pos_2 = this->pos_fing_2.at(i);
            double fing_pos_3 = this->pos_fing_3.at(i);

            string fing_pos_0_str =  boost::str(boost::format("%.2f") % (fing_pos_0)); boost::replace_all(fing_pos_0_str,",",".");
            string fing_pos_1_str =  boost::str(boost::format("%.2f") % (fing_pos_1)); boost::replace_all(fing_pos_1_str,",",".");
            string fing_pos_2_str =  boost::str(boost::format("%.2f") % (fing_pos_2)); boost::replace_all(fing_pos_2_str,",",".");
            string fing_pos_3_str =  boost::str(boost::format("%.2f") % (fing_pos_3)); boost::replace_all(fing_pos_3_str,",",".");


            double des_fing_pos_0 = this->des_pos_fing_0.at(i);
            double des_fing_pos_1 = this->des_pos_fing_1.at(i);
            double des_fing_pos_2 = this->des_pos_fing_2.at(i);
            double des_fing_pos_3 = this->des_pos_fing_3.at(i);

            string des_fing_pos_0_str =  boost::str(boost::format("%.2f") % (des_fing_pos_0)); boost::replace_all(des_fing_pos_0_str,",",".");
            string des_fing_pos_1_str =  boost::str(boost::format("%.2f") % (des_fing_pos_1)); boost::replace_all(des_fing_pos_1_str,",",".");
            string des_fing_pos_2_str =  boost::str(boost::format("%.2f") % (des_fing_pos_2)); boost::replace_all(des_fing_pos_2_str,",",".");
            string des_fing_pos_3_str =  boost::str(boost::format("%.2f") % (des_fing_pos_3)); boost::replace_all(des_fing_pos_3_str,",",".");


            // time
            double time = this->qtime.at(i);
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");

            hand_stream << fing_pos_0_str+string(", ")+fing_pos_1_str+string(", ")+fing_pos_2_str+string(", ")+fing_pos_3_str+string(", ")
                           +des_fing_pos_0_str+string(", ")+des_fing_pos_1_str+string(", ")+des_fing_pos_2_str+string(", ")+des_fing_pos_3_str+string(", ")
                           +t_str+string("\n");
        }
        hand_stream.close();
    }

}

void CompTrackingControlDialog::on_pushButton_save_alpha_clicked()
{
    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/controlling", &st) == -1) {
        mkdir("results/controlling", 0700);
    }
    if (stat("results/controlling/tracking", &st) == -1) {
        mkdir("results/controlling/tracking", 0700);
    }
    if (stat("results/controlling/tracking/alpha", &st) == -1) {
        mkdir("results/controlling/tracking/alpha", 0700);
    }
    path = QString("results/controlling/tracking/alpha/");

    ui->plot_alpha_pos->savePdf(path+QString("alpha_pos_track.pdf"),true,0,0,QString(),QString("Swivel angle position tracking [rad]"));

    // save data
    if(!this->qalpha_pos.empty()){
        string filename("alpha_tracking.txt");
        ofstream alpha_stream;
        alpha_stream.open(path.toStdString()+filename);
        alpha_stream << string("# SWIVEL ANGLE TRACKING \n");
        alpha_stream << string("# alpha pos [deg], des alpha pos [deg], time [s] \n");

        for(size_t i=0;i<this->qalpha_pos.size();++i){

            string alpha_pos_str =  boost::str(boost::format("%.2f") % (this->qalpha_pos.at(i))); boost::replace_all(alpha_pos_str,",",".");
            string des_alpha_pos_str =  boost::str(boost::format("%.2f") % (this->qdes_alpha_pos.at(i))); boost::replace_all(des_alpha_pos_str,",",".");

            // time
            double time = this->qtime.at(i);
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");

            alpha_stream << alpha_pos_str+string(", ")+des_alpha_pos_str+string(", ")+t_str+string("\n");
        }
        alpha_stream.close();



    }

}







} // namespace motion_manager
