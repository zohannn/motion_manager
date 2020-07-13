#include "../include/motion_manager/results_plan_alpha_dialog.hpp"

namespace motion_manager {

ResultsAlphaDialog::ResultsAlphaDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ResultsAlphaDialog)
{
    ui->setupUi(this);
}

ResultsAlphaDialog::~ResultsAlphaDialog()
{
    delete ui;
}

void ResultsAlphaDialog::setDual(bool d)
{
    this->dual = d;
}

void ResultsAlphaDialog::setRight(bool r)
{
    this->right = r;
}


void ResultsAlphaDialog::setupPlots(vector<vector<double>> &pos,vector<vector<double>> &vel,vector<vector<double>> &acc,vector<vector<double>> &timesteps)
{

    vector<double> time;
    qtime.clear(); pos_alpha.clear(); vel_alpha.clear(); acc_alpha.clear();

    for(size_t i=0; i<pos.size();++i){
        vector<double> pos_stage = pos.at(i);
        vector<double> vel_stage = vel.at(i);
        vector<double> acc_stage = acc.at(i);
        vector<double> tsteps_stage = timesteps.at(i);

        double time_init;
        if(time.empty()){
            time_init=0.0;
        }else{
            time_init=time.at(time.size()-1);
        }
        vector<double> time_stage(tsteps_stage.size());
        time_stage.at(0) = time_init;

        for(size_t j=0; j<pos_stage.size();++j){
            pos_alpha.push_back(pos_stage.at(j));
            vel_alpha.push_back(vel_stage.at(j));
            acc_alpha.push_back(acc_stage.at(j));
            if(j>0){time_stage.at(j) = time_stage.at(j-1) + tsteps_stage.at(j-1);}
        }
        time.reserve(time_stage.size());
        std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
    } // loop stages
    qtime = QVector<double>::fromStdVector(time);

    plotAlpha(ui->plot_alpha,QString("Swivel Angle"),qtime,pos_alpha,vel_alpha,acc_alpha);

}

void ResultsAlphaDialog::plotAlpha(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &pos, QVector<double> &vel, QVector<double> &acc)
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

    // position
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(0)->setName("[deg]");
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    plot->graph(0)->keyAxis()->setLabel("time [s]");
    plot->graph(0)->setData(time, pos);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(pos.begin(), pos.end()),
                                                      *std::max_element(pos.begin(), pos.end()));
    plot->graph(0)->rescaleAxes();

    // velocity
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName("[deg/s]");
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("time [s]");
    plot->graph(1)->setData(time, vel);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(vel.begin(), vel.end()),
                                                      *std::max_element(vel.begin(), vel.end()));
    plot->graph(1)->rescaleAxes();

    // acceleration
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    plot->graph(2)->setPen(QPen(Qt::darkGreen));
    plot->graph(2)->setName("[deg/s^2]");
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

void ResultsAlphaDialog::on_pushButton_save_clicked()
{

    QString path;
    if(dual)
    {
        // TO DO
    }else{
        struct stat st = {0};
        if (stat("results", &st) == -1) {
            mkdir("results", 0700);
        }
        if (stat("results/planning", &st) == -1) {
            mkdir("results/planning", 0700);
        }
        if (stat("results/planning/alpha", &st) == -1) {
            mkdir("results/planning/alpha", 0700);
        }
        path = QString("results/planning/alpha/");
    }

    ui->plot_alpha->savePdf(path+QString("alpha.pdf"),true,0,0,QString(),QString("Swivel angle"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("alpha.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("alpha.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    if(!this->pos_alpha.empty()){
        string filename("alpha.txt");
        ofstream stream;
        stream.open(path.toStdString()+filename);

        stream << string("# SWIVEL ANGLE \n");
        stream << string("# position [deg], velocity [deg/s], acceleration [deg/s^2], time [s] \n");

        for(size_t i=0;i<this->pos_alpha.size();++i){
            double pos = this->pos_alpha.at(i);
            double vel = this->vel_alpha.at(i);
            double acc = this->acc_alpha.at(i);
            double time = this->qtime.at(i);
            string pos_str =  boost::str(boost::format("%.2f") % (pos)); boost::replace_all(pos_str,",",".");
            string vel_str =  boost::str(boost::format("%.2f") % (vel)); boost::replace_all(vel_str,",",".");
            string acc_str =  boost::str(boost::format("%.2f") % (acc)); boost::replace_all(acc_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time)); boost::replace_all(t_str,",",".");
            stream << pos_str+string(", ")+vel_str+string(", ")+acc_str+string(", ")+t_str+string("\n");
        }
        stream.close();
    }
}







} // namespace motion_manager
