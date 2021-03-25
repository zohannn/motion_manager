#include "../include/motion_manager/powerlaw3ddialog.hpp"


namespace motion_manager {


PowerLaw3DDialog::PowerLaw3DDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::powerLaw3DDialog)
{
    ui->setupUi(this);
}

PowerLaw3DDialog::~PowerLaw3DDialog()
{
    delete ui;
}

void PowerLaw3DDialog::setupPlots(vector<vector<double> > &hand_position, vector<vector<vector<double> > > &timesteps)
{
    vector<vector<vector<vector<double>>>> hand_position_task;
    // time
    vector<double> time_task; QVector<double> tot_timesteps;
    vector<int> index;
    QVector<double> K; // Curvature of the task
    QVector<double> T; // Torsion of the task
    QVector<double> vel; // Velocity of the task
    QVector<double> acc; // Acceleration of the task
    double coeff_tot=0; // sum of the coefficient
    int n_coeff=0; // number of the non-zero coefficient
    int offset=0; // offset to go trough the stages of hand_position
    // clear
    this->slopes.clear(); this->r_squared.clear(); this->n_points.clear();
    this->vel_task.clear(); this->acc_task.clear();
    this->K_task.clear(); this->T_task.clear();

    for(size_t i=0; i<timesteps.size();++i){
        vector<vector<double>> tsteps_mov = timesteps.at(i);
        double time_init_mov;
        if(time_task.empty()){
            time_init_mov=0.0;
        }else{
            time_init_mov=time_task.at(time_task.size()-1);
        }
        vector<double> time_mov;
        vector<vector<vector<double>>> hand_position_mov;
        for(size_t j=0; j<tsteps_mov.size();++j){
            vector<double> tsteps_stage = tsteps_mov.at(j);
            vector<double> time_stage(tsteps_stage.size());
            double time_init_stage;
            if(time_mov.empty()){
                time_init_stage = time_init_mov;
            }else{
                time_init_stage = time_mov.at(time_mov.size()-1);
            }
            time_stage.at(0) = time_init_stage;
            vector<vector<double>> hand_position_stage;
            for(size_t k=0;k<tsteps_stage.size();++k){
                tot_timesteps.push_back(tsteps_stage.at(k));
                if(k>0){time_stage.at(k) = time_stage.at(k-1) + tsteps_stage.at(k-1);}
                hand_position_stage.push_back(hand_position.at(k+offset));
            }// stage
            offset += hand_position_stage.size();
            index.push_back(tot_timesteps.size());
            time_mov.reserve(time_stage.size());
            std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_mov));
            hand_position_mov.push_back(hand_position_stage);
        }// mov
        time_task.reserve(time_mov.size());
        std::copy (time_mov.begin(), time_mov.end(), std::back_inserter(time_task));
        hand_position_task.push_back(hand_position_mov);
    }// task
    QVector<double> qtime = QVector<double>::fromStdVector(time_task);


    QVector<double> ln_vel_tot; QVector<double> ln_x_tot;
    vector<double> q_tot; vector<double> m_tot; vector<double> r_tot;
    vector<double> q_tot_w; vector<double> m_tot_w; vector<double> r_tot_w;
    vector<double> q_tot_2w; vector<double> m_tot_2w; vector<double> r_tot_2w;

    for(size_t i=0; i<hand_position_task.size();++i){
        vector<vector<vector<double>>> hand_position_mov = hand_position_task.at(i);
        QVector<double> pos_x; QVector<double> pos_y; QVector<double> pos_z;
        QVector<double> timesteps_mov;
        vector<vector<double>> tsteps_mov = timesteps.at(i);
        for(size_t j=0; j<hand_position_mov.size();++j){
            vector<vector<double>> hand_position_stage = hand_position_mov.at(j);
            vector<double> tsteps_stage = tsteps_mov.at(j);
            for(size_t h=0; h<hand_position_stage.size();++h){
                vector<double> hand_point = hand_position_stage.at(h);
                timesteps_mov.push_back(tsteps_stage.at(h));
                pos_x.push_back(hand_point.at(0)/1000); // [m]
                pos_y.push_back(hand_point.at(1)/1000); // [m]
                pos_z.push_back(hand_point.at(2)/1000); // [m]
            }//stage
        }// mov

        // first derivatives
        QVector<double> der_pos_x_1; QVector<double> der_pos_y_1; QVector<double> der_pos_z_1;
        this->getDerivative(pos_x,timesteps_mov,der_pos_x_1);
        this->getDerivative(pos_y,timesteps_mov,der_pos_y_1);
        this->getDerivative(pos_z,timesteps_mov,der_pos_z_1);
        // second derivatives
        QVector<double> der_pos_x_2; QVector<double> der_pos_y_2; QVector<double> der_pos_z_2;
        this->getDerivative(der_pos_x_1,timesteps_mov,der_pos_x_2);
        this->getDerivative(der_pos_y_1,timesteps_mov,der_pos_y_2);
        this->getDerivative(der_pos_z_1,timesteps_mov,der_pos_z_2);
        // third derivatives
        QVector<double> der_pos_x_3; QVector<double> der_pos_y_3; QVector<double> der_pos_z_3;
        this->getDerivative(der_pos_x_2,timesteps_mov,der_pos_x_3);
        this->getDerivative(der_pos_y_2,timesteps_mov,der_pos_y_3);
        this->getDerivative(der_pos_z_2,timesteps_mov,der_pos_z_3);

        // --- Velocity --- //
        vector<double> vel_mov; // velocity of the movement
        for(int i=0; i<der_pos_x_1.size();++i){
            Vector3d der_1(der_pos_x_1.at(i),der_pos_y_1.at(i),der_pos_z_1.at(i));
            vel.push_back(der_1.norm());
            vel_mov.push_back(der_1.norm());
        }
        this->vel_task.push_back(vel_mov);

        // --- Acceleration --- //
        vector<double> acc_mov; // acceleration of the movement
        for(int i=0; i<der_pos_x_2.size();++i){
            Vector3d der_2(der_pos_x_2.at(i),der_pos_y_2.at(i),der_pos_z_2.at(i));
            acc.push_back(der_2.norm());
            acc_mov.push_back(der_2.norm());
        }
        this->acc_task.push_back(acc_mov);

        // --- Curvature --- //
        vector<double> K_mov; // curvature of the movement
        //double curv_th = 10; // curvature threshold
        for(int i=0; i<der_pos_x_1.size();++i){
            Vector3d der_1(der_pos_x_1.at(i),der_pos_y_1.at(i),der_pos_z_1.at(i));
            Vector3d der_2(der_pos_x_2.at(i),der_pos_y_2.at(i),der_pos_z_2.at(i));
            Vector3d cross = der_1.cross(der_2);
            double num = cross.norm();
            double den = pow(der_1.norm(),3);
            if(den==0)
                den=0.0001;
            double curv = ((double)num)/den;
            //if(curv>=curv_th){
              //  K.push_back(curv_th);
            //}else{
            K.push_back(curv);
            K_mov.push_back(curv);
            //}
        }
        this->K_task.push_back(K_mov);

        // --- Torsion --- //
        vector<double> T_mov; // torsion of the movement
        for(int i=0; i<der_pos_x_1.size();++i){
            Vector3d der_1(der_pos_x_1.at(i),der_pos_y_1.at(i),der_pos_z_1.at(i));
            Vector3d der_2(der_pos_x_2.at(i),der_pos_y_2.at(i),der_pos_z_2.at(i));
            Vector3d der_3(der_pos_x_3.at(i),der_pos_y_3.at(i),der_pos_z_3.at(i));
            Vector3d cross = der_2.cross(der_3);
            double num = der_1.dot(cross);
            Vector3d cross_1 = der_1.cross(der_2);
            double den = pow(cross_1.norm(),2);
            if(den==0)
                den=0.0001;
            T.push_back((double)num/den);
            T_mov.push_back((double)num/den);
        }
        this->T_task.push_back(T_mov);

        // --- Curvature and Torsion , Velocity --- //
        QVector<double> ln_vel_mov; QVector<double> ln_x_mov;
        //QVector<int> index_t(index.size()); int k=0; int h=0; int hh;
        for(int i=0; i<der_pos_x_1.size();++i){
            if((abs(T_mov.at(i))>=2) && (K_mov.at(i)>=0.0001)){// threshold value taken into account to eliminate the torsion cups and planar regions
                ln_x_mov.push_back(log(pow(K_mov.at(i),2)*abs(T_mov.at(i))));
                ln_vel_mov.push_back(log(vel_mov.at(i)));
            }
        }

        if(ln_x_mov.size()>1){
            // R-squared linear regression
            double q,m,r;
            this->linreg(ln_x_mov,ln_vel_mov,&q,&m,&r);
            if(ln_x_mov.size()!=0)
                n_coeff++;
            double coeff = ((double)ln_x_mov.size())/T_mov.size();
            q_tot.push_back(q); m_tot.push_back(m); r_tot.push_back(r);
            q_tot_w.push_back(coeff*q); m_tot_w.push_back(coeff*m); r_tot_w.push_back(coeff*r);
            q_tot_2w.push_back(coeff*pow(q,2)); m_tot_2w.push_back(coeff*pow(m,2)); r_tot_2w.push_back(coeff*pow(r,2));

            for(int i=0; i<ln_x_mov.size();++i){
                ln_x_tot.push_back(ln_x_mov.at(i));
                ln_vel_tot.push_back(ln_vel_mov.at(i));
            }
            coeff_tot +=coeff;

            this->slopes.push_back(m);
            this->r_squared.push_back(r);
            this->n_points.push_back(ln_x_mov.size());

        }else{
            // no relevant data
            this->slopes.push_back(9999);
            this->r_squared.push_back(9999);
            this->n_points.push_back(9999);
        }

    }// task

    double norm;
    if(n_coeff==1){
        norm=1;
    }else{
        norm=((double)n_coeff)/(n_coeff-1);
    }

    // q
    double sum_q = std::accumulate(q_tot_w.begin(), q_tot_w.end(), 0.0);
    //double mean_q = ((double)sum_q) / q_tot.size();
    double mean_q = ((double)sum_q) / coeff_tot;
    double sq_sum_q = std::accumulate(q_tot_2w.begin(), q_tot_2w.end(), 0.0);
    double stdev_q = std::sqrt(norm*((((double)sq_sum_q) / coeff_tot) - pow(mean_q,2)));
    // m
    double sum_m = std::accumulate(m_tot_w.begin(), m_tot_w.end(), 0.0);
    //double mean_m = ((double)sum_m) / m_tot.size();
    double mean_m = ((double)sum_m) / coeff_tot;
    double sq_sum_m = std::accumulate(m_tot_2w.begin(), m_tot_2w.end(), 0.0);
    double stdev_m = std::sqrt(norm*((((double)sq_sum_m) / coeff_tot) - pow(mean_m,2)));
    // r
    double sum_r = std::accumulate(r_tot_w.begin(), r_tot_w.end(), 0.0);
    //double mean_r = ((double)sum_r) / r_tot.size();
    double mean_r = ((double)sum_r) / coeff_tot;
    double sq_sum_r = std::accumulate(r_tot_2w.begin(), r_tot_2w.end(), 0.0);
    double stdev_r = std::sqrt(norm*((((double)sq_sum_r) / coeff_tot) - pow(mean_r,2)));

    // total linear regression
    double q_tot_all,m_tot_all,r_tot_all;
    this->linreg(ln_x_tot,ln_vel_tot,&q_tot_all,&m_tot_all,&r_tot_all);
    QVector<double> ln_vel_tot_fit_tot; QVector<double> ln_vel_tot_fit; QVector<double> best_line;
    double m_best = ((double)-1)/6;
    for(int i=0; i < ln_x_tot.size(); ++i){
        ln_vel_tot_fit.push_back(mean_m*ln_x_tot.at(i)+mean_q);
        ln_vel_tot_fit_tot.push_back(m_tot_all*ln_x_tot.at(i)+q_tot_all);
        best_line.push_back(m_best*ln_x_tot.at(i)+mean_q);
    }




    // plot the curvature
    ui->plot_curvature->plotLayout()->clear();
    ui->plot_curvature->clearGraphs();
    ui->plot_curvature->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_curvature);
    wideAxisRect->setupFullAxesBox(true);
    QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_curvature);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_curvature->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    QString title = "Curvature";
    ui->plot_curvature->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_curvature,title));
    ui->plot_curvature->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_curvature->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_curvature->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_curvature->graph(0)->setName(title);
    ui->plot_curvature->graph(0)->valueAxis()->setLabel("curvature [1/m]");
    ui->plot_curvature->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_curvature->graph(0)->setData(qtime, K);
    ui->plot_curvature->graph(0)->valueAxis()->setRange(*std::min_element(K.begin(), K.end()),
                                                       *std::max_element(K.begin(), K.end()));
    ui->plot_curvature->graph(0)->rescaleAxes();
    ui->plot_curvature->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_curvature->replot();

    // plot the velocity
    ui->plot_vel->plotLayout()->clear();
    ui->plot_vel->clearGraphs();
    ui->plot_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_vel);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_vel);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_vel->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Velocity";
    ui->plot_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_vel,title));
    ui->plot_vel->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_vel->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_vel->graph(0)->setName(title);
    ui->plot_vel->graph(0)->valueAxis()->setLabel("velocity [m/s]");
    ui->plot_vel->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_vel->graph(0)->setData(qtime, vel);
    ui->plot_vel->graph(0)->valueAxis()->setRange(*std::min_element(vel.begin(), vel.end()),*std::max_element(vel.begin(), vel.end()));
    ui->plot_vel->graph(0)->rescaleAxes();


    ui->plot_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_vel->replot();

    // plot the acceleration
    ui->plot_acc->plotLayout()->clear();
    ui->plot_acc->clearGraphs();
    ui->plot_acc->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_acc);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_acc);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_acc->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Acceleration";
    ui->plot_acc->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_acc,title));
    ui->plot_acc->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_acc->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_acc->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_acc->graph(0)->setName(title);
    ui->plot_acc->graph(0)->valueAxis()->setLabel("acceleration [m/s^2]");
    ui->plot_acc->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_acc->graph(0)->setData(qtime, acc);

    ui->plot_acc->graph(0)->valueAxis()->setRange(*std::min_element(acc.begin(), acc.end()),
                                                  *std::max_element(acc.begin(), acc.end()));
    ui->plot_acc->graph(0)->rescaleAxes();


    ui->plot_acc->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_acc->replot();

    // plot the torsion
    ui->plot_torsion->plotLayout()->clear();
    ui->plot_torsion->clearGraphs();
    ui->plot_torsion->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_torsion);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_torsion);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_torsion->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Torsion";
    ui->plot_torsion->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_torsion,title));
    ui->plot_torsion->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_torsion->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_torsion->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_torsion->graph(0)->setName(title);
    ui->plot_torsion->graph(0)->valueAxis()->setLabel("torsion [1/m]");
    ui->plot_torsion->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_torsion->graph(0)->setData(qtime, T);
    ui->plot_torsion->graph(0)->valueAxis()->setRange(*std::min_element(T.begin(), T.end()),
                                                      *std::max_element(T.begin(), T.end()));
    ui->plot_torsion->graph(0)->rescaleAxes();
    ui->plot_torsion->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_torsion->replot();


    // plot power law
    ui->plot_16->plotLayout()->clear();
    ui->plot_16->clearGraphs();
    ui->plot_16->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_16);
    wideAxisRect = new QCPAxisRect(ui->plot_16);
    wideAxisRect->setupFullAxesBox(true);
    //QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_16);
    marginGroup = new QCPMarginGroup(ui->plot_16);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_16->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    //QString title = "One-sixth power law";
    title = "One-sixth power law";
    ui->plot_16->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_16,title));
    ui->plot_16->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_16->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_16->graph(0)->setPen(QPen(Qt::black));
    ui->plot_16->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot_16->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->plot_16->graph(0)->setName("ln(V)/ln(C^2|T|)");
    ui->plot_16->graph(0)->valueAxis()->setLabel("ln(V) [m/s]");
    ui->plot_16->graph(0)->keyAxis()->setLabel("ln(C^2|T|) [m^-3]");
    ui->plot_16->graph(0)->setData(ln_x_tot, ln_vel_tot);
    ui->plot_16->graph(0)->valueAxis()->setRange(*std::min_element(ln_vel_tot.begin(), ln_vel_tot.end()),
                                                 *std::max_element(ln_vel_tot.begin(), ln_vel_tot.end()));
    ui->plot_16->graph(0)->rescaleAxes();


    ui->plot_16->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_16->graph(1)->setPen(QPen(Qt::red));
    string mean_m_str =  boost::str(boost::format("%.2f") % (mean_m)); boost::replace_all(mean_m_str,",",".");
    string mean_r_str =  boost::str(boost::format("%.2f") % (mean_r)); boost::replace_all(mean_r_str,",",".");
    string stdev_m_str =  boost::str(boost::format("%.2f") % (stdev_m)); boost::replace_all(stdev_m_str,",",".");
    string stdev_r_str =  boost::str(boost::format("%.2f") % (stdev_r)); boost::replace_all(stdev_r_str,",",".");
    QString name = QString::fromStdString(string("slope=")+mean_m_str+string("(")+stdev_m_str+string(")")+string(" R^2=")+mean_r_str+string("(")+stdev_r_str+string(")"));
    ui->plot_16->graph(1)->setName(name);
    ui->plot_16->graph(1)->setData(ln_x_tot, ln_vel_tot_fit);
    ui->plot_16->graph(1)->rescaleAxes();

    ui->plot_16->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_16->graph(2)->setPen(QPen(Qt::darkGreen));
    string m_str_tot =  boost::str(boost::format("%.2f") % (m_tot_all)); boost::replace_all(m_str_tot,",",".");
    string r_str_tot =  boost::str(boost::format("%.2f") % (r_tot_all)); boost::replace_all(r_str_tot,",",".");
    QString name_tot = QString::fromStdString(string("slope=")+m_str_tot+string(" R^2=")+r_str_tot);
    ui->plot_16->graph(2)->setName(name_tot);
    ui->plot_16->graph(2)->setData(ln_x_tot, ln_vel_tot_fit_tot);
    ui->plot_16->graph(2)->rescaleAxes();

    ui->plot_16->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_16->graph(3)->setPen(QPen(Qt::blue));
    ui->plot_16->graph(3)->setName(QString("best fit slope: ")+QString::number(m_best,'f',2));
    ui->plot_16->graph(3)->setData(ln_x_tot, best_line);
    ui->plot_16->graph(3)->rescaleAxes();


    // legend
    QCPLegend *legend = new QCPLegend();
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    ui->plot_16->plotLayout()->addElement(2, 0, subLayout);
    subLayout->setMargins(QMargins(5, 0, 5, 5));
    subLayout->addElement(0, 0, legend);
    // set legend's row stretch factor very small so it ends up with minimum height:
    ui->plot_16->plotLayout()->setRowStretchFactor(2, 0.001);
    legend->setLayer("legend");
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    legend->setFont(legendFont);
    legend->addElement(0,0,new QCPPlottableLegendItem(legend,ui->plot_16->graph(0)));
    legend->addElement(0,1,new QCPPlottableLegendItem(legend,ui->plot_16->graph(1)));
    legend->addElement(0,2,new QCPPlottableLegendItem(legend,ui->plot_16->graph(2)));
    legend->addElement(0,3,new QCPPlottableLegendItem(legend,ui->plot_16->graph(3)));


    ui->plot_16->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_16->replot();



}



void PowerLaw3DDialog::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
{
    // Formula of the numarical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


       const double MIN_STEP_VALUE = 0.1;
       const double MIN_DER_VALUE = 0.001;

       int h = 1;
       int tnsample;
       double f0;
       double f1;
       double f2;
       double f3;
       double f4;
       double step_value;

       // 1st point
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       tnsample = 0;
       f0 = function.at(tnsample);
       f1 = function.at(tnsample+1);
       f2 = function.at(tnsample+2);
       f3 = function.at(tnsample+3);
       f4 = function.at(tnsample+4);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));
       }

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = function.at(tnsample-1);
       f1 = function.at(tnsample);
       f2 = function.at(tnsample+1);
       f3 = function.at(tnsample+2);
       f4 = function.at(tnsample+3);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));
       }

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< function.size() -2;++i){     // centered
           f0 = function.at(i-2);
           f1 = function.at(i-1);
           f2 = function.at(i);
           f3 = function.at(i+1);
           f4 = function.at(i+2);
           step_value = step_values.at(i);
           if(step_value==0){
               //step_value=MIN_STEP_VALUE;
               derFunction.push_back(MIN_DER_VALUE);
           }else{
               derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
           }
       }

       // 4th point
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       tnsample = function.size()-2;
       f0 = function.at(tnsample-3);
       f1 = function.at(tnsample-2);
       f2 = function.at(tnsample-1);
       f3 = function.at(tnsample);
       f4 = function.at(tnsample+1);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));
       }


       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = function.size()-1;
       f0 = function.at(tnsample-4);
       f1 = function.at(tnsample-3);
       f2 = function.at(tnsample-2);
       f3 = function.at(tnsample-1);
       f4 = function.at(tnsample);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));
       }

}

int PowerLaw3DDialog::linreg(const QVector<double> &x, const QVector<double> &y, double *b, double *m, double *r)
{
    double mm,bb,rr;

    double   sumx = 0.0;                        /* sum of x                      */
    double   sumx2 = 0.0;                       /* sum of x^2                   */
    double   sumxy = 0.0;                       /* sum of x * y                  */
    double   sumy = 0.0;                        /* sum of y                      */
    //double   sumy2 = 0.0;                       /* sum of y^2                   */

    int n = x.size();

   for (int i=0; i<n ;++i)
    {
      sumx  += x.at(i);
      sumx2 += pow((x.at(i)),2);
      sumxy += x.at(i) * y.at(i);
      sumy  += y.at(i);
      //sumy2 += pow(y.at(i),2);
    }

   double   meanx = ((double)sumx)/n;                        /* mean of x                      */
   double   meanx2 = ((double)sumx2)/n;                       /* mean of x^2                   */
   double   meanxy = ((double)sumxy)/n;                       /* mean of x * y                  */
   double   meany = ((double)sumy)/n;                        /* mean of y                      */
   //double   meany2 = sumy2/n;                       /* mean of y^2                   */

   double denom = (pow(meanx,2) - meanx2);
   if (denom == 0) {
       // can't solve the problem.
       *m = 0;
       *b = 0;
       *r = 0;
       return 1;
   }

   mm = ((double)((meanx*meany) - meanxy)) / denom;
   bb = meany - mm*meanx;

   /* compute correlation coeff     */
   double se_y = 0.0; /* squared error in y or total variation in y */
   double se_line = 0.0; /* squared error of the fitted line */
   for(int i=0; i<n; ++i)
   {
       se_y += pow((y.at(i) - meany),2);
       se_line += pow((y.at(i)-(mm*x.at(i)+bb)),2);
   }
   rr = 1 - (((double)se_line)/se_y);

   // results
   *m = mm;
   *b = bb;
   *r = rr;

   return 0;
}

// Q_SLOTS

void PowerLaw3DDialog::on_pushButton_save_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/power_law_3D", &st) == -1) {
        mkdir("results/planning/power_law_3D", 0700);
    }
    QString path("results/planning/power_law_3D/");

    ui->plot_curvature->savePdf(path+QString("curvature.pdf"),true,0,0,QString(),QString("Curvature"));
    ui->plot_vel->savePdf(path+QString("velocity.pdf"),true,0,0,QString(),QString("Velocity"));
    ui->plot_acc->savePdf(path+QString("acceleration.pdf"),true,0,0,QString(),QString("Acceleration"));
    ui->plot_torsion->savePdf(path+QString("torsion.pdf"),true,0,0,QString(),QString("Torsion"));
    ui->plot_16->savePdf(path+QString("power_law_3D.pdf"),true,0,0,QString(),QString("3D Power law"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;


    pdf_qstr = path+QString("curvature.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("curvature.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("velocity.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("velocity.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("acceleration.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("acceleration.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("torsion.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("torsion.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("power_law_3D.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("power_law_3D.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    // hand velocity, acceleration, curvature, torsion csv
    string hand_filename_csv("hand.csv");
    ofstream hand_csv;
    hand_csv.open(path.toStdString()+hand_filename_csv);
    hand_csv << "TRAJ,VELOCITY,ACCELERATION,CURVATURE,TORSION \n";
    for(size_t i=0;i<this->vel_task.size();++i){
        std::string traj_str = QString::number(i+1).toStdString();
        std::vector<double> vel_mov = this->vel_task.at(i);
        std::vector<double> acc_mov = this->acc_task.at(i);
        std::vector<double> K_mov = this->K_task.at(i);
        std::vector<double> T_mov = this->T_task.at(i);
        for(size_t j=0;j<vel_mov.size();++j){
            string vel_str =  boost::str(boost::format("%.8f") % (vel_mov.at(j)));
            string acc_str =  boost::str(boost::format("%.8f") % (acc_mov.at(j)));
            string k_str =  boost::str(boost::format("%.8f") % (K_mov.at(j)));
            string t_str =  boost::str(boost::format("%.8f") % (T_mov.at(j)));
            boost::replace_all(vel_str,",","."); boost::replace_all(acc_str,",","."); boost::replace_all(k_str,",","."); boost::replace_all(t_str,",",".");
            hand_csv << traj_str+","+vel_str+","+acc_str+","+k_str+","+t_str+" \n";
        }
    }
    hand_csv.close();

    // results csv
    string filename_csv("results.csv");
    ofstream results_csv;
    results_csv.open(path.toStdString()+filename_csv);
    results_csv << "TRAJ,SLOPES,R^2,NUMBER OF POINTS \n";
    for(size_t i=0;i<this->slopes.size();++i){
        string slope_str =  boost::str(boost::format("%.8f") % (this->slopes.at(i)));
        string r_str =  boost::str(boost::format("%.8f") % (this->r_squared.at(i)));
        string points_str =  boost::str(boost::format("%.8f") % (this->n_points.at(i)));
        boost::replace_all(slope_str,",","."); boost::replace_all(r_str,",","."); boost::replace_all(points_str,",",".");
        results_csv << QString::number(i+1).toStdString()+","+slope_str+","+r_str+","+points_str+" \n";
    }
    results_csv.close();


}


} // namespace motion_manager
