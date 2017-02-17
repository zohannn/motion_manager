#include "../include/motion_manager/power_law_dialog.hpp"

namespace motion_manager {

PowerLawDialog::PowerLawDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::powerLawDialog)
{
    ui->setupUi(this);
}

PowerLawDialog::~PowerLawDialog()
{
    delete ui;
}



void PowerLawDialog::setupPlots(vector<vector<double> > &hand_position, vector<vector<vector<double> > > &timesteps)
{
    // time
    vector<double> time_task; QVector<double> tot_timesteps; vector<int> index;
    for(size_t i=0; i<timesteps.size();++i){
        vector<vector<double>> tsteps_mov = timesteps.at(i);
        double time_init;
        if(time_task.empty()){
            time_init=0.0;
        }else{
            time_init=time_task.at(time_task.size()-1);
        }
        vector<double> time_mov;
        for(size_t j=0; j<tsteps_mov.size();++j){
            vector<double> tsteps_stage = tsteps_mov.at(j);
            vector<double> time_stage(tsteps_stage.size());
            time_stage.at(0) = time_init;
            for(size_t k=0;k<tsteps_stage.size();++k){
                tot_timesteps.push_back(tsteps_stage.at(k));
                if(k>0){time_stage.at(k) = time_stage.at(k-1) + tsteps_stage.at(k-1);}
            }// stage
            index.push_back(tot_timesteps.size());
            time_mov.reserve(time_stage.size());
            std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_mov));
        }// mov
        time_task.reserve(time_mov.size());
        std::copy (time_mov.begin(), time_mov.end(), std::back_inserter(time_task));
    }// task
    QVector<double> qtime = QVector<double>::fromStdVector(time_task);


    // PCA of the hand position
    vector<vector<double>> hand_position_red;
    int pca_hand = this->doPCA(hand_position,hand_position_red);
    if (pca_hand!=-1){
        // PCA successful
        if((!hand_position_red.empty())&&(hand_position_red.at(0).size()==2)){
            // dimensionality of the hand position reduced to 2 (movement on a plane)
            QVector<double> pos_u; QVector<double> pos_v;
            for(size_t i=0;i<hand_position_red.size();++i){
                vector<double> row = hand_position_red.at(i);
                pos_u.push_back(row.at(0)); // [mm]
                pos_v.push_back(row.at(1)); // [mm]
            }
            /*
            // ecc ellipse
            double ecc = 0.85;  double pos_u_0 = -400;
            double v_max = *std::max_element(pos_v.begin(), pos_v.end());
            double v_min = *std::min_element(pos_v.begin(), pos_v.end());
            double pos_v_0 = (v_max -v_min)/2;
            double ar = v_max - pos_v_0; double br=ar*sqrt(1-pow(ecc,2));
            for(size_t i=0; i<pos_v.size();++i){
                pos_u.push_back(pos_u_0+br*sqrt(1-pow(((pos_v.at(i)-pos_v_0)/ar),2)));
            }
            */

            // plot hand position
            ui->plot_hand_pos->plotLayout()->clear();
            ui->plot_hand_pos->clearGraphs();
            ui->plot_hand_pos->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_hand_pos);
            wideAxisRect->setupFullAxesBox(true);
            QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_hand_pos);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_hand_pos->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            QString title("Hand position");
            ui->plot_hand_pos->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_hand_pos,title));
            ui->plot_hand_pos->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_hand_pos->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_hand_pos->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_hand_pos->graph(0)->setLineStyle(QCPGraph::lsNone);
            ui->plot_hand_pos->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
            ui->plot_hand_pos->graph(0)->setName(title);
            ui->plot_hand_pos->graph(0)->valueAxis()->setLabel("u [mm]");
            ui->plot_hand_pos->graph(0)->keyAxis()->setLabel("v [mm]");
            ui->plot_hand_pos->graph(0)->setData(pos_v, pos_u);
            ui->plot_hand_pos->graph(0)->valueAxis()->setRange(*std::min_element(pos_u.begin(), pos_u.end()),
                                                              *std::max_element(pos_u.begin(), pos_u.end()));
            ui->plot_hand_pos->graph(0)->rescaleAxes();
            ui->plot_hand_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_hand_pos->replot();

            // first derivatives
            QVector<double> der_pos_u_1; QVector<double> der_pos_v_1;
            this->getDerivative(pos_u,tot_timesteps,der_pos_u_1);
            this->getDerivative(pos_v,tot_timesteps,der_pos_v_1);
            // second derivatives
            QVector<double> der_pos_u_2; QVector<double> der_pos_v_2;
            this->getDerivative(der_pos_u_1,tot_timesteps,der_pos_u_2);
            this->getDerivative(der_pos_v_1,tot_timesteps,der_pos_v_2);
            // third derivatives
            QVector<double> der_pos_u_3; QVector<double> der_pos_v_3;
            this->getDerivative(der_pos_u_2,tot_timesteps,der_pos_u_3);
            this->getDerivative(der_pos_v_2,tot_timesteps,der_pos_v_3);


            // --- Curvature and Tangential Velocity --- //
            QVector<double> C; // Curvature
            QVector<double> lnC; // Curvature
            QVector<double> R; // Curvature radius
            QVector<double> lnR; // Curvature radius
            QVector<double> vel_tan; // Velocity
            QVector<double> ln_vel_tan; // Velocity
            for(int i=0; i<der_pos_u_1.size();++i){
                Vector2d der_1(der_pos_u_1.at(i),der_pos_v_1.at(i));
                Vector2d der_2(der_pos_u_2.at(i),der_pos_v_2.at(i));
                double num = abs((der_1(0)*der_2(1))-(der_1(1)*der_2(0)));
                double den = pow(der_1.norm(),3);
                C.push_back((double)num/den); // [m^⁻1]
                lnC.push_back(log(C.at(i)));                
                R.push_back(((double)1)/C.at(i)); // [m]
                lnR.push_back(log(R.at(i)));
                vel_tan.push_back(((double)sqrt(pow(der_1(0),2)+pow(der_1(1),2)))/1000); // [m/s]
                ln_vel_tan.push_back(log(vel_tan.at(i)));
            }

            QVector<double> lnX; // Curvature^2
            QVector<double> lnY; // Velocity
            for(size_t i=0;  i<C.size();++i){
                   lnX.push_back(log(pow(C.at(i),2)));
                   lnY.push_back(ln_vel_tan.at(i));
            }

            /*
            QVector<double> C;// curvature
            QVector<double> lnC;
            QVector<double> R;// curvature radius
            QVector<double> lnR; //QVector<double> lnHand_vel;
            double eexp = ((double)3)/2; double num; double den;
            for(size_t i=0; i<pos_u.size();++i){
                num = pow(pow(der_pos_u_1.at(i),2)+pow(der_pos_v_1.at(i),2),eexp);
                den = 1000*abs((der_pos_u_1.at(i)*der_pos_v_2.at(i))-(der_pos_v_1.at(i)*der_pos_u_2.at(i)));
                R.push_back(((double)num)/den); // [m]
                C.push_back(((double)1)/R.at(i)); // [m^-1]
            }
            */

/*
            // angular and tangential velocity
            QVector<double> theta; QVector<double> der_theta; QVector<double> ln_theta;
            for(size_t i=0; i< pos_u.size(); ++i){
                theta.push_back(std::atan2(pos_v.at(i),pos_u.at(i)));
            }
            this->getDerivative(theta,tot_timesteps,der_theta);

            for(size_t i=0; i<der_theta.size();++i){
                vel_tan.push_back(R.at(i)*der_theta.at(i)); // [m/s]
                //vel_tan.push_back(((double)sqrt(pow(der_pos_u_1.at(i),2)+pow(der_pos_v_1.at(i),2)))/1000);// [m/s]
                //if(abs(der_theta.at(i))>0.05){
                    //ln_theta.push_back(abs(der_theta.at(i)));
                    //lnC.push_back(log(C.at(i)));
                ln_vel_tan.push_back(abs(vel_tan.at(i)));
                    //lnR.push_back(log(R.at(i)));
                //}
            }
            */


/*
            QVector<double> ln_vel_tan_mean; QVector<double> lnR_mean;
            for(size_t i=0; i<index.size();++i){
                if(i==0){
                    lnR_mean.push_back((double)accumulate( lnR.begin(), lnR.begin()+index.at(i), 0.0)/index.at(i));
                    ln_vel_tan_mean.push_back((double)accumulate( ln_vel_tan.begin(), ln_vel_tan.begin()+index.at(i), 0.0)/index.at(i));
                }else{
                    lnR_mean.push_back((double)accumulate( lnR.begin()+index.at(i-1), lnR.begin()+index.at(i), 0.0)/(index.at(i)-index.at(i-1)));
                    ln_vel_tan_mean.push_back((double)accumulate( ln_vel_tan.begin()+index.at(i-1), ln_vel_tan.begin()+index.at(i), 0.0)/(index.at(i)-index.at(i-1)));
                }
            }
            */


            // plot the curvature
            ui->plot_curvature->plotLayout()->clear();
            ui->plot_curvature->clearGraphs();
            ui->plot_curvature->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_curvature);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_curvature);
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
            title = "Curvature";
            ui->plot_curvature->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_curvature,title));
            ui->plot_curvature->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_curvature->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_curvature->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_curvature->graph(0)->setName(title);
            ui->plot_curvature->graph(0)->valueAxis()->setLabel("curvature [1/m]");
            ui->plot_curvature->graph(0)->keyAxis()->setLabel("time [s]");
            ui->plot_curvature->graph(0)->setData(qtime, C);
            ui->plot_curvature->graph(0)->valueAxis()->setRange(*std::min_element(C.begin(), C.end()),
                                                               *std::max_element(C.begin(), C.end()));
            ui->plot_curvature->graph(0)->rescaleAxes();


            ui->plot_curvature->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_curvature->replot();

            // plot the velocity
            ui->plot_ang_vel->plotLayout()->clear();
            ui->plot_ang_vel->clearGraphs();
            ui->plot_ang_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_ang_vel);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_ang_vel);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_ang_vel->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            title = "Velocity";
            ui->plot_ang_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_ang_vel,title));
            ui->plot_ang_vel->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_ang_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_ang_vel->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_ang_vel->graph(0)->setName(title);
            ui->plot_ang_vel->graph(0)->valueAxis()->setLabel("velocity [m/s]");
            ui->plot_ang_vel->graph(0)->keyAxis()->setLabel("time [s]");
            ui->plot_ang_vel->graph(0)->setData(qtime, vel_tan);

            ui->plot_ang_vel->graph(0)->valueAxis()->setRange(*std::min_element(vel_tan.begin(), vel_tan.end()),
                                                               *std::max_element(vel_tan.begin(), vel_tan.end()));
            ui->plot_ang_vel->graph(0)->rescaleAxes();


            ui->plot_ang_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_ang_vel->replot();

            // R-squared regression
            double q,m,r;
            //this->linreg(lnC,ln_vel_tan,&q,&m,&r);
            this->linreg(lnX,ln_vel_tan,&q,&m,&r);
            //this->linreg(lnR_mean,ln_vel_tan_mean,&q,&m,&r);
            //this->linreg(lnR,ln_vel_tan,&q,&m,&r);

            std::cout << " m = " << m << " q = " << q << " R^2 = " << r << endl;
            QVector<double> ln_vel_fit; QVector<double> best_line;
            double m_best = ((double)-1)/6;
            //double m_best = ((double)2)/3;
            for(size_t i=0; i < lnC.size(); ++i){
            //for(size_t i=0; i < lnR_mean.size(); ++i){
                ln_vel_fit.push_back(m*lnC.at(i)+q);
                //ln_vel_fit.push_back(m*lnR_mean.at(i)+q);
                best_line.push_back(m_best*lnC.at(i)+q);
                //best_line.push_back(m_best*lnR_mean.at(i)+q);
            }

            // plot power law
            ui->plot_23->plotLayout()->clear();
            ui->plot_23->clearGraphs();
            ui->plot_23->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_23);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_23);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_23->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            title = "Two-third power law";
            ui->plot_23->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_23,title));
            ui->plot_23->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(0)->setPen(QPen(Qt::black));
            ui->plot_23->graph(0)->setLineStyle(QCPGraph::lsNone);
            ui->plot_23->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
            ui->plot_23->graph(0)->setName("ln(V)/ln(C^2)");
            ui->plot_23->graph(0)->valueAxis()->setLabel("ln(V) [m/s]");
            ui->plot_23->graph(0)->keyAxis()->setLabel("ln(C^2) [m^-2]");
            ui->plot_23->graph(0)->setData(lnX, lnY);
            //ui->plot_23->graph(0)->setData(lnR_mean, ln_vel_tan_mean);
            ui->plot_23->graph(0)->valueAxis()->setRange(*std::min_element(lnY.begin(), lnY.end()),
                                                         *std::max_element(lnY.begin(), lnY.end()));
            //ui->plot_23->graph(0)->valueAxis()->setRange(*std::min_element(ln_vel_tan_mean.begin(), ln_vel_tan_mean.end()),
              //                                           *std::max_element(ln_vel_tan_mean.begin(), ln_vel_tan_mean.end()));
            ui->plot_23->graph(0)->rescaleAxes();


            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(1)->setPen(QPen(Qt::red));
            string m_str =  boost::str(boost::format("%.2f") % (m)); boost::replace_all(m_str,",",".");
            string r_str =  boost::str(boost::format("%.2f") % (r)); boost::replace_all(r_str,",",".");
            QString name = QString::fromStdString(string("slope=")+m_str+string(" R^2=")+r_str);
            ui->plot_23->graph(1)->setName(name);

            //ui->plot_23->graph(1)->setData(lnR_mean, ln_vel_fit);
            ui->plot_23->graph(1)->setData(lnC, ln_vel_fit);
            //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                             // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
            ui->plot_23->graph(1)->rescaleAxes();

            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(2)->setPen(QPen(Qt::blue));
            ui->plot_23->graph(2)->setName(QString("best fit slope: ")+QString::number(m_best));

            //ui->plot_23->graph(2)->setData(lnR_mean, best_line);
            ui->plot_23->graph(2)->setData(lnC, best_line);
            //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                             // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
            ui->plot_23->graph(2)->rescaleAxes();

            // legend
            QCPLegend *legend = new QCPLegend();
            QCPLayoutGrid *subLayout = new QCPLayoutGrid;
            ui->plot_23->plotLayout()->addElement(2, 0, subLayout);
            subLayout->setMargins(QMargins(5, 0, 5, 5));
            subLayout->addElement(0, 0, legend);
            // set legend's row stretch factor very small so it ends up with minimum height:
            ui->plot_23->plotLayout()->setRowStretchFactor(2, 0.001);
            legend->setLayer("legend");
            QFont legendFont = font();  // start out with MainWindow's font..
            legendFont.setPointSize(9); // and make a bit smaller for legend
            legend->setFont(legendFont);
            legend->addElement(0,0,new QCPPlottableLegendItem(legend,ui->plot_23->graph(0)));
            legend->addElement(0,1,new QCPPlottableLegendItem(legend,ui->plot_23->graph(1)));
            legend->addElement(0,2,new QCPPlottableLegendItem(legend,ui->plot_23->graph(2)));


            ui->plot_23->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_23->replot();


        }else{
            ui->plot_hand_pos->plotLayout()->clear();
            ui->plot_hand_pos->clearGraphs();
            ui->plot_23->plotLayout()->clear();
            ui->plot_23->clearGraphs();
            ui->plot_ang_vel->plotLayout()->clear();
            ui->plot_ang_vel->clearGraphs();
            ui->plot_curvature->plotLayout()->clear();
            ui->plot_curvature->clearGraphs();
        }
    }


}




int PowerLawDialog::doPCA(vector<vector<double> > &data, vector<vector<double> > &data_red)
{
    MatrixXd mat(data.size(),data.at(0).size());
    for (size_t i = 0; i < data.size(); ++i) {
        vector<double> row = data.at(i);
      for (size_t j = 0; j < row.size(); ++j) {
          mat(i,j)=row.at(j);
      }
    }
    // Principal component analisys
    Pca *pca = new Pca();
    int init_result = pca->Calculate(mat,true,true,true);
    if (0 != init_result) {
      //There is an error during PCA calculation!
      return -1;
    }
    vector<double> sd = pca->sd(),
                  prop_of_var = pca->prop_of_var(),
                  cum_prop = pca->cum_prop(),
                  scores = pca->scores();
    vector<unsigned int> el_cols = pca->eliminated_columns();
    double         kaiser = pca->kaiser(),
                  thresh95 = pca->thresh95(),
                  thresh99 = pca->thresh99();
    unsigned int
                  ncols = pca->ncols(),
                  nrows = pca->nrows();
    string method = pca->method();
    delete pca;

    // Save the result to text file
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/pca", &st) == -1) {
        mkdir("results/planning/pca", 0700);
    }
    QString path("results/planning/pca/");
    QString fname = path+QString("pca_out.txt");

    ofstream outfile(fname.toStdString());
    if (!outfile) {
      cerr << "Can't create output file!" << endl;
      return -1;
    }
    outfile << "Initial matrix: " << endl;
    for (unsigned int i = 0; i < mat.rows(); ++i) {
      for (unsigned int j = 0; j < mat.cols(); ++j) {
        outfile << setw(7) << mat(i,j) << " ";
      }
      outfile << endl;
    }
    if (0 != el_cols.size()) {
      outfile << "\nNumbers of eliminated columns (0 based):\n";
      copy(el_cols.begin(), el_cols.end(), std::ostream_iterator<unsigned int>(outfile, " "));
      outfile << "\n\nMatrix after the eliminating: " << endl;
      for (unsigned int i = 0; i < mat.rows(); ++i) {
        for (unsigned int j = 0; j < mat.cols(); ++j) {
          if ( std::find(el_cols.begin(), el_cols.end(), j) == el_cols.end() ) {
            outfile << setw(7) << mat(i,j) << " ";
          }
        }
        outfile << endl;
      }
    }
    outfile << "\n\n" << method << " method was used\n";
    outfile << "\n\nStandard deviation:\n";
    copy(sd.begin(), sd.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nProportion of variance:\n";
    copy(prop_of_var.begin(), prop_of_var.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nCumulative proportion:\n";
    copy(cum_prop.begin(), cum_prop.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nKaiser criterion: " << kaiser;
    outfile << "\n\n95% threshold criterion: " << thresh95 << endl;
    outfile << "\n\n99% threshold criterion: " << thresh99 << endl;

    outfile << "\n\nRotated data: " << endl;
    unsigned int row_lim = nrows,
                 col_lim = ncols;
    if (scores.size() != nrows * ncols) {
      row_lim = (nrows < ncols)? nrows : ncols,
      col_lim = (ncols < nrows)? ncols : nrows;
    }
    for (unsigned int i = 0; i < row_lim; ++i) {
      for (unsigned int j = 0; j < col_lim; ++j) {
        outfile << setw(13) << scores[j + col_lim*i];
      }
      outfile << endl;
    }

    // export data reduced in dimentionality
    data_red.clear();
    data_red.resize(data.size());
    data_red.at(0).resize(max(kaiser,thresh99));

    for (size_t i = 0; i < data.size(); ++i) {
      vector<double> data_row = data.at(i);
      vector<double> data_red_row;
      for (size_t j = 0; j < data_row.size(); ++j) {
          if(kaiser<=thresh99){
              if(cum_prop.at(j)<=0.99)
                  data_red_row.push_back(data_row.at(j));
          }else{
              if(sd.at(j)>=1.0)
                  data_red_row.push_back(data_row.at(j));
          }
      }
      data_red.at(i) = data_red_row;
    }

    outfile << "\n\nMatrix reduced: " << endl;
    for (size_t i = 0; i < data_red.size(); ++i) {
      vector<double> data_red_row = data_red.at(i);
      for (size_t j = 0; j < data_red_row.size(); ++j) {
          outfile << setw(7) << data_red_row.at(j) << " ";
      }
      outfile << endl;
    }

    outfile.close();
    return 0;
}



void PowerLawDialog::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
{
    // Formula of the numarical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


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
       if(step_value==0)
           step_value=0.01;
       derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = function.at(tnsample-1);
       f1 = function.at(tnsample);
       f2 = function.at(tnsample+1);
       f3 = function.at(tnsample+2);
       f4 = function.at(tnsample+3);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=0.01;
       derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< function.size() -2;++i){     // centered
           f0 = function.at(i-2);
           f1 = function.at(i-1);
           f2 = function.at(i);
           f3 = function.at(i+1);
           f4 = function.at(i+2);
           step_value = step_values.at(i);
           if(step_value==0)
               step_value=0.01;
           derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
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
       if(step_value==0)
           step_value=0.01;
       derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));

       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = function.size()-1;
       f0 = function.at(tnsample-4);
       f1 = function.at(tnsample-3);
       f2 = function.at(tnsample-2);
       f3 = function.at(tnsample-1);
       f4 = function.at(tnsample);
       step_value = step_values.at(tnsample);
       if(step_value==0)
           step_value=0.01;
       derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));

}

int PowerLawDialog::linreg(const QVector<double> &x, const QVector<double> &y, double *b, double *m, double *r)
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

void PowerLawDialog::on_pushButton_save_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/power_law_2D", &st) == -1) {
        mkdir("results/planning/power_law_2D", 0700);
    }
    QString path("results/planning/power_law_2D/");

    ui->plot_hand_pos->savePdf(path+QString("hand_pos.pdf"),true,0,0,QString(),QString("Hand position"));
    ui->plot_curvature->savePdf(path+QString("radius.pdf"),true,0,0,QString(),QString("Curvature radius"));
    ui->plot_ang_vel->savePdf(path+QString("vel_tan.pdf"),true,0,0,QString(),QString("Tangiental velocity"));
    ui->plot_23->savePdf(path+QString("power_law.pdf"),true,0,0,QString(),QString("Power law"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("radius.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("radius.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("vel_tan.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("vel_tan.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("power_law.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("power_law.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());


}




} // namespace motion_manager
