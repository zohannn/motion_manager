#include "../include/motion_manager/nat_coll_av_dialog.hpp"

namespace motion_manager {

NatCollAvDialog::NatCollAvDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Nat_coll_av_Dialog)
{
    ui->setupUi(this);
}

NatCollAvDialog::~NatCollAvDialog()
{
    delete ui;
}



void NatCollAvDialog::setupPlots(vector<vector<double>> &hand_linear_velocity,vector<vector<double> > &hand_position, vector<vector<vector<double> > > &timesteps)
{
    this->qtime.clear();
    this->linear_vel_x_squared.clear();
    this->lift_vel_squared.clear();
    this->linear_vel_squared.clear();
    this->T_mov.clear();

    vector<vector<vector<vector<double>>>> hand_position_task;
    // time
    vector<double> time_task; QVector<double> tot_timesteps;
    vector<int> index;
    //QVector<double> K; // Curvature of the task
    //QVector<double> T; // Torsion of the task
    //QVector<double> vel; // Velocity of the task
    //QVector<double> acc; // Acceleration of the task
    //double coeff_tot=0; // sum of the coefficient
    //int n_coeff=0; // number of the non-zero coefficient
    int offset=0; // offset to go trough the stages of hand_position
    //this->slopes.clear(); this->r_squared.clear(); this->n_points.clear();

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
    this->qtime = QVector<double>::fromStdVector(time_task);


    QVector<double> linear_vel_x;
    QVector<double> linear_vel_y;
    QVector<double> linear_vel_z;


    for(size_t i=0;i<hand_linear_velocity.size();++i){
        vector<double> linear = hand_linear_velocity.at(i);
        linear_vel_x.push_back(linear.at(0)); linear_vel_x_squared.push_back(pow(linear.at(0),2));
        linear_vel_y.push_back(linear.at(1));
        linear_vel_z.push_back(linear.at(2));
        lift_vel_squared.push_back(pow(linear.at(1),2)+pow(linear.at(2),2));
        linear_vel_squared.push_back(pow(linear.at(0),2)+pow(linear.at(1),2)+pow(linear.at(2),2));
    }

    //QVector<double> tsteps_mov;
    QVector<double> pos_x; QVector<double> pos_y; QVector<double> pos_z;
    for(size_t i=0; i<hand_position_task.size();++i){
        vector<vector<vector<double>>> hand_position_mov = hand_position_task.at(i);
        for(size_t j=0; j<hand_position_mov.size();++j){
            vector<vector<double>> hand_position_stage = hand_position_mov.at(j);
            //vector<double> tsteps_stage = timesteps_mov.at(j);
            for(size_t h=0; h<hand_position_stage.size();++h){
                vector<double> hand_point = hand_position_stage.at(h);
                //tsteps_mov.push_back(tsteps_stage.at(h));
                pos_x.push_back(hand_point.at(0)/1000); // [m]
                pos_y.push_back(hand_point.at(1)/1000); // [m]
                pos_z.push_back(hand_point.at(2)/1000); // [m]
            }//stage
        } // mov
    }// task

    // first derivatives
    QVector<double> der_pos_x_1; QVector<double> der_pos_y_1; QVector<double> der_pos_z_1;
    this->getDerivative(pos_x,tot_timesteps,der_pos_x_1);
    this->getDerivative(pos_y,tot_timesteps,der_pos_y_1);
    this->getDerivative(pos_z,tot_timesteps,der_pos_z_1);
    // second derivatives
    QVector<double> der_pos_x_2; QVector<double> der_pos_y_2; QVector<double> der_pos_z_2;
    this->getDerivative(der_pos_x_1,tot_timesteps,der_pos_x_2);
    this->getDerivative(der_pos_y_1,tot_timesteps,der_pos_y_2);
    this->getDerivative(der_pos_z_1,tot_timesteps,der_pos_z_2);
    //third derivatives
    QVector<double> der_pos_x_3; QVector<double> der_pos_y_3; QVector<double> der_pos_z_3;
    this->getDerivative(der_pos_x_2,tot_timesteps,der_pos_x_3);
    this->getDerivative(der_pos_y_2,tot_timesteps,der_pos_y_3);
    this->getDerivative(der_pos_z_2,tot_timesteps,der_pos_z_3);

    // --- Torsion --- //
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
        T_mov.push_back((double)num/den);
    }

    // plots natural obstacle avoidance
    plotComp(ui->plot_transp_vel,QString("Squared hand transport linear velocity "),qtime,linear_vel_x_squared,true); // plot transport component (x)
    plotComp(ui->plot_lift_vel,QString("Squared hand lift linear velocity"),qtime,lift_vel_squared,true); // plot lift component (y and z)

    // plot the torsion
    ui->plot_torsion->plotLayout()->clear();
    ui->plot_torsion->clearGraphs();
    ui->plot_torsion->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_torsion);
    wideAxisRect->setupFullAxesBox(true);
    QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_torsion);
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
    QString title = "Torsion";
    ui->plot_torsion->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_torsion,title));
    ui->plot_torsion->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_torsion->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_torsion->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_torsion->graph(0)->setName(title);
    ui->plot_torsion->graph(0)->valueAxis()->setLabel("torsion [1/m]");
    ui->plot_torsion->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_torsion->graph(0)->setData(qtime, T_mov);
    ui->plot_torsion->graph(0)->valueAxis()->setRange(*std::min_element(T_mov.begin(), T_mov.end()),
                                                      *std::max_element(T_mov.begin(), T_mov.end()));
    ui->plot_torsion->graph(0)->rescaleAxes();
    ui->plot_torsion->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_torsion->replot();

    //plot the hand linear velocity squared, the lift and the transport squared components
    ui->plot_hand_tot_vel->plotLayout()->clear();
    ui->plot_hand_tot_vel->clearGraphs();
    ui->plot_hand_tot_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_16);
    wideAxisRect = new QCPAxisRect(ui->plot_hand_tot_vel);
    wideAxisRect->setupFullAxesBox(true);
    //QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_16);
    marginGroup = new QCPMarginGroup(ui->plot_hand_tot_vel);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_hand_tot_vel->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }

    title = "Hand velocity composition";
    ui->plot_hand_tot_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_hand_tot_vel,title));
    ui->plot_hand_tot_vel->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_hand_tot_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_hand_tot_vel->graph(0)->setPen(QPen(Qt::black));

    ui->plot_hand_tot_vel->graph(0)->setName("hand vel ^2");
    ui->plot_hand_tot_vel->graph(0)->valueAxis()->setLabel("[mm^2/s^2]");
    ui->plot_hand_tot_vel->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_hand_tot_vel->graph(0)->setData(qtime, linear_vel_squared);
    ui->plot_hand_tot_vel->graph(0)->valueAxis()->setRange(*std::min_element(linear_vel_squared.begin(), linear_vel_squared.end()),
                                                            *std::max_element(linear_vel_squared.begin(), linear_vel_squared.end()));
    ui->plot_hand_tot_vel->graph(0)->rescaleAxes();

    ui->plot_hand_tot_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_hand_tot_vel->graph(1)->setPen(QPen(Qt::red,1.0,Qt::DashDotLine));

    ui->plot_hand_tot_vel->graph(1)->setName("hand lift vel ^2");

    ui->plot_hand_tot_vel->graph(1)->setData(qtime, lift_vel_squared);

    //ui->plot_hand_tot_vel->graph(1)->rescaleAxes();

    ui->plot_hand_tot_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_hand_tot_vel->graph(2)->setPen(QPen(Qt::blue,1.0,Qt::DashLine));
    ui->plot_hand_tot_vel->graph(2)->setName("hand transport vel ^2");

    ui->plot_hand_tot_vel->graph(2)->setData(qtime, linear_vel_x_squared);
    //ui->plot_hand_tot_vel->graph(2)->rescaleAxes();

    // legend
    QCPLegend *legend = new QCPLegend();
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    ui->plot_hand_tot_vel->plotLayout()->addElement(2, 0, subLayout);
    subLayout->setMargins(QMargins(5, 0, 5, 5));
    subLayout->addElement(0, 0, legend);
    // set legend's row stretch factor very small so it ends up with minimum height:
    ui->plot_hand_tot_vel->plotLayout()->setRowStretchFactor(2, 0.001);
    legend->setLayer("legend");
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    legend->setFont(legendFont);
    legend->addElement(0,0,new QCPPlottableLegendItem(legend,ui->plot_hand_tot_vel->graph(0)));
    legend->addElement(0,1,new QCPPlottableLegendItem(legend,ui->plot_hand_tot_vel->graph(1)));
    legend->addElement(0,2,new QCPPlottableLegendItem(legend,ui->plot_hand_tot_vel->graph(2)));


    ui->plot_hand_tot_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_hand_tot_vel->replot();


}


void NatCollAvDialog::plotComp(QCustomPlot *plot, QString title, QVector<double> &time, QVector<double> &var,bool lin)
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
        name="[mm^2/s^2]";
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

void NatCollAvDialog::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
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



// Q_SLOTS


void NatCollAvDialog::on_pushButton_save_nat_coll_av_clicked()
{

    QString path;

    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/nat_coll_av", &st) == -1) {
        mkdir("results/planning/nat_coll_av", 0700);
    }
    path = QString("results/planning/nat_coll_av/");


    // save plots
    ui->plot_lift_vel->savePdf(path+QString("hand_lift_vel.pdf"),true,0,0,QString(),QString("Squared Hand Lift Velocity"));
    ui->plot_transp_vel->savePdf(path+QString("hand_transp_vel.pdf"),true,0,0,QString(),QString("Squared Hand Transport Velocity"));
    ui->plot_torsion->savePdf(path+QString("hand_torsion.pdf"),true,0,0,QString(),QString("Hand Torsion"));
    ui->plot_hand_tot_vel->savePdf(path+QString("hand_tot_vel.pdf"),true,0,0,QString(),QString("Squared Hand Velocity Composition"));


    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_lift_vel.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_lift_vel.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_transp_vel.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_transp_vel.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_torsion.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_torsion.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("hand_tot_vel.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_tot_vel.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());


    // save text data files

    // hand lift velocity
    if(!this->lift_vel_squared.empty()){
        string filename_hand_vel("hand_lift_vel.csv");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND LIFT VELOCITY SQUARED \n");
        hand_vel << string("# velocity [mm^2/s^2], time [s] \n");

        for(size_t i=0;i<this->lift_vel_squared.size();++i){
            double vel = this->lift_vel_squared.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    // hand transport velocity
    if(!this->linear_vel_x_squared.empty()){
        string filename_hand_vel("hand_transp_vel.csv");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND TRANSPORT VELOCITY SQUARED \n");
        hand_vel << string("# velocity [mm^2/s^2], time [s] \n");

        for(size_t i=0;i<this->linear_vel_x_squared.size();++i){
            double vel = this->linear_vel_x_squared.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }


    // hand total velocity
    if(!this->linear_vel_squared.empty()){
        string filename_hand_vel("hand_tot_vel.csv");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND TOTAL VELOCITY SQUARED \n");
        hand_vel << string("# velocity [mm^2/s^2], time [s] \n");

        for(size_t i=0;i<this->linear_vel_squared.size();++i){
            double vel = this->linear_vel_squared.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }


    // TORSION OF THE HAND
    if(!this->T_mov.empty()){
        string filename_hand_vel("hand_torsion.csv");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# TORSION OF THE HAND \n");
        hand_vel << string("# torsion [1/m], time [s] \n");

        for(size_t i=0;i<this->T_mov.size();++i){
            double vel = this->T_mov.at(i);
            double time = this->qtime.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }




}







} // namespace motion_manager
