#include "../include/motion_manager/results_warm_start_dialog.hpp"

namespace motion_manager {

WarmStartResultsDialog::WarmStartResultsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WarmStartResultsDialog)
{
    ui->setupUi(this);
    ui->plot_plan->setEnabled(false);
    ui->plot_bounce->setEnabled(false);
    ui->plot_app->setEnabled(false);
    ui->plot_ret->setEnabled(false);
}

WarmStartResultsDialog::~WarmStartResultsDialog()
{
    delete ui;
}


void WarmStartResultsDialog::plotIterStats(QCustomPlot *plot, QString title, QVector<double> &iter,
                                           QVector<double> &obj, QVector<double> &dual_inf, QVector<double> &constr_viol,QVector<double> &error,QVector<double> &der_error)
{

    double der_average = 0.0;
    if(!der_error.empty())
        der_average = std::accumulate(der_error.begin(),der_error.end(),0.0)/der_error.size();
    QVector<double> der_err_mean(der_error.size(),der_average);

    plot->plotLayout()->clear();
    plot->clearGraphs();
    plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(plot);
    wideAxisRect->setupFullAxesBox(true);
    wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::red));
    wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::darkGreen));
    //wideAxisRect->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor(Qt::black));
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

    // objective function
    //plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    //plot->graph(0)->setPen(QPen(Qt::blue));
    //plot->graph(0)->setName("Obj func");
    //plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    //plot->graph(0)->keyAxis()->setLabel("iterations");
    //plot->graph(0)->setData(iter, obj);
    //plot->graph(0)->valueAxis()->setRange(*std::min_element(obj.begin(), obj.end()),*std::max_element(obj.begin(), obj.end()));
    //plot->graph(0)->rescaleAxes();

    // dual infeasibility
    //plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    //plot->graph(1)->setPen(QPen(Qt::red));
    //plot->graph(1)->setName("Dual inf");
    //plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    //plot->graph(1)->keyAxis()->setLabel("iterations");
    //plot->graph(1)->setData(iter, dual_inf);
    //plot->graph(1)->valueAxis()->setRange(*std::min_element(dual_inf.begin(), dual_inf.end()), *std::max_element(dual_inf.begin(), dual_inf.end()));
    //plot->graph(1)->rescaleAxes();

    // constraint violation
    //plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    //plot->graph(2)->setPen(QPen(Qt::darkGreen));
    //plot->graph(2)->setName("Constr viol");
    //plot->graph(2)->valueAxis()->setTickLabelColor(Qt::darkGreen);
    //plot->graph(2)->keyAxis()->setLabel("iterations");
    //plot->graph(2)->setData(iter, constr_viol);
    //plot->graph(2)->valueAxis()->setRange(*std::min_element(constr_viol.begin(), constr_viol.end()), *std::max_element(constr_viol.begin(), constr_viol.end()));
    //plot->graph(2)->rescaleAxes();

    // overall error
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(0)->setName("NLP Error");
    plot->graph(0)->valueAxis()->setTickLabelColor(Qt::blue);
    plot->graph(0)->keyAxis()->setLabel("iterations");
    plot->graph(0)->setData(iter, error);
    plot->graph(0)->valueAxis()->setRange(*std::min_element(error.begin(), error.end()), *std::max_element(error.begin(), error.end()));
    plot->graph(0)->rescaleAxes();

    // derivative of the overall error
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,1));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(1)->setName("Derivative NLP Error");
    plot->graph(1)->valueAxis()->setTickLabelColor(Qt::red);
    plot->graph(1)->keyAxis()->setLabel("iterations");
    plot->graph(1)->setData(iter, der_error);
    plot->graph(1)->valueAxis()->setRange(*std::min_element(der_error.begin(), der_error.end()), *std::max_element(der_error.begin(), der_error.end()));
    plot->graph(1)->rescaleAxes();

    // average derivative of the overall error
    plot->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft,2));
    plot->graph(2)->setPen(QPen(Qt::darkGreen));
    plot->graph(2)->setName("Derivative NLP Error Mean");
    plot->graph(2)->valueAxis()->setTickLabelColor(Qt::darkGreen);
    plot->graph(2)->keyAxis()->setLabel("iterations");
    plot->graph(2)->setData(iter, der_err_mean);
    plot->graph(2)->valueAxis()->setRange(*std::min_element(der_err_mean.begin(), der_err_mean.end()), *std::max_element(der_err_mean.begin(), der_err_mean.end()));
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
    //legend->addElement(0,3,new QCPPlottableLegendItem(legend,plot->graph(3)));

    // interactions
    connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(1)->valueAxis(), SLOT(setRange(QCPRange)));
    connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(2)->valueAxis(), SLOT(setRange(QCPRange)));
    //connect(plot->graph(0)->valueAxis(), SIGNAL(rangeChanged(QCPRange)), plot->graph(3)->valueAxis(), SLOT(setRange(QCPRange)));
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    plot->replot();
}

void WarmStartResultsDialog::setPlanData(int iter, double cpu_time, double obj, double overall_error,
                                         vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps,
                                         vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values, vector<double> &der_error_values)
{

    this->iterations_plan = iter;
    this->cpu_time_plan = cpu_time;
    this->obj_plan = obj;
    this->error_plan = overall_error;
    this->x_plan = x;
    this->zL_plan = zL;
    this->zU_plan = zU;
    this->dual_plan = dual_vars;
    this->n_plan_steps = warm_n_steps;
    this->obj_values_plan = obj_values;
    this->dual_inf_values_plan = dual_inf_values;
    this->constr_viol_values_plan = constr_viol_values;
    this->error_values_plan = error_values;
    this->der_error_values_plan = der_error_values;

    this->ui->label_warm_n_plan_steps_value->setText(QString::number(warm_n_steps));
    this->ui->label_iter_count_plan_value->setText(QString::number(iter));
    this->ui->label_cpu_time_plan_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_plan_value->setText(QString::number(obj));
    this->ui->label_error_plan_value->setText(QString::number(overall_error));

    this->ui->tableWidget_init_guess_plan->clear();
    this->ui->tableWidget_dual_vars_plan->clear();

    // initial guess and bounds multipliers
    QStringList h_headers; h_headers << "Solution [rad]" << "Lower Bounds [rad]" << "Upper Bounds [rad]";
    QStringList v_headers;
    this->ui->tableWidget_init_guess_plan->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_plan->setRowCount(x.size());
    for(int i =0; i < x.size(); ++i){
        double x_value = x.at(i);
        double zL_value = zL.at(i);
        double zU_value = zU.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_plan->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_plan->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_plan->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    QStringList h_dual_headers;  h_dual_headers << "Lagrange multipliers";
    QStringList v_dual_headers;
    this->ui->tableWidget_dual_vars_plan->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_plan->setRowCount(dual_vars.size());
    for(int i =0; i < dual_vars.size(); ++i){
        double d_value = dual_vars.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_plan->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_plan->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_plan->setVerticalHeaderLabels(v_dual_headers);

    // ---- plot ---- //
    this->ui->plot_plan->setEnabled(true);
    std::vector<int> iterations; iterations.resize(this->obj_values_plan.size()); std::iota (std::begin(iterations), std::end(iterations), 0);
    std::vector<double> diterations(iterations.begin(), iterations.end());
    QVector<double> qiter = QVector<double>::fromStdVector(diterations);
    QVector<double> qobj_values_plan = QVector<double>::fromStdVector(this->obj_values_plan);
    QVector<double> qdual_inf_values_plan = QVector<double>::fromStdVector(this->dual_inf_values_plan);
    QVector<double> qconstr_viol_values_plan = QVector<double>::fromStdVector(this->constr_viol_values_plan);
    QVector<double> qerror_values_plan = QVector<double>::fromStdVector(this->error_values_plan);
    QVector<double> qder_error_values_plan = QVector<double>::fromStdVector(this->der_error_values_plan);
    this->plotIterStats(this->ui->plot_plan,QString("Plan"),qiter,qobj_values_plan,qdual_inf_values_plan,qconstr_viol_values_plan,qerror_values_plan,qder_error_values_plan);


}

void WarmStartResultsDialog::setApproachData(int iter, double cpu_time, double obj, double overall_error,
                                             vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps,
                                             vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values,vector<double> &der_error_values)
{

    this->iterations_approach = iter;
    this->cpu_time_approach = cpu_time;
    this->obj_approach = obj;
    this->error_approach = overall_error;
    this->x_approach = x;
    this->zL_approach = zL;
    this->zU_approach = zU;
    this->dual_approach = dual_vars;
    this->n_app_steps = warm_n_steps;
    this->obj_values_app = obj_values;
    this->dual_inf_values_app = dual_inf_values;
    this->constr_viol_values_app = constr_viol_values;
    this->error_values_app = error_values;
    this->der_error_values_app = der_error_values;

    this->ui->label_warm_n_app_steps_value->setText(QString::number(warm_n_steps));
    this->ui->label_iter_count_app_value->setText(QString::number(iter));
    this->ui->label_cpu_time_app_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_app_value->setText(QString::number(obj));
    this->ui->label_error_app_value->setText(QString::number(overall_error));

    this->ui->tableWidget_init_guess_app->clear();
    this->ui->tableWidget_dual_vars_app->clear();

    // initial guess and bounds multipliers
    QStringList h_headers; h_headers << "Solution [rad]" << "Lower Bounds [rad]" << "Upper Bounds [rad]";
    QStringList v_headers;
    this->ui->tableWidget_init_guess_app->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_app->setRowCount(x.size());
    for(int i =0; i < x.size(); ++i){
        double x_value = x.at(i);
        double zL_value = zL.at(i);
        double zU_value = zU.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_app->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_app->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_app->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    QStringList h_dual_headers;  h_dual_headers << "Lagrange multipliers";
    QStringList v_dual_headers;
    this->ui->tableWidget_dual_vars_app->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_app->setRowCount(dual_vars.size());
    for(int i =0; i < dual_vars.size(); ++i){
        double d_value = dual_vars.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_app->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_app->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_app->setVerticalHeaderLabels(v_dual_headers);

    // ---- plot ---- //
    this->ui->plot_app->setEnabled(true);
    std::vector<int> iterations; iterations.resize(this->obj_values_app.size()); std::iota (std::begin(iterations), std::end(iterations), 0);
    std::vector<double> diterations(iterations.begin(), iterations.end());
    QVector<double> qiter = QVector<double>::fromStdVector(diterations);
    QVector<double> qobj_values_app = QVector<double>::fromStdVector(this->obj_values_app);
    QVector<double> qdual_inf_values_app = QVector<double>::fromStdVector(this->dual_inf_values_app);
    QVector<double> qconstr_viol_values_app = QVector<double>::fromStdVector(this->constr_viol_values_app);
    QVector<double> qerror_values_app = QVector<double>::fromStdVector(this->error_values_app);
    QVector<double> qder_error_values_app = QVector<double>::fromStdVector(this->der_error_values_app);
    this->plotIterStats(this->ui->plot_app,QString("Approach"),qiter,qobj_values_app,qdual_inf_values_app,qconstr_viol_values_app,qerror_values_app,qder_error_values_app);

}

void WarmStartResultsDialog::setRetreatData(int iter, double cpu_time, double obj, double overall_error,
                                            vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps,
                                            vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values, vector<double> &der_error_values)
{

    this->iterations_retreat = iter;
    this->cpu_time_retreat = cpu_time;
    this->obj_retreat = obj;
    this->error_retreat = overall_error;
    this->x_retreat = x;
    this->zL_retreat = zL;
    this->zU_retreat = zU;
    this->dual_retreat = dual_vars;
    this->n_ret_steps = warm_n_steps;
    this->obj_values_ret = obj_values;
    this->dual_inf_values_ret = dual_inf_values;
    this->constr_viol_values_ret = constr_viol_values;
    this->error_values_ret = error_values;
    this->der_error_values_ret = der_error_values;

    this->ui->label_warm_n_ret_steps_value->setText(QString::number(warm_n_steps));
    this->ui->label_iter_count_retreat_value->setText(QString::number(iter));
    this->ui->label_cpu_time_retreat_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_retreat_value->setText(QString::number(obj));
    this->ui->label_error_retreat_value->setText(QString::number(overall_error));

    this->ui->tableWidget_init_guess_retreat->clear();
    this->ui->tableWidget_dual_vars_retreat->clear();

    // initial guess and bounds multipliers
    QStringList h_headers; h_headers << "Solution [rad]" << "Lower Bounds [rad]" << "Upper Bounds [rad]";
    QStringList v_headers;
    this->ui->tableWidget_init_guess_retreat->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_retreat->setRowCount(x.size());
    for(int i =0; i < x.size(); ++i){
        double x_value = x.at(i);
        double zL_value = zL.at(i);
        double zU_value = zU.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_retreat->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_retreat->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_retreat->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    QStringList h_dual_headers;  h_dual_headers << "Lagrange multipliers";
    QStringList v_dual_headers;
    this->ui->tableWidget_dual_vars_retreat->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_retreat->setRowCount(dual_vars.size());
    for(int i =0; i < dual_vars.size(); ++i){
        double d_value = dual_vars.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_retreat->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_retreat->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_retreat->setVerticalHeaderLabels(v_dual_headers);

    // ---- plot ---- //
    this->ui->plot_ret->setEnabled(true);
    std::vector<int> iterations; iterations.resize(this->obj_values_ret.size()); std::iota (std::begin(iterations), std::end(iterations), 0);
    std::vector<double> diterations(iterations.begin(), iterations.end());
    QVector<double> qiter = QVector<double>::fromStdVector(diterations);
    QVector<double> qobj_values_ret = QVector<double>::fromStdVector(this->obj_values_ret);
    QVector<double> qdual_inf_values_ret = QVector<double>::fromStdVector(this->dual_inf_values_ret);
    QVector<double> qconstr_viol_values_ret = QVector<double>::fromStdVector(this->constr_viol_values_ret);
    QVector<double> qerror_values_ret = QVector<double>::fromStdVector(this->error_values_ret);
    QVector<double> qder_error_values_ret = QVector<double>::fromStdVector(this->der_error_values_ret);
    this->plotIterStats(this->ui->plot_ret,QString("Approach"),qiter,qobj_values_ret,qdual_inf_values_ret,qconstr_viol_values_ret,qerror_values_ret,qder_error_values_ret);

}

void WarmStartResultsDialog::setBounceData(int iter, double cpu_time, double obj, double overall_error,
                                           vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars,
                                           vector<double> &obj_values, vector<double> &dual_inf_values, vector<double> &constr_viol_values, vector<double> &error_values,vector<double> &der_error_values)
{

    this->iterations_bounce = iter;
    this->cpu_time_bounce = cpu_time;
    this->obj_bounce = obj;
    this->error_bounce = overall_error;
    this->x_bounce = x;
    this->zL_bounce = zL;
    this->zU_bounce = zU;
    this->dual_bounce = dual_vars;
    this->obj_values_bounce = obj_values;
    this->dual_inf_values_bounce = dual_inf_values;
    this->constr_viol_values_bounce = constr_viol_values;
    this->error_values_bounce = error_values;
    this->der_error_values_bounce = der_error_values;

    this->ui->label_iter_count_bounce_value->setText(QString::number(iter));
    this->ui->label_cpu_time_bounce_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_bounce_value->setText(QString::number(obj));
    this->ui->label_error_bounce_value->setText(QString::number(overall_error));

    this->ui->tableWidget_init_guess_bounce->clear();
    this->ui->tableWidget_dual_vars_bounce->clear();

    // initial guess and bounds multipliers
    QStringList h_headers; h_headers << "Solution [rad]" << "Lower Bounds [rad]" << "Upper Bounds [rad]";
    QStringList v_headers;
    this->ui->tableWidget_init_guess_bounce->setColumnCount(h_headers.size());
    this->ui->tableWidget_init_guess_bounce->setRowCount(x.size());
    for(int i =0; i < x.size(); ++i){
        double x_value = x.at(i);
        double zL_value = zL.at(i);
        double zU_value = zU.at(i);
        v_headers.push_back(QString("Joint ")+QString::number(i));
        std::vector<QString> row = {QString::number(x_value),QString::number(zL_value),QString::number(zU_value)};
        for(int j=0; j < h_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_init_guess_bounce->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_init_guess_bounce->setHorizontalHeaderLabels(h_headers);
    this->ui->tableWidget_init_guess_bounce->setVerticalHeaderLabels(v_headers);

    // constraints lagrange multipliers
    QStringList h_dual_headers;  h_dual_headers << "Lagrange multipliers";
    QStringList v_dual_headers;
    this->ui->tableWidget_dual_vars_bounce->setColumnCount(h_dual_headers.size());
    this->ui->tableWidget_dual_vars_bounce->setRowCount(dual_vars.size());
    for(int i =0; i < dual_vars.size(); ++i){
        double d_value = dual_vars.at(i);
        v_dual_headers.push_back(QString("Constraint ")+QString::number(i));
        std::vector<QString> row = {QString::number(d_value)};
        for(int j=0; j < h_dual_headers.size(); ++j){
           QString item = row.at(j);
           this->ui->tableWidget_dual_vars_bounce->setItem(i,j,new QTableWidgetItem(item));
        }
    }
    this->ui->tableWidget_dual_vars_bounce->setHorizontalHeaderLabels(h_dual_headers);
    this->ui->tableWidget_dual_vars_bounce->setVerticalHeaderLabels(v_dual_headers);

    // ---- plot ---- //
    this->ui->plot_bounce->setEnabled(true);
    std::vector<int> iterations; iterations.resize(this->obj_values_bounce.size()); std::iota (std::begin(iterations), std::end(iterations), 0);
    std::vector<double> diterations(iterations.begin(), iterations.end());
    QVector<double> qiter = QVector<double>::fromStdVector(diterations);
    QVector<double> qobj_values_bounce = QVector<double>::fromStdVector(this->obj_values_bounce);
    QVector<double> qdual_inf_values_bounce = QVector<double>::fromStdVector(this->dual_inf_values_bounce);
    QVector<double> qconstr_viol_values_bounce = QVector<double>::fromStdVector(this->constr_viol_values_bounce);
    QVector<double> qerror_values_bounce = QVector<double>::fromStdVector(this->error_values_bounce);
    QVector<double> qder_error_values_bounce = QVector<double>::fromStdVector(this->der_error_values_bounce);
    this->plotIterStats(this->ui->plot_bounce,QString("Bounce"),qiter,qobj_values_bounce,qdual_inf_values_bounce,qconstr_viol_values_bounce,qerror_values_bounce,qder_error_values_bounce);

}

void WarmStartResultsDialog::enablePlanData(bool en)
{
    this->en_plan = en;
    this->ui->tabWidget->setTabEnabled(0, en);
}

void WarmStartResultsDialog::enableApproachData(bool en)
{
    this->en_approach = en;
    this->ui->tabWidget->setTabEnabled(1, en);
}

void WarmStartResultsDialog::enableRetreatData(bool en)
{
    this->en_retreat = en;
    this->ui->tabWidget->setTabEnabled(2, en);
}

void WarmStartResultsDialog::enableBounceData(bool en)
{
    this->en_bounce = en;
    this->ui->tabWidget->setTabEnabled(3, en);
}

// Q_SLOTS

void WarmStartResultsDialog::on_pushButton_save_warm_start_res_clicked()
{

    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of dual variables"),
                                                    QString(MAIN_PATH)+"/Duals",
                                                    "All Files (*.*);;Dual Files (*.dual)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){

        QTextStream stream( &f );
        stream << qSetRealNumberPrecision(17);
        stream << "### Dual variables and solutions of the optimization problems ###" << endl;
        if(this->en_plan)
        {
            stream << "## Plan target posture selection data ##"<<endl;
            stream << "Iterations_plan=" << QString::number(this->iterations_plan).toStdString().c_str() << endl;
            stream << "Cpu_time_plan=" << QString::number(this->cpu_time_plan).toStdString().c_str() << endl;
            stream << "Obj_plan=" << QString::number(this->obj_plan).toStdString().c_str() << endl;
            stream << "Error_plan=" << QString::number(this->error_plan).toStdString().c_str() << endl;
            stream << "Warm_n_plan_steps=" << QString::number(this->n_plan_steps).toStdString().c_str() << endl;

            stream << "X_plan=";
            for(size_t i=0; i<this->x_plan.size();++i)
            {
                stream << this->x_plan.at(i);
                if(i!=this->x_plan.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZL_plan=";
            for(size_t i=0; i<this->zL_plan.size();++i)
            {
                stream << this->zL_plan.at(i);
                if(i!=this->zL_plan.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZU_plan=";
            for(size_t i=0; i<this->zU_plan.size();++i)
            {
                stream << this->zU_plan.at(i);
                if(i!=this->zU_plan.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "Dual_plan=";
            for(size_t i=0; i<this->dual_plan.size();++i)
            {
                stream << this->dual_plan.at(i);
                if(i!=this->dual_plan.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;
        }

        if(this->en_approach)
        {
            stream << "## Approach target posture selection data ##"<<endl;
            stream << "Iterations_approach=" << QString::number(this->iterations_approach).toStdString().c_str() << endl;
            stream << "Cpu_time_approach=" << QString::number(this->cpu_time_approach).toStdString().c_str() << endl;
            stream << "Obj_approach=" << QString::number(this->obj_approach).toStdString().c_str() << endl;
            stream << "Error_approach=" << QString::number(this->error_approach).toStdString().c_str() << endl;
            stream << "Warm_n_app_steps=" << QString::number(this->n_app_steps).toStdString().c_str() << endl;

            stream << "X_approach=";
            for(size_t i=0; i<this->x_approach.size();++i)
            {
                stream << this->x_approach.at(i);
                if(i!=this->x_approach.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZL_approach=";
            for(size_t i=0; i<this->zL_approach.size();++i)
            {
                stream << this->zL_approach.at(i);
                if(i!=this->zL_approach.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZU_approach=";
            for(size_t i=0; i<this->zU_approach.size();++i)
            {
                stream << this->zU_approach.at(i);
                if(i!=this->zU_approach.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "Dual_approach=";
            for(size_t i=0; i<this->dual_approach.size();++i)
            {
                stream << this->dual_approach.at(i);
                if(i!=this->dual_approach.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;
        }

        if(this->en_retreat)
        {
            stream << "## Retreat target posture selection data ##"<<endl;
            stream << "Iterations_retreat=" << QString::number(this->iterations_retreat).toStdString().c_str() << endl;
            stream << "Cpu_time_retreat=" << QString::number(this->cpu_time_retreat).toStdString().c_str() << endl;
            stream << "Obj_retreat=" << QString::number(this->obj_retreat).toStdString().c_str() << endl;
            stream << "Error_retreat=" << QString::number(this->error_retreat).toStdString().c_str() << endl;
            stream << "Warm_n_ret_steps=" << QString::number(this->n_ret_steps).toStdString().c_str() << endl;

            stream << "X_retreat=";
            for(size_t i=0; i<this->x_retreat.size();++i)
            {
                stream << this->x_retreat.at(i);
                if(i!=this->x_retreat.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZL_retreat=";
            for(size_t i=0; i<this->zL_retreat.size();++i)
            {
                stream << this->zL_retreat.at(i);
                if(i!=this->zL_retreat.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZU_retreat=";
            for(size_t i=0; i<this->zU_retreat.size();++i)
            {
                stream << this->zU_retreat.at(i);
                if(i!=this->zU_retreat.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "Dual_retreat=";
            for(size_t i=0; i<this->dual_retreat.size();++i)
            {
                stream << this->dual_retreat.at(i);
                if(i!=this->dual_retreat.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;
        }

        if(this->en_bounce)
        {
            stream << "## Bounce posture selection data ##"<<endl;
            stream << "Iterations_bounce=" << QString::number(this->iterations_bounce).toStdString().c_str() << endl;
            stream << "Cpu_time_bounce=" << QString::number(this->cpu_time_bounce).toStdString().c_str() << endl;
            stream << "Obj_bounce=" << QString::number(this->obj_bounce).toStdString().c_str() << endl;
            stream << "Error_bounce=" << QString::number(this->error_bounce).toStdString().c_str() << endl;

            stream << "X_bounce=";
            for(size_t i=0; i<this->x_bounce.size();++i)
            {
                stream << this->x_bounce.at(i);
                if(i!=this->x_bounce.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZL_bounce=";
            for(size_t i=0; i<this->zL_bounce.size();++i)
            {
                stream << this->zL_bounce.at(i);
                if(i!=this->zL_bounce.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "ZU_bounce=";
            for(size_t i=0; i<this->zU_bounce.size();++i)
            {
                stream << this->zU_bounce.at(i);
                if(i!=this->zU_bounce.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;

            stream << "Dual_bounce=";
            for(size_t i=0; i<this->dual_bounce.size();++i)
            {
                stream << this->dual_bounce.at(i);
                if(i!=this->dual_bounce.size()-1)
                {
                    stream << "|";
                }
            }
            stream<< endl;
        }

        f.close();
    }

}

void WarmStartResultsDialog::on_pushButton_save_warm_start_plots_clicked()
{

    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                 "results/planning",
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks);

    if(this->ui->plot_plan->isEnabled()){
        QFile f( dir+QString("/plan_stats.csv") );
        if(f.open( QIODevice::WriteOnly )){
            QTextStream stream( &f );
            stream << qSetRealNumberPrecision(6);
            stream << "iter,obj_func,dual_inf,constr_viol,NLP_error,der_NLP_error" << endl;
            for(size_t i=0;i<this->obj_values_plan.size();++i)
            {
                stream << i << "," << this->obj_values_plan.at(i) << "," << this->dual_inf_values_plan.at(i) << ","
                       << this->constr_viol_values_plan.at(i) << "," << this->error_values_plan.at(i) << "," << this->der_error_values_plan.at(i) << endl;
            }
            f.close();
        }
        this->ui->plot_plan->savePdf(dir+QString("/plan_plot.pdf"),true,0,0,QString(),QString("Iterations of the Plan problem"));
    }

    if(this->ui->plot_app->isEnabled()){
        QFile f( dir+QString("/app_stats.csv") );
        if(f.open( QIODevice::WriteOnly )){
            QTextStream stream( &f );
            stream << qSetRealNumberPrecision(6);
            stream << "iter,obj_func,dual_inf,constr_viol,NLP_error,der_NLP_error" << endl;
            for(size_t i=0;i<this->obj_values_app.size();++i)
            {
                stream << i << "," << this->obj_values_app.at(i) << "," << this->dual_inf_values_app.at(i) << ","
                       << this->constr_viol_values_app.at(i) << "," << this->error_values_app.at(i) << "," << this->der_error_values_app.at(i) << endl;
            }
            f.close();
        }
        this->ui->plot_app->savePdf(dir+QString("/app_plot.pdf"),true,0,0,QString(),QString("Iterations of the Approach problem"));
    }

    if(this->ui->plot_ret->isEnabled()){
        QFile f( dir+QString("/ret_stats.csv") );
        if(f.open( QIODevice::WriteOnly )){
            QTextStream stream( &f );
            stream << qSetRealNumberPrecision(6);
            stream << "iter,obj_func,dual_inf,constr_viol,NLP_error,der_NLP_error" << endl;
            for(size_t i=0;i<this->obj_values_ret.size();++i)
            {
                stream << i << "," << this->obj_values_ret.at(i) << "," << this->dual_inf_values_ret.at(i) << ","
                       << this->constr_viol_values_ret.at(i) << "," << this->error_values_ret.at(i) << "," << this->der_error_values_ret.at(i) << endl;
            }
            f.close();
        }
        this->ui->plot_ret->savePdf(dir+QString("/ret_plot.pdf"),true,0,0,QString(),QString("Iterations of the Retreat problem"));
    }

    if(this->ui->plot_bounce->isEnabled()){
        QFile f( dir+QString("/bounce_stats.csv") );
        if(f.open( QIODevice::WriteOnly )){
            QTextStream stream( &f );
            stream << qSetRealNumberPrecision(6);
            stream << "iter,obj_func,dual_inf,constr_viol,NLP_error,der_NLP_error" << endl;
            for(size_t i=0;i<this->obj_values_bounce.size();++i)
            {
                stream << i << "," << this->obj_values_bounce.at(i) << "," << this->dual_inf_values_bounce.at(i) << ","
                       << this->constr_viol_values_bounce.at(i) << "," << this->error_values_bounce.at(i) << "," << this->der_error_values_bounce.at(i) << endl;
            }
            f.close();
        }
        this->ui->plot_bounce->savePdf(dir+QString("/bounce_plot.pdf"),true,0,0,QString(),QString("Iterations of the Bounce problem"));
    }
}






} // namespace motion_manager
