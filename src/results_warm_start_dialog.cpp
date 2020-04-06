#include "../include/motion_manager/results_warm_start_dialog.hpp"

namespace motion_manager {

WarmStartResultsDialog::WarmStartResultsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WarmStartResultsDialog)
{
    ui->setupUi(this);

}

WarmStartResultsDialog::~WarmStartResultsDialog()
{
    delete ui;
}




void WarmStartResultsDialog::setPlanData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps)
{

    this->iterations_plan = iter;
    this->cpu_time_plan = cpu_time;
    this->obj_plan = obj;
    this->x_plan = x;
    this->zL_plan = zL;
    this->zU_plan = zU;
    this->dual_plan = dual_vars;
    this->n_plan_steps = warm_n_steps;
    this->ui->label_warm_n_plan_steps_value->setText(QString::number(warm_n_steps));

    this->ui->label_iter_count_plan_value->setText(QString::number(iter));
    this->ui->label_cpu_time_plan_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_plan_value->setText(QString::number(obj));

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

}

void WarmStartResultsDialog::setApproachData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps)
{

    this->iterations_approach = iter;
    this->cpu_time_approach = cpu_time;
    this->obj_approach = obj;
    this->x_approach = x;
    this->zL_approach = zL;
    this->zU_approach = zU;
    this->dual_approach = dual_vars;
    this->n_app_steps = warm_n_steps;
    this->ui->label_warm_n_app_steps_value->setText(QString::number(warm_n_steps));

    this->ui->label_iter_count_app_value->setText(QString::number(iter));
    this->ui->label_cpu_time_app_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_app_value->setText(QString::number(obj));

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

}

void WarmStartResultsDialog::setRetreatData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars, int warm_n_steps)
{

    this->iterations_retreat = iter;
    this->cpu_time_retreat = cpu_time;
    this->obj_retreat = obj;
    this->x_retreat = x;
    this->zL_retreat = zL;
    this->zU_retreat = zU;
    this->dual_retreat = dual_vars;
    this->n_ret_steps = warm_n_steps;
    this->ui->label_warm_n_ret_steps_value->setText(QString::number(warm_n_steps));

    this->ui->label_iter_count_retreat_value->setText(QString::number(iter));
    this->ui->label_cpu_time_retreat_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_retreat_value->setText(QString::number(obj));

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

}

void WarmStartResultsDialog::setBounceData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars)
{

    this->iterations_bounce = iter;
    this->cpu_time_bounce = cpu_time;
    this->obj_bounce = obj;
    this->x_bounce = x;
    this->zL_bounce = zL;
    this->zU_bounce = zU;
    this->dual_bounce = dual_vars;

    this->ui->label_iter_count_bounce_value->setText(QString::number(iter));
    this->ui->label_cpu_time_bounce_value->setText(QString::number(cpu_time));
    this->ui->label_obj_func_bounce_value->setText(QString::number(obj));

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
                                                    "All Files (*.*);;Tol Files (*.dual)");
    QFile f( filename );
    if(f.open( QIODevice::WriteOnly )){

        QTextStream stream( &f );
        stream << "### Dual variables and solutions of the optimization problems ###" << endl;
        if(this->en_plan)
        {
            stream << "## Plan target posture selection data ##"<<endl;
            stream << "Iterations_plan=" << QString::number(this->iterations_plan).toStdString().c_str() << endl;
            stream << "Cpu_time_plan=" << QString::number(this->cpu_time_plan).toStdString().c_str() << endl;
            stream << "Obj_plan=" << QString::number(this->obj_plan).toStdString().c_str() << endl;
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






} // namespace motion_manager
