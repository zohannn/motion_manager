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




void WarmStartResultsDialog::setPlanData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars)
{
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

void WarmStartResultsDialog::setApproachData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars)
{
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

void WarmStartResultsDialog::setRetreatData(int iter, double cpu_time, double obj, vector<double> &x, vector<double> &zL, vector<double> &zU, vector<double> &dual_vars)
{
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
    this->ui->tabWidget->setTabEnabled(0, en);
}

void WarmStartResultsDialog::enableApproachData(bool en)
{
    this->ui->tabWidget->setTabEnabled(1, en);
}

void WarmStartResultsDialog::enableRetreatData(bool en)
{
    this->ui->tabWidget->setTabEnabled(2, en);
}

void WarmStartResultsDialog::enableBounceData(bool en)
{
    this->ui->tabWidget->setTabEnabled(3, en);
}

// Q_SLOTS

void WarmStartResultsDialog::on_pushButton_save_warm_start_res_clicked()
{
/*
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

*/
}






} // namespace motion_manager
