#ifndef TOLDIALOGHUML_H
#define TOLDIALOGHUML_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include<QMessageBox>
#include <ui_toldialoghuml.h>
#include <eigen3/Eigen/Dense>

namespace motion_manager{

using namespace std;
using namespace Eigen;

class TolDialogHUML : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    void on_pushButton_save_clicked(); // save the .tol file
    void on_pushButton_load_clicked(); // load the .tol file

public:
    //constructor
    explicit TolDialogHUML(QWidget *parent = 0);
    //destructor
    ~TolDialogHUML();

    //getters
    void getTolsArm(std::vector<float>& tols);
    void getTolsHand(MatrixXf& tols);
    void getTolsTable(std::vector<float>& tols);
    void getLambda(std::vector<float>& lambda);
    void getTolsObstacles(MatrixXf& tols);
    void getTolsTarget(MatrixXf& tols);
    int getSteps();
    float getWMax();
    int getApproachAxis();
    float getTolTarPos();
    float getTolTarOr();
    float getTolStop();
    void getEngageParams(float& dist, int& dir, std::vector<float>& tols);
    void getDisengageParams(float& dist, int& dir);
    bool getTargetAvoidance();
    bool getObstacleAvoidance();

    // setters
    void setInfo(string info);





private:
    Ui::TolDialogHUML *ui;
    string infoLine;
};

} // namespace motion_manager

#endif // TOLDIALOGHUML_H
