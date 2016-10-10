#ifndef ROSCOMMDIALOG_HPP
#define ROSCOMMDIALOG_HPP

#include <QDialog>
#include <ui_roscommdialog.h>
#include "qnode.hpp"

namespace MotionPlanner{

class RosCommDialog : public QDialog
{
    Q_OBJECT

public:
    // constructor
    explicit RosCommDialog(QNode* q, QWidget *parent = 0);
    // setters
    void setMasterUrl(QString& master_url);
    void setHostUrl(QString& host_url);
    void setRememberCheckbox(bool r);
    void setUseEnvCheckbox(bool r);
    //getters
    QString getMasterUrl();
    QString getHostUrl();
    bool getRememberCheckbox();
    bool getUseEnvCheckbox();

    //enablers
    void enableMasterUrl(bool e);
    void enableHostUrl(bool e);
    // destructor
    ~RosCommDialog();

public Q_SLOTS:

    void on_button_connect_clicked(bool check); // connect button

Q_SIGNALS:
    void rosConnected(bool c);

private:
    Ui::RosCommDialogDesign *ui;
    void showNoMasterMessage();
    QNode *qnode;
};

} // namespace MotionPlanner

#endif // ROSCOMMDIALOG_HPP
