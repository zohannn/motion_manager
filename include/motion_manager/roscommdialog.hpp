#ifndef ROSCOMMDIALOG_HPP
#define ROSCOMMDIALOG_HPP

#include <QDialog>
#include <ui_roscommdialog.h>
#include "qnode.hpp"

namespace motion_manager{

//! The RosCommDialog class
/**
 * @brief This class defines the dialog of the ROS communication functionalities
 */
class RosCommDialog : public QDialog
{
    Q_OBJECT

public:

    /**
     * @brief RosCommDialog, a constructor
     */
    RosCommDialog();

    /**
     * @brief RosCommDialog, a constructor
     * @param q
     * @param parent
     */
    RosCommDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief This method sets the url of the master
     * @param master_url
     */
    void setMasterUrl(QString& master_url);

    /**
     * @brief This method sets the url of the host
     * @param host_url
     */
    void setHostUrl(QString& host_url);

    /**
     * @brief This method sets the checkbox to remebmer the settings
     * @param r
     */
    void setRememberCheckbox(bool r);

    /**
     * @brief This method sets the checkbox to use the environemt variables
     * @param r
     */
    void setUseEnvCheckbox(bool r);

    /**
     * @brief This method gets the url of the master
     * @return
     */
    QString getMasterUrl();

    /**
     * @brief This method gets the url of the host
     * @return
     */
    QString getHostUrl();

    /**
     * @brief This method gets the value of the checkbox to remebmer the settings
     * @return
     */
    bool getRememberCheckbox();

    /**
     * @brief This method gets the value of the checkbox to use the environemt variables
     * @return
     */
    bool getUseEnvCheckbox();

    /**
     * @brief This method enables the editing of the url of the master
     * @param e
     */
    void enableMasterUrl(bool e);

    /**
     * @brief This method enables the editing of the url of the host
     * @param e
     */
    void enableHostUrl(bool e);

    /**
     * @brief ~RosCommDialog, a destructor
     */
    ~RosCommDialog();

public Q_SLOTS:

    /**
     * @brief This method connects/disconnects the node to/from the ROS server
     * @param check
     */
    void on_button_connect_clicked(bool check);

Q_SIGNALS:
    /**
     * @brief This method signals when the node is connected to ROS (c=true)
     * @param c
     */
    void rosConnected(bool c);

private:
    Ui::RosCommDialogDesign *ui;/**< handle of the user interface */
    /**
     * @brief This method shows an error message
     */
    void showNoMasterMessage();
    QNode *qnode; /**< pointer of the ROS node*/
};

} // namespace motion_manager

#endif // ROSCOMMDIALOG_HPP
