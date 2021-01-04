#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <AKROS_bridge_msgs/tweak.h>
#endif

#include <std_srvs/Empty.h>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>

// spinBox
#include <QSpinBox>

class tweak_controller : public QDialog{
    Q_OBJECT
public:
    tweak_controller(QWidget *parent);
private Q_SLOTS:
    void tweak_up_Cb();
    void tweak_down_Cb();
    void up_Cb();
    void down_Cb();


private:
    ros::NodeHandle nh;

    AKROS_bridge_msgs::tweak tweak_srv;
    ros::ServiceClient tweak_client;

    QSpinBox *CAN_ID_spinBox;
    QLabel *CAN_ID_label;

    QPushButton* enter_tweak_mode_button;
    QPushButton* exit_tweak_mode_button;
    
    QPushButton* tweak_up_button;
    QPushButton* tweak_down_button;
    QPushButton* up_button;
    QPushButton* down_button;

};