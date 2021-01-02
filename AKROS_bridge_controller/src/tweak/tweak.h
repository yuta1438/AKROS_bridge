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

private:
    ros::NodeHandle nh;

    AKROS_bridge_msgs::tweak srv;
    ros::ServiceClient tweak_up_client;     // +1
    ros::ServiceClient tweak_down_client;   // -1
    ros::ServiceClient up_client;           // +10
    ros::ServiceClient down_client;         // -10

    QSpinBox *CAN_ID_spinBox;
    QLabel *CAN_ID_label;
    void createSpinBox();

};