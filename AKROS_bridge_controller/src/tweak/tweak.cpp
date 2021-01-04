#include "tweak.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


tweak_controller::tweak_controller(QWidget* parent)
  : QDialog(parent),
    nh(){
    CAN_ID_label = new QLabel("CAN_ID");

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(enter_tweak_mode_button, 0, 1);
    layout->addWidget(exit_tweak_mode_button, 0, 2);


    tweak_client = nh.serviceClient<AKROS_bridge_msgs::tweak>("tweak");
    ROS_INFO("Ready for tweak !");
}

void tweak_controller::tweak_up_Cb(void){
    // CAN_IDをspinBocから取得
    //tweak_srv.request.CAN_ID = ;
    tweak_srv.request.control = 0;

    if(tweak_client.call(tweak_srv)){
        tweak_srv.response.success = true;
    }
}

void tweak_controller::tweak_down_Cb(void){
    //tweak_srv.request.CAN_ID = ;
    tweak_srv.request.control = 1;

    if(tweak_client.call(tweak_srv)){
        tweak_srv.response.success = true;
    }
}

void tweak_controller::up_Cb(void){
    //tweak_srv.request.CAN_ID = ;
    tweak_srv.request.control = 2;

    if(tweak_client.call(tweak_srv)){
        tweak_srv.response.success = true;
    }
}

void tweak_controller::down_Cb(void){
    //tweak_srv.request.CAN_ID = ;
    tweak_srv.request.control = 3;

    if(tweak_client.call(tweak_srv)){
        tweak_srv.response.success = true;
    }
}