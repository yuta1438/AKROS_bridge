#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

Prototype2020_BaseController::Prototype2020_BaseController(void){
    // タイマ割り込みの設定
    timer = nh.createTimer(ros::Duration(1.0/controller_frequency), &Prototype2020_BaseController::loop, this);
    pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>(topic_name, 1);
    currentState_client = nh.serviceClient<AKROS_bridge_msgs::currentState>("current_state");
    
    robot_cmd.motor.resize(JOINTNUM);
    qref.resize(JOINTNUM);
    qref_old.resize(JOINTNUM);
    q_init.resize(JOINTNUM);

    read_State();

    // rosparamの読み取り
    XmlRpc::XmlRpcValue rosparams;
    nh.getParam("motor_list", rosparams);
    int counter = 0;

    // エラー箇所 ---------------------
    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        robot_cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        robot_cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        robot_cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        robot_cmd.motor[counter].effort = 0.0;
        counter++;
    }
    // -----------------------

    usleep(50000);  // delayを入れないと指令値の最初のほうが欠落する！
    timer_start();
}


void Prototype2020_BaseController::read_State(void){
    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINTNUM; i++){
            q_init[i] = currentState_srv.response.reply.motor[i].position;
            // std::cout << "q_init[" << i << "] : " << q_init[i] << std::endl; // debug
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }
}

void Prototype2020_BaseController::sendCommand(void){
    for(int i=0; i<JOINTNUM; i++){
        robot_cmd.motor[i].position = qref[i];
        robot_cmd.motor[i].velocity = (qref[i] - qref_old[i]) * controller_frequency;
    }
    qref_old = qref;
    pub.publish(robot_cmd);
}


void Prototype2020_BaseController::timer_start(){
    t_start = ros::Time::now();
}


double Prototype2020_BaseController::getTime(void){
    return (ros::Time::now() - t_start).toSec();
}


void Prototype2020_BaseController::stopController(void){
    timer.stop();
}


Eigen::Vector2d Prototype2020_BaseController::solve_sagittal_FK(Eigen::Vector2d q){
    Eigen::Vector2d p;
    p[0] = -l1*sin(q[0]) - l2*sin(q[0]+q[1]);
    p[1] = -l1*cos(q[0]) - l2*cos(q[0]+q[1]);
    return p;
}


Eigen::Vector2d Prototype2020_BaseController::solve_sagittal_IK(Eigen::Vector2d p, bool &error){
    Eigen::Vector2d q;
    double L = p.norm();
    error = (L>(l1+l2)) ? true : false;

    q = Eigen::Vector2d::Zero();

    q[0] = -atan2(p[0], -p[1]) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    q[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));

    return q;
}