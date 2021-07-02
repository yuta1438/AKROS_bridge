#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

Prototype2020_BaseController::Prototype2020_BaseController(void){
    // タイマ割り込みの設定
    timer = nh.createTimer(ros::Duration(1.0/controller_frequency), &Prototype2020_BaseController::loop, this);
    pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>(topic_name, 1);
    currentState_client = nh.serviceClient<AKROS_bridge_msgs::currentState>("current_state");
    
    robot_cmd.motor.resize(JOINTNUM);   // メンバイニシャライザではダメだった！
    qref.resize(JOINTNUM);
    qref_old.resize(JOINTNUM);
    q_init.resize(JOINTNUM);
    q_initialPose.resize(JOINTNUM);
    read_State(q_init);

    q_initialPose << deg2rad(initialPose_deg[0]), deg2rad(initialPose_deg[1]), deg2rad(initialPose_deg[2]);

    // rosparamの読み取り
    XmlRpc::XmlRpcValue rosparams;
    if(!nh.getParam("motor_list", rosparams)){
        ROS_ERROR("Failed to load rosparams");
    }

    int counter = 0;
    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        robot_cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        robot_cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        robot_cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        robot_cmd.motor[counter].effort = 0.0;
        counter++;
    }
    usleep(50000);  // delayを入れないと指令値の最初のほうが欠落する！
    timer_start();
}


// ロボットの関節角度の読み取り
// 引数には関節角を格納する変数を入れる
// ロボットの関節数(=n)次元のベクトルを引数に与えること！最悪segment faultが発生する
void Prototype2020_BaseController::read_State(Eigen::VectorXd& q_){
    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINTNUM; i++){
            q_[i] = currentState_srv.response.reply.motor[i].position;
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }
}


// 指令角度と前の指令角度から指令角速度を算出
// 指令値をPublish
void Prototype2020_BaseController::sendCommand(void){
    for(int i=0; i<JOINTNUM; i++){
        robot_cmd.motor[i].position = qref[i];
        robot_cmd.motor[i].velocity = (qref[i] - qref_old[i]) * controller_frequency;
    }
    qref_old = qref;    // qrefベクトルの更新

    #ifdef USE_TIMESTAMP
    robot_cmd.header.stamp = ros::Time::now();  // publisherの方はそのままの時間を使用したほうが良いか？
    #endif
    pub.publish(robot_cmd); // 指令値をpublish
}


// タイマ開始(t_startの更新)
void Prototype2020_BaseController::timer_start(void){
    t_start = ros::Time::now();
}


// 現在時刻の取得
double Prototype2020_BaseController::getTime(void){
    return (ros::Time::now() - t_start).toSec();
}


// タイマ，コントローラの停止
void Prototype2020_BaseController::stopController(void){
    timer.stop();
    ros::shutdown();
}


// 矢状面内での順運動学計算
// 返り値には計算結果を返す
bool Prototype2020_BaseController::solve_sagittal_FK(const Eigen::VectorXd& q, Eigen::Vector2d& p){
    p[0] = -l1*sin(q[0]) - l2*sin(q[0]+q[1]);
    p[1] = -l1*cos(q[0]) - l2*cos(q[0]+q[1]);
    return true;
}

// 矢状面内での順運動学計算
// 返り値に計算結果を返す
bool Prototype2020_BaseController::solve_sagittal_IK(const Eigen::Vector2d& p, Eigen::VectorXd& q){
    double L = p.norm();

    if(!RANGE_CHECK(L, Lmin, Lmax)){
        return false;
    }

    // そもそもその目標位置に到達可能か？
    // 「脚をこれ以上曲げられない」という判別も出来ないか？
    q[0] = -atan2(p[0], -p[1]) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    q[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));

    return true;  // 脚長チェック
}