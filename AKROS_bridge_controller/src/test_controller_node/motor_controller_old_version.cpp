// 複数モータ制御

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>
#include <AKROS_bridge_msgs/Initialize.h>
#include <std_srvs/Empty.h>

#define MOTOR_NUM   2   // モータ数を指定

static const double endTime = 15.0; // [s]
static const double control_frequency = 50.0;  // [Hz]

static const double wave_frequency[2] = {1.0, 0.5};   // [Hz]
static const double amplitude[2] = {M_PI/2, M_PI/4};   //[rad]


ros::Publisher cmd_pub; // Publisher
ros::ServiceClient initialize_client, finalize_client;  // client

AKROS_bridge_msgs::motor_cmd cmd_msg;    // Publish_msg
AKROS_bridge_msgs::Initialize initialize_srv;  // client_srv
std_srvs::Empty finalize_srv;


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    initialize_client = nh.serviceClient<AKROS_bridge_msgs::Initialize>("/cmd/enter_control_mode");
    finalize_client = nh.serviceClient<std_srvs::Empty>("/cmd/exit_control_mode");
    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("/cmd/motor_cmd", 2);
    
    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(0.5);

    initialize_srv.request.joint_num = MOTOR_NUM;
    initialize_client.call(initialize_srv);
    ROS_INFO("Motor Initialized");
    
    float q_init[MOTOR_NUM];

    std::cout << argv[1] << std::endl;


    /* PCにマイコンを接続しないとコアダンプが起こる(空データ参照) */
    /* テストしたいときに困るので引数つきで実行したらココをスルーするようにしたい */
    if(argc != 1){
        if(argv[1] == "debug"){
            for(uint8_t i=0; i<MOTOR_NUM; i++){
                q_init[i] = initialize_srv.response.jointstate.position[i];
                ROS_INFO("pos %d : %f", i, initialize_srv.response.jointstate.position[i]);
                ROS_INFO("vel %d : %f", i, initialize_srv.response.jointstate.velocity[i]);
                ROS_INFO("eff %d : %f", i, initialize_srv.response.jointstate.effort[i]);
            }
        }
    }

    q_init[0] = 0.0;
    q_init[1] = 0.0;
    
    // resize
    cmd_msg.cmd.positions.resize(MOTOR_NUM);
    cmd_msg.cmd.velocities.resize(MOTOR_NUM);
    cmd_msg.cmd.effort.resize(MOTOR_NUM);
    cmd_msg.Kp.resize(MOTOR_NUM);
    cmd_msg.Kd.resize(MOTOR_NUM);

    float omega[2];
    omega[0] = 2*M_PI*wave_frequency[0];
    omega[1] = 2*M_PI*wave_frequency[1];

    cmd_msg.Kp[0] = 3.0;
    cmd_msg.Kp[1] = 8.0;
    cmd_msg.Kd[0] = 0.5;
    cmd_msg.Kd[1] = 0.5;
    cmd_msg.cmd.effort[0] = 0.0;
    cmd_msg.cmd.effort[1] = 0.0;

    ROS_INFO("controller start!");

    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        cmd_msg.cmd.positions[0] = (amplitude[0] + q_init[0]) + amplitude[0] * cos(omega[0]*current_time - M_PI);
        cmd_msg.cmd.positions[1] = (amplitude[1] + q_init[1]) + amplitude[1] * cos(omega[1]*current_time - M_PI);
        
        cmd_msg.cmd.velocities[0] = -omega[0]*amplitude[0]*sin(omega[0]*current_time-M_PI);
        cmd_msg.cmd.velocities[1] = -omega[1]*amplitude[1]*sin(omega[1]*current_time-M_PI);

        // 時間が来たらpublishをやめる
        if(current_time <= endTime){
            cmd_pub.publish(cmd_msg);
        }else{
            ROS_INFO("control finished !");
            finalize_client.call(finalize_srv);
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}