#include <CAN_controller.h>

// Constructor
CAN_controller::CAN_controller()
  : can(CAN_RX_PIN, CAN_TX_PIN)
{
    can.frequency(CAN_FREQ);
    can.attach(callback(this, &CAN_controller::can_Cb));
    initializeFlag = false;
}


// モータから受け取った情報をmotor_statusに格納
void CAN_controller::can_Cb(void){
    CANMessage msg_;
    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            deserialize_reply(msg_);
        }
    }
}


// デジタル指令値をCANメッセージに変換（結合といったほうが正しい？）
// 送信先のCAN_IDをこの関数よりも前に入れておけ！
void CAN_controller::serialize_cmd(CANMessage &msg_){
    uint8_t index_ = find_index(msg_.id);
    msg_.data[0] = motor[index_].position_ref >> 8;
    msg_.data[1] = motor[index_].position_ref & 0xFF;
    msg_.data[2] = motor[index_].velocity_ref >> 4;
    msg_.data[3] = ((motor[index_].velocity_ref & 0xF)<<4) | (motor[index_].Kp >> 8);
    msg_.data[4] = motor[index_].Kp & 0xFF;
    msg_.data[5] = motor[index_].Kd >> 4;
    msg_.data[6] = ((motor[index_].Kd & 0xF)<<4) | (motor[index_].effort_ref>>8);
    msg_.data[7] = motor[index_].effort_ref & 0xFF;
}


// モータから返ってくるデジタル値（結合）をそれぞれの要素に分解
void CAN_controller::deserialize_reply(const CANMessage& msg_){
    uint8_t index_ = find_index(msg_.data[0]);

    motor[index_].position = ((msg_.data[1]<<8) | msg_.data[2]);
    motor[index_].velocity = ((msg_.data[3]<<4) | (msg_.data[4]>>4));
    motor[index_].effort   = (((msg_.data[4]&0xF)<<8) | msg_.data[5]);
}



// モータから受け取った情報をmotor_statusに格納
void CAN_controller::can_send(uint8_t index_){
    CANMessage msg_;
    msg_.id = motor[index_].CAN_ID;
    serialize_cmd(msg_);
    can.write(msg_);
}


// enter control mode(モータコントロールモードに入る)
// 配列の添字をCAN_IDとするのではなく，motor_statusに記載されたCAN_IDを用いる！
// モータを制御するためには必須！
void CAN_controller::enter_control_mode(uint8_t CAN_ID_){
    CANMessage msg_;
    msg_.id = CAN_ID_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFC;
    
    if(can.write(msg_)){
        // pc.printf("Motor %d : Enter control mode \r\n", );
    }
}


// Exit motor control mode
void CAN_controller::exit_control_mode(uint8_t CAN_ID_){
    CANMessage msg_;
    msg_.id = CAN_ID_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFD;
    
    if(can.write(msg_)){
        // pc.printf("Motor %d : Exit control mode \r\n", );
    }
}

// set the current motor position to zero
void CAN_controller::set_position_to_zero(uint8_t CAN_ID_){
    CANMessage msg_;
    msg_.id = CAN_ID_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFE;
    
    if(can.write(msg_)){
        // pc.printf("Set the current motor position to zero \r\n");    // Debug
    }
}


// モータを追加する
// motor_statusをpush_back()
// CAN_IDの割当を行う
void CAN_controller::add_motor(uint8_t CAN_ID_){
    motor_status m_temp;
    m_temp.CAN_ID = CAN_ID_;
    motor.push_back(m_temp);
}


uint8_t CAN_controller::getMotorNum(void){
    return motor.size();
}

// モータのCAN_IDとvectorの番号は異なるのでそれを求める関数
uint8_t CAN_controller::find_index(uint8_t CAN_ID_){
    for(uint8_t i=0; i<motor.size(); i++){
        if(motor[i].CAN_ID == CAN_ID_){
            return i;
        }
    }
    return ERROR_VALUE;
}


// モータ個数がすでに決定したかどうか
bool CAN_controller::getInitializeFlag(void){
    return initializeFlag;
}


void CAN_controller::setInitializeFlag(bool b){
    initializeFlag = b;
}