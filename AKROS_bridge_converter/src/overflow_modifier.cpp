// 前回と今回の値を比較してオーバーフローしたかどうかを確認
// 閾値はビット数をNとして，CENTER_VALUE +- 2^N-2とした
void AKROS_bridge_converter::overflow_check(motor_status& m){
    // Position
    // 上限を突破したか？
    if((m.position_old > (CENTER_POSITION + (1 << (POSITION_BIT_NUM - 2)))) && (m.position < (CENTER_POSITION - (1 << (POSITION_BIT_NUM - 2))))){
        // 超えるとどんどんインクリメントしてしまう！
        m.position_overflow_count++;
        std::cout << m.position_overflow_count << std::endl;
    }
    // 下限を突破したか？
    else if((m.position_old < (CENTER_POSITION - (1 << (POSITION_BIT_NUM-2)))) && (m.position > (CENTER_POSITION + (1 << (POSITION_BIT_NUM-2))))){
        m.position_overflow_count--;
        std::cout << m.position_overflow_count << std::endl;
    }

    // Velocity
    // 上限を突破したか？
    if((m.velocity_old > (CENTER_VELOCITY + (1 << (VELOCITY_BIT_NUM - 2)))) && (m.velocity < (CENTER_VELOCITY - (1 << (VELOCITY_BIT_NUM - 2))))){
        m.velocity_overflow_count++;
    }
    // 下限を突破したか？
    else if((m.velocity_old < (CENTER_VELOCITY - (1 << (VELOCITY_BIT_NUM-2)))) && (m.velocity > (CENTER_VELOCITY + (1 << (VELOCITY_BIT_NUM-2))))){
        m.position_overflow_count--;
    }

    // Effort
    // 上限を突破したか？
    if((m.effort_old > (CENTER_EFFORT + (1 << (EFFORT_BIT_NUM - 2)))) && (m.effort < (CENTER_EFFORT - (1 << (EFFORT_BIT_NUM - 2))))){
        m.effort_overflow_count++;
    }
    // 下限を突破したか？
    else if((m.effort_old < (CENTER_EFFORT - (1 << (EFFORT_BIT_NUM-2)))) && (m.effort > (CENTER_EFFORT + (1 << (EFFORT_BIT_NUM-2))))){
        m.effort_overflow_count--;
    }

    m.position_old = m.position;
    m.velocity_old = m.velocity;
    m.effort_old   = m.effort;
}
