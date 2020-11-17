# AKROS_bridgeとは？
ROSノードからAKシリーズモータ(T-motor製)を制御できるようになるライブラリのこと．

## 仕組み
STMマイコンをPCとモータの間に接続することにより，ROSとCAN通信の橋渡し(bridge)を行う．
また，目標値，応答値のやり取りや，初期化を行うサービス通信なども実装した．

# 詳細
## 動かし方
「AKROS_bridge_controller」パッケージのmotor_controller.cppを参照．
基本的には
 1. モータの初期化を行う．（このとき，モータの個数をRequestで渡す必要あり）
 2. 目標指令とPDゲインをメッセージにセットしてpublish

## topic一覧
・/cmd/motor_cmd(型: AKROS_bridge_msgs::motor_cmd.msg)
    ->モータ指令topic
（角度，角速度，トルク，Pゲイン，Dゲイン）

・/cmd/reply(型: AKROS_bridge_msgs::motor_reply.msg)
    -> モータ応答（角度，角速度，トルク）

## service一覧
・/cmd/enter_control_mode -> モータ制御モードON
・/cmd/exit_control_mode -> モータ制御モードOFF
・/cmd/set_zero_pos -> エンコーダの値を0にセット

