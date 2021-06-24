# AKROS_bridge
## 概要
ROSノードからAKシリーズモータ(T-motor製)を制御できるようになるライブラリのこと．

## 仕組み
STMマイコンをPCとモータの間に接続することにより，ROSとCAN通信の橋渡し(bridge)を行う．  
また，目標値，応答値のやり取りや，初期化を行うサービス通信なども実装した．  
PCとROSの接続にはrosserialを用いた．

# 詳細
## 動かし方
 1. マイコンをROSネットワークに接続 
```
    $ roslaunch AKROS_bridge_ros connect.launch
```
 2. モータの初期化を行う．設定やパラメータは```AKROS_bridge_controller/config/robot_config.yaml ``` 
 に記載．  
 ```
    $ roslaunch AKROS_bridge_controller initialize_robot.launch
 ```
 3. ロボットの原点合わせを行う  
 ```
    $ rosservice call /set_position_to_zero <対応するモータのCAN_ID>
 ```
 4. 目標指令とPDゲインをメッセージにセットしてノードからpublish(=コントローラを実行)  
 ```
    $ rosrun AKROS_bridge_controller <controller_node>
 ```

## パッケージ一覧
・AKROS_bridge = メタパッケージ  
・AKROS_bridge_controller=ロボットの制御に関連するパッケージ  
・AKROS_bridge_converter=ROSのメッセージとCANメッセージとの変換などを行う  
・AKROS_bridge_msgs=ROSでの通信に使用するメッセージ，サービスをまとめたパッケージ  
・AKROS_bridge_ros=その他通信に関するパッケージ  
(・mbed=マイコンのファームウェア用フォルダ．**ROSパッケージではない！**)

## topic一覧
・/motor_cmd -> モータ指令topic（角度，角速度，トルク，Pゲイン，Dゲイン）  
・/motor_reply -> モータ応答（角度，角速度，トルク）

## service一覧
・/set_position_to_zero -> 現在位置を0にセット  
・/servo_setting -> 関節のサーボ剛性を変更   
・/current_state -> 現在の関節状態をAKROS_bridge_converterから受け取る  
・/tweak_control -> 関節の微調整に使用