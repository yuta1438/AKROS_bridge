# 概要
ロボット(2020年度単脚プロトタイプ)のコントローラ，セットアップ関連をまとめたパッケージ．  
## フォルダ
・config -> ロボットの設定，パラメータを記載したファイル  
・include -> ヘッダファイル集  
・launch -> launchファイル集  
・src -> 実装ファイル集

# 詳細
src/Prototype2020内にコントローラが記載されたファイル．
基本的にPrototype2020_BaseControllerクラスを継承して作成．
## コントローラ一覧
・JointsMove.cpp = 関節を目標角度まで動かす  
・LegMove.cpp = 脚先を任意の軌道に追従させる  
・Prototype2020_BaseController.cpp = コントローラの親クラス  
・WheelMove.cpp = 車輪移動  
・WheelMove_teleop.cpp = キーボード入力で操作して車輪移動  
・bending_stretch.cpp = 屈伸運動  
・bending_stretch_offset.cpp = x軸方向に脚先位置をずらした屈伸運動  
・initialPose.cpp = 初期姿勢に戻す  
（・check_rosparam.cpp = rosparamに関するテスト．役に立つかもしれないので載せた）