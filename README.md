# AKROS_bridgeとは？
T-motor社が販売している「A Series Dynamical Modular」モータをROS1で動かせるようにするパッケージ．  
https://store-en.tmotor.com/category.php?id=97

動かすには別途マイコン（STM32F446RE）が必要．

# ノードとその詳細
## converter
モータドライバに搭載されたマイコンに送信する値はデジタル値であるが，コントローラ上では
実数値（float, double）を使用するほうが望ましい．  
よって，その変換をこのノードで行う．  
