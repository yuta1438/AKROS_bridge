# このフォルダについて  
マイコンに書き込むファームウェア．  

# 環境構築
gcc4mbedとrosserial_mbedを使って開発を行いました．  
方法は下の記事を作成しましたので，それに従って下さい．  
https://qiita.com/FAL19/items/98f6d7d6e0bf2517da2c
  
# フォルダ構成
・AKROS_bridge_mbed/AKROS_bridge_mbed.cpp  
 -> main関数が記載されたメインのファイル  
   
・include/**
 -> ヘッダファイル集  
   
・AKROS_bridge_mbed/**
 -> 関数，クラスの実装部