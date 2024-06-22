# Communication for R1
R1の通信備忘録

## 配線
　AはUSB-A(A9,A10,SERIAL1)、BはUSB-B(C10,C11,SERIAL3)の意  
　　USB-B  
　*Board-MDD1(undercarriage)*  
　　USB-A  
　　　↕  
　　USB-B  
　*Board-DD*　USB-miniB  ↔ *PC*  
　　USB-A  
　　　↕  
　　USB-B  
　*Board-Sensor*  
　　USB-A  
　　　↕  
　　USB-B  
　*Board-MDD2(launcher)*

## Board-DD --> PC
送信情報   
　ロボットの自己位置（x[mm], y[mm], θ[deg]）  
　フィールド（青0, 赤1）  
　動作番号の送り返し

通信開始部　2Byte  
　0xA5  
　0xA5  
データ部　14Byte  
　x[mm] : float = uint8_t[4]  
　y[mm] : float = uint8_t[4]  
　θ[deg] : float = uint8_t[4]  
　フィールド : uint8_t  
　現動作番号 : uint8_t

## Board-DD <-- PC
送信情報  
　移動指令（速度[m/s], 方向[deg], 角度[deg]（方向は絶対座標））  
　フィールド（青0, 赤1）  
　動作番号  

通信開始部　2Byte  
　0xA5  
　0xA5  
データ部　14Byte  
　速度[m/s] : float = uint8_t[4]  
　方向[deg] : float = uint8_t[4]  
　角度[deg] : float = uint8_t[4]  
　フィールド : uint8_t  
　動作番号 : uint8_t  

## Board-DD --> Board-MDD1  
送信情報  
　速度[m/s], 方向[deg], 角度[deg]（方向は絶対座標）

通信開始部　2Byte  
　0xA5  
　0xA5  
データ部　12Byte  
　速度[m/s] : float = uint8_t[4]  
　方向[deg] : float = uint8_t[4]  
　角度[deg] : float = uint8_t[4]  

## Board-DD <-- Board-MDD1
送信情報   

通信開始部　0Byte  
データ部　0Byte  

## Board-DD --> Board-Sensor
送信情報   
　上機構のモーター情報（実装予定）  

通信開始部　nByte  
データ部　nByte  

## Board-DD <-- Board-Sensor
送信情報   
　ロボットの自己位置（x[mm], y[mm], θ[deg]）  

通信開始部　2Byte  
　0xA5  
　0xA5  
データ部　12Byte  
　x[mm] : float = uint8_t[4]  
　y[mm] : float = uint8_t[4]  
　θ[deg] : float = uint8_t[4]  

## Board-Sensor --> Board-MDD2
送信情報   
　上機構のモーター情報（実装予定）  

通信開始部　nByte  
データ部　nByte  

## Board-Sensor <-- Board-MDD2
送信情報  

通信開始部　0Byte  
データ部　0Byte  


Created by Tikuwa404