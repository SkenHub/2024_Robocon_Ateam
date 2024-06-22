<!-- リファレンスのテンプレート -->
<!-- readmeにライブラリのリファレンスを記載 -->

# myuart

R1のUART通信を簡単にするためのクラス。  
モジュール基盤以外での使用は想定されていない。  
通信自体はsken_libraryのUartクラスで行っている。

# enum USBname
* USB_A
* USB_B

# class MyUart
## MyUart(void)
コンストラクタ。テンプレート引数に受信するデータ部（通信開始部と処理部を除く）のバイト数を渡す。  
[サンプルコード]  
void MyUart::init(USBname usb, uint32_t baudrate)のサンプルコードを参照

## void MyUart::init(USBname usb, uint32_t baudrate)
シリアル通信の初期化を行う関数。自動でDMA受信を開始する。  
[パラメータ]  
usb: 使用するUSBポート  
baudrate: 転送レート。単位は[bps]

[戻り値]  
なし

[サンプルコード]  
USB_Aをボーレート9600で設定する  
12バイトのデータ部を受信する
``` c++
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

MyUart<12> serial;

int main(void) {
    sken_system.init();
    serial.init(USB_A, 9600)

    while (true) {}
}
```

## bool MyUart::read(uint8_t* container)
受信したデータを配列に代入する関数。データ部のみ代入される。  
containerの大きさはコンストラクタに渡したテンプレート引数と同じでなくてはならない。  
新しいデータを受信していたらtrue、していなかったらfalseを返す。

[パラメータ]  
受信データ配列の先頭アドレス

[戻り値]  
受信成功フラグ

[サンプルコード]
``` c++
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

MyUart<12> serial;

uint8_t data[12];

int main(void)
{
    sken_system.init();
    serial.init(USB_A, 9600);

    while (true) {
        serial.read(data);
    }
}
```

## void Uart::write(uint8_t* data, uint8_t size)
ポーリングモードでデータを送信する関数。  
コンストラクタに渡したテンプレート引数は関係無い。  
デッドタイムは100[ms]。

[パラメータ]  
送信データ配列の先頭アドレス  
送信データ配列の大きさ  

[戻り値]  
なし

[サンプルコード]
``` c++
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

MyUart<12> serial;

uint8_t data[2] = {1, 2};

int main(void)
{
    sken_system.init();
    serial.init(USB_A, 9600);

    while (true) {
        serial.write(data, 2);
    }
}
```