# RB・R北条"コキ"のプログラムを上げるところ
main.cpp：メイン  
lib：改造したsken_libraryのファイル

note:
- sken_system.canTransmit()を毎1msに行うとSTM Studioとの通信が行えない。
- このプログラムでは10ms毎に送信をしており、STM Studioの動作を確認した。
- 定数「DIAMETER」は、誤差が出て使用できなかったため、代わりに円周を表す定数「TEST」を使用している。
- モーターの進んだ距離は400mmで5mmの誤差が出た（相対誤差1.25%）。この調子だとコンパス/ジャイロが必要かも知れない。