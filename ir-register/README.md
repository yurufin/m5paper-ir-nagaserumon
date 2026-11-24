# ir-register

M5Paper + IR Unit (U002, PORT.B) で赤外線リモコン信号を受信し、生のパルス幅(µs)で記録する。
流せるもん(CWA-67B)の洗浄信号を取り、`ir-timer` で送り返すための前段。

## 書き込み

```
cd ir-register
pio run -t upload --upload-port COMx
pio device monitor -p COMx
```

## 使い方

1. 起動すると受信ピンを自動判定(受信モジュールは待機中 HIGH)。画面に `受信 G33 / 送信 G26(自動判定)` と出れば OK
2. 覚えさせたいリモコンを受光部から 5〜15cm で押す
3. 受信ごとに
   - シリアルに `uint16_t rawData[N] = {...};`
   - SD の `/ir/capNNN.txt` に保存
   - 画面に長さ・全長・フレーム数と、**直前の受信と一致したか**
4. 3 回ほど押して「一致」が続いたら、**押込ボタン(G38)** で最後の信号を採用 → `/ir/flush.txt`

## 仕組み

- IRremoteESP8266 は Arduino コア 3.x(espressif32 8.0.0)でビルドできないため使わず、GPIO の CHANGE 割り込みでエッジ時刻を取る
- 最初の立ち下がりから 150ms 無信号になるまでを 1 回分とする(リピートフレームも含めてそのまま記録)。10ms 以上のスペースをフレーム区切りとして数える
- 一致判定は各値 ±25%(最低 ±150µs)、長さが違えば不一致
- 最大 2048 エッジ。超えたら `overflow=1`

## ファイル形式(`/ir/capNNN.txt`, `/ir/flush.txt`)

```
# ir-register cap003
# rx_pin=33 len=211 total_us=123456 frames=2 overflow=0
3400,1700,450,400,...
```

`#` 行はコメント。データはカンマ区切りの µs で、マーク(発光)から始まり マーク/スペース が交互。
