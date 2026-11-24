# m5paper-ir

M5Paper(初代, ESP32)と M5Stack IR Unit (U002) で、赤外線リモコンの信号を **覚えて、決まった時刻に送る**。

もとは LIXIL「流せるもん」(CWA-67B)で無人のトイレを定期的に洗浄するために作ったが、
信号は生のパルス幅(µs)で記録・再生するので、38kHz 系の赤外線リモコンならたいてい使える。

| フォルダ | 役割 |
| :--- | :--- |
| [ir-register](ir-register/) | リモコン信号を受信して SD に記録。何回か受けて一致を確かめ、使う信号を `/ir/flush.txt` に採用する |
| [ir-timer](ir-timer/) | `/ir/flush.txt` を予定時刻(既定は 1 日 4 回)に送る。タッチボタン・本体ボタンで手動送信も可 |

## ハードウェア

- M5Paper(初代, ESP32)
- M5Stack IR Unit (U002) を **PORT.B**(G26 / G33)に接続
  - 受信側・送信側のどちらがどのピンかは起動時に自動判定(受信モジュールは待機中に HIGH を出すため)
  - Grove の 5V は G5 を HIGH にして給電
- microSD(信号・予定・ログ・Wi-Fi 設定を置く)

## 使い方

1. `ir-register` を書き込み、リモコンを受光部から 5〜15cm で数回押す。「一致」が続いたら押込ボタン(G38)で採用
2. 同じ SD のまま `ir-timer` を書き込む
3. 必要なら SD に次を置く
   - `/ir/schedule.txt`: 送信時刻を `HH:MM` で 1 行ずつ
   - `/ssid.txt`: `SSID,PASSWORD`(NTP で時刻を合わせる。無ければ RTC の時刻のまま)

## SD の中身

```
/ssid.txt            Wi-Fi(任意)
/ir/capNNN.txt       ir-register が受信ごとに保存
/ir/flush.txt        採用した信号(ir-timer が送る)
/ir/schedule.txt     送信時刻(任意)
/ir/log.txt          ir-timer の実行履歴
```

## 開発環境

PlatformIO(espressif32 / Arduino コア 3.x)、M5Unified + M5GFX。
IRremoteESP8266 は Arduino コア 3.x でビルドできないため使わず、受信は GPIO 割り込み、送信は RMT で実装している。

```
cd ir-register   # または ir-timer
pio run -t upload --upload-port COMx
```
