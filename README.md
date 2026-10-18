# gnss-logger

M5Stack CoreS3 + Module GNSS (u-blox NEO-M9N) を、[RaceChrono](https://racechrono.com/) の
**BLE DIY GPS デバイス**として使うためのファームウェアです。

- UBX-NAV-PVT / UBX-NAV-DOP を直接パースし、RaceChrono BLE DIY API (service `0x1FF8`) で配信
- 測位レート 10Hz
- NMEA を使わないので、使用衛星数が 12 で頭打ちにならない（マルチ GNSS で 20 個超もそのまま表示）
- 画面に BLE 接続状態・衛星数・更新レート・緯度経度・速度を表示

## ハードウェア

| 部品 | 備考 |
| --- | --- |
| M5Stack CoreS3 | |
| M5Stack Module GNSS (NEO-M9N) | CoreS3 の下に積む |

GNSS とは UART1 で接続しています（`src/main.cpp` の定数で変更可）。

| CoreS3 | GNSS |
| --- | --- |
| GPIO18 (RX) | TX |
| GPIO17 (TX) | RX |

起動時に受信機のボーレートを 38400 → 115200 に引き上げ、NMEA 出力を止めて
UBX-NAV-PVT / NAV-DOP を毎エポック出すよう設定します。設定は RAM のみに書くので、
電源を切れば工場出荷状態に戻ります。

## ビルドと書き込み

[PlatformIO](https://platformio.org/) を使います。

```sh
pio run -t upload
pio device monitor
```

`platformio.ini` の `upload_port` / `monitor_port` は `COM3` に固定しているので、
環境に合わせて書き換えるか、行ごと削除して自動検出に任せてください。

## RaceChrono での使い方

1. デバイスの電源を入れる（画面に `BLE waiting` と表示される）
2. RaceChrono の設定 → 他のデバイス → デバイスを追加 → **RaceChrono DIY** → Bluetooth LE
3. `M5-RC-GPS` を選ぶ
4. 接続すると画面が `BLE connected` に変わる

初回接続時にスマホ側でペアリングの確認が出たら許可してください（PIN 入力は不要）。
ペアリング情報はデバイスのフラッシュ (NVS) に保存されるので、2 回目以降や
電源を入れ直したあとはペアリングなしで再接続できます。

フラッシュを消去して書き込み直した場合はデバイス側の鍵が消えるので、
スマホの Bluetooth 設定から `M5-RC-GPS` を一度削除してから接続し直してください。

## 参考

- [RaceChrono BLE DIY device API](https://github.com/aollin/racechrono-ble-diy-device)
- u-blox M9 SPG Interface Description (UBX プロトコル仕様)

## ライセンス

[MIT License](LICENSE)
