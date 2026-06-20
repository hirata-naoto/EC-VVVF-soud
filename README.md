# Elecrow 5-inch LovyanGFX VT100 serial monitor

Elecrow 5インチ ESP32-S3 RGBディスプレイ向けの、LovyanGFXベース VT100 風シリアルモニタです。USB CDC と UART1 をブリッジしつつ、ANSI/VT100 系エスケープシーケンスを画面に描画します。

## 構成

- ボード: Elecrow 5-inch ESP32-S3 RGB display (800x480)
- 描画: LovyanGFX
- フレームワーク: Arduino / PlatformIO
- シリアル: USB CDC ↔ UART1(GPIO44 RX / GPIO43 TX)

## 実装済み機能

- 800x480 RGB パネルの LovyanGFX 初期化
- GT911 タッチ対応パネル設定
- 66x30 文字グリッドのターミナル描画
- ANSI/VT100 の基本制御
  - 文字表示
  - CR / LF / BS / TAB
  - `ESC [ A/B/C/D` カーソル移動
  - `ESC [ H` / `ESC [ f` カーソル位置指定
  - `ESC [ J` / `ESC [ K` 画面・行クリア
  - `ESC [ m` での基本色変更
  - `ESC 7` / `ESC 8`, `ESC [ s` / `ESC [ u`
  - `ESC [ ?25h` / `ESC [ ?25l` カーソル表示制御
- USB CDC から UART1 への送信とローカルエコー

## ビルド

```bash
pio run
```

## 書き込みとモニタ

```bash
pio run -t upload
pio device monitor
```

## 調整ポイント

- UART の速度変更: `src/main.cpp` の `kTargetBaudRate`
- UART ピン変更: `src/main.cpp` の `kTargetRxPin` / `kTargetTxPin`
- ローカルエコー無効化: `kLocalEcho = false`

## 注意

- Elecrow 5-inch 系でも基板リビジョン差分があるため、表示やタッチが合わない場合は `include/LGFX_Elecrow_5inch.hpp` のピン設定を見直してください。
- 現状は表示中心のシリアルモニタで、タッチキーボードは未実装です。
