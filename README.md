# EC-VVVF-soud

電車（電気車）のVVVFインバータサウンドをマイコン（RP2040）で再現するプロジェクトです。  
A project to reproduce the VVVF inverter sound of electric trains on the RP2040 microcontroller.

---

## 概要 / Overview

VVVFインバータ（可変電圧可変周波数制御）は、現代の電車の駆動装置に広く使われています。  
このプロジェクトでは、**SPWM（正弦波パルス幅変調）** を用いてインバータが発する独特のサウンドをソフトウェアで生成し、I2S対応DACスピーカーから出力します。

VVVF (Variable Voltage Variable Frequency) inverters are widely used in modern electric trains.  
This project generates the characteristic VVVF inverter sound using **SPWM (Sinusoidal Pulse Width Modulation)** in software and outputs it via an I2S DAC.

---

## 動作原理 / How It Works

### SPWMによるサウンド生成

VVVFインバータの音は、**信号波（サイン波）** と **キャリア波（三角波）** を比較することで生成されます。

1. **信号波（サイン波）**：モーターに印加したい電圧の波形。周波数は列車の速度（マスコン位置）に比例します。
2. **キャリア波（三角波）**：スイッチングの基準となる高周波三角波。
3. 両者を比較し、信号波 > 三角波 のとき HIGH、それ以外は LOW のパルスを生成します（PWM）。

これにより、実際のVVVFインバータのスイッチング音に近い波形が再現されます。

### パルスモード切替

実機のVVVFインバータは速度に応じてキャリア周波数を切り替えます（非同期→多パルス同期）。  
このコードでも同様に、信号波周波数に応じて以下のモードを切り替えます：

| 信号波周波数 | モード | キャリア周波数 |
|---|---|---|
| 0〜25 Hz | 非同期（800Hz固定＋可変） | 800 + f×12 Hz |
| 25〜55 Hz | 9パルス同期 | f × 9 Hz |
| 55〜95 Hz | 5パルス同期 | f × 5 Hz |
| 95 Hz〜 | 3パルス（最低保証） | f × 3 Hz |

---

## ハードウェア構成 / Hardware

| 部品 | 説明 |
|---|---|
| **RP2040** (Raspberry Pi Pico など) | マイコン本体 |
| **I2S DAC モジュール** (MAX98357A など) | デジタルオーディオ出力 |
| **可変抵抗 / マスコン** | 速度入力（ADC） |
| スピーカー | サウンド出力 |

### ピン配置 / Pin Assignment

| RP2040ピン | 機能 |
|---|---|
| GP2 | I2S LRCK（LR クロック） |
| GP3 | I2S BCLK（ビットクロック） |
| GP4 | I2S DIN（データ） |
| GP26 | ADC0（マスコン入力） |

---

## 使い方 / Usage

1. [MicroPython](https://micropython.org/) をRP2040に書き込みます。
2. `src/ec-vvvf-sound.py` をRP2040に転送します（Thonny IDEまたは`mpremote`が便利です）。
3. RP2040を起動すると自動的にサウンドが再生されます。
4. GP26に接続した可変抵抗（マスコン）を操作して速度を変化させると、VVVFサウンドが変化します。

---

## ソフトウェア構成 / Software Structure

```
src/
└── ec-vvvf-sound.py   # メインスクリプト（SPWM生成・I2S出力）
```

### 主な処理フロー

```
起動
 │
 ├─ I2Sの初期化（GP2/3/4、22050Hz、16bit、モノラル）
 ├─ サイン波テーブルの生成（4096点）
 │
 └─ メインループ
       ├─ A. ADCでマスコン値読み取り → 信号波周波数を更新
       ├─ B. 信号波周波数に応じてキャリア周波数を決定
       ├─ C. SPWMバッファを生成（約20ms分、441サンプル）
       └─ D. I2Sへバッファを転送して出力
```
