"""
EC-VVVF-soud: 電気車VVVFインバータサウンド再現プログラム
=============================================================
このスクリプトはRP2040（MicroPython）上で動作し、電車のVVVFインバータが
発する特徴的なサウンドをSPWM（正弦波パルス幅変調）で生成してI2S DACへ出力します。

【サウンド生成の仕組み】
  VVVFインバータの音は「信号波（サイン波）」と「キャリア波（三角波）」の
  2つの波形を比較（SPWM）することで生成します。
    - 信号波 > キャリア波 → HIGH (+12000)
    - 信号波 ≤ キャリア波 → LOW  (-12000)
  この比較結果のパルス列がインバータのスイッチング音に相当します。

  信号波の周波数はマスコン（可変抵抗）入力で制御し、それに連動して
  キャリア周波数もパルスモード切替（非同期→9→5→3パルス）します。

【ピン配置】
  GP2  : I2S LRCK（LRクロック）
  GP3  : I2S BCLK（ビットクロック）
  GP4  : I2S DIN（データ出力）
  GP26 : ADC0（マスコン入力、可変抵抗など）
"""

import math
import struct
from machine import Pin, ADC, I2S

# ==========================================
# 1. I2S・ハードウェアの初期設定
# ==========================================
# サンプルレート 22050Hz（CD品質の半分）。RP2040で安定して動作する値。
SAMPLE_RATE = 22050

# I2S（Inter-IC Sound）インターフェースの初期化
# I2S は DAC（デジタル→アナログ変換）モジュールとの通信に使用する
audio_out = I2S(
    0,
    sck=Pin(3),   # BCLK: ビットクロック（各ビットに同期）
    ws=Pin(2),    # LRCK: 左右チャンネル切替クロック（モノラルでもLRCKは必要）
    sd=Pin(4),    # DIN : シリアルデータ出力
    mode=I2S.TX,
    bits=16,      # 16bitサンプル（-32768〜32767の範囲）
    format=I2S.MONO,
    rate=SAMPLE_RATE,
    ibuf=4096     # バッファを4096バイトに設定（転送安定性を確保）
)

# マスコン（速度操作器）代わりの可変抵抗をADCで読み取る
# GP26 = ADC0 (0〜65535の16bit値)
mascon_adc = ADC(Pin(26))

# ==========================================
# 2. サイン波テーブル（信号波用）
# ==========================================
# あらかじめサイン波の値を配列に格納しておく（ウェーブテーブル方式）。
# ループ中に毎回 math.sin() を呼ぶと遅いため、この方法で高速化する。
#
# TABLE_SIZE=4096: 1周期を4096点で表現。精度が高く補間処理が不要。
# RAM消費: 4096 × 4byte(float) ≈ 16KB（RP2040の264KBに対して問題なし）
TABLE_SIZE = 4096
TABLE_MASK = TABLE_SIZE - 1  # ビットマスク（インデックスの剰余を & 演算で高速化）
SINE_TABLE = []
for i in range(TABLE_SIZE):
    SINE_TABLE.append(math.sin(2 * math.pi * i / TABLE_SIZE))

# ==========================================
# 3. 再生用変数の初期化
# ==========================================
# 1チャンク = 約20ms分のサンプル数（SAMPLE_RATE/50 = 441サンプル）
# メインループの1反復でこのサイズのバッファを生成してI2Sに送る
CHUNK_SIZE = SAMPLE_RATE // 50  # = 441サンプル ≈ 20ms

current_base_f  = 0.0   # 現在の信号波（モーター周波数）[Hz]。列車速度に相当。
phase_base      = 0.0   # サイン波テーブル上の現在読み出し位置（0〜TABLE_SIZE）

# キャリア（三角波）の状態変数
# tri_val: 三角波の現在値（0.0〜1.0）。-1.0〜1.0 にマッピングして使用する。
# tri_direction: 1=上昇中, -1=下降中
tri_val       = 0.0
tri_direction = 1

# キャリア周波数をスムージングするための変数（急変によるクリックノイズ防止）
current_carrier_f = 800.0

# ADCローパスフィルタの初期値（起動直後の急変を防ぐ）
adc_filtered = mascon_adc.read_u16()

print("【SPWM】I2S VVVFシステム起動")

# ==========================================
# 4. メインループ
# ==========================================
try:
    while True:

        # ----------------------------------------------------------
        # A. マスコン読み取りと速度（周波数）計算
        # ----------------------------------------------------------

        # ADCにローパスフィルタ（指数移動平均）を適用してチャタリングを抑制する。
        # adc_filtered = 前回値×0.8 + 今回値×0.2 → 急激な変動を平滑化
        adc_raw      = mascon_adc.read_u16()
        adc_filtered = adc_filtered * 0.8 + adc_raw * 0.2

        # マスコン位置に応じて動作モードを切り替える。
        # ヒステリシスにより境界付近の誤動作を防止している。
        #   ブレーキ : adc_filtered < 25000 → 信号波を素早く0Hzへ
        #   力行     : adc_filtered > 40000 → 信号波周波数を目標値へ加速
        #   惰行     : 25000〜40000 → 現在の周波数を維持（変化なし）
        if adc_filtered < 25000:
            target_base_f = 0.0
            change_rate   = 1.0          # ブレーキ：1Hz/チャンクで素早く停止
        elif adc_filtered > 40000:
            # ADC値（40000〜65535）を0〜150Hzの信号波周波数に変換
            target_base_f = ((adc_filtered - 40000) / 25535.0) * 150.0
            change_rate   = 0.4          # 力行：0.4Hz/チャンクで滑らかに加速
        else:
            # 惰行：ターゲットを現在値に保ち、周波数を変化させない
            target_base_f = current_base_f
            change_rate   = 0.0

        # スルーレート制限：急激な周波数変化を抑えて滑らかな加減速を実現
        if change_rate > 0.0:
            if current_base_f < target_base_f:
                current_base_f = min(current_base_f + change_rate, target_base_f)
            elif current_base_f > target_base_f:
                current_base_f = max(current_base_f - change_rate, target_base_f)
        # change_rate == 0.0（惰行）のときは current_base_f を変えない

        # ----------------------------------------------------------
        # B. キャリア（三角波）目標周波数の決定
        # ----------------------------------------------------------
        # 実機のVVVFインバータは速度（信号波周波数）に応じてキャリア周波数を
        # 切り替える「パルスモード切替」を行う。低速時は非同期PWM、中高速では
        # 信号波周波数の整数倍に同期させることでスイッチング損失を低減する。
        if current_base_f < 25.0:
            # 非同期モード：キャリアを固定基準から徐々に上昇させる
            target_carrier_f = 800.0 + (current_base_f * 12.0)
        elif current_base_f < 55.0:
            # 9パルス同期モード：キャリア = 信号波 × 9
            target_carrier_f = current_base_f * 9.0
        elif current_base_f < 95.0:
            # 5パルス同期モード：キャリア = 信号波 × 5
            target_carrier_f = current_base_f * 5.0
        else:
            # 3パルスモード（最低保証）：1パルス相当では音量が激減するため
            # 実機同様に最低でも3パルス相当のキャリア周波数を維持する
            target_carrier_f = current_base_f * 3.0

        # キャリア周波数を急変させずスムージングしてパルスモード切替時の
        # クリックノイズ（不連続音）を防止する（一次IIRフィルタ）
        current_carrier_f += (target_carrier_f - current_carrier_f) * 0.05

        # ----------------------------------------------------------
        # C. オーディオバッファ生成（SPWM：サイン波 vs 三角波の比較）
        # ----------------------------------------------------------
        # CHUNK_SIZE × 2バイト（16bitサンプル）のバッファをゼロ初期化。
        # 停車中（current_base_f ≤ 0.5）はここの状態のまま無音になる。
        buffer = bytearray(CHUNK_SIZE * 2)

        if current_base_f > 0.5:
            # サイン波テーブルを1サンプルごとに進めるステップ量
            # step_base = TABLE_SIZE × (信号波周波数 / サンプルレート)
            step_base = (TABLE_SIZE * current_base_f) / SAMPLE_RATE

            # 三角波の1サンプルあたりの変化量
            # 三角波は0→1→0の1往復が1キャリア周期なので変化量 = 2f/fs
            step_tri = (2.0 * current_carrier_f) / SAMPLE_RATE

            for i in range(CHUNK_SIZE):

                # 1. サイン波（信号波）の値を取得（-1.0〜1.0）
                #    & TABLE_MASK で剰余演算を高速化（TABLE_SIZEは2の冪乗）
                sine_val = SINE_TABLE[int(phase_base) & TABLE_MASK]

                # 2. 三角波（キャリア）の値を 0〜1 から -1.0〜1.0 にマッピング
                triangle_val = (tri_val * 2.0) - 1.0

                # 3. SPWM比較：信号波 > 三角波 → HIGH、それ以外 → LOW
                #    これによりSPWMパルスが生成され、VVVFの音が作られる
                if sine_val > triangle_val:
                    sample = 12000    # HIGH（最大値32767の約37%）
                else:
                    sample = -12000   # LOW

                # 16bitリトルエンディアンでバッファに格納
                struct.pack_into('<h', buffer, i * 2, sample)

                # 4. 次サンプルへの位相・三角波状態を更新
                phase_base = (phase_base + step_base) % TABLE_SIZE

                tri_val += step_tri * tri_direction
                # 三角波が上限（1.0）に達したら折り返す
                if tri_val >= 1.0:
                    tri_val       = 1.0
                    tri_direction = -1
                # 三角波が下限（0.0）に達したら折り返す
                elif tri_val <= 0.0:
                    tri_val       = 0.0
                    tri_direction = 1

        # ----------------------------------------------------------
        # D. I2Sへ転送
        # ----------------------------------------------------------
        # write() はバッファを転送し終えるまでブロッキングする。
        # time.sleep() は不要（むしろ追加すると音途切れの原因になる）。
        audio_out.write(buffer)

except KeyboardInterrupt:
    # Ctrl+C で停止したときにI2Sリソースを解放する
    audio_out.deinit()
    print("停止しました")
