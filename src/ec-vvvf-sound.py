import math
import struct
from machine import Pin, ADC, I2S

# ==========================================
# 1. I2S・ハードウェアの初期設定
# ==========================================
SAMPLE_RATE = 22050

# ==========================================
# ポート・ピン番号の定数定義
# ==========================================
I2S_PORT    = 0   # I2S ポート番号
PIN_I2S_SCK = 3   # BCLK
PIN_I2S_WS  = 2   # LRCK
PIN_I2S_SD  = 4   # DIN
PIN_MASCON  = 26  # マスコン ADC 入力

audio_out = I2S(
    I2S_PORT,
    sck=Pin(PIN_I2S_SCK),
    ws=Pin(PIN_I2S_WS),
    sd=Pin(PIN_I2S_SD),
    mode=I2S.TX,
    bits=16,
    format=I2S.MONO,
    rate=SAMPLE_RATE,
    ibuf=4096     # [修正] バッファを2048→4096に拡大して転送安定性を向上
)

mascon_adc = ADC(Pin(PIN_MASCON))

# ==========================================
# 2. サイン波テーブル（信号波用）
# ==========================================
# [最適化] TABLE_SIZE を 512→4096 に拡大。補間なしでも十分な精度が出る。
# RAM消費は約16KB（RP2040の264KBに対して問題なし）
TABLE_SIZE = 4096
TABLE_MASK = TABLE_SIZE - 1  # ビットマスク（% の代わりに & で使う）
SINE_TABLE = []
for i in range(TABLE_SIZE):
    SINE_TABLE.append(math.sin(2 * math.pi * i / TABLE_SIZE))

# ==========================================
# 3. 再生用変数の初期化
# ==========================================
# [修正] CHUNK_SIZE を明示的にサンプルレート基準で計算（約20ms分）
CHUNK_SIZE = SAMPLE_RATE // 50  # = 441サンプル

current_base_f  = 0.0   # 現在の信号波（出力）周波数
phase_base      = 0.0   # サイン波テーブル上の現在位相

# 三角波（キャリア）の状態
tri_val       = 0.0
tri_direction = 1       # 1: 上昇, -1: 下降

# [追加] キャリア周波数をスムージングするための変数
current_carrier_f = 800.0

# [追加] ADCローパスフィルタ用の初期値
adc_filtered = mascon_adc.read_u16()

print("【修正版・真SPWM】I2S VVVFシステム起動")

# ==========================================
# 4. メインループ
# ==========================================
try:
    while True:

        # ----------------------------------------------------------
        # A. マスコン読み取りと速度（周波数）計算
        # ----------------------------------------------------------

        # [修正] ADCにローパスフィルタを適用してチャタリングを抑制
        adc_raw      = mascon_adc.read_u16()
        adc_filtered = adc_filtered * 0.8 + adc_raw * 0.2

        # [修正] ヒステリシスを持たせるため、境界付近の閾値を分離
        #   ブレーキ  : adc_filtered < 25000
        #   力行      : adc_filtered > 40000
        #   惰行      : 25000 〜 40000  → 現状維持（0.99倍ではなく変化なし）
        if adc_filtered < 25000:
            target_base_f = 0.0
            change_rate   = 1.0          # ブレーキ：速やかに0Hzへ
        elif adc_filtered > 40000:
            target_base_f = ((adc_filtered - 40000) / 25535.0) * 150.0
            change_rate   = 0.4          # 力行：加速
        else:
            # [修正] 惰行は現状維持。毎ループ*0.99して減速するバグを修正
            target_base_f = current_base_f
            change_rate   = 0.0

        # 現在周波数をターゲットへスルーレート制限しながら追従
        if change_rate > 0.0:
            if current_base_f < target_base_f:
                current_base_f = min(current_base_f + change_rate, target_base_f)
            elif current_base_f > target_base_f:
                current_base_f = max(current_base_f - change_rate, target_base_f)
        # change_rate == 0.0（惰行）のときは current_base_f を変えない

        # ----------------------------------------------------------
        # B. キャリア（三角波）目標周波数の決定
        # ----------------------------------------------------------
        if current_base_f < 25.0:
            target_carrier_f = 800.0 + (current_base_f * 12.0)  # 非同期
        elif current_base_f < 55.0:
            target_carrier_f = current_base_f * 9.0              # 9パルス同期
        elif current_base_f < 95.0:
            target_carrier_f = current_base_f * 5.0              # 5パルス同期
        else:
            # [修正] 1パルスモードでは信号波と同周波数→音量激減するため
            #        実機同様に最低でも3パルス相当を維持する
            target_carrier_f = current_base_f * 3.0              # 3パルス（最低保証）

        # [修正] キャリア周波数を急変させずスムージングしてクリックノイズを防止
        current_carrier_f += (target_carrier_f - current_carrier_f) * 0.05

        # ----------------------------------------------------------
        # C. オーディオバッファ生成（SPWM：サイン波 vs 三角波の比較）
        # ----------------------------------------------------------
        buffer = bytearray(CHUNK_SIZE * 2)

        if current_base_f > 0.5:
            # サイン波テーブルの1サンプルあたり進む量
            step_base = (TABLE_SIZE * current_base_f) / SAMPLE_RATE

            # 三角波が1往復（0→1→0）するのに必要な1サンプルあたりの変化量
            step_tri = (2.0 * current_carrier_f) / SAMPLE_RATE

            for i in range(CHUNK_SIZE):

                # 1. サイン波（信号波）の値を取得
                # [最適化] TABLE_SIZE=4096で精度確保 → 補間不要
                # [最適化] % 除算 → & ビットマスクに変更（高速）
                sine_val = SINE_TABLE[int(phase_base) & TABLE_MASK]

                # 2. 三角波（キャリア）の値を -1.0〜1.0 にマッピング
                triangle_val = (tri_val * 2.0) - 1.0

                # 3. 比較してPWM（方形波）を生成
                if sine_val > triangle_val:
                    sample = 12000    # HIGH
                else:
                    sample = -12000   # LOW

                struct.pack_into('<h', buffer, i * 2, sample)

                # 4. 位相・三角波状態を次サンプルへ更新
                phase_base = (phase_base + step_base) % TABLE_SIZE

                tri_val += step_tri * tri_direction
                if tri_val >= 1.0:
                    tri_val       = 1.0
                    tri_direction = -1
                elif tri_val <= 0.0:
                    tri_val       = 0.0
                    tri_direction = 1

        # buffer は停車時ゼロ初期化のままなので無音になる

        # ----------------------------------------------------------
        # D. I2Sへ転送
        # ----------------------------------------------------------
        # [修正] time.sleep() を削除 — write() 自体がブロッキングするため
        #        sleep を重ねると音途切れの原因になる
        audio_out.write(buffer)

except KeyboardInterrupt:
    audio_out.deinit()
    print("停止しました")
