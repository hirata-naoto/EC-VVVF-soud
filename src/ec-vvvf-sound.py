import math
import struct
from array import array
from machine import Pin, ADC, I2S

# ==========================================
# 1. I2S・ハードウェアの初期設定
# ==========================================
SAMPLE_RATE = 22050

# ==========================================
# ポート・ピン番号の定数定義
# ==========================================
I2S_PORT    = 0   # I2S ポート番号
PIN_I2S_SCK = 2   # BCLK
PIN_I2S_WS  = 3   # LRCK
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
    ibuf=4096
)

mascon_adc = ADC(Pin(PIN_MASCON))

# ==========================================
# 2. サイン波テーブル（信号波用）
# ==========================================
# [修正] list(float) はメモリ消費が大きく MemoryError の原因になりやすい。
#       int16 配列(array('h'))で保持してRAM使用量を削減する。
TABLE_SIZE = 1024
TABLE_MASK = TABLE_SIZE - 1
SINE_SCALE = 32767
SINE_TABLE = array('h', [0] * TABLE_SIZE)
for i in range(TABLE_SIZE):
    SINE_TABLE[i] = int(math.sin(2 * math.pi * i / TABLE_SIZE) * SINE_SCALE)

# ==========================================
# 3. 再生用変数の初期化
# ==========================================
CHUNK_SIZE = SAMPLE_RATE // 50  # = 441サンプル
BUFFER_SIZE = CHUNK_SIZE * 2
buffer = bytearray(BUFFER_SIZE)
SILENT_BUFFER = bytes(BUFFER_SIZE)

current_base_f  = 0.0
phase_base      = 0.0

tri_val       = 0.0
tri_direction = 1

current_carrier_f = 800.0
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
        adc_raw      = mascon_adc.read_u16()
        adc_filtered = adc_filtered * 0.8 + adc_raw * 0.2

        if adc_filtered < 25000:
            target_base_f = 0.0
            change_rate   = 1.0
        elif adc_filtered > 40000:
            target_base_f = ((adc_filtered - 40000) / 25535.0) * 150.0
            change_rate   = 0.4
        else:
            target_base_f = current_base_f
            change_rate   = 0.0

        if change_rate > 0.0:
            if current_base_f < target_base_f:
                current_base_f = min(current_base_f + change_rate, target_base_f)
            elif current_base_f > target_base_f:
                current_base_f = max(current_base_f - change_rate, target_base_f)

        # ----------------------------------------------------------
        # B. キャリア（三角波）目標周波数の決定
        # ----------------------------------------------------------
        if current_base_f < 25.0:
            target_carrier_f = 800.0 + (current_base_f * 12.0)
        elif current_base_f < 55.0:
            target_carrier_f = current_base_f * 9.0
        elif current_base_f < 95.0:
            target_carrier_f = current_base_f * 5.0
        else:
            target_carrier_f = current_base_f * 3.0

        current_carrier_f += (target_carrier_f - current_carrier_f) * 0.05

        # ----------------------------------------------------------
        # C. オーディオバッファ生成（SPWM）
        # ----------------------------------------------------------
        if current_base_f > 0.5:
            step_base = (TABLE_SIZE * current_base_f) / SAMPLE_RATE
            step_tri = (2.0 * current_carrier_f) / SAMPLE_RATE

            for i in range(CHUNK_SIZE):
                # [修正] int16 テーブル値を -1.0〜1.0 に正規化して比較
                sine_val = SINE_TABLE[int(phase_base) & TABLE_MASK] / SINE_SCALE
                triangle_val = (tri_val * 2.0) - 1.0

                if sine_val > triangle_val:
                    sample = 12000
                else:
                    sample = -12000

                struct.pack_into('<h', buffer, i * 2, sample)

                phase_base = (phase_base + step_base) % TABLE_SIZE

                tri_val += step_tri * tri_direction
                if tri_val >= 1.0:
                    tri_val       = 1.0
                    tri_direction = -1
                elif tri_val <= 0.0:
                    tri_val       = 0.0
                    tri_direction = 1
        else:
            buffer[:] = SILENT_BUFFER

        # ----------------------------------------------------------
        # D. I2Sへ転送
        # ----------------------------------------------------------
        audio_out.write(buffer)

except KeyboardInterrupt:
    audio_out.deinit()
    print("停止しました")
