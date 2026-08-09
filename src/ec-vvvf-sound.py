import math
import struct
from array import array
from machine import Pin, ADC, I2S

# esp32c5 電車のVVVFインバータ音再現
# ==========================================
# 1. I2S・ハードウェアの初期設定
# ==========================================
SAMPLE_RATE = 22050

# ==========================================
# ポート・ピン番号の定数定義（ESP32-C5用）
# ==========================================
I2S_PORT    = 0   # I2S ポート番号
PIN_I2S_SCK = 6   # BCLK
PIN_I2S_WS  = 7   # LRCK
PIN_I2S_SD  = 8   # DIN
PIN_MASCON  = 4   # マスコン ADC 入力

audio_out = I2S(
    I2S_PORT,
    sck=Pin(PIN_I2S_SCK),
    ws=Pin(PIN_I2S_WS),
    sd=Pin(PIN_I2S_SD),
    mode=I2S.TX,
    bits=16,
    format=I2S.MONO,
    rate=SAMPLE_RATE,
    ibuf=32768
)

mascon_adc = ADC(Pin(PIN_MASCON))
_mascon_adc_atten = getattr(ADC, "ATTN_12DB", getattr(ADC, "ATTN_11DB", None))
if _mascon_adc_atten is not None:
    mascon_adc.atten(_mascon_adc_atten)
else:
    print("警告: ADC attenuation設定をサポートしていません")

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
CHUNK_SIZE = SAMPLE_RATE // 10  # = 2205サンプル（バッファアンダーフロー対策）
BUFFER_SIZE = CHUNK_SIZE * 2
buffer = bytearray(BUFFER_SIZE)
SILENT_BUFFER = bytes(BUFFER_SIZE)

current_base_f  = 0.0
phase_base      = 0.0

tri_val       = 0.0
tri_direction = 1

current_carrier_f = 800.0
adc_filtered = mascon_adc.read_u16()

# デバッグ用：前回値（変化検知に使用）
_dbg_prev_adc      = -1
_dbg_prev_base_f   = -1
_dbg_prev_carrier_f= -1.0
_dbg_prev_tri_dir  = 0

print("【ESP32-C5対応版・真SPWM】I2S VVVFシステム起動")

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
            # ブレーキ／停止ゾーン：基底周波数をゼロへ引き下げる
            target_base_f = 0.0
            change_rate   = 1.0
            if current_base_f > target_base_f:
                current_base_f = max(current_base_f - change_rate, target_base_f)
        elif adc_filtered > 40000:
            # 力行ゾーン：ADC値に応じた目標周波数へ追従
            target_base_f = ((adc_filtered - 40000) / 25535.0) * 150.0
            change_rate   = 0.4
            if current_base_f < target_base_f:
                current_base_f = min(current_base_f + change_rate, target_base_f)
            elif current_base_f > target_base_f:
                current_base_f = max(current_base_f - change_rate, target_base_f)
        # else: デッドゾーン（25000〜40000）は current_base_f を保持

        # ----------------------------------------------------------
        # B. キャリア（三角波）目標周波数の決定
        # ----------------------------------------------------------
        # キャリア周波数：各区間境界で連続になるよう線形補間
        # f=0〜25Hz: 1100Hz固定（低速域は高PWM周波数）
        # f=25〜55Hz: 1100Hz→275Hz（線形降下）
        # f=55〜95Hz: 275Hz→190Hz（線形降下）
        # f≥95Hz: f×2.0Hz（比例）
        if current_base_f < 25.0:
            target_carrier_f = 1100.0
        elif current_base_f < 55.0:
            target_carrier_f = 1100.0 - (current_base_f - 25.0) * (825.0 / 30.0)
        elif current_base_f < 95.0:
            target_carrier_f = 275.0 - (current_base_f - 55.0) * (85.0 / 40.0)
        else:
            target_carrier_f = current_base_f * 2.0

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

        # ----------------------------------------------------------
        # E. デバッグ出力（変化があったときのみ）
        # ----------------------------------------------------------
        adc_int = int(adc_filtered)
        base_f_int = int(current_base_f * 10)
        carrier_f_int = int(current_carrier_f)
        if (adc_int != _dbg_prev_adc or
                base_f_int != _dbg_prev_base_f or
                carrier_f_int != _dbg_prev_carrier_f or
                tri_direction != _dbg_prev_tri_dir):
            print(
                "ADC={:5d} base_f={:5.1f}Hz carrier_f={:5.0f}Hz "
                "tri_val={:.3f} tri_dir={:+d}".format(
                    adc_int, current_base_f, current_carrier_f,
                    tri_val, tri_direction
                )
            )
            _dbg_prev_adc       = adc_int
            _dbg_prev_base_f    = base_f_int
            _dbg_prev_carrier_f = carrier_f_int
            _dbg_prev_tri_dir   = tri_direction

except KeyboardInterrupt:
    audio_out.deinit()
    print("停止しました")
except Exception as e:
    audio_out.deinit()
    print("エラー:", e)
