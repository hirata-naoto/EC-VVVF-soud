// ec-vvvf-sound.ino
// ESP32 電車のVVVFインバータ音再現（MicroPython版からの変換）
// 対象ボード: Seeed XIAO ESP32C5（または互換ESP32系ボード）
//
// 必要ライブラリ:
//   ESP32-audioI2S (またはArduinoのI2Sライブラリ)
//   Arduino IDE のボードマネージャで "esp32 by Espressif" をインストール
//
// ピン配置:
//   D8 (GPIO8)  : I2S BCLK (SCK)
//   D1 (GPIO0)  : I2S LRCK (WS)
//   D6 (GPIO11) : I2S DIN  (SD)
//   D0 (GPIO1)  : マスコン ADC 入力

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ==========================================
// 1. ハードウェア定数
// ==========================================
static const int SAMPLE_RATE   = 22050;

static const gpio_num_t PIN_I2S_SCK = GPIO_NUM_8;   // BCLK
static const gpio_num_t PIN_I2S_WS  = GPIO_NUM_0;   // LRCK
static const gpio_num_t PIN_I2S_SD  = GPIO_NUM_11;  // DIN
static const int        PIN_MASCON  = 1;             // ADC GPIO1

static const i2s_port_t I2S_PORT = I2S_NUM_0;

// ==========================================
// 2. サイン波テーブル
// ==========================================
static const int TABLE_SIZE = 1024;
static const int TABLE_MASK = TABLE_SIZE - 1;
static const int SINE_SCALE = 32767;
static int16_t SINE_TABLE[TABLE_SIZE];

// ==========================================
// 3. バッファ
// ==========================================
static const int CHUNK_SIZE   = SAMPLE_RATE / 10;   // 2205 サンプル
static const int BUFFER_SIZE  = CHUNK_SIZE * 2;      // バイト数（16bit mono）
static uint8_t audio_buffer[BUFFER_SIZE];

// ==========================================
// 4. 再生用変数
// ==========================================
static float current_base_f   = 0.0f;
static float phase_base        = 0.0f;
static float tri_val           = 0.0f;
static int   tri_direction     = 1;
static float current_carrier_f = 800.0f;
static float adc_filtered      = 0.0f;

// デバッグ用（変化検知）
static int   _dbg_prev_adc       = -1;
static int   _dbg_prev_base_f    = -1;
static int   _dbg_prev_carrier_f = -1;
static int   _dbg_prev_tri_dir   = 0;

// ==========================================
// 5. I2S 初期化
// ==========================================
static void i2s_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,  // MONO
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 512,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);

    i2s_pin_config_t pins = {
        .bck_io_num   = PIN_I2S_SCK,
        .ws_io_num    = PIN_I2S_WS,
        .data_out_num = PIN_I2S_SD,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(I2S_PORT, &pins);
}

// ==========================================
// 6. setup()
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // サイン波テーブル生成
    for (int i = 0; i < TABLE_SIZE; i++) {
        SINE_TABLE[i] = (int16_t)(sinf(2.0f * (float)M_PI * i / TABLE_SIZE) * SINE_SCALE);
    }

    // ADC 設定（12bit、フル電圧レンジ）
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0〜3.3V

    // ADC 初期値取得
    adc_filtered = (float)analogRead(PIN_MASCON);

    // I2S 初期化
    i2s_init();

    Serial.println("【Seeed XIAO ESP32C5対応版・真SPWM】I2S VVVFシステム起動");
}

// ==========================================
// 7. loop()
// ==========================================
void loop() {
    // ----------------------------------------------------------
    // A. マスコン読み取りと速度（周波数）計算
    // ----------------------------------------------------------
    // MicroPython の read_u16() (0〜65535) に合わせて
    // Arduino の 12bit ADC (0〜4095) を 16bit スケールへ変換する
    int adc_raw_12 = analogRead(PIN_MASCON);
    float adc_raw  = (float)adc_raw_12 * (65535.0f / 4095.0f);

    adc_filtered = adc_filtered * 0.8f + adc_raw * 0.2f;

    if (adc_filtered < 25000.0f) {
        // ブレーキ／停止ゾーン
        float target_base_f = 0.0f;
        float change_rate   = 1.0f;
        if (current_base_f > target_base_f) {
            current_base_f -= change_rate;
            if (current_base_f < target_base_f) current_base_f = target_base_f;
        }
    } else if (adc_filtered > 40000.0f) {
        // 力行ゾーン
        float target_base_f = ((adc_filtered - 40000.0f) / 25535.0f) * 150.0f;
        float change_rate   = 0.4f;
        if (current_base_f < target_base_f) {
            current_base_f += change_rate;
            if (current_base_f > target_base_f) current_base_f = target_base_f;
        } else if (current_base_f > target_base_f) {
            current_base_f -= change_rate;
            if (current_base_f < target_base_f) current_base_f = target_base_f;
        }
    }
    // else: デッドゾーン（25000〜40000）は current_base_f を保持

    // ----------------------------------------------------------
    // B. キャリア（三角波）目標周波数の決定
    // ----------------------------------------------------------
    float target_carrier_f;
    if (current_base_f < 25.0f) {
        target_carrier_f = 800.0f + (current_base_f * 12.0f);
    } else if (current_base_f < 55.0f) {
        target_carrier_f = current_base_f * 9.0f;
    } else if (current_base_f < 95.0f) {
        target_carrier_f = current_base_f * 5.0f;
    } else {
        target_carrier_f = current_base_f * 3.0f;
    }

    current_carrier_f += (target_carrier_f - current_carrier_f) * 0.05f;

    // ----------------------------------------------------------
    // C. オーディオバッファ生成（SPWM）
    // ----------------------------------------------------------
    if (current_base_f > 0.5f) {
        float step_base = ((float)TABLE_SIZE * current_base_f) / (float)SAMPLE_RATE;
        float step_tri  = (2.0f * current_carrier_f) / (float)SAMPLE_RATE;

        for (int i = 0; i < CHUNK_SIZE; i++) {
            float sine_val    = (float)SINE_TABLE[(int)phase_base & TABLE_MASK] / (float)SINE_SCALE;
            float triangle_val = (tri_val * 2.0f) - 1.0f;

            int16_t sample = (sine_val > triangle_val) ? 12000 : -12000;

            // 16bit little-endian でバッファへ格納（モノラル）
            audio_buffer[i * 2]     = (uint8_t)(sample & 0xFF);
            audio_buffer[i * 2 + 1] = (uint8_t)((sample >> 8) & 0xFF);

            phase_base = fmodf(phase_base + step_base, (float)TABLE_SIZE);

            tri_val += step_tri * (float)tri_direction;
            if (tri_val >= 1.0f) {
                tri_val       = 1.0f;
                tri_direction = -1;
            } else if (tri_val <= 0.0f) {
                tri_val       = 0.0f;
                tri_direction = 1;
            }
        }
    } else {
        // 停止時は無音
        memset(audio_buffer, 0, BUFFER_SIZE);
    }

    // ----------------------------------------------------------
    // D. I2S へ転送
    // ----------------------------------------------------------
    size_t bytes_written = 0;
    i2s_write(I2S_PORT, audio_buffer, BUFFER_SIZE, &bytes_written, portMAX_DELAY);

    // ----------------------------------------------------------
    // E. デバッグ出力（変化があったときのみ）
    // ----------------------------------------------------------
    int adc_int      = (int)adc_filtered;
    int base_f_int   = (int)(current_base_f * 10.0f);
    int carrier_f_int= (int)current_carrier_f;

    if (adc_int      != _dbg_prev_adc       ||
        base_f_int   != _dbg_prev_base_f    ||
        carrier_f_int!= _dbg_prev_carrier_f ||
        tri_direction!= _dbg_prev_tri_dir)
    {
        Serial.printf(
            "ADC=%5d base_f=%5.1f Hz carrier_f=%5.0f Hz tri_val=%.3f tri_dir=%+d\n",
            adc_int, current_base_f, current_carrier_f, tri_val, tri_direction
        );
        _dbg_prev_adc       = adc_int;
        _dbg_prev_base_f    = base_f_int;
        _dbg_prev_carrier_f = carrier_f_int;
        _dbg_prev_tri_dir   = tri_direction;
    }
}
