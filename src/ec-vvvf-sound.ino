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
static float   SINE_TABLE_NORM[TABLE_SIZE];  // -1.0〜1.0 に正規化済み（setup時に初期化）

// ==========================================
// 3. バッファ
// ==========================================
static const int CHUNK_SIZE   = SAMPLE_RATE / 50;   // 441 サンプル
static const int BUFFER_SIZE  = CHUNK_SIZE * 2;      // バイト数（16bit mono）
static uint8_t audio_buffer[BUFFER_SIZE];

// 1ループ当たりの実時間（秒）。レート計算をこれに基づかせることで、
// チャンクサイズを変えても実時間ベースの挙動を維持できる。
static const float LOOP_DT = (float)CHUNK_SIZE / (float)SAMPLE_RATE;

// ==========================================
// 3x. 応答性パラメータ（すべて1秒当たり）
// ==========================================
// 加減速レート（Hz/秒）
// 実車寄りに早くしたい場合はここを大きくする
static const float BRAKE_RATE_HZ_PER_SEC = 10.0f;
static const float POWER_RATE_HZ_PER_SEC =  4.0f;

// ADCフィルタとキャリア追従の時定数（秒）。値を小さくするほど反応が早く、
// 大きくするほど滑らかだが遅くなる。20msチャンクでも一定の応答性を保つため
// alphaをループ周期から逆算する
static const float ADC_FILTER_TAU     = 0.15f;
static const float CARRIER_FILTER_TAU = 0.15f;
static const float ADC_ALPHA     = LOOP_DT / (ADC_FILTER_TAU     + LOOP_DT);
static const float CARRIER_ALPHA = LOOP_DT / (CARRIER_FILTER_TAU + LOOP_DT);

// ==========================================
// 3xx. Siemns ドレミファインバータ
// ==========================================
// 加減速レート（Hz/秒）
static const int   DOREMIFA_NOTE_COUNT = 9;
static const float DOREMIFA_NOTES[DOREMIFA_NOTE_COUNT] = {
    174.61f,    // F3
    196.00f,    // G3
    220.00f,    // A3
    233.08f,    // Bb3
    261.63f,    // C4
    293.66f,    // D4
    311.13f,    // Eb4
    349.23f,    // F4
    392.00f,    // G4
};

// 音階が鳴る規定周波数の上限周波数
static const float DOREMIFA_MAX_BASE_F = 25.0f;

// ドレミファ区間だけキャリア追従の時定数を短くして瞬時に切り替える
static const float DOREMIFA_CARRIER_TAU = 0.03f;
static const float DOREMIFA_CARRIER_ALPHA = LOOP_DT / (DOREMIFA_CARRIER_TAU + LOOP_DT);

// ==========================================
// 4. 再生用変数
// ==========================================
static float current_base_f   = 0.0f;
static float phase_base        = 0.0f;
static float tri_val           = 0.0f;
static int   tri_direction     = 1;
static float current_carrier_f = 800.0f;
static float adc_filtered      = 0.0f;

// Low Pass Filter
static float lpf_state  = 0.0f;
static float lpf_state2 = 0.0f;
// ザラザラが残る場合はカットオフ周波数を下げる
// こもって聞こえる場合は上げる
static const float LPF_CUTOFF_HZ = 3000.0f;
static const float LPF_ALPHA = 1.0f - expf(-2.0f*(float)M_PI * LPF_CUTOFF_HZ / (float)SAMPLE_RATE);

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
        .dma_buf_count        = 4,
        .dma_buf_len          = 128,
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

    // サイン波テーブル生成（整数テーブルと正規化済みfloatテーブルを同時に構築）
    for (int i = 0; i < TABLE_SIZE; i++) {
        SINE_TABLE_NORM[i] = (float)(sinf(2.0f * (float)M_PI * i / TABLE_SIZE));
    }

    // ADC 設定（12bit、フル電圧レンジ）
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MASCON, ADC_11db);  // 0〜3.3V

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
    // adc読み込み値が4095まで上がらないので、強制的にmap, constrainで0～4095にマッピング
    int adc_raw_12 = analogRead(PIN_MASCON);
    int corrected_val = map(adc_raw_12, 0, 3300, 0, 4095);
    adc_raw_12 = constrain(corrected_val, 0, 4095);
    float adc_raw  = (float)adc_raw_12 * (65535.0f / 4095.0f);

    // 時定数ベースのローパスフィルタ（ループ周期が変わっても応答性は一緒）
    adc_filtered = adc_filtered * (1.0f-ADC_ALPHA) + adc_raw * ADC_ALPHA;

    if (adc_filtered < 25000.0f) {
        // ブレーキ／停止ゾーン
        float target_base_f = 0.0f;
        float change_rate   = BRAKE_RATE_HZ_PER_SEC * LOOP_DT;
        if (current_base_f > target_base_f) {
            current_base_f -= change_rate;
            if (current_base_f < target_base_f) current_base_f = target_base_f;
        }
    } else if (adc_filtered > 40000.0f) {
        // 力行ゾーン
        float target_base_f = ((adc_filtered - 40000.0f) / 25535.0f) * 150.0f;
        float change_rate   = POWER_RATE_HZ_PER_SEC * LOOP_DT;
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
    // キャリア周波数：基底周波数帯ごとの段階的な倍率制御
    // f<25Hz:  800 + f×12
    // 25≤f<55Hz: f×9
    // 55≤f<95Hz: f×5
    // f≥95Hz:    f×3
    float target_carrier_f;
    bool in_doremifa_zone = (current_base_f < DOREMIFA_MAX_BASE_F);

    if (current_base_f < 25.0f) {
        // Not Siemens DoReMiFa; target_carrier_f = 800.0f + (current_base_f * 12.0f);
        int note_index = (int)(current_base_f / (DOREMIFA_MAX_BASE_F / (float)DOREMIFA_NOTE_COUNT));
        if (note_index >= DOREMIFA_NOTE_COUNT) note_index = DOREMIFA_NOTE_COUNT - 1;
        if (note_index < 0) note_index = 0;
        target_carrier_f = DOREMIFA_NOTES[note_index];
    } else if (current_base_f < 55.0f) {
        target_carrier_f = current_base_f * 9.0f;
    } else if (current_base_f < 95.0f) {
        target_carrier_f = current_base_f * 5.0f;
    } else {
        target_carrier_f = current_base_f * 3.0f;
    }

    float carrier_alpha = in_doremifa_zone ? DOREMIFA_CARRIER_ALPHA : CARRIER_ALPHA;
    current_carrier_f += (target_carrier_f - current_carrier_f) * carrier_alpha;

    // ----------------------------------------------------------
    // C. オーディオバッファ生成（SPWM）
    // ----------------------------------------------------------
    // current_base_f が 0.5Hz を超えるときのみ、VVVF音を生成する。
    // それ以下は停止相当とみなし、無音バッファをそのまま送る。
    if (current_base_f > 0.5f) {
        // step_base:
        //   サイン波テーブル上を 1 サンプルごとにどれだけ進めるか。
        //   基底周波数 current_base_f が高いほど進み幅が大きくなり、音程が上がる。
        float step_base = ((float)TABLE_SIZE * current_base_f) / (float)SAMPLE_RATE;
        // step_tri:
        //   三角波の現在値 tri_val(0.0〜1.0) の更新量。
        //   0→1→0 の往復で 1 周期になるため 2.0 を掛ける。
        float step_tri  = (2.0f * current_carrier_f) / (float)SAMPLE_RATE;

        for (int i = 0; i < CHUNK_SIZE; i++) {
            // 正規化済みテーブルから直接参照（ループ内の除算を削除）
            float sine_val    = SINE_TABLE_NORM[(int)phase_base & TABLE_MASK];
            // tri_val は 0.0〜1.0 で保持しているので、比較用に -1.0〜1.0 へ変換
            float triangle_val = (tri_val * 2.0f) - 1.0f;

            // SPWM 比較:
            // サイン波 > 三角波 の期間を HIGH、その他を LOW にして
            // 疑似PWM波形（VVVFインバータ風の音源）を作る。
            int16_t raw_sample = (sine_val > triangle_val) ? 15000 : -15000;

            // モーターのインダクタンスによる鈍りを再現するローパスフィルタ
            // 1段階目のLPF
            lpf_state += LPF_ALPHA * ((float)raw_sample - lpf_state);
            // 2段階目でさらに滑らかにする（高域のジャリジャリ感を消す）
            lpf_state2 += LPF_ALPHA * (lpf_state - lpf_state2);
            int16_t sample = (int16_t)lpf_state2;

            // 16bit little-endian でバッファへ格納（モノラル）
            audio_buffer[i * 2]     = (uint8_t)(sample & 0xFF);
            audio_buffer[i * 2 + 1] = (uint8_t)((sample >> 8) & 0xFF);

            // サイン波位相を進める（テーブル末尾を超えたら先頭へ循環）
            phase_base += step_base;
            while (phase_base >= (float)TABLE_SIZE) {
                phase_base -= (float)TABLE_SIZE;
            }

            // 三角波を現在方向に進め、端に達したら折り返す
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
