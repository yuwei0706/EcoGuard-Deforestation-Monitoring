#include <M5Core2.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// ==========================================
// 🔴 USER SETTINGS: FILL THIS DATA IN 🔴
// ==========================================
const char* ssid = "Funtastic";        // Enter your Wi-Fi SSID
const char* password = "15512994"; // Enter your Wi-Fi Password

// Telegram Bot Settings
const char* botToken = "8439065947:AAF0tdb0q1lV-n_aqyORp82iEZEJK2zBaQs"; // Get from BotFather
const char* chatID = "1597070441";     // Get from UserInfoBot

// Alert Settings
unsigned long lastAlertTime = 0;
const long alertInterval = 15000; // 15000ms = 15 seconds (Don't spam Telegram)
// ==========================================

// Disable quantization of the filterbank to improve accuracy
#define EIDSP_QUANTIZE_FILTERBANK 0

// Include necessary libraries
#include <Illegal_Logging_Detection_inferencing.h>
#include <driver/i2s.h>

// I2S pin configuration for M5Core2
#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34

#define Speak_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1

#define DATA_SIZE 1024

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define WAVEFORM_HEIGHT 100
#define WAVEFORM_Y_OFFSET 70

int data_offset = 0;

typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false;

#define CHAINSAW_THRESHOLD 0.7

// --- FUNCTION: SEND TELEGRAM MESSAGE ---
void sendTelegramAlert(float confidence) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate validation for simplicity

        HTTPClient https;
        
        // Create the message
        String message = "⚠️ ALERT: Chainsaw Detected! Confidence: " + String(confidence * 100, 1) + "%";
        
        // Construct the URL
        String url = "https://api.telegram.org/bot" + String(botToken) + "/sendMessage?chat_id=" + String(chatID) + "&text=" + message;

        ei_printf("Sending Telegram Alert...\n");
        
        if (https.begin(client, url)) {
            int httpCode = https.GET();
            if (httpCode > 0) {
                ei_printf("Telegram sent! Code: %d\n", httpCode);
            } else {
                ei_printf("Telegram failed. Error: %s\n", https.errorToString(httpCode).c_str());
            }
            https.end();
        } else {
            ei_printf("Unable to connect to Telegram API\n");
        }
    } else {
        ei_printf("Wi-Fi not connected, cannot send alert.\n");
    }
}

// --- FUNCTION: INIT WIFI ---
void initWiFi() {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println("Connecting to WiFi...");
    
    WiFi.begin(ssid, password);
    
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < 20) {
        delay(500);
        Serial.print(".");
        M5.Lcd.print(".");
        attempt++;
    }

    if(WiFi.status() == WL_CONNECTED) {
        M5.Lcd.println("\nWiFi Connected!");
        Serial.println("WiFi Connected");
        delay(1000);
    } else {
        M5.Lcd.println("\nWiFi Failed.");
        Serial.println("WiFi Failed");
        delay(1000);
    }
}

bool InitI2SSpeakOrMic(int mode) {
    esp_err_t err = ESP_OK;
    i2s_driver_uninstall(Speak_I2S_NUMBER);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = EI_CLASSIFIER_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };

    if (mode == MODE_MIC) {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    } else {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    }

    err = i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);
    if (err != ESP_OK) return false;

    i2s_pin_config_t pin_config = {
        .bck_io_num = CONFIG_I2S_BCK_PIN,
        .ws_io_num = CONFIG_I2S_LRCK_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = CONFIG_I2S_DATA_IN_PIN
    };

    err = i2s_set_pin(Speak_I2S_NUMBER, &pin_config);
    if (err != ESP_OK) return false;

    err = i2s_set_clk(Speak_I2S_NUMBER, EI_CLASSIFIER_FREQUENCY, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) return false;

    return true;
}

void DisplayInit(void) {
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(2);
}

void setup() {
    M5.begin(true, true, true, true);
    M5.Axp.SetSpkEnable(true);
    
    // Initialize WiFi First
    initWiFi();

    DisplayInit();
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("Chainsaw Detector");
    delay(100);

    Serial.begin(115200);
    Serial.println("Edge Impulse Chainsaw Detection");

    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / (EI_CLASSIFIER_FREQUENCY / 1000));
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }

    if (!InitI2SSpeakOrMic(MODE_MIC)) {
        ei_printf("ERR: Failed to initialize I2S\r\n");
        return;
    }
}

void loop() {
    ei_printf("Starting inferencing in 2 seconds...\n");
    delay(2000);

    M5.Lcd.fillRect(10, 40, SCREEN_WIDTH - 20, 40, WHITE);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 40);
    M5.Lcd.printf("Recording...");
    
    M5.Lcd.fillRect(0, WAVEFORM_Y_OFFSET, SCREEN_WIDTH, WAVEFORM_HEIGHT, WHITE);
    displayWaveform();

    ei_printf("Recording...\n");

    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    ei_printf("Recording done\n");

    M5.Lcd.fillRect(10, 100, SCREEN_WIDTH - 20, 40, WHITE);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 100);
    M5.Lcd.printf("Processing...");

    displayWaveform();

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    M5.Lcd.fillRect(10, 100, SCREEN_WIDTH - 20, 40, WHITE);
    M5.Lcd.setTextColor(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 100);

    M5.Lcd.fillRect(10, 180, SCREEN_WIDTH - 20, 40, WHITE);

    bool chainsaw_detected = false;
    float max_confidence = 0.0;
    const char* detected_label = "";

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > max_confidence) {
            max_confidence = result.classification[ix].value;
            detected_label = result.classification[ix].label;
        }
        
        // CHECK FOR CHAINSAW
        if ((strcmp(result.classification[ix].label, "chainsaw") == 0 || 
             strcmp(result.classification[ix].label, "Chainsaw") == 0) && 
            result.classification[ix].value > CHAINSAW_THRESHOLD) {
            chainsaw_detected = true;
        }
    }

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 140);
    
    if (chainsaw_detected) {
        M5.Lcd.setTextColor(RED);
        M5.Lcd.fillRect(0, 180, SCREEN_WIDTH, 50, RED);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(10, 190);
        M5.Lcd.printf("CHAINSAW!");
        
        ei_printf("*** CHAINSAW DETECTED! Confidence: %.2f ***\n", max_confidence);

        // ===============================================
        // 🔴 TELEGRAM NOTIFICATION LOGIC 🔴
        // ===============================================
        unsigned long currentMillis = millis();
        if (currentMillis - lastAlertTime >= alertInterval) {
            M5.Lcd.setTextSize(2);
            M5.Lcd.setCursor(200, 10);
            M5.Lcd.setTextColor(BLUE);
            M5.Lcd.print("Sending...");
            
            sendTelegramAlert(max_confidence);
            
            lastAlertTime = currentMillis; // Reset timer
            
            M5.Lcd.fillRect(200, 10, 120, 20, WHITE); // Clear "Sending..."
        }
        // ===============================================

    } else {
        M5.Lcd.setTextColor(GREEN);
        M5.Lcd.setCursor(10, 140);
        M5.Lcd.printf("Monitoring...");
        
        M5.Lcd.setTextColor(BLUE);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(10, 190);
        M5.Lcd.printf("%s: %.2f", detected_label, max_confidence);
    }

    M5.Lcd.setTextColor(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(10, 220);
    M5.Lcd.printf("DSP:%dms Class:%dms", result.timing.dsp, result.timing.classification);
}

void displayWaveform() {
    M5.Lcd.fillRect(0, WAVEFORM_Y_OFFSET, SCREEN_WIDTH, WAVEFORM_HEIGHT, WHITE);
    
    int16_t x = 0;
    int16_t last_y = WAVEFORM_Y_OFFSET + WAVEFORM_HEIGHT / 2;
    
    for (int i = 0; i < SCREEN_WIDTH; i++) {
        int index = i * (EI_CLASSIFIER_RAW_SAMPLE_COUNT / SCREEN_WIDTH);
        int16_t sample = inference.buffer[index];
        int16_t y = map(sample, -32768, 32767, WAVEFORM_Y_OFFSET + WAVEFORM_HEIGHT, WAVEFORM_Y_OFFSET);
        M5.Lcd.drawLine(x, last_y, x + 1, y, BLACK);
        x++;
        last_y = y;
    }
}

static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (inference.buffer == NULL) return false;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;
    return true;
}

static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    size_t bytes_read = 0;
    size_t bytes_to_read = inference.n_samples * sizeof(int16_t);
    int32_t sum = 0;
    int16_t max_audio = 0;

    while (bytes_read < bytes_to_read) {
        size_t bytes_left = bytes_to_read - bytes_read;
        size_t chunk_size = (bytes_left < DATA_SIZE) ? bytes_left : DATA_SIZE;
        size_t chunk_bytes_read = 0;
        i2s_read(Speak_I2S_NUMBER, (char *)(inference.buffer + (bytes_read / sizeof(int16_t))), chunk_size, &chunk_bytes_read, portMAX_DELAY);
        if (chunk_bytes_read == 0) return false;
        bytes_read += chunk_bytes_read;
    }
    inference.buf_ready = 1;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif