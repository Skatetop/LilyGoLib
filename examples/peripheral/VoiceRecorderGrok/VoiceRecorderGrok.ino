/**
 * @file      VoiceRecorderGrok.ino
 * @license   MIT
 * @date      2026-09-19
 *
 * @brief     Voice memo recorder:
 *              - Tap the button to start/stop recording on demand (no fixed duration).
 *              - A lightweight DC-blocking filter + adaptive noise gate strips
 *                background hiss/hum ("parasite" noise) from the audio in real time.
 *              - Every recording is saved to its own timestamped .wav file under
 *                /recordings on the SD card, so older recordings are never
 *                overwritten or lost.
 *              - When Wi-Fi + an xAI API key are configured, the recording is
 *                uploaded to the Grok Speech-to-Text API and the returned
 *                transcript is shown on screen, printed to Serial and appended
 *                (never overwritten) to /recordings/index.txt as a running log.
 *
 * @note      Requires a board with an SD card socket (HAS_SD_CARD_SOCKET).
 *            Transcription is optional: fill in WIFI_SSID/WIFI_PASSWORD/XAI_API_KEY
 *            below to enable it. Without them the recorder still works fully
 *            offline (record / clean up / archive to SD).
 */

#include <LilyGoLib.h>
#include <LV_Helper.h>

#ifdef HAS_SD_CARD_SOCKET

#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* ------------------------------------------------------------------------ */
/*  User configuration                                                      */
/* ------------------------------------------------------------------------ */

// Leave as-is to keep the recorder fully offline (SD only, no transcription).
static const char *WIFI_SSID     = "YOUR_SSID";
static const char *WIFI_PASSWORD = "YOUR_PASS";

// Create a key at https://console.x.ai and paste it here to enable transcription.
static const char *XAI_API_KEY   = "YOUR_XAI_API_KEY";

// xAI Grok Speech-to-Text (batch) endpoint, see https://docs.x.ai
static const char *GROK_API_HOST  = "api.x.ai";
static const char *GROK_STT_PATH  = "/v1/stt";
static const char *GROK_STT_MODEL = "grok-voice-transcribe-2.0";

// Safety cap so a missed "stop" tap can't fill the SD card.
static const uint32_t MAX_RECORD_SECONDS = 180;

/* ------------------------------------------------------------------------ */
/*  Audio format                                                            */
/* ------------------------------------------------------------------------ */

static const uint32_t SAMPLE_RATE   = 16000;
static const uint16_t BITS_PER_SAMP = 16;
static const uint16_t NUM_CHANNELS  = 1;
static const size_t   CHUNK_SAMPLES = 1024;                       // ~64ms per block
static const size_t   CHUNK_BYTES   = CHUNK_SAMPLES * sizeof(int16_t);

/* ------------------------------------------------------------------------ */
/*  Noise cleanup: DC blocker + adaptive noise gate ("parasite" noise)      */
/* ------------------------------------------------------------------------ */

struct NoiseFilterState {
    float    dcPrevIn      = 0.0f;
    float    dcPrevOut     = 0.0f;
    bool     calibrating   = true;
    uint32_t calibSamples  = 0;
    double   calibSumSq    = 0.0;
    float    noiseFloor    = 40.0f;    // RMS estimate of the "silent" background
    float    gateGain      = 1.0f;
};

// ~300ms of calibration at start of every recording: stay quiet so the gate
// can learn the room's own noise floor (fan hiss, electrical hum, etc.)
static const uint32_t CALIB_SAMPLES = SAMPLE_RATE * 300 / 1000;

static void noiseFilterProcessBlock(int16_t *samples, size_t count, NoiseFilterState *st)
{
    // 1) One-pole DC-blocking / high-pass filter: removes hum & rumble.
    double sumSq = 0.0;
    for (size_t i = 0; i < count; i++) {
        float in = (float)samples[i];
        float out = in - st->dcPrevIn + 0.995f * st->dcPrevOut;
        st->dcPrevIn = in;
        st->dcPrevOut = out;
        if (out > 32767.0f)  out = 32767.0f;
        if (out < -32768.0f) out = -32768.0f;
        samples[i] = (int16_t)out;
        sumSq += (double)out * (double)out;
    }

    float blockRms = sqrtf((float)(sumSq / count));

    // 2) Calibrate the noise floor from the first moments of silence.
    if (st->calibrating) {
        st->calibSumSq += sumSq;
        st->calibSamples += count;
        if (st->calibSamples >= CALIB_SAMPLES) {
            float rms = sqrtf((float)(st->calibSumSq / st->calibSamples));
            st->noiseFloor = rms * 2.5f + 20.0f;   // margin above measured noise
            st->calibrating = false;
        }
        // Mute output entirely while calibrating so the calibration tone/click
        // (if any) never ends up in the recording.
        for (size_t i = 0; i < count; i++) samples[i] = 0;
        return;
    }

    // 3) Adaptive noise gate: fade out blocks that sit at/under the noise
    //    floor, fade quickly back in as soon as real signal appears.
    float targetGain = (blockRms > st->noiseFloor) ? 1.0f : 0.06f;
    float rate = (targetGain > st->gateGain) ? 0.6f : 0.08f;   // fast attack, slow release
    st->gateGain += (targetGain - st->gateGain) * rate;

    for (size_t i = 0; i < count; i++) {
        samples[i] = (int16_t)((float)samples[i] * st->gateGain);
    }
}

/* ------------------------------------------------------------------------ */
/*  WAV header helper (own 44-byte canonical PCM header, no extra deps)     */
/* ------------------------------------------------------------------------ */

static void writeWavHeader(File &f, uint32_t dataBytes)
{
    uint32_t byteRate   = SAMPLE_RATE * NUM_CHANNELS * BITS_PER_SAMP / 8;
    uint16_t blockAlign = NUM_CHANNELS * BITS_PER_SAMP / 8;
    uint32_t riffSize   = 36 + dataBytes;

    uint8_t hdr[44];
    memcpy(hdr + 0,  "RIFF", 4);
    memcpy(hdr + 4,  &riffSize, 4);
    memcpy(hdr + 8,  "WAVE", 4);
    memcpy(hdr + 12, "fmt ", 4);
    uint32_t fmtSize = 16;      memcpy(hdr + 16, &fmtSize, 4);
    uint16_t audioFmt = 1;      memcpy(hdr + 20, &audioFmt, 2);
    memcpy(hdr + 22, &NUM_CHANNELS, 2);
    memcpy(hdr + 24, &SAMPLE_RATE, 4);
    memcpy(hdr + 28, &byteRate, 4);
    memcpy(hdr + 32, &blockAlign, 2);
    memcpy(hdr + 34, &BITS_PER_SAMP, 2);
    memcpy(hdr + 36, "data", 4);
    memcpy(hdr + 40, &dataBytes, 4);

    f.seek(0);
    f.write(hdr, sizeof(hdr));
}

/* ------------------------------------------------------------------------ */
/*  Recording task                                                          */
/* ------------------------------------------------------------------------ */

enum AppState { STATE_IDLE, STATE_RECORDING, STATE_FINISHING, STATE_TRANSCRIBING };

struct RecordResult {
    bool ok;
    char path[80];
};

struct TranscribeResult {
    bool  ok;
    char  path[80];
    char *text;     // heap allocated by the producer, freed by the consumer
};

static volatile AppState appState = STATE_IDLE;
static volatile bool     stopRequested = false;
static volatile uint32_t recordedBytes = 0;

static QueueHandle_t recordResultQueue     = NULL;
static QueueHandle_t transcribeResultQueue = NULL;

static lv_obj_t *statusLabel;
static lv_obj_t *transcriptLabel;
static lv_obj_t *recordBtn;
static lv_obj_t *recordBtnLabel;

static void makeTimestampedPath(char *out, size_t outLen, const char *ext)
{
    struct tm t = {0};
    instance.rtc.getDateTime(&t);
    if (t.tm_year > 0) {
        strftime(out, outLen, "/recordings/rec_%Y%m%d_%H%M%S", &t);
    } else {
        // RTC has never been set: fall back to an uptime-based name.
        snprintf(out, outLen, "/recordings/rec_%010lu", (unsigned long)millis());
    }
    strncat(out, ".", outLen - strlen(out) - 1);
    strncat(out, ext, outLen - strlen(out) - 1);
}

static void recordTask(void *param)
{
    RecordResult result = { false, "" };

    char path[80];
    makeTimestampedPath(path, sizeof(path), "wav");

    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("Failed to create %s\n", path);
        xQueueSend(recordResultQueue, &result, 0);
        vTaskDelete(NULL);
        return;
    }

    // Reserve space for the header, patched with real sizes once we know them.
    uint8_t blank[44] = {0};
    f.write(blank, sizeof(blank));

    NoiseFilterState filter;
    int16_t buf[CHUNK_SAMPLES];
    uint32_t dataBytes = 0;
    uint32_t maxBytes = MAX_RECORD_SECONDS * SAMPLE_RATE * sizeof(int16_t);

#ifdef ARDUINO_T_LORA_PAGER
    instance.codec.setGain(50.0);
    instance.codec.open(BITS_PER_SAMP, NUM_CHANNELS, SAMPLE_RATE);
#endif

    while (!stopRequested && dataBytes < maxBytes) {
        size_t gotBytes = 0;

#ifdef ARDUINO_T_LORA_PAGER
        if (instance.codec.read((uint8_t *)buf, CHUNK_BYTES) == 0) {
            gotBytes = CHUNK_BYTES;
        }
#else
        gotBytes = instance.mic.readBytes((uint8_t *)buf, CHUNK_BYTES);
#endif
        if (gotBytes == 0) {
            break;
        }

        size_t samples = gotBytes / sizeof(int16_t);
        noiseFilterProcessBlock(buf, samples, &filter);

        f.write((uint8_t *)buf, gotBytes);
        dataBytes += gotBytes;
        recordedBytes = dataBytes;
    }

#ifdef ARDUINO_T_LORA_PAGER
    instance.codec.close();
#endif

    writeWavHeader(f, dataBytes);
    f.close();

    // A recording shorter than the calibration window is almost certainly
    // an accidental tap; drop it instead of archiving a near-empty file.
    if (dataBytes < (CALIB_SAMPLES * sizeof(int16_t))) {
        SD.remove(path);
        result.ok = false;
    } else {
        result.ok = true;
        strncpy(result.path, path, sizeof(result.path) - 1);
    }

    xQueueSend(recordResultQueue, &result, 0);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------------ */
/*  Grok transcription task                                                 */
/* ------------------------------------------------------------------------ */

// Minimal helper: pull out the value of "text":"..." from a JSON reply,
// unescaping the handful of escape sequences a transcript can contain.
static String jsonExtractString(const String &json, const char *key)
{
    String needle = String("\"") + key + "\"";
    int k = json.indexOf(needle);
    if (k < 0) return String();
    int colon = json.indexOf(':', k + needle.length());
    if (colon < 0) return String();
    int i = colon + 1;
    while (i < (int)json.length() && isspace((unsigned char)json[i])) i++;
    if (i >= (int)json.length() || json[i] != '"') return String();
    i++;
    String out;
    while (i < (int)json.length()) {
        char c = json[i++];
        if (c == '"') break;
        if (c == '\\' && i < (int)json.length()) {
            char e = json[i++];
            switch (e) {
            case 'n': out += '\n'; break;
            case 't': out += '\t'; break;
            case 'r': break;
            case 'u': i += 4; break; // skip \uXXXX, good enough for a demo
            default:  out += e; break;
            }
        } else {
            out += c;
        }
    }
    return out;
}

static bool httpReadResponseBody(WiFiClientSecure &client, String &body)
{
    unsigned long start = millis();

    String statusLine = client.readStringUntil('\n');
    if (statusLine.indexOf("200") < 0) {
        Serial.printf("Grok HTTP status: %s\n", statusLine.c_str());
    }

    bool chunked = false;
    long contentLength = -1;
    while (client.connected() || client.available()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) break;
        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("transfer-encoding:") && lower.indexOf("chunked") >= 0) chunked = true;
        if (lower.startsWith("content-length:")) contentLength = line.substring(line.indexOf(':') + 1).toInt();
        if (millis() - start > 20000) return false;
    }

    body = "";
    if (chunked) {
        while (true) {
            String sizeLine = client.readStringUntil('\n');
            sizeLine.trim();
            if (sizeLine.length() == 0) {
                if (millis() - start > 20000) return false;
                continue;
            }
            long chunkSize = strtol(sizeLine.c_str(), NULL, 16);
            if (chunkSize <= 0) break;
            long remaining = chunkSize;
            while (remaining > 0) {
                if (client.available()) {
                    body += (char)client.read();
                    remaining--;
                } else if (!client.connected() && !client.available()) {
                    break;
                }
            }
            client.readStringUntil('\n'); // trailing CRLF after each chunk
        }
    } else if (contentLength >= 0) {
        while ((long)body.length() < contentLength) {
            if (client.available()) {
                body += (char)client.read();
            } else if (!client.connected()) {
                break;
            }
            if (millis() - start > 20000) break;
        }
    } else {
        while (client.connected() || client.available()) {
            if (client.available()) body += (char)client.read();
        }
    }
    return true;
}

static bool transcribeWithGrok(const char *wavPath, String &outText)
{
    File f = SD.open(wavPath, FILE_READ);
    if (!f) return false;
    size_t fileSize = f.size();

    WiFiClientSecure client;
    client.setInsecure(); // demo simplicity; pin/verify the xAI CA in production
    client.setTimeout(20000);

    if (!client.connect(GROK_API_HOST, 443)) {
        Serial.println("Failed to connect to api.x.ai");
        f.close();
        return false;
    }

    const char *fileName = strrchr(wavPath, '/');
    fileName = fileName ? fileName + 1 : wavPath;

    String boundary = "----LilyGoGrok" + String((uint32_t)millis(), HEX);

    String modelPart = "--" + boundary + "\r\n"
                        "Content-Disposition: form-data; name=\"model\"\r\n\r\n" +
                        String(GROK_STT_MODEL) + "\r\n";

    String filePartHeader = "--" + boundary + "\r\n"
                             "Content-Disposition: form-data; name=\"file\"; filename=\"" +
                             String(fileName) + "\"\r\n"
                             "Content-Type: audio/wav\r\n\r\n";

    String closing = "\r\n--" + boundary + "--\r\n";

    size_t contentLength = modelPart.length() + filePartHeader.length() + fileSize + closing.length();

    client.printf("POST %s HTTP/1.1\r\n", GROK_STT_PATH);
    client.printf("Host: %s\r\n", GROK_API_HOST);
    client.printf("Authorization: Bearer %s\r\n", XAI_API_KEY);
    client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary.c_str());
    client.printf("Content-Length: %u\r\n", (unsigned)contentLength);
    client.print("Connection: close\r\n\r\n");

    client.print(modelPart);
    client.print(filePartHeader);

    uint8_t buf[1024];
    while (f.available()) {
        size_t n = f.read(buf, sizeof(buf));
        client.write(buf, n);
    }
    f.close();
    client.print(closing);

    String body;
    bool got = httpReadResponseBody(client, body);
    client.stop();
    if (!got) return false;

    outText = jsonExtractString(body, "text");
    if (outText.length() == 0) {
        Serial.println("Grok response:");
        Serial.println(body);
        return false;
    }
    return true;
}

static void appendToIndexLog(const char *wavPath, const char *text)
{
    File idx = SD.open("/recordings/index.txt", FILE_APPEND);
    if (!idx) return;
    struct tm t = {0};
    char stamp[32] = "unknown-time";
    instance.rtc.getDateTime(&t);
    if (t.tm_year > 0) {
        strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &t);
    }
    idx.printf("[%s] %s\n%s\n\n", stamp, wavPath, text);
    idx.close();
}

static void transcribeTask(void *param)
{
    char *path = (char *)param;
    TranscribeResult result = { false, "", nullptr };
    strncpy(result.path, path, sizeof(result.path) - 1);

    String text;
    if (transcribeWithGrok(path, text)) {
        result.ok = true;
        result.text = strdup(text.c_str());

        // Keep the transcript next to the audio so nothing is lost even if
        // the index log is ever cleared: audio + text both live on the SD card.
        String txtPath = String(path);
        txtPath.replace(".wav", ".txt");
        File tf = SD.open(txtPath, FILE_WRITE);
        if (tf) {
            tf.print(text);
            tf.close();
        }
        appendToIndexLog(path, text.c_str());
    }

    free(path);
    xQueueSend(transcribeResultQueue, &result, 0);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------------ */
/*  UI                                                                      */
/* ------------------------------------------------------------------------ */

static void setStatus(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    lv_label_set_text(statusLabel, buf);
    Serial.println(buf);
}

static bool wifiCredentialsConfigured()
{
    return strcmp(WIFI_SSID, "YOUR_SSID") != 0 && strcmp(XAI_API_KEY, "YOUR_XAI_API_KEY") != 0;
}

static void startTranscription(const char *path)
{
    if (!wifiCredentialsConfigured()) {
        setStatus("Saved %s (set Wi-Fi/API key to transcribe)", path);
        appState = STATE_IDLE;
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        setStatus("Connecting to Wi-Fi...");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
            delay(100);
            lv_task_handler();
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        setStatus("Saved %s (Wi-Fi unavailable)", path);
        appState = STATE_IDLE;
        return;
    }

    appState = STATE_TRANSCRIBING;
    setStatus("Uploading to Grok...");

    char *pathCopy = strdup(path);
    xTaskCreatePinnedToCore(transcribeTask, "grokUpload", 32768, pathCopy, 1, NULL, 1);
}

static void recordButtonEvent(lv_event_t *e)
{
    if (appState == STATE_IDLE) {
        appState = STATE_RECORDING;
        stopRequested = false;
        recordedBytes = 0;
        lv_label_set_text(recordBtnLabel, "Stop");
        lv_obj_set_style_bg_color(recordBtn, lv_palette_main(LV_PALETTE_RED), 0);
        setStatus("Calibrating noise floor, stay quiet...");
        xTaskCreatePinnedToCore(recordTask, "recordWav", 12288, NULL, 1, NULL, 1);
    } else if (appState == STATE_RECORDING) {
        stopRequested = true;
        appState = STATE_FINISHING;
        lv_label_set_text(recordBtnLabel, "...");
        setStatus("Finishing recording...");
    }
}

static void buildUI()
{
    lv_obj_t *scr = lv_screen_active();

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Grok Voice Notes");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    statusLabel = lv_label_create(scr);
    lv_label_set_long_mode(statusLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(statusLabel, LV_PCT(90));
    lv_obj_align(statusLabel, LV_ALIGN_TOP_MID, 0, 34);
    lv_label_set_text(statusLabel, "Tap Record to start");

    recordBtn = lv_button_create(scr);
    lv_obj_set_size(recordBtn, 100, 100);
    lv_obj_set_style_radius(recordBtn, LV_RADIUS_CIRCLE, 0);
    lv_obj_align(recordBtn, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_event_cb(recordBtn, recordButtonEvent, LV_EVENT_CLICKED, NULL);

    recordBtnLabel = lv_label_create(recordBtn);
    lv_label_set_text(recordBtnLabel, "Rec");
    lv_obj_center(recordBtnLabel);

    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, LV_PCT(92), 90);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);

    transcriptLabel = lv_label_create(cont);
    lv_label_set_long_mode(transcriptLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(transcriptLabel, LV_PCT(96));
    lv_label_set_text(transcriptLabel, "Transcript will appear here");

    // Create input device group, only needed for T-LoRa-Pager (keyboard/rotary).
    lv_group_t *group = lv_group_create();
    lv_set_default_group(group);
    lv_group_add_obj(lv_group_get_default(), recordBtn);
}

/* ------------------------------------------------------------------------ */
/*  Arduino entry points                                                    */
/* ------------------------------------------------------------------------ */

void setup()
{
    Serial.begin(115200);

    instance.begin();

    beginLvglHelper(instance);

    instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL);

    // Turn on the audio power amplifier / codec input path.
    instance.powerControl(POWER_SPEAK, true);

    buildUI();

    recordResultQueue     = xQueueCreate(1, sizeof(RecordResult));
    transcribeResultQueue = xQueueCreate(1, sizeof(TranscribeResult));

    int retry = 10;
    bool mounted = false;
    do {
        mounted = instance.installSD();
        if (!mounted) delay(500);
    } while (!mounted && --retry);

    if (!mounted) {
        setStatus("No SD card found - insert one and reset");
        lv_obj_add_state(recordBtn, LV_STATE_DISABLED);
        return;
    }

    if (!SD.exists("/recordings")) {
        SD.mkdir("/recordings");
    }
}

void loop()
{
    RecordResult recRes;
    if (xQueueReceive(recordResultQueue, &recRes, 0) == pdTRUE) {
        lv_label_set_text(recordBtnLabel, "Rec");
        lv_obj_set_style_bg_color(recordBtn, lv_palette_main(LV_PALETTE_BLUE), 0);

        if (recRes.ok) {
            startTranscription(recRes.path);
        } else {
            setStatus("Recording discarded (too short)");
            appState = STATE_IDLE;
        }
    }

    TranscribeResult txRes;
    if (xQueueReceive(transcribeResultQueue, &txRes, 0) == pdTRUE) {
        appState = STATE_IDLE;
        if (txRes.ok && txRes.text) {
            setStatus("Saved %s", txRes.path);
            lv_label_set_text(transcriptLabel, txRes.text);
            free(txRes.text);
        } else {
            setStatus("Saved %s (transcription failed)", txRes.path);
        }
    }

    if (appState == STATE_RECORDING) {
        uint32_t secs = recordedBytes / (SAMPLE_RATE * sizeof(int16_t));
        lv_label_set_text_fmt(statusLabel, "Recording... %lus (tap Stop when done)", (unsigned long)secs);
    }

    lv_task_handler();
    delay(5);
}

#else

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("This example requires a board with an SD card socket (HAS_SD_CARD_SOCKET).");
    delay(1000);
}

#endif // HAS_SD_CARD_SOCKET
