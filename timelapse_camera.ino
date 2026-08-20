#include <Arduino.h>
#include <esp_camera.h>
#include <SD_MMC.h>
#include <FS.h>
#include <esp_sleep.h>
#include <string.h>

// ============================================================================
// USER SETTINGS - CHANGE THESE
// ============================================================================

const uint32_t STARTUP_DELAY_MS = 1500;

// How long the ESP32 should sleep AFTER each completed photo.
// Example: 180 = wake approximately every 3 minutes.
const uint32_t SLEEP_INTERVAL_SECONDS = 60;

// Change this string when you want to start a completely new timelapse run.
// The firmware uses it to reset the image counter stored in RTC memory.
const char* RUN_ID = "TIMELAPSE_01";

// Camera settings.
// Common useful choices: FRAMESIZE_VGA (640x480), SVGA (800x600),
// XGA (1024x768), SXGA (1280x1024), UXGA (1600x1200).
const framesize_t PHOTO_FRAME_SIZE = FRAMESIZE_SVGA;

// JPEG quality: lower number = higher quality / larger file.
// Typical useful range: 10-16.
const int PHOTO_JPEG_QUALITY = 12;

// Do not use the flash LED during normal timelapse operation.
const bool USE_FLASH = false;

// GPIO4 is the flash LED on the AI-Thinker-style ESP32-CAM.
const int FLASH_LED_PIN = 4;

// Delay before taking the photo after waking, in milliseconds.
// This gives the camera/sensor a little time to stabilise.
const uint32_t CAMERA_SETTLE_MS = 250;

// If true, write a one-line CSV log for every successful image.
const bool ENABLE_CSV_LOG = true;

// If true, print detailed information to Serial.
const bool ENABLE_SERIAL_LOG = true;

// If true, create the SD card filesystem if mounting fails.
// IMPORTANT: changing this to true can format a card if mounting fails.
const bool FORMAT_SD_IF_MOUNT_FAILS = false;

// If true, the firmware will stop taking photos when an image write fails.
const bool STOP_ON_IMAGE_ERROR = false;

// Maximum number of images. Set to 0 for no firmware limit.
// For 3 days at 3 minutes: about 1,440 images.
const uint32_t MAX_IMAGES = 0;

// ============================================================================
// FILE NAMES
// ============================================================================

const char* LOG_FILE = "/TIMELAPSE.CSV";
const char* SUMMARY_FILE = "/SUMMARY.TXT";

// ============================================================================
// AI-THINKER / AZ-DELIVERY ESP32-CAM CAMERA PINOUT
// ============================================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================================
// RTC MEMORY - SURVIVES DEEP SLEEP RESTARTS
// ============================================================================

const uint32_t RTC_MAGIC = 0x544C5031; // "TLP1"

RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR char rtcRunId[32] = "";
RTC_DATA_ATTR uint32_t rtcImageCount = 0;
RTC_DATA_ATTR uint32_t rtcFailedImages = 0;
RTC_DATA_ATTR uint64_t rtcBootCount = 0;

// ============================================================================
// RUNTIME VARIABLES
// ============================================================================

uint32_t currentImageNumber = 0;
uint32_t currentImageBytes = 0;
uint32_t currentCaptureTimeMs = 0;
uint32_t currentElapsedFromFirstSec = 0;

// ============================================================================
// HELPERS
// ============================================================================

void logSerial(const String& message) {
  if (ENABLE_SERIAL_LOG) {
    Serial.println(message);
  }
}

void initialiseRtcState() {
  bool newRun = false;

  if (rtcMagic != RTC_MAGIC) {
    newRun = true;
  }

  if (strncmp(rtcRunId, RUN_ID, sizeof(rtcRunId)) != 0) {
    newRun = true;
  }

  if (newRun) {
    rtcMagic = RTC_MAGIC;
    memset(rtcRunId, 0, sizeof(rtcRunId));
    strncpy(rtcRunId, RUN_ID, sizeof(rtcRunId) - 1);
    rtcImageCount = 0;
    rtcFailedImages = 0;
    rtcBootCount = 0;
  }

  rtcBootCount++;
}

bool initialiseCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = PHOTO_FRAME_SIZE;
    config.jpeg_quality = PHOTO_JPEG_QUALITY;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    // Fallback for a board without PSRAM.
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    logSerial("ERROR: Camera init failed: 0x" + String((uint32_t)err, HEX));
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_framesize(sensor, psramFound() ? PHOTO_FRAME_SIZE : FRAMESIZE_VGA);
    sensor->set_quality(sensor, psramFound() ? PHOTO_JPEG_QUALITY : 15);
  }

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  return true;
}

bool initialiseSD() {
  // 1-bit mode avoids using the extra SD data pins and the GPIO4 flash-LED conflict.
  if (!SD_MMC.begin("/sdcard", true, FORMAT_SD_IF_MOUNT_FAILS)) {
    logSerial("ERROR: SD card mount failed.");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    logSerial("ERROR: No SD card detected.");
    return false;
  }

  uint64_t cardSizeMB = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  logSerial("SD card detected: " + String((unsigned long long)cardSizeMB) + " MB");

  return true;
}

void ensureCsvHeader() {
  if (!ENABLE_CSV_LOG) return;

  if (!SD_MMC.exists(LOG_FILE)) {
    File file = SD_MMC.open(LOG_FILE, FILE_WRITE);
    if (!file) {
      logSerial("WARNING: Could not create CSV log.");
      return;
    }

    file.println("run_id,image_number,filename,elapsed_from_first_s,planned_interval_s,image_bytes,capture_time_ms,boot_count,failed_images");
    file.close();
  }
}

void writeCsvLog(const String& filename) {
  if (!ENABLE_CSV_LOG) return;

  File file = SD_MMC.open(LOG_FILE, FILE_APPEND);
  if (!file) {
    logSerial("WARNING: Could not append to CSV log.");
    return;
  }

  file.print(RUN_ID);
  file.print(",");
  file.print(currentImageNumber);
  file.print(",");
  file.print(filename);
  file.print(",");
  file.print(currentElapsedFromFirstSec);
  file.print(",");
  file.print(SLEEP_INTERVAL_SECONDS);
  file.print(",");
  file.print(currentImageBytes);
  file.print(",");
  file.print(currentCaptureTimeMs);
  file.print(",");
  file.print((unsigned long long)rtcBootCount);
  file.print(",");
  file.println(rtcFailedImages);
  file.close();
}

void updateSummary() {
  File file = SD_MMC.open(SUMMARY_FILE, FILE_WRITE);
  if (!file) {
    logSerial("WARNING: Could not update summary file.");
    return;
  }

  // FILE_WRITE overwrites the file on the ESP32 Arduino SD filesystem implementation.
  file.println("ESP32-CAM TIMELAPSE SUMMARY");
  file.println("===========================");
  file.print("Run ID: ");
  file.println(RUN_ID);
  file.print("Images saved: ");
  file.println((unsigned long)rtcImageCount);
  file.print("Failed image attempts: ");
  file.println((unsigned long)rtcFailedImages);
  file.print("Sleep interval: ");
  file.print(SLEEP_INTERVAL_SECONDS);
  file.println(" seconds");
  file.print("Estimated time from first to last image: ");
  if (rtcImageCount > 0) {
    uint64_t elapsed = (uint64_t)(rtcImageCount - 1) * SLEEP_INTERVAL_SECONDS;
    file.print((unsigned long long)elapsed);
    file.println(" seconds");
    file.print("Estimated duration: ");
    file.print((unsigned long long)(elapsed / 3600ULL));
    file.print(" h ");
    file.print((unsigned long long)((elapsed % 3600ULL) / 60ULL));
    file.print(" min ");
    file.print((unsigned long long)(elapsed % 60ULL));
    file.println(" s");
  } else {
    file.println("0 seconds");
  }
  file.print("Boot count: ");
  file.println((unsigned long long)rtcBootCount);
  file.println();
  file.println("Note: elapsed time is based on the configured sleep interval and does not include the small capture/boot overhead between wakeups.");
  file.close();
}

bool takeAndSavePhoto() {
  if (MAX_IMAGES > 0 && rtcImageCount >= MAX_IMAGES) {
    logSerial("Maximum image count reached. Going to deep sleep indefinitely.");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_deep_sleep_start();
  }

  if (USE_FLASH) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(100);
  }

  uint32_t captureStart = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  currentCaptureTimeMs = millis() - captureStart;

  if (USE_FLASH) {
    digitalWrite(FLASH_LED_PIN, LOW);
  }

  if (fb == nullptr) {
    rtcFailedImages++;
    logSerial("ERROR: Camera capture failed.");
    return false;
  }

  currentImageNumber = rtcImageCount + 1;
  currentImageBytes = fb->len;
  currentElapsedFromFirstSec = (currentImageNumber - 1) * SLEEP_INTERVAL_SECONDS;

  char filenameBuffer[32];
  snprintf(filenameBuffer, sizeof(filenameBuffer), "/IMG_%06lu.jpg", (unsigned long)currentImageNumber);
  String filename(filenameBuffer);

  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    rtcFailedImages++;
    esp_camera_fb_return(fb);
    logSerial("ERROR: Could not open " + filename + " for writing.");
    return false;
  }

  size_t written = file.write(fb->buf, fb->len);
  file.close();
  esp_camera_fb_return(fb);

  if (written != currentImageBytes) {
    rtcFailedImages++;
    logSerial("ERROR: Incomplete write for " + filename + ".");
    return false;
  }

  rtcImageCount++;

  writeCsvLog(filename);
  updateSummary();

  logSerial("Saved " + filename +
            " | " + String(currentImageBytes) + " bytes" +
            " | capture " + String(currentCaptureTimeMs) + " ms" +
            " | image " + String((unsigned long)rtcImageCount));

  return true;
}

void goToDeepSleep() {
  // Camera and SD are no longer needed before sleep.
  esp_camera_deinit();
  SD_MMC.end();

  if (USE_FLASH) {
    digitalWrite(FLASH_LED_PIN, LOW);
  }

  logSerial("Sleeping for " + String(SLEEP_INTERVAL_SECONDS) + " seconds...");
  Serial.flush();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  if (ENABLE_SERIAL_LOG) {
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32-CAM BATTERY TIMELAPSE");
    Serial.println("========================================");
  }

  initialiseRtcState();

  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  logSerial("Run ID: " + String(RUN_ID));
  logSerial("Wake cause: " + String((int)wakeCause));
  logSerial("Images already saved: " + String((unsigned long)rtcImageCount));

  // Give the camera hardware a moment after power-up/wakeup.
  delay(CAMERA_SETTLE_MS);

  if (!initialiseCamera()) {
    if (STOP_ON_IMAGE_ERROR) {
      esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SECONDS * 1000000ULL);
      esp_deep_sleep_start();
    }
  }

  if (!initialiseSD()) {
    if (STOP_ON_IMAGE_ERROR) {
      esp_camera_deinit();
      esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SECONDS * 1000000ULL);
      esp_deep_sleep_start();
    }
  }

  ensureCsvHeader();

  takeAndSavePhoto();
  
  if (STOP_ON_IMAGE_ERROR && rtcImageCount == 0 && rtcFailedImages > 0) {
    logSerial("Stopping because the first image could not be saved.");
    while (true) {
      delay(1000);
    }
  }

  goToDeepSleep();
}

void loop() {
  // Never reached: the ESP32 deep-sleeps after each photo.
}
