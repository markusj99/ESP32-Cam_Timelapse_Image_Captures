/*
  ESP32-CAM Battery Timelapse
  ===========================

  Designed for AI-Thinker style ESP32-CAM boards
  with OV2640 camera and onboard microSD card slot.

  The sketch:
    - wakes from deep sleep
    - captures one JPEG photo
    - saves the JPEG to microSD
    - records metadata in a CSV file
    - updates a human-readable summary
    - returns to deep sleep
    - repeats automatically

  SD card interfaces:
    1. Native SD_MMC
    2. SPI SD

  IMPORTANT
  ---------
  Select exactly ONE SD interface in USER CONFIGURATION:

      const bool USE_SPI_SD = false;
      const bool USE_SPI_SD = true;

  OR:

      const bool USE_SPI_SD = true;
      const bool USE_SPI_SD = false;

  The SPI configuration was tested with a PNY 16 GB SDHC card that
  failed to initialize through SD_MMC but worked reliably through SPI.

  No Wi-Fi is used. This keeps the project simple and reduces power use.
*/

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_sleep.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <SD_MMC.h>
#include <string.h>

// ============================================================================
// USER CONFIGURATION
// ============================================================================

// --------------------------- TIMELAPSE ---------------------------------------

// Deep-sleep time after a completed image cycle.
//
// Examples:
//   60   = 1 minute
//   180  = 3 minutes
//   300  = 5 minutes
//   600  = 10 minutes
const uint32_t SLEEP_INTERVAL_SECONDS = 30;

// Delay after waking before initializing the camera.
const uint32_t CAMERA_SETTLE_MS = 2500;

// Delay before initializing the SD card.
const uint32_t SD_STARTUP_DELAY_MS = 500;

// --------------------------- RUN ID ------------------------------------------

// Change RUN_ID when starting a new timelapse.
//
// Changing RUN_ID resets the image counter stored in RTC memory.
// It does NOT delete files from the SD card.
//
// Recommended workflow:
//   1. Copy the previous timelapse from the SD card.
//   2. Delete old IMG_*.jpg files on your computer.
//   3. Change RUN_ID.
//   4. Start the next timelapse.
const char* RUN_ID = "TIMELAPSE_01";

// --------------------------- CAMERA ------------------------------------------

// Camera resolution.
//
// Common choices:
//   FRAMESIZE_QVGA  = 320x240
//   FRAMESIZE_VGA   = 640x480
//   FRAMESIZE_SVGA  = 800x600
//   FRAMESIZE_XGA   = 1024x768
//   FRAMESIZE_SXGA  = 1280x1024
//   FRAMESIZE_UXGA  = 1600x1200
const framesize_t PHOTO_FRAME_SIZE = FRAMESIZE_SVGA;

// JPEG quality:
//   10 = high quality / larger file
//   12 = high quality / reasonable file size
//   15 = moderate quality
//   20+ = lower quality / smaller file
//
// Valid JPEG quality values are generally 10-63.
const int PHOTO_JPEG_QUALITY = 12;

// Use onboard flash LED?
// false is recommended for a battery-powered outdoor timelapse.
const bool USE_FLASH = false;

const int FLASH_LED_PIN = 4;

// --------------------------- SD INTERFACE ------------------------------------
//
// Select the SD interface.
//
// false = native SD_MMC
// true  = SPI
//
// The two interfaces use the same physical SD socket on the ESP32-CAM.
// For the PNY 16 GB card tested with this project, SPI worked while
// native SD_MMC did not.
const bool USE_SPI_SD = true;

// --------------------------- SD_MMC SETTINGS ---------------------------------

// true  = 1-bit SD_MMC mode
// false = 4-bit SD_MMC mode
//
// 1-bit is a good default for ESP32-CAM boards.
const bool SD_MMC_USE_1BIT_MODE = true;

// --------------------------- SD SPI SETTINGS ---------------------------------

// ESP32-CAM onboard microSD SPI wiring.
const int SD_SPI_CS_PIN   = 13;
const int SD_SPI_SCK_PIN  = 14;
const int SD_SPI_MISO_PIN = 2;
const int SD_SPI_MOSI_PIN = 15;

// SPI clock frequency.
//
// The PNY 16 GB card tested with this project worked at:
//   100 kHz, 400 kHz, 1 MHz, 4 MHz, 10 MHz and 20 MHz.
//
// 10 MHz is used as a conservative default.
const uint32_t SD_SPI_FREQUENCY_HZ = 10000000;

// --------------------------- LOGGING -----------------------------------------

const bool ENABLE_CSV_LOG = true;
const bool ENABLE_SUMMARY_FILE = true;
const bool ENABLE_SERIAL_LOG = true;

// --------------------------- ERROR HANDLING ----------------------------------

// false:
//   A failed cycle is logged and the ESP32 tries again after sleeping.
//
// true:
//   The ESP32 stops permanently after a camera/SD/image failure.
const bool STOP_ON_IMAGE_ERROR = false;

// Maximum number of images.
// 0 = unlimited.
//
// For a 3-day timelapse at 3-minute intervals:
//   1441 images gives approximately 72 hours from first to last image.
const uint32_t MAX_IMAGES = 0;

// --------------------------- SAFETY ------------------------------------------
//
// Never format the SD card automatically.
// This is deliberately hard-coded as false for a public GitHub project.
const bool FORMAT_SD_IF_MOUNT_FAILS = false;

// ============================================================================
// FILES
// ============================================================================

const char* LOG_FILE = "/TIMELAPSE.CSV";
const char* SUMMARY_FILE = "/SUMMARY.TXT";

// ============================================================================
// CAMERA PINOUT
// ============================================================================

// AI-Thinker / AZ-Delivery ESP32-CAM style pin mapping.

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
// RTC MEMORY
// ============================================================================

const uint32_t RTC_MAGIC = 0x544C5031; // "TLP1"

RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR char rtcRunId[32] = "";
RTC_DATA_ATTR uint32_t rtcImageCount = 0;
RTC_DATA_ATTR uint32_t rtcFailedImages = 0;
RTC_DATA_ATTR uint64_t rtcBootCount = 0;

// ============================================================================
// RUNTIME STATE
// ============================================================================

uint32_t currentImageNumber = 0;
uint32_t currentImageBytes = 0;
uint32_t currentCaptureTimeMs = 0;
uint64_t currentElapsedFromFirstSec = 0;

// JPEG copied from the camera framebuffer into PSRAM/heap.
// This lets us deinitialize the camera before using SPI SD.
uint8_t* photoBuffer = nullptr;
size_t photoBufferSize = 0;

// ============================================================================
// SERIAL
// ============================================================================

void logSerial(const String& message) {
  if (ENABLE_SERIAL_LOG) {
    Serial.println(message);
  }
}

// ============================================================================
// CONFIGURATION VALIDATION
// ============================================================================

bool validateConfiguration() {
  if (RUN_ID == nullptr || strlen(RUN_ID) == 0) {
    Serial.println("ERROR: RUN_ID cannot be empty.");
    return false;
  }

  if (strlen(RUN_ID) >= sizeof(rtcRunId)) {
    Serial.println("ERROR: RUN_ID is too long.");
    Serial.println("Maximum length: 31 characters.");
    return false;
  }

  if (SLEEP_INTERVAL_SECONDS == 0) {
    Serial.println("ERROR: SLEEP_INTERVAL_SECONDS must be > 0.");
    return false;
  }

  if (PHOTO_JPEG_QUALITY < 10 || PHOTO_JPEG_QUALITY > 63) {
    Serial.println("ERROR: PHOTO_JPEG_QUALITY should be 10-63.");
    return false;
  }

  return true;
}

// ============================================================================
// RTC STATE
// ============================================================================

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

// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================

bool initialiseCamera() {
  camera_config_t config = {};

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
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    // Fallback for boards without PSRAM.
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  logSerial("Initializing camera...");
  logSerial(String("PSRAM: ") + (psramFound() ? "YES" : "NO"));

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    logSerial(
        "ERROR: Camera initialization failed: 0x" +
        String((uint32_t)err, HEX)
    );
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();

  if (sensor != nullptr) {
    if (psramFound()) {
      sensor->set_framesize(sensor, PHOTO_FRAME_SIZE);
      sensor->set_quality(sensor, PHOTO_JPEG_QUALITY);
    } else {
      sensor->set_framesize(sensor, FRAMESIZE_VGA);
      sensor->set_quality(sensor, 15);
    }
  }

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  logSerial("Camera initialized successfully.");

  return true;
}

// ============================================================================
// SD_MMC
// ============================================================================

bool initialiseSD_MMC() {
  delay(SD_STARTUP_DELAY_MS);

  logSerial("Initializing SD_MMC...");
  logSerial(
      String("SD_MMC bus mode: ") +
      (SD_MMC_USE_1BIT_MODE ? "1-bit" : "4-bit")
  );

  bool mounted = SD_MMC.begin(
      "/sdcard",
      SD_MMC_USE_1BIT_MODE,
      FORMAT_SD_IF_MOUNT_FAILS
  );

  if (!mounted) {
    logSerial("ERROR: SD_MMC mount failed.");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    logSerial("ERROR: No SD card detected.");

    SD_MMC.end();
    return false;
  }

  uint64_t cardSizeMB =
      SD_MMC.cardSize() / (1024ULL * 1024ULL);

  logSerial(
      "SD_MMC card detected: " +
      String((unsigned long long)cardSizeMB) +
      " MiB"
  );

  return true;
}

// ============================================================================
// SPI SD
// ============================================================================

bool initialiseSD_SPI() {
  delay(SD_STARTUP_DELAY_MS);

  logSerial("Initializing SPI SD...");
  logSerial(
      "SPI frequency: " +
      String(SD_SPI_FREQUENCY_HZ) +
      " Hz"
  );

  // Make sure no previous SD/SPI session is active.
  SD.end();
  SPI.end();

  delay(100);

  // ESP32-CAM onboard SD socket.
  SPI.begin(
      SD_SPI_SCK_PIN,
      SD_SPI_MISO_PIN,
      SD_SPI_MOSI_PIN,
      SD_SPI_CS_PIN
  );

  delay(100);

  bool mounted = SD.begin(
      SD_SPI_CS_PIN,
      SPI,
      SD_SPI_FREQUENCY_HZ,
      "/sdcard",
      5,
      false
  );

  if (!mounted) {
    logSerial("ERROR: SPI SD mount failed.");

    SD.end();
    SPI.end();
    return false;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    logSerial("ERROR: No SD card detected.");

    SD.end();
    SPI.end();
    return false;
  }

  uint64_t cardSizeMB =
      SD.cardSize() / (1024ULL * 1024ULL);

  logSerial(
      "SPI SD card detected: " +
      String((unsigned long long)cardSizeMB) +
      " MiB"
  );

  return true;
}

// ============================================================================
// SD ABSTRACTION
// ============================================================================
//
// The remainder of the sketch can use these helper functions regardless of
// which interface is selected above.
// ============================================================================

bool initialiseSD() {
  if (USE_SPI_SD) {
    return initialiseSD_SPI();
  }

  return initialiseSD_MMC();
}

void closeSD() {
  if (USE_SPI_SD) {
    SD.end();
    SPI.end();
  } else {
    SD_MMC.end();
  }
}

bool sdExists(const char* path) {
  if (USE_SPI_SD) {
    return SD.exists(path);
  }

  return SD_MMC.exists(path);
}

bool sdRemove(const char* path) {
  if (USE_SPI_SD) {
    return SD.remove(path);
  }

  return SD_MMC.remove(path);
}

File sdOpen(const char* path, const char* mode) {
  if (USE_SPI_SD) {
    return SD.open(path, mode);
  }

  return SD_MMC.open(path, mode);
}

// ============================================================================
// CSV LOG
// ============================================================================

void ensureCsvHeader() {
  if (!ENABLE_CSV_LOG) {
    return;
  }

  if (sdExists(LOG_FILE)) {
    return;
  }

  File file = sdOpen(LOG_FILE, FILE_WRITE);

  if (!file) {
    logSerial("WARNING: Could not create CSV log.");
    return;
  }

  file.println(
      "run_id,"
      "image_number,"
      "filename,"
      "elapsed_from_first_s,"
      "planned_interval_s,"
      "image_bytes,"
      "capture_time_ms,"
      "boot_count,"
      "failed_images"
  );

  file.close();
}

void writeCsvLog(const String& filename) {
  if (!ENABLE_CSV_LOG) {
    return;
  }

  File file = sdOpen(LOG_FILE, FILE_APPEND);

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

  file.print((unsigned long long)currentElapsedFromFirstSec);
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

// ============================================================================
// SUMMARY
// ============================================================================

void updateSummary() {
  if (!ENABLE_SUMMARY_FILE) {
    return;
  }

  // Remove the previous summary so the file is replaced rather than appended.
  if (sdExists(SUMMARY_FILE)) {
    sdRemove(SUMMARY_FILE);
  }

  File file = sdOpen(SUMMARY_FILE, FILE_WRITE);

  if (!file) {
    logSerial("WARNING: Could not update summary file.");
    return;
  }

  file.println("ESP32-CAM TIMELAPSE SUMMARY");
  file.println("===========================");
  file.println();

  file.print("Run ID: ");
  file.println(RUN_ID);

  file.print("SD interface: ");
  file.println(USE_SPI_SD ? "SPI" : "SD_MMC");

  if (USE_SPI_SD) {
    file.print("SPI frequency: ");
    file.print(SD_SPI_FREQUENCY_HZ);
    file.println(" Hz");
  } else {
    file.print("SD_MMC bus mode: ");
    file.println(SD_MMC_USE_1BIT_MODE ? "1-bit" : "4-bit");
  }

  file.print("Images saved: ");
  file.println((unsigned long)rtcImageCount);

  file.print("Failed image attempts: ");
  file.println((unsigned long)rtcFailedImages);

  file.print("Sleep interval: ");
  file.print(SLEEP_INTERVAL_SECONDS);
  file.println(" seconds");

  file.print("JPEG quality: ");
  file.println(PHOTO_JPEG_QUALITY);

  file.print("PSRAM detected: ");
  file.println(psramFound() ? "YES" : "NO");

  file.print("Estimated time from first to last image: ");

  if (rtcImageCount > 0) {
    uint64_t elapsed =
        (uint64_t)(rtcImageCount - 1) *
        SLEEP_INTERVAL_SECONDS;

    file.print((unsigned long long)elapsed);
    file.println(" seconds");

    file.print("Estimated duration: ");
    file.print((unsigned long long)(elapsed / 3600ULL));
    file.print(" h ");

    file.print(
        (unsigned long long)((elapsed % 3600ULL) / 60ULL)
    );

    file.print(" min ");

    file.print(
        (unsigned long long)(elapsed % 60ULL)
    );

    file.println(" s");
  } else {
    file.println("0 seconds");
  }

  file.print("Boot count: ");
  file.println((unsigned long long)rtcBootCount);

  file.println();
  file.println(
      "Timing note: elapsed time is estimated from the configured"
  );
  file.println(
      "sleep interval. Actual photo intervals are slightly longer"
  );
  file.println(
      "because of boot, camera, SD and file-write overhead."
  );

  file.close();
}

// ============================================================================
// CAPTURE JPEG TO PSRAM/HEAP
// ============================================================================
//
// The JPEG is copied out of the camera framebuffer before the camera is
// deinitialized. This is important when using SPI SD because the camera and
// SD subsystems are intentionally operated sequentially.
// ============================================================================

bool capturePhotoToBuffer() {
  if (USE_FLASH) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(100);
  }

  uint32_t captureStart = millis();

  camera_fb_t* fb = esp_camera_fb_get();

  currentCaptureTimeMs =
      millis() - captureStart;

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

  currentElapsedFromFirstSec =
      (uint64_t)(currentImageNumber - 1) *
      SLEEP_INTERVAL_SECONDS;

  // Release any previous buffer.
  if (photoBuffer != nullptr) {
    free(photoBuffer);
    photoBuffer = nullptr;
    photoBufferSize = 0;
  }

  photoBufferSize = fb->len;

  // Prefer PSRAM for the JPEG copy.
  photoBuffer =
      (uint8_t*)ps_malloc(photoBufferSize);

  // Fallback to normal heap if PSRAM allocation failed.
  if (photoBuffer == nullptr) {
    photoBuffer =
        (uint8_t*)malloc(photoBufferSize);
  }

  if (photoBuffer == nullptr) {
    rtcFailedImages++;

    esp_camera_fb_return(fb);

    logSerial(
        "ERROR: Could not allocate JPEG buffer."
    );

    return false;
  }

  memcpy(
      photoBuffer,
      fb->buf,
      photoBufferSize
  );

  esp_camera_fb_return(fb);

  return true;
}

// ============================================================================
// SAVE JPEG BUFFER
// ============================================================================

bool savePhotoBuffer() {
  if (photoBuffer == nullptr ||
      photoBufferSize == 0) {

    rtcFailedImages++;

    logSerial(
        "ERROR: JPEG buffer is empty."
    );

    return false;
  }

  char filenameBuffer[32];

  snprintf(
      filenameBuffer,
      sizeof(filenameBuffer),
      "/IMG_%06lu.jpg",
      (unsigned long)currentImageNumber
  );

  String filename(filenameBuffer);

  File file =
      sdOpen(filename.c_str(), FILE_WRITE);

  if (!file) {
    rtcFailedImages++;

    logSerial(
        "ERROR: Could not open " +
        filename +
        " for writing."
    );

    return false;
  }

  size_t written =
      file.write(
          photoBuffer,
          photoBufferSize
      );

  file.flush();
  file.close();

  if (written != photoBufferSize) {
    rtcFailedImages++;

    logSerial(
        "ERROR: Incomplete write for " +
        filename +
        "."
    );

    return false;
  }

  rtcImageCount++;

  writeCsvLog(filename);
  updateSummary();

  logSerial(
      "Saved " +
      filename +
      " | " +
      String((unsigned long)currentImageBytes) +
      " bytes" +
      " | capture " +
      String(currentCaptureTimeMs) +
      " ms" +
      " | image " +
      String((unsigned long)rtcImageCount)
  );

  return true;
}

// ============================================================================
// DEEP SLEEP
// ============================================================================

void goToDeepSleep() {
  if (photoBuffer != nullptr) {
    free(photoBuffer);
    photoBuffer = nullptr;
    photoBufferSize = 0;
  }

  if (USE_FLASH) {
    digitalWrite(FLASH_LED_PIN, LOW);
  }

  closeSD();

  // Camera might already have been deinitialized.
  // Calling deinit again is avoided by managing it in setup().
  logSerial(
      "Sleeping for " +
      String(SLEEP_INTERVAL_SECONDS) +
      " seconds..."
  );

  Serial.flush();

  esp_sleep_enable_timer_wakeup(
      (uint64_t)SLEEP_INTERVAL_SECONDS *
      1000000ULL
  );

  esp_deep_sleep_start();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  if (ENABLE_SERIAL_LOG) {
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32-CAM TIMELAPSE");
    Serial.println("========================================");
  }

  if (!validateConfiguration()) {
    while (true) {
      delay(1000);
    }
  }

  initialiseRtcState();

  esp_sleep_wakeup_cause_t wakeCause =
      esp_sleep_get_wakeup_cause();

  logSerial("Run ID: " + String(RUN_ID));

  logSerial(
      "Wake cause: " +
      String((int)wakeCause)
  );

  logSerial(
      "Images already saved: " +
      String((unsigned long)rtcImageCount)
  );

  logSerial(
      "SD interface: " +
      String(USE_SPI_SD ? "SPI" : "SD_MMC")
  );

  // ---------------------------------------------------------
  // Hardware startup delay
  // ---------------------------------------------------------

  delay(CAMERA_SETTLE_MS);

  // ---------------------------------------------------------
  // Stop permanently when MAX_IMAGES is reached.
  // ---------------------------------------------------------

  if (MAX_IMAGES > 0 &&
      rtcImageCount >= MAX_IMAGES) {

    logSerial(
        "Maximum image count reached. Stopping."
    );

    esp_sleep_disable_wakeup_source(
        ESP_SLEEP_WAKEUP_TIMER
    );

    esp_deep_sleep_start();
  }

  // ---------------------------------------------------------
  // CAMERA
  // ---------------------------------------------------------

  if (!initialiseCamera()) {
    rtcFailedImages++;

    logSerial("Camera initialization failed.");

    if (STOP_ON_IMAGE_ERROR) {
      while (true) {
        delay(1000);
      }
    }

    goToDeepSleep();
  }

  bool captureOk = capturePhotoToBuffer();

  // Camera is no longer required after the JPEG has been copied.
  esp_camera_deinit();

  if (!captureOk) {
    if (STOP_ON_IMAGE_ERROR) {
      logSerial("Stopping because photo capture failed.");

      while (true) {
        delay(1000);
      }
    }

    goToDeepSleep();
  }

  // ---------------------------------------------------------
  // SD CARD
  // ---------------------------------------------------------

  if (!initialiseSD()) {
    logSerial("SD initialization failed.");

    if (STOP_ON_IMAGE_ERROR) {
      if (photoBuffer != nullptr) {
        free(photoBuffer);
        photoBuffer = nullptr;
        photoBufferSize = 0;
      }

      while (true) {
        delay(1000);
      }
    }

    goToDeepSleep();
  }

  // ---------------------------------------------------------
  // LOGGING
  // ---------------------------------------------------------

  ensureCsvHeader();

  // ---------------------------------------------------------
  // SAVE PHOTO
  // ---------------------------------------------------------

  bool saveOk = savePhotoBuffer();

  if (!saveOk) {
    logSerial("ERROR: Could not save the JPEG.");

    if (STOP_ON_IMAGE_ERROR) {
      closeSD();

      if (photoBuffer != nullptr) {
        free(photoBuffer);
        photoBuffer = nullptr;
        photoBufferSize = 0;
      }

      while (true) {
        delay(1000);
      }
    }
  }

  // ---------------------------------------------------------
  // SLEEP
  // ---------------------------------------------------------

  goToDeepSleep();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  // Never reached.
  // The ESP32 enters deep sleep from setup().
}
