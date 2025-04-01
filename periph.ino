#include <SPI.h>
#include <SparkFun_KX13X.h>      // For KX-134 sensor
#include <bluefruit.h>           // Adafruit Bluefruit BLE library for nRF52840
#include <Adafruit_TinyUSB.h>
#include <Adafruit_SPIFlash.h>
#include <PDM.h>                 // PDM microphone library for nRF52840 Sense

// Flash transport for power savings
Adafruit_FlashTransport_QSPI flashTransport;

// KX-134 sensor object; using SPI mode
SparkFun_KX134_SPI kxAccel;

// Use the raw output data structure
// struct rawOutputData
// {
//   int16_t xData;
//   int16_t yData;
//   int16_t zData;
// };

rawOutputData myData;

// PDM buffer and sample variables
#define PDM_BUFFER_SIZE 256
int16_t pdmSampleBuffer[PDM_BUFFER_SIZE];
volatile int16_t pdmSample = 0;      // Current PDM sample value

// Chip select pin for SPI
const byte chipSelect = 1;

// Custom BLE Service and Characteristic UUIDs
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE configuration
#define SEND_INTERVAL_MS 10    // Interval for BLE notifications (10 ms)
#define MAX_RETRY_COUNT 3      // Maximum number of retry attempts
#define RETRY_DELAY_MS 5       // Delay between retries in milliseconds

// Global connection flag and task handle
bool isConnected = false;
static TaskHandle_t dataTaskHandle = NULL;

// BLE Service and Characteristic objects
BLEService customService(SERVICE_UUID);
BLECharacteristic customCharacteristic(CHARACTERISTIC_UUID);

// data packet format: counter | ax | ay | az | pdm
const int numSamples = 16;                     // Number of samples per packet
const int bytesPerSample = 10;                 // 5 values × 2 bytes each
uint8_t packetBuffer[numSamples * bytesPerSample]; // Buffer to hold the formatted data
uint16_t packetCounter = 0;                    // Counter for packets

// PDM data callback
void onPDMdata() {
  // Read the PDM samples
  int bytesRead = PDM.available();
  PDM.read(pdmSampleBuffer, bytesRead);
  
  // Just grab the last sample for simplicity, could implement more complex audio processing here
  if (bytesRead > 0) {
    pdmSample = pdmSampleBuffer[bytesRead/2 - 1]; // Get last sample
  }
}

// Start BLE advertising with custom settings
void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(customService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // 20 ms to 152.5 ms interval
  Bluefruit.Advertising.setFastTimeout(30);    // Fast advertising for 30 seconds
  Bluefruit.Advertising.start(0);              // Continuous advertising
}

// BLE connection callback
void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected");
  isConnected = true;
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(6, 8);  // ~7.5-10 ms connection interval

  // Activate the high-frequency mode when connected
  enableHighSamplingRate();
  
  // Restart PDM if it was stopped
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to restart PDM!");
  }

  // Resume data collection task if suspended
  vTaskResume(dataTaskHandle);
}

// BLE disconnection callback
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  isConnected = false;
  
  // Drop to low-power mode when disconnected
  enableLowPowerMode();
  
  // Stop PDM to save power
  PDM.end();
  
  // Suspend data collection task to save power
  vTaskSuspend(dataTaskHandle);
}

// Enable high sampling rate (8kHz) for connected mode
void enableHighSamplingRate() {
  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX134_RANGE64G);
  kxAccel.enableDataEngine();
  kxAccel.setOutputDataRate(11);  // Set to a value corresponding to ~8KHz
  kxAccel.enableAccel();
}

// Enable low power mode when disconnected
void enableLowPowerMode() {
  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX134_RANGE64G);
  kxAccel.setOutputDataRate(2);  // Lower ODR to save power (12.5Hz)
  kxAccel.enableAccel();
  
  // Can also put the device into low-power mode here
  // For even lower power, could use motion detection interrupts instead of polling
}

// Function to pack data into the format: counter | ax | ay | az | pdm 
void packSensorData(uint8_t* buffer, int sampleIndex, uint16_t counter, int16_t ax, int16_t ay, int16_t az, int16_t pdm) {
  int offset = sampleIndex * bytesPerSample;
  
  // Counter (2 bytes)
  buffer[offset] = counter & 0xFF;
  buffer[offset + 1] = (counter >> 8) & 0xFF;
  
  // Accelerometer data (6 bytes)
  buffer[offset + 2] = ax & 0xFF;
  buffer[offset + 3] = (ax >> 8) & 0xFF;
  buffer[offset + 4] = ay & 0xFF;
  buffer[offset + 5] = (ay >> 8) & 0xFF;
  buffer[offset + 6] = az & 0xFF;
  buffer[offset + 7] = (az >> 8) & 0xFF;
  
  // PDM microphone data (2 bytes)
  buffer[offset + 8] = pdm & 0xFF;
  buffer[offset + 9] = (pdm >> 8) & 0xFF;
}

// Implements a backoff strategy for BLE transmissions
bool sendBLEDataWithRetry(BLECharacteristic& characteristic, const uint8_t* data, uint16_t len) {
  int retryCount = 0;
  bool success = false;
  
  while (retryCount < MAX_RETRY_COUNT && !success) {
    success = characteristic.notify(data, len);
    
    if (!success) {
      // Failed to send, implement backoff strategy
      retryCount++;
      
      if (retryCount < MAX_RETRY_COUNT) {
        // Exponential backoff: delay increases with each retry
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS * (1 << retryCount)));
      }
    }
  }
  
  return success;
}

// Data task: polls sensor data in batches and sends over BLE
void vDataTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SEND_INTERVAL_MS);
  
  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    if (isConnected) {
      // Wait for the next cycle
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      
      // Collect samples and pack them into the buffer
      for (int i = 0; i < numSamples; i++) {
        // Read accelerometer data
        kxAccel.getRawAccelData(&myData);
        
        // Pack the data in the required format (with PDM data)
        packSensorData(packetBuffer, i, packetCounter++, 
                       myData.xData, myData.yData, myData.zData, pdmSample);
      }
      
      // Transmit the data packet over BLE with retry logic
      if (sendBLEDataWithRetry(customCharacteristic, packetBuffer, sizeof(packetBuffer))) {
        // Notification was sent successfully
      } else {
        // Failed to send notification after all retries
        // Could log this or take additional action
        Serial.println("Failed to send BLE notification after max retries");
      }
    } else {
      // Low power mode when disconnected - task should be suspended anyway
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure chip select for SPI
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  // Enable DC-DC converter for power efficiency
  NRF_POWER->DCDCEN = 1;

  // Put QSPI flash into deep power-down mode to reduce consumption
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // Command to enter deep power-down mode
  delayMicroseconds(5);
  flashTransport.end();

  // Initialize SPI and sensor settings
  SPI.begin();
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
  if (!kxAccel.begin(SPI, settings, chipSelect)) {
    Serial.println("Could not communicate with the KX13X sensor. Halting.");
    while (1);
  }

  if (kxAccel.softwareReset()) {
    Serial.println("Sensor reset successful.");
  }
  delay(5);

  // Initialize PDM microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {  // 1 channel, 16kHz sample rate
    Serial.println("Failed to start PDM microphone!");
  }
  // Set gain (between 0 and 80)
  PDM.setGain(40);
  
  // Initial configuration in low-power mode
  enableLowPowerMode();
  
  // Stop PDM initially to save power (will restart when connected)
  PDM.end();

  // --- BLE Initialization ---
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);           // Reduced from 8 to 4 to save power, adjust if needed
  Bluefruit.setName("AxTest");       // Device name
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Setup BLE Service and Characteristic
  customService.begin();
  customCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  customCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  customCharacteristic.setFixedLen(numSamples * bytesPerSample); // Set fixed length for the new format
  customCharacteristic.begin();

  // Start BLE advertising
  startAdv();

  // Create the data task for sensor data transmission
  xTaskCreate(
    vDataTask,                 // Task function
    "DataTask",                // Task name
    configMINIMAL_STACK_SIZE,  // Minimal stack size
    NULL,                      // Task parameters
    1,                         // Task priority
    &dataTaskHandle            // Task handle
  );
  
  // Start with the task suspended until connected
  vTaskSuspend(dataTaskHandle);

  // Suspend the main loop since BLE and data handling are managed by tasks
  suspendLoop();
}

void loop() {
  // Empty loop: BLE events and data transmission are handled in FreeRTOS tasks.
}
