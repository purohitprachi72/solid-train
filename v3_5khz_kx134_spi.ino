#include <Wire.h>
#include <bluefruit.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#include <SparkFun_KX13X.h>  // KX134 Sensor

// ====== CONFIGURATION ======
#define DEVICE_NAME "DMG"
#define DEBUG 1
#define CS_PIN 1

// Sampling configuration
#define LPM_SAMPLE_RATE 5     // Low power mode sampling rate (Hz)
#define HPM_SAMPLE_RATE 5000  // High power mode sampling rate (Hz)
#define DURATION_MS 6000      // Sampling duration in milliseconds

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Data buffer configuration
#define NUM_VALUES 3  // X, Y, Z values per sample
#define NUM_SAMPLES (HPM_SAMPLE_RATE * (DURATION_MS / 1000))  // Total samples to collect
#define MAX_BLE_PACKET_SIZE 240

// Global variables
int16_t dataBuffer[NUM_SAMPLES * NUM_VALUES];  // Buffer to hold all acceleration data
int data_index = 0;  // Current position in data buffer
uint8_t connection_status = 0;  // 0: disconnected, 1: connected, 2: sampling, 3: sending
bool sampling_complete = false;

// BLE service and characteristic
BLEService customService(SERVICE_UUID);
BLECharacteristic customCharacteristic(CHARACTERISTIC_UUID);

// Synchronization
SemaphoreHandle_t taskSemaphore = NULL;
TaskHandle_t samplingTaskHandle = NULL;
TaskHandle_t sendingTaskHandle = NULL;

// Flash transport for power management
Adafruit_FlashTransport_QSPI flashTransport;

// KX-134 sensor object (SPI mode)
SparkFun_KX134_SPI kxAccel;
rawOutputData myData;  // Raw output data structure

// ====== DEBUG MACROS ======
#if DEBUG == 1
  #define SERIAL_BEGIN(X) Serial.begin(X)
  #define SERIAL_PRINT(X) Serial.print(X)
  #define SERIAL_PRINT_F(X, Y) Serial.print(X, Y)
  #define SERIAL_PRINTLN(X) Serial.println(X)
  #define SERIAL_PRINTLN_F(X, Y) Serial.println(X, Y)
#else
  #define SERIAL_BEGIN(X)
  #define SERIAL_PRINT(X)
  #define SERIAL_PRINT_F(X, Y)
  #define SERIAL_PRINTLN(X)
  #define SERIAL_PRINTLN_F(X, Y)
#endif

// ====== UTILS ======
inline TickType_t calculateFrequency(uint16_t rate) {
  return pdMS_TO_TICKS(1000 / rate);  // Convert Hz to ticks
}

void disable_QSPI() {
  NRF_POWER->DCDCEN = 1;
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // Enter deep power-down
  delayMicroseconds(5);
  flashTransport.end();
}

// ====== BLE SETUP ======
void bleAdvInit(void) {
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);
  Bluefruit.Advertising.setFastTimeout(1);
}

void startAdv(void) {
  SERIAL_PRINTLN("Starting advertisement...");
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(customService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);  // Advertise indefinitely
}

// ====== SENSOR CONFIGURATION ======
// Enable high sampling rate (5kHz) for data collection
void enableHighSampling() {
  kxAccel.enableAccel(false);  // Disable accelerometer during configuration
  kxAccel.setRange(SFE_KX134_RANGE64G);
  kxAccel.enableDataEngine();
  
  // Set to 5kHz (ODR value 13 is for ~6.4kHz, closest available to 5kHz)
  // From the datasheet, use the appropriate ODR value for closest to 5kHz
  kxAccel.setOutputDataRate(13);  
  
  kxAccel.enableAccel();  // Re-enable accelerometer with new settings
  SERIAL_PRINTLN("High sampling mode enabled (5kHz)");
}

// Enable low power mode (5Hz) when idle
void enableLowSampling() {
  kxAccel.enableAccel(false);  // Disable accelerometer during configuration
  kxAccel.setRange(SFE_KX134_RANGE64G);
  
  // Set to 5Hz (ODR value 2 is for 12.5Hz, closest available to 5Hz)
  kxAccel.setOutputDataRate(2);  
  
  kxAccel.enableAccel();  // Re-enable accelerometer with new settings
  SERIAL_PRINTLN("Low sampling mode enabled (5Hz)");
}

// ====== BLE CALLBACKS ======
void connect_callback(uint16_t conn_handle) {
  SERIAL_PRINTLN("Connected!");
  connection_status = 1;
  
  // Request connection parameters for better throughput
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(6, 8);
  
  // Start the sampling task
  vTaskResume(samplingTaskHandle);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  SERIAL_PRINT("Disconnected, reason = 0x");
  SERIAL_PRINTLN_F(reason, HEX);
  
  // Reset states
  data_index = 0;
  connection_status = 0;
  sampling_complete = false;
  
  // Enable low power sampling mode
  enableLowSampling();
  
  // Start advertising again
  startAdv();
}

// ====== DATA TASKS ======
// Task for sampling data
void samplingTask(void* pvParameters) {
  while (1) {
    // Suspend until needed
    vTaskSuspend(NULL);
    
    SERIAL_PRINTLN("Starting data collection...");
    
    // Reset data index
    data_index = 0;
    
    // Enable high sampling rate
    enableHighSampling();
    
    // Mark as in sampling state
    connection_status = 2;
    
    // Calculate tick rate for desired frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = calculateFrequency(HPM_SAMPLE_RATE);
    
    // Collect samples
    while (data_index < NUM_SAMPLES * NUM_VALUES) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      
      if (kxAccel.getRawAccelData(&myData)) {
        // Check if buffer has space
        if (data_index < NUM_SAMPLES * NUM_VALUES - 3) {
          dataBuffer[data_index++] = myData.xData;
          dataBuffer[data_index++] = myData.yData;
          dataBuffer[data_index++] = myData.zData;
        }
      }
    }
    
    SERIAL_PRINT("Sampling complete. Collected ");
    SERIAL_PRINT(data_index / 3);
    SERIAL_PRINTLN(" samples.");
    
    // Switch to low power mode
    enableLowSampling();
    
    sampling_complete = true;
    connection_status = 3;  // Change to sending state
    
    // Resume sending task
    vTaskResume(sendingTaskHandle);
  }
}

// Task for sending data over BLE
void sendingTask(void* pvParameters) {
  while (1) {
    // Suspend until needed
    vTaskSuspend(NULL);
    
    SERIAL_PRINTLN("Starting data transmission...");
    
    // Only send if connected
    if (Bluefruit.connected() && sampling_complete) {
      int sendCount = 0;
      int sendRetryCount = 0;
      
      // Calculate how many samples we can fit in each BLE packet
      // Each sample is 3 values (x,y,z) of 2 bytes each = 6 bytes per sample
      const int bytesPerSample = NUM_VALUES * sizeof(int16_t);
      const int samplesPerBatch = MAX_BLE_PACKET_SIZE / bytesPerSample;
      const int batchSize = samplesPerBatch * bytesPerSample;
      
      // Buffer for a single BLE packet
      uint8_t byteBuffer[MAX_BLE_PACKET_SIZE];
      
      // Send all data in batches
      for (int i = 0; i < data_index; i += (samplesPerBatch * NUM_VALUES)) {
        int sampleCount = min(samplesPerBatch, (data_index - i) / NUM_VALUES);
        int bufferIndex = 0;
        
        // Convert int16_t values to bytes for the current batch
        for (int j = 0; j < sampleCount; j++) {
          int baseIndex = i + j * NUM_VALUES;
          
          // Pack each value (little-endian)
          for (int k = 0; k < NUM_VALUES; k++) {
            int16_t value = dataBuffer[baseIndex + k];
            byteBuffer[bufferIndex++] = value & 0xFF;        // Low byte
            byteBuffer[bufferIndex++] = (value >> 8) & 0xFF; // High byte
          }
        }
        
        // Try to send data
        bool success = false;
        int retries = 0;
        const int maxRetries = 5;
        
        while (!success && retries < maxRetries) {
          success = customCharacteristic.notify(byteBuffer, bufferIndex);
          
          if (success) {
            sendCount++;
          } else {
            retries++;
            sendRetryCount++;
            delay(20);  // Wait before retry
          }
        }
        
        // Small delay between packets to avoid flooding
        delay(10);
      }
      
      SERIAL_PRINT("Batched packets sent: ");
      SERIAL_PRINTLN(sendCount);
      SERIAL_PRINT("Retry attempts: ");
      SERIAL_PRINTLN(sendRetryCount);
      
      // Send final acknowledgment packet
      uint8_t ack[1] = {1};
      customCharacteristic.notify(ack, sizeof(ack));
      
      SERIAL_PRINTLN("All data sent. Disconnecting...");
      
      // Reset for next time
      data_index = 0;
      sampling_complete = false;
      
      // Disconnect from central
      if (Bluefruit.connected()) {
        uint16_t conn_handle = Bluefruit.connHandle();
        Bluefruit.disconnect(conn_handle);
      }
    }
  }
}

// ====== MAIN ======
void setup(void) {
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("DMG Sensor Setup");
  
  // Configure SPI chip select
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // Disable QSPI flash for power saving
  disable_QSPI();
  
  // Initialize SPI and sensor
  SPI.begin();
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
  if (!kxAccel.begin(SPI, settings, CS_PIN)) {
    SERIAL_PRINTLN("ERROR: Could not communicate with the KX13X sensor. Halting.");
    while (1);
  }
  
  SERIAL_PRINTLN("KX13X sensor initialized");
  
  // Reset sensor
  if (kxAccel.softwareReset()) {
    SERIAL_PRINTLN("Sensor reset successful");
  }
  delay(5);
  
  // Configure sensor for low-power initially
  enableLowSampling();
  
  // Create FreeRTOS tasks
  xTaskCreate(samplingTask, "Sampling", 4096, NULL, 2, &samplingTaskHandle);
  xTaskCreate(sendingTask, "Sending", 4096, NULL, 1, &sendingTaskHandle);
  
  // Initialize BLE
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.setTxPower(4);  // Set max power
  Bluefruit.autoConnLed(false);  // Disable LED to save power
  
  // Set callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  // Configure BLE service and characteristic
  customService.begin();
  customCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  customCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  customCharacteristic.setMaxLen(MAX_BLE_PACKET_SIZE);
  customCharacteristic.begin();
  
  // Initialize BLE advertising
  bleAdvInit();
  
  SERIAL_PRINTLN("Setup complete, starting advertisement");
  startAdv();
}

void loop() {
  // Main loop doesn't need to do anything - tasks handle the work
  delay(1000);
}
