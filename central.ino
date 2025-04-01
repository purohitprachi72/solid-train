#include "BLEDevice.h"

// Custom BLE Service and Characteristic UUIDs - must match the sender
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Data packet format: counter | ax | ay | az | pdm
const int numSamples = 16;                     // Number of samples per packet
const int bytesPerSample = 10;                 // 5 values × 2 bytes each
const int packetSize = numSamples * bytesPerSample; // Total packet size

// Structures to decode the data
struct SensorSample {
  uint16_t counter;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t pdm;
};

SensorSample sensorData[numSamples];
bool deviceConnected = false;
bool doConnect = false;
bool doScan = false;
BLEAdvertisedDevice* myDevice;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

// Task handle for BLE operations
TaskHandle_t bleTaskHandle = nullptr;

// Function to extract sensor data from received buffer
void extractSensorData(uint8_t* buffer) {
  for (int i = 0; i < numSamples; i++) {
    int offset = i * bytesPerSample;
    
    // Extract counter (2 bytes)
    sensorData[i].counter = buffer[offset] | (buffer[offset + 1] << 8);
    
    // Extract accelerometer data (6 bytes)
    sensorData[i].ax = buffer[offset + 2] | (buffer[offset + 3] << 8);
    sensorData[i].ay = buffer[offset + 4] | (buffer[offset + 5] << 8);
    sensorData[i].az = buffer[offset + 6] | (buffer[offset + 7] << 8);
    
    // Extract PDM microphone data (2 bytes)
    sensorData[i].pdm = buffer[offset + 8] | (buffer[offset + 9] << 8);
  }
}

// Custom notification callback
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length == packetSize) {
    extractSensorData(pData);
    
    // Process and use the data as needed
    // For demonstration, print the first sample
    Serial.printf("Counter: %u, Accel: (%d, %d, %d), PDM: %d\n", 
                  sensorData[0].counter, 
                  sensorData[0].ax, 
                  sensorData[0].ay, 
                  sensorData[0].az,
                  sensorData[0].pdm);
    
    // You can process all samples here or send them to another task
    // Example: calculate average acceleration magnitude
    // float totalMag = 0;
    // for (int i = 0; i < numSamples; i++) {
    //   float mag = sqrt(pow(sensorData[i].ax, 2) + 
    //                    pow(sensorData[i].ay, 2) + 
    //                    pow(sensorData[i].az, 2));
    //   totalMag += mag;
    // }
    // Serial.printf("Average acceleration magnitude: %.2f\n", totalMag / numSamples);
  } else {
    Serial.printf("Received notification with unexpected length: %d\n", length);
  }
}

// Connect to the BLE Server
bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the remote device
  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  // Obtain a reference to the service
  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found service");

  // Obtain a reference to the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic UUID: ");
    Serial.println(CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found characteristic");

  // Read the value of the characteristic
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  // Register for notifications
  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Registered for notifications");
  }

  deviceConnected = true;
  return true;
}

// Scan for BLE servers and find the first one with our service UUID
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if the device advertises our service
    if (advertisedDevice.haveServiceUUID() && 
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID)) && 
        advertisedDevice.getName() == "AxTest") {  // Match the name from nRF52840 code
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

// BLE Task that handles connection, scanning, and reconnection
void bleTask(void * parameter) {
  for(;;) {
    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.
    if (doConnect) {
      if (connectToServer()) {
        Serial.println("Connected to the BLE Server.");
        // Request MTU exchange to match the nRF52840 (247 bytes)
        pClient->setMTU(247);
      } else {
        Serial.println("Failed to connect to the server.");
      }
      doConnect = false;
    }
    
    // Disconnected, start scanning again
    if (doScan) {
      BLEDevice::getScan()->start(5, false);  // Scan for 5 seconds
    }
    
    // Check if we're still connected, reconnect if needed
    if (!deviceConnected && !doConnect) {
      delay(1000);  // Wait a bit before scanning again
      doScan = true;
    }
    
    delay(10);  // Small delay to prevent CPU hogging
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 BLE Client...");

  // Initialize BLE
  BLEDevice::init("ESP32_BLE_Client");

  // Retrieve a Scanner and set the callback
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);   // How often the scan occurs (in ms)
  pBLEScan->setWindow(449);      // How long each scan occurs (in ms)
  pBLEScan->setActiveScan(true); // Active scan gets more data

  // Create the BLE task
  xTaskCreatePinnedToCore(
    bleTask,          // Task function
    "BLE Task",       // Task name
    4096,             // Stack size
    NULL,             // Task parameters
    1,                // Priority
    &bleTaskHandle,   // Task handle
    0                 // Core (0 or 1)
  );

  // Start scanning
  doScan = true;
}

void loop() {
  // Main loop can be used for other tasks
  // BLE operations are handled in bleTask
  delay(1000);
}
