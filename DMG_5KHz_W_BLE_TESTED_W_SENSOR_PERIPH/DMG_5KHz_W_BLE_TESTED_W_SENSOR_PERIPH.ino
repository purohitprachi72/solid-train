#include <SPI.h>
#include <SparkFun_KX13X.h>
#include <bluefruit.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define TOTAL_SAMPLES 30000  //5khz for 6 seconds
#define BATCH_SIZE 40
#define TOTAL_NOTIFICATIONS 750

SparkFun_KX134_SPI kxAccel;

rawOutputData myData;
const byte chipSelect = 1;
const int N = 30000;  //TOTAL SAMPLES
int16_t accelData[N][3];

bool startSampling = true;
bool valuesSent = false;
bool connected_flag = false;

int bufIndex = 0;
// bool buffer_not_empty = (bufIndex > 0);

// Setup BLE service and characteristic for accelerometer data
BLEService accelService = BLEService(SERVICE_UUID);
BLECharacteristic accelCharacteristic = BLECharacteristic(CHARACTERISTIC_UUID);

void setup() {
  // put your setup code here, to run once:
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  SPI.begin();

  Serial.begin(115200);
  Serial.println("In setup");
  Serial.flush();

  if (!kxAccel.begin(chipSelect)) {
    Serial.println("Could not communicate with the KX134. Freezing");
    while (1) {}
  }

  Serial.println("Ready");

  if (kxAccel.softwareReset()) {
    Serial.println("Reset.");
  }
  delay(5);

  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX134_RANGE64G);
  kxAccel.setOutputDataRate(13);  //6.4KHz
  kxAccel.enableDataEngine();
  // kxAccel.enableAccel();

  // BLE CONFIGS HERE
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);      // Reduced from 8 to 4 to save power, adjust if needed
  Bluefruit.setName("AxTest");  // Device name
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);

  accelService.begin();
  accelCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  accelCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelCharacteristic.setMaxLen(BATCH_SIZE * sizeof(int16_t) * 3);
  accelCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(accelService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(244, 244);  // 20 ms to 152.5 ms interval
  Bluefruit.Advertising.setFastTimeout(1);      // Fast advertising for 30 seconds
  Bluefruit.Advertising.start(0);               // Continuous advertising

  Serial.println("Started advertising...");
  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Bluefruit.connected()) {
    if (startSampling) {
      kxAccel.enableAccel();

      Serial.println("Starting Sampling...");

      for (int i = 0; i < N; i++) {
        if (kxAccel.dataReady()) {
          kxAccel.getRawAccelData(&myData);

          accelData[i][0] = myData.xData;
          accelData[i][1] = myData.yData;
          accelData[i][2] = myData.zData;

          // bufIndex += 3;
          bufIndex++;
        }
        delayMicroseconds(166);  //for ODR 6.4KHz
      }

      Serial.println("Sampled Successfully!!!");
      startSampling = false;
    }

    if (connected_flag && buffer_not_empty()) {

      Serial.println("Sending samples over BLE...");

      sendBufferInBatches();
      // if all values sent then
      valuesSent = true;

      Serial.println("Sent!");
    } else {
      startSampling = true;
    }

    Serial.flush();
  }

  if (valuesSent) {

    Serial.println("Completed. Shutting down sensor and starting advertising...");

    Bluefruit.Connection(0)->disconnect();
    Bluefruit.Advertising.stop();
    kxAccel.enableAccel(false);
    startSampling = true;
    valuesSent = false;
    bufIndex = 0;
    Bluefruit.Advertising.start(0);

    Serial.println("Started Advertising!");
    Serial.flush();
  }
}

void connect_cb(uint16_t conn_handle) {
  // set_timeout = 30 seconds;
  connected_flag = true;
  Serial.println("Connected");
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(6, 8);  // ~7.5-10 ms connection interval
}

void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  connected_flag = false;
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}

bool buffer_not_empty() {
  return bufIndex > 0;
}

// Function that sends (notifies) a batch of data.
// data: pointer to the first value to send in the batch (int16_t array)
// numValues: the number of int16_t values to send
bool notifyValues(int16_t *data, int numValues) {
  if (accelCharacteristic.notify(data, numValues * 3 * sizeof(int16_t))) {
    return true;
  }
  return false;
}

void sendBufferInBatches() {
  const int MAX_RETRIES = 3;
  int retryCounter = 0;
  int dataSent = 0;
  // Treat the 2D accelerometer data as a contiguous block of memory.
  int16_t *dataBuffer = &accelData[0][0];

  while (dataSent < bufIndex) {
    // Always check if the device is connected.
    if (!connected_flag) {
      Serial.println("Device disconnected. Aborting sending remaining values.");
      return;
    }

    // Determine the size of the current batch.
    int currentBatchSize = (bufIndex - dataSent >= BATCH_SIZE) ? BATCH_SIZE : (bufIndex - dataSent);

    // Try to notify the current batch.
    if (!notifyValues(dataBuffer + dataSent, currentBatchSize)) {
      retryCounter++;
      Serial.print("Notification failed for batch starting at index ");
      Serial.print(dataSent);
      Serial.print(" - Retry count: ");
      Serial.println(retryCounter);

      // After three consecutive failures, check connection status.
      if (retryCounter >= MAX_RETRIES) {
        if (!connected_flag) {
          Serial.println("Device disconnected after multiple notify failures. Aborting sending.");
          return;
        } else {
          Serial.println("Three failures but still connected, skipping this batch and moving to the next values.");
          // Skip the current batch.
          dataSent += currentBatchSize;
          // Reset the retry counter before processing the next batch.
          retryCounter = 0;
          continue;
        }
      }
      // Optional: Delay before retrying the same batch.
      delay(10);
    } else {
      // If the notification succeeded, move to the next batch.
      dataSent += currentBatchSize;
      retryCounter = 0;
    }
  }
}
