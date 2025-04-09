#include <bluefruit.h>
#include <SparkFun_KX13X.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>

// Constants
#define SAMPLE_RATE_HZ 8000         // 8 kHz sampling
#define SAMPLING_DURATION_MS 11000  // 11 seconds
#define BUFFER_SIZE 800             // 100ms worth of samples at 8kHz
#define NUM_BUFFERS 2               // Double buffer
#define BYTES_PER_SAMPLE 6          // 2 bytes each for X, Y, Z
#define TOTAL_SAMPLES (SAMPLE_RATE_HZ * (SAMPLING_DURATION_MS / 1000))
#define TOTAL_BYTES (TOTAL_SAMPLES * BYTES_PER_SAMPLE)
#define FLASH_PAGE_SIZE 4096  // 4KB pages for typical QSPI flash

// SPI Flash setup
// Use built-in QSPI Flash on XIAO nRF52840
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// KX134 sensor
SparkFun_KX134 kxAccel;
rawOutputData accelData;  // Structure to hold raw accelerometer data

// Buffer management
struct AccelData {
  int16_t x;
  int16_t y;
  int16_t z;
};

volatile AccelData sampleBuffers[NUM_BUFFERS][BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;
volatile uint8_t activeBuffer = 0;
volatile uint8_t writeBuffer = 1;
volatile bool bufferReady = false;
volatile bool samplingActive = false;
volatile uint32_t samplesCollected = 0;
volatile uint32_t flashWriteAddress = 0;

// BLE Service and Characteristic UUIDs
#define DATA_SERVICE_UUID "1ABC2DEF-3456-7890-1234-56789ABCDEF0"
#define COMMAND_CHAR_UUID "2ABC2DEF-3456-7890-1234-56789ABCDEF0"
#define DATA_CHAR_UUID "3ABC2DEF-3456-7890-1234-56789ABCDEF0"
#define STATUS_CHAR_UUID "4ABC2DEF-3456-7890-1234-56789ABCDEF0"

BLEService dataService(DATA_SERVICE_UUID);
BLECharacteristic commandChar(COMMAND_CHAR_UUID, BLEWrite, 1);
BLECharacteristic dataChar(DATA_CHAR_UUID, BLERead | BLENotify, 20);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, 4);

// Command definitions
#define CMD_START_SAMPLING 1
#define CMD_STOP_SAMPLING 2
#define CMD_READ_DATA 3
#define CMD_ERASE_FLASH 4

// Status codes for statusChar
#define STATUS_IDLE 0
#define STATUS_SAMPLING 1
#define STATUS_WRITING 2
#define STATUS_COMPLETE 3
#define STATUS_ERROR 0xFF

// Timer for precise sampling
NRF_TIMER_Type* sampleTimer = NRF_TIMER4;

// Global for data transfer
uint32_t transferAddress = 0;
bool dataTransferActive = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("KX134 Data Acquisition System");

  // Initialize QSPI Flash
  if (!flash.begin()) {
    Serial.println("Failed to initialize flash!");
    while (1) delay(10);
  }

  Serial.print("Flash size: ");
  Serial.println(flash.size());

  // Initialize KX134 sensor over SPI
  Wire.begin();
  if (!kxAccel.begin()) {
    Serial.println("Could not communicate with the the KX13X!");
    while (1) delay(10);
  }

  // Configure KX134
  kxAccel.softwareReset();  // Reset the accelerometer
  // kxAccel.enableContinuousMode(false); // Put in standby mode
  kxAccel.forceSleep();
  kxAccel.setRange(SFE_KX134_RANGE16G);  // Use proper constant
  kxAccel.setOutputDataRate(14);         // Use proper constant (12.5kHz)
  kxAccel.enableDataEngine();

  // Setup BLE
  setupBLE();

  // Configure hardware timer for precise sampling
  setupSampleTimer();
}

void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setName("KX134-Logger");

  // Setup the data service
  dataService.begin();

  // Command characteristic
  commandChar.setProperties(CHR_PROPS_WRITE);
  commandChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  commandChar.setFixedLen(1);
  commandChar.begin();
  commandChar.setWriteCallback(commandCallback);

  // Data characteristic for retrieving stored data
  dataChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  dataChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  dataChar.setFixedLen(20);
  dataChar.begin();

  // Status characteristic
  statusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  statusChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  statusChar.setFixedLen(4);
  statusChar.begin();

  // Set initial status
  uint32_t status = STATUS_IDLE;
  statusChar.write(&status, sizeof(status));

  // Start advertising
  Bluefruit.Advertising.addService(dataService);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);  // 0 = Don't stop advertising

  Serial.println("BLE advertising started");
}

void setupSampleTimer() {
  // Use nRF52 hardware timer for precise sampling at 8kHz
  // For nRF52840, we'll use the built-in NRF_TIMER functions

  // Configure timer interrupt for 8kHz sampling rate
  sampleTimer->TASKS_STOP = 1;                            // Stop timer
  sampleTimer->MODE = TIMER_MODE_MODE_Timer;              // Set timer mode
  sampleTimer->BITMODE = TIMER_BITMODE_BITMODE_16Bit;     // 16-bit mode
  sampleTimer->PRESCALER = 4;                             // Prescaler 4 gives 4MHz timer frequency
  sampleTimer->CC[0] = 500;                               // 500 ticks at 4MHz = 125us = 8kHz
  sampleTimer->INTENSET = TIMER_INTENSET_COMPARE0_Msk;    // Enable CC[0] interrupt
  sampleTimer->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;  // Auto-clear on compare

  // Enable timer interrupt
  NVIC_SetPriority(TIMER4_IRQn, 0);  // Highest priority
  NVIC_EnableIRQ(TIMER4_IRQn);

  // Do not start the timer yet - will be started when sampling begins
}

void commandCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len != 1) return;

  uint8_t command = data[0];
  Serial.print("Command received: ");
  Serial.println(command);

  switch (command) {
    case CMD_START_SAMPLING:
      startSampling();
      break;

    case CMD_STOP_SAMPLING:
      stopSampling();
      break;

    case CMD_READ_DATA:
      // Start data transfer process
      // This will be handled in the main loop
      startDataTransfer();
      break;

    case CMD_ERASE_FLASH:
      eraseFlash();
      break;
  }
}

void startSampling() {
  if (samplingActive) return;

  Serial.println("Starting sampling");

  // Reset variables
  bufferIndex = 0;
  activeBuffer = 0;
  writeBuffer = 1;
  bufferReady = false;
  samplesCollected = 0;
  flashWriteAddress = 0;

  // Update status
  uint32_t status = STATUS_SAMPLING;
  statusChar.write(&status, sizeof(status));

  // Erase the required flash space first
  uint32_t pagesToErase = (TOTAL_BYTES + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
  for (uint32_t i = 0; i < pagesToErase; i++) {
    flash.eraseSector(i * FLASH_PAGE_SIZE / FLASH_PAGE_SIZE);  // Fix erase call
  }

  // Start the accelerometer
  // kxAccel.enableContinuousMode(true); // Set to operating mode
  kxAccel.forceWake();

  // Start the timer for sampling
  samplingActive = true;
  sampleTimer->TASKS_START = 1;
}

void stopSampling() {
  if (!samplingActive) return;

  // Stop timer
  sampleTimer->TASKS_STOP = 1;

  // Put accelerometer in standby mode
  // kxAccel.enableContinuousMode(false); // Set to standby mode
  kxAccel.forceSleep();

  // Finish any pending writes
  if (bufferReady) {
    writeBufferToFlash();
  }

  // Update status
  samplingActive = false;
  uint32_t status = STATUS_COMPLETE;
  statusChar.write(&status, sizeof(status));

  Serial.print("Sampling complete. Collected ");
  Serial.print(samplesCollected);
  Serial.println(" samples.");
}

extern "C" void TIMER4_IRQHandler(void) {
  // Clear the interrupt
  sampleTimer->EVENTS_COMPARE[0] = 0;

  // Call our sampling function
  sampleISR();
}

void sampleISR() {
  if (!samplingActive) return;

  // Read sensor data
  kxAccel.getRawAccelData(&accelData);  // Pass the structure to receive the data

  // Store in current buffer
  sampleBuffers[activeBuffer][bufferIndex].x = accelData.xData;
  sampleBuffers[activeBuffer][bufferIndex].y = accelData.yData;
  sampleBuffers[activeBuffer][bufferIndex].z = accelData.zData;

  bufferIndex++;
  samplesCollected++;

  // Check if buffer is full
  if (bufferIndex >= BUFFER_SIZE) {
    // Swap buffers
    uint8_t temp = activeBuffer;
    activeBuffer = writeBuffer;
    writeBuffer = temp;

    bufferIndex = 0;
    bufferReady = true;
  }

  // Check if we've collected all samples
  if (samplesCollected >= TOTAL_SAMPLES) {
    // Auto-stop after reaching desired sample count
    samplingActive = false;
    sampleTimer->TASKS_STOP = 1;  // Disable the timer
  }
}

void writeBufferToFlash() {
  // Update status
  uint32_t status = STATUS_WRITING;
  statusChar.write(&status, sizeof(status));

  // Write the filled buffer to flash
  flash.writeBuffer(flashWriteAddress,
                    (uint8_t*)sampleBuffers[writeBuffer],
                    BUFFER_SIZE * sizeof(AccelData));

  flashWriteAddress += BUFFER_SIZE * sizeof(AccelData);
  bufferReady = false;

  // Update status back to sampling or complete
  status = samplingActive ? STATUS_SAMPLING : STATUS_COMPLETE;
  statusChar.write(&status, sizeof(status));
}

void startDataTransfer() {
  Serial.println("Starting data transfer");
  transferAddress = 0;
  dataTransferActive = true;

  // The actual transfer will be handled in the main loop
}

void eraseFlash() {
  Serial.println("Erasing flash");

  // Update status
  uint32_t status = STATUS_WRITING;
  statusChar.write(&status, sizeof(status));

  // Erase the portion of flash we use
  uint32_t pagesToErase = (TOTAL_BYTES + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
  for (uint32_t i = 0; i < pagesToErase; i++) {
    flash.eraseSector(i * FLASH_PAGE_SIZE / FLASH_PAGE_SIZE);  // Fix erase call
  }

  flashWriteAddress = 0;
  samplesCollected = 0;

  // Update status
  status = STATUS_IDLE;
  statusChar.write(&status, sizeof(status));

  Serial.println("Flash erased");
}

void handleDataTransfer() {
  if (!dataTransferActive) return;

  // Check if we've sent all data
  if (transferAddress >= flashWriteAddress) {
    dataTransferActive = false;
    return;
  }

  // Read a chunk of data from flash
  uint8_t chunk[20];  // Size matches our BLE characteristic
  uint32_t readSize = min(20UL, flashWriteAddress - transferAddress);

  flash.readBuffer(transferAddress, chunk, readSize);  // Fix read call
  dataChar.notify(chunk, readSize);

  transferAddress += readSize;

  // Small delay to not overwhelm BLE
  delay(10);
}

void loop() {
  // Check if a buffer is ready to write to flash
  if (bufferReady && !dataTransferActive) {
    writeBufferToFlash();
  }

  // Handle data transfer if active
  handleDataTransfer();

  // Auto-stop sampling after duration
  if (samplingActive && samplesCollected >= TOTAL_SAMPLES) {
    stopSampling();
  }
}
