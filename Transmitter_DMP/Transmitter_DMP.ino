#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <esp_now.h>
#include <WiFi.h>

MPU6050 mpu;
#define INTERRUPT_PIN 23

volatile bool mpuInterrupt = false;
volatile uint8_t fifoBuffer[64];
uint8_t packetSize;
uint8_t mpuIntStatus;
volatile uint16_t fifoCount;

Quaternion q;
VectorFloat gravity;
float ypr[3];
float yaw, pitch, roll;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float alpha = 0.8; // Complementary constant

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0xA6, 0xF3, 0x30};

// Define a data structure
typedef struct struct_message {
  float pitch;
  float roll;
  //float yaw;
} struct_message;

// Create a structured object
struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

void dmpDataReady() {
    mpuInterrupt = true;
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}

void setup() {
  Wire.begin(21, 22);
  mpu.initialize();

  // Enable DMP
  Serial.println("Initializing DMP...");
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(90);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuInterrupt = false;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP Initialization failed.");
  }

  // Set up Serial Monitor
  Serial.begin(9600);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (!mpuInterrupt) {
    return;
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  if (mpuIntStatus & 0x10) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
    
    uint8_t fifoData[packetSize];
    mpu.getFIFOBytes(fifoData, packetSize);
    mpu.resetFIFO();
    mpu.dmpGetQuaternion(&q, fifoData);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //yaw = ypr[0];
    pitch = ypr[1];
    roll = ypr[2];

    // Update the structured data
    myData.pitch = pitch * 180/M_PI;
    myData.roll = roll * 180/M_PI;
    //myData.yaw = yaw * 180/M_PI;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    } else {
      Serial.println("Sending error");
    }
  }
  delay(90); // Adjust the delay as needed
}