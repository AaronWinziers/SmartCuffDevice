#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <string>
#include <sstream>
#include <regex>
#include <helper.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

Adafruit_MPU6050 mpu;

bool deviceConnected = false;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

//BLE server name
#define bleServerName "MyESP32"

// Sensor values
#define SERVICE_UUID "39282bca-6b33-4b76-98ff-48d402a77d0b"
#define CHARACTERISTIC_UUID "040f5df9-7793-4734-850e-37ae7770eb20"
#define DESCRIPTOR_UUID "f811e592-15f6-4c32-a286-c8bf3059a10e"

// Sensor Characteristic
BLECharacteristic mpuDataCharacteristics(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuDataDescriptor(DESCRIPTOR_UUID);

bool transfer;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  init_mpu(mpu);

  Serial.println("");
  delay(100);

  transfer = true;

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *mpuService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  mpuService->addCharacteristic(&mpuDataCharacteristics);
  mpuDataDescriptor.setValue("MPU-6050 Sensor Values");
  mpuDataCharacteristics.addDescriptor(&mpuDataDescriptor);

  // Start the service
  mpuService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {

/*    // Get new sensor events with the readings 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // std::format("{:.2f}", a.acceleration.z);
    std::string accx = format(a.acceleration.x);
    std::string accy = format(a.acceleration.y);
    std::string accz = format(a.acceleration.z);
    std::string rotx = format(g.gyro.x);
    std::string roty = format(g.gyro.y);
    std::string rotz = format(g.gyro.z);
    std::string tempstr = format(temp.temperature);
    std::string message = "{ acc: { x:" + accx + ", y:" + accy + ", z:" + accz + " }, rot: { x:" + rotx + ", y:" + accy + ", z:" + accz + " }, temp:" + tempstr + " }";
    Serial.println(message.c_str());
    SerialBT.println(message.c_str());
    delay(500); */

  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      Serial.println("Sending");
      // Read values
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  
      
      //Notify humidity reading from MPU
      static char accelX[6];
      dtostrf(a.acceleration.x, 6, 2, accelX);
      //Set humidity Characteristic value and notify connected client
      mpuDataCharacteristics.setValue(accelX);
      mpuDataCharacteristics.notify();   
      Serial.print(" - Values: ");
      Serial.print(a.acceleration.x);
      Serial.println(" %");
      
      lastTime = millis();
    }
  }
}