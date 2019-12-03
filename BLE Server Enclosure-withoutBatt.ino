/*Program to use GATT service on ESP32 to send Battery Level
 * ESP32 works as server - Mobile as client
 * Program by: B.Aswinth Raj
 * Dated on: 13-10-2018
 * Website: www.circuitdigest.com
 * 
 * DHT code from https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-web-server-arduino-ide/
 * 
 * Changelog:
 * 
 * Version
 * 1.0.0        Initial Version
 * 
 */
 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> //Library to use BLE as server
#include <BLE2902.h> 
#include <DHT.h>

#define BLEName "Enclosure1" 

bool _BLEClientConnected = false;

#define DHTPIN 23     // Digital pin connected to the DHT sensor
#define LEDPIN 22     // LED pin 
#define maxBLESize 5  // Max size we expect the temp/humidity to be.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

 #ifdef __cplusplus
  extern "C" {
 #endif
 
  uint8_t temprature_sens_read();
 
#ifdef __cplusplus
}
#endif

// Set ledLight initial value and setup internal cpu temp sensor
uint8_t ledLight = 0x00;
uint8_t temprature_sens_read();

// CPU Temp
#define CPUTempService BLEUUID((uint16_t)0x1809) 
BLECharacteristic CPUTempCharacteristic(BLEUUID((uint16_t)0x2A6E), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor CPUTempDescriptor(BLEUUID((uint16_t)0x2901));

// Environment Sensor Service
#define EnvSensorService BLEUUID((uint16_t)0x181A)

// DHT Temp
BLECharacteristic DHTTempCharacteristic(BLEUUID((uint16_t)0x2A6E), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor DHTTempDescriptor(BLEUUID((uint16_t)0x2901));

// DHT Humidity
BLECharacteristic DHTHumidCharacteristic(BLEUUID((uint16_t)0x2A6F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor DHTHumidDescriptor(BLEUUID((uint16_t)0x2901));

// LED Lights
#define LEDLightService BLEUUID((uint16_t)0x1818) 
BLECharacteristic LEDLightCharacteristic(BLEUUID((uint16_t)0x2A66), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor LEDLightDescriptor(BLEUUID((uint16_t)0x2901));

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

class LedCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++)
        {
          if (rxValue[i] > 0) {
            digitalWrite(LEDPIN, HIGH);
          }
          else {
            digitalWrite(LEDPIN, LOW);
          }
        }
      }
    }
};

float readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return -9999;
  }
  else {
    return t;
  }
}

float readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return -9999;
  }
  else {
    return h;
  }
}

void InitBLE() {
  BLEDevice::init(BLEName);
  
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService *pCPUTemp = pServer->createService(CPUTempService);
  BLEService *pEnvSensor = pServer->createService(EnvSensorService);
  BLEService *pLEDLight = pServer->createService(LEDLightService);

  // Setup CPU Internal Temp
  pCPUTemp->addCharacteristic(&CPUTempCharacteristic);
  CPUTempDescriptor.setValue("Temperature in Celsius");
  CPUTempCharacteristic.addDescriptor(&CPUTempDescriptor);
  CPUTempCharacteristic.addDescriptor(new BLE2902());

  // Setup DHT Temp
  pEnvSensor->addCharacteristic(&DHTTempCharacteristic);
  DHTTempDescriptor.setValue("Temperature in Celsius");
  DHTTempCharacteristic.addDescriptor(&DHTTempDescriptor);
  DHTTempCharacteristic.addDescriptor(new BLE2902());

  // Setup DHT Humid
  pEnvSensor->addCharacteristic(&DHTHumidCharacteristic);
  DHTHumidDescriptor.setValue("Percentage 0 - 100");
  DHTHumidCharacteristic.addDescriptor(&DHTHumidDescriptor);
  DHTHumidCharacteristic.addDescriptor(new BLE2902());

  // Setup LED Light
  pLEDLight->addCharacteristic(&LEDLightCharacteristic);
  LEDLightDescriptor.setValue("State on or off");
  LEDLightCharacteristic.addDescriptor(&LEDLightDescriptor);
  BLE2902 *desc = new BLE2902();
  desc->setNotifications(true);
  LEDLightCharacteristic.addDescriptor(desc);
  LEDLightCharacteristic.setCallbacks(new LedCallback());
  
  // Advertise Services
  pServer->getAdvertising()->addServiceUUID(CPUTempService);
  pServer->getAdvertising()->addServiceUUID(EnvSensorService);
  pServer->getAdvertising()->addServiceUUID(LEDLightService);

  pCPUTemp->start();
  pEnvSensor->start();
  pLEDLight->start();
  
  // Start advertising
  pServer->getAdvertising()->start();
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  InitBLE();

  LEDLightCharacteristic.setValue(&ledLight, 1);
  LEDLightCharacteristic.notify();
}

void loop() {
  float cpuTemp = ((temprature_sens_read() - 32) / 1.8);
  char cpuTempBLE[maxBLESize];
  dtostrf(cpuTemp, 1, 2, cpuTempBLE); // float_val, min_width, digits_after_decimal, char_buffer
  CPUTempCharacteristic.setValue((unsigned char*)cpuTempBLE, sizeof(cpuTempBLE));
  CPUTempCharacteristic.notify();
  
  float temp = readDHTTemperature();
  char tempBLE[maxBLESize]; 
  dtostrf(temp, 2, 3, tempBLE);
  DHTTempCharacteristic.setValue((unsigned char*)tempBLE, sizeof(tempBLE));  
  DHTTempCharacteristic.notify();
  
  uint8_t humid = readDHTHumidity();
  char humidBLE[maxBLESize];
  dtostrf(humid, 2, 3, humidBLE);
  DHTHumidCharacteristic.setValue((unsigned char*)humidBLE, sizeof(humidBLE));
  DHTHumidCharacteristic.notify();
  delay(1000);
}  
