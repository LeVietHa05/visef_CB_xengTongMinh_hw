


#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <SocketIoClient.h>
#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>
#include <MQUnifiedsensor.h>
// #include <BH1750.h>
#include <Wire.h>
//==============================================================================
#define dw digitalWrite
#define dr digitalRead

#define RS_TX 16
#define RS_RX 17
#define MQ_PIN 34
//---------------------------------------
#define SERVER "abc.com"
#define PORT 80
#define ssid "Wokwi-GUEST" // test
#define pass ""            // test

#define TOPIC_MEASURE "/esp/measure"
#define TOPIC_CONTROL "/esp/control"
#define TOPIC_OTHER "/esp/other"

/************************Hardware Related Macros************************************/
#define Board ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define Pin (MQ_PIN)     // IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define Type ("MQ-135")          // MQ3 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12)  // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ135CleanAir 3.6   // RS / R0 = 3.6 ppm
//==============================================================================
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
ModbusMaster node;
SocketIOclient socketIO;
Adafruit_AHTX0 aht;
// BH1750 lightMeter;
//==============================================================================
float ph = -1.0, soilMoisture = -1.0, soilTemp = -1.0, EC = -1.0, n = -1.0, p = -1.0, k = -1.0;
float co2 = -1, so2 = -1, no2 = -1;
float lux = -1;
bool isRS485On = false;
bool isWifiConnected = false;
bool isSocketIOConnected = false;
unsigned long lastCheckSensor = 0;
sensors_event_t humidity, temp;
//==============================================================================
// Function to read various sensor data via RS485 Modbus
void readRS485()
{
  uint8_t result;

  // Read pH value
  result = node.readHoldingRegisters(0x0006, 2);
  if (result == node.ku8MBSuccess)
  {
    ph = (float)node.receive() / 100.0;
    Serial.print("pH: ");
    Serial.print(ph);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading pH - Code: ");
    Serial.println(result);
    ph = -1.0; // Set to invalid value on error
  }

  // Read Electrical Conductivity (EC)
  result = node.readHoldingRegisters(0x0015, 2);
  if (result == node.ku8MBSuccess)
  {
    EC = (float)node.receive();
    Serial.print("EC: ");
    Serial.print(EC);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading EC - Code: ");
    Serial.println(result);
    EC = -1.0; // Set to invalid value on error
  }

  // Read Nitrogen (N)
  result = node.readHoldingRegisters(0x001e, 2);
  if (result == node.ku8MBSuccess)
  {
    n = (float)node.receive();
    Serial.print("Nitrogen: ");
    Serial.print(n);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading Nitrogen - Code: ");
    Serial.println(result);
    n = -1.0; // Set to invalid value on error
  }

  // Read Phosphorus (P)
  result = node.readHoldingRegisters(0x001f, 2);
  if (result == node.ku8MBSuccess)
  {
    p = (float)node.receive();
    Serial.print("Phosphorus: ");
    Serial.print(p);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading Phosphorus - Code: ");
    Serial.println(result);
    p = -1.0; // Set to invalid value on error
  }

  // Read Potassium (K)
  result = node.readHoldingRegisters(0x0020, 2);
  if (result == node.ku8MBSuccess)
  {
    k = (float)node.receive();
    Serial.print("Potassium: ");
    Serial.print(k);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading Potassium - Code: ");
    Serial.println(result);
    k = -1.0; // Set to invalid value on error
  }

  // Read Soil Moisture
  result = node.readHoldingRegisters(0x0012, 2);
  if (result == node.ku8MBSuccess)
  {
    soilMoisture = (float)node.receive() / 10.0;
    Serial.print("Soil Moisture: ");
    Serial.print(soilMoisture);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading Soil Moisture - Code: ");
    Serial.println(result);
    soilMoisture = -1.0; // Set to invalid value on error
  }

  // Read Soil Temperature
  result = node.readHoldingRegisters(0x0013, 2);
  if (result == node.ku8MBSuccess)
  {
    soilTemp = (float)node.receive() / 10.0;
    Serial.print("Soil Temperature: ");
    Serial.print(soilTemp);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading Soil Temperature - Code: ");
    Serial.println(result);
    soilTemp = -1.0; // Set to invalid value on error
  }

  // Add a newline for better serial monitor readability
  Serial.println();
}

//==============================================================================
// Optional: Add a function to check RS485 connection
bool checkRS485Connection()
{
  uint8_t result = node.readHoldingRegisters(0x0006, 2);
  return (result == node.ku8MBSuccess);
}
//==============================================================================
// read aht sensor and save to humidity and temp
void readAHT()
{
  aht.getEvent(&humidity, &temp);
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" *C");
}
//==============================================================================
// calibMQ135
void calibMQ135()
{
  Serial.print("Calibrating MQ135 please wait.");

  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  }
}
//==============================================================================
// void readLight()()
// {
//   lux = lightMeter.readLightLevel();
//   lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
// }
//==============================================================================
void readMQ()
{
  MQ135.setA(110.47);
  MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  co2 = MQ135.readSensor();
  MQ135.setA(605.18);
  MQ135.setB(-3.937); // Configure the equation to calculate so2 concentration value
  so2 = MQ135.readSensor();
  MQ135.setA(102.2);
  MQ135.setB(-2.473); // Configure the equation to calculate no2 concentration value
  no2 = MQ135.readSensor();
}
//==============================================================================
// send data to server
void sendDataToServer(int type, int buttonType, String message)
{
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  if (type == 1)
  {
    array.add(TOPIC_MEASURE);
    JsonObject data = array.createNestedObject();
    data["temp"] = String(temp.temperature, 2);
    data["humi"] = String(humidity.relative_humidity, 2);
    data["ph"] = String(ph, 2);
    data["soilMoisture"] = String(soilMoisture, 2);
    data["soilTemp"] = String(soilTemp, 2);
    data["n"] = String(n, 2);
    data["p"] = String(p, 2);
    data["k"] = String(k, 2);
    data["ec"] = String(EC, 2);
    // data["lux"] = String(lux, 2);
    data["co2"] = String(co2, 2);
    data["so2"] = String(so2, 2);
    data["no2"] = String(no2, 2);
  }
  else if (type == 2)
  {
    array.add(TOPIC_CONTROL);
    JsonObject data = array.createNestedObject();
    data["deviceID"] = "esp32";
    data["button"] = buttonType;
  }
  else
  {
    array.add(TOPIC_OTHER);
    JsonObject data = array.createNestedObject();
    data["deviceID"] = "esp32";
    data["message"] = message;
  }
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
}
//==============================================================================
#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case sIOtype_DISCONNECT:
  {
    isSocketIOConnected = false;
    USE_SERIAL.printf("[IOc] Disconnected!\n");
    break;
  }
  case sIOtype_CONNECT:
  {
    USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
    isSocketIOConnected = true;
    // join default namespace (no auto join in Socket.IO V3)
    socketIO.send(sIOtype_CONNECT, "/");
    break;
  }
  case sIOtype_EVENT:
  {
    String temp = String((char *)payload);
    if (temp.indexOf(TOPIC_CONTROL) != -1)
    {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, temp);
      JsonObject data = doc["data"];
      int button = data["button"];
    }
    else if (temp.indexOf(TOPIC_OTHER) != -1)
    {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, temp);
      JsonObject data = doc["data"];
      String message = data["message"];
      Serial.println(message);
    }
  }
  break;
  case sIOtype_ACK:
    USE_SERIAL.printf("[IOc] get ack: %u\n", length);
    break;
  case sIOtype_ERROR:
    USE_SERIAL.printf("[IOc] get error: %u\n", length);
    break;
  case sIOtype_BINARY_EVENT:
    USE_SERIAL.printf("[IOc] get binary: %u\n", length);
    break;
  case sIOtype_BINARY_ACK:
    USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
    break;
  }
}
//==============================================================================
void setup()
{
  Serial.begin(115200);
  // Initialize the node
  Serial2.begin(9600, SERIAL_8O1, RS_RX, RS_TX);
  node.begin(1, Serial2); // Modbus slave ID 1, use Serial2
  isRS485On = checkRS485Connection();
  isRS485On ? Serial.println("RS485 Connection Successful!") : Serial.println("RS485 Connection Failed!");

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862); // Configure the equation to to calculate Co2 concentration                  

  MQ135.init();

  calibMQ135();

  Wire.begin();

  aht.begin();

  // lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  WiFiManager wifiManager;                 // Create a WiFiManager instance
  wifiManager.autoConnect("ESP32_Config"); // Creates a WiFi access point with the specified name

  // server address, port and URL
  socketIO.begin(SERVER, PORT, "/socket.io/?EIO=4");

  // event handler
  socketIO.onEvent(socketIOEvent);
}

void loop()
{
  socketIO.loop();
  MQ135.update();
  if (millis() - lastCheckSensor > 3000)
  {
    if (checkRS485Connection())
    {
      readRS485(); 
    }
    readAHT();
    readMQ();
    sendDataToServer(1, 0, "");
    lastCheckSensor = millis();
  }
}

//==============================================================================
