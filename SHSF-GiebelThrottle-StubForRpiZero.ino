/*********************************************************************************
  Sketch Name: SHSF_GiebelThrottle_StubForRpiZero_2
  Written By: Kevin R. Smith
  Created: 2025-Sep-27

  This sketch will simulate a single command from Giebel Throttle through WiFi
  for testing with a Rsapberry Pi Zero (gift from Dylan & Megan).
  Using Arduino R4 WiFi as Giebel Throttle.
  two-way communication so your Arduino R4 WiFi can send a command, and your Raspberry Pi Zero 2 W responds with a confirmation.  
  Arduino sends a message directly to the Pi running a small server script.
  - Raspberry Pi MQTT Broker to Nano
  - Arduino R4 WiFi MQTT Client
**********************************************************************************
*/
//-----------------Calling libraries needed to run-----------//
#include <Adafruit_SSD1306.h> // for OLED display
#include "Arduino_LED_Matrix.h" // Include the LED_Matrix library
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <Tweakly.h> // for non-blocking (no use of delay()) schedule of tasks and input processing.
#include <EEPROM.h>
//
//--------------------- I2C Addresses ----------------------//
#define OLED_I2C_ADDR 0x3C // Address 0x3C for 128x32
//
//---------- For SSD1306 display connected to I2C ----------//
#define OLED_WIDTH 128 // OLED display width, in pixels
#define OLED_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
//
//-------------------Object Instantiation-------------------//
ArduinoLEDMatrix matrix;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
//
//------------------------Tweakly---------------------------//
TickTimer timerCheckConnection;
//
//---------------------State Machine -----------------------//
bool blnCheckConnection = true;
enum ConnectionState { CONNECT_WIFI, AWAIT_WIFI, CONNECT_MQTT, RUNNING };
ConnectionState currentState = CONNECT_WIFI;
//
//--------------------GLOBAL VARIABLES----------------------//
// Set this to true if a OLED is connected.
bool displayConnected = false;
//
// Define a structure to hold EEPROM data.
struct StoreStruct {
  char ssid[32];
  char pass[64];
  int checkValue; // Used to verify data in EEPROM is valid.
};
//Variable to store data read from EEPROM.
StoreStruct storage;
//
// A unique ID to check if we have valid data in EEPROM.
const int CONFIG_ID = 1234;
//
// The IP Address of Raspberry Pi Zero 2 W
const char broker[] = "192.168.1.117"; 
int        port     = 1883;
//
// Topics
const char topic_cmd[]  = "shsf/giebel_throttle/commands";
const char topic_res[]  = "shsf/giebel_throttle/responses";
const char topic_sts[]  = "shsf/giebel_throttle/status";
const char topic_hrt[]  = "shsf/heartbeat";
//
// Watchdog
unsigned long lastHeartbeatMillis = 0;
const unsigned long watchdogTimeout = 25000; // 25 seconds (allows for 2 missed heartbeats)
bool systemOnline = false;
//
// Frame for LED matrix
const uint32_t frown[] = {
  0x19819,
  0x80000001,
  0xf8108000
};
const uint32_t danger[] = {
	0x400a015,
	0x1502082,
	0x484047fc
};
const uint32_t chip[] = {
	0x1503f811,
	0x3181103,
	0xf8150000
};
const uint32_t happy[] = {
	0x19819,
	0x80000001,
	0x81f8000
};
//
unsigned long lastButtonH_Press = 0;
unsigned long lastButton7_Press = 0;
unsigned long lastButton5_Press = 0;
unsigned long buttonPressInterval = 1500;
//
//-------------------- Pin assignments ---------------------//
// LED_BUILTIN = 13
const uint8_t LED_12 = 12;
const uint8_t BUTTON_H = 8;
const uint8_t BUTTON_7 = 7;
const uint8_t BUTTON_5 = 5;
//----------------------------------------------------------//
//                      Setup code
//----------------------------------------------------------//
void setup() {
  int eeAddress = 0; //EEPROM address to start reading from
  EEPROM.get(eeAddress, storage);
  if (storage.checkValue == CONFIG_ID) {
    Serial.println("Credentials found in EEPROM.");
  } else {
    Serial.println("No credentials found in EEPROM!");
  }
  //
  unsigned long timeout = 5000; // Serial() timeout in milliseconds.
  unsigned long startMillis = millis(); // Record the start time for Serial() timeout.
  //
  // Initialize Serial().
  Serial.begin(115200);
  // This code is from SHSF MyMeter.
  while (!Serial) {
    // Check if the timeout has been reached
    if (millis() - startMillis > timeout) {
        break; // Exit the loop after timeout
    }
    delay(10);
  }
  //
  // Set the message receive callback
  mqttClient.onMessage(onMqttMessage);
  //
  // Initialize the LED matrix
  matrix.begin();
  matrix.loadFrame(danger);
  //
  Serial.println(F("SH&SF - Giebel Throttle Stub #3 for Raspberry Pi"));
  Serial.println(F("Starting setup."));
  if (displayConnected) {
    // initialize OLED display
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR))
    {
      Serial.println(F("SSD1306 display allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
    Serial.println(F("OLED Display Module online."));
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("SH&SF Giebel Throttle");
    display.setCursor(0, 10);
    display.println("Stub #2 for Raspberry Pi");
    display.display();
  }
  //
  // Setup BUTTON_H and led
  pinMode(BUTTON_H, INPUT_PULLUP);
  pinMode(BUTTON_7, INPUT_PULLUP);
  pinMode(BUTTON_5, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_12, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_12, LOW);
  //
  // Initialize Tweakly.
  timerCheckConnection.attach(5000, []{ blnCheckConnection = true; });
  //
  Serial.print(F("Setup is complete.\n"));
}
//----------------------------------------------------------//
//                       Loop code
//----------------------------------------------------------//
void loop() {
  //Call Tweakly forever
  TweaklyRun();
  //Put your code here :-)
  //
  String strTemp = "";
  //
  if (blnCheckConnection) {
    blnCheckConnection = false;
    switch (currentState) {
      case CONNECT_WIFI:
        Serial.print("[WiFi] Initiating connection to: ");
        Serial.println(storage.ssid);
        WiFi.begin(storage.ssid, storage.pass);
        currentState = AWAIT_WIFI;
        break;
        //
      case AWAIT_WIFI:
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("[WiFi] Connected!");
          matrix.loadFrame(chip);
          printWifiStatus();
          currentState = CONNECT_MQTT;
        }
        //
      case CONNECT_MQTT:
        Serial.print("[MQTT] Attempting connection to broker: ");
        Serial.println(broker);
        if (mqttClient.connect(broker, port)) {
          Serial.println("[MQTT] Connected to broker.");
          matrix.loadFrame(happy);
          //
          // Subscribe to responses coming back from the Nano
          Serial.print("[MQTT] Subscribing to topic: ");
          Serial.println(topic_res);
          mqttClient.subscribe(topic_res);
          //
          // Subscribe to heartbeat topic
          mqttClient.subscribe(topic_hrt);
          //
          currentState = RUNNING;
        } else {
          Serial.print("[MQTT] Failed, error code: ");
          Serial.println(mqttClient.connectError());
          currentState = CONNECT_WIFI; // Drop back to check WiFi first
        }
        break;
        //
      case RUNNING:
        // This is the active state
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("[WiFi] Connection lost!");
          currentState = CONNECT_WIFI;
        } else if (!mqttClient.connected()) {
          Serial.println("[MQTT] Connection lost!");
          currentState = CONNECT_MQTT;
        } else {
          mqttClient.poll(); // Process incoming messages
          //
          // Check if the Pi has "gone dark"
          if (millis() - lastHeartbeatMillis > watchdogTimeout) {
            if (systemOnline) {
              Serial.println("[WATCHDOG ALERT] SHSF Hub Offline !!!");
              matrix.loadFrame(frown);
              systemOnline = false;
              // ACTION: Turn off critical components here if needed
            }
          }
        }
        break;
    }
  }
  //
  if (!digitalRead(BUTTON_H) && millis() - lastButtonH_Press >= buttonPressInterval) {
    digitalWrite(LED_BUILTIN, HIGH);
    //
    if (displayConnected) {
      display.clearDisplay();
      display.display();
      display.setCursor(0, 0);
    }
    //
    strTemp = "h";
    sendMqttCommand(strTemp);
    //
    lastButtonH_Press = millis();
    //
    if (displayConnected) {
      display.display();
    }
  }
  //
  if (!digitalRead(BUTTON_7) && millis() - lastButton7_Press >= buttonPressInterval) {
    digitalWrite(LED_BUILTIN, HIGH);
    //
    if (displayConnected) {
      display.clearDisplay();
      display.display();
      display.setCursor(0, 0);
    }
    //
    strTemp = "ba o";
    sendMqttCommand(strTemp);
    //
    lastButton7_Press = millis();
    //
    if (displayConnected) {
      display.display();
    }
  }
  //
  if (!digitalRead(BUTTON_5) && millis() - lastButton5_Press >= buttonPressInterval) {
    digitalWrite(LED_BUILTIN, HIGH);
    //
    if (displayConnected) {
      display.clearDisplay();
      display.display();
      display.setCursor(0, 0);
    }
    //
    strTemp = "vn";
    sendMqttCommand(strTemp);
    //
    lastButton5_Press = millis();
    //
    if (displayConnected) {
      display.display();
    }
  }
  //
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_12, LOW);
}
//----------------------------------------------------------//
//                  End of Loop code
//----------------------------------------------------------//
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
//
void sendMqttCommand(String cmd) {
  Serial.print("[MQTT] Sending command to topic '");
  Serial.print(topic_cmd);
  Serial.print("': ");
  Serial.println(cmd);
  //
  mqttClient.beginMessage(topic_cmd);
  mqttClient.print(cmd);
  mqttClient.endMessage();
}
//
void onMqttMessage(int messageSize) { // We received a message
  // 1. Grab the topic immediately!
  String topic = mqttClient.messageTopic();
  //
  // 2. Now handle the payload
  String content = "";
  while (mqttClient.available()) {
    content += (char)mqttClient.read();
  }
  //
  if (topic == topic_hrt) {
    lastHeartbeatMillis = millis();
    if (!systemOnline) {
      Serial.println("[WATCHDOG ALERT] SHSF Hub Online");
      matrix.loadFrame(happy);
      systemOnline = true;
    }
  } 
  else {
    Serial.print("[MQTT] Message received on topic '");
    Serial.print(topic);
    Serial.print("': ");
    Serial.println(content);
  }
}