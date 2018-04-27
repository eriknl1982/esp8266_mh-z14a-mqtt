/***************************************************************************
  Sample sketch for using a mh-z14a co2 with a ESP8266 sensor and sending the result over MQTT once a minute. 

  Written by Erik Lemcke, combined out of the following samples:

  https://www.letscontrolit.com/forum/viewtopic.php?f=2&t=1785&start=40, calibration sample by s3030150 
  https://www.home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/, hjome assistant mqqt by Paulus Schoutsen

  wiring:
  ESP --> mh-z14a
  4       rx 
  5       tx
  gnd     gnd
  
  Make sure the ESP is supplied with 3.3v, while the mh-z14a needs 5v
 ***************************************************************************/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define INTERVAL 5000
#define MH_Z19_RX 4 //RX pin
#define MH_Z19_TX 5 //TX pin

#define wifi_ssid "<YOUR WIFI SSID>"
#define wifi_password "<YOUR WIFI PASSWORD>"

#define mqtt_server "<YOUR MQTT SERVER IP>"
#define mqtt_user "<YOUR MQTT USERNAME>"
#define mqtt_password "<YOUR MQTT PASSWORD>"

#define co2_topic "MHZ14A/co2"

WiFiClient espClient;
PubSubClient client(espClient);

byte mhzResp[9];    // 9 bytes bytes response
byte mhzCmdReadPPM[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
byte mhzCmdCalibrateZero[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
byte mhzCmdABCEnable[9] = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6};
byte mhzCmdABCDisable[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86};
byte mhzCmdReset[9] = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};
byte mhzCmdMeasurementRange1000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
byte mhzCmdMeasurementRange2000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
byte mhzCmdMeasurementRange3000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
byte mhzCmdMeasurementRange5000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};

int shifts=0,co2ppm;

long previousMillis = 0;

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z19

byte checksum(byte response[9]){
  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += response[i];
  }
  crc = 255 - crc + 1;
  return crc;
}

void disableABC() {
 co2Serial.write(mhzCmdABCDisable, 9);
}

void enableABC() {
 co2Serial.write(mhzCmdABCEnable, 9);
}

void setRange5000() {
 co2Serial.write(mhzCmdMeasurementRange5000, 9);
}

void calibrateZero(){
 co2Serial.write(mhzCmdCalibrateZero, 9);
}

int readCO2() {
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9];
  co2Serial.write(cmd, 9);
  // The serial stream can get out of sync. The response starts with 0xff, try to resync.
  while (co2Serial.available() > 0 && (unsigned char)co2Serial.peek() != 0xFF) {
    co2Serial.read();
    shifts++;
  }

  memset(response, 0, 9);
  co2Serial.readBytes(response, 9);

  for (int i=0; i<9; i++) {
    Serial.print(" 0x");
    Serial.print(response[i], HEX);
  }
  Serial.println(" Response OK. Shifts="+String(shifts));

  if (response[1] != 0x86)
  {
    Serial.println(" Invalid response from co2 sensor!");
    return -1;
  }

  if (response[8] == checksum(response)) {
    int responseHigh = (int) response[2];
    int responseLow = (int) response[3];
    int ppm = (256 * responseHigh) + responseLow;
    return ppm;
  } else {
    Serial.println("CRC error!");
    return -1;
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA); //Make sure the ESP isn't exposed as a access point
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Startup");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  
  unsigned long previousMillis = millis();
  delay(500);
  Serial.println("Disabling ABC");
  disableABC();
  Serial.println("Setting range to 5000"); 
  setRange5000();
  Serial.println("Waiting half an hour before calibrating zero"); 
  delay(1800000);
  calibrateZero();
  Serial.println("Zero was calibrated");
}

long lastMsg = 0;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  //send a meaage every minute
  if (now - lastMsg > 60 * 1000) {
    lastMsg = now;
    unsigned long currentMillis = millis();
    if (abs(currentMillis - previousMillis) > INTERVAL)
    {
      previousMillis = currentMillis;
      Serial.print("Requesting CO2 concentration...");
      co2ppm=-999;
      co2ppm = readCO2();

      //If no proper co2 value is returned, try again
      while (co2ppm == -1){
        Serial.print("re-Requesting CO2 concentration...");
        co2ppm = readCO2();  
      }
      
      Serial.println("  PPM = " + String(co2ppm));
      client.publish(co2_topic, String(co2ppm).c_str(), true);
    }
  }
}