#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "NB_BC95_G.h"


#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>


HardwareSerial modbus(2);
HardwareSerial myserial(1);


#define SERIAL1_RXPIN 27
#define SERIAL1_TXPIN 14

char buffer_receive[1000];
unsigned int count_buffer = 0;
String data_str = "";
int read_timeout = 0;

NB_BC95_G AISnb;

BluetoothSerial SerialBT;

// thingcontrol.io setup
String deviceToken = "";
String serverIP = "147.50.151.130";
String serverPort = "19956";
String json = "";

ModbusMaster node;


#define trigWDTPin    32
#define ledHeartPIN   0




struct pm2510
{
  String SO2;
  String NO2;
  String CO;
  String O3;
  String temp;
  String hum;
  String PM2_5;
  String PM10;
};
pm2510 sensor ;


unsigned long ms;
uint16_t dataWeather[8];

//WiFi&OTA 参数
String HOSTNAME = "CPN-";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

void setup()
{
  HeartBeat();
  Serial.begin(115200);
  modbus.begin(9600, SERIAL_8N1, 16, 17);


  HeartBeat();
  delay(2000);

  HeartBeat();
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.delayAfterCommand = 1000;
  AISnb.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial.println("Waiting for AIS NB test status!");
  while (!AISnb.testCommand().status)
  {
    Serial.print('.');
    SerialBT.print(',');
  }
  Serial.println("AIS NB OK!");
  SerialBT.println("AIS NB OK!");

  AISnb.setupDevice(serverPort);
  nb_resp_t res_DeviceIP = AISnb.getDeviceIP();
  nb_resp_t res_testPing = AISnb.testPing(serverIP);
  nb_resp_t res_nccid = AISnb.getNCCID();
  deviceToken = res_nccid.data;
  SerialBT.print("NCCID:");
  SerialBT.println(deviceToken);
  Serial.print("NCCID:");
  Serial.println(deviceToken);
  HeartBeat();


  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  HeartBeat();
  setupWIFI();
  HeartBeat();
  setupOTA();
  HeartBeat();

}

void loop()
{

  ArduinoOTA.handle();
  ms = millis();
  if (ms % 60000 == 0)
  {
    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    SerialBT.println("Attach WiFi for OTA"); SerialBT.println(WiFi.RSSI() );
    setupWIFI();
    HeartBeat();
    setupOTA();
  }
  if (ms % 120000 == 0)
  {
    t2CallsendViaNBIOT();
    nb_resp_t res_send = AISnb.sendUDPMessage(1, serverIP, serverPort, json.length(), json, MODE_STRING_HEX);
    if (!res_send.status)
    {
      AISnb.createUDPSocket(serverPort);
    }
    String getResponse = AISnb.getSocketResponse();
    if (getResponse.length() > 0) {
      Serial.print("UDP response: ");
      Serial.println(getResponse);
      SerialBT.print("UDP response: ");
      SerialBT.println(getResponse);
    }
  }

}




char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}


/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  //No authentication by default
  ArduinoOTA.setPassword(password);
  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    HeartBeat();
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);
    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    ms = millis();
    if (ms % 10000 == 0)
    {
      HeartBeat();
    }
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart();
  });
  ArduinoOTA.begin();
}

void setupWIFI()
{
  Serial.println("Connecting...");
  Serial.println(String(ssid));
  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);
  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);
  // Return to high-Z
  pinMode(trigWDTPin, INPUT);
  Serial.println("Heartbeat");
  SerialBT.println("Heartbeat");
}
\
void t2CallsendViaNBIOT ()
{
  readsensor();
  nb_signal_t res_signal = AISnb.getSignal();
  SerialBT.print("RSSI:");
  SerialBT.println(res_signal.rssi);
  Serial.print("RSSI:");
  Serial.println(res_signal.rssi);
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"so2\":");
  json.concat(sensor.SO2);
  json.concat(",\"no2\":");
  json.concat(sensor.NO2);
  json.concat(",\"co\":");
  json.concat(sensor.CO);
  json.concat(",\"o3\":");
  json.concat(sensor.O3);
  json.concat(",\"pm2.5\":");
  json.concat(sensor.PM2_5);
  json.concat(",\"pm10\":");
  json.concat(sensor.PM10);
  json.concat(",\"temp\":");
  json.concat(sensor.temp);
  json.concat(",\"hum\":");
  json.concat(sensor.hum);
  json.concat(",\"RSSI\":");
  json.concat(res_signal.rssi);

  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);


}

void readWeather(uint16_t  REG)
{
  static uint32_t i;
  uint16_t j, result;
  uint32_t value = 0;
  float val = 0.0;
  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_PM25, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 8);
  Serial.print("result:"); Serial.print(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 8; j++)
    {
      dataWeather[j] = node.getResponseBuffer(j);
      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(dataWeather[j]);
      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(dataWeather[j]);
    }
    sensor.SO2 = dataWeather[0];
    sensor.NO2 = dataWeather[1];
    sensor.CO = dataWeather[2];
    sensor.O3 = dataWeather[3];
    sensor.PM2_5 = dataWeather[4];
    sensor.PM10 = dataWeather[5];
    sensor.temp = dataWeather[6] / 100 - 40;
    sensor.hum = dataWeather[7] / 100;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    SerialBT.print("Connec modbus fail. REG >>> "); SerialBT.println(REG, HEX); // Debug
  }
}

void readsensor()
{
  readWeather(_SO2);
  Serial.print("SO2 : "); Serial.println( sensor.SO2);
  Serial.print("NO2 : "); Serial.println( sensor.NO2);
  Serial.print("CO : "); Serial.println( sensor.CO);
  Serial.print("O3 : "); Serial.println( sensor.O3);
  Serial.print("PM25 : "); Serial.println( sensor.PM2_5);
  Serial.print("PM10 : "); Serial.println( sensor.PM10);
  Serial.print("temp : "); Serial.println( sensor.temp);
  Serial.print("hum : "); Serial.println( sensor.hum);
  Serial.println("");
  SerialBT.print("SO2 : "); SerialBT.println( sensor.SO2);
  SerialBT.print("NO2 : "); SerialBT.println( sensor.NO2);
  SerialBT.print("CO : "); SerialBT.println( sensor.CO);
  SerialBT.print("O3 : "); SerialBT.println( sensor.O3);
  SerialBT.print("PM25 : "); SerialBT.println( sensor.PM2_5);
  SerialBT.print("PM10 : "); SerialBT.println( sensor.PM10);
  SerialBT.print("temp : "); SerialBT.println( sensor.temp);
  SerialBT.print("hum : "); SerialBT.println( sensor.hum);
  SerialBT.println("");
}


float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}

String decToHex(int decValue) {
  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {
  unsigned int decValue = 0;
  int nextInt;
  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}

int getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  Serial.print("hex:");  Serial.println(hex2);
  Serial.print("dec:");  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  return hexToDec(hex2);
}

/****************************************************
   [通用函数]ESP32 WiFi Kit 32事件处理
*/
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;
    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;
    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;
    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;
    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;
    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
