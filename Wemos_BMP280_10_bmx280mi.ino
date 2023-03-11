//https://robotzero.one/esp8266-and-bme280-temp-pressure-and-humidity-sensor-spi/
//https://randomnerdtutorials.com/esp8266-bme280-arduino-ide/
//http://www.esp8266learning.com/esp8266-bmp280-sensor-example.php
// examples of 6 pin version
// INSTALL NEW BMx280 library from arduino library manager!
 
#include <Wire.h>
#include <BMx280I2C.h> // INSTALL THIS LIBRARY
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

//---------------------------------------------
#define BATTERY 0
#define myid 19
#if (myid==7 || myid==8 || myid==9 || myid==10 || myid==11)
  const char * ssid = "dlinkpince"; // garazs, elelmiszertarolo. A kortefa
#else
  const char * ssid = "Tenda_067808";
#endif

#ifdef Xiaomi
  const char * ssid = "Xiaomi_DFF1";
  const char * pw = "12345678";
#endif

//#if (myid==9)
//const char * ssid = "Tenda_Kicsi";
//#else
//#endif

float temperature = 0.0, humidity = -1.0, vcc = 0.0, pressure = 0.0;
const int sendinterval = 5 * 60; //in seconds

#define I2C_ADDRESS 0x76
//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);

unsigned long timetillsleep = 0;
const float adcvmul = 0.0090 ;//magic constant, even though adc is pulled up with 520K, and pulled down with 100K
char packet_buff[256];
bool bme_detected = false;
char sbuf[256];

struct RRTC{ //512 bytes of RTC memory data that persists through deep sleep.
  uint32_t failures = 0;
} rtcData;

const char * pw = "wanandras net";
unsigned int udpport = 12345;
  IPAddress udpserverip = IPAddress(192, 168, 4, 250);

  IPAddress clientip(192, 168, 4, 35 + myid);
  IPAddress gateway(192, 168, 4, 250);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns1(192, 168, 4, 250);
  IPAddress dns2(192, 168, 4, 250);

void setup() {
//---------------------------------------------
  timetillsleep = millis();

  Wire.begin(D3, D4);

  Serial.begin(115200);
  Serial.println("Welcome to the BMP Sensor");

  if (!bmx280.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    bme_detected = false;
  } else {
    bme_detected = true;
    bmx280.resetToDefaults();
    //by default sensing is disabled and must be enabled by setting a non-zero
    //oversampling setting.
    //set an oversampling setting for pressure and temperature measurements. 
    bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
    bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  
    if (bmx280.isBME280()){
      Serial.println("sensor is a BME280");
  
      //if sensor is a BME280, set an oversampling setting for humidity measurements.
      bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
    }else{
      Serial.println("sensor is a BMP280");
    }
  }

  WiFi.config(clientip, gateway, subnet, dns1, dns2); // kellett neki ez a dns is! Fraszt.
  delay(400);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  
  sprintf(sbuf, "SSID = %s, Target IP = %s, Password = %s, MAC = %s", 
      ssid, udpserverip.toString().c_str(), pw, WiFi.macAddress().c_str());
  Serial.println(sbuf);
  
  int timeout = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    timeout += 100;
    Serial.print('.');
    if (timeout > 10000) {
      Serial.println("Cannot connect to wifi!");
      break;
    }
  }
  Serial.println();
}

//---------------------------------------------

void createpacketstr(char * packet, float temp, float p, float h, float v, int udpsendfails) {
  char tempstr[16];
  char pstr[16];
  char hstr[16];
  char vstr[16];
  dtostrf(temp, -7, 2, tempstr);
  dtostrf(p, -10, 2, pstr);
  dtostrf(h, -7, 2, hstr);
  dtostrf(v, -8, 3, vstr);
  sprintf(packet, "@BMP=%d\tFromIP=%s\tTemperature=%s\tPressure=%s\tHumidity=%s\tVoltage=%s\tRSSI=%d\tUDPSendFails=%d\tMAC=\"%s\"\n", myid, WiFi.localIP().toString().c_str(), tempstr, pstr, hstr, vstr, WiFi.RSSI(), udpsendfails, WiFi.macAddress().c_str());
//  sprintf(packet, "@BMP=%d FromIP=%s Temperature=%s Pressure=%s Humidity=%s Voltage=%s RSSI=%d UDPSendFails=%d MAC=\"%s\"\n", myid, WiFi.localIP().toString().c_str(), tempstr, pstr, hstr, vstr, WiFi.RSSI(), udpsendfails, WiFi.macAddress().c_str());
}
//---------------------------------------------
void senddataUDP(char * packet , IPAddress ip, unsigned int port) {
  return;
  WiFiUDP udp;
  udp.beginPacket(ip, port);
  udp.write(packet);
  if (udp.endPacket()) {
    Serial.println("UDP Send succesful");
  } else {
    Serial.println("UDP Send failed");
    increment_send_failure_rtc();
  }
  yield();
}
//---------------
void increment_send_failure_rtc() {
  ESP.rtcUserMemoryRead(0,  (uint32_t*) &rtcData, sizeof(rtcData));
  rtcData.failures = rtcData.failures + 1;
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
}
//---------------------------------------------
void senddataTCP(char * packet , IPAddress ip, unsigned int port) {
  WiFiClient client;
  Serial.println("Send TCP");
  if (client.connect(ip, port)) {
    Serial.print("TCP Connection established to ");
    client.print(packet);
    client.flush();
    client.stop();
  } else {
    Serial.print("TCP Failed to connect to ");
    //    increment_send_failure_rtc();
  }
  Serial.print(ip.toString().c_str());
  Serial.print(':');
  Serial.println(port);
  yield();
}

//---------------------------------------------
void loop() {
  if (bme_detected) {
    temperature = bmx280.readTemperature();
    pressure = bmx280.readPressure();
    if (bmx280.isBME280())
    {
      humidity = bmx280.getHumidity();
    }else{
      humidity = -1;
    }
  }
  analogRead(A0);
  float advc = analogRead(A0);
  vcc = advc * adcvmul ;
  Serial.setTimeout(5000);

  createpacketstr(packet_buff, temperature, pressure, humidity, vcc, rtcData.failures);
  Serial.print("Sending packet: ");
  Serial.print(packet_buff);
  // senddataUDP(packet_buff,udpserverip,udpport);
  senddataTCP(packet_buff, udpserverip, udpport);
  yield();
  Serial.print(millis() - timetillsleep);
  Serial.println("ms worked till sleep");
  delay(1);
  #if (BATTERY == 1)
    ESP.deepSleep(sendinterval * 1e6);  //Performs a reset (init) on wake, use RTC memory for persistence
  #else
    delay(sendinterval * 1000);
    Serial.println("Rebooting...");
    ESP.restart();
  #endif
  // short D0 to RST jumpers for reset executed on wake!
}
