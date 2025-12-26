// Install ALL of the following libraries via Arduino Library Manager:
//   Adafruit Unified Sensor by Adafruit
//   Adafruit BMP280 Library by Adafruit
//   Adafruit BME280 Library by Adafruit
//   Adafruit AHTX0 Library by Adafruit
// 
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

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

float temperature = 0.0, humidity = -1.0, vcc = 0.0, pressure = -1.0;
const int sendinterval = 5 * 60; //in seconds

TwoWire twi;
Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
enum sensor_type_enum { SENSOR_UNKNOWN, SENSOR_BME280_0x76, SENSOR_BMP280_0x76, SENSOR_BME280_0x77, SENSOR_BMP280_0x77, SENSOR_AHT10 };
sensor_type_enum sensor_type = SENSOR_UNKNOWN;
const char* sensor_type_names[] = { "SENSOR_UNKNOWN", "SENSOR_BME280_0x76", "SENSOR_BMP280_0x76", "SENSOR_BME280_0x77", "SENSOR_BMP280_0x77", "SENSOR_AHT10" };

sensor_type_enum detect_sensor_type() {
  twi = TwoWire();
  twi.begin(D3, D4); //sda, scl
  // Go through all possible sensors and see which one responds
  bme = Adafruit_BME280();
  if (bme.begin(0x76, &twi)) { return SENSOR_BME280_0x76;}
  if (bme.begin(0x77, &twi)) { return SENSOR_BME280_0x77;}
  bmp = Adafruit_BMP280(&twi);
  if (bmp.begin(0x76)) { return SENSOR_BMP280_0x76;}
  if (bmp.begin(0x77)) { return SENSOR_BMP280_0x77;}
  if (aht.begin(&twi)) { return SENSOR_AHT10;}
  return SENSOR_UNKNOWN;
}

void readTempHumidityPressure() {
  switch (sensor_type) {
    case SENSOR_BME280_0x76:
    case SENSOR_BME280_0x77:
      temperature = bme.readTemperature();
      pressure = bme.readPressure();
      humidity = bme.readHumidity();
      break;
    case SENSOR_BMP280_0x76:
    case SENSOR_BMP280_0x77:
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure();
      humidity = -1;
      break;
    case SENSOR_AHT10: {
      sensors_event_t temp, hum;
      aht.getEvent(&hum, &temp);
      temperature = temp.temperature;
      humidity = hum.relative_humidity;
      pressure = -1;
      break;
    }
    default: // minus one
      temperature = -1.0;
      pressure = -1.0;
      humidity = -1.0;
  }
}


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

  Serial.begin(115200);
  Serial.println("Welcome to the BMP Sensor");

  uint32_t detect_time = millis();
  sensor_type = detect_sensor_type();
  Serial.printf("Detected sensor type: %s in %lu ms\n", sensor_type_names[(int)sensor_type], millis() - detect_time);

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
  readTempHumidityPressure();
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
