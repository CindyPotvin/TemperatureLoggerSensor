/*
* Weather sensor module for the temperature logger with a ESP8266 (ESP-12F) 
* 
* ESP8266
* i2c:
* SDA = GPIO2
* SCL = GPIO14
* 
* VCC = 3,3V
* GND = GND
* GPIO00 (GND to program, VCC to run program)

Requires:
Adafruit Unified sensor
Adafruit BMP280
ESP8266 board
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>

#include <Adafruit_BMP280.h>

ESP8266WiFiMulti WiFiMulti;

const char* SSID     = "";
const char* PASSWORD = "";
const char* SERVER_ADDRESS = "";

Adafruit_BMP280 WeatherSensor;
// Default i2c address set in the board for the BMP280 sensor).
const int BMP_I2C_ADDRESS = 0x76;

void setup () {
   Serial.begin(115200);

   // Wifi connection setup
   Serial.println();
   Serial.println("Connecting");
   WiFi.mode(WIFI_STA);
   // Parameters are is only needed if wifi password changes or new chip; values are not kept with 
   // the sketch and stay the same even after reprogramming.
   WiFi.begin();
   //WiFi.begin(SSID, PASSWORD);
   while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Wifi status ");
      Serial.println(WiFi.status());
      delay(2000);
   }
   Serial.print("Connected");
   Serial.println(WiFi.localIP());
  // The ESP doesn’t have a hardware TWI (Two Wire Interface), but it is implemented in software. 
  // This means that you can use pretty much any two digital pins.
  Wire.begin(2, 14);

  // Temperature and humidity sensor setup
  if (!WeatherSensor.begin(BMP_I2C_ADDRESS)) {
    Serial.println("Could not find the BMP280 sensor, check wiring!");
  }

  // Leave time for everything to get setup before starting the control loop
  delay(10000);
  sendCurrentWeather();
   // Deep sleep (only real-time clock active), will wake up and reboot after 
   // the delay to log a new temperature.
   ESP.deepSleep(5*60e6);//5 minutes
}
 
void loop () {
   //Nothing, the deep sleep will restart everything anyway.
}

String getTemperaturePressure(Adafruit_BMP280* sensor) {
   float temperature = sensor->readTemperature();
   float pressure = sensor->readPressure();
    
   String TemperaturePressureData = "{ temperature: \"";
          TemperaturePressureData += String(temperature, 1);
          TemperaturePressureData += "\", pressure: \"";
          TemperaturePressureData += String(pressure, 1);
          TemperaturePressureData += "\" }";
   Serial.println(TemperaturePressureData);

   return (TemperaturePressureData);
}
  
/* Sends the current weather to the temperature logger base station. */
void sendCurrentWeather() {
   // wait for WiFi connection
   int statusWifi = WiFiMulti.run();
   Serial.print("[HTTP] wifi status...");
   Serial.println(statusWifi);
    
   if (statusWifi == WL_CONNECTED) {
      HTTPClient http;
      Serial.print("[HTTP] Begins...\n");
      bool hasStarted = http.begin(SERVER_ADDRESS); //HTTP ipv4 ip adress
      if (hasStarted)
         Serial.print("[HTTP] Started...\n");
      else
         Serial.print("[HTTP] Not started...\n");

      Serial.print("[HTTP] POST...\n");
      // Send latest temperature information
      String TemperaturePressureData = getTemperaturePressure(&WeatherSensor);
      http.addHeader("Content-Type", "application/json");
      int httpCode = http.POST("{ weather: " + TemperaturePressureData + " }");
      http.writeToStream(&Serial);

      // httpCode will be negative when HTTP calls does not receive an answer
      if (httpCode > 0) {
         // HTTP header has been send and Server response header has been handled
         Serial.printf("[HTTP] POST... code: %d\n", httpCode);

         if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.println(payload);
            }
         else {
            Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
         }
      } 
      else {
         Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    }
 }
