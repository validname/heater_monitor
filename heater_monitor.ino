#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
// for OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "WiFiSettings.h"
// contains next settings:
#define WIFI_SSID     CONFIG_WIFI_SSID
#define WIFI_PASSWORD	CONFIG_WIFI_PASSWORD
#define WIFI_OTA_PASSWORD CONFIG_OTA_PASSWORD

#define DEBUG
#define DEBUG_COMMON
#define DEBUG_COMMON_UDP
//#define DEBUG_WEB

#ifdef DEBUG
#ifdef DEBUG_COMMON_UDP
#include <WiFiUdp.h>
#define DEBUG_COMMON_UDP_BUFFER_LENGTH 256

IPAddress udpDebugMulticastAddress(192, 168, 0, 255);
WiFiUDP DebugUDP;
char DebugUDPBuffer[DEBUG_COMMON_UDP_BUFFER_LENGTH];
#define UDP_DEBUG_PORT 5555
#endif
#endif

const unsigned firmwareRevision = 17;

// updates some sensor every N milliseconds
const unsigned int sensorPollingInterval = 1000;
const unsigned int maxSensors = 5;
unsigned long previousSensorPollingTime = 0;
unsigned int currentSensor;
unsigned int oneSensorInterval;

/*
  DS18B20 settings
*/

OneWire oneWire(0); // at GPIO0 / D8 for Wemos D1 R1 (clone)

DallasTemperature DS18B20Sensors(&oneWire);

// indexes are: 0 is input (for heater), 1 is output and 2 for hot water
DeviceAddress DS18B20SensorAddressInput = { 0x28, 0x4F, 0x1B, 0x07, 0xD6, 0x01, 0x3C, 0xD4 };
DeviceAddress DS18B20SensorAddressOutput = { 0x28, 0xA5, 0x0D, 0x75, 0xD0, 0x01, 0x3C, 0xAC };
DeviceAddress DS18B20SensorAddressWater = { 0x28, 0xD2, 0x5D, 0x07, 0xD6, 0x01, 0x3C, 0x5A };

const float outputSpeedHeatingThreshold = 0.03;
const float outputSpeedCoolingThreshold = -0.02;
const float inputSpeedHeatingThreshold = 0.02;
const float waterSpeedHeatingThreshold = 0.02;
const float heatingCicleTimeCorrectionFraction = 0.1;	// from 0 to 1
const float forceHeatingIntervalFraction = 2; // from 1 to beyond

double inputTemperature, previuosInputTemperature, inputTemperatureSpeed;
double outputTemperature, previuosOutputTemperature, outputTemperatureSpeed;
double waterTemperature, previuosWaterTemperature, waterTemperatureSpeed;

unsigned long heatingStartTime, prevHeatingStartTime;
unsigned int heatingInterval, lastHeatingDelay, lastForcedHeatingDelay; // in seconds
unsigned int heaterState;
float minHeatingTemperature, maxHeatingTemperature;

/*
 LCD 1602 settings
*/

LiquidCrystal_I2C lcd(0x27,16,2);

/*
 DHT22 / AM2302 settings
*/

DHT dht22(2, DHT22); // GPIO2 / TX1/D9

float airTemperature;
float airHumidity;

/*
 Wi-Fi 
*/

// Set web server port number to 80
WiFiServer server(80);

IPAddress WiFILocalIP(192, 168, 0, 200);
IPAddress WiFIGateWay(192, 168, 0, 1);
IPAddress WiFISubNetwork(255, 255, 255, 0);

// Variable to store the HTTP request
String header;

unsigned long previousClientTime = 0; 
const long clientTimeout = 500;

// --------------------------------------------------------------------

void setup(void) {
  setupDebug();

  lcd.init();
  lcd.backlight();

  WiFi.mode(WIFI_STA);

  // Connect to Wi-Fi network with SSID and password
  commonDebug("Connecting to Wi-Fi network" + (String)(WIFI_SSID));

  if (!WiFi.config(WiFILocalIP, WiFIGateWay, WiFISubNetwork)) {
    commonDebug("Failed to configure STATION mode!");
	  lcd.setCursor(0, 0);
	  lcd.print("Failed configur");
	  lcd.setCursor(0, 1);
	  lcd.print("-ing Wi-Fi!");
  } else {
	  lcd.setCursor(0, 0);
	  lcd.print("Connecting");
	  lcd.setCursor(0, 1);
	  lcd.print("to Wi-Fi...");
	}

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    commonDebug(".");
    lcd.print(".");
  }

  ArduinoOTA.setHostname("heaterMonitor");
  //ArduinoOTA.setPassword(WIFI_OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    commonDebug("Start updating " + type);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Start OTA");
  });
  ArduinoOTA.onEnd([]() {
    commonDebug("End OTA");
    lcd.noBacklight();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int tmp = (unsigned long) progress * 100 / total;
    commonDebug("Progress: " + tmp);
    lcd.setCursor(0, 1);
    lcd.print("Progress: " + tmp);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    commonDebug("Error number: " + error);
    lcd.setCursor(0, 0);
    lcd.print("Error in OTA: " + error);
    lcd.setCursor(0, 1);

    if (error == OTA_AUTH_ERROR) {
      commonDebug("Auth Failed");
      lcd.print("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      commonDebug("Begin Failed");
      lcd.print("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      commonDebug("Connect Failed");
      lcd.print("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      commonDebug("Receive Failed");
      lcd.print("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      commonDebug("End Failed");
      lcd.print("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Print local IP address and start web server
  commonDebug("");
  commonDebug("WiFi connected.");
  commonDebug("IP address: ");
  commonDebug(WiFi.localIP().toString());
  lcd.setCursor(0, 0);
  lcd.print("IP address:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  // delay to read
  delay(2000);
	lcd.noBacklight();

  server.begin();

  lcd.setCursor(7, 0);
  lcd.print(" | ");
  lcd.setCursor(7, 1);
  lcd.print(" | ");

  DS18B20Sensors.begin();

  dht22.begin();

  currentSensor = 1;

	inputTemperature = NAN;
	previuosInputTemperature = NAN;
	inputTemperatureSpeed = NAN;

	outputTemperature = NAN;
	previuosOutputTemperature = NAN;
	outputTemperatureSpeed = NAN;

	waterTemperature = NAN;
	previuosWaterTemperature = NAN;
	waterTemperatureSpeed = NAN;

	airTemperature = NAN;
	airHumidity = NAN;

	oneSensorInterval = sensorPollingInterval * maxSensors;

	heatingStartTime = 0;
	prevHeatingStartTime = 0;
	heatingInterval = 0;
  lastHeatingDelay = 0;
  lastForcedHeatingDelay = 0;
  heaterState = 0;

	minHeatingTemperature = NAN;
	maxHeatingTemperature = NAN;
}

void loop(void) {
  unsigned int tmpHeatingInterval;
  unsigned long currentTime = millis();

  if (currentTime < previousSensorPollingTime) {
    previousSensorPollingTime = currentTime;
    return; // protect from counter overflow
  }

  if (currentTime - previousSensorPollingTime  >= sensorPollingInterval) {
    // save the last time you updated the sensor value
    previousSensorPollingTime = currentTime;
    
    lcd.setCursor(7, 0);
    lcd.print(" | ");
    lcd.setCursor(7, 1);
    lcd.print(" | ");

		switch( currentSensor ) {
			case 1:
				if (DS18B20Sensors.requestTemperaturesByAddress(DS18B20SensorAddressInput)) {

					previuosInputTemperature = inputTemperature;
					inputTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressInput);
					inputTemperatureSpeed = (inputTemperature - previuosInputTemperature) * 1000 / (double)oneSensorInterval;

					printTemperatureAtLCD(inputTemperature, "i", 0, 0);
					commonDebug((String)("Temperature of heater input: ") + inputTemperature);
					commonDebug((String)("Temperature speed of heater input: ") + inputTemperatureSpeed);
				}
				break;

			case 2:
				if (DS18B20Sensors.requestTemperaturesByAddress(DS18B20SensorAddressOutput)) {

					previuosOutputTemperature = outputTemperature;
					outputTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressOutput);
					outputTemperatureSpeed = (outputTemperature - previuosOutputTemperature) * 1000 / (double)oneSensorInterval;

					printTemperatureAtLCD(outputTemperature, "o", 0, 1);
					commonDebug((String)("Temperature of heater output: ") + outputTemperature);
					commonDebug((String)("Temperature speed of heater output: ") + outputTemperatureSpeed);
				}
				break;

			case 3:
				if (DS18B20Sensors.requestTemperaturesByAddress(DS18B20SensorAddressWater)) {

					previuosWaterTemperature = waterTemperature;
					waterTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressWater);
					waterTemperatureSpeed = (waterTemperature - previuosWaterTemperature) * 1000 / (double)oneSensorInterval;

					printTemperatureAtLCD(waterTemperature, "w", 10, 0);
					commonDebug((String)("Temperature of hot water: ") + waterTemperature);
					commonDebug((String)("Temperature speed of heater water: ") + waterTemperatureSpeed);
				}
				break;

			case 4:
				airTemperature = dht22.readTemperature(); // Read temperature as Celsius (the default)
				// Check if any reads failed and exit early (to try again).
				if (isnan(airTemperature)) {
					commonDebug("Failed to read from DHT sensor!");
				} else {
          printTemperatureAtLCD(airTemperature, "a", 10, 1);
          commonDebug((String)("Temperature of air: ") + airTemperature);
				}
				break;

			case 5:
				airHumidity = dht22.readHumidity(); // Reading temperature or humidity takes about 250 milliseconds! // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
				// Check if any reads failed and exit early (to try again).
				if (isnan(airHumidity)) {
					commonDebug("Failed to read from DHT sensor!");
				} else {
          commonDebug((String)("Humidity of air: ") + airHumidity);
				}
				break;
		}

    commonDebug("---- end of sensor polling");

		currentSensor++;
		if (currentSensor > maxSensors)
			currentSensor = 1;

		// ############ do some math and evaluate heater state

    // decisions based on previous state
    lastForcedHeatingDelay += sensorPollingInterval/1000;
    lastHeatingDelay += sensorPollingInterval/1000;

    // new state evaluation
		if (outputTemperatureSpeed <= outputSpeedCoolingThreshold ) {
			heaterState = 1;	// cooling / idle
		}
		if (waterTemperatureSpeed >= waterSpeedHeatingThreshold ) {
			heaterState = 4;	// heat water, most precedence over other heating modes
		}
		if (outputTemperatureSpeed >= outputSpeedHeatingThreshold ) {
			if ( heaterState == 1 ) {
				heaterState = 2;	// some heating mode: either room or water
				heatingStartTime = currentTime;
			}
		}
		if (inputTemperatureSpeed >= inputSpeedHeatingThreshold ) {
			if ( heaterState == 2 ) {
				heaterState = 3;	// heat room
        lastHeatingDelay = 0;

				// calculate heatingInterval;
				if (heatingInterval == 0) {	// first or second cicle
					if ( prevHeatingStartTime > 0 ) { // certainly second cicle
						heatingInterval = (heatingStartTime -  prevHeatingStartTime) / 1000;
					}
				} else {
          if (heatingStartTime >  prevHeatingStartTime) { // protect from counter overflow
					  tmpHeatingInterval = (heatingStartTime -  prevHeatingStartTime) / 1000;
					  heatingInterval = (1 - heatingCicleTimeCorrectionFraction/2)* heatingInterval + heatingCicleTimeCorrectionFraction/2 * tmpHeatingInterval;
          }
				}

				prevHeatingStartTime = heatingStartTime;
			}
		}

    commonDebug((String)("Heater state (num): ") + heaterState);
    commonDebug((String)("Heater state: ") + getHeaterStateString());

    // decisions based on new state
		if (heaterState == 1) { // cooling
      if (isnan(minHeatingTemperature)) {
        minHeatingTemperature = outputTemperature;
      }
			else if (outputTemperature <= minHeatingTemperature) {
				minHeatingTemperature = (minHeatingTemperature + outputTemperature)/2;
			}

      if( heatingInterval > 0 && currentTime > prevHeatingStartTime + heatingInterval * forceHeatingIntervalFraction ){
        commonDebug("Start forced room heating!");
        lastForcedHeatingDelay = 0;
      }
		}

		if (heaterState == 3) { // heat room
      if (isnan(maxHeatingTemperature)) {
        maxHeatingTemperature = outputTemperature;
      } else if (outputTemperature >= maxHeatingTemperature) {
				maxHeatingTemperature = (maxHeatingTemperature + outputTemperature)/2;
			}
		}

  } else {

    ArduinoOTA.handle();

		// ###########  Wi-Fi block
    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                             // If a new client connects,
      webServerDebug("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      currentTime = millis();
      previousClientTime = currentTime;
      while (client.connected() && currentTime - previousClientTime <= clientTimeout) { // loop while the client's connected
        currentTime = millis();
        if (currentTime < previousClientTime ) { // counter overflow
          header = ""; // just break this session
          client.stop();
          webServerDebug("Break sesson due time counter overflow.");
          break;
        }
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          webServerDebugByte(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              if (header.indexOf("GET /inputTemperature") >= 0) {
                webServerDebug("Input temperature requested");
                if( isnan(inputTemperature)==false ) {
                  client.println(inputTemperature);
                }
              } else if (header.indexOf("GET /outputTemperature") >= 0) {
                webServerDebug("Output temperature requested");
                if( isnan(outputTemperature)==false ) {
                  client.println(outputTemperature);
                }
              } else if (header.indexOf("GET /waterTemperature") >= 0) {
                webServerDebug("Water temperature requested");
                if( isnan(waterTemperature)==false ) {
                  client.println(waterTemperature);
                }
              } else if (header.indexOf("GET /airTemperature") >= 0) {
                webServerDebug("Air temperature requested");
                if( isnan(airTemperature)==false ) {
                  client.println(airTemperature);
                }
              } else if (header.indexOf("GET /airHumidity") >= 0) {
                webServerDebug("Air humidity requested");
                if( isnan(airHumidity)==false ) {
                  client.println(airHumidity);
                }
              } else if (header.indexOf("GET /heaterState") >= 0) {
                webServerDebug("Heater state requested");
                client.println(heaterState);
              } else if (header.indexOf("GET /heatingInterval") >= 0) {
                webServerDebug("Heating interval requested");
                if( heatingInterval > 0 ) {
                  client.println(heatingInterval);
                }
              } else if (header.indexOf("GET /minHeatingTemperature") >= 0) {
                webServerDebug("Minimal heating temperature requested");
                if( isnan(minHeatingTemperature)==false ) {
                  client.println(minHeatingTemperature);
                }
              } else if (header.indexOf("GET /maxHeatingTemperature") >= 0) {
                webServerDebug("Maximal heating temperature requested");
                if( isnan(maxHeatingTemperature)==false ) {
                  client.println(maxHeatingTemperature);
                }
              } else if (header.indexOf("GET /lastHeatingDelay") >= 0) {
                webServerDebug("Last heating delay requested");
                client.println(lastHeatingDelay);
              } else if (header.indexOf("GET /lastForcedHeatingDelay") >= 0) {
                webServerDebug("Last forced heating delay requested");
                client.println(lastForcedHeatingDelay);
              } else if (header.indexOf("GET /revision") >= 0) {
                webServerDebug("Firmware revision requested");
                client.println(firmwareRevision );
              }

              client.println();
              // Break out of the while loop
              break;
            } else { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        } else {
					yield();
				}
      }
      // Clear the header variable
      header = "";
      // Close the connection
      client.stop();
      webServerDebug("Client disconnected.");
    }
  }
}

void printTemperatureAtLCD(float temperature, String name, int column, int row) {
/*
i 99.9 | w 99.9
o 99.9 | a 99.9
*/

  lcd.setCursor(column, row);
  lcd.print(name + " ");
  lcd.setCursor(column+2, row);
  lcd.print(temperature);
}

const char *getHeaterStateString() {
  switch( heaterState ) {
    case 0:
      return "unknown";
    case 1:
      return "cooling/idle";
    case 2:
      return "heatingUnknown";
    case 3:
      return "heatingRoom";
    case 4:
      return "heatingWater";
  }
}

void setupDebug() {
#ifdef DEBUG
#ifndef DEBUG_UDP
  Serial.begin(115200);
#endif
#endif
}

void commonDebug(String buffer) {
#ifdef DEBUG
#ifdef DEBUG_COMMON
#ifdef DEBUG_COMMON_UDP
  buffer.toCharArray(DebugUDPBuffer, DEBUG_COMMON_UDP_BUFFER_LENGTH);
  DebugUDP.beginPacketMulticast(udpDebugMulticastAddress, UDP_DEBUG_PORT, WiFILocalIP);
  DebugUDP.write(DebugUDPBuffer);
  DebugUDP.endPacket();
#else
  Serial.println(buffer);
#endif
#endif
#endif
}

void webServerDebug(String buffer) {
#ifdef DEBUG
#ifdef DEBUG_WEB
  Serial.println(buffer);
#endif
#endif
}

void webServerDebugByte(char buffer) {
#ifdef DEBUG
#ifdef DEBUG_WEB
  Serial.print(buffer);
#endif
#endif
}

// vim: ts=2 sts=2 sw=2
