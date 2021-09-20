#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP8266WiFi.h>

#include "WiFiSettings.h"
// contains next settings:
#define WIFI_SSID     CONFIG_WIFI_SSID
#define WIFI_PASSWORD	CONFIG_WIFI_PASSWORD

// updates some sensor every N milliseconds
const long SensorInterval = 1000;
const int maxSensors = 5;
int currentSensor;

/*
  DS18B20 settings
*/

OneWire oneWire(0); // at GPIO0 / D8 for Wemos D1 R1 (clone)

DallasTemperature DS18B20Sensors(&oneWire);

// indexes are: 0 is input (for heater), 1 is output and 2 for hot water
DeviceAddress DS18B20SensorAddressInput = { 0x28, 0x4F, 0x1B, 0x07, 0xD6, 0x01, 0x3C, 0xD4 };
DeviceAddress DS18B20SensorAddressOutput = { 0x28, 0xA5, 0x0D, 0x75, 0xD0, 0x01, 0x3C, 0xAC };
DeviceAddress DS18B20SensorAddressWater = { 0x28, 0xD2, 0x5D, 0x07, 0xD6, 0x01, 0x3C, 0x5A };

float inputTemperature, outputTemperature, waterTemperature;

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

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Set web server port number to 80
WiFiServer server(80);

IPAddress local_IP(192, 168, 0, 200);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// Variable to store the HTTP request
String header;

unsigned long currentTime = millis();
unsigned long previousSensorTime = 0; 
unsigned long previousClientTime = 0; 
const long ClientTimeoutTime = 500;

// --------------------------------------------------------------------

void setup(void) {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  WiFi.mode(WIFI_STA);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to Wi-Fi network ");
  Serial.println(ssid);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure STATION mode!");
  }

  lcd.setCursor(0, 0);
  lcd.print("Connecting");
  lcd.setCursor(0, 1);
  lcd.print("to Wi-Fi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
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
	outputTemperature = NAN;
	waterTemperature = NAN;
	airTemperature = NAN;
	airHumidity = NAN;
}

void loop(void) {
  unsigned long currentTime = millis();
  if (currentTime - previousSensorTime  >= SensorInterval) {
    // save the last time you updated the sensor value
    previousSensorTime = currentTime;
    
		switch( currentSensor ) {
			case 1:
				DS18B20Sensors.requestTemperatures(); // Send the command to get temperatures

				inputTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressInput);
				printTemperatureAtLCD(inputTemperature, "i", 0, 0);
				Serial.print("Temperature of heater input: ");
				Serial.println(inputTemperature);
				break;

			case 2:
				outputTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressOutput);
				printTemperatureAtLCD(outputTemperature, "o", 0, 1);
				Serial.print("Temperature of heater output: ");
				Serial.println(outputTemperature);
				break;

			case 3:
				waterTemperature = DS18B20Sensors.getTempC(DS18B20SensorAddressWater);
				printTemperatureAtLCD(waterTemperature, "w", 10, 0);
				Serial.print("Temperature of hot water: ");
				Serial.println(waterTemperature);
				break;

			case 4:
				airTemperature = dht22.readTemperature(); // Read temperature as Celsius (the default)
				// Check if any reads failed and exit early (to try again).
				if (isnan(airTemperature)) {
					Serial.println(F("Failed to read from DHT sensor!"));
				} else {
          printTemperatureAtLCD(airTemperature, "a", 10, 1);
          Serial.print("Temperature of air: ");
          Serial.println(airTemperature);
				}
				break;

			case 5:
				airHumidity = dht22.readHumidity(); // Reading temperature or humidity takes about 250 milliseconds! // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
				// Check if any reads failed and exit early (to try again).
				if (isnan(airHumidity)) {
					Serial.println(F("Failed to read from DHT sensor!"));
				} else {
          Serial.print("Humidity of air: ");
          Serial.println(airHumidity);
				}
				break;
		}

    Serial.println("---- end of sensor polling");

		currentSensor++;
		if (currentSensor > maxSensors)
			currentSensor = 1;

  } else {

    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                             // If a new client connects,
      Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      currentTime = millis();
      previousClientTime = currentTime;
      while (client.connected() && currentTime - previousClientTime <= ClientTimeoutTime) { // loop while the client's connected
        currentTime = millis();         
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
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
                Serial.println("Input temperature requested");
                client.println(inputTemperature);
              } else if (header.indexOf("GET /outputTemperature") >= 0) {
                Serial.println("Output temperature requested");
                client.println(outputTemperature);
              } else if (header.indexOf("GET /waterTemperature") >= 0) {
                Serial.println("Water temperature requested");
                client.println(waterTemperature);
              } else if (header.indexOf("GET /airTemperature") >= 0) {
                Serial.println("Air temperature requested");
                client.println(airTemperature);
              } else if (header.indexOf("GET /airHumidity") >= 0) {
                Serial.println("Air humidity requested");
                client.println(airHumidity);
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
      Serial.println("Client disconnected.");
      Serial.println("");
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

// vim: ts=2 sts=2 sw=2
