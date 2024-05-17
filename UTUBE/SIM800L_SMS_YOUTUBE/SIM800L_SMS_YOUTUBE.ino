
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define rxPin 2
#define txPin 3
SoftwareSerial sim800L(rxPin,txPin); 

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // Create an object for the BME680
String buff;
void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  sim800L.begin(9600);

  Serial.println("Initializing...");
  
  sim800L.println("AT");
  waitForResponse();

  sim800L.println("ATE1");
  waitForResponse();

  sim800L.println("AT+CMGF=1");
  waitForResponse();

  sim800L.println("AT+CNMI=1,2,0,0,0");
  waitForResponse();
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
  if (simModule.available()) {
    String incomingData = simModule.readString(); // Read the data from the SIM module
    if (incomingData.indexOf("RING") >= 0) { // Check if the data contains "RING"
      sendTemperatureSMS(); // Function to send the temperature via SMS
      simModule.println("ATH"); // Hang up the call to reset for the next call
      delay(5000); // Simple debounce to avoid multiple detections for the same call
    }
  }
}

void sendTemperatureSMS() {
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  String message = "Temperature: " + String(bme.temperature) + " C";

  simModule.println("AT+CMGF=1"); // Set the SIM800L to SMS text mode
  delay(100); // Short delay to ensure the command is processed
  simModule.println("AT+CMGS=\"+1234567890\""); // Replace with your phone number
  delay(100);
  simModule.println(message); // Send the temperature message
  delay(100);
  simModule.write(26); // CTRL+Z to send the message
  delay(100);

  // Log to the serial monitor
  Serial.println("Temperature SMS Sent.");
}



