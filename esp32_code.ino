//The code for The project
//This is IoT code to recieve the sensor values from the Arduino Mega to esp32 using uart communication protocol and then sends the value to cloud platform such as Blynk

#define BLYNK_TEMPLATE_ID "TMPL3LlZLGXs5"
#define BLYNK_TEMPLATE_NAME "CROP PROTECTION OVER SUN AND RAIN"
#define BLYNK_AUTH_TOKEN "yg_qQMfC1fYdetsLd_1qChYU683-lGUd"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char auth[] = "yg_qQMfC1fYdetsLd_1qChYU683-lGUd";  // Replace with your Blynk authentication token

// Virtual pins for Blynk widgets
#define V1 1
#define V2 2

// UART Settings
#define SERIAL_BAUD_RATE 115200

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Serial communication initialized.");

  Blynk.begin(auth, "Traffic_Monitor", "Project2024");  // Replace with your WiFi credentials
  Serial.println("Blynk initialized.");

}

void loop() {
  if (Serial.available() > 0) {
    String sensorType = Serial.readStringUntil(':');
    sensorType.trim();

    Serial.print("Received sensor type: ");
    Serial.println(sensorType);

    if (sensorType.equals("Raindrop Sensor Value: ")) {
      handleRaindropSensor();
    } else if (sensorType.equals("Temperature Sensor Value: ")) {
      handleTemperatureSensor();
    } else {
      Serial.println("Unknown sensor type. Ignoring.");
    }
  }

  Blynk.run();
}

void handleRaindropSensor() {
  float rainValue = Serial.parseInt();
  Serial.print("Raindrop sensor value received: ");
  Serial.println(rainValue);

  // Update Blynk with rain value on virtual pin V2
  Blynk.virtualWrite(V1, rainValue);
}

void handleTemperatureSensor() {
  float tempValue = Serial.parseFloat();
  Serial.print("Temperature sensor value received: ");
  Serial.println(tempValue);

  // Update Blynk with temperature value on virtual pin V1
  Blynk.virtualWrite(V2, tempValue);
}
