//This code is designed to run on the Arduino Mega microcontroller.
It handles the control of the motor mechanism and the reading of sensor values.
Specifically, it reads data from temperature and rain sensors to determine environmental conditions.
Based on the sensor readings, it controls the motor mechanism to open or close the shed accordingly.
The code implements logic to respond to changes in temperature and rain levels to protect crops stored in the shed.





#include <AFMotor.h>
#include <DHT11.h>
#include <SoftwareSerial.h> 

// Pin Definitions for Motor Driver 1 (L298N)
int ena1 = 2; // PWM pin for motor speed control 1
int in1_1 = 3; // Motor 1 control input 1
int in2_1 = 4; // Motor 1 control input 2
int in3_1 = 5; // Motor 2 control input 1
int in4_1 = 6; // Motor 2 control input 2


// Pin Definitions for Motor Driver 2 (L298N)
int ena2 = 8;  // PWM pin for motor speed control 2
int in1_2 = 9; // Motor 3 control input 1
int in2_2 = 10;// Motor 3 control input 2
int in3_2 = 11;// Motor 4 control input 1
int in4_2 = 12;// Motor 4 control input 2

SoftwareSerial mySerial(18, 19); // RX, TX
// Raindrop Sensor
int rainSensorPin = A0;

// Temperature Sensor (DHT11)
DHT11 dht11(7);

int rainThreshold = 500;
int tempThreshold = 35;

// UART Settings
#define SERIAL_BAUD_RATE 115200

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  mySerial.begin(SERIAL_BAUD_RATE);
  Serial.println("Setup...");

  // Motor Driver 1 (L298N) Pin Initialization
  pinMode(ena1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in3_1, OUTPUT);
  pinMode(in4_1, OUTPUT);

  // Motor Driver 2 (L298N) Pin Initialization
  pinMode(ena2, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
  pinMode(in3_2, OUTPUT);
  pinMode(in4_2, OUTPUT);
  // Set motor speeds
  analogWrite(ena1, 50); // Set initial speed for motor 1
  analogWrite(ena2, 255); // Set initial speed for motor 3
}

void loop() {
  // Raindrop Sensor
  int rainValue = analogRead(rainSensorPin);
  mySerial.print("Raindrop Sensor Value: ");  // Indicate Raindrop Sensor value
  mySerial.println(rainValue);

  if (rainValue < rainThreshold) {
    Serial.println("Rain detected, starting clockwise rotation...");

    // Clockwise rotation
    for (int i = 0; i < 7; i++) {  /////////6 sets
      analogWrite(ena1, 50); // Set speed for motor 1
      analogWrite(ena2, 255); // Set speed for motor 3
      digitalWrite(in1_1, HIGH);
      digitalWrite(in2_1, LOW);
      digitalWrite(in3_1, HIGH);
      digitalWrite(in4_1, LOW);
      Serial.println("Clockwise rotation - Set " + String(i + 1));
      delay(3000);
      digitalWrite(in1_1, LOW);
      digitalWrite(in2_1, LOW);
      delay(4000);
    }

    Serial.println("Stopping motors...");
    // Stop motors
    digitalWrite(in1_1, LOW);
    digitalWrite(in2_1, LOW);
    digitalWrite(in3_1, LOW);
    digitalWrite(in4_1, LOW);

    // Wait until rain sensor is clear
    Serial.println("Waiting for rain sensor to clear...");
    while (rainValue < 1023) {
      rainValue = analogRead(rainSensorPin);
      delay(500);
    }

    Serial.println("Rain sensor clear, starting anticlockwise rotation...");

    // Anti-clockwise rotation
    for (int i = 0; i < 6; i++) {    /////////// 6 sets
      analogWrite(ena1, 180); // Set speed for motor 1
      analogWrite(ena2, 255); // Set speed for motor 3
      digitalWrite(in1_1, LOW);
      digitalWrite(in2_1, HIGH);
      digitalWrite(in3_1, LOW);
      digitalWrite(in4_1, HIGH);
      Serial.println("Anti-clockwise rotation - Set " + String(i + 1));
      delay(3000);
      digitalWrite(in1_1, LOW);
      digitalWrite(in2_1, LOW);
      delay(4000);
    }

    Serial.println("Stopping motors...");
    // Stop motors
    digitalWrite(in1_1, LOW);
    digitalWrite(in2_1, LOW);
    digitalWrite(in3_1, LOW);
    digitalWrite(in4_1, LOW);
  }
  // Temperature Sensor
  float tempValue = dht11.readTemperature();
  mySerial.print("Temperature Sensor Value: ");  // Indicate Temperature Sensor value
  mySerial.println(tempValue);
if (!isnan(tempValue)) {
  if (tempValue >= 0 && tempValue <= 34) {
    Serial.println("Temperature within the normal range. No action needed.");
  } else if (tempValue >= 35) {
    Serial.println("High temperature detected, starting clockwise rotation...");

    // Clockwise rotation
    analogWrite(ena2, 255); // Set speed for motor 3
    digitalWrite(in1_2, HIGH);
    digitalWrite(in2_2, LOW);
    digitalWrite(in3_2, HIGH);
    digitalWrite(in4_2, LOW);
    delay(12000);

    // Stop motors
    digitalWrite(in1_2, LOW);
    digitalWrite(in2_2, LOW);
    digitalWrite(in3_2, LOW);
    digitalWrite(in4_2, LOW);

    Serial.println("Waiting for temperature to go below threshold...");
    // Wait for temperature to go below threshold
    while (tempValue >= 35 || tempValue < 28) {
      tempValue = dht11.readTemperature();
      delay(1000);
    }

    Serial.println("Temperature back to normal range, starting anticlockwise rotation...");

    // Anti-clockwise rotation
    analogWrite(ena2, 255); // Set speed for motor 3
    digitalWrite(in1_2, LOW);
    digitalWrite(in2_2, HIGH);
    digitalWrite(in3_2, LOW);
    digitalWrite(in4_2, HIGH);
    delay(11000);

    Serial.println("Stopping motors...");
    // Stop motors
    digitalWrite(in1_2, LOW);
    digitalWrite(in2_2, LOW);
    digitalWrite(in3_2, LOW);
    digitalWrite(in4_2, LOW);
  }
}

Serial.println("Loop delay...");
  // Rest of your code remains unchanged
  // ...

  delay(1000);  // Add any necessary delays
}
