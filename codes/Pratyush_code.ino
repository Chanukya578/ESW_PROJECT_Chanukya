#include <WiFi.h>
#include <Servo.h>
#include "ThingSpeak.h"

// WiFi details
const char *ssid = "AndroidAPPL";
const char *password = "pratyush";
const char *server = "api.thingspeak.com";
const char * myWriteAPIKey = "KE47IYP6S875I6OH";
unsigned long channelID = 2344352;

// Motor control pins
const int motor1A = 26;  
const int motor1B = 27;
const int motor2A = 25;
const int motor2B = 33;

const int motorsigA = 12;
const int motorsigB = 13;

// Ultrasonic sensor pins
const int trigPin = 19;
const int echoPin = 18;

// Servo details
Servo servo;
const int servoMiddle = 80;
const int servoLeft = 180;
const int servoRight = -40;

// Distances for obstacle detection
const int obstacleThreshold = 20;

// Other constants
const int freq = 30000;
const int pwmChannel = 14;
const int resolution = 8;
int dutyCycle = 160;

long distance;
WiFiClient  client;

// Distance sensor pin
const int distanceSensorPin = A0;

// Variables for distance measurements
long long int dis1 = 0;
long long int dis2 = 0;

// Time intervals for updating ThingSpeak
unsigned long previousThingSpeakUpdate = 0;
const long thingSpeakUpdateInterval = 15000; // Update every 15 seconds

// Task handle for ThingSpeak update task
TaskHandle_t updateThingSpeakTask;

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motorsigA, OUTPUT);
  pinMode(motorsigB, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize servo
  servo.attach(4);
  servo.write(servoMiddle);

  // Initialize PWM for motor control
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(motorsigA, pwmChannel);
  ledcAttachPin(motorsigB, pwmChannel);

   // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);   
    
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  // Connect to WiFi
  connectToWiFi();

  // Create a task for updating ThingSpeak
  xTaskCreatePinnedToCore(
    updateThingSpeakTaskFunction,   // Task function
    "UpdateThingSpeak",             // Task name
    10000,                           // Stack size (bytes)
    NULL,                            // Task parameter
    1,                               // Task priority
    NULL,           // Task handle
    0                                // Core to run the task on (0 or 1)
  );
}

void loop() {
  ledcWrite(pwmChannel, dutyCycle);
  // Read distance from the ultrasonic sensor
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1; // Calculate distance in centimeters

  // Serial.print("distance -- ");
  // Serial.println(distance);

  // Print distance for debugging
  Serial.print("Distance: ");
  Serial.println(distance);

  // Obstacle avoidance logic
  if (distance <= obstacleThreshold) {
    // If an obstacle is detected, turn left (adjust as needed)
    // motorControl(LOW, HIGH, HIGH, LOW); // Turn left
    motorControl(1, 0, 1, 0);
    delay(600);
    motorControl(0, 0, 0, 0);
    servo.write(servoLeft); // Turn the servo left
    // delay(250);
    delay(500);
    dis1=cal_dis();
    Serial.print("dis1: ");
    Serial.println(dis1);

    servo.write(servoRight);
    delay(500);
    dis2=cal_dis();
    Serial.print("dis2: ");
    Serial.println(dis2);

    servo.write(servoMiddle);

    if(dis1>dis2){
      //left
      motorControl(1, 0, 0, 1);
      delay(2000);
    }
    else{
      motorControl(0, 1, 1, 0);
      delay(2000);
    }
  } else {
    // If no obstacle is detected, move forward
    motorControl(0, 1, 0, 1); // Move forward
    servo.write(servoMiddle); // Return servo to the middle position
    delay(500);
  }
  // dutyCycle = 0;
  // while (dutyCycle <= 255){
  //   ledcWrite(pwmChannel, dutyCycle);   
  //   Serial.print("Forward with duty cycle: ");
  //   Serial.println(dutyCycle);
  //   dutyCycle = dutyCycle + 5;
  //   delay(500);
  // }
  ledcWrite(pwmChannel, dutyCycle);  
  delay(100);
}

void connectToWiFi() {

  if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(2000);     
      } 
      Serial.println("\nConnected.");
      Serial.println(WiFi.localIP());
    }
}

void updateThingSpeakTaskFunction(void *pvParameters) {
  
  while (1) {
    // Check if it's time to update ThingSpeak
    // if (millis() - previousThingSpeakUpdate >= thingSpeakUpdateInterval) {
      // Your ThingSpeak update logic here
      updateThingSpeak(distance, dis2);

      // Update the timestamp for the next update
      previousThingSpeakUpdate = millis();
    // }

    // Delay for a short period to avoid high CPU usage
    delay(100);
  }
}

void updateThingSpeak(long long int value1, long long int value2) {
  // Establish a connection to the ThingSpeak server
  String taskMessage = "Task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  // delay(1000);

  ThingSpeak.setField(1, static_cast<float>(distance));
  ThingSpeak.setField(2, static_cast<float>(dis1));
  ThingSpeak.setField(3, static_cast<float>(dis2));
  // Uncomment if you want to get temperature in Fahrenheit
  // ThingSpeak.setField(3, temperatureF);

  int x = ThingSpeak.writeFields(channelID, myWriteAPIKey);

  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(8000);
}


long long int cal_dis() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}

// Function to control the motors
void motorControl(int motor1AState, int motor1BState, int motor2AState, int motor2BState) {
  digitalWrite(motor1A, motor1AState);
  digitalWrite(motor1B, motor1BState);
  digitalWrite(motor2A, motor2AState);
  digitalWrite(motor2B, motor2BState);
}
