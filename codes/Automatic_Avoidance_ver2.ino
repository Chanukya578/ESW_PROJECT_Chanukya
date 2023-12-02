/*
Assumptions: 
  1. Every distance we calculate are in centimeters.

Limitations: 
  1. In remote control part, The vehicle is not easy to control. 
     since motor driver works only when speed is high for a 9V battery.
  2. Since the servo motor is present at a considerable height in front of robot,
      Obstacles like speed breakers are not identified. 
      -> In real world scenario, it can be clarified easily since if we keep it lower, 
         A new limitation like avoiding the speed breaker also arises if we keep it low. 
  3. 

TO TEST: 
  1. check whether rightside() & leftside() function turns for 45 angle or not
  2. 
*/

#include <ESP32Servo.h>
Servo myservo; // object used to control servo motor
#define Servo_pin 4

// Trigger and echo of ultrasonic sensors
#define TRIG 19  
#define ECHO 18  
// Pins of L298N motor Driver enable and input pins.
#define ENA 12
#define ENB 13
#define IN1 33  // 1 for motor A
#define IN2 25  // 2 for motor A 
#define IN3 26  // 1 for motor B
#define IN4 27  // 2 for motor B

// variables to find positions of obstacles in that direction.
float carspeed = 180;
float speed_of_sound = 34300;
float middleDistance = 0;
float leftDistance = 0;
float rightDistance = 0;
float left45Distance = 0;
float right45Distance = 0;
float Distance=0;
float minDistance = 20;
float move_time = 0;
float obstacle_distance = 0;  // stores the distance of obstacle in the new direction of motion to give time argument to motion functions
unsigned long startTime = 0; // useful to measure the time car need to travel.

// function to measure distance using ultrasonic sensor
float Distance_measure();       // DONE
int is_obstacle_present();      // DONE
int max_dist_angle();           // DONE
void avoid_obstacle();          // DONE
void forward(float delay_sec);  
void backward(float delay_sec);
void rightside(float delay_sec);
void leftside(float delay_sec);
void stop();

void setup() {
  myservo.attach(Servo_pin);
  Serial.begin(9600);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
}

void loop() {
  myservo.write(90);
  int obstacle = is_obstacle_present();
  if (obstacle==0) {
    // no obstacle
    move_time = (Distance-minDistance) / carspeed;
    forward(move_time);
  } else {
    // obstacle is present infront of the car. 
    avoid_obstacle();
  }
  delay(1000);
}

// measures Distance using ultrasonic sensor
float Distance_measure() {
  float duration, distance; // duration is time in micro sec and distance is distance in cm
  digitalWrite(TRIG, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  duration = pulseIn(ECHO,HIGH);
  distance = duration*speed_of_sound / (float)2000000; // duration is divided by 2 to get accurate distance
  return (float)distance;
}
// confirmation for obstacle present or not. 
int is_obstacle_present() {
  Distance = Distance_measure();
  Serial.print("middleDistance in cm: ");
  Serial.println(Distance);
  if (Distance <= minDistance) 
    return 1;
  else 
    return 0;
}
// function to find direction where obstacle is not present
int max_dist_angle() {
  // returns the angle where the there is no obstacle. 
  myservo.write(90);
  delay(1000);
  middleDistance = Distance_measure();
  Serial.print("middleDistance: ");
  Serial.println(middleDistance);
  myservo.write(45);
  delay(1000);
  right45Distance = Distance_measure();
  Serial.print("right45Distance: ");
  Serial.println(right45Distance);
  myservo.write(0);
  delay(1000);
  rightDistance = Distance_measure();
  Serial.print("rightDistance: ");
  Serial.println(rightDistance);
  myservo.write(135);
  delay(1000);
  left45Distance = Distance_measure();
  Serial.print("left45Distance: ");
  Serial.println(left45Distance);
  myservo.write(180);
  delay(1000);
  leftDistance = Distance_measure();
  Serial.print("leftDistance: ");
  Serial.println(leftDistance);
  int long_middle = (middleDistance>=right45Distance && middleDistance>=rightDistance)
                    && (middleDistance>=left45Distance && middleDistance>=leftDistance);
  int long_left = (leftDistance>=right45Distance && leftDistance>=rightDistance)
                    && (leftDistance>=left45Distance && leftDistance>=middleDistance);
  int long_right = (rightDistance>=right45Distance && rightDistance>=middleDistance)
                    && (rightDistance>=left45Distance && rightDistance>=leftDistance);
  int long_left45 = (left45Distance>=right45Distance && left45Distance>=rightDistance)
                    && (left45Distance>=middleDistance && left45Distance>=leftDistance);
  int long_right45 =(right45Distance>=middleDistance && right45Distance>=rightDistance)
                    && (right45Distance>=left45Distance && right45Distance>=leftDistance);
  if(long_middle) {
    obstacle_distance = middleDistance;
    return 90;
  } else if (long_left) {
    obstacle_distance = leftDistance;
    return 0;
  } else if (long_left45) {
    obstacle_distance = left45Distance;
    return 45;
  } else if (long_right) {
    obstacle_distance = rightDistance;
    return 180;
  } else if (long_right45) {
    obstacle_distance = right45Distance;
    return 135;
  }
}
// Finds the obstacle and avoids it.
void avoid_obstacle() {
  int max_angle = max_dist_angle();
  myservo.write(max_angle);
  float move_time = (obstacle_distance-minDistance)/carspeed;
  int turnAngle;
  if (max_angle==90) {
    forward(move_time);
    stop();
  } else if (max_angle==0) {
    rightside(90);
    forward(move_time); 
    stop();
  } else if (max_angle==45) {
    rightside(45);
    forward(move_time);
    stop();
  } else if (max_angle==135) {
    leftside(45);
    forward(move_time);
    stop();
  } else if (max_angle==180) {
    leftside(90);
    forward(move_time);
    stop();
  }
}
// moves the car forward for delay_sec time.
void forward(float delay_sec) {
  startTime = millis();
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  while (millis()-startTime < delay_sec*1000) {
    // checks for delay_sec time and then stops the car.
  }
  Serial.println("Robot is going Forward.");
  stop();
}
// moves the car backward for delay_sec time.
void backward(float delay_sec) {
  startTime = millis();
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  while (millis()-startTime < delay_sec*1000) {
    // checks for delay_sec time and then stops the car.
  }
  Serial.println("Robot is going backward.");
  stop();
}
// Turns the car rightside by some angle. 
void rightside(float delay_sec) {
  startTime = millis();
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  while (millis()-startTime < delay_sec*1000) {
    // checks for delay_sec time and then stops the car.
  }
  Serial.println("Robot is turning rightside.");
  stop();
}
// Turns the car leftside by some angle.
void leftside(float delay_sec) {
  startTime = millis();
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  while (millis()-startTime < delay_sec*1000) {
    // checks for delay_sec time and then stops the car.
  }
  Serial.println("Robot is turning leftside.");
  stop();
}
// Stops the car.
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Robot stopped.");
}














