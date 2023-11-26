// Logic:
/*
1. Readings taken from ultrasonic sensor and hence obstacle is detected
2. Then find the dirction where obstacle is not present using servo motor
3. Turn in that direction and move
4. above three steps continue.
*/

/*
TODO:
1. send obstacle_distance values to thingspeak.
2.  

TO TEST:
2. Turning by angle 45 degrees.
3. avoid_obstacle function  
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
float obstacle_distance = 0;  // stores the distance of obstacle in the new direction of motion to give time argument to motion functions
unsigned long startTime; // useful to measure the time car need to travel.

// function to measure distance using ultrasonic sensor
float Distance_measure();
int is_obstacle_present();
void forward(float delay_sec);
void backward(float delay_sec);
void rightside(float delay_sec);
void leftside(float delay_sec);
void stop();
int max_dist_angle();
void avoid_obstacle();

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
  startTime=millis();
  // inital position of servo motor is in middle position. So...
  myservo.write(90);
}

void loop() {
  // find obstacle
  // find obstacle free path
  myservo.write(90);
  int obstacle = is_obstacle_present();
  if (!obstacle) {
    // obstacle is not present. 
    Distance = Distance_measure();
    Serial.print("Distance in cm: ");
    Serial.println(Distance);
    float move_time = (Distance-minDistance)/carspeed;
    forward(move_time);
  } else {
    Distance = Distance_measure();
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
  float distance = Distance_measure();
  Serial.print("Distance in cm: ");
  Serial.println(distance);
  if (distance <= minDistance) { 
    return 1;
  } else {
    return 0;
  }
}
// functions for movement of car using L298N

// moves the car forward
void forward(float delay_sec) {
  startTime=millis();
  while (millis()-startTime < delay_sec*1000) {
    analogWrite(ENA, carspeed);
    analogWrite(ENB, carspeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  Serial.println("Robot is going Forward.\n");
  stop();
  delay(1000);
}
// moves the car backward
void backward(float delay_sec) {
  startTime=millis();
  while (millis()-startTime < delay_sec*1000) {
    analogWrite(ENA, carspeed);
    analogWrite(ENB, carspeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(1000);
  }
  Serial.println("Robot is going backward.\n");
  stop();
}
// moves the car rightward
void rightside(float delay_sec) {
  startTime=millis();
  while (millis()-startTime < delay_sec*1000) {
    analogWrite(ENA, carspeed);
    analogWrite(ENB, carspeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Robot is going rightside.\n");
    delay(1000);
  }
  stop();
}
// moves the car leftward
void leftside(float delay_sec) {
  startTime=millis();
  while (millis()-startTime < delay_sec*1000) {
    analogWrite(ENA, carspeed);
    analogWrite(ENB, carspeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(1000);
  }
  Serial.println("Robot is going leftside.\n");
  stop();
}
// stops the car
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Robot stopped."); 
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
  left45Distance = Distance_measure();
  Serial.print("left45Distance: ");
  Serial.println(left45Distance);
  myservo.write(0);
  delay(1000);
  leftDistance = Distance_measure();
  Serial.print("leftDistance: ");
  Serial.println(leftDistance);
  myservo.write(135);
  delay(1000);
  right45Distance = Distance_measure();
  Serial.print("right45Distance: ");
  Serial.println(right45Distance);
  myservo.write(180);
  delay(1000);
  rightDistance = Distance_measure();
  Serial.print("rightDistance: ");
  Serial.println(rightDistance);
  int long_middle = (middleDistance>=right45Distance && middleDistance>=rightDistance)
                    && (middleDistance>=left45Distance && middleDistance>=leftDistance);
  int long_left45 = (left45Distance>=right45Distance && left45Distance>=rightDistance)
                    && (left45Distance>=middleDistance && left45Distance>=leftDistance);
  int long_right45 =(right45Distance>=middleDistance && right45Distance>=rightDistance)
                    && (right45Distance>=left45Distance && right45Distance>=leftDistance);
  int long_left = (leftDistance>=right45Distance && leftDistance>=rightDistance)
                    && (leftDistance>=left45Distance && leftDistance>=middleDistance);
  int long_right = (rightDistance>=right45Distance && rightDistance>=middleDistance)
                    && (rightDistance>=left45Distance && rightDistance>=leftDistance);
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
    myservo.write(90);
    forward(move_time);
    stop();
  } else if (max_angle==0) {
    myservo.write(0);
    leftside(90);
    forward(move_time); 
    stop();
  } else if (max_angle==45) {
    myservo.write(45);
    leftside(45);
    forward(move_time);
    stop();
  } else if (max_angle==135) {
    myservo.write(135);
    rightside(45);
    forward(move_time);
    stop();
  } else if (max_angle==180) {
    myservo.write(180);
    rightside(90);
    forward(move_time);
    stop();
  }
}

