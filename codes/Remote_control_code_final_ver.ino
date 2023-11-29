#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
const char* ssid = "Chanukya_svsk";  // my mobile's WiFi. 
const char* password = "esw_project";

// Pins of L298N motor Driver enable and input pins.
#define ENA 12
#define ENB 13
#define IN1 33  // 1 for motor A
#define IN2 25  // 2 for motor A 
#define IN3 26  // 1 for motor B
#define IN4 27  // 2 for motor B
#define carspeed 180
WiFiClient client;
WebServer server(80); // Webserver object to handle HTTP requests.

String currentState = "Unknown";
void sendHTML(); // Function prototype

// functions for movement of car using L298N
void forward();
void backward();
void rightside(int turnAngle);
void leftside(int turnAngle);
void stop();
void handleForward();
void handleBackward();
void handleStop();
void handleRightward();
void handleLeftward();

void setup() {
  Serial.begin(9600); // Higher baud rate for faster data transfer.
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  WiFi.mode(WIFI_STA);
  // Serial.println("Before WiFi begin");
  delay(2000);
  WiFi.begin(ssid,password);
  Serial.print("Connecting to WiFi..");
  delay(2000);
  while(WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
  }
  // Serial.print("\n");
  Serial.println("WiFi Connected.");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP()); // Print the local IP address

  server.on("/", sendHTML); // Handle root URL with sendHTML function.  
  server.on("/forward", HTTP_GET, handleForward);
  server.on("/backward", HTTP_GET, handleBackward);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/rightward", HTTP_GET, handleRightward);
  server.on("/leftward", HTTP_GET, handleLeftward);
  server.begin(); // start the HTTP server
  Serial.println("\nHTTP Server Started.");
}

void loop() {
  server.handleClient(); // Handle incoming HTTP requests.
}

// HTML code for control webpage
const char HTMLCode[] PROGMEM = R"rawliteral(
  <!DOCTYPE html><html>
    <head>
        <style>
            .header1 {
                font-style: italic;
                color: red;
                text-align: center;
                font-size: 50px;
            }
            .para1 {
                font-size: 30px;
                text-align: center;
                color: blue;
            }
            .button12 {
                text-align: center;
            }
            .button1 {
                background-color:aquamarine;
                font-size: 40px;
                margin-bottom: 70px;
                margin-top: 60px;
                cursor: pointer;
            }
            .button-container {
                text-align: center;
                margin-left: 110px;
            }
            .button2 {
                background-color: lightgreen;
                font-size: 40px;
                margin-right: 100px;
                cursor: pointer;
            }
            .button3 {
                background-color: red;
                font-size: 40px;
                margin-right: 100px;
                cursor: pointer;
            }
        </style>
    </head>
    <body>
        <h1 class = "header1">Remote control Obstacle avoiding Robot</h1>
        <!-- print the current state of the vehicle i.e., moving in a direction or stop -->
        <p id="currentStateParagraph" class="para1">Robot is currently:<span id="currentState">Unknown</span></p>
        <div class="button12">
            <button class="button1" onclick="sendCommand('forward')">Forward</button>
        </div>
        <div class="button-container">
            <button class="button2" onclick="sendCommand('leftward')">Leftward</button>
            <button class="button3" onclick="sendCommand('stop')">Stop</button>
            <button class="button2" onclick="sendCommand('rightward')">Rightward</button>
        </div>
        <div class="button12">
            <button class="button1" onclick="sendCommand('backward')">Backward</button>
        </div>
        <script>
            let currentState = "Unknown";
            function sendCommand(command) {
                fetch('/' + command)
                .then(response => response.text())
                .then(data => {
                    console.log(data);
                    currentState = data;
                })
                .catch(error => {
                    console.error('Error:',error);
                });
                document.getElementById('currentState').innerText = currentState;
            }
        </script>
    </body>
  </html>
)rawliteral";

void sendHTML() {
  server.send(200, "text/html", String(HTMLCode));
}
void handleForward() {
  forward();
  currentState = "Moving Forward";
  server.send(200, "text/plain", currentState);
}
void handleBackward() {
  backward();
  currentState = "Moving backward";
  server.send(200, "text/plain", currentState);
}
void handleStop() {
  stop();
  currentState = "Stopped";
  server.send(200, "text/plain", currentState);
}
void handleRightward() {
  rightside();
  currentState = "Moving rightside";
  server.send(200, "text/plain", currentState);
}
void handleLeftward() {
  leftside();
  currentState = "Moving leftside";
  server.send(200, "text/plain", currentState);
}

void forward() {
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Robot is going Forward.");
}
void backward() {
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Robot is going backward.");
}
void rightside() {
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Robot is going rightside.");
}
void leftside() {
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Robot is going leftside.");
}
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Robot stopped.");
}
