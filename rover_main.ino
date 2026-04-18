// #include <WiFi.h>
// #include <WebServer.h>

// const char* ssid = "RobotController";
// const char* password = "12345678";  // must be at least 8 chars

// WebServer server(80);

// int speedValue = 0;

// // Webpage (slider UI)
// const char webpage[] = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <body style="font-family:sans-serif;text-align:center;">
// <h2>Robot Speed Control</h2>

// <input type="range" min="0" max="100" value="0"
//        oninput="send(this.value)">
// <p>Speed: <span id="val">0</span></p>

// <script>
// function send(val) {
//   document.getElementById("val").innerHTML = val;
//   fetch("/set?speed=" + val);
// }
// </script>

// </body>
// </html>
// )rawliteral";

// // Serve webpage
// void handleRoot() {
//   server.send(200, "text/html", webpage);
// }

// // Handle slider updates
// void handleSet() {
//   if (server.hasArg("speed")) {
//     speedValue = server.arg("speed").toInt();
//     Serial.print("Speed set to: ");
//     Serial.println(speedValue);
//   }
//   server.send(200, "text/plain", "OK");
// }

// void setup() {
//   Serial.begin(115200);

//   // Start Access Point
//   WiFi.softAP(ssid, password);

//   Serial.println("AP started");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.softAPIP());  // usually 192.168.4.1

//   // Routes
//   server.on("/", handleRoot);
//   server.on("/set", handleSet);

//   server.begin();
// }

// void loop() {
//   server.handleClient();

//   // 👉 Use speedValue here in your robot logic
//   Serial.print("Speed = ");
//   Serial.println(speedValue);
// }
#include <Arduino.h> // Needed for float, Serial, etc.
#include "imu.h"
#include "motor.h"
Motor motor1({16, 17, 18});
// Motor motor2({11, 12, 13});
// Servo servo1;


void setup() {
  Serial.begin(115200);

  initIMU();
}

void loop() {
  Orientation o = getFilteredOrientation();
  
  // Print roll, pitch, and yaw in degrees
  Serial.print("Roll: "); 
  Serial.print(o.roll, 1);
  Serial.print(" | Pitch: "); 
  Serial.println(o.pitch, 1);

  motor1.move(100, 1);

  delay(100); // Stable reading interval
}