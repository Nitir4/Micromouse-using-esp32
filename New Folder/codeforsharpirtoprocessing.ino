#include <WiFi.h>
#include <WebServer.h>

// IR sensor pins
const int irPins[3] = {35, 39, 36};  // Right, Front, Left
int rawValues[3];
float voltages[3];
float distances[3];

// WiFi credentials (Access Point mode)
const char* ssid = "ESP32_IR_Server";
const char* password = "12345678";  // at least 8 chars

WebServer server(80);

float voltageToDistance(float v) {
  if (v > 0.42) return 27.728 / (v - 0.42);
  else return 80;  // max distance
}

// HTML page
String getHTMLPage() {
  String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='1'/>"; 
  html += "<title>IR Sensor Readings</title></head><body>";
  html += "<h2>Sharp IR GP2Y0A21 Readings</h2>";
  html += "<p><b>Right (GPIO 35): </b>" + String(distances[0], 2) + " cm</p>";
  html += "<p><b>Front (GPIO 39): </b>" + String(distances[1], 2) + " cm</p>";
  html += "<p><b>Left (GPIO 36): </b>" + String(distances[2], 2) + " cm</p>";
  html += "<p><i>Also available as JSON at <a href='/data'>/data</a></i></p>";
  html += "</body></html>";
  return html;
}

// JSON data endpoint
String getJSON() {
  String json = "{";
  json += "\"right\":" + String(distances[0], 2) + ",";
  json += "\"front\":" + String(distances[1], 2) + ",";
  json += "\"left\":" + String(distances[2], 2);
  json += "}";
  return json;
}

void handleRoot() {
  server.send(200, "text/html", getHTMLPage());
}

void handleData() {
  server.send(200, "application/json", getJSON());
}

void setup() {
  Serial.begin(115200);

  // Start WiFi in Access Point mode
  WiFi.softAP(ssid, password);
  Serial.println("WiFi Access Point started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Setup server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Read sensor values
  for (int i = 0; i < 3; i++) {
    rawValues[i] = analogRead(irPins[i]);
    voltages[i] = (rawValues[i] / 4095.0) * 3.3;
    distances[i] = voltageToDistance(voltages[i]);
  }

  server.handleClient(); // Handle web requests
}
