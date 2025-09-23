#include <WiFi.h>
#include <WebServer.h>

// --- WiFi credentials ---
const char* ssid = "ESP32_IR_Sensor";   // SSID (WiFi Name)
const char* password = "12345678";      // WiFi Password (>= 8 chars)

// --- Web Server ---
WebServer server(80);

// --- IR Sensor Pins ---
const int irPins[3] = {36, 39, 35}; // SP=36, SN=39, ADC1_CH7=35
int rawValues[3];
float voltages[3];
float distances[3];

float voltageToDistance(float v) {
  if (v > 0.42) return 27.728 / (v - 0.42);
  else return 80; // max range
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>IR Sensor Readings</title>";
  html += "<meta http-equiv='refresh' content='1'>"; // auto refresh every second
  html += "<style>body{font-family:Arial;text-align:center;}</style></head><body>";
  html += "<h2>ESP32 IR Sensor Readings</h2>";
  
  for (int i = 0; i < 3; i++) {
    rawValues[i] = analogRead(irPins[i]);
    voltages[i] = (rawValues[i] / 4095.0) * 3.3;
    distances[i] = voltageToDistance(voltages[i]);
  }

  html += "<p><b>SP(36): </b>" + String(distances[0], 2) + " cm</p>";
  html += "<p><b>SN(39): </b>" + String(distances[1], 2) + " cm</p>";
  html += "<p><b>35: </b>" + String(distances[2], 2) + " cm</p>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);

  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);

  Serial.println("Access Point started");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Setup server routes
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
}
