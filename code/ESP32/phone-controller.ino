#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32";
const char* password = "01234567";

HardwareSerial mySerial(2);

WebServer server(80);

const int ledPin1 = 1;
const int ledPin2 = 2;

void webHost() {
  String html = "<html><head><meta charset='UTF-8' name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body { font-family: sans-serif; text-align: center; } ";
  html += "button { width: 80%; height: 60px; font-size: 20px; margin: 10px; cursor: pointer; border-radius: 10px; border: none; color: white; }</style>";
  html += "</head><body>";
  html += "<h1>Sterowanie zydsonem</h1>";
  html += "<p><a href='/przod'><button style='background-color: #2ecc71;'>Do przodu</button></a></p>";
  html += "<p><a href='/stop'><button style='background-color: #e74c3c;'>Stop</button></a></p>";
  html += "<p><a href='/lewo'><button style='background-color: #2ecc71;'>Lewo</button></a></p>";
  html += "<p><a href='/prawo'><button style='background-color: #e74c3c;'>Prawo</button></a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  Serial.print("Konfigurowanie sieci AP... ");
  WiFi.softAP(ssid, password);

  // Pobranie adresu IP (domyślnie 192.168.4.1)
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Gotowe!");
  Serial.print("Nazwa sieci (SSID): ");
  Serial.println(ssid);
  Serial.print("Adres IP serwera: ");
  Serial.println(IP);

  // Obsługa ścieżek
  server.on("/", webHost);
  server.on("/przod", []() {
    digitalWrite(ledPin1, HIGH);
    mySerial.print('0');
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/stop", []() {
    digitalWrite(ledPin1, LOW);
    mySerial.print('3');
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/lewo", []() {
    digitalWrite(ledPin2, HIGH);
    mySerial.print('2');
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/prawo", []() {
    digitalWrite(ledPin2, LOW);
    mySerial.print('1');
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
}

void loop() {
  server.handleClient();
}