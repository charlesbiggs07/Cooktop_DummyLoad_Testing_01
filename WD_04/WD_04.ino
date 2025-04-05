// WD_04 with Coordinator Ready Handshake and Serial Monitoring
//#include <Wire.h>

// --- Pin Definitions ---
const int FAN_PIN   = 7;     // Active-LOW
const int RELAY_PIN = 5;     // Active-HIGH (via transistor)
const int INT_PIN   = 2;     // External interrupt from CO RESET
const int CO_READY_PIN = 6;  // Input from CO D6 (goes HIGH when CO is ready)
const int ALERT_PIN = 4;     // Output to CO D2 (emergency flag)

volatile bool emergencyTriggered = false;

// --- Thresholds ---
const int TEMP_THRESHOLD = 600;       // Raw ADC value, or convert later
const float CURRENT_THRESHOLD = 5000; // mA
const float VGS_THRESHOLD = 5.5;      // V (future use)

String inputLine = "";

void setup() {
  Serial.begin(9600);
  delay(10);
  Serial.println("WD_04.ino");
  delay(1500);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  pinMode(CO_READY_PIN, INPUT);
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);       // Default: no alert

  digitalWrite(FAN_PIN, HIGH);        // Fan OFF (active-low)
  digitalWrite(RELAY_PIN, HIGH);      // Relay ON (active-high)

  attachInterrupt(digitalPinToInterrupt(INT_PIN), emergencyISR, FALLING);

  Serial.println("WD booted. Waiting for Coordinator ready signal...");

  while (digitalRead(CO_READY_PIN) == LOW) {
    delay(10);
  }

  Serial.println("Coordinator ready. Monitoring RESET line and serial stream...");
}

void loop() {
  if (emergencyTriggered) {
    Serial.println("!! Emergency shutdown triggered by Coordinator RESET !!");
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    while (1);
  }

  // --- Serial Monitoring from CO ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseLine(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }

  delay(10);
}

void emergencyISR() {
  emergencyTriggered = true;
}

void parseLine(String line) {
  // Example input: "PWM: 35 | Vbus: 12.3 V | Vshunt: 66.5 mV | Current: 6650.0 mA | TempRaw: 543"
  int temp = extractIntAfter(line, "TempRaw: ");
  float current = extractFloatAfter(line, "Current: ");

  if (temp > TEMP_THRESHOLD || current > CURRENT_THRESHOLD) {
    Serial.println("** Parameter out of range — triggering alert to CO **");
    digitalWrite(ALERT_PIN, HIGH);
  } else {
    digitalWrite(ALERT_PIN, LOW);
  }
}

int extractIntAfter(String src, String key) {
  int idx = src.indexOf(key);
  if (idx == -1) return 0;
  int end = src.indexOf('|', idx);
  String val = src.substring(idx + key.length(), end == -1 ? src.length() : end);
  return val.toInt();
}

float extractFloatAfter(String src, String key) {
  int idx = src.indexOf(key);
  if (idx == -1) return 0.0;
  int end = src.indexOf('|', idx);
  String val = src.substring(idx + key.length(), end == -1 ? src.length() : end);
  return val.toFloat();
}
