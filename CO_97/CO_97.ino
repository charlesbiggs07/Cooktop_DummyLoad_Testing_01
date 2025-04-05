// Refactored CO_98.ino   NOW CO_97 with Watchdog interrupt handling
#include <Wire.h>

#define VGS_CONTROLLER_ADDR 0x08
#define INA219_ADDR 0x40

const int relayPin = 5;    // Active-HIGH
const int fanPin = 7;      // Active-LOW
const int tempPin = A0;
const int wdAlertPin = 2;  // Input from WD D4
const int coReadyPin = 6;  // Output to WD D6

volatile bool emergencyShutdown = false;

int pwmValue = 0;
const int pwmMax = 127;
const int pwmStep = 5;
const int delayMs = 1000;
const int shutdownCurrent_mA = 5000;

bool testRunning = false;
unsigned long lastBeat = 0;

void setup() {
  Wire.begin();           
  Serial.begin(9600);
  delay(1000);  // Let Serial settle
  Serial.println("CO_97.ino");

  pinMode(relayPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(wdAlertPin, INPUT);
  pinMode(coReadyPin, OUTPUT);

  digitalWrite(relayPin, LOW);  // Load OFF
  digitalWrite(fanPin, HIGH);   // Fan OFF
  digitalWrite(coReadyPin, LOW); // Default: not ready

  sendVgsPWM(0);  // Vgs = 0

  attachInterrupt(digitalPinToInterrupt(wdAlertPin), shutdownISR, RISING);

  Serial.println("System initialized.");
  Serial.println("Type 'start' to begin or 'stop' to cancel test.");

  delay(200);  // Let WD boot
  digitalWrite(coReadyPin, HIGH); // Signal ready to WD
}

void loop() {
  handleSerialCommands();

  if (!testRunning) {
    return;
  }

  // --- Emergency Check ---
  if (emergencyShutdown) {
    Serial.println("!! Emergency flag received from Watchdog !!");
    sendVgsPWM(0);
    digitalWrite(relayPin, LOW);  // Load OFF
    digitalWrite(fanPin, LOW);    // Fan ON
    // Continue logging
  }

  // --- Vgs Control ---
  sendVgsPWM(pwmValue);
  delay(50);  // Settle time

  // --- Sensor Reads ---
  float bus_V = readBusVoltage_V();
  float shunt_mV = readShuntVoltage_mV();
  float current_mA = shunt_mV / 0.001;
  int tempRaw = analogRead(tempPin);

  // --- Serial Output ---
  Serial.print("PWM: "); Serial.print(pwmValue);
  Serial.print(" | Vbus: "); Serial.print(bus_V, 2);
  Serial.print(" V | Vshunt: "); Serial.print(shunt_mV, 1);
  Serial.print(" mV | Current: "); Serial.print(current_mA, 1);
  Serial.print(" mA | TempRaw: "); Serial.println(tempRaw);

  // --- Auto-Shutdown (Legacy) ---
  if (current_mA > shutdownCurrent_mA && !emergencyShutdown) {
    Serial.println("!! Overcurrent detected — shutting down...");
    sendVgsPWM(0);
    digitalWrite(relayPin, LOW);  // Load OFF
    digitalWrite(fanPin, LOW);    // Fan ON
    testRunning = false;
    pwmValue = 0;

    Serial.println("Fan will run for 60 seconds...");
    delay(60000);
    digitalWrite(fanPin, HIGH);   // Fan OFF

    Serial.println("Test halted. Type 'start' to retry.");
    return;
  }

  // --- PWM Step ---
  pwmValue += pwmStep;
  if (pwmValue > pwmMax) {
    pwmValue = pwmMax;
  }

  delay(delayMs);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("start")) {
      if (!testRunning) {
        Serial.println("Starting ramp test...");
        pwmValue = 0;
        sendVgsPWM(0);
        digitalWrite(relayPin, HIGH);  // Load ON
        delay(200);
        testRunning = true;
      } else {
        Serial.println("Test already running.");
      }
    }

    if (input.equalsIgnoreCase("stop")) {
      if (testRunning) {
        Serial.println("Manual stop issued.");
        sendVgsPWM(0);
        digitalWrite(relayPin, LOW);  // Load OFF
        digitalWrite(fanPin, LOW);    // Fan ON
        testRunning = false;
        pwmValue = 0;

        Serial.println("Fan will run for 60 seconds...");
        delay(60000);
        digitalWrite(fanPin, HIGH);   // Fan OFF

        Serial.println("Test halted. Type 'start' to retry.");
      } else {
        Serial.println("No test is running.");
      }
    }
  }
}

void sendVgsPWM(uint8_t value) {
  Wire.beginTransmission(VGS_CONTROLLER_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}

float readBusVoltage_V() {
  uint16_t raw = readRegister16(0x02);
  return (raw >> 3) * 0.004;  // 4mV/bit
}

float readShuntVoltage_mV() {
  int16_t raw = (int16_t)readRegister16(0x01);
  return raw * 0.01;  // 10uV/bit
}

uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(INA219_ADDR, 2);
  return (Wire.read() << 8) | Wire.read();
}

void shutdownISR() {
  emergencyShutdown = true;
}