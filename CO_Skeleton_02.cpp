// CO_Sketch_01 - Coordinator Arduino Code Based on CO_Attributes_01.txt

#include <Wire.h>

// === Pin Definitions ===
const int THERMISTOR_PIN = A0;
const int RELAY_PIN      = 5;   // Active HIGH
const int FAN_PIN        = 7;   // Active LOW
const int STATUS_PIN     = 3;   // Output to WD (HIGH = alive)
const int WD_INTERRUPT   = 2;   // Input from WD (interrupt)

// === State Variables ===
volatile bool emergencyTriggered = false;
bool shutdownState = false;

// === INA219 Register Addresses ===
const byte INA219_ADDR      = 0x40;
const byte REG_BUS_VOLTAGE  = 0x02;
const byte REG_SHUNT_VOLTAGE = 0x01;

// === Function Prototypes ===
float readBusVoltage();
float readShuntVoltage();
float readCurrent();
float readTemperatureF();
void emergencyShutdown();

void setup() {
  // === Pin Setup ===
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(STATUS_PIN, OUTPUT);
  pinMode(WD_INTERRUPT, INPUT);

  digitalWrite(RELAY_PIN, LOW);   // Relay OFF
  digitalWrite(FAN_PIN, HIGH);    // Fan OFF (active LOW)
  digitalWrite(STATUS_PIN, LOW);  // Not ready yet

  // === Serial & I2C ===
  Serial.begin(9600);
  delay(500);
  Wire.begin();

  // === Attach Interrupt ===
  attachInterrupt(digitalPinToInterrupt(WD_INTERRUPT), emergencyShutdown, FALLING);

  // === Safety Checks ===
  digitalWrite(FAN_PIN, LOW);  // Turn ON fan
  delay(5000);
  float tempCheck = readTemperatureF();
  if (tempCheck > 150.0) emergencyShutdown();

  digitalWrite(RELAY_PIN, HIGH);  // Turn ON relay (no load expected)
  delay(500);
  float vbus = readBusVoltage();
  if (vbus < 10.0 || vbus > 15.0) emergencyShutdown();

  float vshunt = readShuntVoltage();
  if (vshunt < -0.1 || vshunt > 0.1) emergencyShutdown();

  float therm = readTemperatureF();
  if (therm < 32.0 || therm > 180.0) emergencyShutdown();

  // === Signal Ready ===
  digitalWrite(STATUS_PIN, HIGH);
}

void loop() {
  if (emergencyTriggered) emergencyShutdown();

  float tempF = readTemperatureF();
  float vgs = 4.25;       // Placeholder: replace with actual Vgs read logic
  float shunt_mV = readShuntVoltage() * 1000.0;
  float current = readCurrent();  // Placeholder
  int pwmVal = 127;       // Placeholder: replace with actual PWM tracking

  Serial.print("VGS:"); Serial.print(vgs);
  Serial.print(" CURRENT:"); Serial.print(current);
  Serial.print(" SHUNT_mV:"); Serial.print(shunt_mV);
  Serial.print(" TEMP_F:"); Serial.print(tempF);
  Serial.print(" PWM:"); Serial.println(pwmVal);

  delay(500); // Adjust to suit data logging rate
}

void emergencyShutdown() {
  if (shutdownState) return;
  shutdownState = true;
  digitalWrite(FAN_PIN, LOW);   // Fan ON
  digitalWrite(RELAY_PIN, LOW); // Relay OFF
  Serial.println("ALL STOP");
}

float readBusVoltage() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_BUS_VOLTAGE);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t raw = Wire.read() << 8 | Wire.read();
    return (raw >> 3) * 4.0 / 1000.0; // volts
  }
  return -1.0;
}

float readShuntVoltage() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_SHUNT_VOLTAGE);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    int16_t raw = Wire.read() << 8 | Wire.read();
    return raw * 0.01; // mV scale factor (0.01mV per LSB)
  }
  return 0.0;
}

float readCurrent() {
  // Use proper calibration math
  // Shunt resistor = 0.0015 ohms
  // Current_LSB = 1.5mA = 0.0015
  // Calibration register value = 18204 (not needed in this sketch unless writing to the INA219)

  float vShunt_mV = readShuntVoltage(); // in mV
  float vShunt_V = vShunt_mV / 1000.0;
  float current_A = vShunt_V / 0.0015;  // Ohm’s law: I = V / R
  return current_A;
}


float readTemperatureF() {
  int raw = analogRead(THERMISTOR_PIN);
  float voltage = raw * (5.0 / 1023.0);
  float resistance = (5.0 - voltage) * 10000.0 / voltage; // Assuming 10k pull-up
  float steinhart;
  steinhart = resistance / 10000.0;
  steinhart = log(steinhart);
  steinhart = 1.0 / (0.001129148 + 0.000234125 * steinhart + 0.0000000876741 * steinhart * steinhart * steinhart);
  return steinhart - 273.15 * 9.0 / 5.0 + 32.0;
}

