// CO_Code_04 - Coordinator Arduino Code Based on CO_Attributes_01.txt

#include <Wire.h>

// === Pin Definitions ===
const int THERMISTOR_PIN = A0;
const int RELAY_PIN      = 5;   // Active HIGH
const int FAN_PIN        = 7;   // Active LOW
const int STATUS_PIN     = 3;   // Output to WD (HIGH = alive)
const int WD_INTERRUPT   = 2;   // Input from WD (interrupt)
const int VGS_PWM_PIN    = 9;   // PWM output to Vgs Controller

// === State Variables ===
volatile bool emergencyTriggered = false;
bool shutdownState = false;

// === INA219 Register Addresses ===
const byte REG_POWER = 0x03;
const byte REG_CALIBRATION = 0x05;
const byte REG_CURRENT = 0x04;
const byte INA219_ADDR      = 0x40;
const byte REG_BUS_VOLTAGE  = 0x02;
const byte REG_SHUNT_VOLTAGE = 0x01;

// === Shutdown Reason Definitions ===
#define SHUTDOWN_OVERTEMP         "Overtemperature"
#define SHUTDOWN_BUS_VOLTAGE_LOW  "Bus Voltage Low"
#define SHUTDOWN_BUS_VOLTAGE_HIGH "Bus Voltage High"
#define SHUTDOWN_VGS_HIGH         "Vgs Too High"
#define SHUTDOWN_SHUNT_FAULT      "Shunt Voltage Abnormal"
#define SHUTDOWN_THERM_FAULT      "Thermistor Fault"
#define SHUTDOWN_INA_FAIL         "INA219 Comm Fail"
#define SHUTDOWN_USER_COMMAND     "User Command"
#define SHUTDOWN_STARTUP_FAIL     "Startup Check Fail"

// === Function Prototypes ===
float readBusVoltage();
float readShuntVoltage();
float readCurrent();
float readPower();
float readTemperatureF();
void shutdown(const char* reason);

void writeCalibration() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_CALIBRATION);
  Wire.write(18204 >> 8);          // High byte
  Wire.write(18204 & 0xFF);        // Low byte
  Wire.endTransmission();
}

void setup() {
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(STATUS_PIN, OUTPUT);
  pinMode(WD_INTERRUPT, INPUT);
  pinMode(VGS_PWM_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(STATUS_PIN, LOW);

  Serial.begin(9600);
  delay(500);
  Wire.begin();

  attachInterrupt(digitalPinToInterrupt(WD_INTERRUPT), [](){ emergencyTriggered = true; }, FALLING);

  digitalWrite(FAN_PIN, LOW);
  delay(5000);
  float tempCheck = readTemperatureF();
  if (tempCheck > 150.0) shutdown(SHUTDOWN_OVERTEMP);

  digitalWrite(RELAY_PIN, HIGH);
  delay(500);
  float vbus = readBusVoltage();
  if (vbus < 10.0) shutdown(SHUTDOWN_BUS_VOLTAGE_LOW);
  if (vbus > 15.0) shutdown(SHUTDOWN_BUS_VOLTAGE_HIGH);

  float vshunt = readShuntVoltage();
  if (vshunt < -0.1 || vshunt > 0.1) shutdown(SHUTDOWN_SHUNT_FAULT);

  float therm = readTemperatureF();
  if (therm < 32.0 || therm > 180.0) shutdown(SHUTDOWN_THERM_FAULT);

  writeCalibration();
  digitalWrite(STATUS_PIN, HIGH);
}

void loop() {
  if (emergencyTriggered) shutdown("Emergency Interrupt");

  float tempF = readTemperatureF();
  float vgs = 4.25;
  float shunt_mV = readShuntVoltage() * 1000.0;
  float current = readCurrent();
  int pwmVal = 127;

  Serial.print(vgs); Serial.print(",");
  Serial.print(current); Serial.print(",");
  Serial.print(shunt_mV); Serial.print(",");
  Serial.print(tempF); Serial.print(",");
  Serial.print(readPower()); Serial.print(",");
  Serial.println(pwmVal);

  delay(500);

  while (shutdownState) {
    float cooldownTemp = readTemperatureF();
    Serial.print("Cooldown Temp: "); Serial.println(cooldownTemp);
    if (cooldownTemp < 85.0) {
      Serial.println("Cooldown complete.");
      shutdownState = false;
      digitalWrite(STATUS_PIN, HIGH);
      break;
    }
    delay(1000);
  }
}

void shutdown(const char* reason) {
  if (shutdownState) return;
  shutdownState = true;

  Serial.print("CONTROLLED SHUTDOWN: ");
  Serial.println(reason);

  analogWrite(VGS_PWM_PIN, 0);
  delay(250);

  float current = readCurrent();
  unsigned long t_start = millis();
  while (abs(current) > 0.05 && (millis() - t_start) < 1000) {
    current = readCurrent();
    delay(100);
  }
  if (abs(current) > 0.05) {
    Serial.println("WARNING: Current did not fall to zero before relay cutoff.");
  }

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(STATUS_PIN, LOW);
  Serial.println("Shutdown procedure complete. Entering cooldown mode...");
}

float readBusVoltage() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_BUS_VOLTAGE);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t raw = Wire.read() << 8 | Wire.read();
    return (raw >> 3) * 4.0 / 1000.0;
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
    return raw * 0.01;
  }
  return 0.0;
}

float readCurrent() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_CURRENT);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    int16_t raw = Wire.read() << 8 | Wire.read();
    return raw * 0.0015;
  }
  return 0.0;
}

float readPower() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(REG_POWER);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t raw = Wire.read() << 8 | Wire.read();
    return raw * 0.03;
  }
  return 0.0;
}

float readTemperatureF() {
  int raw = analogRead(THERMISTOR_PIN);
  float voltage = raw * (5.0 / 1023.0);
  float resistance = (5.0 - voltage) * 10000.0 / voltage;
  float steinhart;
  steinhart = resistance / 10000.0;
  steinhart = log(steinhart);
  steinhart = 1.0 / (0.001129148 + 0.000234125 * steinhart + 0.0000000876741 * steinhart * steinhart * steinhart);
  return steinhart - 273.15 * 9.0 / 5.0 + 32.0;
}
