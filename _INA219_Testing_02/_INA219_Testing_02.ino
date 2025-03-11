#include <Wire.h>

const int INA219_ADDR = 0x40;  // Default I2C address for INA219

const int potPin = A0;  // Potentiometer input
const int pwmPin = 9;   // PWM output to PWM-to-Voltage module
const int relayPin = 5; // Relay control pin
const int pwmMax = 127; // PWM cap (5V max)
const int pwmMin = 0;   // PWM min (0V)

// INA219 register addresses
const int REG_SHUNT_VOLTAGE = 0x01;
const int REG_BUS_VOLTAGE   = 0x02;
const int REG_CURRENT       = 0x04;
const int REG_CALIBRATION   = 0x05;

// INA219 calibration factor for 50A/75mV shunt
const int INA219_CAL = 4096;  // Adjust if needed

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Configure INA219 calibration
    writeRegister16(REG_CALIBRATION, INA219_CAL);

    pinMode(pwmPin, OUTPUT);
    pinMode(relayPin, OUTPUT);

    // Turn on the relay
    digitalWrite(relayPin, HIGH);
    Serial.println("Relay ON | Vgs Control + Direct INA219 Test Ready");
    delay(1000);
}

void loop() {
    // Read potentiometer (0-1023)
    int potValue = analogRead(potPin);

    // Map to PWM range (0 - 127 max)
    int pwmValue = map(potValue, 0, 1023, pwmMin, pwmMax);
    analogWrite(pwmPin, pwmValue);

    // Read INA219 values
    float shuntVoltage_mV = readShuntVoltage();
    float busVoltage_V = readBusVoltage();
    float current_A = (shuntVoltage_mV * 50.0) / 75.0;  // Convert shunt voltage to Amps

    // Print readings
    Serial.print("Pot: "); Serial.print(potValue);
    Serial.print(" | PWM: "); Serial.print(pwmValue);
    Serial.print(" | Vshunt_mV: "); Serial.print(shuntVoltage_mV);
    Serial.print(" | BusV: "); Serial.print(busVoltage_V);
    Serial.print(" | Current_A: "); Serial.println(current_A);

    delay(500); // Slow down updates for readability
}

// Function to read 16-bit shunt voltage register
float readShuntVoltage() {
    int16_t rawValue = readRegister16(REG_SHUNT_VOLTAGE);
    return rawValue * 0.01;  // Convert to mV (10µV per LSB)
}

// Function to read 16-bit bus voltage register
float readBusVoltage() {
    int16_t rawValue = readRegister16(REG_BUS_VOLTAGE);
    return (rawValue >> 3) * 4.0 / 1000.0;  // Convert to V (LSB = 4mV)
}

// Function to write to INA219 register
void writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    Wire.write(value >> 8);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

// Function to read 16-bit value from INA219 register
int16_t readRegister16(uint8_t reg) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(INA219_ADDR, 2);
    
    if (Wire.available() < 2) return 0;  // Prevent errors

    int16_t value = (Wire.read() << 8) | Wire.read();
    return value;
}
