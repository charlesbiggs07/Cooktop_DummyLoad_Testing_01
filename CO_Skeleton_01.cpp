#include <Wire.h>

#define INA219_ADDR 0x40  // Default I2C address for INA219
#define SHUTDOWN_PIN 2
#define RELAY_PIN 5       // Controls dummy load
#define FAN_PIN 6         // Fan control
#define THERMISTOR_PIN A1
#define SERIAL_DELAY 250
#define COOLING_TEMP_THRESHOLD 85.0  // Fan stays on until Tj ≤ 85°F

volatile bool shutdownTriggered = false;

void shutdownISR() {
    shutdownTriggered = true;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    pinMode(SHUTDOWN_PIN, INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // Dummy load ON
    digitalWrite(FAN_PIN, LOW);    // Fan OFF initially

    attachInterrupt(digitalPinToInterrupt(SHUTDOWN_PIN), shutdownISR, FALLING);

    // INA219 Calibration (Configure for 32V, 2A max)
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(0x00); // Configuration Register
    Wire.write(0x39); // MSB: Set to 32V range, 10-bit ADC
    Wire.write(0x9F); // LSB: Set averaging mode
    Wire.endTransmission();
}

void loop() {
    float shuntVoltage_mV = readShuntVoltage_mV();
    float busVoltage_V = readBusVoltage_V();
    float current_mA = readCurrent_mA();
    float mosfetTemp_F = readThermistorTemp_F();

    // **Continue logging even in shutdown mode**
    Serial.print("CURRENT:");
    Serial.print(current_mA / 1000.0, 2);
    Serial.print(",SHUNT_VOLTAGE:");
    Serial.print(shuntVoltage_mV, 2);
    Serial.print(",BUS_VOLTAGE:");
    Serial.print(busVoltage_V, 2);
    Serial.print(",MOSFET_TEMP:");
    Serial.println(mosfetTemp_F, 1);

    if (shutdownTriggered) {
        Serial.println("CO: SHUTDOWN");
        digitalWrite(RELAY_PIN, LOW);  // Disconnect dummy load
        digitalWrite(FAN_PIN, HIGH);   // Ensure fan is ON

        // Continue logging after shutdown
        Serial.println("CO: Monitoring MOSFET Temperature...");

        while (readThermistorTemp_F() > COOLING_TEMP_THRESHOLD) {
            Serial.print("CO: Cooling... Tj = ");
            Serial.println(readThermistorTemp_F(), 1);
            delay(2000);
        }

        Serial.println("CO: Temperature Safe. Fan OFF. Restarting...");
        digitalWrite(FAN_PIN, LOW);    
        digitalWrite(RELAY_PIN, HIGH);
        shutdownTriggered = false;
    }

    delay(SERIAL_DELAY);
}
