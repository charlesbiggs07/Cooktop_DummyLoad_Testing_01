#include <Wire.h>

// Define inputs
#define THERMISTOR_PIN A0    // Thermistor for plate temp
#define VGS_PIN A1           // Vgs measurement (voltage divider or opto-isolated V-F converter)
#define RELAY_PIN 5          // E-STOP relay control
#define FAN_PIN 7            // Fan control
#define PWM_VGS_PIN 9        // PWM output for Vgs
#define INA219_ADDR 0x40     // Default I2C address of INA219

bool use_INA219 = true; // Set to false to use voltage measurement instead

void setup() {
    Serial.begin(115200);
    Serial.println("System Test: Initializing...");
    
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(PWM_VGS_PIN, OUTPUT);
    
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    
    Wire.begin();
    if (!checkINA219()) {
        Serial.println("INA219 not found, switching to voltage measurement mode.");
        use_INA219 = false;
    }
    Serial.println("System Ready. Beginning tests...");
}

void loop() {
    Serial.println("---------------------------");
    Serial.println("Reading Inputs...");
    
    float plateTemp = readThermistor();
    float vgs = analogRead(VGS_PIN) * (5.0 / 1023.0) * 2; // Assuming a 10k/10k divider
    
    Serial.print("Plate Temp: "); Serial.print(plateTemp); Serial.println(" C");
    Serial.print("Vgs: "); Serial.print(vgs); Serial.println(" V");
    
    if (use_INA219) {
        float busVoltage, current_mA;
        readINA219(busVoltage, current_mA);
        Serial.print("Bus Voltage: "); Serial.print(busVoltage); Serial.println(" V");
        Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
    } else {
        float measuredVoltage = analogRead(A2) * (5.0 / 1023.0) * 2; // Assuming voltage divider on A2
        Serial.print("Measured Voltage: "); Serial.print(measuredVoltage); Serial.println(" V");
    }
    
    Serial.println("Testing Outputs...");
    
    // Toggle fan
    Serial.println("Fan ON");
    digitalWrite(FAN_PIN, HIGH);
    delay(2000);
    Serial.println("Fan OFF");
    digitalWrite(FAN_PIN, LOW);
    delay(2000);
    
    // Toggle relay
    Serial.println("Relay ON");
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);
    Serial.println("Relay OFF");
    digitalWrite(RELAY_PIN, LOW);
    delay(2000);
    
    // Sweep PWM output for Vgs
    Serial.println("Sweeping Vgs PWM...");
    for (int pwmValue = 0; pwmValue <= 127; pwmValue += 32) { // Max 127 to stay in ohmic region
        analogWrite(PWM_VGS_PIN, pwmValue);
        Serial.print("PWM: "); Serial.println(pwmValue);
        delay(1000);
    }
    analogWrite(PWM_VGS_PIN, 0);
    Serial.println("Vgs PWM Test Complete.");
    
    Serial.println("System Test Complete. Restarting in 10 seconds...");
    delay(10000);
}

bool checkINA219() {
    Wire.beginTransmission(INA219_ADDR);
    return (Wire.endTransmission() == 0);
}

void readINA219(float &busVoltage, float &current_mA) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(0x02); // Bus Voltage Register
    Wire.endTransmission();
    Wire.requestFrom(INA219_ADDR, 2);
    if (Wire.available() == 2) {
        int16_t rawBusVoltage = (Wire.read() << 8) | Wire.read();
        busVoltage = (rawBusVoltage >> 3) * 0.004;
    }
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(0x04); // Current Register
    Wire.endTransmission();
    Wire.requestFrom(INA219_ADDR, 2);
    if (Wire.available() == 2) {
        int16_t rawCurrent = (Wire.read() << 8) | Wire.read();
        current_mA = rawCurrent * 0.01; // Assuming default calibration
    }
}

float readThermistor() {
    int adcValue = analogRead(THERMISTOR_PIN);
    float voltage = adcValue * (5.0 / 1023.0);
    // Convert to temperature (this equation will need calibration for your specific thermistor)
    float resistance = (10000.0 * voltage) / (5.0 - voltage); // Assuming 10k pull-up resistor
    float temperature = 1.0 / (0.001129148 + (0.000234125 * log(resistance)) + (0.0000000876741 * pow(log(resistance), 3))) - 273.15;
    return temperature;
}
