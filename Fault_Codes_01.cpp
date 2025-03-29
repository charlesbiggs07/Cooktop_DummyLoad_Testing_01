// Fault Codes & Shutdown Procedure

#define SHUTDOWN_OVERTEMP         "Overtemperature"
#define SHUTDOWN_BUS_VOLTAGE_LOW  "Bus Voltage Low"
#define SHUTDOWN_BUS_VOLTAGE_HIGH "Bus Voltage High"
#define SHUTDOWN_VGS_HIGH         "Vgs Too High"
#define SHUTDOWN_SHUNT_FAULT      "Shunt Voltage Abnormal"
#define SHUTDOWN_THERM_FAULT      "Thermistor Fault"
#define SHUTDOWN_INA_FAIL         "INA219 Comm Fail"
#define SHUTDOWN_USER_COMMAND     "User Command"
#define SHUTDOWN_STARTUP_FAIL     "Startup Check Fail"


void shutdown(const char* reason) {
  if (shutdownState) return;  // Already shutting down
  shutdownState = true;

  Serial.print("CONTROLLED SHUTDOWN: ");
  Serial.println(reason);

  // === Step 1: Set Vgs to 0 ===
  // If CO controls PWM directly:
  analogWrite(9, 0);  // Replace '9' with your actual Vgs PWM pin

  // If Vgs Controller handles it, you'd send a command here instead

  // === Step 2: Wait for gate discharge ===
  delay(250);  // Allow gate to settle

  // === Step 3: (Optional) Confirm current has dropped ===
  float current = readCurrent();
  unsigned long t_start = millis();
  while (abs(current) > 0.05 && (millis() - t_start) < 1000) {
    current = readCurrent();
    delay(100);
  }
  if (abs(current) > 0.05) {
    Serial.println("WARNING: Current did not fall to zero before relay cutoff.");
  }

  // === Step 4: Cut the relay ===
  digitalWrite(RELAY_PIN, LOW);  // Relay OFF

  // === Step 5: Enable fan for cooldown ===
  digitalWrite(FAN_PIN, LOW);    // Fan ON (active LOW)

  // === Step 6: Signal system is no longer running ===
  digitalWrite(STATUS_PIN, LOW); // Tell Watchdog we're inactive

  Serial.println("Shutdown procedure complete. Entering cooldown mode...");
}
