const int pwmPin = 9;  // PWM output pin
const int maxPWM = 127; // Maximum PWM value
const int stepDelay = 50; // Delay in milliseconds between steps

void setup() {
    pinMode(pwmPin, OUTPUT);
}

void loop() {
    for (int pwmValue = 0; pwmValue <= maxPWM; pwmValue++) {
        analogWrite(pwmPin, pwmValue);
        delay(stepDelay);
    }
    delay(2500);
}
