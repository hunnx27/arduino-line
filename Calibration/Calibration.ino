#include <Arduino.h>
#include <EEPROM.h>

#define START_ADDRESS 240

const uint8_t LeftWheelPWM      = 5;
const uint8_t RightWheelPWM     = 6;
const uint8_t LeftWheelDir      = 7;
const uint8_t RightWheelDir     = 8;
const uint8_t SensorBottomLeft  = A6;
const uint8_t SensorBottomRight = A7;

const int TEST_POWER   = 120;
const int TEST_TIME_MS = 2000;
const int SAMPLE_COUNT = 32;
const int SAMPLE_DELAY = 10;

int16_t rightWhite = 0;
int16_t leftWhite  = 0;
int16_t rightBlack = 1023;
int16_t leftBlack  = 1023;
float   motorCalibR = 1.0f;
float   motorCalibL = 1.0f;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    pinMode(LeftWheelPWM,  OUTPUT);
    pinMode(RightWheelPWM, OUTPUT);
    pinMode(LeftWheelDir,  OUTPUT);
    pinMode(RightWheelDir, OUTPUT);
    pinMode(SensorBottomLeft,  INPUT);
    pinMode(SensorBottomRight, INPUT);

    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);

    loadFromEEPROM();
    Serial.println();
    Serial.println(F("=== Calibration sketch ==="));
    printValues();
    printMenu();
}

void loop() {
    if (Serial.available() == 0) return;

    int cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

    switch (cmd) {
        case 'w': sampleSurface(true);  break;
        case 'b': sampleSurface(false); break;
        case 't': driveBoth();          break;
        case 'L': driveOne(true);       break;
        case 'R': driveOne(false);      break;
        case 'l': promptCalib(true);    break;
        case 'r': promptCalib(false);   break;
        case 'p': printValues();        break;
        case 'd': resetDefaults();      break;
        case 's': saveToEEPROM();       break;
        case '?': printMenu();          break;
        default:
            Serial.print(F("Unknown command: "));
            Serial.println((char)cmd);
            break;
    }
    flushInput();
}

void sampleSurface(bool isWhite) {
    Serial.print(F("Sampling "));
    Serial.print(isWhite ? F("WHITE") : F("BLACK"));
    Serial.println(F(" surface in 1s ..."));
    delay(1000);

    long lSum = 0, rSum = 0;
    int16_t lMin = 1023, lMax = 0, rMin = 1023, rMax = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int16_t l = analogRead(SensorBottomLeft);
        int16_t r = analogRead(SensorBottomRight);
        lSum += l;
        rSum += r;
        if (l < lMin) lMin = l;
        if (l > lMax) lMax = l;
        if (r < rMin) rMin = r;
        if (r > rMax) rMax = r;
        delay(SAMPLE_DELAY);
    }
    int16_t lAvg = lSum / SAMPLE_COUNT;
    int16_t rAvg = rSum / SAMPLE_COUNT;

    if (isWhite) { leftWhite = lAvg; rightWhite = rAvg; }
    else         { leftBlack = lAvg; rightBlack = rAvg; }

    Serial.print(F("  L avg=")); Serial.print(lAvg);
    Serial.print(F(" (min=")); Serial.print(lMin); Serial.print(F(", max=")); Serial.print(lMax); Serial.println(F(")"));
    Serial.print(F("  R avg=")); Serial.print(rAvg);
    Serial.print(F(" (min=")); Serial.print(rMin); Serial.print(F(", max=")); Serial.print(rMax); Serial.println(F(")"));
    Serial.println(F("  (in RAM only — press 's' to write EEPROM)"));
}

void driveBoth() {
    Serial.println(F("Both motors forward — lift wheels off ground first!"));
    delay(1500);
    digitalWrite(LeftWheelDir,  HIGH);
    digitalWrite(RightWheelDir, LOW);
    analogWrite(LeftWheelPWM,  (int)(TEST_POWER * motorCalibL));
    analogWrite(RightWheelPWM, (int)(TEST_POWER * motorCalibR));
    delay(TEST_TIME_MS);
    analogWrite(LeftWheelPWM,  0);
    analogWrite(RightWheelPWM, 0);
    Serial.println(F("Done."));
}

void driveOne(bool left) {
    Serial.print(F("Driving "));
    Serial.print(left ? F("LEFT") : F("RIGHT"));
    Serial.println(F(" motor only — lift wheels!"));
    delay(1500);
    if (left) {
        digitalWrite(LeftWheelDir, HIGH);
        analogWrite(LeftWheelPWM, (int)(TEST_POWER * motorCalibL));
        delay(TEST_TIME_MS);
        analogWrite(LeftWheelPWM, 0);
    } else {
        digitalWrite(RightWheelDir, LOW);
        analogWrite(RightWheelPWM, (int)(TEST_POWER * motorCalibR));
        delay(TEST_TIME_MS);
        analogWrite(RightWheelPWM, 0);
    }
    Serial.println(F("Done."));
}

void promptCalib(bool left) {
    Serial.print(F("Enter new "));
    Serial.print(left ? F("LEFT") : F("RIGHT"));
    Serial.println(F(" motor calib (e.g. 1.00) within 10s:"));
    unsigned long start = millis();
    while (Serial.available() == 0) {
        if (millis() - start > 10000) {
            Serial.println(F("Timeout — no change."));
            return;
        }
    }
    float v = Serial.parseFloat();
    if (v <= 0.0f || v > 3.0f) {
        Serial.print(F("Out of range (0,3]: "));
        Serial.println(v, 3);
        return;
    }
    if (left) motorCalibL = v;
    else      motorCalibR = v;
    Serial.print(left ? F("motorCalibL=") : F("motorCalibR="));
    Serial.println(v, 3);
    Serial.println(F("  (in RAM only — press 's' to write EEPROM)"));
}

void resetDefaults() {
    rightWhite = leftWhite = 0;
    rightBlack = leftBlack = 1023;
    motorCalibR = motorCalibL = 1.0f;
    Serial.println(F("Reset to defaults (sensors 0/1023, motors 1.00)."));
    Serial.println(F("  (in RAM only — press 's' to write EEPROM)"));
}

void saveToEEPROM() {
    if (leftBlack <= leftWhite || rightBlack <= rightWhite) {
        Serial.println(F("REFUSED: black sample is not greater than white sample."));
        Serial.println(F("Re-run 'w' and 'b' on the correct surfaces."));
        printValues();
        return;
    }
    int addr = START_ADDRESS;
    EEPROM.put(addr, rightWhite);  addr += 2;
    EEPROM.put(addr, leftWhite);   addr += 2;
    EEPROM.put(addr, rightBlack);  addr += 2;
    EEPROM.put(addr, leftBlack);   addr += 2;
    EEPROM.put(addr, motorCalibR); addr += 4;
    EEPROM.put(addr, motorCalibL); addr += 4;
    Serial.println(F("Saved to EEPROM."));
    loadFromEEPROM();
    printValues();
}

void loadFromEEPROM() {
    int addr = START_ADDRESS;
    EEPROM.get(addr, rightWhite);  addr += 2;
    EEPROM.get(addr, leftWhite);   addr += 2;
    EEPROM.get(addr, rightBlack);  addr += 2;
    EEPROM.get(addr, leftBlack);   addr += 2;
    EEPROM.get(addr, motorCalibR); addr += 4;
    EEPROM.get(addr, motorCalibL); addr += 4;
}

void printValues() {
    Serial.println(F("--- Current values (RAM) ---"));
    Serial.print(F("  leftWhite="));   Serial.print(leftWhite);
    Serial.print(F("  rightWhite="));  Serial.println(rightWhite);
    Serial.print(F("  leftBlack="));   Serial.print(leftBlack);
    Serial.print(F("  rightBlack="));  Serial.println(rightBlack);
    Serial.print(F("  motorCalibL=")); Serial.print(motorCalibL, 3);
    Serial.print(F("  motorCalibR=")); Serial.println(motorCalibR, 3);
    Serial.print(F("  live sensor: L=")); Serial.print(analogRead(SensorBottomLeft));
    Serial.print(F(" R="));               Serial.println(analogRead(SensorBottomRight));
}

void printMenu() {
    Serial.println();
    Serial.println(F("=== Menu ==="));
    Serial.println(F(" w - sample WHITE (place both bottom sensors on white)"));
    Serial.println(F(" b - sample BLACK (place both bottom sensors on black line)"));
    Serial.println(F(" t - drive BOTH motors forward 2s (lift wheels!)"));
    Serial.println(F(" L - drive LEFT  motor only 2s (lift wheels!)"));
    Serial.println(F(" R - drive RIGHT motor only 2s (lift wheels!)"));
    Serial.println(F(" l - set LEFT  motor calib (float, e.g. 1.00)"));
    Serial.println(F(" r - set RIGHT motor calib (float, e.g. 1.00)"));
    Serial.println(F(" p - print current values + live sensor"));
    Serial.println(F(" d - reset RAM to defaults"));
    Serial.println(F(" s - SAVE RAM values to EEPROM"));
    Serial.println(F(" ? - show this menu"));
}

void flushInput() {
    delay(20);
    while (Serial.available()) Serial.read();
}
