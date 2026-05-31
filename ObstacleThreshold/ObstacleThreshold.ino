#include <Arduino.h>

const uint8_t SensorFrontCenter = A0;

const unsigned long SAMPLE_PERIOD_MS  = 5000;
const unsigned long SAMPLE_INTERVAL_MS = 20;
const unsigned long LIVE_INTERVAL_MS   = 100;

bool clearMeasured    = false;
bool obstacleMeasured = false;
int16_t clearMin = 1023, clearMax = 0;
int16_t obsMin   = 1023, obsMax   = 0;
long    clearSum = 0,    obsSum   = 0;
int     clearN   = 0,    obsN     = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}
    pinMode(SensorFrontCenter, INPUT);

    Serial.println();
    Serial.println(F("=== Obstacle threshold tuner (A0) ==="));
    Serial.print(F("Front center raw at boot: "));
    Serial.println(analogRead(SensorFrontCenter));
    printMenu();
}

void loop() {
    if (Serial.available() == 0) return;
    int cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

    switch (cmd) {
        case 'l': liveStream();   break;
        case 'n': sample(false);  break;
        case 'o': sample(true);   break;
        case 's': suggest();      break;
        case 'r': resetSamples(); break;
        case 'p': printSamples(); break;
        case '?': printMenu();    break;
        default:
            Serial.print(F("Unknown command: "));
            Serial.println((char)cmd);
            break;
    }
    flushInput();
}

void liveStream() {
    flushInput();   // drop the newline/CR left over from the 'l' command
    Serial.println(F("Live stream — press any key (then Enter) to stop"));
    while (true) {
        Serial.print(F("front center: "));
        Serial.println(analogRead(SensorFrontCenter));
        unsigned long t0 = millis();
        while (millis() - t0 < LIVE_INTERVAL_MS) {
            if (Serial.available()) {
                int c = Serial.read();
                if (c == '\n' || c == '\r' || c == ' ') continue;  // ignore line endings
                while (Serial.available()) Serial.read();
                Serial.println(F("Stopped."));
                return;
            }
        }
    }
}

void sample(bool obstaclePresent) {
    Serial.print(F("Sampling "));
    Serial.print(obstaclePresent ? F("OBSTACLE") : F("CLEAR"));
    Serial.println(F(" for 5s — starts in 2s. Hold the scene steady."));
    delay(2000);

    int16_t mn = 1023, mx = 0;
    long sum = 0;
    int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < SAMPLE_PERIOD_MS) {
        int16_t v = analogRead(SensorFrontCenter);
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        n++;
        delay(SAMPLE_INTERVAL_MS);
    }
    int avg = (n > 0) ? (int)(sum / n) : 0;

    if (obstaclePresent) {
        obsMin = mn; obsMax = mx; obsSum = sum; obsN = n;
        obstacleMeasured = true;
    } else {
        clearMin = mn; clearMax = mx; clearSum = sum; clearN = n;
        clearMeasured = true;
    }

    Serial.print(F("  min=")); Serial.print(mn);
    Serial.print(F(" max="));  Serial.print(mx);
    Serial.print(F(" avg="));  Serial.print(avg);
    Serial.print(F(" n="));    Serial.println(n);
}

void suggest() {
    if (!clearMeasured || !obstacleMeasured) {
        Serial.println(F("Run BOTH 'n' (no obstacle) and 'o' (obstacle at trigger distance) first."));
        return;
    }
    int clearAvg = clearSum / clearN;
    int obsAvg   = obsSum   / obsN;

    Serial.println(F("--- Summary ---"));
    Serial.print(F("CLEAR    min=")); Serial.print(clearMin); Serial.print(F(" max=")); Serial.print(clearMax); Serial.print(F(" avg=")); Serial.println(clearAvg);
    Serial.print(F("OBSTACLE min=")); Serial.print(obsMin);   Serial.print(F(" max=")); Serial.print(obsMax);   Serial.print(F(" avg=")); Serial.println(obsAvg);

    if (obsAvg >= clearAvg) {
        Serial.println(F("WARN: obstacle reads HIGHER than clear."));
        Serial.println(F("CheckObstacle() in Controller.cpp uses 'reading < threshold' => assumes obstacle is LOWER."));
        Serial.println(F("Check wiring / sensor type, or flip the comparison in CheckObstacle() if intentional."));
        return;
    }

    if (obsMax >= clearMin) {
        Serial.println(F("WARN: obstacle range and clear range OVERLAP — no safe threshold exists."));
        Serial.print(F("  obstacle max=")); Serial.print(obsMax);
        Serial.print(F(" >= clear min="));  Serial.println(clearMin);
        Serial.println(F("Move obstacle closer, change material/angle, or clean the sensor lens, then retry."));
        return;
    }

    int midpoint = (obsMax + clearMin) / 2;
    int margin   = (clearMin - obsMax) / 2;
    Serial.print(F("Suggested OBSTACLE_THRESHOLD = "));
    Serial.println(midpoint);
    Serial.print(F("  margin to each side: ±")); Serial.println(margin);
    Serial.println(F("Update GoandBack_fix/Controller.h:"));
    Serial.print(F("  #define OBSTACLE_THRESHOLD "));
    Serial.println(midpoint);
}

void printSamples() {
    Serial.println(F("--- Stored samples ---"));
    if (clearMeasured) {
        Serial.print(F("CLEAR    min=")); Serial.print(clearMin); Serial.print(F(" max=")); Serial.print(clearMax);
        Serial.print(F(" avg=")); Serial.println(clearSum / clearN);
    } else {
        Serial.println(F("CLEAR    (not measured)"));
    }
    if (obstacleMeasured) {
        Serial.print(F("OBSTACLE min=")); Serial.print(obsMin); Serial.print(F(" max=")); Serial.print(obsMax);
        Serial.print(F(" avg=")); Serial.println(obsSum / obsN);
    } else {
        Serial.println(F("OBSTACLE (not measured)"));
    }
}

void resetSamples() {
    clearMeasured = obstacleMeasured = false;
    clearMin = 1023; clearMax = 0; clearSum = 0; clearN = 0;
    obsMin   = 1023; obsMax   = 0; obsSum   = 0; obsN   = 0;
    Serial.println(F("Reset."));
}

void printMenu() {
    Serial.println();
    Serial.println(F("=== Menu ==="));
    Serial.println(F(" l - LIVE stream front-center raw (any key + Enter to stop)"));
    Serial.println(F(" n - sample CLEAR    state for 5s (no obstacle in front)"));
    Serial.println(F(" o - sample OBSTACLE state for 5s (obstacle at trigger distance)"));
    Serial.println(F(" s - suggest threshold from samples"));
    Serial.println(F(" p - print stored samples"));
    Serial.println(F(" r - reset samples"));
    Serial.println(F(" ? - show this menu"));
}

void flushInput() {
    delay(20);
    while (Serial.available()) Serial.read();
}
