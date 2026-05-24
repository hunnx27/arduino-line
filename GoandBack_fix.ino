#include "Controller.h"

Controller ctrlr;

void setup() {
    Serial.begin(9600);
    ctrlr.init();
    state = STATE_RFIDREAD;

    if (Serial) {
        Serial.println("init");
    }
}

void loop() {
    ctrlr.RunOnce();
}
