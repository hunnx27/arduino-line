#include "Controller.h"

Controller ctrlr;

void setup() {
    Serial.begin(9600);
    ctrlr.init();

    if (Serial) {
        Serial.println("init");
    }
}

void loop() {
    ctrlr.RunOnce();
}
