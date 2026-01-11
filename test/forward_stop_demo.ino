#include <Arduino.h>

// 핀 맵은 기존 스케치(main/test)와 동일하게 사용
const int L_BK_PIN  = 6;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 4;
const int R_BK_PIN  = 13;
const int R_PWM_PIN = 12;
const int R_DIR_PIN = 11;

const int PWM_FORWARD = 175;               // 전진 PWM
const unsigned long RUN_DURATION_MS = 5000; // 5초 전진

bool forwardActive = false;
unsigned long forwardStartMs = 0;

void stopMotors(bool announce) {
    forwardActive = false;
    digitalWrite(L_BK_PIN, HIGH);
    digitalWrite(R_BK_PIN, HIGH);
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
    if (announce) Serial.println("Motors stopped.");
}

void startForward() {
    forwardActive = true;
    forwardStartMs = millis();
    digitalWrite(L_BK_PIN, LOW);
    digitalWrite(R_BK_PIN, LOW);
    digitalWrite(L_DIR_PIN, HIGH);
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, PWM_FORWARD);
    analogWrite(R_PWM_PIN, PWM_FORWARD);
    Serial.println("Forward started for 5s. ('S' to stop, 'F' to resume)");
}

void emergencyStop() {
    stopMotors(false);
    Serial.println("Emergency stop by command 'S'.");
}

void setup() {
    Serial.begin(115200);

    pinMode(L_BK_PIN, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_BK_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);

    stopMotors(false); // 초기 정지
    startForward();    // 부팅 후 즉시 5초 전진
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'S' || cmd == 's') {
            emergencyStop();
        } else if (cmd == 'F' || cmd == 'f') {
            startForward();
        }
    }

    // 5초 경과 시 자동 정지
    if (forwardActive && (millis() - forwardStartMs >= RUN_DURATION_MS)) {
        stopMotors(true);
    }
}
