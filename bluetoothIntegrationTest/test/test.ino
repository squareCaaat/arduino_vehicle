#include <Adafruit_PWMServoDriver.h>

// SERVO PWM 범위
const int SERVO_PWM_MIN = 150;
const int SERVO_PWM_MAX = 600;

// 0x40
const int STEER_SERVO_CH = 15;

// 0x41
const int BOTTOM_CH   = 14;
const int LINK_TWO_CH = 15;

// 0x42
const int LINK_ONE_CH = 14;
const int GRIPPER_CH  = 15;

// ARM 기본값
const int BOTTOM_DEFAULT_ANGLE   = 0;    // 하부 회전
const int LINK_ONE_DEFAULT_ANGLE = 170;  // 링크 1 틸트
const int LINK_TWO_DEFAULT_ANGLE = 0;    // 링크 2 틸트
const int GRIPPER_DEFAULT_ANGLE  = 80;   // 그리퍼
const int ARM_STEP_DEG           = 3;    // 각도 1회 증감량
const int ARM_STEP_TIME          = 10;   // 각도 1회 증감 딜레이 (ms)

// ARM 최소, 최대값
const int BOTTOM_MIN_ANGLE   = 0;    // 하부 최소 각도
const int BOTTOM_MAX_ANGLE   = 175;  // 하부 최대 각도
const int LINK_ONE_MIN_ANGLE = 10;   // 링크 1 최소 각도
const int LINK_ONE_MAX_ANGLE = 175;  // 링크 1 최대 각도
const int LINK_TWO_MIN_ANGLE = 10;   // 링크 2 최소 각도
const int LINK_TWO_MAX_ANGLE = 175;  // 링크 2 최대 각도
const int GRIPPER_MIN_ANGLE  = 10;   // 그리퍼 최소 각도
const int GRIPPER_MAX_ANGLE  = 80;   // 그리퍼 최대 각도

// 조향 서보 기본값
const int STEER_CENTER_DEG = 80;     // 조향 중앙 각도
const int STEER_MIN_DEG    = 55;     // 조향 최소 각도
const int STEER_MAX_DEG    = 105;    // 조향 최대 각도
const int STEER_STEP_DEG   = 1;      // 조향 1회 증감량
const int STEER_STEP_TIME  = 20;     // 조향 1회 증감 딜레이 (ms)

// 모터 핀
// const int L_SC_PIN  = 2;
const int L_PWM_PIN = 5;             // 왼쪽 모터 PWM 핀
const int L_DIR_PIN = 9;             // 왼쪽 모터 방향 핀
const int L_BK_PIN  = 12;            // 왼쪽 모터 브레이크 핀

// const int R_SC_PIN  = 3;
const int R_PWM_PIN = 6;             // 오른쪽 모터 PWM 핀
const int R_DIR_PIN = 10;            // 오른쪽 모터 방향 핀
const int R_BK_PIN  = 13;            // 오른쪽 모터 브레이크 핀

// 모터 테스트 상수
const int MOTOR_PWM_MIN   = 0;       // 모터 PWM 최소값
const int MOTOR_PWM_MAX   = 30;      // 모터 PWM 최대값
const int MOTOR_STEP      = 1;       // 모터 PWM 1회 증감량
const int MOTOR_STEP_TIME = 20;      // 모터 Ramping 딜레이 (ms)

// 시간 상수
const int REPEAT_TIME           = 100;             // 주기 시간 (ms)
const unsigned long CMD_TIMEOUT = 1000;            // 명령 타임아웃 (ms)

Adafruit_PWMServoDriver steerPwmDriver            = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver linkTwoBottomServoDriver  = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver linkOneGripperServoDriver = Adafruit_PWMServoDriver(0x42);

// 제어 변수 (모든 서보는 각도 단위로 추적)
int currentSteerAngle   = STEER_CENTER_DEG;         // 현재 조향 각도 (중앙 80도)
int currentMotorPWM     = 0;                        // 현재 모터 PWM 값
bool motorForward       = true;                     // 모터 방향 상태 (true=전진, false=후진)
int currentLinkOneAngle = LINK_ONE_DEFAULT_ANGLE;   // 현재 링크 1 각도
int currentBottomAngle  = BOTTOM_DEFAULT_ANGLE;     // 현재 하부 각도
int currentLinkTwoAngle = LINK_TWO_DEFAULT_ANGLE;   // 현재 링크 2 각도
int currentGripperAngle = GRIPPER_DEFAULT_ANGLE;    // 현재 그리퍼 각도

unsigned long perTimer    = 0;                      // 주기 타이머
unsigned long lastCmdTime = 0;                      // 마지막 명령 시간

// 디버그 설정
const bool serialEnabled = true;

// 각도를 PWM 값으로 변환 (0~180도 -> SERVO_PWM_MIN~SERVO_PWM_MAX)
int angleToPWM(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return map(angle, 0, 180, SERVO_PWM_MIN, SERVO_PWM_MAX);
}

// 현재 PWM에서 목표 PWM까지 1씩 Ramping
void softSetMotorPWM(int targetPWM) {
    targetPWM = constrain(targetPWM, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

    if (serialEnabled) {
        Serial.print("Motor Ramp: ");
        Serial.print(currentMotorPWM);
        Serial.print(" -> ");
        Serial.println(targetPWM);
    }

    while (currentMotorPWM != targetPWM) {
        if (currentMotorPWM < targetPWM) {
            currentMotorPWM++;
        } else {
            currentMotorPWM--;
        }
        analogWrite(L_PWM_PIN, currentMotorPWM);
        analogWrite(R_PWM_PIN, currentMotorPWM);
        delay(MOTOR_STEP_TIME);
    }
}

// Soft-Stop: PWM을 서서히 0으로 내린 뒤 브레이크 적용
void softStopMotors() {
    softSetMotorPWM(0);
    digitalWrite(L_BK_PIN, HIGH);
    digitalWrite(R_BK_PIN, HIGH);
    if (serialEnabled) Serial.println("모터 정지 (Soft-Stop)");
}

/*!
@brief 양쪽 BK 핀을 검사하여 안전 상태 확인 및 브레이크 해제
@return true=안전(주행 가능), false=비정상(긴급 정지 수행됨)
*/
bool checkAndReleaseBrakes() {
    bool lBrake = (digitalRead(L_BK_PIN) == HIGH);
    bool rBrake = (digitalRead(R_BK_PIN) == HIGH);

    if (lBrake != rBrake) {
        // 비대칭 BK 상태 → 한쪽만 브레이크 → 위험 → 긴급 정지
        analogWrite(L_PWM_PIN, 0);
        analogWrite(R_PWM_PIN, 0);
        currentMotorPWM = 0;
        digitalWrite(L_BK_PIN, HIGH);
        digitalWrite(R_BK_PIN, HIGH);
        if (serialEnabled) Serial.println("ERROR: 비대칭 BK 감지 - 긴급 정지");
        return false;
    }

    if (lBrake && rBrake) {
        // 양쪽 모두 BK ON → 정상 정지 상태 → 해제
        digitalWrite(L_BK_PIN, LOW);
        digitalWrite(R_BK_PIN, LOW);
    }
    // 양쪽 모두 OFF → 이미 주행 가능 상태
    return true;
}

/*!
@brief 서보를 각도 단위로 제어하는 함수
@param targetAngle  : 목표 각도
@param driver       : 서보 드라이버
@param channel      : 서보 채널
@param currentAngle : 현재 각도 (참조, 함수 내부에서 갱신)
@param minAngle     : 허용 최소 각도
@param maxAngle     : 허용 최대 각도
@param stepInterval : PWM 1단계당 딜레이 (ms)
*/
void softTurnToAngle(int targetAngle, Adafruit_PWMServoDriver& driver, int channel,
                     int& currentAngle, int minAngle, int maxAngle, int stepInterval) {
    targetAngle = constrain(targetAngle, minAngle, maxAngle);

    int startPWM  = angleToPWM(currentAngle);
    int targetPWM = angleToPWM(targetAngle);

    if (serialEnabled) {
        Serial.print("Servo Ramp: ");
        Serial.print(currentAngle);
        Serial.print(" -> ");
        Serial.print(targetAngle);
        Serial.print("도 (PWM ");
        Serial.print(startPWM);
        Serial.print(" -> ");
        Serial.print(targetPWM);
        Serial.println(")");
    }

    if (startPWM < targetPWM) {
        for (int pwm = startPWM; pwm <= targetPWM; pwm++) {
            driver.setPWM(channel, 0, pwm);
            delay(stepInterval);
        }
    } else if (startPWM > targetPWM) {
        for (int pwm = startPWM; pwm >= targetPWM; pwm--) {
            driver.setPWM(channel, 0, pwm);
            delay(stepInterval);
        }
    }

    currentAngle = targetAngle;
    if (serialEnabled) Serial.println("서보 이동 완료");
}

void handleCharCommand(char cmd) {
    switch (cmd) {
        // ── 모터 전진 ──
        case 'W': {
            // 양쪽 BK 핀 안전 검사 (비대칭이면 긴급 정지)
            if (!checkAndReleaseBrakes()) break;

            if (!motorForward && currentMotorPWM > 0) {
                // 후진 주행 중 전진 명령 → PWM 감소 (방향 전환 감속)
                softSetMotorPWM(currentMotorPWM - MOTOR_STEP);
                if (serialEnabled) {
                    Serial.print("방향 전환 감속 중 (PWM: ");
                    Serial.print(currentMotorPWM);
                    Serial.println(")");
                }
                if (currentMotorPWM == 0) {
                    // STOP 상태 도달 (BRAKE 없음) → 전진 DIR 전환
                    digitalWrite(L_DIR_PIN, HIGH);
                    digitalWrite(R_DIR_PIN, LOW);
                    motorForward = true;
                    if (serialEnabled) Serial.println("방향 전환 완료: 전진");
                }
            } else {
                // 정지 상태 또는 이미 전진 중 → 가속
                if (!motorForward && currentMotorPWM == 0) {
                    // 정지 상태에서 방향 설정
                    digitalWrite(L_DIR_PIN, HIGH);
                    digitalWrite(R_DIR_PIN, LOW);
                    motorForward = true;
                }
                softSetMotorPWM(currentMotorPWM + MOTOR_STEP);
            }
            break;
        }
        // ── 모터 후진 ──
        case 'S': {
            // 양쪽 BK 핀 안전 검사 (비대칭이면 긴급 정지)
            if (!checkAndReleaseBrakes()) break;

            if (motorForward && currentMotorPWM > 0) {
                // 전진 주행 중 후진 명령 → PWM 감소 (방향 전환 감속)
                softSetMotorPWM(currentMotorPWM - MOTOR_STEP);
                if (serialEnabled) {
                    Serial.print("방향 전환 감속 중 (PWM: ");
                    Serial.print(currentMotorPWM);
                    Serial.println(")");
                }
                if (currentMotorPWM == 0) {
                    // STOP 상태 도달 (BRAKE 없음) → 후진 DIR 전환
                    digitalWrite(L_DIR_PIN, LOW);
                    digitalWrite(R_DIR_PIN, HIGH);
                    motorForward = false;
                    if (serialEnabled) Serial.println("방향 전환 완료: 후진");
                }
            } else {
                // 정지 상태 또는 이미 후진 중 → 가속
                if (motorForward && currentMotorPWM == 0) {
                    // 정지 상태에서 방향 설정
                    digitalWrite(L_DIR_PIN, LOW);
                    digitalWrite(R_DIR_PIN, HIGH);
                    motorForward = false;
                }
                softSetMotorPWM(currentMotorPWM + MOTOR_STEP);
            }
            break;
        }
        // ── 조향 좌/우 ──
        case 'A':
            softTurnToAngle(currentSteerAngle + STEER_STEP_DEG, steerPwmDriver, STEER_SERVO_CH,
                            currentSteerAngle, STEER_MIN_DEG, STEER_MAX_DEG, STEER_STEP_TIME);
            break;
        case 'D':
            softTurnToAngle(currentSteerAngle - STEER_STEP_DEG, steerPwmDriver, STEER_SERVO_CH,
                            currentSteerAngle, STEER_MIN_DEG, STEER_MAX_DEG, STEER_STEP_TIME);
            break;
        // ── 모터 정지 (Soft-Stop) ──
        case 'Q':
            softStopMotors();
            break;
        // ── ARM: 바텀 회전 +/- ──
        case 'H':
            softTurnToAngle(currentBottomAngle + ARM_STEP_DEG, linkTwoBottomServoDriver, BOTTOM_CH,
                            currentBottomAngle, BOTTOM_MIN_ANGLE, BOTTOM_MAX_ANGLE, ARM_STEP_TIME);
            break;
        case 'L':
            softTurnToAngle(currentBottomAngle - ARM_STEP_DEG, linkTwoBottomServoDriver, BOTTOM_CH,
                            currentBottomAngle, BOTTOM_MIN_ANGLE, BOTTOM_MAX_ANGLE, ARM_STEP_TIME);
            break;
        // ── ARM: 링크1 틸트 +/- ──
        case 'K':
            softTurnToAngle(currentLinkOneAngle + ARM_STEP_DEG, linkOneGripperServoDriver, LINK_ONE_CH,
                            currentLinkOneAngle, LINK_ONE_MIN_ANGLE, LINK_ONE_MAX_ANGLE, ARM_STEP_TIME);
            break;
        case 'J':
            softTurnToAngle(currentLinkOneAngle - ARM_STEP_DEG, linkOneGripperServoDriver, LINK_ONE_CH,
                            currentLinkOneAngle, LINK_ONE_MIN_ANGLE, LINK_ONE_MAX_ANGLE, ARM_STEP_TIME);
            break;
        // ── ARM: 링크2 틸트 +/- ──
        case 'T':
            softTurnToAngle(currentLinkTwoAngle + ARM_STEP_DEG, linkTwoBottomServoDriver, LINK_TWO_CH,
                            currentLinkTwoAngle, LINK_TWO_MIN_ANGLE, LINK_TWO_MAX_ANGLE, ARM_STEP_TIME);
            break;
        case 'R':
            softTurnToAngle(currentLinkTwoAngle - ARM_STEP_DEG, linkTwoBottomServoDriver, LINK_TWO_CH,
                            currentLinkTwoAngle, LINK_TWO_MIN_ANGLE, LINK_TWO_MAX_ANGLE, ARM_STEP_TIME);
            break;
        // ── ARM: 그리퍼 열기/닫기 ──
        case 'O':
            softTurnToAngle(currentGripperAngle + ARM_STEP_DEG, linkOneGripperServoDriver, GRIPPER_CH,
                            currentGripperAngle, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE, ARM_STEP_TIME);
            break;
        case 'I':
            softTurnToAngle(currentGripperAngle - ARM_STEP_DEG, linkOneGripperServoDriver, GRIPPER_CH,
                            currentGripperAngle, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE, ARM_STEP_TIME);
            break;
        default:
            break;
    }
}

void processBluetooth() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (serialEnabled) {
            Serial.print("CMD: ");
            Serial.println(c);
        }
        handleCharCommand(c);
        lastCmdTime = millis();
    }
}

void setup() {
    if (serialEnabled) Serial.begin(115200);
    Serial1.begin(9600);

    steerPwmDriver.begin();
    steerPwmDriver.setPWMFreq(60);
    linkOneGripperServoDriver.begin();
    linkOneGripperServoDriver.setPWMFreq(60);
    linkTwoBottomServoDriver.begin();
    linkTwoBottomServoDriver.setPWMFreq(60);
    
    // 조향 서보 초기화 (중앙 80도)
    softTurnToAngle(STEER_CENTER_DEG, steerPwmDriver, STEER_SERVO_CH,
                    currentSteerAngle, STEER_MIN_DEG, STEER_MAX_DEG, STEER_STEP_TIME);

    // ARM 서보 초기화 (각도 기반)
    softTurnToAngle(LINK_ONE_DEFAULT_ANGLE, linkOneGripperServoDriver, LINK_ONE_CH,
                    currentLinkOneAngle, LINK_ONE_MIN_ANGLE, LINK_ONE_MAX_ANGLE, ARM_STEP_TIME);
    softTurnToAngle(GRIPPER_DEFAULT_ANGLE, linkOneGripperServoDriver, GRIPPER_CH,
                    currentGripperAngle, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE, ARM_STEP_TIME);
    softTurnToAngle(BOTTOM_DEFAULT_ANGLE, linkTwoBottomServoDriver, BOTTOM_CH,
                    currentBottomAngle, BOTTOM_MIN_ANGLE, BOTTOM_MAX_ANGLE, ARM_STEP_TIME);
    softTurnToAngle(LINK_TWO_DEFAULT_ANGLE, linkTwoBottomServoDriver, LINK_TWO_CH,
                    currentLinkTwoAngle, LINK_TWO_MIN_ANGLE, LINK_TWO_MAX_ANGLE, ARM_STEP_TIME);
    
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT); // ACTIVE LOW
    pinMode(L_BK_PIN, OUTPUT); // ACTIVE HIGH
    // pinMode(L_SC_PIN, INPUT_PULLUP);

    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(R_BK_PIN, OUTPUT);
    // pinMode(R_SC_PIN, INPUT_PULLUP);

    analogWrite(L_PWM_PIN, 0);
    digitalWrite(L_BK_PIN, HIGH);
    digitalWrite(L_DIR_PIN, HIGH);
    
    analogWrite(R_PWM_PIN, 0);
    digitalWrite(R_BK_PIN, HIGH);
    digitalWrite(R_DIR_PIN, LOW);
}

void loop() {
    processBluetooth();

    if (millis() - lastCmdTime > CMD_TIMEOUT) {
        // 이미 정지+브레이크 상태면 재호출 방지
        if (currentMotorPWM > 0 || digitalRead(L_BK_PIN) == LOW || digitalRead(R_BK_PIN) == LOW) {
            softStopMotors();
        }
    }

    if (millis() - perTimer > REPEAT_TIME) {
        perTimer = millis();

        Serial1.println("m:" + "left" + ":0:0:0:" + String(currentMotorPWM));
        Serial1.println("m:" + "right" + ":0:0:0:" + String(currentMotorPWM));
        Serial1.println("s:" + "0:" + String(currentSteerAngle));
        Serial1.println("a:" + String(currentBottomAngle) + ":0:" + String(currentLinkOneAngle) + ":0:" + String(currentLinkTwoAngle) + ":0:" + String(currentGripperAngle) + ":0");
    }
}