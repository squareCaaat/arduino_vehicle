#include <Adafruit_PWMServoDriver.h>

// --- 디버그 설정 ---
const bool DEBUG_SERIAL = true;

// --- 핀 설정 (후륜 모터) ---
const int L_BK_PIN  = 12;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 9;
const int L_SC_PIN  = 2;  // 왼쪽 속도 센서(인터럽트)

const int R_BK_PIN  = 13;
const int R_PWM_PIN = 6;
const int R_DIR_PIN = 10;
const int R_SC_PIN  = 3;  // 오른쪽 속도 센서(인터럽트)

// --- PCA9685 채널 0x41 (ARM) ---
const int ARM_BOTTOM_CH   = 13;
const int ARM_LINK_ONE_CH = 14;
const int ARM_LINK_TWO_CH = 15;
const int GRAPPER_CH      = 12;

// --- 조향 서보 설정 (PCA9685 0x40) ---
const int STEERING_SERVO_CH = 15;   // PCA9685 채널 15번
const int STEER_CENTER_DEG  = 80;  // 중앙 각도 (실측)
const int STEER_LEFT_MAX    = 105; // 좌 최대 각도
const int STEER_RIGHT_MIN   = 55;  // 우 최대 각도
const int STEER_CENTER_PWM  = 350; // 중앙 PWM 값

const int SERVO_PWM_MIN = 150;  // 0도
const int SERVO_PWM_MAX = 600;  // 180도

// --- 제어 상수 ---
const float THROTTLE_ALPHA     = 0.7f;
const float MAX_DELTA          = 0.08f;
const int   CONTROL_INTERVAL   = 50;
const unsigned long CMD_TIMEOUT = 1000;
const int ARM_MIN_ANGLE = 150;
const int ARM_MAX_ANGLE = 600;

// --- Non-blocking 조향 ---
const int   STEER_STEP_INTERVAL = 20;  // 조향 업데이트 주기 (ms)
const float STEER_STEP_DEG      = 1.0f; // 한 번에 움직이는 각도

// --- 비블로킹 브레이크 ---
const unsigned long BRAKE_DURATION = 500;
unsigned long brakeStartTime = 0;
bool isBraking = false;
enum BrakeState { BRAKE_NONE, BRAKE_STOP, BRAKE_FORWARD, BRAKE_BACKWARD };
BrakeState brakeState = BRAKE_NONE;

// --- 제어 기능 플래그 ---
bool enableSoftStart   = true;
bool enablePID         = false;
bool enableInputFilter = true;
bool enableCenterHold  = false;  // 중앙 유지 기능 (주행 중 흔들림 보정)

// PID 계수
const float Kp = 0.75f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// 속도 프리셋
const float FWD_SPEED     = 0.2f;
const float BWD_SPEED     = -0.2f;
const float THROTTLE_STEP = 0.01f;

Adafruit_PWMServoDriver steerServoDriver = Adafruit_PWMServoDriver(0x40);  // 조향 
Adafruit_PWMServoDriver armServoDriver = Adafruit_PWMServoDriver(0x41);  // TODO: ARM 예정
Adafruit_PWMServoDriver pca_0x42 = Adafruit_PWMServoDriver(0x42);  // TODO: 미정

class Motor {
public:
    int pwmPin, dirPin, scPin, bkPin;
    volatile long pulseCount = 0;
    float currentTarget = 0.0f;
    float activeSpeed   = 0.0f;
    float integral      = 0.0f;
    float lastError     = 0.0f;
    int   lastPwmOut    = 0;

    Motor(int p, int d, int s, int b) : pwmPin(p), dirPin(d), scPin(s), bkPin(b) {}

    void init() {
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(bkPin, OUTPUT);
        pinMode(scPin, INPUT_PULLUP);
        analogWrite(pwmPin, 0);
        digitalWrite(bkPin, HIGH);
        digitalWrite(dirPin, LOW);
    }

    void setTarget(float speed) {
        currentTarget = constrain(speed, -1.0f, 1.0f);
    }

    void applySoftStart(bool enabled) {
        if (enabled) {
            if (currentTarget > activeSpeed + MAX_DELTA) {
                activeSpeed += MAX_DELTA;
            } else if (currentTarget < activeSpeed - MAX_DELTA) {
                activeSpeed -= MAX_DELTA;
            } else {
                activeSpeed = currentTarget;
            }
        } else {
            activeSpeed = currentTarget;
        }
    }

    void updatePID(bool enabled, bool debugSerial) {
        if (enabled) {
            long safePulseCount;
            noInterrupts();
            safePulseCount = pulseCount;
            pulseCount = 0;
            interrupts();

            float measuredSpeed = static_cast<float>(safePulseCount);
            if (activeSpeed < 0) measuredSpeed *= -1.0f;

            float targetPulses = activeSpeed * 80.0f;

            noInterrupts();
            float error = targetPulses - measuredSpeed;
            interrupts();
            integral += error;
            integral = constrain(integral, -100.0f, 100.0f);

            float derivative = error - lastError;
            float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            lastError = error;

            int pwmValue = constrain(static_cast<int>(fabs(output)), 0, 255);
            if (activeSpeed == 0.0f) {
                pwmValue = 0;
                integral = 0.0f;
            }
            analogWrite(pwmPin, pwmValue);
            lastPwmOut = pwmValue;
        } else {
            noInterrupts();
            pulseCount = 0;
            interrupts();
            integral = 0.0f;
            lastError = 0.0f;

            int pwmValue = constrain(static_cast<int>(fabs(activeSpeed) * 255.0f), 0, 255);
            if (activeSpeed == 0.0f) {
                pwmValue = 0;
                digitalWrite(bkPin, HIGH);
            }
            analogWrite(pwmPin, pwmValue);
            lastPwmOut = pwmValue;
        }
    }
};

class SteerController {
public:
    float currentAngle;  // 현재 각도
    float targetAngle;   // 목표 각도
    unsigned long lastUpdateMs = 0;

    SteerController() {
        currentAngle = STEER_CENTER_DEG;
        targetAngle  = STEER_CENTER_DEG;
    }

    void init() {
        currentAngle = STEER_CENTER_DEG;
        targetAngle  = STEER_CENTER_DEG;
        applyAngle(currentAngle);
    }

    // 목표 각도 설정
    void setTarget(float angle) {
        targetAngle = constrain(angle, (float)STEER_RIGHT_MIN, (float)STEER_LEFT_MAX);
    }

    // 좌회전 요청
    void turnLeft() {
        targetAngle = constrain(targetAngle + STEER_STEP_DEG * 3, (float)STEER_RIGHT_MIN, (float)STEER_LEFT_MAX);
    }

    // 우회전 요청
    void turnRight() {
        targetAngle = constrain(targetAngle - STEER_STEP_DEG * 3, (float)STEER_RIGHT_MIN, (float)STEER_LEFT_MAX);
    }

    // 중앙 복귀
    void center() {
        targetAngle = STEER_CENTER_DEG;
    }

    // Non-blocking 업데이트 (주기적 호출 필요)
    void update() {
        if (millis() - lastUpdateMs < STEER_STEP_INTERVAL) return;
        lastUpdateMs = millis();

        if (fabs(currentAngle - targetAngle) < STEER_STEP_DEG) {
            currentAngle = targetAngle;
        } else if (currentAngle < targetAngle) {
            currentAngle += STEER_STEP_DEG;
        } else {
            currentAngle -= STEER_STEP_DEG;
        }

        applyAngle(currentAngle);
    }

    // 중앙 유지 기능 (주행 중 목표가 없을 때 자동 복귀)
    void holdCenter(bool active) {
        if (active && fabs(targetAngle - STEER_CENTER_DEG) < 0.1f) {
            targetAngle = STEER_CENTER_DEG;
        }
    }

    bool isAtCenter() {
        return fabs(currentAngle - STEER_CENTER_DEG) < 1.0f;
    }

private:
    void applyAngle(float angle) {
        // 각도 → PWM 변환 (선형 보간)
        int pwmVal = map((int)(angle * 10), 0, 1800, SERVO_PWM_MIN, SERVO_PWM_MAX);
        steerServoDriver.setPWM(STEERING_SERVO_CH, 0, pwmVal);
    }
};

// 객체 생성
Motor leftMotor(L_PWM_PIN, L_DIR_PIN, L_SC_PIN, L_BK_PIN);
Motor rightMotor(R_PWM_PIN, R_DIR_PIN, R_SC_PIN, R_BK_PIN);
SteerController steer;

// 인터럽트
void countL() { leftMotor.pulseCount++; }
void countR() { rightMotor.pulseCount++; }

// 제어 변수
float targetThrottle    = 0.0f;
float filteredThrottle  = 0.0f;
bool  steerInputActive  = false;

unsigned long lastCmdTime    = 0;
unsigned long lastControlMs  = 0;
unsigned long lastSteerCmdMs = 0;

// ARM 제어 변수
int armBottomAngle   = 90;
int armLinkOneAngle  = 90;
int armLinkTwoAngle  = 180;
int grapperAngle     = 10;

void processBluetooth();
void handleCharCommand(char cmd);
void updateDrive();
void updateBrake();
void updateSteering();
void driveStop();
void driveForward();
void driveBackward();
void steerLeft();
void steerRight();

void armTiltUp();
void armTiltDown();
void armLinkTwoUp();
void armLinkTwoDown();
void armTurnLeft();
void armTurnRight();
void grapperOpen();
void grapperClose();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);  // HC-06

    steerServoDriver.begin();
    steerServoDriver.setPWMFreq(60);
    
    // TODO: 0x41, 0x42는 미정 - 연결 시 주석 해제
    // armServoDriver.begin();
    // armServoDriver.setPWMFreq(60);
    // pca_0x42.begin();
    // pca_0x42.setPWMFreq(60);

    // ARM 초기 위치
    armServoDriver.setPWM(GRAPPER_CH, 0, map(0, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    armServoDriver.setPWM(ARM_BOTTOM_CH, 0, map(180, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    armServoDriver.setPWM(ARM_LINK_ONE_CH, 0, map(90, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    armServoDriver.setPWM(ARM_LINK_TWO_CH, 0, map(90, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));

    leftMotor.init();
    rightMotor.init();
    steer.init();

    attachInterrupt(digitalPinToInterrupt(leftMotor.scPin), countL, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.scPin), countR, RISING);

    targetThrottle   = 0.0f;
    filteredThrottle = 0.0f;

    if (DEBUG_SERIAL) {
        Serial.println("Setup complete.");
        Serial.println("Steering: L/R commands");
        Serial.println("Drive: W/S commands");
    }
}

unsigned long perTimer = 0;
const int REPEAT_TIME = 1000;

void loop() {
    processBluetooth();
    updateBrake();
    updateDrive();
    updateSteering();

    // 디버그 출력
    if (DEBUG_SERIAL && millis() > perTimer + REPEAT_TIME) {
        perTimer = millis();
        Serial.print("Throttle: ");
        Serial.print(targetThrottle);
        Serial.print(" | Steer Angle: ");
        Serial.print(steer.currentAngle);
        Serial.print(" → ");
        Serial.println(steer.targetAngle);
        Serial.print("L PWM: ");
        Serial.print(leftMotor.lastPwmOut);
        Serial.print(" | R PWM: ");
        Serial.println(rightMotor.lastPwmOut);
        Serial.println("---------");
    }

    // Fail-safe: 타임아웃 시 정지
    if (millis() - lastCmdTime > CMD_TIMEOUT && !isBraking) {
        driveStop();
    }

    // 조향 입력 없으면 중앙 복귀 (선택 사항)
    if (enableCenterHold && millis() - lastSteerCmdMs > 300) {
        steer.center();
        steerInputActive = false;
    }
}

void updateSteering() {
    steer.update();
}

void updateBrake() {
    if (!isBraking) return;

    if (millis() - brakeStartTime >= BRAKE_DURATION) {
        isBraking = false;

        switch (brakeState) {
            case BRAKE_STOP:
                targetThrottle = 0.0f;
                break;
            case BRAKE_FORWARD:
                digitalWrite(leftMotor.dirPin, HIGH);
                digitalWrite(rightMotor.dirPin, LOW);
                digitalWrite(leftMotor.bkPin, LOW);
                digitalWrite(rightMotor.bkPin, LOW);
                break;
            case BRAKE_BACKWARD:
                digitalWrite(leftMotor.dirPin, LOW);
                digitalWrite(rightMotor.dirPin, HIGH);
                digitalWrite(leftMotor.bkPin, LOW);
                digitalWrite(rightMotor.bkPin, LOW);
                break;
            default:
                break;
        }
        brakeState = BRAKE_NONE;
    }
}

void updateDrive() {
    if (isBraking) return;
    if (millis() - lastControlMs < CONTROL_INTERVAL) return;
    lastControlMs = millis();

    // 입력 필터링
    if (enableInputFilter) {
        filteredThrottle = filteredThrottle * (1.0f - THROTTLE_ALPHA) + (targetThrottle * THROTTLE_ALPHA);
    } else {
        filteredThrottle = targetThrottle;
    }

    leftMotor.setTarget(filteredThrottle);
    rightMotor.setTarget(filteredThrottle);

    // Soft Start
    leftMotor.applySoftStart(enableSoftStart);
    rightMotor.applySoftStart(enableSoftStart);

    // PWM 출력
    leftMotor.updatePID(enablePID, DEBUG_SERIAL);
    rightMotor.updatePID(enablePID, DEBUG_SERIAL);
}

void processBluetooth() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (DEBUG_SERIAL) {
            Serial.print("CMD: ");
            Serial.println(c);
        }
        handleCharCommand(c);
        lastCmdTime = millis();
    }
}

void handleCharCommand(char cmd) {
    switch (cmd) {
        case 'W':
            driveForward();
            break;
        case 'S':
            driveBackward();
            break;
        case 'A':
            steerLeft();
            break;
        case 'D':
            steerRight();
            break;
        case 'Q':
            digitalWrite(leftMotor.bkPin, HIGH);
            digitalWrite(rightMotor.bkPin, HIGH);
            driveStop();
            break;
        case 'H':
            armTurnLeft();
            break;
        case 'J':
            armTiltDown();
            break;
        case 'K':
            armTiltUp();
            break;
        case 'L':
            armTurnRight();
            break;
        case 'Y':
            armLinkTwoUp();
            break;
        case 'U':
            armLinkTwoDown();
            break;
        case 'I':
            grapperOpen();
            break;
        case 'O':
            grapperClose();
            break;
        default:
            driveStop();
            break;
    }
}

void driveStop() {
    if (isBraking) return;

    if (targetThrottle > 0.0f) {
        targetThrottle = -0.1f;
    } else if (targetThrottle < 0.0f) {
        targetThrottle = 0.1f;
    } else {
        return;
    }

    isBraking = true;
    brakeStartTime = millis();
    brakeState = BRAKE_STOP;
}

void driveForward() {
    if (isBraking) return;

    if (digitalRead(leftMotor.bkPin) && digitalRead(rightMotor.bkPin)) {
        digitalWrite(leftMotor.bkPin, LOW);
        digitalWrite(rightMotor.bkPin, LOW);
    }

    // 후진 중이면 브레이크 후 전진
    if (!digitalRead(leftMotor.dirPin) && digitalRead(rightMotor.dirPin)) {
        digitalWrite(leftMotor.bkPin, HIGH);
        digitalWrite(rightMotor.bkPin, HIGH);
        if (DEBUG_SERIAL) Serial.println("Brake → Forward");
        isBraking = true;
        brakeStartTime = millis();
        brakeState = BRAKE_FORWARD;
        return;
    }

    digitalWrite(leftMotor.dirPin, HIGH);
    digitalWrite(rightMotor.dirPin, LOW);
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    targetThrottle = constrain(targetThrottle + THROTTLE_STEP, 0.0f, FWD_SPEED);
}

void driveBackward() {
    if (isBraking) return;

    if (digitalRead(leftMotor.bkPin) && digitalRead(rightMotor.bkPin)) {
        digitalWrite(leftMotor.bkPin, LOW);
        digitalWrite(rightMotor.bkPin, LOW);
    }

    // 전진 중이면 브레이크 후 후진
    if (digitalRead(leftMotor.dirPin) && !digitalRead(rightMotor.dirPin)) {
        digitalWrite(leftMotor.bkPin, HIGH);
        digitalWrite(rightMotor.bkPin, HIGH);
        if (DEBUG_SERIAL) Serial.println("Brake → Backward");
        isBraking = true;
        brakeStartTime = millis();
        brakeState = BRAKE_BACKWARD;
        return;
    }

    digitalWrite(leftMotor.dirPin, LOW);
    digitalWrite(rightMotor.dirPin, HIGH);
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    targetThrottle = constrain(targetThrottle - THROTTLE_STEP, BWD_SPEED, 0.0f);
}

void steerLeft() {
    steer.turnLeft();
    lastSteerCmdMs = millis();
    steerInputActive = true;
    if (DEBUG_SERIAL) Serial.println("Steer Left");
}

void steerRight() {
    steer.turnRight();
    lastSteerCmdMs = millis();
    steerInputActive = true;
    if (DEBUG_SERIAL) Serial.println("Steer Right");
}

void armTiltUp() {
    armLinkOneAngle = constrain(armLinkOneAngle + 3, 0, 180);
    armServoDriver.setPWM(ARM_LINK_ONE_CH, 0, map(armLinkOneAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Up");
}

void armTiltDown() {
    armLinkOneAngle = constrain(armLinkOneAngle - 3, 0, 180);
    armServoDriver.setPWM(ARM_LINK_ONE_CH, 0, map(armLinkOneAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Down");
}

void armLinkTwoUp() {
    armLinkTwoAngle = constrain(armLinkTwoAngle - 3, 0, 180);
    armServoDriver.setPWM(ARM_LINK_TWO_CH, 0, map(armLinkTwoAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Up");
}

void armLinkTwoDown() {
    armLinkTwoAngle = constrain(armLinkTwoAngle + 3, 0, 180);
    armServoDriver.setPWM(ARM_LINK_TWO_CH, 0, map(armLinkTwoAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Down");
}

void armTurnLeft() {
    armBottomAngle = constrain(armBottomAngle + 3, 0, 180);
    armServoDriver.setPWM(ARM_BOTTOM_CH, 0, map(armBottomAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Turn Left");
}

void armTurnRight() {
    armBottomAngle = constrain(armBottomAngle - 3, 0, 180);
    armServoDriver.setPWM(ARM_BOTTOM_CH, 0, map(armBottomAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Turn Right");
}

void grapperOpen() {
    grapperAngle = constrain(grapperAngle + 3, 0, 60);
    armServoDriver.setPWM(GRAPPER_CH, 0, map(grapperAngle, 0, 60, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Grapper Open");
}

void grapperClose() {
    grapperAngle = constrain(grapperAngle - 3, 0, 60);
    armServoDriver.setPWM(GRAPPER_CH, 0, map(grapperAngle, 0, 60, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Grapper Close");
}
