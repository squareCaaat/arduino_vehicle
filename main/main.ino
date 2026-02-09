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

// --- PCA9685 채널 0x41 (Bottom, Gripper) ---
const int ARM_BOTTOM_CH   = 14;
const int GRIPPER_CH      = 15;

// --- PCA9685 채널 0x42 (Link One, Link Two) ---
const int ARM_LINK_ONE_CH = 14;
const int ARM_LINK_TWO_CH = 15;

// --- PCA9685 채널 0x40 (Steering) ---
const int STEERING_SERVO_CH = 15;

// 조향 상수
const int STEER_CENTER_DEG  = 80;  // 중앙 각도
const int STEER_LEFT_MAX    = 105; // 좌 최대 각도
const int STEER_RIGHT_MIN   = 55;  // 우 최대 각도
const int STEER_CENTER_PWM  = 350; // 중앙 PWM 값

// ARM 상수
const int ARM_BOTTOM_DEFAULT_ANGLE = 0;
const int ARM_LINK_ONE_DEFAULT_ANGLE = 170;
const int ARM_LINK_TWO_DEFAULT_ANGLE = 0;
const int GRIPPER_DEFAULT_ANGLE = 80;

const int ARM_BOTTOM_MIN_ANGLE = 0;
const int ARM_BOTTOM_MAX_ANGLE = 175;
const int ARM_LINK_ONE_MIN_ANGLE = 10;
const int ARM_LINK_ONE_MAX_ANGLE = 175;
const int ARM_LINK_TWO_MIN_ANGLE = 10;
const int ARM_LINK_TWO_MAX_ANGLE = 175;
const int GRIPPER_MIN_ANGLE = 10;
const int GRIPPER_MAX_ANGLE = 80;

const int SERVO_PWM_MIN = 150;  // 0도
const int SERVO_PWM_MAX = 600;  // 180도

// --- 제어 상수 ---
const float THROTTLE_ALPHA     = 0.7f;
const float MAX_DELTA          = 0.08f;
const int   CONTROL_INTERVAL   = 50;
const unsigned long CMD_TIMEOUT = 1000;

// --- Non-blocking 조향 ---
const int   STEER_STEP_INTERVAL = 20;  // 조향 업데이트 주기 (ms)
const float STEER_STEP_DEG      = 1.0f; // 한 번에 움직이는 각도

// --- ARM Non-blocking 상수 ---
const int   ARM_STEP_INTERVAL = 20;   // ARM 업데이트 주기 (ms)
const float ARM_STEP_DEG      = 1.0f; // 한 번에 움직이는 각도
const float ARM_CMD_DELTA     = 5.0f; // 명령 당 목표 각도 변화량

// --- 주행 방향 상태 ---
enum DriveDirection { DIR_STOPPED, DIR_FORWARD, DIR_BACKWARD };
DriveDirection currentDriveDir = DIR_STOPPED;
const float SPEED_ZERO_THRESHOLD = 0.015f;  // 속도 0 판정 임계값

// --- 제어 기능 플래그 ---
bool enableSoftStart   = true;
bool enablePID         = false;
bool enableInputFilter = true;
bool enableCenterHold  = true;  // 중앙 유지 기능 (주행 중 흔들림 보정)

// PID 계수
const float Kp = 0.75f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// 속도 프리셋
const float FWD_SPEED     = 0.3f;
const float BWD_SPEED     = -0.2f;
const float THROTTLE_STEP = 0.01f;

Adafruit_PWMServoDriver steerServoDriver = Adafruit_PWMServoDriver(0x40);  // 조향 
Adafruit_PWMServoDriver bottomGripServoDriver = Adafruit_PWMServoDriver(0x41);  // Bottom, Gripper
Adafruit_PWMServoDriver linkServoDriver = Adafruit_PWMServoDriver(0x42);  // Link One, Link Two

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

class ArmServo {
public:
    float currentAngle;
    float targetAngle;
    int minAngle;
    int maxAngle;
    int channel;
    Adafruit_PWMServoDriver* driver;
    unsigned long lastUpdateMs;

    ArmServo(Adafruit_PWMServoDriver* drv, int ch, int minA, int maxA)
        : driver(drv), channel(ch),
          currentAngle(0), targetAngle(0),
          minAngle(minA), maxAngle(maxA), lastUpdateMs(0) {}

    void init(float startAngle) {
        currentAngle = startAngle;
        targetAngle  = startAngle;
        applyAngle();
    }

    void setTarget(float angle) {
        targetAngle = constrain(angle, (float)minAngle, (float)maxAngle);
    }

    void addTarget(float delta) {
        targetAngle = constrain(targetAngle + delta, (float)minAngle, (float)maxAngle);
    }

    // Non-blocking 업데이트 (주기적 호출 필요)
    void update() {
        if (millis() - lastUpdateMs < ARM_STEP_INTERVAL) return;
        lastUpdateMs = millis();

        if (fabs(currentAngle - targetAngle) < ARM_STEP_DEG) {
            currentAngle = targetAngle;
        } else if (currentAngle < targetAngle) {
            currentAngle += ARM_STEP_DEG;
        } else {
            currentAngle -= ARM_STEP_DEG;
        }

        applyAngle();
    }

    bool isAtTarget() {
        return fabs(currentAngle - targetAngle) < 0.5f;
    }

private:
    void applyAngle() {
        int pwmVal = map((long)(currentAngle * 10), (long)(minAngle * 10), (long)(maxAngle * 10),
                         SERVO_PWM_MIN, SERVO_PWM_MAX);
        driver->setPWM(channel, 0, pwmVal);
    }
};

// 객체 생성
Motor leftMotor(L_PWM_PIN, L_DIR_PIN, L_SC_PIN, L_BK_PIN);
Motor rightMotor(R_PWM_PIN, R_DIR_PIN, R_SC_PIN, R_BK_PIN);
SteerController steer;

ArmServo armBottom(&bottomGripServoDriver, ARM_BOTTOM_CH, ARM_BOTTOM_MIN_ANGLE,ARM_BOTTOM_MAX_ANGLE);
ArmServo armLinkOne(&linkServoDriver, ARM_LINK_ONE_CH, ARM_LINK_ONE_MIN_ANGLE, ARM_LINK_ONE_MAX_ANGLE);
ArmServo armLinkTwo(&linkServoDriver, ARM_LINK_TWO_CH, ARM_LINK_TWO_MIN_ANGLE, ARM_LINK_TWO_MAX_ANGLE);
ArmServo gripper(&bottomGripServoDriver, GRIPPER_CH, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);

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

void processBluetooth();
void handleCharCommand(char cmd);
void updateDrive();
void updateSteering();
void updateArm();
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
void gripperOpen();
void gripperClose();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);  // HC-06

    steerServoDriver.begin();
    steerServoDriver.setPWMFreq(60);
    bottomGripServoDriver.begin();
    bottomGripServoDriver.setPWMFreq(60);
    linkServoDriver.begin();
    linkServoDriver.setPWMFreq(60);

    // ARM 초기 위치 (Non-blocking으로 부드럽게 이동)
    armBottom.init(ARM_BOTTOM_DEFAULT_ANGLE);
    armLinkOne.init(ARM_LINK_ONE_DEFAULT_ANGLE);
    armLinkTwo.init(ARM_LINK_TWO_DEFAULT_ANGLE);
    gripper.init(GRIPPER_DEFAULT_ANGLE);

    leftMotor.init();
    rightMotor.init();
    steer.init();

    attachInterrupt(digitalPinToInterrupt(leftMotor.scPin), countL, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.scPin), countR, RISING);

    targetThrottle   = 0.0f;
    filteredThrottle = 0.0f;

    if (DEBUG_SERIAL) {
        Serial.println("Setup complete.");
    }
}

unsigned long perTimer = 0;
const int REPEAT_TIME = 1000;

void loop() {
    processBluetooth();
    updateDrive();
    updateSteering();
    updateArm();

    if (DEBUG_SERIAL && millis() > perTimer + REPEAT_TIME) {
        perTimer = millis();
        if (targetThrottle != 0.0f) {
            Serial.print("Throttle: ");
            Serial.print(targetThrottle);
            Serial.print(" | Drive Direction: ");
            Serial.println(currentDriveDir);
            Serial.print("Active Speed (L|R): ");
            Serial.print(leftMotor.activeSpeed);
            Serial.print(" | ");
            Serial.println(rightMotor.activeSpeed);
            Serial.print("Current Pulse Count (L|R): ");
            Serial.print(leftMotor.pulseCount);
            Serial.print(" | ");
            Serial.println(rightMotor.pulseCount);
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

        if (targetThrottle != 0.0f) {
            String leftOut = "m:" + String(leftMotor.scPin) + ":" + String(targetThrottle) + ":" + String(leftMotor.activeSpeed) + ":" + String(leftMotor.pulseCount) + ":" + String(leftMotor.lastPwmOut);
            String rightOut = "m:" + String(rightMotor.scPin) + ":" + String(targetThrottle) + ":" + String(rightMotor.activeSpeed) + ":" + String(rightMotor.pulseCount) + ":" + String(rightMotor.lastPwmOut);
            Serial1.println(leftOut);
            Serial1.println(rightOut);
        }

        if (steer.currentAngle != steer.targetAngle) {
            String steerOut = "s:" + String(steer.currentAngle) + ":" + String(steer.targetAngle);
            Serial1.println(steerOut);
        }
        
        if (armBottom.currentAngle != armBottom.targetAngle || armLinkOne.currentAngle != armLinkOne.targetAngle || armLinkTwo.currentAngle != armLinkTwo.targetAngle || gripper.currentAngle != gripper.targetAngle) {
            String armOut = "a:" + String(armBottom.currentAngle) + ":" + String(armBottom.targetAngle) + ":" + String(armLinkOne.currentAngle) + ":" + String(armLinkOne.targetAngle) + ":" + String(armLinkTwo.currentAngle) + ":" + String(armLinkTwo.targetAngle) + ":" + String(gripper.currentAngle) + ":" + String(gripper.targetAngle);
            Serial1.println(armOut);
        }
    }

    // Fail-safe: 타임아웃 시 정지
    if (millis() - lastCmdTime > CMD_TIMEOUT) {
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

void updateArm() {
    armBottom.update();
    armLinkOne.update();
    armLinkTwo.update();
    gripper.update();
}

void updateDrive() {
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
            // 긴급 정지: 즉시 모든 출력 0 + BK 활성화
            targetThrottle = 0.0f;
            filteredThrottle = 0.0f;
            leftMotor.activeSpeed = 0.0f;
            rightMotor.activeSpeed = 0.0f;
            currentDriveDir = DIR_STOPPED;
            analogWrite(leftMotor.pwmPin, 0);
            analogWrite(rightMotor.pwmPin, 0);
            digitalWrite(leftMotor.bkPin, HIGH);
            digitalWrite(rightMotor.bkPin, HIGH);
            if (DEBUG_SERIAL) Serial.println("Emergency Stop!");
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
        case 'R':
            armLinkTwoUp();
            break;
        case 'T':
            armLinkTwoDown();
            break;
        case 'I':
            gripperOpen();
            break;
        case 'O':
            gripperClose();
            break;
        default:
            driveStop();
            break;
    }
}

void driveStop() {
    targetThrottle = 0.0f;
    currentDriveDir = DIR_STOPPED;
    // Soft Start가 activeSpeed를 서서히 0으로 감속시킴
}

void driveForward() {
    // 브레이크 해제
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);

    switch (currentDriveDir) {
        case DIR_BACKWARD:
            // 후진 중 → 감속만 수행 (DIR 변경 안 함)
            targetThrottle += THROTTLE_STEP;  // 음수 → 0 방향으로 증가

            if (targetThrottle >= -SPEED_ZERO_THRESHOLD) {
                // 속도가 0에 도달 → 방향 전환 + 전진 시작
                targetThrottle = THROTTLE_STEP;
                filteredThrottle = 0.0f;
                leftMotor.activeSpeed = 0.0f;
                rightMotor.activeSpeed = 0.0f;

                digitalWrite(leftMotor.dirPin, HIGH);
                digitalWrite(rightMotor.dirPin, LOW);
                currentDriveDir = DIR_FORWARD;

                if (DEBUG_SERIAL) Serial.println("Dir change -> Forward");
            }
            // 아직 0이 아니면 → 후진 방향 유지, 속도만 줄어든 상태로 대기
            break;

        case DIR_STOPPED:
            // 정지 상태 → DIR 설정 후 전진 시작
            digitalWrite(leftMotor.dirPin, HIGH);
            digitalWrite(rightMotor.dirPin, LOW);
            currentDriveDir = DIR_FORWARD;
            targetThrottle = THROTTLE_STEP;
            break;

        case DIR_FORWARD:
            // 이미 전진 중 → 가속
            targetThrottle = constrain(targetThrottle + THROTTLE_STEP, 0.0f, FWD_SPEED);
            break;
    }
}

void driveBackward() {
    // 브레이크 해제
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);

    switch (currentDriveDir) {
        case DIR_FORWARD:
            // 전진 중 → 감속만 수행 (DIR 변경 안 함)
            targetThrottle -= THROTTLE_STEP;  // 양수 → 0 방향으로 감소

            if (targetThrottle <= SPEED_ZERO_THRESHOLD) {
                // 속도가 0에 도달 → 방향 전환 + 후진 시작
                targetThrottle = -THROTTLE_STEP;
                filteredThrottle = 0.0f;
                leftMotor.activeSpeed = 0.0f;
                rightMotor.activeSpeed = 0.0f;

                digitalWrite(leftMotor.dirPin, LOW);
                digitalWrite(rightMotor.dirPin, HIGH);
                currentDriveDir = DIR_BACKWARD;

                if (DEBUG_SERIAL) Serial.println("Dir change -> Backward");
            }
            // 아직 0이 아니면 → 전진 방향 유지, 속도만 줄어든 상태로 대기
            break;

        case DIR_STOPPED:
            // 정지 상태 → DIR 설정 후 후진 시작
            digitalWrite(leftMotor.dirPin, LOW);
            digitalWrite(rightMotor.dirPin, HIGH);
            currentDriveDir = DIR_BACKWARD;
            targetThrottle = -THROTTLE_STEP;
            break;

        case DIR_BACKWARD:
            // 이미 후진 중 → 가속 (절대값 증가)
            targetThrottle = constrain(targetThrottle - THROTTLE_STEP, BWD_SPEED, 0.0f);
            break;
    }
}

void steerLeft() {
    steer.turnLeft();
    lastSteerCmdMs = millis();
    steerInputActive = true;
}

void steerRight() {
    steer.turnRight();
    lastSteerCmdMs = millis();
    steerInputActive = true;
}

void armTiltUp() {
    armLinkOne.addTarget(ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Up");
}

void armTiltDown() {
    armLinkOne.addTarget(-ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Down");
}

void armLinkTwoUp() {
    armLinkTwo.addTarget(-ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Up");
}

void armLinkTwoDown() {
    armLinkTwo.addTarget(ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Down");
}

void armTurnLeft() {
    armBottom.addTarget(ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Turn Left");
}

void armTurnRight() {
    armBottom.addTarget(-ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Arm Turn Right");
}

void gripperOpen() {
    gripper.addTarget(ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Gripper Open");
}

void gripperClose() {
    gripper.addTarget(-ARM_CMD_DELTA);
    if (DEBUG_SERIAL) Serial.println("Gripper Close");
}
