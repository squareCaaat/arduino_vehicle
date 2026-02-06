#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

// --- 디버그 설정 ---
const bool DEBUG_SERIAL = false;  // Serial(USB) 디버그 출력 활성화

// --- 핀 설정 ---
const int L_BK_PIN = 12;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 10;
const int L_SC_PIN  = 2;  // 왼쪽 속도 센서(인터럽트)

const int R_BK_PIN = 13;
const int R_PWM_PIN = 6;
const int R_DIR_PIN = 9;
const int R_SC_PIN  = 3;  // 오른쪽 속도 센서(인터럽트)

// PCA9685 사용으로 변경 예정
const int STEERING_SERVO_PIN = 8; // 조향 서보

const int ARM_BOTTOM_CH = 13;     // 모터 드라이버 채널
const int ARM_LINK_ONE_CH = 14;
const int ARM_LINK_TWO_CH = 15;
const int GRAPPER_CH = 12;

// --- 통신 (HC-06) ---
const int TX = 18;
const int RX = 19;

// --- 제어 상수 ---
const float THROTTLE_ALPHA = 0.2f;          // 입력 필터 계수 (낮을수록 부드러움)
const float STEERING_SLOWDOWN_MAX = 0.5f;   // 조향 시 감속 비율
const float MAX_DELTA = 0.05f;              // Soft Start 가속도 제한 (더 완만하게)
const int   PID_INTERVAL = 50;              // PID 연산 주기 (ms)
const unsigned long CMD_TIMEOUT = 1000;     // 명령 타임아웃 (ms)
const float STEER_GAIN = 0.4f;              // 차동 조향 분배 계수
const int ARM_MIN_ANGLE = 150;
const int ARM_MAX_ANGLE = 600;

// --- 비블로킹 브레이크 타이머 ---
const unsigned long BRAKE_DURATION = 500;   // 브레이크 지속 시간 (ms)
unsigned long brakeStartTime = 0;
bool isBraking = false;
enum BrakeState { BRAKE_NONE, BRAKE_STOP, BRAKE_FORWARD, BRAKE_BACKWARD };
BrakeState brakeState = BRAKE_NONE;

// --- 제어 기능 활성화 플래그 ---
bool enableSoftStart = true;                // Soft Start 활성화
bool enablePID = false;                     // PID 제어 활성화
bool enableInputFilter = true;              // 입력 필터링 활성화
bool enableDifferentialSteering = true;     // 차동 조향 분배 활성화

// PID 계수 (실차 테스트 후 조정 필요)
const float Kp = 0.75f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// 목표 속도 프리셋
const float FWD_SPEED    = 0.4f;            // 전진 최대 속도
const float BWD_SPEED    = -0.2f;           // 후진 최대 속도
const float TURN_SPEED   = 0.3f;            // 조향 최대 속도
const float STEER_STEP   = 0.05f;           // 조향 증가 폭
const float THROTTLE_STEP = 0.05f;          // F 명령 시 증가 폭

class Motor {
public:
    int pwmPin, dirPin, scPin;
    int bkPin;
    volatile long pulseCount = 0;
    float currentTarget = 0.0f; // 목표 속도 (-1.0 ~ 1.0)
    float activeSpeed = 0.0f;   // Soft Start가 적용된 실제 목표 속도

    // PID 내부 변수
    float integral = 0.0f;
    float lastError = 0.0f;
    int lastPwmOut = 0;

    Motor(int p, int d, int s, int b) : pwmPin(p), dirPin(d), scPin(s), bkPin(b) {}

    void init() {
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT); // ACTIVE LOW
        pinMode(bkPin, OUTPUT); // ACTIVE HIGH
        pinMode(scPin, INPUT_PULLUP);
        analogWrite(pwmPin, 0);
        digitalWrite(bkPin, HIGH);
        digitalWrite(dirPin, LOW);
    }

    void setTarget(float speed) {
        currentTarget = constrain(speed, -1.0f, 1.0f);
    }

    // 가속도 제한(Soft Start) - 활성화 여부에 따라 적용
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
            // Soft Start 비활성화: 즉시 목표 속도 적용
            activeSpeed = currentTarget;
        }
    }

    // PID 제어 - 활성화 여부에 따라 적용
    void updatePID(bool enabled, bool debugSerial) {
        if (enabled) {
            long safePulseCount;

            noInterrupts();
            safePulseCount = pulseCount;
            pulseCount = 0;
            interrupts();

            float measuredSpeed = static_cast<float>(safePulseCount);

            // 후진 시 펄스 방향 반영
            // 실제 출력 기준으로 변경
            if (lastPwmOut < 0) measuredSpeed *= -1.0f;

            // 목표를 펄스 단위 스케일로 변환 (예: ±80)
            float targetPulses = activeSpeed * 90.0f;

            // PID
            noInterrupts();
            float error = targetPulses - measuredSpeed;
            interrupts();
            integral += error;
            integral = constrain(integral, -100.0f, 100.0f); // 윈드업 방지

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

            // 디버그 출력 (조건부)
            if (measuredSpeed != 0.0f && targetPulses != 0.0f) {
                if (debugSerial) {
                    Serial.print("SC pin ");
                    Serial.print(scPin);
                    Serial.print(" pulses/ms: ");
                    Serial.print(measuredSpeed);
                    Serial.print(" target: ");
                    Serial.print(targetPulses);
                    Serial.print(" pwm: ");
                    Serial.println(pwmValue);
                    Serial.print(" dir: ");
                    Serial.println(digitalRead(dirPin) ? 1 : 0);
                    Serial.print(" break: ");
                    Serial.println(digitalRead(bkPin) ? 1 : 0);
                }

                // 블루투스 출력은 항상 유지
                String btOutput = String(scPin) + ":" + String(measuredSpeed) + ":" + String(targetPulses) + ":" + String(pwmValue) + ":" + String(digitalRead(dirPin) ? 1 : 0) + ":" + String(digitalRead(bkPin) ? 1 : 0);
                Serial1.println(btOutput);
            }
        } else {
            // PID 비활성화: activeSpeed를 직접 PWM으로 변환
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

// --- 객체 생성 ---
Motor leftMotor(L_PWM_PIN, L_DIR_PIN, L_SC_PIN, L_BK_PIN);
Motor rightMotor(R_PWM_PIN, R_DIR_PIN, R_SC_PIN, R_BK_PIN);
Adafruit_PWMServoDriver servoPwmDriver = Adafruit_PWMServoDriver(0x40);
Servo steeringServo;

// 인터럽트 서비스 루틴
void countL() { leftMotor.pulseCount++; }
void countR() { rightMotor.pulseCount++; }

// 제어 변수
float targetThrottle = 0.0f;
float filteredThrottle = 0.0f;
float steerCmd = 0.0f;

unsigned long lastCmdTime = 0;
unsigned long lastPIDMs = 0;

// 함수 선언
void processBluetooth();
void handleCharCommand(char cmd);
void updateDrive();
void updateBrake();
void driveStop();
void driveForward();
void driveBackward();
void driveTurnLeft();
void driveTurnRight();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600); // HC-06
    servoPwmDriver.begin();
    servoPwmDriver.setPWMFreq(60);

    servoPwmDriver.setPWM(GRAPPER_CH, 0, map(0, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    servoPwmDriver.setPWM(ARM_BOTTOM_CH, 0, map(180, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    servoPwmDriver.setPWM(ARM_LINK_ONE_CH, 0, map(90, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    servoPwmDriver.setPWM(ARM_LINK_TWO_CH, 0, map(90, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));

    leftMotor.init();
    rightMotor.init();
    steeringServo.attach(STEERING_SERVO_PIN);

    attachInterrupt(digitalPinToInterrupt(leftMotor.scPin), countL, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.scPin), countR, RISING);

    // 초기 정지 상태
    targetThrottle = 0.0f;
    steerCmd = 0.0f;

    if (DEBUG_SERIAL) {
        Serial.println("Setup complete.");
    }
}

unsigned long perTimer = 0;
const int REPEAT_TIME = 1000;

void loop() {
    processBluetooth();
    updateBrake();  // 비블로킹 브레이크 처리
    updateDrive();

    // 디버그 출력 (조건부)
    if (DEBUG_SERIAL && millis() > perTimer + REPEAT_TIME) {
        perTimer = millis();
        Serial.print("Target Throttle: ");
        Serial.println(targetThrottle);
        Serial.print("L PWM: ");
        Serial.println(leftMotor.lastPwmOut);
        Serial.print("L Pulse: ");
        Serial.println(leftMotor.pulseCount);
        Serial.print("L DIR: ");
        Serial.println(digitalRead(leftMotor.dirPin) ? "HIGH" : "LOW");
        Serial.print("L BK: ");
        Serial.println(digitalRead(leftMotor.bkPin) ? "HIGH" : "LOW");
        Serial.println("=========");
        Serial.print("R PWM: ");
        Serial.println(rightMotor.lastPwmOut);
        Serial.print("R Pulse: ");
        Serial.println(rightMotor.pulseCount);
        Serial.print("R DIR: ");
        Serial.println(digitalRead(rightMotor.dirPin) ? "HIGH" : "LOW");
        Serial.print("R BK: ");
        Serial.println(digitalRead(rightMotor.bkPin) ? "HIGH" : "LOW");
    }
    
    // Fail-safe: 일정 시간 명령 없음 → 정지
    if (millis() - lastCmdTime > CMD_TIMEOUT && !isBraking) {
        driveStop();
    }
}

// 비블로킹 브레이크 상태 처리
void updateBrake() {
    if (!isBraking) return;
    
    if (millis() - brakeStartTime >= BRAKE_DURATION) {
        // 브레이크 시간 종료
        isBraking = false;
        
        switch (brakeState) {
            case BRAKE_STOP:
                targetThrottle = 0.0f;
                steerCmd = 0.0f;
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

// 주행 제어 업데이트 - 활성화 플래그에 따라 각 기능 적용
void updateDrive() {
    // 브레이킹 중이면 드라이브 업데이트 스킵
    if (isBraking) return;
    
    // 1. 입력 필터링 (Low-pass Filter)
    /* TODO: 필터 계산 방법 검토 필요
    * PID 주기와 차이가 발생할 수 있으므로 
    * 필터 계산을 주기 내로 적용하는 것도 고려 중...
    */
    
    float throttleToUse;
    if (enableInputFilter) {
        filteredThrottle = filteredThrottle * (1.0f - THROTTLE_ALPHA) + (targetThrottle * THROTTLE_ALPHA);
        throttleToUse = filteredThrottle;
    } else {
        filteredThrottle = targetThrottle;
        throttleToUse = targetThrottle;
    }

    // 2. 조향 감속 로직
    float absSteer = fabs(steerCmd);
    float steeringFactor = 1.0f - (STEERING_SLOWDOWN_MAX * absSteer);
    steeringFactor = constrain(steeringFactor, 0.5f, 1.0f);
    float finalThrottle = throttleToUse * steeringFactor;

    // 3. 차동 조향 분배
    float leftCmd, rightCmd;
    if (enableDifferentialSteering) {
        float steerDiff = STEER_GAIN * steerCmd;
        leftCmd = finalThrottle + steerDiff;
        rightCmd = finalThrottle - steerDiff;
        leftCmd = constrain(leftCmd, -1.0f, 1.0f);
        rightCmd = constrain(rightCmd, -1.0f, 1.0f);
    } else {
        // 차동 조향 비활성화: 양쪽 동일 속도
        leftCmd = finalThrottle;
        rightCmd = finalThrottle;
    }

    leftMotor.setTarget(leftCmd);
    rightMotor.setTarget(rightCmd);

    // 4. 조향 서보 출력 (90도 중심, ±45도)
    int servoAngle = 90 + static_cast<int>(steerCmd * 45.0f);
    servoAngle = constrain(servoAngle, 45, 135);
    steeringServo.write(servoAngle);

    // 5. Soft Start + PID (주기 실행)
    if (millis() - lastPIDMs >= PID_INTERVAL) {
        lastPIDMs = millis();

        leftMotor.applySoftStart(enableSoftStart);
        rightMotor.applySoftStart(enableSoftStart);

        leftMotor.updatePID(enablePID, DEBUG_SERIAL);
        rightMotor.updatePID(enablePID, DEBUG_SERIAL);
    }
}

void processBluetooth() {
    while(Serial1.available() > 0) {
        char c = Serial1.read();
        if (DEBUG_SERIAL) {
            Serial.print("Received command: ");
            Serial.println(c);
        }
        handleCharCommand(c);
        lastCmdTime = millis();
    }
}

void handleCharCommand(char cmd) {
    switch (cmd)
    {
    case 'W':
        driveForward();
        break;
    case 'S':
        driveBackward();
        break;
    case 'A':
        driveTurnLeft();
        break;
    case 'D':
        driveTurnRight();
        break;
    case 'Q':
        // 급 브레이크
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
    if (isBraking) return;  // 이미 브레이킹 중이면 무시
    
    // 역방향 토크로 브레이킹 시작 (비블로킹)
    if (targetThrottle > 0.0f) {
        targetThrottle = -0.1f;
    } else if (targetThrottle < 0.0f) {
        targetThrottle = 0.1f;
    } else {
        // 이미 정지 상태
        steerCmd = 0.0f;
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
    
    // 후진 중이면 브레이크 후 전진 (비블로킹)
    if (!digitalRead(leftMotor.dirPin) && digitalRead(rightMotor.dirPin)) {
        digitalWrite(leftMotor.bkPin, HIGH);
        digitalWrite(rightMotor.bkPin, HIGH);
        if (DEBUG_SERIAL) {
            Serial.println("Brake and Forward!");
        }
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
    
    // 전진 중이면 브레이크 후 후진 (비블로킹)
    if (digitalRead(leftMotor.dirPin) && !digitalRead(rightMotor.dirPin)) {
        digitalWrite(leftMotor.bkPin, HIGH);
        digitalWrite(rightMotor.bkPin, HIGH);
        if (DEBUG_SERIAL) {
            Serial.println("Brake and Backward!");
        }
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

void driveTurnLeft() {
    if (isBraking) return;
    
    if (digitalRead(leftMotor.bkPin) && digitalRead(rightMotor.bkPin)) {
        digitalWrite(leftMotor.bkPin, LOW);
        digitalWrite(rightMotor.bkPin, LOW);
    }
    steerCmd = constrain(steerCmd - STEER_STEP, -1.0f, 1.0f);
}

void driveTurnRight() {
    if (isBraking) return;
    
    if (digitalRead(leftMotor.bkPin) && digitalRead(rightMotor.bkPin)) {
        digitalWrite(leftMotor.bkPin, LOW);
        digitalWrite(rightMotor.bkPin, LOW);
    }
    steerCmd = constrain(steerCmd + STEER_STEP, -1.0f, 1.0f);
}

// ARM 제어 변수
int armBottomAngle = 90;
int armLinkOneAngle = 90;
int armLinkTwoAngle = 180;
int grapperAngle = 10;

void armTiltUp() {
    armLinkOneAngle = constrain(armLinkOneAngle + 3, 0, 180);
    servoPwmDriver.setPWM(ARM_LINK_ONE_CH, 0, map(armLinkOneAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Up");
}

void armTiltDown() {
    armLinkOneAngle = constrain(armLinkOneAngle - 3, 0, 180);
    servoPwmDriver.setPWM(ARM_LINK_ONE_CH, 0, map(armLinkOneAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Tilt Down");
}

void armLinkTwoUp() {
    armLinkTwoAngle = constrain(armLinkTwoAngle - 3, 0, 180);
    servoPwmDriver.setPWM(ARM_LINK_TWO_CH, 0, map(armLinkTwoAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Up");
}

void armLinkTwoDown() {
    armLinkTwoAngle = constrain(armLinkTwoAngle + 3, 0, 180);
    servoPwmDriver.setPWM(ARM_LINK_TWO_CH, 0, map(armLinkTwoAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Link Two Down");
}

void armTurnLeft() {
    armBottomAngle = constrain(armBottomAngle + 3, 0, 180);
    servoPwmDriver.setPWM(ARM_BOTTOM_CH, 0, map(armBottomAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Turn Left");
}

void armTurnRight() {
    armBottomAngle = constrain(armBottomAngle - 3, 0, 180);
    servoPwmDriver.setPWM(ARM_BOTTOM_CH, 0, map(armBottomAngle, 0, 180, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Arm Turn Right");
}

void grapperOpen() {
    grapperAngle = constrain(grapperAngle + 3, 0, 60);
    servoPwmDriver.setPWM(GRAPPER_CH, 0, map(grapperAngle, 0, 60, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Grapper Open");
}

void grapperClose() {
    grapperAngle = constrain(grapperAngle - 3, 0, 60);
    servoPwmDriver.setPWM(GRAPPER_CH, 0, map(grapperAngle, 0, 60, ARM_MIN_ANGLE, ARM_MAX_ANGLE));
    if (DEBUG_SERIAL) Serial.println("Grapper Close");
}
