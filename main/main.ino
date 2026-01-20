#include <Servo.h>

// --- 핀 설정 ---
const int L_BK_PIN = 6;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 4;
const int L_SC_PIN  = 3;  // 왼쪽 속도 센서(인터럽트)

const int R_BK_PIN = 13;
const int R_PWM_PIN = 12;
const int R_DIR_PIN = 11;
const int R_SC_PIN  = 10;  // 오른쪽 속도 센서(인터럽트)

const int STEERING_SERVO_PIN = 8; // 조향 서보

// --- 통신 (HC-06) ---
const int TX = 18;
const int RX = 19;

// --- 제어 상수 ---
const float THROTTLE_ALPHA = 0.2f;          // 입력 필터 계수 (낮을수록 부드러움)
const float STEERING_SLOWDOWN_MAX = 0.5f;   // 조향 시 감속 비율
const float MAX_DELTA = 0.05f;              // Soft Start 가속도 제한 (더 완만하게)
const int   PID_INTERVAL = 50;              // PID 연산 주기 (ms)

// PID 계수 (실차 테스트 후 조정 필요)
const float Kp = 1.5f;
const float Ki = 0.8f;
const float Kd = 0.1f;

// 목표 속도 프리셋 (최대 속도를 절반으로 축소)
const float FWD_SPEED    = 0.3f;
const float BWD_SPEED    = -0.2f;
const float TURN_SPEED   = 0.2f;
const float TURN_STEER   = 0.7f;
const float THROTTLE_STEP = 0.05f;           // F 명령 시 증가 폭

// --- 모터 클래스 ---
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
        pinMode(dirPin, OUTPUT);
        pinMode(bkPin, OUTPUT);
        pinMode(scPin, INPUT_PULLUP);
        analogWrite(pwmPin, 0);
        digitalWrite(bkPin, HIGH);
        digitalWrite(dirPin, HIGH);
    }

    void setTarget(float speed) {
        currentTarget = constrain(speed, -1.0f, 1.0f);
    }

    // 가속도 제한(Soft Start)
    void applySoftStart() {
        if (currentTarget > activeSpeed + MAX_DELTA) {
            activeSpeed += MAX_DELTA;
        } else if (currentTarget < activeSpeed - MAX_DELTA) {
            activeSpeed -= MAX_DELTA;
        } else {
            activeSpeed = currentTarget;
        }
    }

    // 실제 펄스 기반 PID 계산
    void updatePID() {
        float measuredSpeed = static_cast<float>(pulseCount);
        pulseCount = 0; // 다음 주기 준비

        // 후진 시 펄스 방향 반영
        if (activeSpeed < 0) measuredSpeed *= -1.0f;

        // 목표를 펄스 단위 스케일로 변환 (예: ±80)
        float targetPulses = activeSpeed * 80.0f;
        float error = targetPulses - measuredSpeed;

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

        digitalWrite(dirPin, activeSpeed >= 0.0f ? HIGH : LOW);
        analogWrite(pwmPin, pwmValue);
        lastPwmOut = pwmValue;

        // 디버그: SC 펄스 기반 측정값 출력
        Serial.print("SC pin ");
        Serial.print(scPin);
        Serial.print(" pulses/ms: ");
        Serial.print(measuredSpeed);
        Serial.print(" target: ");
        Serial.print(targetPulses);
        Serial.print(" pwm: ");
        Serial.println(pwmValue);
    }
};

// --- 객체 생성 ---
Motor leftMotor(L_PWM_PIN, L_DIR_PIN, L_SC_PIN, L_BK_PIN);
Motor rightMotor(R_PWM_PIN, R_DIR_PIN, R_SC_PIN, R_BK_PIN);
Servo steeringServo;

// 인터럽트 서비스 루틴
void countL() { leftMotor.pulseCount++; }
void countR() { rightMotor.pulseCount++; }

// 제어 변수
float targetThrottle = 0.0f;
float filteredThrottle = 0.0f;
float steerCmd = 0.0f;

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 1000; // 1초
unsigned long lastPIDMs = 0;

void processBluetooth();
void handleCommand(const String& cmd);
void driveStop();
void driveForward();
void driveBackward();
void driveTurnLeft();
void driveTurnRight();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600); // HC-06

    leftMotor.init();
    rightMotor.init();
    steeringServo.attach(STEERING_SERVO_PIN);

    attachInterrupt(digitalPinToInterrupt(leftMotor.scPin), countL, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.scPin), countR, RISING);

    driveStop(); // 초기 정지 상태

    Serial.println("Setup complete.");
}

void loop() {
    processBluetooth();

    // 1. 입력 필터링 (Low-pass Filter)
    filteredThrottle = filteredThrottle * (1.0f - THROTTLE_ALPHA) + (targetThrottle * THROTTLE_ALPHA);

    // 2. 조향 감속 로직
    float absSteer = fabs(steerCmd);
    float steeringFactor = 1.0f - (STEERING_SLOWDOWN_MAX * absSteer);
    steeringFactor = constrain(steeringFactor, 0.5f, 1.0f);
    float finalThrottle = filteredThrottle * steeringFactor;

    // 3. 차동 조향 분배
    float steerGain = 0.4f;
    float leftCmd = finalThrottle + (steerGain * steerCmd);
    float rightCmd = finalThrottle - (steerGain * steerCmd);

    leftCmd = constrain(leftCmd, -1.0f, 1.0f);
    rightCmd = constrain(rightCmd, -1.0f, 1.0f);

    leftMotor.setTarget(leftCmd);
    rightMotor.setTarget(rightCmd);

    // 4. 조향 서보 출력 (90도 중심, ±45도)
    int servoAngle = 90 + static_cast<int>(steerCmd * 45.0f);
    servoAngle = constrain(servoAngle, 45, 135);
    steeringServo.write(servoAngle);

    // 5. Soft Start + PID (주기 실행)
    if (millis() - lastPIDMs >= PID_INTERVAL) {
        lastPIDMs = millis();

        leftMotor.applySoftStart();
        rightMotor.applySoftStart();

        leftMotor.updatePID();
        rightMotor.updatePID();
    }

    // Fail-safe: 일정 시간 명령 없음 → 정지
    if (millis() - lastCmdTime > CMD_TIMEOUT) {
        driveStop();
    }
}

void processBluetooth() {
    String cmd = "";
    if (Serial1.available() > 0) {
        cmd = Serial1.readStringUntil('\n');
        cmd.trim();
    }

    if (cmd.length() == 0) {
        return;
    }

    handleCommand(cmd);
    lastCmdTime = millis();

    Serial.print("Received command: ");
    Serial.println(cmd);
}

void handleCommand(const String& cmd) {
    if (cmd == "F") {
        driveForward();
    } else if (cmd == "B") {
        driveBackward();
    } else if (cmd == "L") {
        driveTurnLeft();
    } else if (cmd == "R") {
        driveTurnRight();
    } else if (cmd == "S") {
        driveStop();
    } else {
        Serial.println("Unknown command");
    }
}

void driveStop() {
    digitalWrite(leftMotor.bkPin, HIGH);
    digitalWrite(rightMotor.bkPin, HIGH);
    targetThrottle = 0.0f;
    steerCmd = 0.0f;
}

void driveForward() {
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    // F 명령을 반복 입력하면 목표 속도가 단계적으로 상승
    targetThrottle = constrain(targetThrottle + THROTTLE_STEP, 0.0f, FWD_SPEED);
    steerCmd = 0.0f;
}

void driveBackward() {
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    // 후진은 고정 속도로 설정
    targetThrottle = BWD_SPEED;
    steerCmd = 0.0f;
}

void driveTurnLeft() {
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    targetThrottle = TURN_SPEED * 0.5f;
    steerCmd = -TURN_STEER;
}

void driveTurnRight() {
    digitalWrite(leftMotor.bkPin, LOW);
    digitalWrite(rightMotor.bkPin, LOW);
    targetThrottle = TURN_SPEED * 0.5f;
    steerCmd = TURN_STEER;
}