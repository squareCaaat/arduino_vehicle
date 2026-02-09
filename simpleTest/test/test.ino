#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servoPwmDriver = Adafruit_PWMServoDriver(0x40);

const int STEER_MIN = 150;
const int STEER_MAX = 600;
const int STEER_SERVO_CH = 15;

const int L_BK_PIN = 12;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 9;
// const int L_SC_PIN  = 2;

const int R_BK_PIN = 13;
const int R_PWM_PIN = 6;
const int R_DIR_PIN = 10;
// const int R_SC_PIN  = 3;

const int TARGET_PWM = 20;
const int DELAY_TIME = 3000;

bool isSteeringMode = false;  // 조향 테스트 모드
int currentSteerPWM = 350;  // 현재 조향 PWM 값 (중앙)

void setup() {
    Serial.begin(115200);
    servoPwmDriver.begin();
    servoPwmDriver.setPWMFreq(60);
    
    // 조향 서보 초기화 (중앙)
    servoPwmDriver.setPWM(STEER_SERVO_CH, 0, currentSteerPWM);
    
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT); // ACTIVE LOW
    pinMode(L_BK_PIN, OUTPUT); // ACTIVE HIGH
    // pinMode(L_SC_PIN, INPUT_PULLUP);
    analogWrite(L_PWM_PIN, 0);
    digitalWrite(L_BK_PIN, HIGH);
    digitalWrite(L_DIR_PIN, HIGH);

    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT); // ACTIVE LOW
    pinMode(R_BK_PIN, OUTPUT); // ACTIVE HIGH
    // pinMode(R_SC_PIN, INPUT_PULLUP);
    analogWrite(R_PWM_PIN, 0);
    digitalWrite(R_BK_PIN, HIGH);
    digitalWrite(R_DIR_PIN, LOW);

    Serial.println("========================================");
    Serial.println("차량 테스트 프로그램");
    Serial.println("F - 전진 테스트 (2초)");
    Serial.println("B - 후진 테스트 (2초)");
    Serial.println("T - 조향 테스트 (각도 입력: 0~180)");
    Serial.println("Q - 테스트 종료");
    Serial.println("========================================");
}

void stopMotors() {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
    digitalWrite(L_BK_PIN, HIGH);
    digitalWrite(R_BK_PIN, HIGH);
    Serial.println("모터 정지");
}

// 전진 테스트 (2초)
void testForward() {
    Serial.println("========================================");
    Serial.println("전진 테스트 시작");
    
    digitalWrite(L_BK_PIN, LOW);
    digitalWrite(R_BK_PIN, LOW);
    
    // 전진 방향 설정
    digitalWrite(L_DIR_PIN, HIGH);
    digitalWrite(R_DIR_PIN, LOW);
    
    // 상태 출력
    Serial.print("L BK: ");
    Serial.println(digitalRead(L_BK_PIN) ? "HIGH" : "LOW");
    Serial.print("R BK: ");
    Serial.println(digitalRead(R_BK_PIN) ? "HIGH" : "LOW");
    Serial.print("L DIR: ");
    Serial.println(digitalRead(L_DIR_PIN) ? "HIGH" : "LOW");
    Serial.print("R DIR: ");
    Serial.println(digitalRead(R_DIR_PIN) ? "HIGH" : "LOW");
    
    // 전진 (2초)
    analogWrite(L_PWM_PIN, TARGET_PWM);
    analogWrite(R_PWM_PIN, TARGET_PWM);
    delay(DELAY_TIME);
    
    // 정지
    stopMotors();
    Serial.println("전진 테스트 종료");
    Serial.println("========================================");
}

// 후진 테스트 (2초)
void testBackward() {
    Serial.println("========================================");
    Serial.println("후진 테스트 시작");
    
    digitalWrite(L_BK_PIN, LOW);
    digitalWrite(R_BK_PIN, LOW);
    
    // 후진 방향 설정
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_DIR_PIN, HIGH);
    
    // 상태 출력
    Serial.print("L BK: ");
    Serial.println(digitalRead(L_BK_PIN) ? "HIGH" : "LOW");
    Serial.print("R BK: ");
    Serial.println(digitalRead(R_BK_PIN) ? "HIGH" : "LOW");
    Serial.print("L DIR: ");
    Serial.println(digitalRead(L_DIR_PIN) ? "HIGH" : "LOW");
    Serial.print("R DIR: ");
    Serial.println(digitalRead(R_DIR_PIN) ? "HIGH" : "LOW");
    
    // 후진 (2초)
    analogWrite(L_PWM_PIN, TARGET_PWM);
    analogWrite(R_PWM_PIN, TARGET_PWM);
    delay(DELAY_TIME);
    
    // 정지
    stopMotors();
    Serial.println("후진 테스트 종료");
    Serial.println("========================================");
}

// 각도를 PWM 값으로 변환 (0~180도 -> STEER_MIN~STEER_MAX)
int angleToPWM(int angle) {
    // 범위 제한
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    return map(angle, 0, 180, STEER_MIN, STEER_MAX);
}

// 조향 서보를 천천히 회전
void steerToAngle(int targetAngle) {
    int targetPWM = angleToPWM(targetAngle);
    
    Serial.print("목표 각도: ");
    Serial.print(targetAngle);
    Serial.print("도 (PWM: ");
    Serial.print(targetPWM);
    Serial.println(")");
    
    // 현재 위치에서 목표 위치까지 천천히 이동
    if (currentSteerPWM < targetPWM) {
        // 증가
        for (int pwm = currentSteerPWM; pwm <= targetPWM; pwm++) {
            servoPwmDriver.setPWM(STEER_SERVO_CH, 0, pwm);
            delay(10);  // 10ms 간격으로 이동
        }
    } else {
        // 감소
        for (int pwm = currentSteerPWM; pwm >= targetPWM; pwm--) {
            servoPwmDriver.setPWM(STEER_SERVO_CH, 0, pwm);
            delay(10);  // 10ms 간격으로 이동
        }
    }
    
    currentSteerPWM = targetPWM;
    Serial.println("조향 완료!");
}

// 조향 테스트 모드
void handleSteeringMode() {
    // 시리얼 버퍼 비우기 (이전 입력의 개행 문자 등 제거)
    while (Serial.available() > 0) {
        Serial.read();
    }
    delay(100);
    
    Serial.println("========================================");
    Serial.println("조향 테스트 모드");
    Serial.println("0~180 사이의 각도를 입력하세요");
    Serial.println("현재 각도: " + String(map(currentSteerPWM, STEER_MIN, STEER_MAX, 0, 180)) + "도");
    Serial.println("Q - 조향 테스트 종료");
    Serial.println("========================================");
    
    while (isSteeringMode) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            // 빈 문자열 무시
            if (input.length() == 0) {
                continue;
            }
            
            if (input == "Q" || input == "q") {
                isSteeringMode = false;
                Serial.println("조향 테스트 종료!");
                break;
            }
            
            int angle = input.toInt();
            
            // toInt()는 변환 실패 시 0을 반환하므로, "0"을 입력한 경우와 구분 필요
            if ((angle == 0 && input != "0") || angle < 0 || angle > 180) {
                Serial.println("오류: 0~180 사이의 각도를 입력하세요");
            } else {
                steerToAngle(angle);
            }
        }
    }
}

void loop() {
    // 시리얼 입력 체크
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == 'F' || cmd == 'f') {
            // 전진 테스트
            testForward();
        }
        else if (cmd == 'B' || cmd == 'b') {
            // 후진 테스트
            testBackward();
        }
        else if (cmd == 'Q' || cmd == 'q') {
            stopMotors();
            Serial.println("테스트 종료!");
        }
        else if (cmd == 'T' || cmd == 't') {
            isSteeringMode = true;
            handleSteeringMode();
        }
    }
}