const int L_BK_PIN = 6;
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 9;
const int L_SC_PIN = 3;

const int R_BK_PIN = 13;
const int R_PWM_PIN = 12;
const int R_DIR_PIN = 11;
const int R_SC_PIN = 10;

volatile float feedback=0.00f, tic, tac;
float leftSc=0, rightSc=0, leftTrgt=0, rightTrgt=0, leftTrgtMin=0, leftTrgtMax=255, rightTrgtMin=0, rightTrgtMax=255, leftFeedbackMin=0, leftFeedbackMax=0, rightFeedbackMin=0, rightFeedbackMax=0;
bool measuring = false;

void countPulse() {
    tic = millis();
    feedback = tic - tac;
    tac = tic;
}

void setup() {
    Serial.begin(115200);
    
    pinMode(L_BK_PIN, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(L_SC_PIN, INPUT);

    pinMode(R_BK_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(R_SC_PIN, INPUT);
    
    
    digitalWrite(L_BK_PIN, HIGH);
    analogWrite(L_PWM_PIN, 0);
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_BK_PIN, HIGH);
    analogWrite(R_PWM_PIN, 0);
    digitalWrite(R_DIR_PIN, LOW);
    
    Serial.println("===== 펄스 측정 프로그램 =====");
    Serial.println("M: 최대 속도 측정 시작");
    Serial.println("N: 최소 속도 측정 시작");
    Serial.println("F: 모터 정지 및 측정 종료");
    Serial.println("================================");
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == 'M' || cmd == 'm') {
            measuring = true;

            attachInterrupt(digitalPinToInterrupt(L_SC_PIN), countPulse, RISING);
            attachInterrupt(digitalPinToInterrupt(R_SC_PIN), countPulse, RISING);
            
            digitalWrite(L_BK_PIN, LOW);
            analogWrite(L_PWM_PIN, 255);
            digitalWrite(L_DIR_PIN, LOW);

            digitalWrite(R_BK_PIN, LOW);
            analogWrite(R_PWM_PIN, 255);
            digitalWrite(R_DIR_PIN, LOW);
            
            Serial.println("\n>>> 모터 시작 (PWM: 255)");
            Serial.println(">>> 펄스 측정 시작...\n");
            
        } else if (cmd == 'F' || cmd == 'f') {
            // 모터 정지 및 측정 종료
            if (measuring) {
                measuring = false;

                detachInterrupt(digitalPinToInterrupt(L_SC_PIN));
                detachInterrupt(digitalPinToInterrupt(R_SC_PIN));
                
                analogWrite(L_PWM_PIN, 0);
                digitalWrite(L_BK_PIN, HIGH);
                
                analogWrite(R_PWM_PIN, 0);
                digitalWrite(R_BK_PIN, HIGH);
                
                Serial.println("\n>>> 모터 정지");
                Serial.println(">>> 측정 종료");
                Serial.print(">>> 총 펄스 수: ");
                Serial.println();
                Serial.println("================================\n");
            }
        }
    }
    
    // 50ms마다 펄스 수 출력
    if (measuring) {
        leftSc = analogRead(L_SC_PIN);
        rightSc = analogRead(R_SC_PIN);

        leftTrgt = map(leftSc, 0, 1023, leftTrgtMin, leftTrgtMax);
        rightTrgt = map(rightSc, 0, 1023, rightTrgtMin, rightTrgtMax);

        Serial.print("Left Target: ");
        Serial.println(leftTrgt);
        Serial.print("Left Feedback: ");
        Serial.println(leftFeedback);
        Serial.print("Right Target: ");
        Serial.println(rightTrgt);
        Serial.print(" Right Feedback: ");
        Serial.println(rightFeedback);
    }
}