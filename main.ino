#include <Servo.h>

// Motor
const int L_PWM_PIN = 5;
const int L_DIR_PIN = 4;
const int L_EN_PIN = 3; // Brake

const int R_PWM_PIN = 12;
const int R_DIR_PIN = 11;
const int R_EN_PIN = 10; // Brake

// Servo
const int STEERING_SERVO_PIN = 8;
const int ARM_SERVO1_PIN = 9;
const int ARM_SERVO2_PIN = 10;

// HC-06
const int TX = 18;
const int RX = 19;

class Motor {
public:
    Motor(int pwmPin, int dirPin, int enPin)
        : pwmPin(pwmPin), dirPin(dirPin), enPin(enPin),
        currentSpeed(0.0f), maxDelta(0.02f) { // Max delta per update
        // Constructor implementation
    }

    void begin() {
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enPin, OUTPUT);
        disable();
    }

    void setSpeed(float speed) {
        speed = constrain(speed, -1.0f, 1.0f);

        if (fabs(speed) < 0.05f) {
            speed = 0.0f; // Dead zone
        }

        // Soft Start
        if (speed > currentSpeed + maxDelta) {
            currentSpeed += maxDelta;
        } else if (speed < currentSpeed - maxDelta)
        {
            currentSpeed -= maxDelta;
        } else {
            currentSpeed = speed;
        }
        
        if (currentSpeed > 0) {
            digitalWrite(dirPin, HIGH);
        } else if (currentSpeed < 0) {
            digitalWrite(dirPin, LOW);
        }

        // Calculate PWM value
        float absSpeed = abs(currentSpeed);
        int pwmValue = static_cast<int>(absSpeed * 255.0f);
        analogWrite(pwmPin, pwmValue);
    }

    void enable() {
        digitalWrite(enPin, HIGH);
    }

    void disable() {
        digitalWrite(enPin, LOW);
        analogWrite(pwmPin, 0);
        currentSpeed = 0.0f;
    }

    void stop() {
        setSpeed(0.0f);
        analogWrite(pwmPin, 0);
    }

    void setMaxDelta(float delta) {
        maxDelta = delta;
    }
private:
    int pwmPin;
    int dirPin;
    int enPin;
    float currentSpeed;
    float maxDelta;
};

class SteeringController {
public:
    SteeringController(int pin, int centerDeg, int maxDeltaDeg)
        : pin(pin), center(centerDeg), maxDelta(maxDeltaDeg) {
        // Constructor implementation
    }

    void begin() {
        servo.attach(pin);
        setSteering(0.0f);
    }

    void setSteering(float steer) {
        steer = constrain(steer, -1.0f, 1.0f);
        int angle = center + static_cast<int>(steer * maxDelta);
        angle = constrain(angle, center - maxDelta, center + maxDelta);
        servo.write(angle);
    }

    void setAngle(int degree) {
        servo.write(degree);
    }
private:
    int pin;
    int center;
    int maxDelta;
    Servo servo;
};

Motor leftMotor(L_PWM_PIN, L_DIR_PIN, L_EN_PIN);
Motor rightMotor(R_PWM_PIN, R_DIR_PIN, R_EN_PIN);
SteeringController steering(STEERING_SERVO_PIN, 90, 30);

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 1000; // 1 second

const float FWD_SPEED = 0.6f;
const float REV_SPEED = -0.4f;
const float TURN_SPEED = 0.4f;
const float TURN_STEER = 0.7f;

void processBluetooth();
void handleCommand(const string& cmd);
void driveStop();
void driveForward();
void driveBackward();
void driveTurnLeft();
void driveTurnRight();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600); // HC-06

    leftMotor.begin();
    rightMotor.begin();
    steering.begin();

    leftMotor.enable();
    rightMotor.enable();

    driveStop();

    Serial.println("Setup complete.");
}

void loop() {
    processBluetooth();

    // Fail-safe: stop if no command received recently
    if(millis() - lastCmdTime > CMD_TIMEOUT) {
        driveStop();
    }
}

void processBluetooth() {
    if (Serial1.available() > 0) {
        string cmd = Serial1.readStringUntil('\n');
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

void handleCommand(const string& cmd) {
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
    leftMotor.setSpeed(0.0f);
    rightMotor.setSpeed(0.0f);
    steering.setSteering(0.0f);
}

void driveForward() {
    steering.setSteering(0.0f);
    leftMotor.setSpeed(FWD_SPEED);
    rightMotor.setSpeed(FWD_SPEED);
}

void driveBackward() {
    steering.setSteering(0.0f);
    leftMotor.setSpeed(REV_SPEED);
    rightMotor.setSpeed(REV_SPEED);
}

void driveTurnLeft() {
    steering.setSteering(-TURN_STEER);
    leftMotor.setSpeed(TURN_SPEED * 0.5f);
    rightMotor.setSpeed(TURN_SPEED);
}

void driveTurnRight() {
    steering.setSteering(TURN_STEER);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED * 0.5f);
}