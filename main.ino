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

// TODO
// - negotiate with team to add the arm servos
// const int ARM_SERVO1_PIN = 9;
// const int ARM_SERVO2_PIN = 10;

// HC-06
const int TX = 18;
const int RX = 19;

// GY-61
const int ACC1_X_PIN = A10;
const int ACC1_Y_PIN = A11;
const int ACC1_Z_PIN = A12;

const int ACC2_X_PIN = A13;
const int ACC2_Y_PIN = A14;
const int ACC2_Z_PIN = A15;

// KY-020
const int TILT1_PIN = 30;
const int TILT2_PIN = 31;

class Motor {
public:
    Motor(int pwmPin, int dirPin, int enPin)
        : pwmPin(pwmPin), dirPin(dirPin), enPin(enPin),
        currentSpeed(0.0f), maxDelta(0.02f), deadZone(0.03f) { // Max delta per update
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

        if (fabs(speed) < deadZone) {
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
        currentSpeed = 0.0f;
        analogWrite(pwmPin, 0);
    }

    void setMaxDelta(float delta) {
        maxDelta = delta;
    }

    void setDeadZone(float zone) {
        deadZone = zone;
    }
private:
    int pwmPin;
    int dirPin;
    int enPin;
    float currentSpeed;
    float maxDelta;
    float deadZone;
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

    // steer: -1.0(left) 0.0(forward) 1.0(right)
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

// Intention variables
float targetThrottle = 0.0f;
float steerCmd = 0.0f;

// low-pass filter variables
float filteredThrottle = 0.0f;

float steeringFactor = 1.0f;
float vibrationFactor = 1.0f;

float vibrationLevel = 0.0f;

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 1000; // 1 second

const float FWD_SPEED = 0.6f;
const float BWD_SPEED = -0.4f;
const float TURN_SPEED = 0.4f;
const float TURN_STEER = 0.7f;

// throttle filter factor (larger to react faster)
const float THROTTLE_ALPHA = 0.05f;

// vibration filter factor (larger to react faster)
const float VIBRATION_ALPHA = 0.1f;

// steering slowdown rate (when reach limit then slow down to 50% of max speed)
const float STEERING_SLOWDOWN_MAX = 0.5f;

// vibration slowdown rate (when reach limit then slow down to 50% of max speed)
const float VIBRATION_SLOWDOWN_MAX = 0.5f;


void processBluetooth();
void handleCommand(const string& cmd);
void updateSensors();
void updateThrottle();
void updateDrive();
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

    pinMode(TILT1_PIN, INPUT);
    pinMode(TILT2_PIN, INPUT);

    leftMotor.enable();
    rightMotor.enable();

    driveStop();

    Serial.println("Setup complete.");
}

void loop() {
    processBluetooth();
    updateSensors();
    updateThrottle();
    updateDrive();

    // Fail-safe: stop if no command received recently
    if(millis() - lastCmdTime > CMD_TIMEOUT) {
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
    targetThrottle = 0.0f;
    steerCmd = 0.0f;
}

void driveForward() {
    targetThrottle = FWD_SPEED;
    steerCmd = 0.0f;
}

void driveBackward() {
    targetThrottle = BWD_SPEED;
    steerCmd = 0.0f;
}

void driveTurnLeft() {
    targetThrottle = TURN_SPEED * 0.5f;
    steerCmd = -TURN_STEER;
}

void driveTurnRight() {
    targetThrottle = TURN_SPEED * 0.5f;
    steerCmd = TURN_STEER;
}

void updateSensors() {
    // roughly calculate vibration level
    // read GY-61
    int ax1 = analogRead(ACC1_X_PIN);
    int ay1 = analogRead(ACC1_Y_PIN);
    int az1 = analogRead(ACC1_Z_PIN);
    int ax2 = analogRead(ACC2_X_PIN);
    int ay2 = analogRead(ACC2_Y_PIN);
    int az2 = analogRead(ACC2_Z_PIN);

    // how far from center?
    long devSum = abs(ax1 - 512) + abs(ay1 - 512) + abs(az1 - 512) + abs(ax2 - 512) + abs(ay2 - 512) + abs(az2 - 512);
    
    // heuristic value. lately should be tuned.
    float instantLevel = devSum / 4000.0f;
    instantLevel = constrain(instantLevel, 0.0f, 1.5f);
    if (instantLevel > 1.0f) {
        instantLevel = 1.0f;
    }

    // KY-020
    bool tilt1 = (digitalRead(TILT1_PIN) == LOW);
    bool tilt2 = (digitalRead(TILT2_PIN) == LOW);
    if (tilt1 || tilt2) {
        instantLevel = 1.0f;
    }

    // low-pass filter to smooth the value
    vibrationLevel = vibrationLevel * VIBRATION_ALPHA + (instantLevel - vibrationLevel);

    vibrationFactor = 1.0f - VIBRATION_SLOWDOWN_MAX * vibrationLevel;
    vibrationFactor = constrain(vibrationFactor, 0.5f, 1.0f);

    Serial.print("Vibration level: ");
    Serial.println(vibrationLevel);
    Serial.print("Vibration factor: ");
    Serial.println(vibrationFactor);
}

void updateThrottle() {
    // targetThrottle + filteredThrottle (low-pass filter)
    filteredThrottle = filteredThrottle * THROTTLE_ALPHA + (targetThrottle - filteredThrottle);
}

void updateDrive() { // for real motor command
    // steering slowdown rate
    float absSteer = fabs(steerCmd);
    steeringFactor = 1.0f - STEERING_SLOWDOWN_MAX * absSteer;
    steeringFactor = constrain(steeringFactor, 0.5f, 1.0f);

    // limit factor (apply hard regulation)
    float limitFactor = min(steeringFactor, vibrationFactor);

    // final throttle (apply limit factor)
    float finalThrottle = filteredThrottle * limitFactor;

    float steerGain = 0.5f; // larger to turn more sharply, smaller to turn more gently
    float leftCmd = finalThrottle - steerGain * steerCmd;
    float rightCmd = finalThrottle + steerGain * steerCmd;

    leftCmd = constrain(leftCmd, -1.0f, 1.0f);
    rightCmd = constrain(rightCmd, -1.0f, 1.0f);

    steering.setSteering(steerCmd);
    leftMotor.setSpeed(leftCmd);
    rightMotor.setSpeed(rightCmd);
}