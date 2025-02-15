#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include "Adafruit_VL53L0X.h"

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 16
#define SHT_LOX2 2
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X lidarLeft;
VL53L0X lidarRight;
#define MPU_INT 5 // MPU6050 Interrupt Pin
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
// Define PID Constants
double KP = 3.5;  // Proportional gain
double KI = 0.02;  // Integral gain (NEW - Corrects long-term drift)
double KD = 1.8; 
double initialAngle = 0;  // To store the initial heading
double integral = 0;  // Integral error storage
double prevError = 0; // Stores last error for derivative calculation

#define ENA 32  
#define ENB 33  
#define IN1 25  
#define IN2 26  
#define IN3 27  
#define IN4 14  

#define DECODER_C1 13  // Decoder 1 C1
#define DECODER_C2 23  // Decoder 1 C2
#define DECODER2_C1 4  // Decoder 2 C1
#define DECODER2_C2 15 // Decoder 2 C2

#define TRIG_PIN 17 // Ultrasonic Trigger
#define ECHO_PIN 19 // Ultrasonic Echo
int sensor1, sensor2;
#define BASE_SPEED 100      
#define ERROR_CORRECTION 5 
#define OBSTACLE_DISTANCE 50 // Distance threshold in mm (5 cm)
#define OBSTACLE_DISTANCELL 70 // Distance threshold in mm (5 cm)
void setID() {
  // All reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // All unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // Activating LOX1 and resetting others
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // Initializing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  // Activating LOX2 and resetting others
  digitalWrite(SHT_LOX2, HIGH);


  // Initializing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }
  delay(10);



}

MPU6050 mpu;
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    pinMode(DECODER_C1, INPUT_PULLUP);
    pinMode(DECODER_C2, INPUT_PULLUP);
    pinMode(DECODER2_C1, INPUT_PULLUP);
    pinMode(DECODER2_C2, INPUT_PULLUP);
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
while (!Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println("All sensors in reset mode...(pins are low)");

  Serial.println("Starting...");
  setID();
    Serial.println("Two LiDAR Sensors Initialized Successfully!");
    Serial.println("Motors and Sensors Initialized");


    mpu.initialize();
if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
} else {
    Serial.println("MPU6050 initialized successfully.");
}
   // Set initial reference angle
    initialAngle = getRotationAngle();
}
// Function to move forward
void moveForward(int speed_R, int speed_L) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed_R);
    analogWrite(ENB, speed_L);
}
void stopMotors() {

    // Set both input pins LOW to cut power (coasting stop)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // Ensure no PWM signal
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
      Serial.print("MANNNNNNNNNNNNNNNNNNNNN! WORK!");
}
double absoluteAngle = 0;
double gyroZ_offset = 19;  // Use your calculated offset

double getRotationAngle() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double gyroZ = (gz - gyroZ_offset) / 131.0;  // Apply offset correction
    absoluteAngle += gyroZ * 0.05;  // Reduce drift with a smaller integration step

    return absoluteAngle;
}





// Function to measure distance using ultrasonic sensor
long getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = duration * 0.034 / 2; // Convert to mm
    return distance;
}
void moveForward2(int speed_L, int speed_R) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed_R);
    analogWrite(ENB, speed_L);
}
void moveForwardPID() {
    int baseSpeed = BASE_SPEED;  
    int baseSpeedL = BASE_SPEED - 1.15;
    int minSpeed = 40;  // Prevents wheels from stopping

    integral = 0;  // Reset integral
    absoluteAngle = 0;  // Reset absolute angle
    initialAngle = getRotationAngle();  // Get correct reference angle

    unsigned long startTime = millis();  // *Start the timer*

    while (millis() - startTime < 500) {  // *Run for 3 seconds only*
        double currentAngle = getRotationAngle();
        double error = currentAngle - initialAngle;  

        Serial.print("Current Angle: "); Serial.println(currentAngle);  // Debugging
        
        // *Integral Term*: Accumulates small errors over time
        integral += error;  
        
        // *Derivative Term*: How fast the error is changing
        double derivative = error - prevError;  
        prevError = error;  // Store for next loop

        // *PID Correction*
        double correction = (KP * error) + (KI * integral) + (KD * derivative);

        // *Balanced motor speeds*
        int speed_L = constrain(baseSpeedL - correction - 1.15, minSpeed, 85);  
        int speed_R = constrain(baseSpeed - correction  , minSpeed, 85);

        moveForward(speed_R, speed_L);  

        Serial.print("Error: "); Serial.print(error);
        Serial.print(" | Integral: "); Serial.print(integral);
        Serial.print(" | Speed L: "); Serial.print(speed_L);
        Serial.print(" | Speed R: "); Serial.println(speed_R);

        delay(50);  // Small delay for stability
    }

    stopMotors();  // *Ensure motors stop after 3 seconds*
    Serial.println("Move forward complete!");
}



void moveRight(int speed_L) {
    digitalWrite(IN1, LOW);  // Right motor stops
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  // Left motor moves forward
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 0);
    analogWrite(ENB, speed_L);
}
void moveRightPID() {
    int baseSpeed = BASE_SPEED;  
    int baseSpeedL = BASE_SPEED - 2;
    int minSpeed = 60;  // Prevents wheels from stopping

    double KP_turn = 2.8;  // Proportional gain
    double KI_turn = 0.05;  // Integral gain
    double KD_turn = 1.1;  // Derivative gain

    double integral_turn = 0;
    double prevError_turn = 0;

    double startAngle = getRotationAngle();
    double targetAngle = startAngle + 90;  // 90-degree turn

    Serial.println("Starting Right Turn (PID)...");

    unsigned long startTime = millis(); // Timeout protection

    while (millis() - startTime < 380) {  // Max 5 seconds to complete the turn
        double currentAngle = getRotationAngle();
        double error = targetAngle - currentAngle;

        Serial.print("Current Angle: "); Serial.print(currentAngle);
        Serial.print(" | Target: "); Serial.print(targetAngle);
        Serial.print(" | Error: "); Serial.println(error);

        if (abs(error) <= 1.0) {
            break;  // Exit loop when the angle is reached
        }

        // *Integral Term*: Accumulate small errors
        integral_turn += error;  
        integral_turn = constrain(integral_turn, -60, 60);  // Prevents integral windup

        // *Derivative Term*: Change in error
        double derivative_turn = error - prevError_turn;
        prevError_turn = error;

        // *PID Correction*
        double correction = (KP_turn * error) + (KI_turn * integral_turn) + (KD_turn * derivative_turn);

        // *Separate motor speeds*
        int speed_L = constrain(baseSpeedL + correction, minSpeed, 100);  // Left motor moves faster
        int speed_R = constrain(baseSpeed - correction, minSpeed, 100);  // Right motor moves slower or in reverse

        moveRight(speed_L);  // Apply motor speeds

        Serial.print(" | Speed L: "); Serial.print(speed_L);
        Serial.print(" | Speed R: "); Serial.println(speed_R);

        delay(20);  // Small delay for smooth control
    }
    stopMotors();
          delay(1000);  // Small delay for stability
}
void moveLeft(int speed_L) {
    analogWrite(ENA, speed_L);  // Left motor moves
    analogWrite(ENB, 0);  // Right motor completely OFF

    digitalWrite(IN1, HIGH);  // Left motor forward
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);  // Right motor stays OFF
    digitalWrite(IN4, LOW);
}



void moveLeftPID() {
     int baseSpeed = BASE_SPEED;  
    int baseSpeedL = BASE_SPEED - 2;
    int minSpeed = 60;  // Prevents wheels from stopping

    double KP_turn = 2.3;  // Proportional gain
    double KI_turn = 0.05;  // Integral gain
    double KD_turn = 1.8;  // Derivative gain

    double integral_turn = 0;
    double prevError_turn = 0;

    double startAngle = getRotationAngle();
    double targetAngle = startAngle - 90;  // 90-degree turn

    Serial.println("Starting Right Turn (PID)...");

    unsigned long startTime = millis(); // Timeout protection

    while (millis() - startTime <330) {  // Max 5 seconds to complete the turn
        double currentAngle = getRotationAngle();
        double error = targetAngle - currentAngle;

        Serial.print("Current Angle: "); Serial.print(currentAngle);
        Serial.print(" | Target: "); Serial.print(targetAngle);
        Serial.print(" | Error: "); Serial.println(error);

        if (abs(error) <= 1.0) {
            break;  // Exit loop when the angle is reached
        }

        // *Integral Term*: Accumulate small errors
        integral_turn += error;  
        integral_turn = constrain(integral_turn, -60, 60);  // Prevents integral windup

        // *Derivative Term*: Change in error
        double derivative_turn = error - prevError_turn;
        prevError_turn = error;

        // *PID Correction*
        double correction = (KP_turn * error) + (KI_turn * integral_turn) + (KD_turn * derivative_turn);

        // *Separate motor speeds*
        int speed_L = constrain(baseSpeedL + correction, minSpeed, 100);  // Left motor moves faster
        int speed_R = constrain(baseSpeed - correction, minSpeed, 100);  // Right motor moves slower or in reverse

        moveLeft(speed_R);  // Apply motor speeds

        Serial.print(" | Speed L: "); Serial.print(speed_L);
        Serial.print(" | Speed R: "); Serial.println(speed_R);

        delay(20);  // Small delay for smooth control
    }
    stopMotors();
          delay(1000);  // Small delay for stability
}



void moveBackwardPID() {
    int baseSpeed = BASE_SPEED;  
    int baseSpeedL = BASE_SPEED - 2;
    int minSpeed = 60;  // Prevents wheels from stopping

    integral = 0;  // Reset integral
    absoluteAngle = 0;  // Reset absolute angle
    initialAngle = getRotationAngle();  // Get correct reference angle

    unsigned long startTime = millis();  // *Start the timer*

    while (millis() - startTime < 500) {  // *Run for 3 seconds only*
        double currentAngle = getRotationAngle();
        double error = currentAngle - initialAngle;  

        Serial.print("Current Angle: "); Serial.println(currentAngle);  // Debugging
        
        // *Integral Term*: Accumulates small errors over time
        integral += error;  
        
        // *Derivative Term*: How fast the error is changing
        double derivative = error - prevError;  
        prevError = error;  // Store for next loop

        // *PID Correction*
        double correction = (KP * error) + (KI * integral) + (KD * derivative);

        // *Balanced motor speeds*
        int speed_L = constrain(baseSpeedL - correction - 2, minSpeed, 85);  
        int speed_R = constrain(baseSpeed - correction , minSpeed, 85);

        moveForward2(speed_L, speed_R);  

        Serial.print("Error: "); Serial.print(error);
        Serial.print(" | Integral: "); Serial.print(integral);
        Serial.print(" | Speed L: "); Serial.print(speed_L);
        Serial.print(" | Speed R: "); Serial.println(speed_R);

        delay(50);  // Small delay for stability
    }

    stopMotors();  // *Ensure motors stop after 3 seconds*
    Serial.println("Move forward complete!");
}

double leftlidarreading() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
   sensor1 = measure1.RangeMilliMeter;    
    double leftmea = (sensor1 / 10.0) - 3;  // Convert to cm & adjust for offset
    Serial.print(" | Converted: ");
    Serial.print(leftmea);
    Serial.println(" cm");

    return leftmea;
}

double rightlidarreading() {
lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
   sensor2 = measure2.RangeMilliMeter;    
    double rightmea = (sensor2 / 10.0) - 3;  // Convert to cm & adjust for offset
    Serial.print(" | Converted: ");
    Serial.print(rightmea);
    Serial.println(" cm");

    return rightmea;
}



void loop() {
    delay(200);
bool usingRightHand = true;  // Default to right-hand rule

    // Read sensor values
    double leftDistance = leftlidarreading();
    delay(100);
    double rightDistance = rightlidarreading();
    delay(100);
    double frontDistance = getUltrasonicDistance();
    delay(100);

    // Debugging output
    Serial.print("Left: "); Serial.print(leftDistance);
    Serial.print(" | Right: "); Serial.print(rightDistance);
    Serial.print(" | Front: "); Serial.println(frontDistance);

    if (usingRightHand) {  
        // * Right-Hand Rule *
        if (rightDistance > 5) {
            Serial.println("No wall on the right. Turning right...");
            moveRightPID(); // Calls function for precise 90° turn
        } 
        else if (!(frontDistance > 0 && frontDistance <= (OBSTACLE_DISTANCE / 6))) {
            Serial.println("No wall in front. Moving forward...");
            moveForwardPID();
            delay(500);  // Moves forward by 10 cm
        } 
        else if (!(leftDistance > 0 && leftDistance <= 5)) {
            Serial.println("No wall on the left. Turning left...");
            moveLeftPID();  // Calls function for precise 90° turn
        } 
        else {
    Serial.println("Dead-end! Moving backward...");
    moveBackwardPID();
    delay(1000);  // Adjust delay for 10 cm backward movement
    stopMotors();
     usingRightHand = false;
}

    } 
    else {  
        // * Left-Hand Rule *
        if (leftDistance > 5) {
            Serial.println("No wall on the left. Turning left...");
            moveLeftPID(); 
        } 
        else if (!(frontDistance > 0 && frontDistance <= (OBSTACLE_DISTANCE / 6))) {
            Serial.println("No wall in front. Moving forward...");
            moveForwardPID();
            delay(500);
        } 
        else if (!(rightDistance > 0 && rightDistance <= 5)) {
            Serial.println("No wall on the right. Turning right...");
           moveRightPID();
        } 
        else {
             moveBackwardPID();
    delay(1000);  // Adjust delay for 10 cm backward movement
    stopMotors();
            usingRightHand = true;
        }
    }

    stopMotors();
    delay(200);
}
