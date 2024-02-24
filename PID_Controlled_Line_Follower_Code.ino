// Motor control pins for the left side
#define LEFT_MOTOR1 2
#define LEFT_MOTOR2 3

// Motor control pins for the right side
#define RIGHT_MOTOR1 4
#define RIGHT_MOTOR2 5

#define ENA 10
#define ENB 9

// IR Sensor Pins
#define LEFT_SENSOR A3
#define MIDDLE_SENSOR A4
#define RIGHT_SENSOR A5

float Kp = 4.35;  
float Ki = 0.0;
float Kd = 0.0;
float PIDvalue = 0;
float P = 0, I = 0, D = 0;
float previousError = 0;

double leftProportional, rightProportional;

int error=0;

double setPoint = 1023;  

void setup() {
  // Configure motor control pins as outputs
  pinMode(LEFT_MOTOR1, OUTPUT);
  pinMode(LEFT_MOTOR2, OUTPUT);
  pinMode(RIGHT_MOTOR1, OUTPUT);
  pinMode(RIGHT_MOTOR2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(MIDDLE_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  int leftSensorValue = analogRead(LEFT_SENSOR);
  int middleSensorValue = analogRead(MIDDLE_SENSOR);
  int rightSensorValue = analogRead(RIGHT_SENSOR);

  leftProportional = Kp * (setPoint - leftSensorValue);
  rightProportional = Kp * (setPoint - rightSensorValue);

  // Display sensor values on the serial monitor
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(" | Middle Sensor: ");
  Serial.print(middleSensorValue);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightSensorValue);

  // Implement line-following logic based on sensor readings
  if ((leftSensorValue < 165 && middleSensorValue >= 165 && rightSensorValue < 165) || (leftSensorValue >= 165 && middleSensorValue >= 165 && rightSensorValue >= 165)) {
    error=0;
    calculatePID();
    motorPIDcontrol();
    moveForward(leftProportional, rightProportional);
  } else if ((leftSensorValue >= 165 && middleSensorValue < 165 && rightSensorValue < 165) || (leftSensorValue >= 165 && middleSensorValue >= 165 && rightSensorValue < 165)) {
    error=-1;
    calculatePID();
    turnRight();
  } else if ((leftSensorValue < 165 && middleSensorValue < 165 && rightSensorValue >= 165) || (leftSensorValue < 165 && middleSensorValue >= 165 && rightSensorValue >= 165)) {
    error=-1;
    calculatePID();
    turnLeft();
  } else {
    stopMotors();
  }
}

// Rest of your motor control functions remain the same

// Function to move the car forward
void moveForward(double leftSpeed, double rightSpeed) {
  analogWrite(ENA, 60); // Set the enable (speed) to 55
  analogWrite(ENB, 55); // Set the enable (speed) to 55
  analogWrite(LEFT_MOTOR1, leftSpeed);
  digitalWrite(LEFT_MOTOR2, LOW);
  analogWrite(RIGHT_MOTOR1, rightSpeed);
  digitalWrite(RIGHT_MOTOR2, LOW);
}

// Function to move the car backward
void moveBackward() {
  analogWrite(ENA, 60); // Set the enable (speed) to 55
  analogWrite(ENB, 55); // Set the enable (speed) to 55
  digitalWrite(LEFT_MOTOR1, LOW);
  digitalWrite(LEFT_MOTOR2, HIGH);
  digitalWrite(RIGHT_MOTOR1, LOW);
  digitalWrite(RIGHT_MOTOR2, HIGH);
}

// Function to turn the car left
void turnLeft() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 85); // Set the enable (speed) to 55
  digitalWrite(LEFT_MOTOR1, LOW);
  digitalWrite(LEFT_MOTOR2, LOW);
  digitalWrite(RIGHT_MOTOR1, HIGH);
  digitalWrite(RIGHT_MOTOR2, LOW);
}

// Function to turn the car right
void turnRight() {
  analogWrite(ENA, 85); // Set the enable (speed) to 55
  analogWrite(ENB, 0);
  digitalWrite(LEFT_MOTOR1, HIGH);
  digitalWrite(LEFT_MOTOR2, LOW);
  digitalWrite(RIGHT_MOTOR1, LOW);
  digitalWrite(RIGHT_MOTOR2, LOW);
}

// Function to stop the motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(LEFT_MOTOR1, LOW);
  digitalWrite(LEFT_MOTOR2, LOW);
  digitalWrite(RIGHT_MOTOR1, LOW);
  digitalWrite(RIGHT_MOTOR2, LOW);
}

void motorPIDcontrol() {
  int leftMotorSpeed = leftMotorSpeed + PIDvalue;
  int rightMotorSpeed = rightMotorSpeed - PIDvalue;

  constrain(leftMotorSpeed, 55, 90);
  constrain(rightMotorSpeed, 55, 90);

}

void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}
