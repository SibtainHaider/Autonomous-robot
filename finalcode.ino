#include <Servo.h>

char val;

const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 4; // Left motor Direction 2
const int in3Pin = 2; // Right motor Direction 1
const int in4Pin = 3; // Right motor Direction 2
const int enBPin = 5; // Right motor PWM speed control

// ultrasonic sensor and servo motor pins
const int trigPin = 11;
const int echoPin = 12;
const int servoPin = 8;

// IR sensor pins
const int leftSensorPin = A5; // Analog pin for left IR sensor
const int rightSensorPin = A0; // Analog pin for right IR sensor

// threshold values for sensor readings
const int threshold = 500; 

// obstacle detection parameters
const int obstacleThreshold = 35; // Distance in centimeters

Servo ultrasonicServo;

void setup() {
  // Motor control pins as output
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  digitalWrite(9,LOW);
  pinMode(9, OUTPUT);

  // Ultrasonic sensor and servo motor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  ultrasonicServo.attach(servoPin);

    // IR sensor pins as inputs
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

}
void loop() {
  while (Serial.available() > 0) {   // For input over bluetooth device
    val = Serial.read();
    Serial.println(val);
  } 

  if (val == '1') {        // Obstacle Avoider
    digitalWrite(9, HIGH); 
  // Rotate the servo to scan the area
  for (int angle = 0; angle <= 180; angle += 45) {
    ultrasonicServo.write(angle);
    delay(300);
    int distance = getDistance();

    // Check if an obstacle is detected
    if (distance < obstacleThreshold) {
      // If obstacle detected, avoid
      avoidObstacle();
      break; // Exit the loop to avoid continuous scanning
    }
  }

  // If no obstacle is found, move forward
  moveForward();

} 
 else if (val == '2') {   // Line following
    // Read sensor values
  int leftSensorValue = analogRead(leftSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  // Compare sensor values with threshold
  if (leftSensorValue > threshold && rightSensorValue > threshold) {
    // Both sensors are on the line - move forward
    moveForward();
  } else if (leftSensorValue > threshold) {
    // Left sensor is on the line - turn right
    turnRight();
  } else if (rightSensorValue > threshold) {
    // Right sensor is on the line - turn left
    turnLeft();
  } else {
    // Both sensors are off the line - stop
    stopMotors();
  }
 }
  
else{

  stopMotors();

}
}

void avoidObstacle() {
  stopMotors();
  delay(500); // Delay for 0.5 seconds
  reverse();
  delay(1000); // Delay for 1 second
  turnLeft();
  delay(1000); // Delay for 1 second
  stopMotors();
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) * 0.034 / 2; 
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enAPin, 80); 
  analogWrite(enBPin, 80); 
}

// Function to stop the robot
void stopMotors() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
}

// Function to turn the robot left
void turnRight() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enAPin, 80); 
  analogWrite(enBPin, 80); 
}

// Function to turn the robot right
void turnLeft() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enAPin, 80); 
  analogWrite(enBPin, 80);
}
// Function to move the robot backward
void reverse() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enAPin, 80); 
  analogWrite(enBPin, 80); 
}