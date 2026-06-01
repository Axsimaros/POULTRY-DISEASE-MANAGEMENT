#include <Servo.h>

Servo leftservo;
Servo rightservo;

// Sensor pins
#define leftsensor A0 
#define rightsensor A1

// Servo pins
int leftpin = 6;
int rightpin = 5;

// Neutral microseconds (stop position)
int leftneutral = 1500;
int rightneutral = 1490;

// PID constants
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.15;

// Base speed
int baseSpeed = 100;

// Variables
float error = 0, lastError = 0, integral = 0, derivative = 0;
float total = 0;
int leftValue, rightValue;
int threshold = 400;  // white-black separation
int lastDirection = 0; // -1 = left, 1 = right

// ======= Setup =======
void setup() {
  Serial.begin(9600);
  leftservo.attach(leftpin);
  rightservo.attach(rightpin);
}

// ======= Read average function =======
int readAverage(int pin) {
  float total = 0;
  for (int i = 0; i < 50; i++) {
    total += analogRead(pin);
  }
  return total / 50;
}

// ======= Main loop =======
void loop() {
  leftValue = readAverage(leftsensor);
  rightValue = readAverage(rightsensor);

  // Calculate error
  error = (leftValue - rightValue);

  // PID calculations
  integral += error;
  derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // ====== Line Lost Detection ======
  if (leftValue < threshold && rightValue < threshold) {
    Serial.println("Line lost! Recovering...");

    // Move backward a bit
    leftservo.writeMicroseconds(leftneutral - 80);
    rightservo.writeMicroseconds(rightneutral + 80);
    delay(150);

    // Turn to last known direction
    if (lastDirection == -1) {
      // Turn left to search
      leftservo.writeMicroseconds(leftneutral - 100);
      rightservo.writeMicroseconds(rightneutral - 100);
    } else if (lastDirection == 1) {
      // Turn right to search
      leftservo.writeMicroseconds(leftneutral + 100);
      rightservo.writeMicroseconds(rightneutral + 100);
    }

    delay(150);
  } 
  else {
    // ====== Normal PID Control ======
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftservo.writeMicroseconds(leftneutral + leftSpeed);
    rightservo.writeMicroseconds(rightneutral - rightSpeed);

    // Remember last direction
    if (error > 0) lastDirection = -1; // line on left
    else if (error < 0) lastDirection = 1; // line on right
  }

  // ====== Serial Monitor Output ======
  Serial.print("Left: "); Serial.print(leftValue);
  Serial.print("  Right: "); Serial.print(rightValue);
  Serial.print("  Error: "); Serial.print(error);
  Serial.print("  Dir: "); Serial.println(lastDirection);

  lastError = error;
  delay(20);
}
