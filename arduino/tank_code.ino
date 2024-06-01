#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4 for Motor 1.
CytronMD motor2(PWM_DIR, 9, 8);  // PWM = Pin 6, DIR = Pin 7 for Motor 2.

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bits per second.

  while (Serial.available() > 0) {
    char junk = Serial.read(); // Read and discard bytes
  }
}

void loop() {
  if (Serial.available() > 0) {  // Check if data is available to read.
    char command = Serial.read(); // Read the incoming byte.

    switch (command) {
      case 's':  // Stop
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;
      case 'f':  // Forward
        motor1.setSpeed(50);
        motor2.setSpeed(-50);
        break;
      case 'r':  // Spin Left
        char t;
        while (true)
        {  
        t= Serial.read();
            motor1.setSpeed(50);  // Spin Left
            motor2.setSpeed(50);  //
            delay(100);
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            delay(100);
            if(t!='r' and t!=NULL)
            break;
        }
        break;
      case 'l':  // Spin Left
          char p;
          while (true)
          {  
          p= Serial.read();
              motor1.setSpeed(-50);  // Spin Left
              motor2.setSpeed(-50);  //
              delay(100);
              motor1.setSpeed(0);
              motor2.setSpeed(0);
              delay(100);
              if(p!='l' and p!=NULL)
              break;
          }
          break;
      case 'b':  // Backward
        motor1.setSpeed(-50);
        motor2.setSpeed(50);
        break;
      case 'q':  // Spin Left
        motor1.setSpeed(-50);
        motor2.setSpeed(-50);
        break;
      case 'e':  // Spin Right
        motor1.setSpeed(50);
        motor2.setSpeed(50);
        break;
      case 'c':  // Turn Right
        motor1.setSpeed(50);
        motor2.setSpeed(0);
        break;
      case 'z':  // Turn Left
        motor1.setSpeed(0);
        motor2.setSpeed(-50);
        break;
      default:   // Optional: handle unexpected characters
        motor1.setSpeed(-12);
        motor2.setSpeed(-12);
        delay(100);
        motor1.setSpeed(0);
        motor2.setSpeed(0);        
        break;
    }
  }
}
