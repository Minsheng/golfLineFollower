/*
** Line Follower Basic v. 0.5
** Last Update: 2013-05-21
*/

#include <Servo.h>

// the threshold for black line
#define THRESHOLD 1020
#define DEBUG 1
#define SHARPTURN 1

/* Define motor controll inputs */
//motor A connected between A01 and A02 - Left Motor
//motor B connected between B01 and B02 - Right Motor
#define STBY 10 //standby

//Motor A
#define PWMA 5 //Speed control
#define AIN1 9 //Direction
#define AIN2 8 //Direction

//Motor B
#define PWMB 6 //Speed control
#define BIN1 11 //Direction
#define BIN2 12 //Direction

/* Define the pins for the IR receivers */
int irPins[5] = {A0, A1, A2, A3, A4};

/* Define values for the IR Sensor readings */

// an array to hold values from analogRead on the ir sensor (0-1023)
int irSensorAnalog[5] = {0,0,0,0,0};

// an array to hold boolean values (1/0) for the ir sensors, based on the analog read and the predefined THRESHOLD
int irSensorDigital[5] = {0,0,0,0,0}; 

// binary representation of the sensor reading from left to right
int irSensors = B00000;

// a score to determine deviation from the line [-180 ; +180]. Negative means the robot is left of the line.
int error = 0;

//  store the last value of error
int errorLast = 0;

int turnMode = 0;

/* Set up maximum speed and speed for turning (to be used with PWM) */
// PWM to control motor speed [0 - 255]
int maxSpeed = 100;
int fullSpeed = 255; // for very sharp turn

/* variables to keep track of current speed of motors */
int motorLSpeed = 90;
int motorRSpeed = 90;

void setup() {
  /* Set up motor controll pins as output */
  pinMode(STBY, OUTPUT);
  
  pinMode(AIN1,OUTPUT);        
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  
  pinMode(BIN1,OUTPUT);        
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  
  Serial.begin(115200);
}

void loop() {
  if (DEBUG == 1) {
    testMotor();
  } else {
    Scan();
    ErrorCorrection();
    Drive();  
  }
}

void testMotor() {
  digitalWrite(STBY, HIGH);
  // Left motor
  analogWrite(PWMA, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // Right motor
  digitalWrite(PWMB, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void Scan() {
  irSensors = B00000;
    
  for (int i = 0; i < 5; i++) {
    irSensorAnalog[i] = analogRead(irPins[i]);

    if (irSensorAnalog[i] >= THRESHOLD) {
      irSensorDigital[i] = 1;
    } else {
      irSensorDigital[i] = 0;
    }

//    if (DEBUG) {
//      Serial.print("A");
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.print(irSensorAnalog[i]);
//      Serial.print(" | ");
//    }

    // calculate binary representation for sensor values
    int b = 4-i;
    irSensors = irSensors + (irSensorDigital[i]<<b);
  }
}


void ErrorCorrection() {
  errorLast = error;  
  turnMode = 0; //reset turn mode
  
  switch (irSensors) {
    case B00000:
      if (errorLast < 0) {
        turnMode = 1;
        error = -fullSpeed;
        break;  
      } else if (errorLast > 0) {
        turnMode = 1;
        error = fullSpeed;
        break;
      }
      Serial.println("Out of track!Crap!");
      break;
    case B10000: // leftmost sensor on the line
      error = maxSpeed;
      Serial.println("Move left!");
      break;
    case B11000:
      error = maxSpeed*1.5;
      break;
    case B01000:
      error = maxSpeed*0.2;
      Serial.println("Move slightly left!");
      break;
    case B01100:
      error = maxSpeed*0.15;
      break;
    case B00100:
      error = 0;
      Serial.println("Move forward!");
      break;
    case B00110: // turn right slightly
      error = -maxSpeed*0.15;
      break;
    case B00010:
      error = -maxSpeed*0.2;
      Serial.println("Move slightly right!");
      break;
    case B00001: // rightmost sensor on the line
      error = -maxSpeed;
      Serial.println("Move right!");
      break;
    case B00011: 
      error = -maxSpeed*1.5;
      break;   
    case B11100: // turn left
      turnMode = 1;
      error = fullSpeed;
      break;
//      
//    case B01110:
//      if (errorLast > 0 && errorLast <= 255) {
//        error = maxSpeed;
//      } else {
//        error = -maxSpeed;
//      }
//      break;
//    
    case B00111:  // turn right
      turnMode = 1;
      error = fullSpeed;
      break;     
    case B11110:
      turnMode = 1;
      error = fullSpeed;
      break;
    case B01111:
      turnMode = 1;
      error = -fullSpeed;
      break;
    default:
      error = errorLast;
  }

  if (error >= 0) { // turn left (right motor speed > left motor speed)
    motorLSpeed = maxSpeed - error;
    motorRSpeed = maxSpeed;
  } else { // turn right (right motor speed < left motor speed)
    motorLSpeed = maxSpeed;
    motorRSpeed = maxSpeed + error;
  }
}

void Drive() {

  // recalculate speed for sharp turns
  if (turnMode == 1) {
    if (motorLSpeed <= 0 && motorRSpeed >= 0) {
      motorLSpeed = -fullSpeed*0.3;
      motorRSpeed = fullSpeed;
    } else {
      motorLSpeed = fullSpeed;
      motorRSpeed = -fullSpeed*0.3;
    }
  }
  
  // disable standby
  digitalWrite(STBY, HIGH);
  
  if (motorRSpeed > 0) { // right motor forward (using PWM)
     analogWrite(PWMB, motorRSpeed);
     digitalWrite(BIN1, HIGH);
     digitalWrite(BIN2, LOW);
  } else if (motorRSpeed < 0) { // right motor reverse (using PWM)
     analogWrite(PWMB, abs(motorRSpeed));
     digitalWrite(BIN1, LOW);
     digitalWrite(BIN2, HIGH);
  } else if (motorRSpeed == 0) { // right motor fast stop
     digitalWrite(PWMB, HIGH);
     digitalWrite(BIN1, LOW);
     digitalWrite(BIN2, LOW);
  }
  
  if (motorLSpeed > 0) { // left motor forward (using PWM)
     analogWrite(PWMA, motorLSpeed);
     digitalWrite(AIN1, HIGH);
     digitalWrite(AIN2, LOW);
  } else if (motorLSpeed < 0) { // left motor reverse (using PWM)
     analogWrite(PWMA, abs(motorLSpeed));
     digitalWrite(AIN1, LOW);
     digitalWrite(AIN2, HIGH);
  } else if (motorLSpeed == 0) { // left motor fast stop
     digitalWrite(PWMA, HIGH);
     digitalWrite(AIN1, LOW);
     digitalWrite(AIN2, LOW);
  }
}
