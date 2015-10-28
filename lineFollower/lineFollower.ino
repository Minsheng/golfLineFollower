#include <QTRSensors.h>

// The PID control algorithm is based on the Advanced Line Following Algorithm with 3pi robot,
// please refer to https://www.pololu.com/docs/0J21/7.c for original code.

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

#define DEBUG 0 // if DEBUG is on, only test motor and/or sensors
#define INFO 0 // if INFO is on, print debug info

/* For PID control */
#define Kp 0.05 // proportional constant
#define Ki 0.0001 // integral constant
#define Kd 1.5 // derivative constant
#define aggrKp 4 // aggressive proportional constant
#define aggrKd 0.2 // aggressive integral constant
#define aggrKi 1 // aggressive derivative constant

/* For QTR sensors */
#define NUM_SENSORS 6 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading

/* For motor controll */
#define STBY 10 //standby

/* motor A connected between A01 and A02 - Left Motor */
#define PWMA 5
#define AIN1 9
#define AIN2 8
/* motor B connected between B01 and B02 - Right Motor */
#define PWMB 6
#define BIN1 11
#define BIN2 12

#define MAX_SPEED 90

/* sensors 0 through 5 are connected to analog inputs 0 through 5, respectively */
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

int lastError = 0;

void setup() {
  /* Set up motor controll pins as output */
  pinMode(STBY, OUTPUT);
  pinMode(AIN1,OUTPUT);        
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT);        
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);

  if (!DEBUG) {
    // Auto-calibration: turn right and left while calibrating the
    // sensors.
    for (int counter=0; counter<100; counter++) {
      if (counter < 20 || counter >= 60)
        set_motors(50,0);
      else
        set_motors(0,50);
  
      // This function records a set of sensor readings and keeps
      // track of the minimum and maximum values encountered.  The
      // IR_EMITTERS_ON argument means that the IR LEDs will be
      // turned on during the reading, which is usually what you
      // want.
      qtra.calibrate();
  
      // the total delay is (# of loops)*(delay time/loop),
      // i.e., 100*20 = 2000 ms.
      delay(20);
    }
    
    set_motors(0,0); // stop the motor for a second
  }
  
  Serial.begin(9600);
}

void loop() {
  if (DEBUG == 1) {
    // enable the following functions as needed
//    testSensor();
//    testMotor();
  } else {
    // execute main function
    lineFollowDrive();
  }
}

void lineFollowDrive() {
  unsigned int position = 0;
  long integral = 0;
  int power_difference = 0;
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  
  // get calibrated readings along with the line position, 
  // refer to the QTR Sensors Arduino Library for more details on line position.
  position = qtra.readLine(sensorValues);
  
//  Serial.println(position);
  
  // The "error" term should be 0 when we are on the line.
  // i.e., if there are 6 sensors and the readings are from (0~5000),
  // the error will be -2500 ~ 2500
  int error = (int) position - 2500;

  // Compute the derivative (change), which determines the rate of change of
  // the error value
  int derivative = error - lastError;

  // Compute the integral (sum) of the previous position values,
  // which records the history of the robot's motion
  integral += error;

  // Remember the last position of the robot
  lastError = error;

  if (INFO == 1) {
    Serial.print("Current Error: ");
    Serial.print(error);
    Serial.print(" | ");
    Serial.print("Integral: ");
    Serial.print(integral);
    Serial.print(" | ");
    Serial.print("Deriva: ");
    Serial.print(derivative);
    Serial.println();
  }

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  power_difference = error*Kp + integral*Ki + derivative*Kd;

  // Aggressive turning alogrithm, needs to be tested out
//  if (derivative <= -1300 || derivative >= 1300) {
//    Serial.println("Big Gap!");
//    power_difference = error*aggrKp + integral*aggrKi + derivative*aggrKd;
//  } else {
//    power_difference = error*Kp + integral*Ki + derivative*Kd;
//  }
  
  if (INFO == 1) {
    Serial.println(power_difference);
  }
  
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if(power_difference > MAX_SPEED)
    power_difference = MAX_SPEED;
  
  if(power_difference < -MAX_SPEED)
    power_difference = -MAX_SPEED;

  if (power_difference < 0) {
    leftMotorSpeed = MAX_SPEED+power_difference;
    rightMotorSpeed = MAX_SPEED;
  } else {
    leftMotorSpeed = MAX_SPEED;
    rightMotorSpeed = MAX_SPEED-power_difference;
  }
  
  set_motors(leftMotorSpeed, rightMotorSpeed);
      
  if (INFO == 1) {
    Serial.print(leftMotorSpeed);
    Serial.print('\t');
    Serial.print(rightMotorSpeed);
    Serial.println();
  }
}

/* Drive the robot by enabling power in left and right motors */
void set_motors(int leftMotorSpeed, int rightMotorSpeed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);
}

/* Read raw sensor values from the QTR Reflectance Sensors */
void testSensor() {
  // read raw sensor values
  qtra.read(sensorValues);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  
  delay(250);  
}

/* Test motors */
void testMotor() {
  int testSpeed = 60;
  digitalWrite(STBY, HIGH);
  // Left motor
  analogWrite(PWMA, testSpeed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // Right motor
  digitalWrite(PWMB, testSpeed);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}
