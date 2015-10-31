#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>

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
#define INFO 1 // if INFO is on, print debug info

int SHOOT_MODE = 0;
int INIT_NAV_MODE = 1;

/* For distance sensor */
#define ECHO_PIN 3
#define TRIG_PIN 4
#define MAX_DISTANCE 80
#define MAX_STOP_DISTANCE 20

/* For Servo Arms */
#define LEFT_SERVO_PIN 10
#define RIGHT_SERVO_PIN 9

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

#define MAX_SPEED 100 // maximum when driving
#define CALI_SPEED 50 // speed when calibrating

/* sensors 0 through 5 are connected to analog inputs 0 through 5, respectively */
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

/* Define distance sensor for wall detection */
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int uS; // distance raw value read from sonar

unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

int lastError = 0;

Servo leftArm;
Servo rightArm;

void setup() {
  // Set up motor controll pins as output
  pinMode(STBY, OUTPUT);
  pinMode(AIN1,OUTPUT);        
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT);        
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  leftArm.attach(LEFT_SERVO_PIN);  // Set left servo to digital pin 10
  rightArm.attach(RIGHT_SERVO_PIN);  // Set right servo to digital pin 9
  
  Serial.begin(9600);

  // Set up front arms to release position
  leftArm.write(180);
  rightArm.write(90);
  
  if (!DEBUG) {
    pingTimer = millis(); // Start now.
    for (int i = 0; i < 100; i++) {
      if (INIT_NAV_MODE) {
        if (millis() >= pingTimer) {
          pingTimer += pingSpeed;
          uS = sonar.ping_median(10);
          int cm = (int)uS / US_ROUNDTRIP_CM;

          if (INFO == 1) {
            Serial.print("Ping: ");
            Serial.print(cm);
            Serial.println();
          }
          
          if (cm > 0 && cm <= MAX_STOP_DISTANCE) {
            Serial.println("Rotating...");
            set_motors(60, -60);
          } else {
            INIT_NAV_MODE = 0;
            Serial.println("Route detected, moving out...");
            set_motors(0,0);
          }
        }
      }
    }

    Serial.println("Starting to calibrate in 3 seconds...");
    delay(3000);

    // Auto-calibration: turn right and left while calibrating the
    // sensors.
    init_calibrate(20, 100);
    
    set_motors(0,0); // stop the motor for a second
  }
}

void loop() {
  if (DEBUG) {
    // enable the following functions as needed
//    test_sensor();
//    test_motor();
//    test_arms();
  } else {
    if (!SHOOT_MODE) {
      // execute main function
      line_follow_drive();
      
      // get a good sample, the larger the sample the longer it takes to read
      uS = sonar.ping_median(5);
    
      // convert to cm
      int cm = (int)uS / US_ROUNDTRIP_CM;
    
      if (cm > 0 && cm <= MAX_STOP_DISTANCE) {
        if (INFO == 1) {
          Serial.print("Ping: ");
          Serial.print(cm);
          Serial.println();
        }
        SHOOT_MODE = 1; // enter shoot mode and stop the robot movement
      }
    } else {
      set_motors(0, 0); // stop the robot
      Serial.println("Entered SHOOTING MODE...");
    }
  }
}

void line_follow_drive() {
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
  int error = (int) position - 3000;

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
  
//  if (INFO == 1) {
//    Serial.println(power_difference);
//  }
  
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

/* Auto-calibration: turn right and left while calibrating the sensors.
 * baseDelayTime: base delay time, delayCounter: the number of loops to delay
 */
void init_calibrate(int baseDelayTime, int delayCounter) {
  for (int counter=0; counter<delayCounter; counter++) {
    if (counter < 20 || counter >= 60)
      set_motors(CALI_SPEED,-CALI_SPEED);
    else
      set_motors(-CALI_SPEED,CALI_SPEED);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    qtra.calibrate();

    // the total delay is (# of loops)*(delay time/loop),
    // i.e., 100*20 = 2000 ms.
    delay(baseDelayTime);
  }
}

/* Drive the robot by enabling power in left and right motors */
void set_motors(int leftMotorSpeed, int rightMotorSpeed) {
//  digitalWrite(STBY, HIGH);

  if (rightMotorSpeed < 0) { // move backwards
    digitalWrite(STBY, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(rightMotorSpeed));
  } else { // move forward
    digitalWrite(STBY, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightMotorSpeed);
  }
  
  if (leftMotorSpeed < 0) { // move backwards
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(leftMotorSpeed));
  } else { // move forward
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftMotorSpeed);
  }
}

/* lower the arms to capture the ball, degree values vary based on the inital servo position */
void grab_ball() {
  leftArm.write(90);
  rightArm.write(180);
}

/* raise the arms to release the ball, degree values vary based on the inital servo position */
void release_ball() {
  leftArm.write(180);
  rightArm.write(90);
}

/* Read raw sensor values from the QTR Reflectance Sensors */
void test_sensor() {
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
void test_motor() {
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

void test_arms() {
  grab_ball();
  delay(2000);
//  release_ball();
}

