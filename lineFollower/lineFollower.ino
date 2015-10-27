#include <QTRSensors.h>

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

#define DEBUG 0
#define INFO 1

#define Kp 1.6 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1.8 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 90 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 90  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4

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

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
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
    for(int counter=0;counter<100;counter++)
    {
      if(counter < 20 || counter >= 60)
        set_motors(40,-40);
      else
        set_motors(-40,40);
  
      // This function records a set of sensor readings and keeps
      // track of the minimum and maximum values encountered.  The
      // IR_EMITTERS_ON argument means that the IR LEDs will be
      // turned on during the reading, which is usually what you
      // want.
      qtra.calibrate();
  
      // Since our counter runs to 80, the total delay will be
      // 80*20 = 1600 ms.
      delay(20);
    }
    set_motors(0,0);
  }
  
  Serial.begin(9600);
}

void loop() {
  if (DEBUG == 1) {
//    testSensor();
    testMotor();
  } else {
    scan();
  }
}

void scan() {
  unsigned int position = 0;
  long integral=0;
  
  // get calibrated readings along with the line position, 
  // refer to the QTR Sensors Arduino Library for more details on line position.
  position = qtra.readLine(sensorValues);
  
  // The "error" term should be 0 when we are on the line.
  int error = (int) position - 2500;

  int derivative = error - lastError;

  integral += error;
  
  lastError = error;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  int power_difference = error/20 + integral/10000 + derivative*3/2;

  if (INFO == 1) {
    Serial.println(power_difference);
  }
  
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int max = 70;
  if(power_difference > max)
    power_difference = max;
  
  if(power_difference < -max)
    power_difference = -max;

  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  if (power_difference < 0) {
    leftMotorSpeed = max+power_difference;
    rightMotorSpeed = max;
  } else {
    leftMotorSpeed = max;
    rightMotorSpeed = max-power_difference;
  }
  
  set_motors(leftMotorSpeed, rightMotorSpeed);
      
  if (INFO == 1) {
    Serial.print(leftMotorSpeed);
    Serial.print('\t');
    Serial.print(rightMotorSpeed);
    Serial.println();
  }
}

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
