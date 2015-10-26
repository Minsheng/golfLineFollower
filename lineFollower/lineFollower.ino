/*
** Line Follower Basic v. 0.5
** Last Update: 2013-05-21
*/

#include <Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define DEBUG 1
#define INFO 1

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

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

/* Set up maximum speed and speed for turning (to be used with PWM) */
// PWM to control motor speed [0 - 255]
int maxSpeed = 120;
int fullSpeed = 255; // for very sharp turn

/* variables to keep track of current speed of motors */
int motorLSpeed = 120;
int motorRSpeed = 120;

/* For PID algorithm */
byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

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

  // auto-calibrate
  for (int i = 0; i < 100; i++) {
    qtra.calibrate();
    delay(20);
  }
  
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if (tuning) {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  
  Serial.begin(115200);
}

void loop() {
  if (DEBUG == 1) {
    testMotor();
  } else {
    Scan();
  }
}

void testMotor() {
  int testSpeed = 255;
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

void Scan() {
  unsigned long now = millis();

  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  unsigned int position = qtra.readLine(sensorValues);
  
  if (tuning) {
    byte val = (aTune.Runtime());
    if (val!=0) {
      tuning = false;
    }
    
    if (!tuning) { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  } else {
    myPID.Compute();
  }

  //send-receive with processing if it's time
  if (millis()>serialTime) {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void Drive() {

  // recalculate speed for sharp turns
  if (turnMode == 1) {
    if (motorLSpeed <= 0 && motorRSpeed >= 0) {
      motorLSpeed = -fullSpeed*0.3;
      motorRSpeed = fullSpeed;
    } else if (motorLSpeed >= 0 && motorRSpeed <= 0){
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

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}
