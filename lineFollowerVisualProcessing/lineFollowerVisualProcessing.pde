/*********************************************************************
 * Visualization of robot movement, data is read from Arduino 
 * via bluetooth.
 * Last edited by Michael Carnevale on November 3rd, 2015
 *********************************************************************/

import processing.serial.*;

// Values for Visualization
int lineLength = 2;
int turnAngle = 2;
int[] mapArray = new int[1];
int Value;
PImage img;
int gridRes = 100; 

// Values for Serial/Bluetooth from Arduino
int bgcolor;                          // Background color
Serial myPort;                        // The serial port
int[] serialInArray = new int[2];     // Where we'll put what we receive
int serialCount = 0;                  // A count of how many bytes we receive
int leftMotor = 0;
int rightMotor = 0;                   // Starting position of the ball
boolean firstContact = false;         // Whether we've heard from the microcontroller


void setup() {
  size(1200, 800);  // Stage size
  noStroke();  // No border on the next thing drawn
  // Open whatever port is the one you're using.
  String portName = Serial.list()[4];  // 4 is bluetooth 5 is usb
  myPort = new Serial(this, portName, 9600);
}

void draw() {
  background(255); 
  //randVal = ((int) random(3)-1)*turnAngle;
  
  // CONVERT MOTOR VALUES FROM SERIAL PORT INTO VALUES FOR THIS SCRIPT (0, -1, OR 1) 
  if (leftMotor > 0 && rightMotor > 0){
    Value = 0;
  } else if (leftMotor > 0 && rightMotor == 0) {
    Value = 1*turnAngle; 
  } else if (leftMotor == 0 && rightMotor > 0){
    Value = -1*turnAngle; 
  } else {
    Value = 0;
  }
  
  mapArray = append(mapArray, Value);
  translate(width/2, height/2-lineLength);
  
  // DRAW THE UPDATING LINE MAP!!!
  pushMatrix(); 
  //stroke(0);
  strokeWeight(2);
  
  for(int i=0; i<mapArray.length; i++) {  // Redraw line and map representation from mapArray
    translate(0, lineLength);
    rotate(radians(mapArray[i]));
    
    if(radians(mapArray[i]) > 0) {
      stroke(0,255,0);
    } else if(radians(mapArray[i]) < 0) {
      stroke(255,0,0); 
    } else {
      stroke(0);
    }
    line(0, 0, 0, lineLength); 
  }
  popMatrix();
  
  // DRAW THE CAR!!!
  pushMatrix(); 
  rectMode(CENTER);
  strokeWeight(1);
  stroke(0);
  triangle(-10, 0, 10, 0, 0, -20);

  if (mapArray[mapArray.length-1] < 0) {
    fill(255,0,0);
  } else {
    fill(255);
  }
  
  ellipse(-12, 0, 5, 10);

  if (mapArray[mapArray.length-1] > 0) {
    fill(0,255,0);
  } else {
    fill(255);
  }

  ellipse(12, 0, 5, 10);
  fill(255);
  rect(0, -20, 20, 5);
  popMatrix();

  // DRAW AND MOVE THE CICRULAR GRID
  pushMatrix();
  ellipseMode(CENTER);
  stroke(100);
  strokeWeight(0);
  noFill();
  
  for(int i=1; i<100; i++){
    ellipse(0,0,i*gridRes,i*gridRes);
  }
  
  popMatrix();
}

void serialEvent(Serial myPort) {
  // read a byte from the serial port:
  int inByte = myPort.read();
  // if this is the first byte received, and it's an A,
  // clear the serial buffer and note that you've
  // had first contact from the microcontroller.
  // Otherwise, add the incoming byte to the array:
  if (firstContact == false) {
    if (inByte == 'A') {
      myPort.clear();          // clear the serial port buffer
      firstContact = true;     // you've had first contact from the microcontroller
      myPort.write('A');       // ask for more
    }
  } else {
    // Add the latest byte from the serial port to array:
    serialInArray[serialCount] = inByte;
    serialCount++;

    // If we have 2 bytes:
    if (serialCount > 1 ) {
      leftMotor = serialInArray[0];
      rightMotor = serialInArray[1];

      // print the values (for debugging purposes only):
      println(leftMotor + "\t" + rightMotor + "\t" );

      // Send a capital A to request new sensor readings:
      myPort.write('A');
      // Reset serialCount:
      serialCount = 0;
    }
  }
}