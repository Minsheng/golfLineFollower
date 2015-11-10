/*****************************************************************************
 * OCAD University DF Creation and Computation GOLF project 2015
 * 
 * This code is made for Line Following Robot. It visualize the trail of the robot.
 * Connect to bluetooth before starting.
 *
 * Last edited by Michael Carnevale on November 10th, 2015
 *********************************************************************/ 
 
import processing.serial.*;

// Values for Visualization
int lineLength = 1;
int turnAngle = 5;
int[] mapArray = new int[1];
int Value;
PImage img;
int gridRes = 100; 

// Values for Serial/Bluetooth from Arduino
int bgcolor;           // Background color
Serial myPort;                       // The serial port
int[] serialInArray = new int[3];    // Where we'll put what we receive
int serialCount = 0;                 // A count of how many bytes we receive
int leftMotor = 0;
int rightMotor = 0;                 // Starting position of the ball
boolean firstContact = false;        // Whether we've heard from the microcontroller
long lastUpdate;
int mode ;     //   
int timer = 200;

void setup() {
  size(1200, 800);  // Stage size
  noStroke();      // No border on the next thing drawn
  // Open whatever port is the one you're using.
  String portName = Serial.list()[4];     // 4 is bluetooth 5 is usb
  myPort = new Serial(this, portName, 9600);
  lastUpdate = millis();
  img = loadImage("worldmap.jpg");
}

void draw() {
  
  if ( myPort.available() > 0) {
    mode = myPort.read();  }    
    println(mode);

  
  
  imageMode(CENTER);
  image(img, 0,0, 0, 0);

 //randVal = ((int) random(3)-1)*turnAngle;
if (millis()- lastUpdate > timer) {
  lastUpdate = millis();  
 
// CONVERT MOTOR VALUES FROM SERIAL PORT INTO VALUES FOR THIS SCRIPT (0, -1, OR 1) 
 
if(mode ==11) {
  Value = 0;    mapArray = append(mapArray, Value); updateMap();
} else if(mode ==12) {
  Value = 1*turnAngle;    mapArray = append(mapArray, Value); updateMap();
} else if(mode == 13) {
  Value = -1*turnAngle;    mapArray = append(mapArray, Value); updateMap();
} else {updateMap();}
 
 //if (abs(leftMotor - rightMotor) < 15 && mode == 1){ 
 // Value = 0;   
 //  mapArray = append(mapArray, Value);
 //  updateMap();
 //} else if (leftMotor > rightMotor + 20 && mode == 1) {
 // Value = 1*turnAngle;  
 //  mapArray = append(mapArray, Value);
 //  updateMap();
 //} else if (rightMotor > leftMotor + 20 && mode == 1){
 // Value = -1*turnAngle;  
 //  mapArray = append(mapArray, Value);
 //  updateMap();
 //} else if(mode == 2) { // Calibration Period
 //  updateMap();
 //} else if(mode == 0) { // Ending
 //  updateMap();
 //} 
}// 
}
 
void updateMap() {
 
 background(255); 
 translate(width/2, height/2-lineLength);
 image(img, 0,0);

 
 // DRAW THE UPDATING LINE MAP!!!
 pushMatrix(); 
   ellipseMode(CENTER);
  //stroke(0);
  strokeWeight(2);
 for(int i=0; i<mapArray.length; i++) {  // Redraw line and map representation from mapArray
  translate(0, lineLength);
  rotate(radians(mapArray[i]));
  if(radians(mapArray[i]) > 0) {
    stroke(0,255,0);
  } else if(radians(mapArray[i]) < 0) {
    stroke(255,0,0); 
  } else { stroke(0);}
  line(0, 0, 0, lineLength); 
 }
 popMatrix();

// DRAW AND UPDATE POSITION OF THE CAR
pushMatrix();  
for(int i=0; i<mapArray.length; i++) {
 translate(0, lineLength);
 rotate(radians(mapArray[i]));
 }
// DRAw THE CAR!!!
rectMode(CENTER);
strokeWeight(1);
stroke(0);
fill(0);
rect(0, 5, 16, 35);
arc(0, -8, 28, 13, radians(180), radians(360)); 
if(mapArray[mapArray.length-1]<0){fill(255,0,0);} else{fill(255);};
ellipse(-12, 0, 6, 10);
if(mapArray[mapArray.length-1]>0){fill(0,255,0);} else{fill(255);};
ellipse(12, 0, 6, 10);
fill(0);
rect(0, 25, 4, 8);
rect(0, 27, 12, 5);
popMatrix();
}








/*
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
  }
  else {
    // Add the latest byte from the serial port to array:
    serialInArray[serialCount] = inByte;
    serialCount++;

    // If we have 2 bytes:
    if (serialCount > 0 ) {
      mode = serialInArray[0];
      //rightMotor = serialInArray[1];
      //mode = serialInArray[2];

      // print the values (for debugging purposes only):
      

      // Send a capital A to request new sensor readings:
      myPort.write('A');
      // Reset serialCount:
      serialCount = 0;
    }
  }
}
*/