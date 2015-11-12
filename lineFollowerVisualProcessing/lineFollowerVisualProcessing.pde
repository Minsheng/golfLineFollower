/*****************************************************************************
 * OCAD University DF Creation and Computation GOLF project 2015
 * 
 * This code is made for Line Following Robot. It visualize the trail of the robot.
 * Connect to bluetooth before starting.
 *
 * Last edited by Michael Carnevale on November 10th, 2015
 *********************************************************************/ 
 
import processing.serial.*;

// Variable and array value for Visualization
int lineLength = 2;        // The length that the car moves every iteration. Manipulate this variable to change the speed
int turnAngle = 5;         // The turn angle of the car... manipulate this value to change the turning angle of the car in visualization
int[] mapArray = new int[1];   // variable used to record all the translations and rotations of the car over time
int Value;                    // variable used below to represent turn angle being inputted during iterations where the car is moving
PImage img;                   // set up to upload a downloaded image of the world

// Values for Serial/Bluetooth communication between Processing and Arduino robot wheels to 
int bgcolor;                         // Background color
Serial myPort;                       // Serial port name
int[] serialInArray = new int[3];    // Where we'll put what we receive from serial
int serialCount = 0;                 // A count of how many bytes we receive. Used to control a for loop
int leftMotor = 0;                  // Value 
int rightMotor = 0;                 // Starting position of the ball
boolean firstContact = false;        // Whether we've heard from the microcontroller
long lastUpdate;                     // Declare the last update as variable type "long". Such a large digit cannot exist as a normal integer
int mode ;     //                   // Important variable... Represents the number given from ARduino to Processing... This number tells Processing what to execute out of a series of options
int timer = 100;                    // Essentially sets framerate using the millis() function below  

void setup() {
  size(1200, 800);                                        // Size of draw screen (in pixels)
  noStroke();                                             // No border on the next thing drawn (can still fill with colour... just no border) 
  // Set up Serial with an available port of your choosing (e.g., USB, bluetooth).

  // This is commented here only so the simulation can work
  //  String portName = Serial.list()[4];                     // Serial.list()[4] selects the 4th available port in the list and saves it to String variable portName
  //  myPort = new Serial(this, portName, 9600);              // Command to initialize Serial at baud rate (baud rate must be same between Processing and Arduino

  lastUpdate = millis();                                  // lastUpdate is last saved millis() timing. Used in a timer loop below.
  img = loadImage("worldmap.jpg");                        // Load up image file (e.g., jpg, bitmap, etc.) from "data" folder to draw the image below
  textSize(40); //Set the size of the text foR upcoming text(); function call;
}

void draw() {

  mode = round(random(11, 15));

  imageMode(CENTER);           // In the next line draw the image from the center outwards 
  image(img, 0, 0, 0, 0);       // paste the image at a set location on the screen.   image(image file, x position, y position

  //randVal = ((int) random(3)-1)*turnAngle;
  if (millis()- lastUpdate > timer) {             // Timer. Proceed if the difference between current time and previous saved time is greater than the set timer value... 
    lastUpdate = millis();                        // recently updated time value.

    //// IMPORTANT: mode variable below represents instructions sent from Arduino.
    //// This variable controls the overall behaviour of the visualization
    //// This value is obtained using the SerialEvent() function below that runs alongs with the draw() 
    println(mode);   

    // USE THE ARDUINO VALUE GIVEN ("Value") TO CONTROL THE NEXT FRAME OF THE VISUALIZATION... 

    // .MODE VALUE LEGEND..
    // mode == 11 - Drive straight 
    // mode == 12 - Drive left
    // mode == 13 - Drive right
    // mode == 14 - Write "Calibrate!" on screen
    // mode == 15 - Write "Release the ball!" on screen

    if (mode == 11) {                                                             
      Value = 0;    
      mapArray = append(mapArray, Value); 
      updateMap();   
      // Value represents the direction the car is going. When zero, car goes straight
      // mapArray represents the record of places the car has gone. This array is updated every iteration to redraw the line map showing where the car has travelled.
      // updateMap() is the major function for drawing/redrawing the car and line map using the mapArray[]
    } else if (mode ==12) {
      Value = 1*turnAngle;    
      mapArray = append(mapArray, Value); 
      updateMap();      
      // Value can be -1 (go left), 0 (go straight), or 1 (go right)
      // Value is scaled by turnAngle (can be modified at beginning of this script) to make the car turn at different rates of rotation
    } else if (mode == 13) {
      Value = -1*turnAngle;    
      mapArray = append(mapArray, Value); 
      updateMap();
    } else if (mode == 14) {   // Write some text on the screen. The map array is not appended with new values so the car stays idle.
      updateMap(); 
      fill(0, 0, 255); 
      text("Calibrate!", -20, -200);// Calibration and avoid walls
    } else if (mode == 15 || mode == 10) {
      updateMap(); 
      fill(0, 0, 255); 
      text("Release the Ball!", -20, -200);// End Mode
    } else {
      updateMap();
    }     // If the serial behaves oddly and you get some random values just redraw the map and let the car idle.
  }
}

void updateMap() {
  background(255);                             // refresh background (0 = black, 255 = white, mid-range is grey)
  translate(width/2, height/2-lineLength);     // translate function is used to position things around the screen. Effects everything on screen that is not put in a Push/PopMatrix (explained below)
  image(img, 0, 0);                             // Paste the loaded image ("worldmap.jpg") onto the screen at given [x, y] coordinates

  /// DRAW THE UPDATING LINE MAP!!! ///
  pushMatrix();                // Any objects inside the pushMatrix() and popMatrix() functions will not be affected by what is going on outside the Matrix. Very useful for having multiple objects with different behaviour/characteristics 
  ellipseMode(CENTER);       // Draw upcoming ellipse from the center outwards
  //stroke(0);
  strokeWeight(2);            // Thickness of lines to be drawn
  for (int i=0; i<mapArray.length; i++) {  // This for loop redraws line and map representation using the recorded values in mapArray
    translate(0, lineLength);              // move the car one movement unit with distance of lineLength
    rotate(radians(mapArray[i]));          // Make the car turn by the amount found in mapArray[i]. Radians used for better angle units. 
    if (radians(mapArray[i]) > 0) {         // If the car goes straight, draw the line in green
      stroke(0, 255, 0);
    } else if (radians(mapArray[i]) < 0) {  // if left, draw line in red
      stroke(255, 0, 0);
    } else { 
      stroke(0);
    }                   // if straight, draw the line in black 
    line(0, 0, 0, lineLength);             // Draw the line!
  }
  popMatrix();                            // close the push/pop matrix


    // DRAW AND UPDATE POSITION OF THE CAR!!! //
  pushMatrix();  
  for (int i=0; i<mapArray.length; i++) {   // use the mapArray values to draw the car at the most updated position relative to first iteration
    translate(0, lineLength);
    rotate(radians(mapArray[i]));
  }
  // DRAw THE CAR!!!
  rectMode(CENTER);                        // draw rectangle from its center (rather than top/left corner of rectangle)
  strokeWeight(1);   // thickness of borderline                      
  stroke(0);         // colour of borderline
  fill(0);           // colour to fill the shape
  rect(0, 5, 16, 35);   // rect(x position, y position, thickness width, thickness height)
  arc(0, -8, 28, 13, radians(180), radians(360));         // Draw portion of an ellipse..    arc(x position, y position, thickness width, thickness height, angle to start, angle to end)
  if (mapArray[mapArray.length-1]<0) {
    fill(255, 0, 0);
  } else {
    fill(255);
  };
  ellipse(-12, 0, 6, 10);
  if (mapArray[mapArray.length-1]>0) {
    fill(0, 255, 0);
  } else {
    fill(255);
  };
  ellipse(12, 0, 6, 10);
  fill(0);
  rect(0, 25, 4, 8);
  rect(0, 27, 12, 5);
  popMatrix();          // All of these shapes combined make up the body and wheels of the car.
}                     // the push/popmatrix ensures that every translation/rotation here only affects the car

/*
// Arduino to Processing SErial Communication //
// Arduino sends one byte through serial at a time to Processing.         This code is commented out so the simulation will work
void serialEvent(Serial myPort) {       
  // read a byte from the serial port
  int inByte = myPort.read();
  // if this is the first byte received, and it's an A,
  // clear the serial buffer and note that you've
  // had first contact from the microcontroller.
  // Otherwise, add the incoming byte to the array:
  if (firstContact == false) {    
    if (inByte == 'A') {
      myPort.clear();          // clear the serial port buffer
      firstContact = true;     // you've had first contact from the microcontroller
      myPort.write('A');       // ask for more serial communication. When Arduino reads "A" it begins sending a new set of data
    }
  } else {      // if this is not the first byte of new values to the array (which can hold multiple values) then read the next byte into a new array position
    // Add the latest byte from the serial port to array: This is particularly useful when sending multiple values into Processing at once
    serialInArray[serialCount] = inByte;      // put the new value into a specific spot in the array
    serialCount++;                            // counts the number of bytes currently in the variable array communicated from Arduino

    // Controls the amount of bytes to wait for before resetting the array
    if (serialCount > 0 ) {      // If there is more then zero bytes, restart the array. This is useful when you want an array to have multiple values
      mode = serialInArray[0];   // mode is the master control variable for controlling the visualization

      // Send a capital A to request new sensor readings: 
      myPort.write('A');
      // Reset serialCount:
      serialCount = 0;
    }
  }
}
*/