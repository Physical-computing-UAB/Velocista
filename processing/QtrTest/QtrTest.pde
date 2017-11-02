  
/*
   Processing Script to test Pololu QTR-8RC  
   
   It expects strings on serial like this: 
   "1000,847,332,20,20,1000,1000,1000\n"
   
   Use QTRRCExample from QTRSensor library from Pololu
   to send serial data from arduino to processing.
   
   Take care about the serial port. 
   
   @autor Steven Macías 
   @autor Antonio Hurtado
*/ 

import processing.serial.*;

//define
Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port
int NUM_SENSORS = 8; 

void setup() {
  //Size of the window.
  size(500, 200);
  // COMX in Windows or /dev/ttyACMX in Linux (X is a number)
  String portName = "/dev/ttyACM2"; 
  myPort = new Serial(this, portName, 9600);
}

void draw() {
  if ( myPort.available() > 0) 
  {  // If data is available,
    val = myPort.readStringUntil('\n');
    if ((val != null)) {
      //Erase '\n' (end of line)  and '\0' (end of string)
      val = val.substring (  0, val.length()-2  ); 
      println(val);
      // Break each value into and array.
      String[] list = split(val, "\t");
      // avoid index out of bound if some data is lost. 
      if (list.length >= NUM_SENSORS) {
        //needed to erase everithing in each iteration
        background(125);
        //Draw all squares
        for (int i = 0; i<NUM_SENSORS; i++) {
          fill(valToRGB(list[i]));
          rect(50+(i*50), 60, 40, 40);
          textSize(10);
          text(list[i], 55+(i*50), 120); 
        }
        println();
      }
    }
  }
}

/*
*Transforms a string into a grey scale value; 
*/
int valToRGB(String val) {
  int ival=int(val);
  int rgb = 0; 
  rgb = (ival*255)/1000; 
  return rgb;
}