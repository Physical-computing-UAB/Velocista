#include <QTRSensors.h>
/*---------------------- Sensors ----------------------*/
#define NUM_SENSORS   8     // Number of sensors used
#define TIMEOUT       2500  // Waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // Emitter is controlled by digital pin 2
/*------------------ Speed & Rotation -----------------*/
#define KP 0.1              // Experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 1                // Experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MAXSPEED 255        // Max speed of the robot
#define BASESPEED 255       // This is the speed at which the motors should spin when the robot is perfectly on the line
#define SPEEDTURN 180
/*----------------------- Motors ----------------------*/
#define rightMotor1   A2
#define rightMotor2   A1
#define rightMotorPWM 11
#define leftMotor1    A4
#define leftMotor2    A5
#define leftMotorPWM  10
#define motorPower    A3

// Sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},
                                      NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
uint8_t i;
uint16_t error = 0;
uint16_t lastError = 0;
uint16_t turnLeft  = 0;
uint16_t turnRight = 0;
uint16_t motorSpeed = 0;
uint16_t leftMotorSpeed = 0;
uint16_t rightMotorSpeed = 0;

unsigned int sensorValues[NUM_SENSORS];
uint32_t position  = qtrrc.readLine(sensorValues);

void setup() {
  Serial.begin(9600);

  /*----------------------- Motors ----------------------*/
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  
  delay(500);
  /*---------------------- Sensors ----------------------*/
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // Turn on Arduino's LED to indicate we are in calibration mode
  for (i = 0; i < 400; i++)  // Make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // Reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // Turn off Arduino's LED to indicate we are through with calibration

  // Print the calibration minimum values measured when emitters were on
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Print the calibration maximum values measured when emitters were on
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();  
  delay(1000);
}

void loop() {
  position = qtrrc.readLine(sensorValues);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print("position is: ");
  Serial.print(position);
  Serial.println();
  /*---------------------- Sensors ----------------------*/
  turnLeft = 0;
  turnRight = 0;
  // Print the sensor values as integers from 0 to 1000, the lower the darker
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(position);
  
  for (i = 0; i < NUM_SENSORS/2; i++){
    turnRight += sensorValues[i];
  }
  for (i = 4; i < NUM_SENSORS; i++){
    turnLeft += sensorValues[i];
  }
  if (turnRight < turnLeft){
    Serial.println("Gotta turn Right");
  }
  else if (turnRight > turnLeft){
    Serial.println("Gotta turn Left");
  }
  else{
    Serial.println("Gotta keep STRAIGHT");
  }



  /*----------------------- Motors ----------------------*/
  if(position>6700){
    move(1, SPEEDTURN, 1);    // Motor derecho hacia adelante
    move(0, SPEEDTURN, 0);    // Motor izquierdo hacia adelante
    return;    
  }
  if(position<300){ 
    move(1, SPEEDTURN, 0);    // Motor derecho hacia adelante
    move(0, SPEEDTURN, 1);    // Motor izquierdo hacia adelante
    return;
  }
  error = position - 3500;
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  rightMotorSpeed = min(max((BASESPEED + motorSpeed), 255), 0);
  leftMotorSpeed = min(max((BASESPEED - motorSpeed), 255), 0);

  move(1, rightMotorSpeed, 1);  // Motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);   // Motor izquierdo hacia adelante
  
  delay(500);
}



void move(int motor, int speed, int direction){
  digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}
