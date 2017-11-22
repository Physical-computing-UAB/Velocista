#include <QTRSensors.h>

#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 4// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 255// max speed of the robot
#define BaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used

#define speedturn 180

#define rightMotor1 A2
#define rightMotor2 A1
#define rightMotorPWM 11
#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 10
#define motorPower A3

QTRSensorsRC qtrrc((unsigned char[]) {2,3,4,5,6,7,8,9} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  
  delay(3000);
  
  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
   //comment this part out for automatic calibration 
    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
    {
      move(1, 70, 1);//motor derecho hacia adelante
      move(0, 70, 0);//motor izquierdo hacia atras 
    }
    else
    {
      move(1, 70, 0);//motor derecho hacia atras
      move(0, 70, 1);//motor izquierdo hacia adelante  
    }
    qtrrc.calibrate();   
    delay(20);
  }
  wait();
  delay(3000); // wait for 2s to position the bot before entering the main loop 
}  

int lastError = 0;
unsigned int sensors[8];
int position = qtrrc.readLine(sensors);

void loop()
{  
  position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  
  if(position>6700){
    move(1, speedturn, 1);//motor derecho hacia adelante
    move(0, speedturn, 0);//motor izquierdo hacia adelante
    return;    
  }
  if(position<300){ 
    move(1, speedturn, 0);//motor derecho hacia adelante
    move(0, speedturn, 1);//motor izquierdo hacia adelante
    return;
  }
  
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
  move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante
}
  
void wait(){
  digitalWrite(motorPower, LOW);
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
