/* 
 *  Motor Driver control function:
 *  Values:
 *    motor: 1 | 2
 *    identifies the motor
 *    
 *    per255: {-255,255}
 *    the bigger the value the higher the rotation speed
 *    sign rotates the motor one way or another
 *    
 *    motorPin# vars need to be declared
 *  
 */
void setMotor(int motor, int per255) {

  int pin1, pin2; // each "side" of the selected motor

  boolean outputs = true; // protection against inappropriate motor index

  switch (motor) {

    case 1:

      pin1 = motorPin1;

      pin2 = motorPin2;

      break;

    case 2:

      pin1 = motorPin3;

      pin2 = motorPin4;

      break;

    default:

      outputs = false; // if we're here, the motor index was bad

      break;

  } // switch motor

  if (outputs) {

    if (per255 < 0) { // negative = run "backwards"

      digitalWrite(pin2, LOW);

      analogWrite(pin1, abs(per255));

    } else { // positive = run "forwards"

      digitalWrite(pin1, LOW);

      analogWrite(pin2, per255);

    } // if per else

  } // if outputs

} // setMotor
