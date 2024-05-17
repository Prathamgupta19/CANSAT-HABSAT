#define IR_SENSOR_LEFT 2 // left sensor
#define IR_SENSOR_RIGHT 3 // right sensor
#define MOTOR_SPEED 180

/*-------definning Outputs------*/
#define leftMotorPin1 4  // left motor
#define leftMotorPin2 5   // left motor
#define rightMotorPin1 6 // right motor
#define rightMotorPin2 7 // right motor

void setup()
{
pinMode(IR_SENSOR_LEFT, INPUT);
pinMode(IR_SENSOR_RIGHT, INPUT);
pinMode(leftMotorPin1, OUTPUT);
pinMode(leftMotorPin2, OUTPUT);
pinMode(rightMotorPin1, OUTPUT);
pinMode(rightMotorPin2, OUTPUT);
rotateMotor(0,0);
}
void loop()
{
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
 //If none of the sensors detects black line, then go straight
   if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
 //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  }
 //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    rotateMotor(0, 0);
  }
}
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
   else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
    if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
 else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  
  
  
  
  
  
  
  
  
  
  







