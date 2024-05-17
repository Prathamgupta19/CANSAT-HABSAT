#include <Servo.h>
Servo servoMotor;

// Laser
int laser = 13;

// Sensor Ultrasonido
int trig = 10;
int echo = 9;
float distancia;
float duracion;

// Potentiometer
int potentiometerPin = A0;

// Vibration Sensor
int VIBRATION_DIGITAL_IN_PIN = 8;
int BUZZER_DIGITAL_OUT_PIN = 12; 
int buzzerDurationMillis = 1000;

void setup() {
  // Servo Motor
  servoMotor.attach(11);
  
  // Laser
  pinMode(laser, OUTPUT);
  
  // Sensor Ultrasonido
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  // Potentiometer
  pinMode(potentiometerPin, INPUT);
  
  // Monitor serial
  Serial.begin(9600);
  
  // Vibration Sensor
  pinMode(VIBRATION_DIGITAL_IN_PIN, INPUT);
  pinMode(BUZZER_DIGITAL_OUT_PIN, OUTPUT);
}

void loop() {
  // Servo Motor
  float pot = analogRead(potentiometerPin);
  float angulo = pot * 180.0 / 1023.0;
  servoMotor.write(angulo);
  delay(1000);
  
  // Laser
  digitalWrite(laser, HIGH);
  
  // Sensor Ultrasonido
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);
  
  duracion = pulseIn(echo, HIGH);
  distancia = duracion / 58.2;
  
  // Calculo
  float x = distancia * tan((90.0 - angulo) * 3.141592 / 180.0);
  
  // Monitor serial
  Serial.print("Angulo= ");
  Serial.print(90 - angulo);
  Serial.print("| x= ");
  Serial.print(x);
  Serial.print("[cm]");
  Serial.print(" | y= ");
  Serial.print(distancia);
  Serial.println("[cm]");
  
  // Vibration Sensor
  if (digitalRead(VIBRATION_DIGITAL_IN_PIN) == HIGH) {
    digitalWrite(BUZZER_DIGITAL_OUT_PIN, HIGH);
    delay(buzzerDurationMillis);
    digitalWrite(BUZZER_DIGITAL_OUT_PIN, LOW);
  }
}
