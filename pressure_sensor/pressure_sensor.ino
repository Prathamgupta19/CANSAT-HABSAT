const int FSRPin1 = A0;
const int FSRPin2 = A1;
int q;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int pressure_L = analogRead(FSRPin1);
  int pressure_R = analogRead(FSRPin2);

  if (pressure_L > 150) {
    Serial.print("Left Pressure (1): ");
    Serial.println(pressure_L);
  }

  if (pressure_R > 150) {
    Serial.print("Right Pressure (2): ");
    Serial.println(pressure_R);
  }

  if (pressure_L - q > pressure_R + q) {
    Serial.println("Left belt tilt detected");
  }

  if (pressure_L + q < pressure_R - q) {
    Serial.println("Right belt tilt detected");
  }

  delay(1000);
}