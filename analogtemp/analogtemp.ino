int ThermistorPin = A0;
int Vo;
float R1 = 10000; // value of R1 on board
float logR2, R2, T;
float c1 = 0.001125308852122, c2 = 0.000234711863267, c3 = 8.987465564E-08; // typical values for an NTC thermistor

void setup() {
  Serial.begin(9600);
}

void loop() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0); // calculate resistance on thermistor
  logR2 = log(R2);
  T = 1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2); // temperature in Kelvin
  T = T - 273.15; // convert Kelvin to Celsius
  Serial.print("Temperature: ");
  Serial.print(T);
  Serial.println(" C");
  delay(500);
}
