

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;
int trvaelt;
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  int initial_thickness = 1;
  int innitial_distace = 19;
  int total_distace = 20;
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
    int total_distace = 20;
  duration = pulseIn(echoPin, HIGH);
  Serial.print("Sound travel time:");
  Serial.println(duration);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  int thickness = total_distace - distance;
  // Prints the distance on the Serial Monitor
  Serial.print("distance: ");
  Serial.println(distance);
   Serial.print("thickness: ");
  Serial.println(thickness);
  delay(1000);
}