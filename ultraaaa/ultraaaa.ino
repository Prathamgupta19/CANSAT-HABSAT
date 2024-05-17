const int trigPin = 9;
const int echoPin = 10;
float duration;
float  distance;


void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
 
   
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float original_thickness = 0.2;
  float total_distance = 18.2;
  float duration_L = 600;

  duration = pulseIn(echoPin, HIGH);
  Serial.print("Sound travel time:");
  Serial.println(duration);

  distance = duration * 0.034/ 2;

  if (abs( total_distance - distance) > original_thickness) {
   Serial.println("Error detected!");
      if (duration < duration_L) {
        Serial.println("Loose belt detected");
      } else {
        Serial.println("Hole detected");
      }
  }
   else {
    float current_thickness = total_distance - distance;
    Serial.print("Current thickness is: ");
    Serial.println(current_thickness);
    float tension = map(current_thickness, 0, 0.2, 7, 0); // Example mapping, adjust as needed
    Serial.print("Tension: ");
    Serial.println(tension);
  }

  delay(1000);
}
