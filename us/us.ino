const int trigPin = 9;
const int echoPin = 10;
long duration;
float distance;
float og_thick;
float og_dist;
float dist_tot;
float curr_thick;
int tension;
float duration_ideal;
int duration_L;
int duration_U;
int etime;

boolean measure = false;
boolean erodeMeasureStarted = false;
unsigned long startTime = 0; // Use unsigned long for time-related variables

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  duration_U = duration_ideal + 500;
  duration_L = duration_ideal - 500;

  if (!measure) {
    if (Serial.available() > 1) {
      og_thick = Serial.parseFloat(); // Use parseFloat instead of parseInt for floating-point values
      Serial.println(og_thick);
      og_dist = Serial.parseFloat(); // Use parseFloat for floating-point values
      Serial.println(og_dist);
      duration_ideal = og_dist * 2 / 0.034;  // Calculate the expected duration based on distance
      measure = true;
    }
  } else {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);

    distance = duration * 0.034 / 2;
    dist_tot = og_dist + og_thick;

    if (abs(distance - dist_tot) > og_thick) {
      if (duration < duration_L) {
        Serial.println("Loose belt detected");
      } else if (duration > duration_U) {
        Serial.println("Hole detected");
      }
    } else {
      curr_thick = distance - dist_tot;
      // Serial.print("Current thickness is: ");
      // Serial.print(curr_thick, 4); // Print the thickness with 4 decimal places
      // Serial.println(" mm");
      tension = map(curr_thick, 0, og_thick, 0, 7); // Adjusted the mapping, assuming 0 to 7 MPa
      // Serial.print("Tension (in MPa): ");
      // Serial.println(tension, 4); // Print tension with 4 decimal places

      Serial.print(curr_thick, 4);
      Serial.print(",");
      Serial.println(tension, 4);

    }
  }
  delay(500);

  