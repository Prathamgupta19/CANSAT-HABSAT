#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mlx.begin();
}

void loop() {
  float ambientTempC = mlx.readAmbientTempC();
  Serial.print("Ambient Temperature: ");
  Serial.print(ambientTempC);
  Serial.println(" °C");
  delay(1000);
}
