/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup()
{
  Serial.begin(57600);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

Vector filtered;

void loop()
{
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();

  filtered = lowPassFilter(filtered, normAccel, 0.15);

  // Calculate Pitch & Roll
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

  int fpitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  int froll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;

  // Output
  Serial.print(pitch);
  Serial.print(":");
  Serial.print(roll);
  Serial.print(":");

  Serial.print(fpitch);
  Serial.print(":");
  Serial.print(froll);
  Serial.println();
}

Vector lowPassFilter(Vector f, Vector vector, float alpha)
{
  f.XAxis = vector.XAxis * alpha + (f.XAxis * (1.0 - alpha));
  f.YAxis = vector.YAxis * alpha + (f.YAxis * (1.0 - alpha));
  f.ZAxis = vector.ZAxis * alpha + (f.ZAxis * (1.0 - alpha));
  return f;
}
