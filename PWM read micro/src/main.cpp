#include <Arduino.h>
#include <math.h>

const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int pin = 7;
int scale = 3;
unsigned long duration;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  duration = pulseIn(pin, HIGH);

  int rawx = analogRead(xpin);
  int rawy = analogRead(ypin);
  int rawz = analogRead(zpin);

  float scaledx, scaledy, scaledz;
  scaledx = mapf(rawx, 0, 1023, -scale, scale);
  scaledy = mapf(rawy, 0, 1023, -scale, scale);
  scaledz = mapf(rawz, 0, 1023, -scale, scale);

  int xtilt, ytilt, ztilt;
  xtilt = 180 * atan(rawx / sqrt(rawy*rawy + rawz*rawz));
  ytilt = 180 * atan(rawy / sqrt(rawx*rawx + rawz*rawz));
  ztilt = 180 * atan(sqrt(rawx*rawx + rawy*rawy)/rawz);
  
  Serial.print("PWM: "); Serial.print(duration); Serial.print("\t");
  Serial.print("Raw x: "); Serial.print(rawx); Serial.print("\t");
  Serial.print("Raw y: "); Serial.print(rawy); Serial.print("\t");
  Serial.print("Raw z: "); Serial.print(rawz); Serial.println();

  Serial.print("Scaled x: "); Serial.print(scaledx); Serial.print(" g"); Serial.print("\t");
  Serial.print("Scaled y: "); Serial.print(scaledy); Serial.print(" g"); Serial.print("\t");
  Serial.print("Scaled z: "); Serial.print(scaledz); Serial.print(" g"); Serial.println();

  Serial.print("X tilt: "); Serial.print(xtilt); Serial.print("\t");
  Serial.print("Y tilt: "); Serial.print(ytilt); Serial.print("\t");
  Serial.print("Z tilt: "); Serial.print(ztilt); Serial.println();
  Serial.println();

  delay(100);
}

int* filter(float x, float y, float z) {
  return 0;
}

void calibration() {

}

double tilt(double x, double y, double z) { // -arctan[(-sin(ax)*cos(az) + cos(ax)*sin(ay)*sin(az)) / (cos(ax)*cos(ay))]
  return atan((-sin(x)*cos(z) + cos(x)*sin(y)*sin(z) )/ (cos(x)*cos(y)) );  
}

double pitch(double x, double y, double z) {
  //pitch = arcsin[ cos(ax)*sin(ay)*cos(az) + sin(ax)*sin(az) ]
  return asin(cos(x)*sin(y)*cos(z) + sin(x)*sin(z));
} 

double roll(double x, double y, double z) {
  //roll = -arctan[(-cos(ax)*sin(az) + sin(ax)*sin(ay)*cos(az)) / (cos(ay)*cos(az))]
  return -atan(() / (cos(y)));
} 

