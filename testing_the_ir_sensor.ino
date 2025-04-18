#include <QTRSensors.h>

//motor 1
#define IN1 7
#define IN2 8
#define ENA 5

//motor 2
#define IN3 9
#define IN4 10
#define ENB 6


QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 3, 4}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(9600);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call 

  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  Serial.println("calibration started");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("calibration Ended");

  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = position - 2500;
  float Kp = 1; // tuning needed
  int baseSpeed = 100;

  float turn = Kp * error;

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  float leftSpeed = baseSpeed + turn;
  float rightSpeed = baseSpeed - turn;

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);


  Serial.print(" | Position: ");
  Serial.print(position);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);

  delay(250);
}

void setMotorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed;
  }
  analogWrite(ENA, constrain(speed, 0, 255));
}

void setMotorRight(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  }
  analogWrite(ENB, constrain(speed, 0, 255));
}

