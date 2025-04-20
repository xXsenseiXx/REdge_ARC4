#include <QTRSensors.h>

//motor 1
#define IN1 7
#define IN2 8
#define ENA 5

//motor 2
#define IN3 9
#define IN4 10
#define ENB 6

#define IR QTRNoEmitterPin


QTRSensors qtr;

// PID properties
const double Kp = 1 ;
const double Kd = 0;
const double ki = 0;
const double Goal = 3500;

double lastError = 0;
int baseSpeed = 100;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  // #2 motor
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(IR);

  calibration();
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

  
  int error = position - Goal;
   // tuning needed
  float adjustment = Kp * error + Kd*(error - lastError);
  
  lastError = error;

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  float leftSpeed = baseSpeed + adjustment;
  float rightSpeed = baseSpeed - adjustment;

  /*if(position==7000) {
    sharpturn();
  }*/

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
  analogWrite(ENA, constrain(speed, 90, 90));
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
  analogWrite(ENB, constrain(speed, 90, 90));
}

void calibration () {
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(9600);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call 

  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  Serial.println("calibration started");
  for (uint16_t i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("calibration Ended");
}

void sharpturn() {
  if (lastError > 0) {
    // Turn right to find the line (means line was more to the right before losing it)
    digitalWrite(IN1, HIGH);  // left motor forward
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);   // right motor reverse
    digitalWrite(IN4, HIGH);
  } else {
    // Turn left to find the line
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, 90);  // You can tune the turning speed
  analogWrite(ENB, 90);
  delay(250);  // Time to spin - adjust based on turning capability
}