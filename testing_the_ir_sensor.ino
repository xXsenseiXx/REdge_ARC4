#include <QTRSensors.h>

//motor 1
#define IN1 7
#define IN2 8
#define ENA 5

//motor 2
#define IN3 9
#define IN4 10
#define ENB 6

bool allWhite = true;
bool allBlack = true;

QTRSensors qtr;

// PID properties
const double Kp = 0.19 ;
const double Kd = 0.001;
const double ki = 0;
const double Goal = 3500;

double lastError = 0;
int baseSpeed = 100;

//flags
int inters;
int edge;

//ultrasonic distance
double front_obst;
double right_obst;
double left_obst;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  Serial.begin(115200);

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
  //qtr.setSamplesPerSensor(3);

  calibration();
  // print the calibration minimum values measured when emitters were on
  /*
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println()*/

  // print the calibration maximum values measured when emitters were on
  /*
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  */
  delay(1000);
}

void loop()
{

  //uint16_t positionW = qtr.readLineWhite(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = position - Goal;
  float adjustment = Kp * error + Kd*(error - lastError);  
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //if(sensorValues[i]<850){sensorValues[i]=0;}else{sensorValues[i]=1000;}
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  float leftSpeed = baseSpeed - adjustment;
  float rightSpeed = baseSpeed + adjustment;


  //if(sensorValues[0]==sensorValues[1]==sensorValues[2]==sensorValues[3]==sensorValues[4]==sensorValues[5]==sensorValues[6]==sensorValues[7])

  /*if(abs(error) >= 400 && abs(error) <= 1000){edge = 1;}

  if (sensorValues[0]<=100 && sensorValues[1]<=100 && sensorValues[2]<=100 && sensorValues[3]<=100 && sensorValues[4]<=100 && sensorValues[5]<=100 && sensorValues[6]<=100 && sensorValues[7]<=100&& edge==1) {
    sharpturn();
    Serial.println("Sharp turn triggered !!!!!!!!!!!!!!!!!!!!!!!!!!!");
    edge = 0;
  }
  
  else {*/
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
  //}

  //controle direction
  if(position > 6800 ){allWhite = true;}
  if(sensorValues[0]>900 &&sensorValues[1]>900 && sensorValues[2]>900 && sensorValues[3]>900 && sensorValues[4]>900 && sensorValues[5]>900 && sensorValues[6]>900 && sensorValues[7]>900){allBlack=true;} 



  if(allBlack){
    
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
    

    allBlack = false;
  }
  else if(allWhite){
    sharpturn();
    allWhite = false;
  }
  else{
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
  }


  
  Serial.print(" | Position: ");
  Serial.print(position);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
  
  lastError = error;
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
  analogWrite(ENA, constrain(speed, 0, 90));
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
  analogWrite(ENB, constrain(speed, 0, 90));
}

void calibration () {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call 

  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  Serial.println("calibration started");
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("calibration Ended");
  delay(500);
}

void sharpturn() {
  uint32_t startTime = millis();
  int turnSpeed = 150;
  if (lastError > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);

  while (millis() - startTime < 1000) { // Max 500ms turn
    uint16_t position = qtr.readLineBlack(sensorValues);
    if (position > 2000 && position < 5000) break; // Line found
  }
}

void ultrason_ON() {
  
}