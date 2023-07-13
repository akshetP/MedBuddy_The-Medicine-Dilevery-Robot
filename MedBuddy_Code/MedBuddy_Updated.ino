/*Pin Connections:
 HC05 Bluetooth Module:
   Rx  - 3
   Tx  - 2
   Gnd - Gnd of Arduino Uno
   Vcc - Vcc of Arduino Uno

 Ultrasonic Sensor SR04:
   Gnd - Gnd of Arduino Uno
   Echo - A0
   Trig - A1
   Vcc  - Vcc of Arduino Uno

 Motor Driver L298DC:
   IN1- 4
   IN2- 5
   IN3- 6
   IN4- 7
*/

#include <SoftwareSerial.h>

#define echopin A0 // Echo pin
#define trigpin A1 // Trigger pin

SoftwareSerial bluetooth(2, 3); // Configure pins 2 and 3 as Tx and Rx.

const int minimumDistanceToObstruction_cm = 35;
const int motor2Pin1      = 6;  // pin 6  on L293D IC
const int motor2Pin2      = 7;  // pin 7  on L293D IC
const int motor2EnablePin = 10; // pin 10 on L293D IC

const int motor1Pin1      = 4;  // pin 4 on L293D IC
const int motor1Pin2      = 5;  // pin 5 on L293D IC
const int motor1EnablePin = 9;  // pin 9 on L293D IC

int  targetSpeed              = 130; // TODO - what are the units of target speed ?
int  distanceToObstruction_cm = 0;
bool bMovingForwards          = false;

void setup()
{
  pinMode(trigpin,         OUTPUT);
  pinMode(echopin,         INPUT );
  pinMode(motor1Pin1,      OUTPUT);
  pinMode(motor1Pin2,      OUTPUT);
  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor2Pin1,      OUTPUT);
  pinMode(motor2Pin2,      OUTPUT);
  pinMode(motor2EnablePin, OUTPUT);

  Serial.begin(   9600);
  bluetooth.begin(9600);
	
  delay(500); // TODO - is this required
}

void updateUI()
{
  const int updateInterval = 200;

  static int timer = 0;
  
  timer++;
  Serial.println(timer);

  if(timer >= uiUpdateInterval)
  {
// TODO - why are we clamping the distance
    if(distanceToObstruction_cm > 200)
    {
      distanceToObstruction_cm = 200;
    } 
    bluetooth.print("A");
    bluetooth.print(";");
    bluetooth.print(distanceToObstruction_cm);
    bluetooth.println(";");
	
	timer = 0;
  }
}

processCommand()
{
  if(bluetooth.available() == 0) return; // no command received

  int command = bluetooth.read(); 

  Serial.println(command);

  if(command > 10) // any commands > 10 are new speed request so update the speed outputs
  {
    targetSpeed = command;

// Sets the PWM percentage for the motor enable pins and hence controls the motor speed
    analogWrite(motor1EnablePin, targetSpeed);
    analogWrite(motor2EnablePin, targetSpeed);
	  
    return;
  }
        
  switch(command)
  {
    case 1: forward();      break;
    case 2: backwardward(); break;
    case 3: turnLeft();     break;
    case 4: turnRight();    break;
    case 5: stop();         break;
  }
}

void loop()
{
  doCollisionDetection();
  updateUI();
  processCommand();
}

void doCollisionDetection()
{
// TODO - should distanceToObstruction_cm be set to zero if we are not moving forwards
// TODO - do we need collision detection when moving backwards
  if(bMovingForwards == false) return;

  distanceToObstruction_cm = getDistanceToObstruction_cm();

  if(distanceToObstruction_cm < minimumDistanceToObstruction_cm)
  {
	  Stop();
  }
}

void setMotorPins(int motor1posState, int motor1negState, int motor2posState, int motor2negState, bool isForwardsMovement, const char* pDebugStr)
{
    digitalWrite(motor1Pin1, motor1posState);
    digitalWrite(motor1Pin2, motor1negState); 
    digitalWrite(motor2Pin1, motor2posState);
    digitalWrite(motor2Pin2, motor2negState);

	bMovingForwards = isForwardsMovement;
	
	println(pDebugStr);
}

void forward()
{
  setMotorPins(HIGH, LOW, LOW, HIGH, true,  "Going Forwards!");
}

void backward()
{
  setMotorPins(LOW, HIGH, HIGH, LOW, false, "Reversing!");
}

void turnRight()
{
  setMotorPins(LOW, HIGH, LOW, HIGH, false, "Turning Right!");
}

void turnLeft()
{
  setMotorPins(HIGH, LOW, HIGH, LOW, false, "Turning Left!");
}

void Stop()
{
  setMotorPins(LOW, LOW, LOW, LOW,   false, "STOPPING!");
}

long getDistanceToObstruction_cm()
{
// Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter. 
  const long sound_usPerCm = 29;

// trigger a sound pulse	
  digitalWrite(trigpin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin,HIGH);

// wait for echo
  delayMicroseconds(10);
  
  long echoTime_us = pulseIn(echopin,HIGH);

// Divide the time for the echo to be detected by time per cm and then by 2 (because the sound has to travel to the object and then back to the sensor).
  return echoTime_us / sound_usPerCm / 2; 
}
