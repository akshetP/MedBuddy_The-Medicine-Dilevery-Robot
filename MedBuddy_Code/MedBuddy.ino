/*Pin Connections:
 HC05 BT Module:
 Rx- 3
 Tx- 2
 Gnd- Gnd of Arduino Uno
 Vcc- Vcc of Arduino Uno

 Ultrasonic Sensor SR04:
 Gnd- Gnd of Arduino Uno
 Echo- A0
 Trig- A1
 Vcc- Vcc of Arduino Uno

 Motor Driver L298DC:
 IN1- 4
 IN2- 5
 In3- 6
 In4- 7

*/

#include <SoftwareSerial.h>

#define echopin  A0 // Echo pin
#define trigpin A1 // Trigger pin

SoftwareSerial BT(2, 3); //Defining the pins 2 and 3 as Tx and Rx.

int motor2Pin1 = 6; // pin 6 on L293D IC
int motor2Pin2 = 7; // pin 7 on L293D IC
int motor2EnablePin = 10; // pin 10 on L293D IC

int motor1Pin1 = 4; // pin 4 on L293D IC
int motor1Pin2 = 5; // pin 5 on L293D IC
int motor1EnablePin = 9; // pin 9 on L293D IC

int state;
int Speed = 130; 

int timer = 0;

int distanceFwd;
long duration;

int chk = 0;
int set = 35;

void setup() {
    pinMode (trigpin, OUTPUT);
    pinMode (echopin, INPUT );
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor1EnablePin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(motor2EnablePin, OUTPUT);

    Serial.begin(9600);
    BT.begin(9600);
    delay(500); 
}

void loop() {

    if(BT.available() > 0){     
      state = BT.read(); 
      Serial.println(state);
      if(state > 10){
         Speed = state;}      
}   
        
   analogWrite(motor1EnablePin, Speed);
   analogWrite(motor2EnablePin, Speed);
   
   distanceFwd = data();
   if((distanceFwd<set) && (chk==1)){chk = 2; Stop();}
   if(distanceFwd>set){chk = 0;}
    
   if ((state == 1) && (chk==0)){chk = 1; forward();Serial.println("Go Forward!");}

   else if (state == 2){backward();Serial.println("Reverse!");}
    
   else if (state == 3){turnLeft();Serial.println("Turn LEFT");}
    
   else if (state == 4){turnRight();Serial.println("Turn RIGHT");}
    
   else if (state == 5) {Stop();Serial.println("STOP!");}    

timer = timer+1;
Serial.println(timer);

if(timer==200)
{
if(distanceFwd>200){distanceFwd=200;} 
 BT.print("A");
 BT.print(";");
 BT.print(distanceFwd);
 BT.println(";");
}
}

void forward(){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void backward(){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void turnRight(){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void turnLeft(){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void Stop(){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
}

long data()
{
  digitalWrite(trigpin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(10);
  duration=pulseIn (echopin,HIGH);
  return ((duration / 29) / 2); 
  //Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter. 
  //So, we have to divide the duration by 29 and then by 2, because the sound has to travel the distance twice. 
  //It travels to the object and then back to the sensor.
}
