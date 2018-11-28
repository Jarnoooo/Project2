#define echoPin 8
#define trigPin 9

void setup() {
  // put your setup code here, to run once:


pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(9600);
}

void loop() {
  
digitalWrite(echoPin, LOW);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

int waardeSonar = pulseIn(echoPin,HIGH);

Serial.println(waardeSonar); 
delay(200);

if (waardeSonar < 1000){
  motor.stop();
  Serial.println("STOPPP!!!");
}else{
 motor.driveForward();
  Serial.println("Weer verder rijden");
  }

}
