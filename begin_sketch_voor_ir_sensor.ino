void setup() {

pinMode(2,INPUT);
pinMode(3,INPUT);
pinMode(4,INPUT);
pinMode(5,INPUT);



Serial.begin(9600);

}

void loop() {
int irsensorlinks = digitalRead(2);
int irsensorrechts = digitalRead(3);
int irsensormidden1 = digitalRead(4);
int irsensormidden2 = digitalRead(5);

Serial.println(irsensorlinks);

if (irsensorlinks == 0 && irsensorrechts == 1) {
  // bocht naar links
  // motor rechts sneller laten draaien
}
if (irsensorlinks == 1 && irsensorrechts == 0){
  // bocht naar rechts 
  // motor links sneller laten draaien
}
if (irsensormidden1 == 1 && irsensormidden2 == 1){
  //doorrijden geen bocht
}


}
