int x;

void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2,HIGH);
  Serial.begin(115200);
  Serial.setTimeout(10);
  
}

void loop(){

 if (Serial.available() > 0) 
 {
  x = Serial.readString().toInt();
  Serial.print(x); 
 }

 if (x > 0) {
  digitalWrite(3, LOW);
  digitalWrite(3, HIGH);
 } else {
  digitalWrite(3, HIGH);
 }

  delayMicroseconds(x); 
}
