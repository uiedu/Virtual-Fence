#include <SoftwareSerial.h>
#define rxPin  7
#define txPin  8

SoftwareSerial mySerial(rxPin,txPin,true);
void setup() {
 
  // initialize both serial ports:
  Serial.begin(115200);
  mySerial.begin(115200);
  
}

void loop() {
 
  while (Serial.available() > 0){ 
    char x = Serial.read();
     Serial.write(x);
     mySerial.write(x);
     
 }

  Serial.println("gnid"); 
  mySerial.println("gnid");  //Do not broadcast ranging results (Ranging is performed by Beacon)
  delay(100);
  while (mySerial.available() > 0){ 
     Serial.write(mySerial.read());
  }
 Serial.println(); 
 delay(2000);
}
