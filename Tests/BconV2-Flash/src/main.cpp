#include <Arduino.h>

// **********************************************************************************
// Dev Shrestha 1/30/2022
// Use to record GPS location once every x minutes to Flash
// Each sentence is Lat, Lon, Year,  Month, Day, Hour,  Minute,  Second,  HDOP,  Satellites,  Imput, Voltage,
// About 50 character long message 524000/50 = 10480 locations 
// Once memory is full LED will be continusly on
// Last memory location is written in EEPROM so restarting does not overwrite
// No wireless communication
// that has an onboard SPI Flash chip. This sketch listens to a few serial commands
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <EEPROM.h>
#include <SPI.h>



uint16_t expectedDeviceID = 0xEF40;
#define SERIAL_BAUD      115200
#define Radiopin         10
#define Flashpin         21
#define LED1              30 //defined as output in beaconinit
SPIFlash flash(Flashpin, expectedDeviceID);//0xEF30 for windbond 4mbit flash
unsigned long Last_Address = 0;
String lat_lon_time;
unsigned long counter = 0;
///////////////////////
void FlashWrite();
void FlashRead();
void Blink(int DELAY_MS, byte loops);

void setup(){

  Serial.begin(SERIAL_BAUD);
  //Serial2.begin(SERIAL_BAUD); Not needed as flash is through SPI
  
  pinMode(Radiopin, OUTPUT);
  pinMode(Flashpin, OUTPUT);

  //Check if falsh is ready
  if (flash.initialize())
  {
    Serial.println("Flash initilaized !");
    Blink(100, 2);//Blink(int DELAY_MS, byte loops)
  }
  else {
    Serial.print("Flash initialization FAIL, expectedDeviceID(0x");
    Serial.print(expectedDeviceID, HEX);
    Serial.print(") mismatched the read value: 0x");
    Serial.println(flash.readDeviceId(), HEX);
  }

 // Loggin to 4Mb Flash. In order to avoid over writing writing on flash memory, the last location of memory writtten is saved in EEPROM
 // Taking a chance that at start up, EEPROM byte 0 is not 10101010 (=170). This may happen randomly, so to make it fail safe, write EEPROM 0 to value other than 170 and clear falsh before using
 // Most of the time this should work 
  if (EEPROM.read(0) != 170){ //This is the first time chances are Byte 0 is not 170
      //EEPROM.write(0,0xaa);  //0xaa = 10101010
      //EEPROM.put(1,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
      //flash.chipErase(); //Erage the flash
      //while(flash.busy()); //Wait until all erased
  }
  else {
      EEPROM.get(1,Last_Address);  //Byte 0 is 170, so falsh has been initialized. Read the Last_address
  }  
    
    
}

void loop(){
  //FlashWrite();
  //delay(1000);
  FlashRead();
  delay(10000);
}


void FlashRead(){

Serial.println(Last_Address);
 digitalWrite(Radiopin, HIGH);
 digitalWrite(Flashpin, LOW);

  for(counter = 0; counter < Last_Address; counter++){
   
    Serial.write(flash.readByte(counter));
  }
   Serial.println();
   Serial.println("Done");
 digitalWrite(Flashpin, HIGH);
 digitalWrite(Radiopin, LOW);
}


void FlashWrite(){
  digitalWrite(Radiopin, HIGH);
  digitalWrite(Flashpin, LOW);
  lat_lon_time = "Test String to Write";
  lat_lon_time += "\n\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
      
  if (Last_Address < 16777000){
    char msg[lat_lon_time.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
    lat_lon_time.toCharArray(msg,lat_lon_time.length());
    
      
      digitalWrite(Flashpin, LOW); // Turnon Flash
    
      flash.writeBytes(Last_Address, &msg,lat_lon_time.length()-1);
      Last_Address += lat_lon_time.length()-1; 
      EEPROM.put(1, Last_Address);
      Blink(500, 3); //Blink 3 times after each successful write.
  
  }
  digitalWrite(Flashpin, HIGH);
  digitalWrite(Radiopin, LOW);  
   
  
  
  }  


void Blink(int DELAY_MS, byte loops)
{
  while (loops--)
  {
    digitalWrite(LED1,HIGH);
    delay(DELAY_MS);
    digitalWrite(LED1,LOW);
    delay(DELAY_MS);  
  }
}
