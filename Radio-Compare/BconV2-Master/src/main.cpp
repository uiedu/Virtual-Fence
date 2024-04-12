/*****************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/03/20
  Modified by Dev Shrestha - 04/05/24
  1. Disabled Display 1.3" OLED by commenting out four places search for Comment 1 through Comment 4 for details
  2. Changed Baud rate to 15200

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#include <SPI.h>
#include <SX128XLT.h>
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <EEPROM.h>
#include "Settings.h"


SX128XLT LT;
SPIFlash flash(Flashpin, expectedDeviceID);

/* Comment 01
#ifdef ENABLEOLED
#include <U8x8lib.h>                                        //https://github.com/olikraus/u8g2 
//U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);      //standard 0.96" SSD1306
U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);     //1.3" OLED often sold as 1.3" SSD1306
#endif
*/
uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;
char CharIn;

//UDF definitions
void FlashWrite(String Str);
void FlashRead();
uint8_t d = 10;
void Broadcast(uint8_t Code);
void loop()
{
  /* Steps to range and record
  1. Range and record from Nanotron
  2. Range and record from Sx1280 with settings 1..53
  3. Go to 1  

  */
    
  // 1. Range using Nanotron
  for(RangingAddress = 1; RangingAddress <= Stations; RangingAddress++){
    Msg2Write ="";
    Msg2Write = millis();
    Msg2Write += "\n\r";
    Msg2Write += "N,"; 
   //Serial.println("Ranging to nanotron ID: B0" + String(RangingAddress));
   Serial2.println("RATO 0 000000000B0" + String(RangingAddress));
   delay(d);
   Msg2Write += RangingAddress;
   Msg2Write += ",";
   while (Serial2.available() > 0){ 
    CharIn = Serial2.read();
    Msg2Write += CharIn;     
   }
    
    
    //2. Range Using SX1280
    //Request setting.
    LT.setupLoRa(Frequency, 0, LORA_SF7, LORA_BW_0400, LORA_CR_4_5); 
    for (int code = 0; code<100;code++){


    }




    Msg2Write += "S,"; 
    LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);
    //delay(packet_delaymS);
    IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
    if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
    digitalWrite(LED1, HIGH);
    range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
    if (range_result > 800000) {range_result = 0;}
    distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment);
    RangingRSSI = LT.getRangingRSSI();
   
    Msg2Write += distance;
    Msg2Write += ",";
    Msg2Write += RangingRSSI;
    Msg2Write += ",";  
    Msg2Write += Bandwidth;
    Msg2Write += ",";
    Msg2Write += SpreadingFactor;
    Msg2Write += ",";
    Msg2Write += RangingTXPower;
    Msg2Write += ",";
    Msg2Write += Calibration;
    }
    Serial.println(Msg2Write);

    delay(1000);
  } 
}

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(4, 125);                                       //two quick LED flashes to indicate program start
  int d = 50; //ms delay between commands
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  
  //Setup Nanotron radios
  Serial.println("BRAR 0"); 
  Serial2.println("BRAR 0");  //Do not broadcast ranging results (Ranging is performed by Beacon)
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }
  
  Serial.println("EBID 0"); 
  Serial2.println("EBID 0"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }
  
  Serial.println("SNID 000000000C01");
  Serial2.println("SNID 000000000C01");
  delay(d); 
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }
  
  Serial.println("GNID");
  Serial2.println("GNID"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }
  
  Serial.println("GPIO 0 0 0 0 2");
  Serial2.println("GPIO 0 0 0 0 2"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }

  Serial.println("GPIO 1 0 0 0 2");
  Serial2.println("GPIO 1 0 0 0 2"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }
  
  Serial.println("GPIO 2 0 0 0 2");
  Serial2.println("GPIO 2 0 0 0 2"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }

  Serial.println("GPIO 3 0 0 0 2");
  Serial2.println("GPIO 3 0 0 0 2"); 
  delay(d);
  while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
  }


  // Initialize Lora Radio
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
   // Serial.println(F("Device found"));
    //led_Flash(2, 125);
    //delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast flash indicates device error
    }
  }


 
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);

  delay(1000);
}

//UDFs
void FlashWrite(String Str){
    
    Str += "\n\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
       
    if (Last_Address < 16777000){  // W25Q128JV Flash memroy 128Mbits = 16 MB = 16*2^6 = 16777216 Bytes // Rounding down to 1000
      char msg[Str.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      Str.toCharArray(msg,Str.length());
      digitalWrite(Flashpin, LOW); // Turnon Flash
      flash.writeBytes(Last_Address, &msg,Str.length()-1);
      Last_Address += Str.length()-1; 
      EEPROM.put(1, Last_Address);    
    }
    else{
      Serial.print("Memory full");
    }
}  

void FlashRead(){
  Serial.print("Reading memory from address 0 to address ");
  Serial.println(Last_Address);
  uint32_t Counter = 0;
  for(Counter = 0; Counter < Last_Address; Counter++){
    Serial.write(flash.readByte(Counter));
  }
   Serial.println();
   Serial.println("Done");
}

// This function returns the command within a pair of () sent from serial port
String ReadCommand() { //Return the command in pair of ()
// This is a better code to ensure commnd from serial is read properly 
// All command starts with ( and end with )
    static bool recvInProgress = false; //Static is necessary not to terminte reading at the middle of the line
    char startMarker = '(';
    char endMarker = ')';
    char rc;
    String SerialMsg ="";
    bool EndCommand = false;
    //See if serial message is available, newCommand is false at this point
    while (Serial.available() > 0 && EndCommand == false) {
        rc = Serial.read();

        if (recvInProgress == true) { //recvInProgress is False when first char is read so skip this condition to Else
            if (rc != endMarker) { //Keep reading
                SerialMsg += rc;
            }
            else { //End marked found
                recvInProgress = false;
                EndCommand = true;
            }
        }

        else if (rc == startMarker) { //No need to read until start marker is found
            SerialMsg = ""; //Start marker found get ready to read
            recvInProgress = true;
        }
    }
    return SerialMsg;
}


//UDF
void Broadcast(uint8_t Code){
  String Msg = "<0," + String(Code)+">";
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
 uint8_t *u = (uint8_t *) buff;
    // Transmit back
    //TXPacketL = sizeof(buff); 
    delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
    LT.transmitIRQ(u, TXPacketL-1, 500, TXpower, WAIT_TX); //This did not take Char array
   
 }

 void ParseCode(uint8_t MsgCode){
 };