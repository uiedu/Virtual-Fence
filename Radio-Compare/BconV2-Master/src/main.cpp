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

//#define DEBUG
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
String MsgIn;
String ReadCommand();
uint8_t d = 10;

void Broadcast(uint32_t RadioID, uint8_t Code);
uint8_t GetMsgCode(uint32_t MyID);
void loop()
{
  /* Steps to range and record
  1. Range and record from Nanotron
  2. Range and record from Sx1280 with settings 1..53
  3. Go to 1  

  */
    
  // 1. Range using Nanotron
  for(RangingAddress = 1; RangingAddress <= Stations; RangingAddress++){

    
    
    Msg2Write = "R,";
    Msg2Write += RangingAddress;
    Msg2Write += ",";
    Msg2Write += millis(); 
        
    Msg2Write += "\n\r";
    Msg2Write += "N,"; 
   //Serial.println("Ranging to nanotron ID: B0" + String(RangingAddress));
   Serial2.println("RATO 0 000000000B0" + String(RangingAddress));
   delay(d);
  
   while (Serial2.available() > 0)
   { 
    CharIn = Serial2.read();
    if (CharIn != '=' &&  CharIn != '\r' && CharIn != '\n' ){Msg2Write += CharIn;  }     
   }
   
   //Get temperature and battery
   Serial2.println("GMYT");
   delay(d);
   Msg2Write += ",";
   while (Serial2.available() > 0)
   { 
    CharIn = Serial2.read();
    if (CharIn != '=' &&  CharIn != '\r' && CharIn != '\n' ){Msg2Write += CharIn;  }
       
   }
   Serial2.println("GBAT");
   delay(d);
   Msg2Write += ",";
   while (Serial2.available() > 0)
   { 
    CharIn = Serial2.read();
    if (CharIn != '=' ){Msg2Write += CharIn;  }     
   }

    //2. Range Using SX1280
   // LT.Sleep();
    //digitalWrite(NSS,HIGH);
    //digitalWrite(NSS,LOW);
    //LT.setupLoRa(Frequency, 0, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
    //Broadcast(RangingAddress, 123);
   // Serial.print("Code Received: ");
    //Serial.println(GetMsgCode(MyID));
    //LT.resetDevice();
    LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
    
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
    delay(2000);
    for (RangingTXPower = 0; RangingTXPower<32; RangingTXPower+=5)
    {
      LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);
      delay(100);
      
      IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
      //while(LT.readIrqStatus());
      Msg2Write += "S,"; 
      if ( IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
        //digitalWrite(LED1, HIGH);
        range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
        delay(d);
        if (range_result > 800000) {range_result = 0;}
        distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment); //Just a calculation
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
        Msg2Write += "\r\n";
      }
      else{
        Msg2Write += "----";
        Msg2Write += "\r\n";
        
      }
      
    }  
      
      Serial.print(Msg2Write);
      FlashWrite(Msg2Write);
      
    
    
      
    }
    uint32_t startMS = millis();
    while (millis() < startMS+300){
      MsgIn = ReadCommand();
      if (MsgIn != ""){Serial.print(MsgIn);}
      if(MsgIn=="reset") 
      {
        Last_Address = 0; 
        EEPROM.put(1,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
        flash.chipErase(); //Erage the flash
        while(flash.busy()); //Wait until all erased
        Serial.println();
        Serial.println("Reset Done");
      }
      else if (MsgIn=="read")
      {
        FlashRead();
      }
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
 
  pinMode(LED1, OUTPUT);                                      //setup pin as output for indicator LED
  pinMode(NSS, OUTPUT);                                       ////setup NSS pin as output for radio
  
  pinMode(Flashpin,OUTPUT);
                                    //setup pin as output for indicator LED
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,HIGH);                                       ////setup NSS pin as output for radio
  digitalWrite(Flashpin,LOW);
   //Check if falsh is ready
  if (flash.initialize())
  {
    //Serial.println("Flash initilaized !");
    led_Flash(100, 2);//Blink(int DELAY_MS, byte loops)
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
      EEPROM.write(0,0xaa);  //0xaa = 10101010
      EEPROM.put(1,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
      flash.chipErase(); //Erage the flash
      while(flash.busy()); //Wait until all erased
  }
  else {
      EEPROM.get(1,Last_Address);  //Byte 0 is 170, so falsh has been initialized. Read the Last_address
  }  
  
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
 delay(d);
}

//UDFs
void FlashWrite(String sMsg){
    
    sMsg += "\n\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
       
    if (Last_Address < 16777000){  // W25Q128JV Flash memroy 128Mbits = 16 MB = 16*2^6 = 16777216 Bytes // Rounding down to 1000
      char cMsg[sMsg.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      sMsg.toCharArray(cMsg,sMsg.length());
      
      digitalWrite(NSS, HIGH); // Turnoff Radio
      digitalWrite(Flashpin, LOW); // Turnon Flash
      
      flash.writeBytes(Last_Address, &cMsg,sMsg.length()-1);
      Last_Address += sMsg.length()-1; 
      EEPROM.put(1, Last_Address);  

      digitalWrite(Flashpin, HIGH); // Turn OFF Flash
      digitalWrite(NSS, LOW); // TurnON Radio 

    }
    else{
      Serial.print("Memory full");
    }
}  

void FlashRead(){
  Serial.print("Reading memory from address 0 to address ");
  
  Serial.println(Last_Address);
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash
  
  uint32_t Counter = 0;
  for(Counter = 0; Counter < Last_Address; Counter++){
    Serial.write(flash.readByte(Counter));
  }
   digitalWrite(Flashpin, HIGH); // Turn OFF Flash
   digitalWrite(NSS, LOW); // TurnON Radio 
   
   Serial.println();
   Serial.println(" Reading Done");
   //delay(10000);
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
        delay(1);

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
void Broadcast(uint32_t RadioID, uint8_t Code){
  String Msg = "<" + String(RadioID) + "," + String(Code)+">";
  Serial.print(Msg);
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
  //Code for SF, BW, and Power
 };

 /////////////////////////////////////////////////////////////////////////////////////////////
 uint8_t GetMsgCode(uint32_t MyID){
   // Check if there is message in RXBuffer and if not wait for Timeout 
   // Will retun the Message code if valid message received for this radio ID. Else return 0. 
   // Valid message has this structure <RadioID, MessageCode> Both RadioID and MessageCode are  integers
    uint8_t RXPacketL;                              //stores length of packet received
    uint8_t RXBUFFER_SIZE = 255;                    //RX buffer size
    uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into
    uint8_t TimeOut = 100;                           //RxTimeout
   //Read buffer
    RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, TimeOut, WAIT_RX); //wait for a packet to arrive 
    
    //Is mesage length > 0?
    bool NoErr = (RXPacketL > 0);
    
    //In no error then Check if RSSI > -100 else return false
    if (NoErr){
        NoErr = NoErr and  LT.readPacketRSSI() >= -120;
      }
     else{
      //if (debug){Serial.println("No message");}
      return 0;
     } 
    //In no error then check if sentence structure is valid else return false
    
    if (NoErr){
        NoErr = NoErr and  (char)RXBUFFER[0] == '<' and (char)RXBUFFER[RXPacketL-1] == '>';
      }
    else{
      return 0;
     } 
    
    if (NoErr)
    {
      String  MsgIn = "";
      for (uint8_t index = 1; index < RXPacketL-1; index++){
        MsgIn += (char)RXBUFFER[index];
      }
      int c1 = MsgIn.indexOf(',');       //Find first comma position
      
      uint32_t BeaconID = MsgIn.substring(0, c1).toInt(); //Substring counts from 0. NOTE: Strangely returns character up to position of c1-1. C1 is not included
      uint8_t MsgCode = MsgIn.substring(c1+1).toInt();
      
      if(BeaconID == MyID ){
          return MsgCode;
        }
      else{
          return 0; //0 shoud not be included in code
      }     
    }
    else
    {
      return 0;
    } 
  
 }
/////////////////////////////////////////////////////////////////////////////////////////////