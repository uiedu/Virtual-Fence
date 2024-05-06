/*******************************************************************************************************
  Programs for Arduino to setup Beacon V2 Virtual Fence (Red Boards) with both radios
  Program uses base code from Stuwart project for Sx1280
  Modified by Dev Shrestha 04/20/2024
  1. Serial monitor baud rate is set at 115200
  2. Set up the Nanotron radio V3 from Inpixion. Successful ranging response will blink first LED from left
  3. Set up Sx1280 radio from Semtech. Successful ranging response will blink second LED from left
  4. Use Settings.h to change any settings local to a radio
  
*******************************************************************************************************/

#include <SPI.h>
#include <SX128XLT.h>
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <EEPROM.h>
#include "Settings.h"
//
// Set Station ID
uint32_t MyID = 10;         //must match address in master (Single didgit only for now, Generalize to have any number)


SX128XLT LT;
SPIFlash flash(Flashpin, expectedDeviceID);

//Quick Setting
uint32_t MasterID = 0;                //must match address in recever
//uint8_t Stations = 5;               
bool debug = false;
bool Tweet = false;
bool Record = false;
bool NewMsg = false;
//UDF
//UDF definitions
//General
void PinInitialize();
bool CheckSerial(); //Reads the code sent from serial console
void ParseCode(uint8_t Code);
bool ParseMsgxyz(String Msg, bool IsSerial);    // Parse MessageIn whenCalled For and splits into Who this message for, and the Message Code. If Message for MyID, execute

void led_Flash(uint16_t flashes, uint16_t delaymS);
void led_code(uint8_t x);
void led_Code2(uint8_t x);

//Flash
void FlashInitialize();
void FlashReset();
void FlashWrite(String Str);
void FlashRead();


//Sx1280
void ListenAny(uint32_t Duration);
bool CheckRadio(uint16_t TimeOut);

bool Request(uint32_t RadioID, uint8_t Code);
void SetSx1280Mode(uint8_t mode);
/*
void ReadSetting();
bool Switch2Slave(uint16_t TimeOut);
bool RecordBroadcast(uint16_t TimeOut);
*/

//Global variables
char CharIn;
String MsgOut;
String MsgIn;
String SerialIn;
uint8_t MsgCode = 0;
uint8_t SerialCode = 0;
uint32_t SendTo = 0;
uint32_t Sender = 0;
uint8_t d = 50; //Standary delay time in ms
uint32_t response_sent;
uint8_t RadioMode = 0; //Listen mode by default

// Ping Variables
uint16_t PayloadCRC;
uint8_t TXPacketL;
#define ACKtimeout 500                         //Acknowledge timeout in mS                      
#define TXtimeout 500                          //transmit timeout in mS. If 0 return from transmit function after send.  
#define TXattempts 1                          //number of times to attempt to TX and get an Ack before failing  

//Pong Variables
const uint8_t RXBUFFER_SIZE = 251;              //RX buffer size, set to max payload length of 251, or maximum expected length
uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into
#define ACKdelay 50                            //delay in mS before sending acknowledge                    
#define RXtimeout 1000                          //receive timeout in mS. only for radio mode 
uint8_t RXPacketL;                              //stores length of packet received
uint8_t RXPayloadL;                             //stores length of payload received
uint8_t PacketOK;                               //set to > 0 if packetOK
int16_t PacketRSSI;                             //stores RSSI of received packet
uint16_t LocalPayloadCRC;                       //locally calculated CRC of payload
uint16_t RXPayloadCRC;                          //CRC of payload received in packet
uint16_t TransmitterNetworkID;                  //the NetworkID from the transmitted and received packet 
/////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  if(CheckSerial()){
    if(ParseMsgxyz(SerialIn, true)){
      if(NewMsg){  //If New message to be sent wait for Code 11 from Sender
        Serial.println(SerialCode);
        if(CheckRadio(5000)){
          if(ParseMsgxyz(MsgIn, false)){ ParseCode(MsgCode);}
        }
      }
      else{
        ParseCode(SerialCode);
      }
    }
  }
  ListenAny(1000);
  
}


void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
   //Inilize pins as output or input
  PinInitialize();
  FlashInitialize(); //Check if flash is ready
  SetSx1280Mode(0);
  
}

//UDF
/////////////////////////////////////////////////////////////////////////////////////////////
void PinInitialize(){
  // Set pins as output or input
  pinMode(LED1, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED2, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED3, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED4, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED5, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(SX1280LED, OUTPUT);
  pinMode(NSS, OUTPUT);                   //setup NSS pin as output for radio
  pinMode(Flashpin,OUTPUT);               //setup pin as output for flash memory
  pinMode(NRESET, OUTPUT);
  pinMode(RFBUSY, INPUT);                                
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);                 //By default radio is kept low it is listening
  digitalWrite(NRESET,HIGH);
  digitalWrite(SX1280LED, HIGH);
  led_code(0);
  led_Code2(0);
  if(debug){Serial.println("Pin Initialized.");}

}

void FlashInitialize(){
  if(debug){Serial.println("Initializing Flash Memory.");}
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash

  if (flash.initialize())
  {
    if (debug){Serial.println("Flash initilaized !");}
    led_code(7);//7 = Flash Initialized
  }
  else {
    Serial.print("Flash initialization FAIL, expectedDeviceID(0x");
    Serial.print(expectedDeviceID, HEX);
    Serial.print(") mismatched the read value: 0x");
    Serial.println(flash.readDeviceId(), HEX);
    led_code(8);
    while(true);
  }

 // In order to avoid overwriting flash memory, the last location of memory writtten is saved in EEPROM
 // Memory location is a valid one if EEPROM byte 0 is 170. 
 // I am taking a chance that at start up, EEPROM byte 0 is not 10101010 (=170). This may happen randomly, so to make it fail safe, write EEPROM 0 to value other than 170 and clear flash before using
 // Most of the time this should work 
  if (EEPROM.read(0) != 170){ //This is the first time chances are Byte 0 is not 170
      EEPROM.write(0,0xaa);  //0xaa = 10101010 = 170
      EEPROM.put(m,Last_Address); // EEPROM, starting Byte m, write Last_Address which is 0 when inialized. It will take 4 bytes as Last_address is Unsigend Long Integer
      flash.chipErase(); //Erage the flash
      while(flash.busy()){led_code(4);}; //Wait until all erased  Code 4 = Flash busy
      led_code(0);
  }
  else {
      EEPROM.get(m,Last_Address);  //Byte 0 is 170, so falsh has been initialized. Read the Last_address
  } 
  digitalWrite(Flashpin, HIGH); // Turn OFF Flash  
}

// FlashReset resets the flash. Can be invoked with code:101
void FlashReset(){
  Serial.print("Resetting Flash: ");
  //while(digitalRead(RFBUSY));//Wait while radio is busy
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash
  Last_Address = 0; 
  EEPROM.write(0,0xaa);
  EEPROM.put(m,Last_Address); // Reset Pointer to 0;
  flash.chipErase(); //Erage the flash (Must eraase to turn all bits to 1)
  while(flash.busy()){led_code(4);}; //Wait until all erased  Code 4 = Flash busy
  led_code(0);
  
  digitalWrite(Flashpin, HIGH); // Turn OFF Flash
  digitalWrite(NSS, LOW); // Turn ON Radio
  Serial.println("Reset Done");
  
}

void FlashWrite(String sMsg){
    
    sMsg += "\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
    led_code(4);    
    if (Last_Address < 16777000){  // W25Q128JV Flash memroy 128Mbits = 16 MB = 16*2^20 = 16777216 Bytes // Rounding down to 1000
      char cMsg[sMsg.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      sMsg.toCharArray(cMsg,sMsg.length());
      //while(digitalRead(RFBUSY)); //Wait while radio is busy
      digitalWrite(NSS, HIGH); // Turnoff Radio
      digitalWrite(Flashpin, LOW); // Turnon Flash
      
      flash.writeBytes(Last_Address, &cMsg,sMsg.length()-1);
      while(flash.busy()){led_code(4);};
      Last_Address += sMsg.length()-1; 
      EEPROM.put(m, Last_Address);  

      digitalWrite(Flashpin, HIGH); // Turn OFF Flash
      digitalWrite(NSS, LOW); // Turn ON Radio 
      led_code(0);

    }
    else{
      Serial.print("Memory full");
    }
}  

void FlashRead(){
  Serial.print("Reading memory from address 0 to address ");
  
  Serial.println(Last_Address);
  led_code(4);
  //while(digitalRead(RFBUSY)); //Wait while radio is busy
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash
  
  uint32_t Counter = 0;
  for(Counter = 0; Counter < Last_Address; Counter++){
    Serial.write(flash.readByte(Counter));
  }
   while(flash.busy()){led_code(4);};
   digitalWrite(Flashpin, HIGH); // Turn OFF Flash
   digitalWrite(NSS, LOW); // TurnON Radio 
   
   Serial.println();
   Serial.println(" Reading Done");
   led_code(0);
}



// This function returns the command within a pair of () sent from serial port
// This function returns true if there is a serial messge within a pair of (). The message inside () is writtine to SerialMsgIn
bool CheckSerial() { //Return the command in pair of ()
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
    if (SerialMsg ==""){
      //Serial.flush();
      return false;
    }
    else{
      SerialIn =  SerialMsg;
      //Serial.flush();
      return true ;
    }
    
}


void ListenAny(uint32_t Duration){
  
  SetSx1280Mode(0); //Set radio as slave
  RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, Duration, WAIT_RX); //
  if (RXPacketL>0){
    LT.printASCIIPacket(RXBUFFER, RXPacketL);
    if (Record){ 
      digitalWrite(NSS, HIGH); // Turnoff Radio
      digitalWrite(Flashpin, LOW); // Turnon Flash
      
      flash.writeBytes(Last_Address, &RXBUFFER,RXPacketL); //sMsg.length() is null Character
      while(flash.busy());
      Last_Address += RXPacketL; 
      EEPROM.put(m, Last_Address);  

      digitalWrite(Flashpin, HIGH); // Turn OFF Flash
      digitalWrite(NSS, LOW); // Turn ON Radio 
      
      
      FlashWrite(MsgOut); } 
  }  
}
  

// Check if there is any incoming message for this radio
// Acknowledge Automatically if message is for this radio after ACKdelay
// Set MessageIn to received message and return true
// If no message received withon Timeout return false


bool CheckRadio(uint16_t TimeOut){ 
  
   if (RadioMode != 0){SetSx1280Mode(0);}
   PacketOK = LT.receiveReliableAutoACK(RXBUFFER, RXBUFFER_SIZE, MyID, ACKdelay, TXpower, TimeOut, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout
   //Will respond to any radio message. If message is not for this radio PacketOk = 0
   if (PacketOK > 0)
   {
      RXPacketL = LT.readRXPacketL();               //get the received packet length
      RXPayloadL = RXPacketL - 4;                   //payload length is always 4 bytes less than packet length
      PacketRSSI = LT.readPacketRSSI();             //read the received packets RSSI value
      //if the LT.receiveReliable() returns a value > 0 for PacketOK then packet was received OK

      MsgIn = "";
      for (uint8_t index = 0; index < RXPayloadL; index++){
      MsgIn += (char)RXBUFFER[index];
      }
      return true;
      
    }
    else
    {
      return false;
    }
  

}


 
void ParseCode(uint8_t Code){
  /*
  Code 0 is prohibited 
  
  Code 1 = Start Recording  Record = true
  Code 2 = Stop Recording
  Code 3 = Boradcast MsgOut (monitor radio ID) Tweet = true
  Code 4 = Do not Boradcast MsgOut 
  Code 5 = set GoRange to true  GoRange = true
  Code 6 = Set GoRange to false
  Code 7 = Flash Read
  Code 8 = Flash Reset
  Code 9 = Range back using Nantotron
  Code 10 = Range back using Sx1280
  Code 11 = Send Clear

  Code 20-200 = Radio Settings
  Code 201 -220 = Broadcast setting to each radio
 */
  Serial.print("Parse Code:");
  Serial.println(Code);
  if(Code == 1 )       {Record = true; Serial.println("Recording ON") ;}
  else if (Code == 2)  {Record = false; Serial.println("Recording OFF") ;}
  //else if (Code == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
  //else if (Code == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}
  //else if (Code == 5)  {GoRange = true;  Serial.println("Ranging ON");}
  //else if (Code == 6)  {GoRange = false;  Serial.println("Ranging OFF");}
  else if (Code == 7)  {FlashRead();}
  else if (Code == 8)  {FlashReset();}
  //else if (Code == 9)  {} //Only for Slave - Range back using Nantotron
  //else if (Code == 10)  {}//Only for Slave - Range back using Sx1280
  else if (Code == 11)  {//Only for Commander
    if (NewMsg){
      Serial.print("Seding");
      Serial.println(SerialCode);
      Request(SendTo,SerialCode);
      NewMsg = false;
    }
  }
  else{}// Do nothing
};

 

bool ParseMsgxyz(String Msg, bool IsSerial){
  // Split Msg with three fields x,y,z
  // if x = MyID, set SendTo to y and MsgCode to z & return true else return false 
     
  if(debug){Serial.println(Msg);}
  int c1 = Msg.indexOf(',');       //Find first comma position
  int c2 = Msg.indexOf(',', c1+1); //Find second comma position
  uint32_t ToID = Msg.substring(0, c1).toInt(); //Substring counts from 0. NOTE: Strangely returns character up to position of c1-1. C1 is not included
  if(ToID == MyID ){
    if(IsSerial){ 
      NewMsg = false;
      SendTo = Msg.substring(c1+1, c2).toInt();
      SerialCode = Msg.substring(c2+1).toInt();
      if(MyID != SendTo){NewMsg = true;}
    }
    else{
      Sender = Msg.substring(c1+1, c2).toInt();
      MsgCode = Msg.substring(c2+1).toInt();
    }
    return true;
  }
  
  else{
    return false;
  } 
}



bool Request(uint32_t RadioID, uint8_t Code){
  //Send Msg to RadioID with Code. Returns false if no acledgemnt received else return true
  SetSx1280Mode(0);
  String Msg = String(RadioID) + "," + String(MyID)+ "," + String(Code);
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
  uint8_t *u = (uint8_t *) buff;
  TXPacketL = LT.transmitReliableAutoACK(u, TXPacketL-1, RadioID, ACKtimeout, TXtimeout, TXpower, WAIT_TX);
  
  if (TXPacketL > 0){ return true;}
  else {return false;}
 
}


void led_code(uint8_t x){
  digitalWrite(LED1,bitRead(x,0));
  digitalWrite(LED2,bitRead(x,1));
  digitalWrite(LED3,bitRead(x,2));
}

void led_Code2(uint8_t x){
  digitalWrite(LED4,bitRead(x,0));
  digitalWrite(LED5,bitRead(x,1));

}

//Sets Sx1280 radio for COMM (mode = 0) or MASTER (1) or SLAVE (2)
void SetSx1280Mode(uint8_t mode){
  while(flash.busy()){led_code(4);} //Wait until flash is busy
  digitalWrite(Flashpin,HIGH);
  //digitalWrite(NSS,LOW);
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  //while(digitalRead(RFBUSY)){led_code(5);} //Wait while radio is busy
  if (mode==0){ //Set radio to receive regular
     LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
  }
  else if (mode==1){ //Set as Master    
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
  }
  else if (mode==2){ //Set as SLAVE    
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_SLAVE);
  }
  delay(1);
  //while (digitalRead(RFBUSY)){led_code(5);} //Wait until Radio busy
  led_Code2(mode);
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(SX1280LED, LOW);
    delay(delaymS);
    digitalWrite(SX1280LED, HIGH);
    delay(delaymS);
  }
}

