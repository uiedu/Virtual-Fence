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
uint32_t MyID = 1;         //must match address in master (Single didgit only for now, Generalize to have any number)

String NanoID = "000000000B0" + String(MyID);  // 9 0s followed by B01 to Bxx
String RemoteID = "000000000C00";

//Ranging Variables
uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;


SX128XLT LT;
SPIFlash flash(Flashpin, expectedDeviceID);

//Quick Setting
uint32_t MasterID = 0;                //must match address in recever
//uint8_t Stations = 5;               
bool debug = false;
bool Tweet = false;
bool Record = true;
//UDF
//UDF definitions
//General
void PinInitialize();
void Rmessage();
bool CheckSerial(); //Reads the code sent from serial console
void ParseCode();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void led_code(uint8_t x);
void led_Code2(uint8_t x);
bool ParseMsgIn();
void ParseSerial();
//Flash
void FlashInitialize();
void FlashReset();
void FlashWrite(String Str);
void FlashRead();

//Nanotron
void NanotronReset();
void NtRange();

//Sx1280
void Listen(uint32_t Duration);
bool Request(uint32_t RadioID, uint8_t Code);
void Broadcast(String Msg);
bool CheckRadio(uint16_t TimeOut);
void SetSx1280Mode(uint8_t mode);
void printPacketDetails();
bool SxRange();
bool SxRangeMe();
void SxErr();
/*
void ReadSetting();
bool Switch2Slave(uint16_t TimeOut);
bool RecordBroadcast(uint16_t TimeOut);
*/

//Global variables
char CharIn;
String MsgOut;
String MsgIn;
uint8_t MsgCode = 0;
uint8_t SerialCode = 0;
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
  SetSx1280Mode(0);
  //if(RadioMode !=0){SetSx1280Mode(0);} //Set radio as slave
  PacketOK = LT.receiveReliableAutoACK(RXBUFFER, RXBUFFER_SIZE, MyID, ACKdelay, TXpower, RXtimeout, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout
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
    ParseMsgIn();  
    
    ParseCode();
    if(MsgCode==5){
      delay(ACKdelay);
      Rmessage();//Adds to MsgOut = "R,RangingAddress,millis()"
      //if (Record){ FlashWrite(MsgOut); } 
      //if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}

      NtRange();//Range using Nanotron
      if (Record){ FlashWrite(MsgOut); } 
      if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
      led_Flash(1,50);
    }
    else if(MsgCode>=20 && MsgCode<= 127){
      if(debug){Serial.println(MsgCode);}
      SxRangeMe();
      delay(ACKdelay);
      SxRange();
      if (Record){ FlashWrite(MsgOut); } 
      if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);} 
      led_Flash(1,50);  
    }
  }
  
  

     
   /*Important observations
    1. LT.receiveRanging(MyID, 0, RangingTXPower, WAIT_RX) returns true ir false on every ranging request on air even if it is not for this radio
    2. Once it response to ranging, LT.receiveRanging must be called before it can response again
    3. If Master calls for repeated ranging to average the reading rapidly, make sure to reset the radio before next call
    for instance, I found out that that led_Flash time has be less than 30ms for Stuart's original ranging code.
    4. Calling LT.receiveRanging with NO_WAIT only works if radios are not changed. It returned false when 


   */
  if(CheckSerial()){ParseSerial();}
  
}


void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
   //Inilize pins as output or input
  PinInitialize();
  FlashInitialize(); //Check if flash is ready
  NanotronReset();   //Reset Nanotron radio 
  SetSx1280Mode(0);
  digitalWrite(SX1280LED, HIGH);
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

void NanotronReset(){
  //Setup Nanotron radios
  if(debug){Serial.println("Starting Nanotron Reset.");}
 //Setup Nanotron radios
  Serial.print("Setting Nanotron Radio to radio ID: "); 
  Serial.println(NanoID); 
  Serial.println("SFAC");
  Serial2.println("SFAC");
  delay(d);
  while (Serial2.available() > 0){ 
    Serial.write(Serial2.read());
  }
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
  
  Serial.println("SNID " + NanoID);
  Serial2.println("SNID " + NanoID);
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
  Serial2.println("SSET"); //Save to permanent address
  if(debug){Serial.println("Nanotron Reset done.");}
}

void Listen(uint32_t Duration){
  uint32_t startMS = millis();
  while (millis() < startMS + Duration){
    if(debug){Serial.println("Listening serial port");}
    if (CheckSerial()){ParseCode();}
    //if(debug){Serial.println("Listening Radio");}
    //if (CheckRadio()){ParseCode();}
  }
}

// This function returns the command within a pair of () sent from serial port
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
      return false;
    }
    else{
      SerialCode =  SerialMsg.toInt();
      return true ;
    }
    
}


void Broadcast(String Msg){
  SetSx1280Mode(0);
  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
  uint8_t *u = (uint8_t *) buff;
  LT.transmitIRQ(u, TXPacketL-1, 500, TXpower, WAIT_TX); //This did not take Char array
  //while(digitalRead(RFBUSY)); //Wait while radio is busy
  // Since radio is default mode do not turn off radio
 }

 void ParseCode(){
  /*
  Code 0 is prohibited 
  Code 255 = Stop Recording
  Code 1 = Start Recording
  Code 2 = Read Flash
  Code 3 = Boradcast MsgOut to radio 101 (monitor radio ID)
  Code 4 = Do not Boradcast MsgOut to radio 101 (monitor radio ID)
  Code 5 = Range Back using Nanotron
  Code 101 = Reset Flash (USE extreme Caution)


  Code 20-73 = BW = 3 factors, SF = 6 factor, Transmission Power = 0, 15, 31 (3 factors) Total 54
  Code 101 = Reset memory
  */
  if(MsgCode == 1 )       {Record = true;} //Since this is slave, echoing is better
  else if (MsgCode == 2)  {FlashRead(); }
  else if (MsgCode == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
  else if (MsgCode == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}

  else if(MsgCode >= 20 && MsgCode <=127) 
  {
    
    //EchoCode();
    //RadioMode = 0; //Go to ranging slave
    
    //Code 1 is Spreading Factors (6 levels)
    uint8_t Code1 = (MsgCode-20)%6; // This will vary from 0 to 5 and repeats for Msg Code from 2 to 55
    switch (Code1)
    {
      case 0: SpreadingFactor= LORA_SF5;break;
      case 1: SpreadingFactor= LORA_SF6;break;
      case 2: SpreadingFactor= LORA_SF7;break;
      case 3: SpreadingFactor= LORA_SF8;break;
      case 4: SpreadingFactor= LORA_SF9;break;
      case 5: SpreadingFactor= LORA_SF10;break;
      default:break;      
    }
    //Code 1 is Spreading Factors (6 levels)
    uint8_t Code2 = (uint8_t)((MsgCode-20)/6)%3;
    switch (Code2)
    {
      case 0: Bandwidth=LORA_BW_0400;break;
      case 1: Bandwidth=LORA_BW_0800;break;
      case 2: Bandwidth=LORA_BW_1600;break;
      default:break;      
    }
    uint8_t Code3 = (uint8_t)((MsgCode-20)/18)%6;
    switch (Code3)
    {
      case 0: RangingTXPower=0;break;
      case 1: RangingTXPower=1;break;
      case 2: RangingTXPower=3;break;
      case 3: RangingTXPower=7;break;
      case 4: RangingTXPower=15;break;
      case 5: RangingTXPower=31;break;
      default:break;      
    }
    if(debug){
    Serial.print(MsgCode);
    Serial.print(",");
    Serial.print(Code1);
    Serial.print(",");
    Serial.print(Code2);
    Serial.print(",");
    Serial.println(Code3);
    }

  }
  else if (MsgCode == 201)  { FlashReset();  }
  
  else if(MsgCode ==255) {Record = false;}
 };
 
void ParseSerial(){
  if(SerialCode == 1 )       {Record = true; Serial.println("Recording ON") ;}
  else if (SerialCode == 2)  {FlashRead();}
  else if (SerialCode == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
  else if (SerialCode == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}
  else if (SerialCode == 201)   {FlashReset();}
  else if (SerialCode ==255)     {Record = false; Serial.println("Recording OFF");}
  else{}

}


 bool CheckRadio(){
    //Listen if NetowrkID is MyID
    SetSx1280Mode(0);
    PacketOK = LT.receiveReliableAutoACK(RXBUFFER, RXBUFFER_SIZE, MyID, ACKdelay, TXpower, RXtimeout, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout
   
    if (PacketOK > 0)
    {
      RXPacketL = LT.readRXPacketL();               //get the received packet length
      RXPayloadL = RXPacketL - 4;                   //payload length is always 4 bytes less than packet length
      PacketRSSI = LT.readPacketRSSI();             //read the received packets RSSI value
      String  MsgIn = "";
      for (uint8_t index = 0; index < RXPayloadL; index++){
        MsgIn += (char)RXBUFFER[index];
      }
      return true;
     
    }
    else
    {
      //if the LT.receiveReliable() function detects an error PacketOK is 0
      uint16_t IRQStatus;

      IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register
      Serial.print(F("Error "));

      if (IRQStatus & IRQ_RX_TIMEOUT)                  //check for an RX timeout
      {
        Serial.print(F(" RXTimeout "));
      }
      else
      {
        //printPacketDetails();
      }
      return false;
    }

 }

bool ParseMsgIn(){
        
  if(debug){Serial.println(MsgIn);}
  int c1 = MsgIn.indexOf(',');       //Find first comma position
  int c2 = MsgIn.indexOf(',', c1+1); //Find second comma position
  uint32_t ToID = MsgIn.substring(0, c1).toInt(); //Substring counts from 0. NOTE: Strangely returns character up to position of c1-1. C1 is not included
  if(ToID == MyID ){
    Sender = MsgIn.substring(c1+1, c2).toInt();
    MsgCode = MsgIn.substring(c2+1).toInt();
    return true;
    }
  else{
    return false;
  }
 
}
void Rmessage(){
  if(debug){Serial.println("Starting R-message.");}
  MsgOut = "R,";
  MsgOut += millis(); 
  MsgOut += "\n\r";
  if(debug){Serial.println("R-message done.");}
}

void NtRange(){
  //Create Line 2 = "N,0,range in cm, RSSI,Temperature, Battery"
  //if(debug){Serial.println("Starting N-message.");}
  MsgOut += "N,"; 
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += MasterID;
  MsgOut += ",";
  Serial2.println("RATO 0 " +  RemoteID); //Returns "=Code,Distance in cm,RSSI \n\r"
  delay(d);

  while (Serial2.available() > 0)
  { 
    CharIn = Serial2.read();
    if (CharIn != '=' &&  CharIn != '\r' && CharIn != '\n' ){MsgOut += CharIn;  }    //Strip away =,/n and/r
  }
  
  //Get temperature and battery
  Serial2.println("GMYT"); //Returns "= Temperature in °C \n\r"
  delay(d);
  MsgOut += ",";
  while (Serial2.available() > 0){ 
    CharIn = Serial2.read();
    if (CharIn != '=' &&  CharIn != '\r' && CharIn != '\n' ){MsgOut += CharIn;  } 
  }
  Serial2.println("GBAT"); // Returns "= Battery voltage like 33 for 3.3V \n\r"
  delay(d);
  MsgOut += ",";
  while (Serial2.available() > 0){ 
    CharIn = Serial2.read();
    if (CharIn != '=' ){MsgOut += CharIn;  }     
  }
 
  //if(debug){Serial.println("N-message done.");}
}

bool SxRange()
{
  SetSx1280Mode(1); //Set Sx1280 radio as master
  if(debug){Serial.println("Starting S-message.");}
  MsgOut = "S,"; 
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += Sender;
  MsgOut += ",";
  //if(debug){Serial.println("Transmitting Ranging request.");}
  LT.transmitRanging(MyID, TXtimeoutmS, RangingTXPower, WAIT_TX);
  IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
  if ( IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
      //digitalWrite(LED1, HIGH);
      range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
      delay(d);
      if (range_result > 800000) {range_result = 0;}
      distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment); //Just a calculation
      RangingRSSI = LT.getRangingRSSI();
      MsgOut += distance;
      MsgOut += ",";
      MsgOut += RangingRSSI;
      MsgOut += ",";  
      MsgOut += Bandwidth;
      MsgOut += ",";
      MsgOut += SpreadingFactor;
      MsgOut += ",";
      MsgOut += RangingTXPower;
      
      MsgOut += "\r\n";
      return true;
    }
  else{
    MsgOut += "-1"; //Ranging not successful write negative distance to indicate invalid result
    MsgOut += "\r\n";
    return false;
    
  }
  if(debug){Serial.println("S-message done.");}
}

bool SxRangeMe(){
  SetSx1280Mode(2);

  //if(LT.receiveRanging(RangingAddress, 0, RangingTXPower, WAIT_RX)){ //If WAIT_RX, the radio is wait until ranging response is sent (No Time Limit)
  //endwaitmS = millis() + RXtimeout;
  LT.receiveRanging(MyID, 0, TXpower, NO_WAIT);

  endwaitmS = millis() + RXtimeout;

  while (!digitalRead(DIO1) && (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  if (millis() >= endwaitmS)
  {
    return false;
    //Serial.println("Error - Ranging Receive Timeout!!");
    //led_Flash(2, 100);                                             //single flash to indicate timeout
  }
  else
  {
    IrqStatus = LT.readIrqStatus();
    //digitalWrite(LED1, HIGH);

    if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
    {
      //response_sent++;
      //Serial.print(response_sent);
      //Serial.print(" Response sent");
      return true;
    }
    else
    {
      //Serial.print("Slave error,");
      //Serial.print(",Irq,");
      //Serial.print(IrqStatus, HEX);
      //LT.printIrqStatus();
      return false;
    }
    //digitalWrite(LED1, LOW);
    //Serial.println();
  } 
  
} 

/*bool SxRangeMe(){
  SetSx1280Mode(2);
  if(debug){Serial.println("Waiting to be ranged");}
  if(LT.receiveRanging(MyID, RXtimeout, RangingTXPower, WAIT_RX)){ //If WAIT_RX, the radio is wait until ranging response is sent (No Time Limit if RxTimeout = 0)
  if(debug){Serial.println("Response sent");}
  return true; 
  }
  else{
    //Serial.println("No Request");
    return false; 
  }
} 
*/

void led_code(uint8_t x){
  digitalWrite(LED1,bitRead(x,0));
  digitalWrite(LED2,bitRead(x,1));
  digitalWrite(LED3,bitRead(x,2));

  /*
  Code 0 = Idle
  1 = Sx1280 in COMM Mode
  2 = Sx1280 Ranging Master
  3 = Sx1280 Ranging Slave 
  4 = Flash busy
  5 = Sx1280 Busy

  */
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
void printPacketDetails()
{
  LocalPayloadCRC = LT.CRCCCITT(RXBUFFER, RXPayloadL, 0xFFFF);  //calculate payload crc from the received RXBUFFER
  TransmitterNetworkID = LT.getRXNetworkID(RXPacketL);
  RXPayloadCRC = LT.getRXPayloadCRC(RXPacketL);
 
}

void SxErr() 
{
 
  MsgOut = "S,"; 
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += RangingAddress;
  MsgOut += ",Err,";
  MsgOut += Bandwidth;
  MsgOut += ",";
  MsgOut += SpreadingFactor;
  MsgOut += ",";
  MsgOut += RangingTXPower;
  MsgOut += ",";
  MsgOut += Calibration;
  MsgOut += "\r\n";
  
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


/*
void EchoCode(){
  
  String Msg = "<" + String(Sender) +"," + String(MyID) + "," +String(MsgCode) + ">";
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
  uint8_t *u = (uint8_t *) buff;
  LT.ResetIRQ();
  delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
  LT.transmitIRQ(u, TXPacketL-1, 500, RangingTXPower, WAIT_TX); //This did not take 
  while(digitalRead(RFBUSY)); 
 }

 bool Listen2RangingRequest(uint16_t TimeOut){
 
  endwaitmS = millis() + TimeOut;
  while (!LT.receiveRanging(MyID, 0, RangingTXPower, WAIT_RX) && (millis() <= endwaitmS)){
    if (millis() >= endwaitmS)
      {
        Serial.println("Ranging request Timeout!!");
        return false;// Setup radio to  communication mode
        
      }
    else{
      IrqStatus = LT.readIrqStatus();
      if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
      {
        //Serial.println(" Response sent");
        return true;

      }
      else
      {
        Serial.print("Slave error,");
        Serial.print(",Irq,");
        Serial.print(IrqStatus, HEX);
        LT.printIrqStatus();
        return false;
      }
    }
  }
  return true; 
}

bool Switch2Master(uint8_t Retries){
  
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);
  //LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while(digitalRead(RFBUSY)); //Wait while radio is busy
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Sender, RANGING_MASTER);
  delay(1);
  LT.transmitRanging(Sender, TXtimeoutmS, RangingTXPower, WAIT_TX);
  rangeing_errors =0;
  IrqStatus = LT.readIrqStatus();
  if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
  {
    Serial.println(F("Valid"));
    range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
    if (range_result > 800000)      {        range_result = 0;      }
    distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment);
    RangingRSSI = LT.getRangingRSSI();
    MsgOut = "*"+ String(Sender) + "," + String(MyID)+ "," + String(distance) + "," + String(RangingRSSI) + "*";
    //Braodcast back the result
    Broadcast(MsgOut);
    RadioMode =2; //Switch to listen mode
    return true;
  }
  else{
    Serial.print("Ranging_error,");
    Serial.println(rangeing_errors++);
    if (rangeing_errors > Retries){
        RadioMode =2; 
        return false;
    }

  }
}
*/