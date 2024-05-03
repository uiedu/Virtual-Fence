/*****************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/03/20
  Modified by Dev Shrestha - 04/05/24
    Ranging to SX1280 has follwoing sequence
      1. Master sends Setting requst with a code to radio it wants to range
      2. Slave receives the code, and confirms; then set its radio as slave with settings and wait up to 5 seconds to get ranging request
          If none received, slave will go to regular cummunication mode
      3. If confirmation code matches, Master changes its own settings and send range request
         
      4. If successful ranging received, it writes to memroty and switch to slave mode
      5. Slave and Master switches role and ranges again
         Remore radio tries to range up to 5 times if ranging is unsuccessful. then switches to comm mode.
         If ranging successulf Slve Broadcast the result
      5. After successful ranging, Master switches to comm mode and listens to broadcast message
      6  Records the boradcast message
      7. Repeats 1 - 6 with different code for same radio
      8. Once all settings are done, switches to next radio      
 

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#include <SPI.h>
#include <SX128XLT.h>
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <EEPROM.h>
#include "Settings.h"

// Set Station ID
uint32_t MyID = 0; 
String NanoID = "000000000C00";              //Nanotron ID  
uint8_t Stations = 5;                       //Number of Stations to query


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

//Quick Settings
                         // 0 of Master, 1..Stations for slaves
uint32_t RangingAddress = Stations;          //must match address in recever

bool debug = false;                          //Debug Message ON
bool Tweet = false;                         //Broadcast instead of serial out
bool Record = true;                        //Record to memory 
    

//UDF definitions

//General
void PinInitialize();
void Rmessage();
bool CheckSerial(); //Reads the code sent from serial console
void ParseCode();
bool ParseMsgIn();
void ParseSerial();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void led_code(uint8_t x);
void led_Code2(uint8_t x);

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
uint8_t buff[] = "Hello World";
uint16_t PayloadCRC;
uint8_t TXPacketL;
#define ACKtimeout 500                         //Acknowledge timeout in mS                      
#define TXtimeout 500                          //transmit timeout in mS. If 0 return from transmit function after send.  
#define TXattempts 1                          //number of times to attempt to TX and get an Ack before failing  

//Pong Variables
const uint8_t RXBUFFER_SIZE = 251;              //RX buffer size, set to max payload length of 251, or maximum expected length
uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into
#define ACKdelay 100                            //delay in mS before sending acknowledge                    
#define RXtimeout 500                         //receive timeout in mS.   
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
  
  // 1. Range to each radio in sequence from 1 to Stations 
  
  if (RangingAddress++ >= Stations){RangingAddress=1;} //Generats sequence 1...Stations
  led_code(RangingAddress); 
  Rmessage();//Adds to MsgOut = "R,millis()"
  //if (Record){ FlashWrite(MsgOut); } 
  //if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
  //  delay(1000);
  
  NtRange();//Range using Nanotron
  if (Record){ FlashWrite(MsgOut); } 
  if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
  
  if(Request(RangingAddress,5)){delay(1000);} //If Confirmed, wait for Nanotron to range back
  
  
  for(MsgCode=20; MsgCode <=127; MsgCode++){
    //Serial.println(MsgCode);
    if (Request(RangingAddress,MsgCode)){
      
      ParseCode();
      delay(ACKdelay);
      SxRange(); 
      SxRangeMe();
      if (Record){ FlashWrite(MsgOut); } 
      if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}   
      led_Flash(1,100);
    }
    else
    {
      //if transmitReliableAutoACK() returns 0 there was an error, timeout etc
      SxErr();
      if (Record){ FlashWrite(MsgOut); } 
      if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
      led_Flash(2,100);
    }
   //Listen(1000);
   if(CheckSerial()){ParseSerial();}
   SetSx1280Mode(0);
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
      //Serial.println(MsgCode);
      ParseCode();
    }
  
  }
 
    
     
}

void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  //Inilize pins as output or input
  led_code(1);
  PinInitialize();
  led_code(2);
  FlashInitialize();  //Check if flash is ready
  led_code(3);
  NanotronReset();   //Reset Nanotron radio
  led_code(4);
  SetSx1280Mode(0);  //COMM (mode = 0) or MASTER (1) or SLAVE (2)
  led_code(5);
  delay(1000);
}

//UDFs

void PinInitialize(){
  // Set pins as output or input
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(SX1280LED, OUTPUT);
  pinMode(NSS, OUTPUT);                   //setup NSS pin as output for radio
  pinMode(Flashpin,OUTPUT);               //setup pin as output for flash memory
  pinMode(NRESET, OUTPUT);
  pinMode(RFBUSY, INPUT);                                
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);                 //By default radio is kept low it is listening
  digitalWrite(NRESET,HIGH);
  digitalWrite(SX1280LED, HIGH);
  if(debug){Serial.println("Pin Initialized.");}

}

void FlashInitialize(){
  if(debug){Serial.println("Initializing Flash Memory.");}
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash

  if (flash.initialize())
  {
    if (debug){Serial.println("Flash initilaized !");} 
    
    
  }
  else {
    Serial.print("Flash initialization FAIL, expectedDeviceID(0x");
    Serial.print(expectedDeviceID, HEX);
    Serial.print(") mismatched the read value: 0x");
    Serial.println(flash.readDeviceId(), HEX);
    led_code(8);
    while(true);

  } //8 = Flash Error
  

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
  EEPROM.put(m,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
  flash.chipErase(); // Erage the flash (Must eraase to turn all bits to 1)
  while(flash.busy()){led_code(4);}; //Wait until all erased  Code 4 = Flash busy
  led_code(0);
  
  digitalWrite(Flashpin, HIGH); // Turn OFF Flash
  digitalWrite(NSS, LOW); // Turn ON Radio
  Serial.println("Reset Done");
  
}

void FlashWrite(String sMsg){
    
    sMsg += "\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
       
    if (Last_Address < 16777000){  // W25Q128JV Flash memroy 128Mbits = 16 MB = 16*2^20 = 16777216 Bytes // Rounding down to 1000
      char cMsg[sMsg.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      sMsg.toCharArray(cMsg,sMsg.length());
      //while(digitalRead(RFBUSY)); //Wait while radio is busy
      digitalWrite(NSS, HIGH); // Turnoff Radio
      digitalWrite(Flashpin, LOW); // Turnon Flash
      
      flash.writeBytes(Last_Address, &cMsg,sMsg.length()-1); //sMsg.length() is null Character
      while(flash.busy());
      Last_Address += sMsg.length()-1; 
      EEPROM.put(m, Last_Address);  

      digitalWrite(Flashpin, HIGH); // Turn OFF Flash
      digitalWrite(NSS, LOW); // Turn ON Radio 
      
    }
    else{
      Serial.print("Memory full");
    }
    
}  

void FlashRead(){
  Serial.print("Reading memory from address 0 to address ");
  led_code(4);
  Serial.println(Last_Address);
  //while(digitalRead(RFBUSY)); //Wait while radio is busy
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
   led_code(0);
}



void NanotronReset(){
  //Setup Nanotron radios
  if(debug){Serial.println("Starting Nanotron Reset.");}
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
  
  //Serial2.println("SSET"); //Save to permanent address 
  
  if(debug){Serial.println("Nanotron Reset done.");}
}

void Listen(uint32_t Duration){
  /*
  uint32_t startMS = millis();
  while (millis() < startMS + Duration){
    if(debug){Serial.println("Listening serial port");}
    if(CheckSerial()){
      Serial.println(SerialCode);
      ParseSerial();
      }
    //if(debug){Serial.println("Listening Radio");}
    //ParseCode(CheckRadio());
  }*/
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
      //Serial.flush();
      return false;
    }
    else{
      SerialCode =  SerialMsg.toInt();
      //Serial.flush();
      return true ;
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
  //uint8_t buff[] = "<8,0,6>";                 //the payload to send
  //TXPacketL = LT.transmitReliableAutoACK(buff, sizeof(buff), NetworkID, ACKtimeout, TXtimeout, TXpower, WAIT_TX);
  TXPacketL = LT.transmitReliableAutoACK(u, TXPacketL-1, RadioID, ACKtimeout, TXtimeout, TXpower, WAIT_TX);
  
  if (TXPacketL > 0){ return true;}
  else {return false;}
 
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
  Code 5 = Read Sx1250 registers
  Code 101 = Reset Flash (USE extreme Caution)


  Code 20-73 = BW = 3 factors, SF = 6 factor, Transmission Power = 0, 15, 31 (3 factors) Total 54
  Code 101 = Reset memory
*/
  if(MsgCode == 1 )       {Record = true; Serial.println("Recording ON") ;}
  else if (MsgCode == 2)  {FlashRead();}
  else if (MsgCode == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
  else if (MsgCode == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}
  
  else if(MsgCode >= 20 && MsgCode <=127) 
  {
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
    uint8_t Code2 = ((MsgCode-20)/6)%3;
    switch (Code2)
    {
      case 0: Bandwidth=LORA_BW_0400;break;
      case 1: Bandwidth=LORA_BW_0800;break;
      case 2: Bandwidth=LORA_BW_1600;break;
      default:break;      
    }
    uint8_t Code3 = ((MsgCode-20)/18)%6;
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
    }
 
  else if (MsgCode == 201)
  {
    FlashReset();
    //Request(CommanderID,101);
  }
  else if (MsgCode == 202)   {
    for (RangingAddress=1;RangingAddress<=Stations;RangingAddress++){
    Request(RangingAddress,201);}
    FlashReset();
    RangingAddress = 1; //Start over the ranging
    }

  else if(MsgCode ==255) {Record = false; Serial.println("Recording OFF");} /**/
};
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
void ParseSerial(){
  if(SerialCode == 1 )       {Record = true; Serial.println("Recording ON") ;}
  else if (SerialCode == 2)  {FlashRead();}
  else if (SerialCode == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
  else if (SerialCode == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}
  else if (SerialCode == 201)   {FlashReset();}
  else if (SerialCode == 202)   {
    for (RangingAddress=1;RangingAddress<=Stations;RangingAddress++){
    Request(RangingAddress,201);}
    FlashReset();
    RangingAddress = 1; //Start over the ranging
    }
  else if (SerialCode ==255)     {Record = false; Serial.println("Recording OFF");}
  else{}

}

bool CheckRadio(uint16_t TimeOut){
  // Check if there is message in RXBuffer and if not wait for Timeout 
  // Will retun the Message code if valid message received for this radio ID. Else return 0. 
  // Valid message has this structure <RadioID, MessageCode> Both RadioID and MessageCode are  integers
  SetSx1280Mode(0);
  
  RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, TimeOut, WAIT_RX); //wait for a packet to arrive 
  
  //Is mesage length > 0?
  bool NoErr = (RXPacketL > 0);
  
  //In no error then Check if RSSI > -100 else return false
  if (NoErr){
      NoErr = NoErr and  LT.readPacketRSSI() >= -120;
    }
    else{
    //if (debug){Serial.println("No message");}
    return false;
    } 
  //In no error then check if sentence structure is valid else return false
  
  if (NoErr){
      NoErr = NoErr and  (char)RXBUFFER[0] == '<' and (char)RXBUFFER[RXPacketL-1] == '>';
    }
  else{
    return false;
    } 
  
  if (NoErr)
  {
    String  MsgIn = "";
    for (uint8_t index = 1; index < RXPacketL-1; index++){
      MsgIn += (char)RXBUFFER[index];
    }
    int c1 = MsgIn.indexOf(',');       //Find first comma position
    int c2 = MsgIn.indexOf(',', c1+1); //Find second comma position
    if(debug){Serial.print("Received: ");Serial.println(MsgIn);}
    uint32_t ToID = MsgIn.substring(0, c1).toInt(); //Substring counts from 0. NOTE: Strangely returns character up to position of c1-1. C1 is not included
    
    
    if(ToID == MyID ){
        Sender = MsgIn.substring(c1+1, c2).toInt();
        MsgCode = MsgIn.substring(c2+1).toInt();
        return true;
      }
    else{
        return false; //0 shoud not be included in code
    }     
  }
  else
  {
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
  MsgOut += RangingAddress;
  MsgOut += ",";
  String Rto =  "RATO 0 000000000B0" + String(RangingAddress);
  Serial2.println(Rto); //Returns "=Code,Distance in cm,RSSI \n\r"
  if(debug){Serial.println("Ranging to: " + Rto);}
  delay(d);

  while (Serial2.available() > 0)
  { 
    CharIn = Serial2.read();
    if(debug){Serial.print(CharIn);}
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

//Range to RangingAddress and update MsgOut
bool SxRange() 
{ 
  SetSx1280Mode(1);
  MsgOut ="S,";
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += RangingAddress;
  MsgOut += ",";
  LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);
  IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
  if ( IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
      //digitalWrite(LED1, HIGH);
      
      range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
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
      //Serial.print(MsgOut);
      return true;
    }
  else{
    MsgOut += "-1"; //Ranging not successful write negative distance to indicate invalid result
    MsgOut += Bandwidth;
    MsgOut += ",";
    MsgOut += SpreadingFactor;
    MsgOut += ",";
    MsgOut += RangingTXPower;
    MsgOut += "\r\n";
    //Serial.print(MsgOut);
    return false;
    
  }
  
}


void SxErr() 
{
 
  MsgOut = "S,"; 
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += RangingAddress;
  MsgOut += ",Err,";
  MsgOut += MsgCode;
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
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while(digitalRead(RFBUSY)); //Wait while radio is busy
  if (mode==0){ //Set radio to receive regular
    //Serial.println("Comm mode");
    LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
  }
  else if (mode==1){ //Set as Master    
    //Serial.println("Master mode");
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
  }
  else if (mode==2){ //Set as SLAVE   
    //Serial.println("Slave mode");
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_SLAVE);
  }
  delay(1);
  //while (digitalRead(RFBUSY)){} //Wait until Radio busy
  led_Code2(mode);
}

void printPacketDetails()
{
  /*
  LocalPayloadCRC = LT.CRCCCITT(RXBUFFER, RXPayloadL, 0xFFFF);  //calculate payload crc from the received RXBUFFER
  TransmitterNetworkID = LT.getRXNetworkID(RXPacketL);
  RXPayloadCRC = LT.getRXPayloadCRC(RXPacketL);
 */
}
/*

void ReadSetting(){
  Serial.println("ModemSettings: ");
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println("OperatingSettings: ");
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println("Registers Ox900-Ox9FF");
  LT.printRegisters(0x900, 0x9FF);                       //print contents of device registers, normally 0x900 to 0x9FF
  Serial.println();
  Serial.println();
}

bool Switch2Slave(uint16_t TimeOut){
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, 0, RANGING_SLAVE);
  endwaitmS = millis() + TimeOut;
  while (!LT.receiveRanging(0, 0, RangingTXPower, WAIT_RX) && (millis() <= endwaitmS)){
    if (millis() >= endwaitmS)
      {
        Serial.println("Ranging request Timeout!!");
        return false;// Setup radio to  communication mode
        
      }
    else{
      IrqStatus = LT.readIrqStatus();
      if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
      {
        Serial.print(" Response sent to radio ");
        Serial.print(RangingAddress);
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


bool RecordBroadcast(uint16_t TimeOut){ //RxTimeout give up to TimeOut ms for remote to respond
  // Check if there is message in RXBuffer and if not wait for Timeout 
  // Will retun the Message code if valid message received for this radio ID. Else return 0. 
  // Valid message has this structure <RadioID, MessageCode> Both RadioID and MessageCode are  integers
  uint8_t RXPacketL;                              //stores length of packet received
  uint8_t RXBUFFER_SIZE = 255;                    //RX buffer size
  uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into
                         
  //Read buffer
  LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
  
  RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, TimeOut, WAIT_RX); //wait for a packet to arrive 
  
  //Is mesage length > 0?
  bool NoErr = (RXPacketL > 0);
    
  if (NoErr){
      NoErr = NoErr and  (char)RXBUFFER[0] == '*' and (char)RXBUFFER[RXPacketL-1] == '*'; //Broadcast message has *---* format
    }
  else{
    return false;
    } 
  
  if (NoErr)
  {
    String  MsgIn = "S,";
    for (uint8_t index = 1; index < RXPacketL-1; index++){
      MsgIn += (char)RXBUFFER[index];
    }
    if (Record){ FlashWrite(MsgIn); } 
    Serial.println(MsgIn);
    return true;
  }
  else
  {
    return false;
  } 
  
}


*/