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
uint32_t MyID = 8;         //must match address in master (Single didgit only for now, Generalize to have any number)
String NanoID = "000000000B0" + String(MyID);  // 9 0s followed by B01 to Bxx

// Flash memory 
#define Flashpin         21				// Flash memory SS pin
uint16_t expectedDeviceID = 0xEF40;
uint32_t Last_Address = 0;
uint8_t m = 6;                  //Address of memory wehre Last_address starts

uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;

uint32_t CommanderID = 101;



SX128XLT LT;
SPIFlash flash(Flashpin, expectedDeviceID);
//UDF
//UDF definitions
void PinInitialize();
void FlashInitialize();
void FlashReset();

void FlashWrite(String Str);
void FlashRead();
void NanotronReset();
void Listen(uint32_t Duration);
uint8_t CheckSerial(); //Reads the code sent from serial console
void Request(uint32_t RadioID, uint8_t Code);
void Broadcast(String Msg);
void ParseCode();
bool CheckRadio();
void Rmessage();
void Nmessage();
void Smessage();
void led_Flash(uint8_t pinID, uint16_t flashes, uint16_t delaymS);
void SetSx1280Mode(bool x);
void EchoCode();
bool Switch2Slave(uint16_t TimeOut);
bool Switch2Master(uint8_t Retries);
//Global variables
/// ///////
String MsgOut;
String MsgIn;
uint8_t d = 50; //Standary delay time in ms
bool debug = true;
bool Tweet = false;
bool Record = false;
uint8_t MsgCode = 0;
uint32_t Sender = 0;
uint32_t response_sent;
char CharIn;
uint8_t RadioMode = 2; //Listen mode by default

void loop()
{
  //while(digitalRead(RFBUSY));
  if (RadioMode == 0) 
  {
    if(Switch2Slave(5000)){ RadioMode = 2;}; //Set as ranging slave for 5 sec
   
  }
  else if (RadioMode == 1)
  { 
    Switch2Master(5); // Set as ranging Master and try ranging for 5 times 

  }
  else if (RadioMode == 2){ //Set for communication 
    //if (debug) { Serial.println("Radio Mode 2"); }
   
    if(CheckRadio()){
      if (debug) { Serial.print("New Code Recieved"); Serial.println(MsgCode);}
      ParseCode();
      RadioMode = 0;
    } 

  }

     
   /*Important observations
    1. LT.receiveRanging(MyID, 0, RangingTXPower, WAIT_RX) returns true ir false on every ranging request on air even if it is not for this radio
    2. Once it response to ranging, LT.receiveRanging must be called before it can response again
    3. If Master calls for repeated ranging to average the reading rapidly, make sure to reset the radio before next call
    for instance, I found out that that led_Flash time has be less than 30ms for Stuart's original ranging code.
    4. Calling LT.receiveRanging with NO_WAIT only works if radios are not changed. It returned false when 


   */

}


void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  PinInitialize();
  FlashInitialize(); //Check if flash is ready
  NanotronReset();   //Reset Nanotron radio 
//Setup Sx1280 radios
  SetSx1280Mode(true); //Set radio to ranging mode
  if(debug){
    Serial.print(F("Calibration,"));
    Serial.println(LT.getSetCalibrationValue());  
  }         //reads the calibratuion value currently set
}

//UDF
/////////////////////////////////////////////////////////////////////////////////////////////
void PinInitialize(){
  // Set pins as output or input
  if(debug){Serial.println("Initializing pins.");}
  pinMode(LED1, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED2, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED3, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED4, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(LED5, OUTPUT);                  //setup pin as output for indicator LED

  pinMode(NSS, OUTPUT);                   //setup NSS pin as output for radio
  pinMode(Flashpin,OUTPUT);               //setup pin as output for flash memory
  pinMode(NRESET, OUTPUT);
  pinMode(RFBUSY, INPUT);                                
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);                 //By default radio is kept low it is listening
  digitalWrite(NRESET,HIGH);
  if(debug){Serial.println("Pin Initialized.");}

}

void FlashInitialize(){
  if(debug){Serial.println("Initializing Flash Memory.");}
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash

  if (flash.initialize())
  {
    if (debug){Serial.println("Flash initilaized !");
    led_Flash(LED1, 100, 2);}//Blink(int DELAY_MS, byte loops)
  }
  else {
    Serial.print("Flash initialization FAIL, expectedDeviceID(0x");
    Serial.print(expectedDeviceID, HEX);
    Serial.print(") mismatched the read value: 0x");
    Serial.println(flash.readDeviceId(), HEX);
  }

 // In order to avoid overwriting flash memory, the last location of memory writtten is saved in EEPROM
 // Memory location is a valid one if EEPROM byte 0 is 170. 
 // I am taking a chance that at start up, EEPROM byte 0 is not 10101010 (=170). This may happen randomly, so to make it fail safe, write EEPROM 0 to value other than 170 and clear flash before using
 // Most of the time this should work 
  if (EEPROM.read(0) != 170){ //This is the first time chances are Byte 0 is not 170
      EEPROM.write(0,0xaa);  //0xaa = 10101010 = 170
      EEPROM.put(m,Last_Address); // EEPROM, starting Byte m, write Last_Address which is 0 when inialized. It will take 4 bytes as Last_address is Unsigend Long Integer
      flash.chipErase(); //Erage the flash
      while(flash.busy()); //Wait until all erased
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
  //flash.chipErase(); //Erage the flash
  while(flash.busy()); //Wait until all erased
  
  digitalWrite(Flashpin, HIGH); // Turn OFF Flash
  digitalWrite(NSS, LOW); // Turn ON Radio
  Serial.println("Reset Done");
  
}

void FlashWrite(String sMsg){
    
    sMsg += "\n\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
       
    if (Last_Address < 16777000){  // W25Q128JV Flash memroy 128Mbits = 16 MB = 16*2^6 = 16777216 Bytes // Rounding down to 1000
      char cMsg[sMsg.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      sMsg.toCharArray(cMsg,sMsg.length());
      while(digitalRead(RFBUSY)); //Wait while radio is busy
      digitalWrite(NSS, HIGH); // Turnoff Radio
      digitalWrite(Flashpin, LOW); // Turnon Flash
      
      flash.writeBytes(Last_Address, &cMsg,sMsg.length()-1);
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
  
  Serial.println(Last_Address);
  while(digitalRead(RFBUSY)); //Wait while radio is busy
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
  if(debug){Serial.println("Nanotron Reset done.");}
}

void Listen(uint32_t Duration){
  uint32_t startMS = millis();
  while (millis() < startMS + Duration){
    if(debug){Serial.println("Listening serial port");}
    if (CheckSerial()){ParseCode();}
    if(debug){Serial.println("Listening Radio");}
    if (CheckRadio()){ParseCode();}
  }
}

// This function returns the command within a pair of () sent from serial port
uint8_t CheckSerial() { //Return the command in pair of ()
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
      MsgCode =  SerialMsg.toInt();
      return true ;
    }
    
}

void Request(uint32_t RadioID, uint8_t Code){
  String Msg = "<" + String(RadioID) + "," + String(Code)+">";
  Serial.print(Msg);
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
 uint8_t *u = (uint8_t *) buff;
    // Transmit back
    //TXPacketL = sizeof(buff); 
    delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
    LT.transmitIRQ(u, TXPacketL-1, 500, RangingTXPower, WAIT_TX); //This did not take Char array
   
 }
 void Broadcast(String Msg){
  while(flash.busy()); //Wait until flash is busy
  while(digitalRead(RFBUSY)); //Wait while radio is busy
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while(digitalRead(RFBUSY)); //Wait while radio is busy

  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
  uint8_t *u = (uint8_t *) buff;
  LT.transmitIRQ(u, TXPacketL-1, 500, RangingTXPower, WAIT_TX); //This did not take Char array
  while(digitalRead(RFBUSY)); //Wait while radio is busy
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
  Code 101 = Reset Flash (USE extreme Caution)


  Code 20-73 = BW = 3 factors, SF = 6 factor, Transmission Power = 0, 15, 31 (3 factors) Total 54
  Code 101 = Reset memory
  */
  if(MsgCode == 1 )       {Record = true; EchoCode(); } //Since this is slave, echoing is better
  else if (MsgCode == 2)  {FlashRead();  EchoCode();  }

  else if(MsgCode >= 20 && MsgCode <=73) 
  {
    
    //EchoCode();
    //RadioMode = 0; //Go to ranging slave
    
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
    uint8_t Code3 = ((MsgCode-20)/18)%3;
    switch (Code3)
    {
      case 0: RangingTXPower=0;break;
      case 1: RangingTXPower=15;break;
      case 2: RangingTXPower=31;break;
      default:break;      
    }
    
  }
  else if (MsgCode == 101)
  {
    EchoCode();
    FlashReset();
    Request(CommanderID,101);
  }
  
  else if(MsgCode ==255) {Record = false;}
 };
 
 bool CheckRadio(){
   // Check if there is message in RXBuffer and if not wait for Timeout 
   // Will retun the Message code if valid message received for this radio ID. Else return 0. 
   // Valid message has this structure <RadioID, MessageCode> Both RadioID and MessageCode are  integers
    uint8_t RXPacketL;                              //stores length of packet received
    uint8_t RXBUFFER_SIZE = 255;                    //RX buffer size
    uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into
    uint16_t TimeOut = 60000;                           //RxTimeout
   //Read buffer
    LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
    
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
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += millis(); 
  MsgOut += "\n\r";
  if(debug){Serial.println("R-message done.");}
}

void Nmessage(){
  //Create Line 2 = "N,0,range in cm, RSSI,Temperature, Battery"
  if(debug){Serial.println("Starting N-message.");}
  MsgOut += "N,"; 
  Serial2.println("RATO 0 000000000B0" + String(MyID)); //Returns "=Code,Distance in cm,RSSI \n\r"
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
  if(debug){Serial.println("N-message done.");}
}

void Smessage()
{
  if(debug){Serial.println("Starting S-message.");}
  MsgOut += "S,"; 
  while(flash.busy()); //Wait until flash is busy
  if(debug){Serial.println("Flash not busy.");}
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while (digitalRead(RFBUSY)) //Wait until Radio busy
  if(debug){Serial.println("RF not busy.");}
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, MyID, RANGING_MASTER);
  delay(1);
  while (digitalRead(RFBUSY)){} //Wait until Radio busy
  if(debug){Serial.println("RF not busy.");}
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
      MsgOut += ",";
      MsgOut += Calibration;
      MsgOut += "\r\n";
    }
  else{
    MsgOut += "-1"; //Ranging not successful write negative distance to indicate invalid result
    MsgOut += "\r\n";
    
  }
  if(debug){Serial.println("S-message done.");}
}

//Sets Sx1280 radio for ranging (x = 0) or communication (1)
void SetSx1280Mode(bool Range){
  while(flash.busy()); //Wait until flash is busy
  if(debug){Serial.println("Flash not busy.");}
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);

  if (Range){
    LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
    delay(1);
    while (digitalRead(RFBUSY)){} //Wait until Radio busy
    //The function call list below shows the complete setup for the LoRa device for ranging using the information
    //defined in the Settings.h file.
    //The 'Setup LoRa device for Ranging' list below can be replaced with a single function call, note that
    //the calibration value will be loaded automatically from the table in the library;
    //LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, MyID, RangingRole);

    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, MyID, RANGING_SLAVE);

    //***************************************************************************************************
    //Setup LoRa device for Ranging Slave
    //***************************************************************************************************
    /*
      LT.setMode(MODE_STDBY_RC);
      LT.setPacketType(PACKET_TYPE_RANGING);
      LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
      LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
      LT.setRfFrequency(Frequency, Offset);
      LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
      LT.setRangingMasterAddress(MyID);
      LT.setRangingSlaveAddress(MyID);
      LT.setRangingCalibration(LT.lookupCalibrationValue(SpreadingFactor, Bandwidth));
      LT.setRangingRole(RANGING_SLAVE);
      LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
      LT.setHighSensitivity();
    */
    //***************************************************************************************************
    delay(1);
    while (digitalRead(RFBUSY)){} //Wait until Radio busy

  }
  else{ //Set radio to receive regular
    while(flash.busy()); //Wait until flash is busy
    while(digitalRead(RFBUSY)); //Wait while radio is busy
    digitalWrite(Flashpin,HIGH);
    digitalWrite(NSS,LOW);
    LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
    delay(1);
    while(digitalRead(RFBUSY)); //Wait while radio is busy
  }
}

void led_Flash(uint8_t pinID, uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(pinID, HIGH);
    delay(delaymS);
    digitalWrite(pinID, LOW);
    delay(delaymS);
  }
}



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

 bool Switch2Slave(uint16_t TimeOut){
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, MyID, RANGING_SLAVE);
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
        Serial.println(" Response sent");
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
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
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
