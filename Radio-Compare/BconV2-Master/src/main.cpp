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


// Flash memory 
#define Flashpin         21				// Flash memory SS pin
uint16_t expectedDeviceID = 0xEF40;
uint32_t Last_Address = 0;
uint8_t m = 6;                  //Address of memory wehre Last_address starts
uint8_t Stations = 3;
uint32_t MyID = 0;
uint32_t CommanderID = 101;

//#define DEBUG
SX128XLT LT;
SPIFlash flash(Flashpin, expectedDeviceID);


uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;



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
void ParseCode(uint8_t MsgCode);
bool CheckRadio(uint16_t TimeOut);
void Rmessage();
void Nmessage();
void Smessage();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void ReadSetting();
bool Switch2Slave(uint16_t TimeOut);
bool RecordBroadcast(uint16_t TimeOut);
//Global variables
/// ///////
String MsgOut;
String MsgIn;
uint8_t d = 50; //Standary delay time in ms
bool debug = false;
bool Tweet = false;
bool Record = false;
char CharIn;
uint8_t MsgCode = 0;
uint32_t Sender = 0;
uint32_t response_sent;
uint8_t RadioMode = 2; //Listen mode by default



void loop()
{
  
  // 1. Range to each radio in sequence from 1 to Stations 
  //for(RangingAddress = 1; RangingAddress <= Stations; RangingAddress++){
  //if (RangingAddress++ >=5){RangingAddress=1;}
  
  MsgOut="";
  Rmessage();//Adds to MsgOut = "R,RangingAddress,millis()"
  if (Record){ FlashWrite(MsgOut); } 
  if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
  delay(1000);
  MsgOut="";
  Nmessage();//Range using Nanotron
  if (Record){ FlashWrite(MsgOut); } 
  if (Tweet){Broadcast(MsgOut);} else{Serial.print(MsgOut);}
  delay(1000);
  
  MsgOut="";
  Smessage();
  if (Tweet){Broadcast(MsgOut);} else{Serial.println(MsgOut);}
  /*
    Ranging to SX1280 has follwoing sequence
      1. Master sends Setting requst with a code to radio it wants to range
      2. Slave receives the code, and confirms, then set its radio as slave and wait for 5 seconds to get ranging request
          If none received, slave will go to regular cummunication mode
      3. If confirmation code matches, Master changes its own settings and send range request
         Tries 5 times if ranging result is invalid
         Moves on to next radio 
      4. If successful ranging received, it writes to memroty and switch to slave mode
      5. Slave and Master switches role and ranges again
         Remore radio tries to range up to 5 times if ranging is unsuccessful. then switches to comm mode.
         If ranging successulf Slve Broadcast the result
      5. After successful ranging, Master switches to comm mode and listens to broadcast message
      6  Records the boradcast message
      7. Repeats 1 - 6 with different code for same radio
      8. Once all settings are done, switches to next radio      
  */
  for(uint8_t Code=20;Code <=73;Code++){
    if (debug)
    {
          Serial.print("Romote radio ");
          Serial.print(RangingAddress);
          Serial.print(" Sending code: ");
          Serial.println(Code); 
    }
    
   
    Request(RangingAddress,Code);
    ParseCode(Code);
    
    MsgOut="";
    Smessage();
    if (Tweet){Broadcast(MsgOut);} else{Serial.println(MsgOut);}


    /*


    
    endwaitmS = millis() + 5000; //Try up to 5 seconds to get confirmation
    while (!CheckRadio(2000) && (millis() <= endwaitmS)){
      if (millis() >= endwaitmS)
        {
          Serial.print("Romote radio ");
          Serial.print(RangingAddress);
          Serial.print(" failed to confirm code: ");
          Serial.println(Code);                                                
        }
      else {  
      Request(RangingAddress,Code); //Try again
      delay(100);
      }
    }
    
    if (Sender == RangingAddress){
      //ParseCode(Code);
      Smessage();// Range Using SX1280
      if (Record){FlashWrite(MsgOut); } 
      if (Tweet){Broadcast(MsgOut);} else{Serial.println(MsgOut);}
      //Switch2Slave(5000);
      //RecordBroadcast(500); //Argument is timeout
    }*/
    delay(2000); //Small delay before next setting
  }
  //Listen(5000);
  delay(2000);
    
     
}

void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  //Inilize pins as output or input
  PinInitialize();
  
  //FlashInitialize(); //Check if flash is ready
  NanotronReset();   //Reset Nanotron radio 
}

//UDFs

void PinInitialize(){
  // Set pins as output or input
  if(debug){Serial.println("Initializing pins.");}
  pinMode(LED1, OUTPUT);                  //setup pin as output for indicator LED
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
    led_Flash(100, 2);}//Blink(int DELAY_MS, byte loops)
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
  while(digitalRead(RFBUSY));//Wait while radio is busy
  digitalWrite(NSS, HIGH); // Turnoff Radio
  digitalWrite(Flashpin, LOW); // Turnon Flash
  Last_Address = 0; 
  EEPROM.put(m,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
  flash.chipErase(); //Erage the flash
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
   //delay(10000);
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
  if(debug){Serial.println("Nanotron Reset done.");}
}

void Listen(uint32_t Duration){
  uint32_t startMS = millis();
  while (millis() < startMS + Duration){
    if(debug){Serial.println("Listening serial port");}
    ParseCode(CheckSerial());
    //if(debug){Serial.println("Listening Radio");}
    //ParseCode(CheckRadio());
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
    String SerialMsg ="0";
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
            SerialMsg = "0"; //Start marker found get ready to read
            recvInProgress = true;
        }
    }
    return SerialMsg.toInt();
    
}

void Request(uint32_t RadioID, uint8_t Code){
  
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);
  LT.begin(NSS, NRESET, RFBUSY, LORA_DEVICE);
  delay(10);
  LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
  String Msg = "<" + String(RadioID) + "," + String(MyID)+ "," + String(Code)+">";
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
  Msg.toCharArray(buff,TXPacketL);
  uint8_t *u = (uint8_t *) buff;
  //delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
  LT.transmitIRQ(u, TXPacketL-1, 500, TXpower, WAIT_TX); //This did not take Char array
  //Wait for code confirmation
  if(debug){
    Serial.print("Sending Request: ");
    Serial.println(Msg);
  } 
  while(digitalRead(RFBUSY));  
}
  
void Broadcast(String Msg){
while(flash.busy()); //Wait until flash is busy
Serial.println("Flash Not Busy");
//while(digitalRead(RFBUSY)); //Wait while radio is busy
digitalWrite(Flashpin,HIGH);
digitalWrite(NSS,LOW);
if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE)){ Serial.println("Radio ready for trasmission");};
delay(1);
//while(digitalRead(RFBUSY)); //Wait while radio is busy

uint8_t  TXPacketL = Msg.length() + 1;
char buff[TXPacketL];
Msg.toCharArray(buff,TXPacketL);
uint8_t *u = (uint8_t *) buff;
LT.transmitIRQ(u, TXPacketL-1, 500, RangingTXPower, WAIT_TX); //This did not take Char array
Serial.println("Broadcast complete.");
//while(digitalRead(RFBUSY)); //Wait while radio is busy
// Since radio is default mode do not turn off radio
}

void ParseCode(uint8_t MsgCode){
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
else if (MsgCode == 2)  {FlashRead();  Serial.println("Reading Falsh");  }
else if (MsgCode == 3)  {Tweet = true; Serial.println("Broadcasting ON");  }
else if (MsgCode == 4)  {Tweet = false;  Serial.println("Broadcasting OFF");}

else if(MsgCode >= 20 && MsgCode <=73) 
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
  FlashReset();
  Request(CommanderID,101);
}

else if(MsgCode ==255) {Record = false; Serial.println("Recording OFF");}
};


bool CheckRadio(uint16_t TimeOut){
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
  MsgOut += MyID;
  MsgOut += ",";
  MsgOut += RangingAddress;
  MsgOut += ",";
  MsgOut += millis(); 
  MsgOut += "\n\r";
  if(debug){Serial.println("R-message done.");}
}

void Nmessage(){
  //Create Line 2 = "N,0,range in cm, RSSI,Temperature, Battery"
  if(debug){Serial.println("Starting N-message.");}
  MsgOut += "N,";
  MsgOut += MyID;
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
  if(debug){Serial.println("N-message done.");}
}


void Smessage()
{
  if(debug){Serial.println("Starting S-message.");}
  MsgOut += "S,"; 
  MsgOut += MyID;
  MsgOut += ",";
  while(flash.busy()); //Wait until flash is busy
  //if(debug){Serial.println("Flash not busy.");}
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);
  if(debug){Serial.println("Beginning Radio.");}
  
  delay(1);
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while (digitalRead(RFBUSY)) //Wait until Radio busy
  if(debug){Serial.println("Setting up radio as ranging master.");}
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
  delay(1);
  while (digitalRead(RFBUSY)){} //Wait until Radio busy
  
  if(debug){Serial.println("Transmitting ranging request.");}
  LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);
  IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
  if ( IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
      //digitalWrite(LED1, HIGH);
      if(debug){Serial.println("Valid Ranging received.");}
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
    if(debug){Serial.println("No valid ranging received.");}
    MsgOut += "-1"; //Ranging not successful write negative distance to indicate invalid result
    MsgOut += "\r\n";
    
  }
  if(debug){Serial.println("S-message done.");}
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

