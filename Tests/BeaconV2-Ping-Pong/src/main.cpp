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
#include "Settings.h"


// Flash memory 
#define Flashpin         21				// Flash memory SS pin
uint16_t expectedDeviceID = 0xEF40;
uint32_t Last_Address = 0;
uint8_t m = 6;                  //Address of memory wehre Last_address starts
uint8_t Stations = 3;
uint32_t MyID = 1;
uint32_t CommanderID = 101;

//#define DEBUG
SX128XLT LT;



uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;



//UDF definitions
void PinInitialize();
void Ping(uint32_t ToID, uint8_t Code);
void Pong();
bool Tune();

void NanotronReset();
void Listen(uint32_t Duration);
uint8_t CheckSerial(); //Reads the code sent from serial console
void Request(uint32_t RadioID, uint8_t Code);
void Broadcast(String Msg);
void ParseCode(uint8_t MsgCode);
uint8_t CheckRadio();
void Rmessage();
void Nmessage();
void Smessage();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void ReadSetting();
//Global variables
/// ///////
String MsgOut;
String MsgIn;
uint8_t d = 50; //Standary delay time in ms
bool debug = false;
bool Tweet = false;
bool Record = true;
char CharIn;

uint8_t TXPacketL;
uint32_t TXPacketCount;
uint32_t Sender = 0;
uint32_t Receiver = 0;
uint8_t MsgCode = 0;
uint8_t buff[120] = "<1,2,100>";      //the message to send



void loop()
{
  if (MyID == 1){
    Serial.print(TXpower);                                       //print the transmit power defined
    Serial.print(F("dBm "));
    Serial.print(F("Packet> "));
    Serial.flush();
    Ping(2,100);
   /* 
    endwaitmS = millis() + 1000;

    while (Tune() || (millis() <= endwaitmS));          //wait for Ranging valid or timeout

    if (millis() >= endwaitmS)
    {
      Serial.println("Pong Timeout!!");
      led_Flash(2, 100);                                             //single flash to indicate timeout
    }
    else{ //Entering this look is because Tune() returned true
      Serial.println("Pong Code!!");
      Serial.println(MsgCode);
      //have a delay between packets
    }
    */
  } 
  else{
    Pong();
  }

  delay(1000);
}

void setup()
{
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  //Inilize pins as output or input
  PinInitialize();
  if (LT.begin(NSS, NRESET, RFBUSY, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    while (1);
  }

  LT.setupLoRa(2445000000, 0, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);      //configure frequency and LoRa settings

  Serial.print(F("Transmitter ready"));
  Serial.println();
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
  digitalWrite(NSS,LOW);                 //By default radio is ON
  digitalWrite(NRESET,HIGH);
  if(debug){Serial.println("Pin Initialized.");}

}

void Ping(uint32_t ToID, uint8_t Code){
  /*TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array
  buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on receiver

  LT.printASCIIPacket(buff, TXPacketL);                           //print the buffer (the sent packet) as ASCII
  if (LT.transmitIRQ(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit error
  {
    TXPacketCount++;
    Serial.print(F("  BytesSent,"));
    Serial.print(TXPacketL);                             //print transmitted packet length
    Serial.print(F("  PacketsSent,"));
    Serial.print(TXPacketCount);  
  }
  else
  {
    //if here there was an error transmitting packet
    uint16_t IRQStatus;
    IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
    Serial.print(F(" SendError,"));
    Serial.print(F("Length,"));
    Serial.print(TXPacketL);                             //print transmitted packet length
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);                        //print IRQ status
    LT.printIrqStatus();                                 //prints the text of which IRQs set
  }

  Serial.println();
  */
  String Msg = "<" + String(MyID) + "," + String(ToID)+ "," + String(Code) + ">"; 
  Serial.print(Msg);
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[255]; 
  Msg.toCharArray(buff,TXPacketL);  //buff must be char array fro Msg.toCharArray to work
  uint8_t *u = (uint8_t *) buff;
    // Transmit back
    //TXPacketL = sizeof(buff); 
    delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
    LT.transmitIRQ(u, TXPacketL-1, 500, TXpower, WAIT_TX); //This did not take Char array
    
}

void Pong(){
  endwaitmS = millis() + 1000;

  while (Tune() || (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  if (millis() >= endwaitmS)
  {
    Serial.println("Pong Timeout!!");
    led_Flash(2, 100);                                             //single flash to indicate timeout
  }
  else{ //Entering this look is because Tune() returned true
     buff[1] ='1';
     buff[3] = '2';
     TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array
     buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on receiver

    LT.printASCIIPacket(buff, TXPacketL);                           //print the buffer (the sent packet) as ASCII
    if (LT.transmitIRQ(buff, TXPacketL, 10000, TXpower, WAIT_TX));
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
    LT.transmitIRQ(u, TXPacketL-1, 500, TXpower, WAIT_TX); //This did not take Char array
    
  }
  
  void Broadcast(String Msg){
  
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

 bool Tune(){
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
        NoErr = NoErr and  LT.readPacketRSSI() >= -130;
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
      if (debug){Serial.println("Sender,Receiver");}
      uint32_t To = MsgIn.substring(0, c1).toInt();
      
      
      if(To == MyID ){
          Receiver = To;
          Sender = MsgIn.substring(c1+1, c2).toInt();;
          MsgCode = MsgIn.substring(c1+1).toInt();
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