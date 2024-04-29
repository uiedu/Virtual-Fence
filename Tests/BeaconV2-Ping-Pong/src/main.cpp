/*****************************************************************************************************
  This Example is a minimum code using Sx1280 to
  1. Communicates between master and Salve to get ack from slave
  2. Master ranges to Slave
  3. Slave Rages to master
  This code Combines
  209_Reliable_Transmitter_AutoACK
  210_Reliable_Receiver_AutoACK
  for reliable response from slave 
  by author Stuart Robinson - 16/03/20
  Modified by Dev Shrestha - 04/22/24
  Serial monitor baud rate should be set at 115200.
*******************************************************************************************************/



#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"


// Flash memory 
#define Flashpin         21				// Flash memory SS pin


//#define DEBUG
SX128XLT LT;

//UDF definitions
void PinInitialize();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void printPacketDetails();
void SetSx1280Mode(uint8_t mode);
bool Range();
bool RangeMe(uint16_t TimeOut);

//Ranging Variables
uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;
String MsgOut;

bool Ping = false;
bool Success = false;
// Ping Variables
uint8_t buff[] = "Hello World";                 //the payload to send
uint8_t TXPacketL;
#define ACKtimeout 1000                         //Acknowledge timeout in mS                      
#define TXtimeout 0                          //transmit timeout in mS. If 0 return from transmit function after send.  
 
const uint16_t NetworkID = 0x3210;              //NetworkID identifies this connection, needs to match value in receiver
//Pong Variables
#define ACKdelay 100                            //delay in mS before sending acknowledge                    
#define RXtimeout 5000                          //receive timeout in mS.   
const uint8_t RXBUFFER_SIZE = 251;              //RX buffer size, set to max payload length of 251, or maximum expected length
uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
uint8_t RXPayloadL;                             //stores length of payload received
uint8_t PacketOK;                               //set to > 0 if packetOK
int16_t PacketRSSI;                             //stores RSSI of received packet
uint16_t LocalPayloadCRC;                       //locally calculated CRC of payload
uint16_t RXPayloadCRC;                          //CRC of payload received in packet
uint16_t TransmitterNetworkID;                  //the NetworkID from the transmitted and received packet




void loop()
{
  SetSx1280Mode(0);
  if (Ping)
  {
    TXPacketL = LT.transmitReliableAutoACK(buff, sizeof(buff), NetworkID, ACKtimeout, TXtimeout, TXpower, WAIT_TX);
    if (TXPacketL > 0)
    {
      //if transmitReliableAutoACK() returns > 0 then transmit and ack was OK
      Range();
      RangeMe(1000);

              
    }
    else
    {
      //if transmitReliableAutoACK() returns 0 there was an error, timeout etc
      Serial.print(F("No Packet acknowledge"));
      LT.printIrqStatus();                                 //prints the text of which IRQs set
      LT.printReliableStatus(); 
      Serial.println();                           //print the reliable status
    }
      
    
  }
  else{ //This is Pong
    
    PacketOK = LT.receiveReliableAutoACK(RXBUFFER, RXBUFFER_SIZE, NetworkID, ACKdelay, TXpower, RXtimeout, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout
    
    if (PacketOK > 0)
    {
      //if the LT.receiveReliable() returns a value > 0 for PacketOK then packet was received OK
      RangeMe(1000);
      //delay(100);
      Range();
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
        printPacketDetails();
      }
    }

    Serial.println();
  }
  delay(1000);
}
void setup()
{
  Serial.begin(115200);
  Serial.println();
  if (Ping){
    Serial.println(F("209_Reliable_Transmitter_AutoACK Starting"));
  }
  else{
    Serial.println(F("209_Reliable_Receiver_AutoACK Starting"));
  }
  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    while (1);
  }

  //LT.setupLoRa(2445000000, 0, LORA_SF5, LORA_BW_1600, LORA_CR_4_5);
  LT.setupLoRa(2445000000, 0, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
  Serial.println(F("Ready"));
  Serial.println();
}
//UDFs

void PinInitialize(){
  // Set pins as output or input
  
  pinMode(LED1, OUTPUT);                  //setup pin as output for indicator LED
  pinMode(NSS, OUTPUT);                   //setup NSS pin as output for radio
  pinMode(Flashpin,OUTPUT);               //setup pin as output for flash memory
  pinMode(NRESET, OUTPUT);
  pinMode(RFBUSY, INPUT);                                
  digitalWrite(Flashpin,HIGH);
  digitalWrite(NSS,LOW);                 //By default radio is ON
  digitalWrite(NRESET,HIGH);
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

void printPacketDetails()
{
  LocalPayloadCRC = LT.CRCCCITT(RXBUFFER, RXPayloadL, 0xFFFF);  //calculate payload crc from the received RXBUFFER
  TransmitterNetworkID = LT.getRXNetworkID(RXPacketL);
  RXPayloadCRC = LT.getRXPayloadCRC(RXPacketL);

  Serial.print(F("LocalNetworkID,0x"));
  Serial.print(NetworkID, HEX);
  Serial.print(F(",TransmitterNetworkID,0x"));
  Serial.print(TransmitterNetworkID, HEX);
  Serial.print(F(",LocalPayloadCRC,0x"));
  Serial.print(LocalPayloadCRC, HEX);
  Serial.print(F(",RXPayloadCRC,0x"));
  Serial.print(RXPayloadCRC, HEX);
  LT.printReliableStatus();
}

//Sets Sx1280 radio for COMM (mode = 0) or MASTER (1) or SLAVE (2)
void SetSx1280Mode(uint8_t mode){
  
  LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE);
  delay(1);
  while(digitalRead(RFBUSY)); //Wait while radio is busy
  if (mode==0){ //Set radio to receive regular
    //Serial.println("Comm mode");
    LT.setupLoRa(Frequency, Offset, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);
  }
  else if (mode==1){ //Set as Master    
    //Serial.println("Master mode");
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, NetworkID, RANGING_MASTER);
  }
  else if (mode==2){ //Set as SLAVE   
    //Serial.println("Slave mode");
    LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, NetworkID, RANGING_SLAVE);
  }
  delay(1);
  while (digitalRead(RFBUSY)){} //Wait until Radio busy
  
}

bool Range(){
  SetSx1280Mode(1);
  MsgOut ="S,";
  LT.transmitRanging(NetworkID, TXtimeoutmS, RangingTXPower, WAIT_TX);
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
      MsgOut += ",";
      MsgOut += Calibration;
      MsgOut += "\r\n";/**/
      Serial.print(MsgOut);
      return true;
    }
  else{
    MsgOut += "-1"; //Ranging not successful write negative distance to indicate invalid result
    MsgOut += "\r\n";
    Serial.print(MsgOut);
    return false;
    
  }
  }

bool RangeMe(){
  SetSx1280Mode(2);

  if(LT.receiveRanging(NetworkID, 0, RangingTXPower, WAIT_RX)){ //If WAIT_RX, the radio is wait until ranging response is sent (No Time Limit)
  //Serial.println("Response sent");
  return true; 
  }
  else{
    //Serial.println("No Request");
    return false; 
  }
} 