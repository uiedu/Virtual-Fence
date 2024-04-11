/*******************************************************************************************************
  Programs for Arduino to setup Beacon V2 Virtual Fence (Red Boards) with both radios
  Program uses base code from Stuwart project for Sx1280
  Modified by Dev Shrestha 04/06/2024
  1. Serial monitor baud rate is set at 115200
  2. Set up the Nanotron radio V3 from Inpixion
  3. Set up Sx1280 radio from Semtech
  4. Use Settings.h to change any settings local to a radio
  
*******************************************************************************************************/



#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

uint32_t endwaitmS;
uint16_t IrqStatus;
uint32_t response_sent;




void led_Flash(unsigned int flashes, unsigned int delaymS);

//UDF
uint8_t GetMsgCode(uint32_t MyID);
void EchoCode(uint8_t Code);
void ParseCode(uint8_t MsgCode);
uint8_t MsgCode = 0;
void loop()
{
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_SLAVE);
  LT.ResetIRQ();
  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate); 
  MsgCode = GetMsgCode(RangingAddress);
  if(MsgCode > 0){
    Echo(MsgCode);
    ParseCode(MsgCode);
  }
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show board is alive
  unsigned int index;

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
  /* 
  Setting up the Radio Beacons (Use BeaconV2-Slave to configure)
  1. Nanotron radios are connected to Mega UART02 (BAUD 115200). Set each of the beacons to
     SFAC   //Factory Reset
     BRAR 0 // Do not broadcast ranging results (Ranging is performed by Beacon)
     SNID 000000000B01 //to 000000000B06
     EBID 0 //Do not blink ID
     GPIO 0 0 0 0 2   //SET GPIO 0 as input 
     GPIO 1 0 0 0 2   //SET GPIO 1 as input 
     GPIO 2 0 0 0 2   //SET GPIO 2 as input 
     GPIO 3 0 0 0 2   //SET GPIO 3 as input 
     SSET   //Save settings
  2. Sx1280 radios are connected through SPI communication (Slave select (NSS) Pin 10)   

*/
  int d = 50; //ms delay between commands
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  
  //Setup Nanotron radios
  Serial.print("Setting Nanotron Radio to radio ID: "); 
  Serial.println(radioID); 
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
  
  Serial.println("SNID " + radioID);
  Serial2.println("SNID " + radioID);
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
  
//Setup Sx1280 radios
  Serial.print("Setting Sx1280 Radio to radio ID: " );
  Serial.println(RangingAddress); 


  /*  Commenting non-essential codes
      Serial.println();
      Serial.println("55_Ranging_Slave Starting");

      pinMode(LED1, OUTPUT);
      led_Flash(2, 125);
  */
  

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    //Serial.println(F("Device found"));
    //led_Flash(2, 125);
    //delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device for ranging using the information
  //defined in the Settings.h file.
  //The 'Setup LoRa device for Ranging' list below can be replaced with a single function call, note that
  //the calibration value will be loaded automatically from the table in the library;
  //LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RangingRole);

  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_SLAVE);

  //***************************************************************************************************
  //Setup LoRa device for Ranging Slave
  //***************************************************************************************************
  /*
    LT.setMode(MODE_STDBY_RC);
    LT.setPacketType(PACKET_TYPE_RANGING);
    LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
    LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
    LT.setRfFrequency(Frequency, Offset);
    LT.setTxParams(TXpower, RADIO_RAMP_02_US);
    LT.setRangingMasterAddress(RangingAddress);
    LT.setRangingSlaveAddress(RangingAddress);
    LT.setRangingCalibration(LT.lookupCalibrationValue(SpreadingFactor, Bandwidth));
    LT.setRangingRole(RANGING_SLAVE);
    LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
    LT.setHighSensitivity();
  */
  //***************************************************************************************************
  //Calibration = LT.GetCalibration(Bandwidth,SpreadingFactor);
  LT.setRangingCalibration(Calibration);               //override automatic lookup of calibration value from library table

  Serial.print(F("Calibration,"));
  Serial.println(LT.getSetCalibrationValue());           //reads the calibratuion value currently set
  delay(2000);
}

//UDF
/////////////////////////////////////////////////////////////////////////////////////////////
 uint8_t GetMsgCode(uint32_t MyID){
   
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

void EchoCode(uint8_t Code){
  String Msg = "<0," + String(Code)+">";
  uint8_t  TXPacketL = Msg.length() + 1;
  char buff[TXPacketL];
    Msg.toCharArray(buff,TXPacketL);
    // Transmit back
    //TXPacketL = sizeof(buff); 
    delay(100); //Delay Needed for the Beacon as it has just sent a message and may not be ready to receive.
    LT.transmitIRQ(buff, TXPacketL-1, 500, TXpower, WAIT_TX)
   
 }