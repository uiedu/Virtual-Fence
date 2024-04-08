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

void loop()
{
  LT.receiveRanging(RangingAddress, 0, TXpower, NO_WAIT);

  endwaitmS = millis() + rangingRXTimeoutmS;

  while (!digitalRead(DIO1) && (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  if (millis() >= endwaitmS)
  {
    Serial.println("Error - Ranging Receive Timeout!!");
    led_Flash(2, 100);                                             //single flash to indicate timeout
  }
  else
  {
    IrqStatus = LT.readIrqStatus();
    digitalWrite(LED1, HIGH);

    if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
    {
      response_sent++;
      Serial.print(response_sent);
      Serial.print(" Response sent");
    }
    else
    {
      Serial.print("Slave error,");
      Serial.print(",Irq,");
      Serial.print(IrqStatus, HEX);
      LT.printIrqStatus();
    }
    digitalWrite(LED1, LOW);
    Serial.println();
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
