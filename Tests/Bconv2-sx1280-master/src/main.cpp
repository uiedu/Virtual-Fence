

/*****************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/03/20
  Modified by Dev Shrestha - 04/05/24
  1. Disabled Display 1.3" OLED by commenting out four places search for Comment 1 through Comment 4 for details
  2. Changed Baud rate to 115200
  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

// Add Flash
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <EEPROM.h>
#define Flashpin         21				// Flash memory SS pin
uint16_t expectedDeviceID = 0xEF40;
uint32_t Last_Address = 0;
SPIFlash flash(Flashpin, expectedDeviceID);

//Radio Variables
#define RXBUFFER_SIZE 255                       //RX buffer size
uint32_t RXpacketCount;
uint8_t RXBUFFER[RXBUFFER_SIZE]; 
uint8_t RXPacketL;                              //stores length of packet received         


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

//UDF definitions
void FlashWrite(String Str);
void FlashRead();
void FlashReset();
String MsgIn;
String ReadCommand(uint8_t Source);
void Execute(String Msg);

void loop()
{
  uint8_t index;
  distance_sum = 0;
  range_result_sum = 0;
  rangeing_results = 0;                           //count of valid results in each loop

  for (index = 1; index <= rangeingcount; index++)
  {

    startrangingmS = millis();

    Serial.println(F("Start Ranging"));

    LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);

    IrqStatus = LT.readIrqStatus();

    if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
    {
      rangeing_results++;
      rangeings_valid++;
      digitalWrite(LED1, HIGH);
      Serial.print(F("Valid"));
      range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
      Serial.print(F(",Register,"));
      Serial.print(range_result);

      if (range_result > 800000)
      {
        range_result = 0;
      }
      range_result_sum = range_result_sum + range_result;

      distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment);
      distance_sum = distance_sum + distance;

      Serial.print(F(",Distance,"));
      Serial.print(distance, 1);
      Serial.print(F(",RSSIReg,"));
      Serial.print(LT.readRegister(REG_RANGING_RSSI));
      RangingRSSI = LT.getRangingRSSI();
      Serial.print(F(",RSSI,"));
      Serial.print(RangingRSSI);
      Serial.print(F("dBm"));
      digitalWrite(LED1, LOW);
    }
    else
    {
      rangeing_errors++;
      distance = 0;
      range_result = 0;
      Serial.print(F("NotValid"));
      Serial.print(F(",Irq,"));
      Serial.print(IrqStatus, HEX);
    }
    delay(packet_delaymS);

    if (index == rangeingcount)
    {
      range_result_average = (range_result_sum / rangeing_results);

      if (rangeing_results == 0)
      {
        distance_average = 0;
      }
      else
      {
        distance_average = (distance_sum / rangeing_results);
      }

      Serial.print(F(",TotalValid,"));
      Serial.print(rangeings_valid);
      Serial.print(F(",TotalErrors,"));
      Serial.print(rangeing_errors);
      Serial.print(F(",AverageRAWResult,"));
      Serial.print(range_result_average);
      Serial.print(F(",AverageDistance,"));
      Serial.print(distance_average, 1);
      MsgIn = ReadCommand(0);
      Execute(MsgIn);
      MsgIn = ReadCommand(1);
      Execute(MsgIn);
      /*
      FlashWrite(String(distance_average));
      
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
      */

      delay(2000);

    }
    Serial.println();
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
  Serial.begin(115200);   //Original 9600
  
  pinMode(LED1, OUTPUT);                                      //setup pin as output for indicator LED
  pinMode(NSS, OUTPUT);                                       ////setup NSS pin as output for radio
  pinMode(Flashpin,OUTPUT);
                                    //setup pin as output for indicator LED
  
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
 
  
  Serial.println();
  Serial.println(F("54_Ranging_Master Starting"));

  //SPI.begin();

  led_Flash(2, 125);

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("Device found"));
    led_Flash(2, 125);
    delay(1000);
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

  //LT.setRangingCalibration(Calibration);               //override automatic lookup of calibration value from library table

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x900, 0x9FF);                       //print contents of device registers, normally 0x900 to 0x9FF
  Serial.println();
  Serial.println();

/* Comment 4
#ifdef ENABLEDISPLAY
  Serial.println("Display Enabled");
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.setCursor(0, 0);
  disp.print(F("Ranging RAW Ready"));
  disp.setCursor(0, 1);
  disp.print(F("Power "));
  disp.print(RangingTXPower);
  disp.print(F("dBm"));
  disp.setCursor(0, 2);
  disp.print(F("Cal "));
  disp.print(Calibration);
  disp.setCursor(0, 3);
  disp.print(F("Adjust "));
  disp.print(distance_adjustment, 4);
#endif
*/
  Serial.print(F("Address "));
  Serial.println(RangingAddress);
  Serial.print(F("CalibrationValue "));
  Serial.println(LT.getSetCalibrationValue());
  Serial.println(F("Ranging master RAW ready"));

  delay(2000);
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

void FlashReset(){
  Serial.print("Resetting Flash: ");
  Last_Address = 0; 
  EEPROM.put(1,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
  flash.chipErase(); //Erage the flash
  while(flash.busy()); //Wait until all erased
  Serial.println();
  Serial.println("Reset Done");
}


// This function returns the command within a pair of () sent from serial port or Sx1280 radio
String ReadCommand(uint8_t Source) { //Return the command in pair of () from serial if Source = 0, from radio if Source = 1
  // This is a better code to ensure commnd from serial is read properly 
  // All command starts with ( and end with )
  static bool recvInProgress = false; //Static is necessary not to terminte reading at the middle of the line
  char startMarker = '(';
  char endMarker = ')';
  char rc;
  String SerialMsg ="";
  bool EndCommand = false;
  //See if serial message is available, newCommand is false at this point
  if (Source == 0) { //Source 0 is serial command
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
  }
  else if (Source == 1) //Listen to radio
  {
    //Assumes that radio is in listen mode
    RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, 10, WAIT_RX); //wait for a packet to arrive with 1seconds (10mS) timeout
    uint8_t i =0;
    while (i < RXPacketL && EndCommand == false) {
      rc = RXBUFFER[i++];
      
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
  
  }
  
 return SerialMsg;   
}

void Execute(String Msg){
  if (Msg == ""){return;}
  
  if(Msg=="reset") 
  {
    FlashReset();
  }
  else if (Msg=="read")
  {
    FlashRead();
  }
}