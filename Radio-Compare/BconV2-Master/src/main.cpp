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
SX128XLT LT;

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


void loop()
{
/* Steps to range and record
  1. Range and record from Nanotron
  2. Range and record from Sx1280 with settings 1..53
  3. Go to 1  

*/

// 1. Range using Nanotron
   int d = 10;
   Serial.println("RATO 0 000000000B01");
   Serial2.println("RATO 0 000000000B01");
   delay(d);
   while (Serial2.available() > 0){ 
     Serial.write(Serial2.read());
   }
    delay(1000);
//2. Range Using SX1280


  
  startrangingmS = millis();
  LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);
  //delay(packet_delaymS);
  IrqStatus = LT.readIrqStatus(); //Irqstatus is a register value true when done
  if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID){
    digitalWrite(LED1, HIGH);
    range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
    if (range_result > 800000) {range_result = 0;}
    distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distance_adjustment);
    RangingRSSI = LT.getRangingRSSI();
    digitalWrite(LED1, LOW);
    Serial.print(RangingAddress);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.println(RangingRSSI);
  }
  delay(1000); 
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
  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(4, 125);                                       //two quick LED flashes to indicate program start
  int d = 50; //ms delay between commands
  Serial.begin(115200);            //setup Serial 
  Serial2.begin(115200);
  SPI.begin();
  
  //Setup Nanotron radios
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


  // Initialize Lora Radio
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
   // Serial.println(F("Device found"));
    //led_Flash(2, 125);
    //delay(1000);
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

  delay(1000);
}