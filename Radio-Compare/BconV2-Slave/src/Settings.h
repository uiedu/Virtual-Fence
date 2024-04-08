/*****************************************************************************************************
  Use this file to change settings for radio operation
  Stuart Robinson's Slave setting code for Sx120180 was used and modified for this application
  Modified by Dev Shrestha 04/06/2024 
  1. DIO1 pin in Sx1280 is used to indicate radio ready and connected to pin 5.
  2. All DIO pins in Nanotron is set to input (not used)

  
*******************************************************************************************************/
// Set Station ID
uint32_t RangingAddress = 1;         //must match address in master (Single didgit only for now, Generalize to have any number)
String radioID = "000000000B0" + String(RangingAddress);  // 9 0s followed by B01 to Bxx


//  Sx1280 Experimental settings
uint8_t Bandwidth = LORA_BW_0800;    //LoRa bandwidth
uint8_t SpreadingFactor = LORA_SF8;  //LoRa spreading factor
int8_t TXpower = 31;                 //Transmit power used 0 to 31
uint16_t Calibration = 11426;        //Ranging calibrarion value from table below or from manual calibration for better results
/*  Calibration values from applicaiton note
__________________________________________________________________
                                           SF
                  ------------------------------------------------          
                    5       6       7       8       9       10
                  ------------------------------------------------  
         | 400   |  10299   10271   10244   10242   10230   10246
  BW(kHz)| 800   |  11486   11474   11453   11426   11417   11401 
         | 1600  |  13308   13493   13528   13515   13430   13376
______________________________________________________________________*/  


//*******  Setup Sx1280 hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 5
#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t  rangingRXTimeoutmS = 0xFFFF;     //ranging RX timeout in mS
