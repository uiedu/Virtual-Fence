//LoRa Modem Varaible Parameters (Put varaibles here )
uint8_t Bandwidth = LORA_BW_0800;           //LoRa bandwidth
uint8_t SpreadingFactor = LORA_SF8;         //LoRa spreading factor
int8_t RangingTXPower = 31;                 //Transmit power used for ranging varies from 0 to 31
uint16_t Calibration = 11426;               // Ranging calibrarion value
/*****************************************************************************************************
  All Variable Definitions
  CAlibration values from applicaiton note
__________________________________________________________________
                                           SF
                  ------------------------------------------------          
                    5       6       7       8       9       10
                  ------------------------------------------------  
         | 400   |  10299   10271   10244   10242   10230   10246
  BW(kHz)| 800   |  11486   11474   11453   11426   11417   11401 
         | 1600  |  13308   13493   13528   13515   13430   13376
______________________________________________________________________  

*******************************************************************************************************/
//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10        //Chip Select pin. From Circuit digram (Beaccon V2 with Mega) NSSCTS is connected to D10
#define RFBUSY 7      //From Circuit digram (Beaccon V2 with Mega) BUSY is connected to D7
#define NRESET 9      //From Circuit digram (Beaccon V2 with Mega) NRREET is connected to D10
#define DIO1 5//3     //From Circuit digram (Beaccon V2 with Mega) DIO1 is connected to D5
#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using

//*******  Setup LoRa Parameters Here ! ***************
//Board LEDs
#define LED1 30 //defined as output in beaconinit
#define LED2 31//defined as output in beaconinit
#define LED3 32//defined as output in beaconinit
#define LED4 33//defined as output in beaconinit
#define LED5 34//defined as output in beaconinit
#define SX1280LED 8//defined as output 

//LoRa Modem Constant Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t  rangingRXTimeoutmS = 0x00FF;     //ranging RX timeout in mS
const uint16_t  waittimemS = 10000;              //wait this long in mS for packet before assuming timeout
const uint16_t  TXtimeoutmS = 5000;              //ranging TX timeout in mS
const uint16_t  packet_delaymS = 0;              //forced extra delay in mS between ranging requests
const uint16_t  rangeingcount = 1;               //number of times ranging is carried out for each distance measurment
const float distance_adjustment = 1.0000;              //adjustment factor to calculated distance
const uint8_t TXpower = 31;                        //Transmit power used to communicate it is always 31


// Flash memory 
#define Flashpin         21				// Flash memory SS pin
uint16_t expectedDeviceID = 0xEF40;
uint32_t Last_Address = 0;
uint8_t m = 6;                  //Address of memory wehre Last_address starts
