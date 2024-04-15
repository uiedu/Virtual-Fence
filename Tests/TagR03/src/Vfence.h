//*************************************************************
// PIN settings
//*************************************************************
//
 
//SPI pins

#define PSS A0					// SS pin for Potentiometer MCP41X2-104
#define FSS A3					// SS pin for Flash memory

// Other Radio Pins
#define NSS 10					// SS pin for SX1280 (SX1280 E28-2G4M12S) Radoo
#define RFBUSY 7				//Radio Rfbusy pin (Stays high if there is any error in chip including solding shorts)
#define NRESET 9				//Rest pin 
#define DIO1 5
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used
#define RX_EN -1                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                //pin for TX enable, used on some SX1280 devices, set to -1 if not used 
#define NMOSI 11				//Transmitt from Radio
#define NMISO 12				//Receive from Radio

//Digital Potentiometer Pins

#define PMOSI 11				//Write Potentiometer value
#define PMISO 12				//Read Potentiometer value


// Flash Pins
#define FMOSI 11				//Write to flash memory
#define FMISO 12				//Read from flash memory

// GPS Pins (CAN BE USED FOR REMOTE)
#define GBE A0				//Boost Enable
#define GRx 3					//Arduino to GPS
#define GTx 4					//GPS to Arduino

//Remote Conctroller Pins

//I2CLCD A4 SDA and A5 SCL
#define RedLED 6
#define GreenLED 8
#define SchockSwitch 3

#define JoyXpin A0
#define JoyYpin A1
#define JoyButton 2 //Attched to interrupt must be pin 2 or 3



//Other pins
#define SHOCK 6					//Controls IRL 540 switch
#define CHARGE 2
#define BUZZER A2				//Controls Buzzer pin high
#define LED1 A7					//Controls LED 1 (need a jumper)
#define LED2 A1					//Controls LED 2 
#define LED3 8					//Controls LED 3
#define LED4 A6					//Controls LED 4 (need a jumper)

// Variable declarations
byte AID = 1;
byte EXP = 1;
int MaxBeeps = 5;
int BeepCounter = MaxBeeps+2;
int SDuration = 20;
byte SIntensity = 1;
int ShockCounter = 0;


//Memory Variables
unsigned long Last_Address = 0;
unsigned long counter = 0;

//Temporary Varaibles
String MsgIn = "";
String MsgOut = "";
String cmd = "";
byte Editing = 0;
bool Active = false;  //True is shock button is pressed until it is cancelled or shock delivered.
bool JoyPressed = false;


// Joystick Constant values
int LowThres = 256;
int HighThres = 768;
int ThresMargin =127;
int JoyXval = 512;
int JoyYval = 512;
bool JoyLeft = false;
bool JoyRight = false;
bool JoyUp = false;
bool JoyDown = false;

 

long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

byte PotR = 0;
int NewPotR = 0;



//Radio settings from Stuart's code
#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using
#define RXBUFFER_SIZE 255                       //RX buffer size
uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
int16_t PacketRSSI;                             //stores RSSI of received packet
int8_t  PacketSNR;                              //stores signal to noise ratio (SNR) of received packet

#define TXpower 10                              //LoRa transmit power in dBm
uint8_t TXPacketL;

void PinInitialize(){
  //Trun off Radio
  pinMode(NSS, OUTPUT); 
  pinMode(PSS, OUTPUT);
  digitalWrite(FSS, OUTPUT);
  
  digitalWrite(PSS, HIGH);
  digitalWrite(FSS, HIGH);

  pinMode(RedLED, OUTPUT); 
  digitalWrite(RedLED, LOW);
  
  pinMode(GreenLED, OUTPUT); 
  digitalWrite(GreenLED, LOW);
  
}

