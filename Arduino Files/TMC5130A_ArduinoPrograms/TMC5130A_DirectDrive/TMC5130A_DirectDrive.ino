#include <SPI.h>
#include "TMC5130_registers.h"

/* The trinamic TMC5130 motor controller and driver operates through an 
 *  SPI interface.  Each datagram is sent to the device as an address byte
 *  followed by 4 data bytes.  This is 40 bits (8 bit address and 32 bit word).
 *  Each register is specified by a one byte (MSB) address: 0 for read, 1 for 
 *  write.  The MSB is transmitted first on the rising edge of SCK.
 *  
 * Arduino Uno  Mega  Eval Board Pins
 *  MOSI   11   51      32 SPI1_SDI
 *  MISO   12   50      33 SPI1_SDO
 *  SCK    13   52      31 SPI1_SCK
 *  DIO    10   10      30 SPI1_CSN
 *  DIO     8    8       8 DIO0 (DRV_ENN)
 *  OC1A    9   11      23 CLK16
 *  GND                  2 GND
 *  +5V                  5 +5V
 */

int chipCS = 10;
int CLOCKOUT = 9;   //Uno=9 Mega=11
int enable = 8;

int SW_A = 7;
int SW_B = 6;
int SW_C = 5;
int SW_D = 4;
int LV_RV = 3;
int buttonPin = 2;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int knobstate =0;
void setup() {
//  put your setup code here, to run once:
  pinMode(chipCS,OUTPUT);
  pinMode(CLOCKOUT,OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(chipCS,HIGH);
  digitalWrite(enable,LOW);

  pinMode(SW_A, INPUT);
  pinMode(SW_B, INPUT);
  pinMode(SW_C, INPUT);
  pinMode(SW_D, INPUT);
  pinMode(LV_RV, INPUT);
  pinMode(buttonPin, INPUT);
 
  digitalWrite(SW_A,HIGH);
  digitalWrite(SW_B,HIGH);
  digitalWrite(SW_C,HIGH);
  digitalWrite(SW_D,HIGH);
  digitalWrite(LV_RV,HIGH);
  digitalWrite(buttonPin,HIGH);
 

  //set up Timer1
  TCCR1A = bit (COM1A0);                //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);    //CTC, no prescaling
  OCR1A = 0;                            //output every cycle

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  Serial.begin(9600);
  
  //Register Setup:
  //REMEMBER -- Add 0x80 to address for Write Privalege.
  
  sendData(0x80,0x00000006);  //GCONF - Global config flags. 
 
  sendData(0x83,0x00000000);  //SLAVECONF - SLAVEADDR = bits 0 to 7. SENDDELAY = bits 8 to 11.
  sendData(0x84,0x00000000);  //INP_OUT - INPUT: Reads the state of all input pins available. OUTPUT: Sets the IO output pin polarity in UART mode.
  sendData(0x85,0x00000000);  //XCOMPARE - Position  comparison  register for motion controller position strobe.
  
  sendData(0x90,0x00051500);  //IHOLD_IRUN - Driver current control.
  sendData(0x91,0x00000002);  //TPOWERDOWN - sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds.
  sendData(0x93,0x00000000);  //TPWMTHRS - This is the upper velocity limit for StealthChop voltage PWM mode.
  sendData(0x94,0x00000000);  //TCOOLTHRS - This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard feature.
  sendData(0x95,0x00000000);  //THIGH - This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque.

  sendData(0xA0,0x00000002);  //RAMPMODE - ramp mode. 0: Positioning mode (using all A, D and V parameters) 1: Velocity mode to positive VMAX (using AMAX acceleration) 2: Velocity mode to negative VMAX (using AMAX acceleration) 3: Hold mode (velocity remains unchanged, unless stop event occurs)
  sendData(0xA1,0x00000000);  //XACTUAL - Actual motor position.
  
  sendData(0xA3,0x00000000);  //VSTART - Motor start velocity.
  sendData(0xA4,0x00000000);  //A1 - First acceleration between VSTART and V1.
  sendData(0xA5,0x00000000);  //V1 - First acceleration / deceleration phase threshold velocity. First acceleration / deceleration phase threshold velocity (unsigned) 0: Disables A1 and D1 phase, use AMAX, DMAX only
  sendData(0xA6,0x000002EE);  //AMAX - Second acceleration between V1 and VMAX
  sendData(0xA7,0x00000000);  //VMAX - Motion ramp target velocity. It can be changed any time during a motion.
  sendData(0xA8,0x000002EE);  //DMAX - Deceleration between VMAX and V1.
  sendData(0xAA,0x00000000);  //D1 - Deceleration between V1 and VSTOP. DO NOT set 0 in positioning mode, even if V1=0!
  sendData(0xAB,0x0000000A);  //VSTOP - Motor stop velocity. DO NOT set 0 in positioning mode, minimum 10 recommended!
  sendData(0xAC,0x00000000);  //TZEROWAIT - Defines the waiting time after ramping down to zero velocity before next movement or direction inversion can start. Time range is about 0 to 2 seconds.
  sendData(0xAD,0x00000000);  //XTARGET - Target position for ramp mode. Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0. Initialize all velocity, acceleration and deceleration parameters before.

  sendData(0xB3,0x00000000);  //VDCMIN - 
  sendData(0xB4,0x00000000);  //SW_MODE - Switch mode configuration.
  
  sendData(0xB8,0x00000000);  //ENCMODE - Encoder configuration and use of N channel.
  sendData(0xB9,0x00000000);  //X_ENC - Actual encoder position
  sendData(0xBA,0x00010000);  //ENC_CONST -

//Programmable Microstep Table:           Triangle:,     50/50:,     Default(100% sine):  
  sendData(0xE0,0x11111110);  //MSLUT 0 - FFFFFFFE,     11111110,       AAAAB554   
  sendData(0xE1,0x21084211);  //MSLUT 1 - FFFFFFF7,     21084211,       4A9554AA
  sendData(0xE2,0x04081084);  //MSLUT 2 - FFFFFF7F,     04081084,       24492929
  sendData(0xE3,0x00020204);  //MSLUT 3 - FFFF7FFF,     00020204,       10104222
  sendData(0xE4,0xF8000000);  //MSLUT 4 - FEFFFFFF,     F8000000,       FBFFFFFF
  sendData(0xE5,0xBF7F7EFD);  //MSLUT 5 - FF7FFFFF,     BF7F7EFD,       B5BB777D
  sendData(0xE6,0xBB776EDD);  //MSLUT 6 - FFFFFFFF,     BB776EDD,       49295556
  sendData(0xE7,0x2AAD6B5A);  //MSLUT 7 - 7FFFEFFF,     2AAD6B5A,       00404222
  sendData(0xE8,0xFFFF9A56);  //MSLUTSEL - FFFFFF55,      FFFF9A56,       FFFF8056     //This register defines four segments within each quarter MSLUT wave. Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT.
  sendData(0xE9,0x00F80000);  //MSLUTSTART - 00F80000,    00F80000,       00F70000
  
  sendData(0xEC,0x00010044);  //CHOPCONF - chopper and driver configuration. TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0xED,0x00000000);  //COOLCONF - CoolStep smart current control register.
  sendData(0xEE,0x00000000);  //DCCTRL - DcStep automatic communication configuration register (enable via pin DCEN or via VDCMIN).

  sendData(0xF0,0x000602C8);  //PWMCONF - Voltage PWM mode chopper configuration
  
  sendData(0xF2,0x00000000);  //ENCM_CTRL - 
 
}

void loop() {
//  put your main code here, to run repeatedly:

  //delay(500);
  //Serial.print((digitalRead(SW_P) ^ 0x1),DEC);          //print state if button
  //delay(5000);
  //Serial.print(" ");
  //Serial.println(((PIND ^ 0xFF) >> 3),DEC);             //print state of knob
  //delay(200);
  
  //Serial.print(readData(GCONF));
  //delay(300);
  
  //Button logic:
  buttonState = digitalRead(buttonPin);                 //constantly read status of button
  if(buttonState == LOW){                                   //if button pushed, do
    int ModeValue = readData(GCONF);
      if (ModeValue == 0){
        sendData(0x80,0x00000006);
        Serial.print("StealthChop Activated");
        Serial.print('\n');
      }
     else if (ModeValue == 4){
        sendData(0x80,0x00000000);
        Serial.print("StealthChop Deactivated");
        Serial.print('\n');
     }
  }//end of if(button pushed) do:

 //Knob logic:
 int knobvalue1 = (PIND^0xFF)>>3;                  //Constantly read status of knob, store knob value
 delay(650);
 int knobvalue2 = (PIND^0xFF)>>3;                  //Constantly read status of knob, store knob value
 
 if (knobvalue1 != knobvalue2){                   //if knob value1 does not equal knob value2, knob was turned so do:
   knobstate = (PIND^0xFF)>>3,DEC;
    if(knobstate == 1){                                    //if knob turned to 1, do
    sendData(0xA7,0x00000000);  //0ppt                     //Set VMax to 0
    Serial.print("Target Speed = 0ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 3){
    sendData(0xA7,0x000001BF);  //447ppt
    Serial.print("Target Speed = 0.5 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 5){
    sendData(0xA7,0x0000037F);  //895ppt
    Serial.print("Target Speed = 1 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 7){
    sendData(0xA7,0x000006FE);  //1790ppt
    Serial.print("Target Speed = 2 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 9){
    sendData(0xA7,0x00000A7C);  //2684ppt
    Serial.print("Target Speed = 3 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 11){
    sendData(0xA7,0x0000117A);  //4474ppt
    Serial.print("Target Speed = 5 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 13){
    sendData(0xA7,0x00001A37);  //6711ppt
    Serial.print("Target Speed = 7.5 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 15){
    sendData(0xA7,0x000022F4);  //8948ppt
    Serial.print("Target Speed = 10 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 17){
    sendData(0xA7,0x0000346E);  //13422ppt
    Serial.print("Target Speed = 15 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 19){
    sendData(0xA7,0x000045E8);  //17896ppt
    Serial.print("Target Speed = 20 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 21){
    sendData(0xA7,0x00005762);  //22370ppt
    Serial.print("Target Speed = 25 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 23){
    sendData(0xA7,0x000068DC);  //26844ppt
    Serial.print("Target Speed = 30 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 25){
    sendData(0xA7,0x00007A55);  //31317ppt
    Serial.print("Target Speed = 35 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 27){
    sendData(0xA7,0x00008BCF);  //35791ppt
    Serial.print("Target Speed = 40 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 29){
    sendData(0xA7,0x00009D49);  //40265ppt
    Serial.print("Target Speed = 45 RPM");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 31){
    sendData(0xA7,0x0000AEC3);  //44739ppt
    Serial.print("Target Speed = 50 RPM");
    Serial.print('\n');
    delay(100);
  }
 } //end of (if knob turned, do:)

 
}//end of loop();

//Send Data Function:
void sendData(unsigned long address, unsigned long datagram) {
  //TMC5130 takes 40 bit data: 8 address and 32 data

  delay(100);
  unsigned long i_datagram;

  digitalWrite(chipCS,LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(chipCS,HIGH);

  delay(100);

  //Serial.print("Received: ");
  //Serial.println(i_datagram,HEX);
  Serial.print("Sent data to register: ");
  Serial.println(address,HEX);
}//end of sendData funciton

//Read Data Function:
unsigned long readData(unsigned long address) {

  delay(100);

  digitalWrite(chipCS,LOW);
  //delayMicroseconds(10);
  
  SPI.transfer(address);
 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(chipCS, HIGH);
  
  delayMicroseconds(200);
  
  digitalWrite(chipCS, LOW);

  SPI.transfer(address);
  unsigned long  value = 0;
 
  value |= SPI.transfer(0x00) << 24;
  value |= SPI.transfer(0x00) << 16;
  value |= SPI.transfer(0x00) << 8;
  value |= SPI.transfer(0x00);
  
  digitalWrite(chipCS, HIGH);

  //Serial.print("Got: ");
  //Serial.println(value,HEX);
  
  return value;  
}//end of readData Function
