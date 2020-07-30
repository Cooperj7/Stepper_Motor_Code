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

SPISettings settingsA(2000000, MSBFIRST, SPI_MODE3);            //SPI settings that can be used by the sendData function

// variables will change:
int buttonState = 0;           //variable for reading the pushbutton status
bool DoSendData = false;       //variable to tell when to send data

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
  
  sendData(0x80,0x00000004);  //GCONF - Global config flags. 
 
  sendData(0x83,0x00000000);  //SLAVECONF - SLAVEADDR = bits 0 to 7. SENDDELAY = bits 8 to 11.
  sendData(0x84,0x00000000);  //INP_OUT - INPUT: Reads the state of all input pins available. OUTPUT: Sets the IO output pin polarity in UART mode.
  sendData(0x85,0x00000000);  //XCOMPARE - Position  comparison  register for motion controller position strobe.
  
  sendData(0x90,0x00031800);  //IHOLD_IRUN - Driver current control.
  sendData(0x91,0x00000002);  //TPOWERDOWN - sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds.
  sendData(0x93,0x0001B4E8);  //TPWMTHRS - This is the upper velocity limit for StealthChop voltage PWM mode.
  sendData(0x94,0x00000000);  //TCOOLTHRS - This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard feature.
  sendData(0x95,0x00000000);  //THIGH - This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque.

  sendData(0xA0,0x00000002);  //RAMPMODE - ramp mode. 0: Positioning mode (using all A, D and V parameters) 1: Velocity mode to positive VMAX (using AMAX acceleration) 2: Velocity mode to negative VMAX (using AMAX acceleration) 3: Hold mode (velocity remains unchanged, unless stop event occurs)
  sendData(0xA1,0x00000000);  //XACTUAL - Actual motor position.
  
  sendData(0xA3,0x00000000);  //VSTART - Motor start velocity.
  sendData(0xA4,0x00000000);  //A1 - First acceleration between VSTART and V1.
  sendData(0xA5,0x00000000);  //V1 - First acceleration / deceleration phase threshold velocity. First acceleration / deceleration phase threshold velocity (unsigned) 0: Disables A1 and D1 phase, use AMAX, DMAX only
  sendData(0xA6,0x000007D0);  //AMAX - Second acceleration between V1 and VMAX
  sendData(0xA7,0x00000000);  //VMAX - Motion ramp target velocity. It can be changed any time during a motion.
  sendData(0xA8,0x000007D0);  //DMAX - Deceleration between VMAX and V1.
  sendData(0xAA,0x00000000);  //D1 - Deceleration between V1 and VSTOP. DO NOT set 0 in positioning mode, even if V1=0!
  sendData(0xAB,0x0000000A);  //VSTOP - Motor stop velocity. DO NOT set 0 in positioning mode, minimum 10 recommended!
  sendData(0xAC,0x00000000);  //TZEROWAIT - Defines the waiting time after ramping down to zero velocity before next movement or direction inversion can start. Time range is about 0 to 2 seconds.
  sendData(0xAD,0x00000000);  //XTARGET - Target position for ramp mode. Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0. Initialize all velocity, acceleration and deceleration parameters before.

  sendData(0xB3,0x00000000);  //VDCMIN - 
  sendData(0xB4,0x00000000);  //SW_MODE - Switch mode configuration.
  
  sendData(0xB8,0x00000000);  //ENCMODE - Encoder configuration and use of N channel.
  sendData(0xB9,0x00000000);  //X_ENC - Actual encoder position
  sendData(0xBA,0x00010000);  //ENC_CONST - Encoder Constant - (Keep this same value)

//Programmable Microstep Table:           Triangle:,     50/50:,     Default(100% sine):  
  sendData(0xE0,0x88844444);  //MSLUT 0 - FFFFBFFF,     88844444,       AAAAB554   
  sendData(0xE1,0x10842110);  //MSLUT 1 - FFFFFBFF,     10842110,       4A9554AA
  sendData(0xE2,0x10102084);  //MSLUT 2 - FFFFFF7F,     10102084,       24492929
  sendData(0xE3,0x00020020);  //MSLUT 3 - FFFFFFF7,     00020020,       10104222
  sendData(0xE4,0xFEFFFFFF);  //MSLUT 4 - EFFFFFFE,     FEFFFFFF,       FBFFFFFF
  sendData(0xE5,0xEF7DF7F7);  //MSLUT 5 - FEFFFFFF,     EF7DF7F7,       B5BB777D
  sendData(0xE6,0x6DB6EDDE);  //MSLUT 6 - FFDFFFFF,     6DB6EDDE,       49295556
  sendData(0xE7,0x55AAB5AD);  //MSLUT 7 - FFFDFFFF,     55AAB5AD,       00404222
  sendData(0xE8,0xFFFF8056);  //MSLUTSEL - FFFF8055,      FFFF8056,       FFFF8056     //This register defines four segments within each quarter MSLUT wave. Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT.
  sendData(0xE9,0x00F70000);  //MSLUTSTART - 00F70000,    00F70000,       00F70000
  
  sendData(0xEC,0x00030404);  //CHOPCONF - chopper and driver configuration. TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
                              //Toff = 4
                              //tbl = 1
                              
  sendData(0xED,0x00000000);  //COOLCONF - CoolStep smart current control register.
  sendData(0xEE,0x00000000);  //DCCTRL - DcStep automatic communication configuration register (enable via pin DCEN or via VDCMIN).

  sendData(0xF0,0x000501C8);  //PWMCONF - Voltage PWM mode chopper configuration
                              //pwm_freq = 1 -> 2/683 f CLK
  
  sendData(0xF2,0x00000000);  //ENCM_CTRL - 

}

void loop() {
// put your main code here, to run repeatedly:
  int knobvalue1 = (PIND^0xFF)>>3;            //constantly read status of knob
  
      //Button logic:
 buttonState = digitalRead(buttonPin);          //constantly read status of button
  if(buttonState == LOW){                       //if button pushed, do:
    DoSendData = true;
    int TKMode = bitRead(PORTD,3);              //read LV-RV bit
      if(TKMode == 1 && knobvalue1 == 1){       //if in RV Mode and knob at 1, switch to LV Mode
        bitWrite(PORTD, 3, 0);
        delay(100);
        Serial.print("Device is in LV mode");
        Serial.print('\n');
      }
      else if(TKMode == 0 && knobvalue1 == 1){  //if in LV Mode and knob at 1, switch to RV Mode
        bitWrite(PORTD, 3, 1);                  
        delay(100);
        Serial.print("Device is in RV mode");
        Serial.print('\n');
      }
      else {//if(knobvalue1 != 1){                 //if knob is not turned to 1 (0 RPM), stop motor
        sendData(0xA7,0x00000000);
        delay(100);
      }
    }
          //knob Logic:
            delay(250);
  if(buttonState == HIGH){                      //if button not pushed, do:
    int TKMode = bitRead(PORTD,3);              //read LV-RV bit
    int knobvalue2 = (PIND^0xFF)>>3;            //Constantly read status of knob, store knob value
    //Serial.print(knobvalue2);             //CURRENT KNOB IS BROKEN --------- Does not show correct knob values when turning. 
    if (knobvalue1 != knobvalue2){              //if knob turned, set dosenddata to true        
      DoSendData = true;
    }
    if (DoSendData == true){                    //if dosenddata is true, do:
          if(knobvalue2 == 1){                  //if knob turned to 1, do
           sendData(0xA7,0x00000000);  //0ppt   //Set VMax to 0
           Serial.print("Target Speed = 0ppt");
           Serial.print('\n');
           DoSendData = false;
           delay(100);
           }
                  //LV Mode Speeds:
          if(TKMode == 0 && knobvalue2 == 3){     //if in LV Mode, do:
          sendData(0xA7,0x0000010C);  //268ppt
          Serial.print("Target Speed = 0.3 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 5){
          sendData(0xA7,0x00000219);  //537
          Serial.print("Target Speed = 0.6 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 7){
          sendData(0xA7,0x0000053E);  //1342ppt
          Serial.print("Target Speed = 1.5 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 9){
          sendData(0xA7,0x00000A7C);;  //2684ppt
          Serial.print("Target Speed = 3 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 11){
          sendData(0xA7,0x000014F9);  //5369ppt
          Serial.print("Target Speed = 6 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 13){
          sendData(0xA7,0x000029F1);;  //10737ppt
          Serial.print("Target Speed = 12 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 15){
          sendData(0xA7,0x000068DC);  //26844ppt
          Serial.print("Target Speed = 30 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 17){
          sendData(0xA7,0x0000D1B7);  //53687ppt
          Serial.print("Target Speed = 60 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 19){
          sendData(0xA7,0x00013A93);  //80531ppt
          Serial.print("Target Speed = 90 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 21){
          sendData(0xA7,0x0001A36E);  //107374ppt
          Serial.print("Target Speed = 120 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 23){
          sendData(0xA7,0x00020C4A);  //134218ppt
          Serial.print("Target Speed = 150 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 25){
          sendData(0xA7,0x00027525);  //161061ppt
          Serial.print("Target Speed = 180 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 27){
          sendData(0xA7,0x0002DE01);  //187905ppt
          Serial.print("Target Speed = 210 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 29){
          sendData(0xA7,0x000346DC);  //214748ppt
          Serial.print("Target Speed = 240 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 0 && knobvalue2 == 31){
          sendData(0xA7,0x000369D0);  //223696ppt
          Serial.print("Target Speed = 250 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          //end of if in LV TKMode, do:

                  //RV Mode Speeds
          if(TKMode == 1 && knobvalue2 == 3){          //if in RV Mode, do:
          sendData(0xA7,0x000001BF);  //447ppt
          Serial.print("Target Speed = 0.5 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 5){
          sendData(0xA7,0x0000037F);  //895ppt
          Serial.print("Target Speed = 1 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 7){
          sendData(0xA7,0x000006FE);  //1790ppt
          Serial.print("Target Speed = 2 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 9){
          sendData(0xA7,0x000008BD);;  //2237ppt
          Serial.print("Target Speed = 2.5 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 11){
          sendData(0xA7,0x00000DFB);  //3579ppt
          Serial.print("Target Speed = 4 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 13){
          sendData(0xA7,0x0000117A);;  //4474
          Serial.print("Target Speed = 5 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 15){
          sendData(0xA7,0x000022F4);  //8948ppt
          Serial.print("Target Speed = 10 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode ==  1 && knobvalue2 == 17){
          sendData(0xA7,0x000045E8);  //17896ppt
          Serial.print("Target Speed = 20 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 19){
          sendData(0xA7,0x0000AEC3);  //44739ppt
          Serial.print("Target Speed = 50 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 21){
          sendData(0xA7,0x00015D86);  //89478ppt
          Serial.print("Target Speed = 100 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 23){
          sendData(0xA7,0x00020C4A);  //134218ppt
          Serial.print("Target Speed = 150 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 25){
          sendData(0xA7,0x000263AB);  //156587ppt
          Serial.print("Target Speed = 175 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 27){
          sendData(0xA7,0x0002BB0D);  //178957ppt
          Serial.print("Target Speed = 200 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          } 
          if(TKMode == 1 && knobvalue2 == 29){
          sendData(0xA7,0x0003126F);  //201327ppt
          Serial.print("Target Speed = 225 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }
          if(TKMode == 1 && knobvalue2 == 31){
          sendData(0xA7,0x000369D0);  //223696ppt
          Serial.print("Target Speed = 250 RPM");
          Serial.print('\n');
          DoSendData = false;
          delay(100);
          }    
            //end of if in RV TKMode, do:

          } //end of if dosenddata=true, do:    
     }// end of if(button not pushed), do:
}//end of loop();



//Send Data Function:
void sendData(unsigned long address, unsigned long datagram) {
 //TMC5130 takes 40 bit data: 8 address and 32 data

 delay(100);
 uint8_t stat;
 uint32_t i_datagram;
 SPI.beginTransaction(settingsA);
 digitalWrite(chipCS,LOW);
 delayMicroseconds(10);

 stat = SPI.transfer(address);

 i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram) & 0xff);
 digitalWrite(chipCS,HIGH);
 SPI.endTransaction();
 
 delayMicroseconds(10);
 Serial.print('\n');
 Serial.print("Register received data: ");
 PrintHex40(stat, datagram);
 Serial.print('\n');
 Serial.print("From: ");
 Serial.print(address, HEX);
 Serial.print('\n');
 
}//end of sendData funciton

void PrintHex40(uint8_t stat, uint32_t data)      // prints 40-bit data in hex with leading zeroes
{
 char tmp[16];
 uint16_t LSB = data & 0xffff;
 uint16_t MSB = data >> 16;
 sprintf(tmp, "0x%.2X%.4X%.4X", stat, MSB, LSB);
 Serial.print(tmp);
}//end of PrintHex funciton

//Read Data Function: 
/*
uint32_t readData(uint8_t address) {
  SPI.beginTransaction(settingsA);
  digitalWrite(chipCS,LOW);
  SPI.transfer(address); 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(chipCS, HIGH);
  SPI.endTransaction();
  
  // skip a beat
  #define nop() asm volatile("nop")
  nop();
  nop();
  nop();
  
  SPI.beginTransaction(settingsA);
  digitalWrite(chipCS, LOW);
  SPI.transfer(address);
  uint32_t value = 0;
  
  value |= SPI.transfer(0x00) << 24;
  value |= SPI.transfer(0x00) << 16;
  value |= SPI.transfer(0x00) << 8;
  value |= SPI.transfer(0x00);
  
  SPI.endTransaction();
  digitalWrite(chipCS, HIGH);
  //Serial.print("Read value = ");
  //PrintHex40(address, value);
  Serial.print('\n');
  //Serial.print("Got: ");
  //Serial.println(value,HEX);
  uint32_t ActVel = ~value+1;
  return ActVel;  
}//end of readData Function

*/



  
