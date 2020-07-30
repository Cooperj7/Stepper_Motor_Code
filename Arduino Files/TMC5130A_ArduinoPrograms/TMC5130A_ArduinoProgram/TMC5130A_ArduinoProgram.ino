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
  
  sendData(0x80,0x00000004);  //GCONF - Global config flags. 
 
  sendData(0x83,0x00000000);  //SLAVECONF - SLAVEADDR = bits 0 to 7. SENDDELAY = bits 8 to 11.
  sendData(0x84,0x00000000);  //INP_OUT - INPUT: Reads the state of all input pins available. OUTPUT: Sets the IO output pin polarity in UART mode.
  sendData(0x85,0x00000000);  //XCOMPARE - Position  comparison  register for motion controller position strobe.
  
  sendData(0x90,0x00071703);  //IHOLD_IRUN - Driver current control.
  sendData(0x91,0x00000000);  //TPOWERDOWN - sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds.
  sendData(0x93,0x00000000);  //TPWMTHRS - This is the upper velocity limit for StealthChop voltage PWM mode.
  sendData(0x94,0x00000000);  //TCOOLTHRS - This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard feature.
  sendData(0x95,0x00000000);  //THIGH - This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque.

  sendData(0xA0,0x00000001);  //RAMPMODE - ramp mode. 0: Positioning mode (using all A, D and V parameters) 1: Velocity mode to positive VMAX (using AMAX acceleration) 2: Velocity mode to negative VMAX (using AMAX acceleration) 3: Hold mode (velocity remains unchanged, unless stop event occurs)
  sendData(0xA1,0x00000000);  //XACTUAL - Actual motor position.
  
  sendData(0xA3,0x00000000);  //VSTART - Motor start velocity.
  sendData(0xA4,0x00000000);  //A1 - First acceleration between VSTART and V1.
  sendData(0xA5,0x00000000);  //V1 - First acceleration / deceleration phase threshold velocity. First acceleration / deceleration phase threshold velocity (unsigned) 0: Disables A1 and D1 phase, use AMAX, DMAX only
  sendData(0xA6,0x000002EE);  //AMAX - Second acceleration between V1 and VMAX
  sendData(0xA7,0x000002EE);  //VMAX - Motion ramp target velocity. It can be changed any time during a motion.
  sendData(0xA8,0x00000000);  //DMAX - Deceleration between VMAX and V1.
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
  sendData(0xE8,0xFFFF9A56);  //MSLUTSEL - FFFFFF55,      FFFF9A56,       FFF8056     //This register defines four segments within each quarter MSLUT wave. Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT.
  sendData(0xE9,0x00F80000);  //MSLUTSTART - 00F80000,    00F80000,       00F70000
  
  sendData(0xEC,0x000101D5);  //CHOPCONF - chopper and driver configuration. TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0xED,0x00000000);  //COOLCONF - CoolStep smart current control register.
  sendData(0xEE,0x00000000);  //DCCTRL - DcStep automatic communication configuration register (enable via pin DCEN or via VDCMIN).

  sendData(0xF0,0x000501C8);  //PWMCONF - Voltage PWM mode chopper configuration
  
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
  
  //Serial.print(readData(TMCRhino_CHOPCONF));
  //delay(300);
  
  //Button logic:
  buttonState = digitalRead(buttonPin);                 //constantly read status of button
  
  if(buttonState == LOW){                                   //if button pushed, do
  unsigned long Steps = readData(CHOPCONF);                 //read MRes
    delay(250);
    if(Steps == 256){                                     //if Mres = 256 msteps
      sendData(0xEC,0x080101D5);                          //change Mres to = 1 step
      sendData(0xA6,0x00000004);                          //change AMax and DMax to 4
      sendData(0xA8,0x00000004);
      
      int ActualSteps = 1;
      Serial.print("Microsteps = 1/");//Mres              //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 257){                                     //if Mres = 128 msteps
      sendData(0xEC,0x000101D5);                          //change Mres to = 256 msteps
      sendData(0xA6,0x000002EE);                          //change AMax and DMax to 750
      sendData(0xA8,0x000002EE);

      int ActualSteps = 256;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
   if(Steps == 258){                                      //if Mres = 64 msteps
      sendData(0xEC,0x010101D5);                          //change Mres to = 128 msteps
      sendData(0xA6,0x0000015E);                          //change AMax and DMax to 350
      sendData(0xA8,0x0000015E);
       
      int ActualSteps = 128;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 259){                                     //if Mres = 32 msteps
      sendData(0xEC,0x020101D5);                          //change Mres to = 64 msteps
      sendData(0xA6,0x000000EB);                          //change AMax and DMax to 235
      sendData(0xA8,0x000000EB);

      int ActualSteps = 64;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 260){                                     //if Mres = 16 msteps
      sendData(0xEC,0x030101D5);                          //change Mres to = 32 msteps
      sendData(0xA6,0x0000007D);                          //change AMax and DMax to 125
      sendData(0xA8,0x0000007D);

      int ActualSteps = 32;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 261){                                     //if Mres = 8 msteps
      sendData(0xEC,0x040101D5);                          //change Mres to = 16 msteps
      sendData(0xA6,0x0000003C);                          //change AMax and DMax to 60
      sendData(0xA8,0x0000003C);

      int ActualSteps = 16;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 262){                                     //if Mres = 4 msteps
      sendData(0xEC,0x050101D5);                          //change Mres to = 8 msteps
      sendData(0xA6,0x0000001E);                          //change AMax and DMax to 30
      sendData(0xA8,0x0000001E);

      int ActualSteps = 8;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 263){                                     //if Mres = 2 msteps
      sendData(0xEC,0x060101D5);                          //change Mres to = 4 msteps
      sendData(0xA6,0x0000000F);                          //change AMax and DMax to 15
      sendData(0xA8,0x0000000F);

      int ActualSteps = 4;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }
    if(Steps == 264){                                     //if Mres = 1 mstep
      sendData(0xEC,0x070101D5);                          //change Mres to = 2 msteps
      sendData(0xA6,0x00000008);                          //change AMax and DMax to 8
      sendData(0xA8,0x00000008);

      int ActualSteps = 2;
      Serial.print("Steps = ");//Mres                     //print new steps value
      Serial.println(ActualSteps);
      Serial.print('\n');
      delay(250);
    }    
  }
 delay(100);

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
    sendData(0xA7,0x00000708);  //1800ppt
    Serial.print("Target Speed = 1800ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 5){
    sendData(0xA7,0x00000EA6);  //3750ppt
    Serial.print("Target Speed = 3750ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 7){
    sendData(0xA7,0x0000157C);  //5500ppt
    Serial.print("Target Speed = 5500ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 9){
    sendData(0xA7,0x00001E46);  //7750ppt
    Serial.print("Target Speed = 7750ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 11){
    sendData(0xA7,0x00003C8C);  //15500ppt
    Serial.print("Target Speed = 15500ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 13){
    sendData(0xA7,0x00004E20);  //20000ppt
    Serial.print("Target Speed = 20000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 15){
    sendData(0xA7,0x00007A12);  //31250ppt
    Serial.print("Target Speed = 31250ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 17){
    sendData(0xA7,0x0000AFC8);  //45000ppt
    Serial.print("Target Speed = 45000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 19){
    sendData(0xA7,0x0000F424);  //62500ppt
    Serial.print("Target Speed = 6250ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 21){
    sendData(0xA7,0x00014C08);  //85000ppt
    Serial.print("Target Speed = 85000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 23){
    sendData(0xA7,0x0001E848);  //125000ppt
    Serial.print("Target Speed = 125000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 25){
    sendData(0xA7,0x0002AB98);  //175000ppt
    Serial.print("Target Speed = 175000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 27){
    sendData(0xA7,0x0003D090);  //2500000ppt
    Serial.print("Target Speed = 250000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 29){
    sendData(0xA7,0x00055730);  //350000ppt
    Serial.print("Target Speed = 350000ppt");
    Serial.print('\n');
    delay(100);
  }
    if(knobstate == 31){
    sendData(0xA7,0x0007A508);  //501000ppt
    Serial.print("Target Speed = 501000ppt");
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

  value |= SPI.transfer(0x00);
  value |= SPI.transfer(0x00) << 8;
  value |= SPI.transfer(0x00) << 16;
  value |= SPI.transfer(0x00) << 24;
  
  digitalWrite(chipCS, HIGH);

  //Serial.print("Got: ");
  //Serial.println(value,HEX);
  
  return value;  
}//end of readData Function
