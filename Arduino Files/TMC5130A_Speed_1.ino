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
int SW_P = 2;

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
  pinMode(SW_P, INPUT);
 
  digitalWrite(SW_A,HIGH);
  digitalWrite(SW_B,HIGH);
  digitalWrite(SW_C,HIGH);
  digitalWrite(SW_D,HIGH);
  digitalWrite(LV_RV,HIGH);
  digitalWrite(SW_P,HIGH);
 

  //set up Timer1
  TCCR1A = bit (COM1A0);                //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);    //CTC, no prescaling
  OCR1A = 0;                            //output every cycle

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  Serial.begin(9600);
  
  sendData(0x80,0x00000000);      //GCONF

  sendData(0xEC,0x000101D5);      //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0x90,0x00070603);      //IHOLD_IRUN: IHOLD=3, IRUN=10 (max.current), IHOLDDELAY=6
  sendData(0x91,0x0000000A);      //TPOWERDOWN=10

  sendData(0xF0,0x00000000);      // PWMCONF
  //sendData(0xF0,0x000401C8);      //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amp limit=200, grad=1

  sendData(0xA4,0x000003E8);     //A1=1000
  sendData(0xA5,0x00030D40);     //V1=200000
  sendData(0xA6,0x0000C350);     //AMAX=50000
  sendData(0xA7,0x00013877);     //VMAX=79991
  sendData(0xAA,0x00000578);     //D1=1400
  sendData(0xAB,0x0000000A);     //VSTOP=10

  sendData(0xA0,0x00000000);     //RAMPMODE=0

  sendData(0xA1,0x00000000);     //XACTUAL=0
  sendData(0xAD,0x00000000);     //XTARGET=0
 
}

void loop() {
//  put your main code here, to run repeatedly:

  //Serial.print((digitalRead(SW_P) ^ 0x1),DEC);
  //Serial.print(" ");
  //Serial.println(((PIND ^ 0xFF) >> 3),DEC);
  //delay(200);
  
  Serial.print(readData(TMCRhino_CHOPCONF));
  delay(300);


/*
  sendData(0xAD,0x000FA000);     //XTARGET=1024000 | 10 revolutions with micro step = 256
  delay(40000);
  sendData(0x21,0x00000000);
  sendData(0xAD,0x00000000);     //XTARGET=0
  delay(40000);
  sendData(0x21,0x00000000);
*/
}

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

  Serial.print("Received: ");
  Serial.println(i_datagram,HEX);
  Serial.print(" from register: ");
  Serial.println(address,HEX);
}

unsigned long readData(unsigned long address) {

  delay(100);

  digitalWrite(chipCS,LOW);
  delayMicroseconds(10);
  SPI.transfer(address);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(chipCS, HIGH);
  delayMicroseconds(200);
  
  digitalWrite(chipCS, LOW);
  SPI.transfer(address);
  
  unsigned long value = 0;
  value |= SPI.transfer(0x00) << 24;
  value |= SPI.transfer(0x00) << 16;
  value |= SPI.transfer(0x00) << 8;
  value |= SPI.transfer(0x00);

  digitalWrite(chipCS, HIGH);

  return value;  
}
