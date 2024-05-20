
#include <Arduino.h>
#include "serialSender.h"

// Header and Footer bytes for start and end of byte-package
const byte hByte = 0x40;
const byte fByte1 = 0x23;
const byte fByte2 = 0x27;

//Serial Variables
static byte dData[17];

//Create the display class
SerialSender::SerialSender()
{
	this->dData[17];
}

void SerialSender::buildByteArr(byte enc0_0, byte enc0_1, byte enc0_2, byte enc1_0, byte enc1_1, byte enc1_2, byte time0, byte time1, byte time2, byte time3, byte volt0, byte volt1, byte volt2, byte volt3)
{

		reset();

    this->dData[0] = hByte;
    this->dData[1] = enc0_0;
    this->dData[2] = enc0_1;
    this->dData[3] = enc0_2;
    this->dData[4] = enc1_0;
    this->dData[5] = enc1_1;
    this->dData[6] = enc1_2;
    this->dData[7] = time0;
    this->dData[8] = time1;
    this->dData[9] = time2;
    this->dData[10] = time3;
    this->dData[11] = volt0;
    this->dData[12] = volt1;
    this->dData[13] = volt2;
    this->dData[14] = volt3;
    this->dData[15] = fByte1;
    this->dData[16] = fByte2;
}

void SerialSender::reset()
{
  // Reset only encoder values
  this->dData[1] = 0;
  this->dData[2] = 0;
  this->dData[3] = 0;
  this->dData[4] = 0;
  this->dData[5] = 0;
  this->dData[6] = 0;
  this->dData[11] = 0;
  this->dData[12] = 0;
  this->dData[13] = 0;
  this->dData[14] = 0;
}
