
#ifndef serialSender_h
#define serialSender_h

#include <Arduino.h>

class SerialSender
{
public:
	SerialSender();
  void buildByteArr(byte enc0_0, byte enc0_1, byte enc0_2, byte enc1_0, byte enc1_1, byte enc1_2, byte time0, byte time1, byte time2, byte time3, byte volt0, byte volt1, byte volt2, byte volt3);

	byte dData[17];
private:
	void reset();
};
#endif
