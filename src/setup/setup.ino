// Basic code for interference with rotary inverted pendulum
// Gathering data and applying input

// Includes
#include <SPI.h>
#include <math.h>
#include "serialSender.h"
#include "stateSpace.h"
#include "feedback.h"
#include "observer.h"

bool startup = true;

unsigned long previousMicros = 0;         // Storing when last SPI data was written
const long sampleTime = 10000;            // Sampling time for discretization [mus]

const uint8_t chipSelectPin = 5;          // Chip select on ESP32 dev board
const uint8_t calibrateButton = 22;       // GPIO of button

unsigned long SPIRate = 4000000;
unsigned long baudRate = 115200;


// Write SPI Data
byte mode = 1;                      // normal mode = 1
byte writeMask = B00011111;         // Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with writeMask
byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with writeMask
byte motorMSB = 0x80;               // motor command MSB must be B1xxxxxxx to enable the amplifier
byte motorLSB = 0;                  // motor command LSB

// Read SPI Data
byte moduleIDMSB = 0;               // module ID MSB (module ID for the QUBE Servo is 777 decimal)
byte moduleIDLSB = 0;               // module ID LSB
byte encoder0Byte[3] = {0,0,0};     // arm encoder counts
byte encoder1Byte[3] = {0,0,0};     // pendulum encoder counts
byte tach0Byte[3] = {0,0,0};        // arm tachometer
byte moduleStatus = 0;              // module status (the QUBE Servo sends status = 0 when there are no errors)
byte currentSenseMSB = 0;           // motor current sense MSB 
byte currentSenseLSB = 0;           // motor current sense LSB

byte timeByte[4];   
byte voltageByte[4];
byte caseByte[4];

float motorVoltage = 0.0;
unsigned int curCase;

unsigned long seed = 42;

// global variables for LED intensity (999 is maximum intensity, 0 is off)
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;

// White Noise intializations
float voltageLimit = 2.5;

// State space initializations
BLA::Matrix<1, 1> uk = {0};
BLA::Matrix<2, 1> yk = {0,0}; 
BLA::Matrix<4, 1> xk_ = {0,0,0,0};

//Serial builder
SerialSender serialSender;

void setup() {
  
  randomSeed(seed);
  pinMode(chipSelectPin, OUTPUT);
  pinMode(calibrateButton, INPUT_PULLUP);

  SPI.begin();

  Serial.begin(baudRate);
  Serial.setTimeout(sampleTime/2);
}

void loop() {
  if (startup){
    resetSetup();
    startup = false;
  }  
  
  if (!digitalRead(calibrateButton)) {
    calibrateSetup();
  }

  if (Serial.available() >= 4 ){
    Serial.readBytes(caseByte, 4);
    // convert bytes to int
    curCase = ((unsigned int)caseByte[3] << 24) | ((unsigned int)caseByte[2] << 16) | (unsigned int)(caseByte[1] << 8) | (unsigned int)caseByte[0];
  }

  // Start new SPI Transaction if currenttime is equal/greater than sample time
  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= sampleTime) {
    previousMicros = previousMicros + sampleTime; 

    SPI.beginTransaction(SPISettings(SPIRate, MSBFIRST, SPI_MODE2));
    digitalWrite(chipSelectPin, LOW);
    
    // Send and receive the data via SPI (except for the motor command, which is sent after the pendulum control code) 
    moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
    moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
    encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
    encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
    encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
    encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
    encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
    encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
    tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
    tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
    tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
    moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
    currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
    currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
    SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0

    /*Motor Encoder Counts*/
    long encoder0 = ((long)encoder0Byte[2] << 16) | (long)(encoder0Byte[1] << 8) | encoder0Byte[0];
    if (encoder0 & 0x00800000) {
      encoder0 = encoder0 | 0xFF000000;
    }
    // Convert the arm encoder counts to angle theta in radians
    float theta = (float)encoder0 * (-2.0 * M_PI / 2048);

    /*Pendulum Encoder Counts*/
    long encoder1 = ((long)encoder1Byte[2] << 16) | (long)(encoder1Byte[1] << 8) | encoder1Byte[0];
    if (encoder1 & 0x00800000) {
      encoder1 = encoder1 | 0xFF000000;
    }
    // Wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
    encoder1 = encoder1 % 2048;
    if (encoder1 < 0) {
      encoder1 = 2048 + encoder1;
    }    
    // Convert the pendulum encoder counts to angle alpha in radians
    float alpha = (float)encoder1 * (2.0 * M_PI / 2048) - M_PI;

    switch (curCase) {
    case 1:
      motorVoltage = 0.0;
      uk = {0};
      yk = {0, 0}; 
      xk_ = {0,0,0,0}; 
      break;
    case 2:
      voltageLimit = 2.5;   // [V]
      motorVoltage = random(-voltageLimit*1000, voltageLimit*1000)/1000.0; // [V]
      break;
    case 3: 
      if ( fabs(alpha) <= (30.0 * M_PI / 180.0) ) {
          yk = {theta, alpha};
          xk_ = Observer::stateEstimates(xk_, yk, uk);
          uk = Feedback::output(xk_);
          motorVoltage = uk(0,0);
        }
        else{
          motorVoltage = 0.0;
        }
      break;
    default:
      motorVoltage = 0.0;
      break;
    }

    // Set the saturation limit to +/- V
    if (motorVoltage > 15.0) {
      motorVoltage = 15.0;
    }
    else if (motorVoltage < -15.0) {
      motorVoltage = -15.0;
    }

    // Invert for positive CCW
    motorVoltage = -motorVoltage;

    // Convert voltage to byteArray
    memcpy(voltageByte, &motorVoltage, sizeof(float));

    float motorPWM = motorVoltage * (625.0 / 15.0);

    int motor = (int)motorPWM;  // convert float to int (2 bytes)
    motor = motor | 0x8000;  // motor command MSB must be B1xxxxxxx to enable the amplifier
    motorMSB = (byte)(motor >> 8);
    motorLSB = (byte)(motor & 0x00FF); 
    
    // Comute time before sending actual voltage
    timeByte[0] = (unsigned long) currentMicros;
    timeByte[1] = (unsigned long) currentMicros >> 8;
    timeByte[2] = (unsigned long) currentMicros >> 16;
    timeByte[3] = (unsigned long) currentMicros >> 24;

    // Send the motor data via SPI
    SPI.transfer(motorMSB);
    SPI.transfer(motorLSB);

    // De-select device and end SPI transaction
    digitalWrite(chipSelectPin, HIGH);
    SPI.endTransaction();

    // Build Byte Array
    serialSender.buildByteArr(encoder0Byte[0], encoder0Byte[1], encoder0Byte[2], encoder1Byte[0], encoder1Byte[1], encoder1Byte[2], timeByte[0], timeByte[1], timeByte[2], timeByte[3], voltageByte[0], voltageByte[1], voltageByte[2], voltageByte[3]);
    
    // Send Byte Array over serial
    if ( currentMicros - previousMicros <= (sampleTime - 100) ) {
      if(Serial.availableForWrite() >= 17) {
        Serial.write(serialSender.dData, 17);
      }
    }
  }
}

void resetSetup(){
  // Enable access to all components of setup
  writeMask = B01111111;      

  // Disable LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;

  // Reset encode values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;

  // Disable motor and amplifier
  motorMSB = 0;
  motorLSB = 0;

  // Initialize SPI bus
  SPI.beginTransaction(SPISettings(SPIRate, MSBFIRST, SPI_MODE2));
  
  digitalWrite(chipSelectPin, HIGH);  // De-select device
  digitalWrite(chipSelectPin, LOW);   // Select device

  interfaceSPI();
  
  digitalWrite(chipSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  writeMask = B00011111;  // Enable motor and LEDs, don't allow writes to Encoders
  motorMSB = 0x80;  // Enable the amplifier
}

void calibrateSetup(){
  // Acces Encoder1 and Encoder0
  writeMask = B01100000;

  // Disable motor and amplifier
  motorMSB = 0;
  motorLSB = 0;

  // Reset the encoder values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;
  
  // Initialize SPI bus
  SPI.beginTransaction(SPISettings(SPIRate, MSBFIRST, SPI_MODE2));
  
  digitalWrite(chipSelectPin, HIGH);  // De-select device
  digitalWrite(chipSelectPin, LOW);   // Select device
  
  // Send and Receive data via SPI 
  interfaceSPI();
  
  digitalWrite(chipSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  writeMask = B00011111;  // Enable motor and LEDs, don't allow writes to Encoders
  motorMSB = 0x80;  // Enable the amplifier

  // Reset Timing
  previousMicros = 0;

}

// Fixed tructure for SPI interface according to packet structure
void interfaceSPI(){
  // Send and Receive data via SPI 
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
  SPI.transfer(motorMSB);                              // send the motor MSB
  SPI.transfer(motorLSB);                              // send the motor LSB
}
