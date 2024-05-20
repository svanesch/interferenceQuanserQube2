import serial
import struct
import random
import sys

random.seed(42)
baudRate = 115200

# Change to actual COM port
serialPort = "COM5"

startMarkerAngle = b'\x40'      # hByte  
endMarkerAngle = b'\x23\x27'    # fByte

startMarker = b'\x24'       # hByte  
endMarker = b'\x25\x26'     # fByte

print('Trying to open serial port ' + serialPort + ' at ' + str(baudRate) + ' baudrate')

try:
    ser = serial.Serial(serialPort, baudRate, timeout=None, write_timeout=None)
    ser.flushInput()
    ser.flushOutput()
    print("Serial port " + serialPort + " opened  at " + str(baudRate) + " baudrate")
except:
    ser = None
    print("Error opening serial port " + serialPort + " at " + str(baudRate) + " baudrate")

# Send package via SPI to the microcontroller
def sendPackage(int):
    byteArray = int.to_bytes(4, byteorder='little')
    ser.write(byteArray)
    ser.flush()
    print('Sent: ', byteArray)

# Take measurements of setup and log them
def logData():
    # Check incoming byteArray on header and footer mask and length
    encRead = ser.read_until(endMarkerAngle)
    if encRead.startswith(startMarkerAngle) and len(encRead) == 17:
        encoder0Byte = encRead[1:4]
        if len(encoder0Byte) == 3:
            encoder0 = (int(encoder0Byte[2]) << 16) | (int(encoder0Byte[1]) << 8) | (encoder0Byte[0])
            
            if (encoder0 & 0x00800000):
                encoder0 = -(0x1000000 - encoder0)

            theta = encoder0 * (-2.0 * 3.14159 / 2048);
    

        encoder1Byte = encRead[4:7]
        if len(encoder1Byte) == 3:
            encoder1 = (int(encoder1Byte[2]) << 16) | (int(encoder1Byte[1]) << 8) | (encoder1Byte[0])
        
            if (encoder1 & 0x00800000):
                encoder1 = -(0x1000000 - encoder1)

            # # Uncomment for wrapping of angle alpha
            # encoder1 = encoder1 % 2048;
            # if (encoder1 < 0):
            #         encoder1 =  2048 + encoder1;
                 
            alpha = encoder1 * (2 * 3.14159 / 2048)  + 3.14159;


        timeStampByte = encRead[7:11]
        if len(timeStampByte) == 4:
            timeStamp = (int(timeStampByte[3]) << 24) | (int(timeStampByte[2]) << 16) | (int(timeStampByte[1]) << 8) | int(timeStampByte[0])
            
            timeStamp = timeStamp / 1000000


        voltageByte = encRead[11:15]
        if len(voltageByte) == 4:
            volt = struct.unpack("<f", voltageByte)[0]

        f.write(str(round(volt,6)) + ',' + str(round(theta,6)) + ',' + str(round(alpha,6)) + ',' + str(timeStamp) + '\n')

# Constants
duration = 15
sampleTime = 0.01
samples = 0
samplesRequired = duration/sampleTime

f = open('pathToSaveFile', 'w+')

# Select desired case
case = 1    # 1: Open-loop, 2: White-Noise, 3: Controller

if ser:
    sendPackage(case)
    while(True):
        # Sample data and log it
        if(samples < samplesRequired):
            logData()
            samples += 1
        
        # Close file and exit program
        else:
            # Reset variables and disable any white-noise input or controller
            sendPackage(1)

            # Close file and serial port
            f.close()
            ser.close()
            sys.exit(0)


