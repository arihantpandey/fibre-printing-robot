#=====================================

#  Function Definitions

#=====================================

def sendToArduino(sendStr):
  global startMarker, endMarker
  txLen = chr(len(sendStr))
  adjSendStr = encodeHighBytes(sendStr)
  adjSendStr = chr(startMarker) + txLen + adjSendStr + chr(endMarker)
  ser.write(adjSendStr)


#======================================

def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    ck = ck + x 
    x = ser.read()
    byteCount += 1
    
  # save the end marker byte
  ck = ck + x 
  
  returnData = []
  returnData.append(ord(ck[1]))
  returnData.append(decodeHighBytes(ck))
#  print"RETURNDATA " + str(returnData[0])
  
  return(returnData)

#======================================

def encodeHighBytes(inStr):
  global specialByte
  
  outStr = ""
  s = len(inStr)
  
  for n in range(0, s):
    x = ord(inStr[n])
    
    if x >= specialByte:
       outStr = outStr + chr(specialByte)
       outStr = outStr + chr(x - specialByte)
    else:
       outStr = outStr + chr(x)
       
#  print"encINSTR  " + bytesToString(inStr)
#  print"encOUTSTR " + bytesToString(outStr)

  return(outStr)


#======================================

def decodeHighBytes(inStr):

  global specialByte
  
  outStr = ""
  n = 0
  
  while n < len(inStr):
     if ord(inStr[n]) == specialByte:
        n += 1
        x = chr(specialByte + ord(inStr[n]))
     else:
        x = inStr[n]
     outStr = outStr + x
     n += 1
     
  print("decINSTR  " + bytesToString(inStr))
  print("decOUTSTR " + bytesToString(outStr))

  return(outStr)


#======================================

def displayData(data):

  n = len(data) - 3

  print("NUM BYTES SENT->   " + str(ord(data[1])))
  print("DATA RECVD BYTES-> " + bytesToString(data[2:-1]))
  print("DATA RECVD CHARS-> " + data[2: -1])

#======================================

def bytesToString(data):

  byteString = ""
  n = len(data)
  
  for s in range(0, n):
    byteString = byteString + str(ord(data[s]))
    byteString = byteString + "-"
    
  return(byteString)


#======================================

def displayDebug(debugStr):

   n = len(debugStr) - 3
   print("DEBUG MSG-> " + debugStr[2: -1])


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global endMarker
    
    msg = ""
    while msg.find("Arduino Ready") == -1:

      while ser.inWaiting() == 0:
        x = 'z'

      # then wait until an end marker is received from the Arduino to make sure it is ready to proceed
      x = "z"
      while ord(x) != endMarker: # gets the initial debugMessage
        x = ser.read()
        msg = msg + x


      displayDebug(msg)
      print
      

#======================================

# THE DEMO PROGRAM STARTS HERE

#======================================

import serial
import time

# NOTE the user must ensure that the next line refers to the correct comm port
ser = serial.Serial("/dev/ttyACM0", 9600)


startMarker = 254
endMarker = 255
specialByte = 253


waitForArduino()

print("Arduino is ready")

testData = []
testData.append("abcde")
testData.append("zxcv1234")
testData.append("a" + chr(16) + chr(32) + chr(0)  + chr(203))
testData.append("b" + chr(16) + chr(32) + chr(253) + chr(255) + chr(254) + chr(253) + chr(0))
testData.append("fghijk")

numLoops = len(testData)
n = 0
waitingForReply = False

while n < numLoops:
  print("LOOP " + str(n))
  teststr = testData[n]

  if ser.inWaiting() == 0 and waitingForReply == False:
    sendToArduino(teststr)
    print("=====sent from PC==========")
    print("LOOP NUM " + str(n))
    print("BYTES SENT -> " + bytesToString(teststr))
    print("TEST STR " + teststr)
    print("===========================")
    waitingForReply = True

  if ser.inWaiting > 0:
    dataRecvd = recvFromArduino()

    if dataRecvd[0] == 0:
      displayDebug(dataRecvd[1])

    if dataRecvd[0] > 0:
      displayData(dataRecvd[1])
      print("Reply Received")
      n += 1
      waitingForReply = False

    print()
    print("===========")
    print()

    time.sleep(0.3)

ser.close

