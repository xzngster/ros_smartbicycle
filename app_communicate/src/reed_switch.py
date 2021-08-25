#!/usr/bin/env python3
import spidev
import time
from datetime import datetime
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
spi=spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1350000

radius = 30
circle = (2*radius*3.14)/100
#remain =0
ckCheck = 0
uckCheck = 0
bySpeed = 0
ckTime = 0
ucTimek = 0
runTime = 0
distance = 0
count = 0
time1 = 1
check = 0

def analog_read(channel):
        r=spi.xfer2([1,(8+channel)<<4,0])
        adc_out=((r[1]&3)<<8)+r[2]
        return adc_out

reading = analog_read(1)
ckCheck = reading

st1 = time.strftime('%H')
st2 = time.strftime('%M')
st3 = time.strftime('%S')
st1 = int(st1)
st2 = int(st2)
st3 = int(st3)

while True:
   reading=analog_read(1)
   #voltage=reading*3.3/1024
       #print(reading)
   time.sleep(0.01)
   showtime= time.strftime('%H : %M : %S')
   uckCheck=reading

   if ckCheck >= 900 and uckCheck <=500:
      check = 0
      ckTime = time.strftime('%S')
      ckTime=float(ckTime)
      time.sleep(0.01)
      uckTime =time.strftime('%S')
      uckTime=float(uckTime)
      count= count + 1
      distance += circle
      #if uckTime<ckTime:
           #   time1 = uckTime +60 - ckTime
      #else :
      #   time1 = uckTime - ckTime
      runTime += 0.01
      bySpeed = distance / runTime

   else:
      check = check + 1

      if check < 1000:
         runTime += 0.01
      if check >= 1000:
         print(time.strftime('%H:%M:%S'))
         et1 = time.strftime('%H')
         et2 = time.strftime('%M')
         et3 = time.strftime('%S')
         et1 = int(et1)
         et2 = int(et2)
         et3 = int(et3)

         if et1 < st1 :
            h = et1+24 - st1
         else :
            h = et1 - st1
         if et2 < st2 :
            m = et2+60 - st2
         else :
            m = et2 - st2
         if et3 < st3 :
            s = et3+60 - st3
         else :
            s = et3 - st3

         finalSpeed = distance / runTime
         print('finalSpeed = %.1f' %finalSpeed)
         print('runTime = %.1f' %runTime)
         print('distance = %.1f' %distance)

         break

   print('=================================')
   print(showtime)
   print('speed = %.1f' %bySpeed)
   print('runTime = %.1f' %runTime)
   print('distance = %.1f' %distance)
   print('=================================\n')
