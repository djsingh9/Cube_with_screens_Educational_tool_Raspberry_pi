"""
*      File Name: cube_with_screens_educational_tool.py
*      Date: 27/02/2016
*
*      Author: Dhanunjaya Singh
*              Varun Tandon
*
*      Email Id: dhanunjay.201889@gmail.com
*
*      Description:
*       1. Cube shaped device with screens on each surface.
*       2. Cube could be used as an gaming device, educational tool, home automation.
*       3. The program is an cube application for educational tool.
*       4. Multiple ILI9341 TFT LCD Screen Interfaced with Raspberry Pi.
*       5. Enhanced IMU Sensor data.
*       6. IMU used to detect shake of the cube for resetting the program.
*       7. Speaker Inerfaced with Raspberry pi for playing Audio Files Stored in SD Card.*
*
*       The author's acknowldege the contribution of the library developers included in this software for this software development.
*
*       The Code below is granted free of charge, to any person obtaining a copy of this software
*       and associated documentation files (the "Software"), to deal in the Software without restriction,
*       including without limitation on the rights to use, copy, modify, merge, publish and distribute.
*
"""

import pygame
import Image
import time
import Adafruit_ILI9341 as TFT
import Adafruit_GPIO as GPIO
import RPi.GPIO as gpio
import Adafruit_GPIO.SPI as SPI
import smbus
import math
import random

face=0
alphabets=0
prevface=0
previous_face=0
levelselect=0
flag_level_disp=0
flag_levelselect=0
flag_firsttime=0
flag_disp_once=0
flag_alphabets_start=0
flag_display_alphabets=0
screenincr_alphabets=1
screenincr_quiz=1
previousscreenincr_alphabets=0
shake=False
face_array=[0 for i in range(5)] # creatting the list
pygame.mixer.init()
select0=0
select1=0
select2=0

DC1= 12
RST1= 23
SPI_PORT = 0
SPI_DEVICE1= 0

DC2= 12
RST2= 13
SPI_PORT = 0
SPI_DEVICE2= 0

DC3= 12
RST3= 26
SPI_PORT = 0
SPI_DEVICE3= 0

DC4= 12
RST4= 24
SPI_PORT = 0
SPI_DEVICE4= 0
'''
DC5= 18
RST5= 12
SPI_PORT = 0
SPI_DEVICE5= 1
'''
################for 
S1=17    #LSB1
S2=27    #MSB1
S3=22    #MSB2

gpio.setmode(gpio.BCM)
gpio.setup(17,gpio.OUT)
gpio.setup(27,gpio.OUT)
gpio.setup(22,gpio.OUT)
gpio.output(17,False)
gpio.output(27,False)
gpio.output(22,False)

disp=[TFT.ILI9341(DC1, rst=RST1, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE1, max_speed_hz=64000000)),TFT.ILI9341(DC2, rst=RST2, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE2, max_speed_hz=64000000)),TFT.ILI9341(DC3, rst=RST3, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE3, max_speed_hz=64000000)),TFT.ILI9341(DC4, rst=RST4, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE4, max_speed_hz=64000000))]

def select_0(k):
    global select0
    
    if(k==0):
        select0=False
        
    elif(k==1):
        select0=True
    
    elif(k==2):
        select0=False
    elif(k==3):
        select0=True
    elif(k==4):
        select0=False
    
    return select0    

def select_1(k):
    global select1
    
    if(k==0):
        select1=False
        
    elif(k==1):
        select1=False
    
    elif(k==2):
        select1=True
    elif(k==3):
        select1=True
    elif(k==4):
        select1=False
    
    return select1

def select_2(k):
    global select2
    
    if(k==0):
        select2=False
        
    elif(k==1):
        select2=False
    
    elif(k==2):
        select2=False
    elif(k==3):
        select2=False
    elif(k==4):
        select2=True
    
    return select2

####################################################################
                #Random Face array sequence generation
####################################################################
def Random_face_array():
 global face_array
 global facerandom_seq
 facerandom_seq=[]                #array for storing the random sequence
 new_face=[]
 k=1                              #for assignning face numbers
 random_variable=random.randint(1,4)
  print'random variable'
 print random_variable
 for i in range(0,4):            #Creating random variable array
    if(random_variable>4):
     random_variable=random_variable-4
    facerandom_seq.append(random_variable)
    random_variable=random_variable+1
 for v in facerandom_seq:       #a[facerandom_seq]=counter++ counter=1,2,3,4,5
    face_array[v]=k
    k=k+1
 for m in range(1,5):        
    print face_array[m]         #face_array to be assigned to the face  

########################End of Face array sequence generation#######

####################################################################
                        #IMU Start
####################################################################
def int_sw(x):
    return x - 0xffff if x > 0x7fff else x

class SensorADXL345(object):
    def __init__(self, bus_nr, addr):
        self.bus = smbus.SMBus(bus_nr)
        self.addr = addr

    def standby(self, stdby = True):
        self.bus.write_byte_data(self.addr, 0x2d, 0x08 if stdby else 0)

    def data_format(self, fullres, range):
        range_bits = {2: 0, 4: 1, 8: 2, 16: 3}
        fr_bit = 8 if fullres else 0
        self.bus.write_byte_data(self.addr, 0x31, fr_bit | range_bits[range])

    def output_data_rate(self, rate, low_power = False):
        lp_bit = 16 if low_power else 0
        if not (rate >= 0 and rate <= 0xf):
            raise ValueError("Invalid output data rate code.")
        self.bus.write_byte_data(self.addr, 0x2c, rate | lp_bit)
        
    def default_init(self):
        self.data_format(False, 2)
        self.output_data_rate(0xA)
        self.standby(False)

    def read_data(self):
        ax = int_sw(self.bus.read_word_data(self.addr, 0x32))
        ay = int_sw(self.bus.read_word_data(self.addr, 0x34))
        az = int_sw(self.bus.read_word_data(self.addr, 0x36))
        return (ax, ay, az)
#######################################
# select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)
x=0
y=0
z=0
# ADXL345 constants
EARTH_GRAVITY_MS2   = 9.80665
SCALE_MULTIPLIER    = 0.004

DATA_FORMAT         = 0x31
BW_RATE             = 0x2C
POWER_CTL           = 0x2D

BW_RATE_1600HZ      = 0x0F
BW_RATE_800HZ       = 0x0E
BW_RATE_400HZ       = 0x0D
BW_RATE_200HZ       = 0x0C
BW_RATE_100HZ       = 0x0B
BW_RATE_50HZ        = 0x0A
BW_RATE_25HZ        = 0x09

RANGE_2G            = 0x00
RANGE_4G            = 0x01
RANGE_8G            = 0x02
RANGE_16G           = 0x03

MEASURE             = 0x08
AXES_DATA           = 0x32

class ADXL345:

    address = None

    def __init__(self, address = 0x53):        
        self.address = address
        self.setBandwidthRate(BW_RATE_100HZ)
        self.setRange(RANGE_2G)
        self.enableMeasurement()

    def enableMeasurement(self):
        bus.write_byte_data(self.address, POWER_CTL, MEASURE)

    def setBandwidthRate(self, rate_flag):
        bus.write_byte_data(self.address, BW_RATE, rate_flag)

    # set the measurement range for 10-bit readings
    def setRange(self, range_flag):
        value = bus.read_byte_data(self.address, DATA_FORMAT)

        value &= ~0x0F;
        value |= range_flag;  
        value |= 0x08;

        bus.write_byte_data(self.address, DATA_FORMAT, value)
    
    # returns the current reading from the sensor for each axis
    #
    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs

    def getAxes(self, gforce = False):
        global x
        global y
        global z
        bytes = bus.read_i2c_block_data(self.address, AXES_DATA, 6)
        
        x = bytes[0] | (bytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = bytes[2] | (bytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = bytes[4] | (bytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)

        x = x * SCALE_MULTIPLIER 
        y = y * SCALE_MULTIPLIER
        z = z * SCALE_MULTIPLIER

        if gforce == False:
            x = x * EARTH_GRAVITY_MS2
            y = y * EARTH_GRAVITY_MS2
            z = z * EARTH_GRAVITY_MS2

        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)

        return {"x": x, "y": y, "z": z}

####################################################################
                        #shake detect
####################################################################
def shake_detect():
     global shake
     #global shake_times
     if __name__ == "__main__":
    # if run directly we'll just create an instance of the class and output 
    # the current readings
      adxl345 = ADXL345()
    
     axes = adxl345.getAxes(True)
     #print "ADXL345 on address 0x%x:" % (adxl345.address)
     #print "   x = %.3fG" % ( axes['x'] )
     #print "   y = %.3fG" % ( axes['y'] )
     #print "   z = %.3fG" % ( axes['z'] )
     accx=abs(x)
     accy=abs(y)
     accz=abs(z)
     if(accx>=1.3 or accy>=1.3 or accz>=1.3 ):
      print '#######################################'
      #shake_times=shake_times+1
      print  'shake detected'
      #print shake_times
      print '#######################################'    
      shake=True
     return shake

###############################################
           #Play kannadaletters
###############################################
def play_kannada():
    global screenincr_alphabets
    global alphabets
    global face
    global previous_face
    global flag_display_alphabets
    global flag_firsttime
    global flag_alphabets_start
    global flag_disp_once
    nextf=face-previous_face
    print'alphabets'
    print alphabets
    print 'nextf'
    print nextf
    if(face==1 and nextf==1):
     alphabets=alphabets+1
    if(face==2 and nextf==1):
     alphabets=alphabets+1
    if(face==3 and nextf==1):
     alphabets=alphabets+1
     '''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
'''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
     screenincr_alphabets=screenincr_alphabets+1
     Random_face_array()
     flag_firsttime=0
     flag_alphabets_start=0
     flag_disp_once=0
     #face=0

    if(nextf>1 or nextf<0):
     face=previous_face
     pygame.mixer.music.load("pleasetryagain.wav")
     pygame.mixer.music.play()
     time.sleep(2)

    if(nextf==1):

     if(alphabets==1):
      pygame.mixer.music.load("kn1.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==2):
      pygame.mixer.music.load("kn2.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==3):
      pygame.mixer.music.load("kn3.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==4):
      pygame.mixer.music.load("kn4.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==5):
      pygame.mixer.music.load("kn5.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==6):
      pygame.mixer.music.load("kn6.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==7):
      pygame.mixer.music.load("kn7.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==8):
      pygame.mixer.music.load("kn8.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==9):
      pygame.mixer.music.load("kn9.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==10):
      pygame.mixer.music.load("kn10.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==11):
      pygame.mixer.music.load("kn11.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==12):
      pygame.mixer.music.load("knl2.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==13):
      pygame.mixer.music.load("kn13.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==14):
      pygame.mixer.music.load("kn14.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==15):
      pygame.mixer.music.load("kn15.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     else :
          dummy_variable=0

     if(alphabets==16):
      alphabets=0
      face=0
     

###############################################
           #Play numbers
###############################################
def play_numbers():
    global screenincr_alphabets
    global alphabets
    global face
    global previous_face
    global flag_display_alphabets
    global flag_firsttime
    global flag_alphabets_start
    global flag_disp_once
    nextf=face-previous_face
    print'alphabets'
    print alphabets
    print 'nextf'
    print nextf
    if(face==1 and nextf==1):
     alphabets=alphabets+1
    if(face==2 and nextf==1):
     alphabets=alphabets+1
    if(face==3 and nextf==1):
     alphabets=alphabets+1
     '''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
'''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
     screenincr_alphabets=screenincr_alphabets+1
     Random_face_array()
     flag_firsttime=0
     flag_alphabets_start=0
     flag_disp_once=0
     #face=0

    if(nextf>1 or nextf<0):
     face=previous_face
     pygame.mixer.music.load("pleasetryagain.wav")
     pygame.mixer.music.play()
     time.sleep(2)

    if(nextf==1):

     if(alphabets==1):
      pygame.mixer.music.load("1.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==2):
      pygame.mixer.music.load("2.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==3):
      pygame.mixer.music.load("3.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==4):
      pygame.mixer.music.load("4.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==5):
      pygame.mixer.music.load("5.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==6):
      pygame.mixer.music.load("6.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==7):
      pygame.mixer.music.load("7.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==8):
      pygame.mixer.music.load("8.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==9):
      pygame.mixer.music.load("9.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==10):
      pygame.mixer.music.load("10.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     else :
          dummy_variable=0

     if(alphabets==20):
      alphabets=0
      face=0
     
###############################################
           #Play alphabets
###############################################
def play_alphabets():
    global screenincr_alphabets
    global alphabets
    global face
    global previous_face
    global flag_display_alphabets
    global flag_firsttime
    global flag_alphabets_start
    global flag_disp_once
    nextf=face-previous_face
    print'alphabets'
    print alphabets
    print 'nextf'
    print nextf
    if(face==1 and nextf==1):
     alphabets=alphabets+1

    if(face==2 and nextf==1):
     alphabets=alphabets+1

    if(face==3 and nextf==1):
     alphabets=alphabets+1

     '''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
'''
    if(face==4 and nextf==1):
     alphabets=alphabets+1
     screenincr_alphabets=screenincr_alphabets+1
     Random_face_array()
     flag_firsttime=0
     flag_alphabets_start=0
     flag_disp_once=0
     #face=0

    if(nextf>1 or nextf<0):
     face=previous_face
     pygame.mixer.music.load("pleasetryagain.wav")
     pygame.mixer.music.play()
     time.sleep(2)

    if(nextf==1):
     if(alphabets==1):
      pygame.mixer.music.load("a.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==2):
      pygame.mixer.music.load("b.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==3):
      pygame.mixer.music.load("c.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==4):
      pygame.mixer.music.load("d.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==5):
      pygame.mixer.music.load("e.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==6):
      pygame.mixer.music.load("f.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==7):
      pygame.mixer.music.load("g.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==8):
      pygame.mixer.music.load("h.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==9):
      pygame.mixer.music.load("i.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==10):
      pygame.mixer.music.load("j.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==11):
      pygame.mixer.music.load("k.wav")
      pygame.mixer.music.play()
      time.sleep(1)
     elif(alphabets==12):
      pygame.mixer.music.load("l.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==13):
      pygame.mixer.music.load("m.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==14):
      pygame.mixer.music.load("n.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==15):
      pygame.mixer.music.load("o.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==16):
      pygame.mixer.music.load("p.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==17):
      pygame.mixer.music.load("q.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==18):
      pygame.mixer.music.load("r.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==19):
      pygame.mixer.music.load("s.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==20):
      pygame.mixer.music.load("t.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==21):
      pygame.mixer.music.load("u.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==22):
      pygame.mixer.music.load("v.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==23):
      pygame.mixer.music.load("w.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==24):
      pygame.mixer.music.load("x.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     elif(alphabets==25):
      pygame.mixer.music.load("y.wav")
      pygame.mixer.music.play()
      flag_display_alphabets=0
      time.sleep(1)

     elif(alphabets==26):
      pygame.mixer.music.load("z.wav")
      pygame.mixer.music.play()
      time.sleep(1)

     else :
          dummy_variable=0

     if(alphabets==26):
      alphabets=0
      face=0
###############################################################
            #Program for display of numbers
###############################################################
def disp_numbers():
 global flag_disp_once
 global screenincr_alphabets
 global previousscreenincr_alphabets
 global facerandom_seq
 global disp
 global flag_display_alphabets
 print '###########################'
 print screenincr_alphabets
 print '###########################'
 print'flag_display'
 print flag_disp_once
 #if (previousscreenincr_alphabets-screenincr_alphabets!=0):
 if (flag_disp_once==0):    
  #for i in range(0,5):
    k0=facerandom_seq[0]
    k0=k0-1
    k1=facerandom_seq[1]
    k1=k1-1
    k2=facerandom_seq[2]
    k2=k2-1
    k3=facerandom_seq[3]
    k3=k3-1
    '''
    k4=facerandom_seq[4]
    k4=k4-1
    '''
    flag_disp_once=1
    if(screenincr_alphabets==1):
             print 'first########################'
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('1.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('2.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('3.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)
             
             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('4.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
          
    if(screenincr_alphabets==2):
             print 'second########################'

             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k4))
             disp[k0].begin()
             image1= Image.open('5.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('6.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
         
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('7.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)
             
             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            # gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('8.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
         
    if(screenincr_alphabets==3):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('9.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('10.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
                 
###############################################################
             #Program for displaying of kannada letters
###############################################################
def disp_kannada():
 global flag_disp_once
 global screenincr_alphabets
 global previousscreenincr_alphabets
 global facerandom_seq
 global disp
 global flag_display_alphabets
 print '###########################'
 print screenincr_alphabets
 print '###########################'
 print'flag_display'
 print flag_disp_once
 #if (previousscreenincr_alphabets-screenincr_alphabets!=0):
 if (flag_disp_once==0):    
  #for i in range(0,5):
    k0=facerandom_seq[0]
    k0=k0-1
    k1=facerandom_seq[1]
    k1=k1-1
    k2=facerandom_seq[2]
    k2=k2-1
    k3=facerandom_seq[3]
    k3=k3-1
    '''
    k4=facerandom_seq[4]
    k4=k4-1
    '''
    flag_disp_once=1
    if(screenincr_alphabets==1):
             print 'first########################'
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('kn1.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('kn2.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
         
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('kn3.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('kn4.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
          
    if(screenincr_alphabets==2):
             print 'second########################'

             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k4))
             disp[k0].begin()
             image1= Image.open('kn5.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('kn6.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('kn7.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            # gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('kn8.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
             
    if(screenincr_alphabets==3):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('kn9.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('kn10.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('kn11.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)
             
             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            # gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('knl2.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
         
    if(screenincr_alphabets==4):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('kn13.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('kn14.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
         
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('kn15.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)
             
             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
             #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('kn16.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
             
###############################################################
            #Program for display of alphabets
###############################################################
def disp_alphabets():
 global flag_disp_once
 global screenincr_alphabets
 global previousscreenincr_alphabets
 global facerandom_seq
 global disp
 global flag_display_alphabets
 print '###########################'
 print screenincr_alphabets
 print '###########################'
 print'flag_display'
 print flag_disp_once
 #if (previousscreenincr_alphabets-screenincr_alphabets!=0):
 if (flag_disp_once==0):    
  #for i in range(0,5):
    k0=facerandom_seq[0]
    k0=k0-1
    k1=facerandom_seq[1]
    k1=k1-1
    k2=facerandom_seq[2]
    k2=k2-1
    k3=facerandom_seq[3]
    k3=k3-1
    '''
    k4=facerandom_seq[4]
    k4=k4-1
    '''
    flag_disp_once=1
    if(screenincr_alphabets==1):
             print 'first########################'
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('a.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('b.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('c.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('d.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
             
    if(screenincr_alphabets==2):
             print 'second########################'

             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k4))
             disp[k0].begin()
             image1= Image.open('e.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('f.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('g.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)
             
             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            # gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('h.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
             
    if(screenincr_alphabets==3):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('i.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('j.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
         
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('k.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
            # gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('l.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
         
    if(screenincr_alphabets==4):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('m.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
         
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('n.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('o.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
             #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('p.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
         
    if(screenincr_alphabets==5):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('q.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('r.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('s.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
             #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('t.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
             
    if(screenincr_alphabets==6):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('u.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('v.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
            
             gpio.output(S1,select_0(k2))
             gpio.output(S2,select_1(k2))
             #gpio.output(S3,select_2(k2))
             disp[k2].begin()
             image1= Image.open('w.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k2].display(image1)

             gpio.output(S1,select_0(k3))
             gpio.output(S2,select_1(k3))
             #gpio.output(S3,select_2(k3))
             
             disp[k3].begin()
             image1= Image.open('x.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k3].display(image1)
    if(screenincr_alphabets==7):
             gpio.output(S1,select_0(k0))
             gpio.output(S2,select_1(k0))
             #gpio.output(S3,select_2(k0))
             disp[k0].begin()
             image1= Image.open('y.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[k0].display(image1)
             
             gpio.output(S1,select_0(k1))
             gpio.output(S2,select_1(k1))
             #gpio.output(S3,select_2(k1))
             disp[k1].begin()
             image1= Image.open('z.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[k1].display(image1)
def disp_quiz():
 global flag_disp_once
 global screenincr_quiz
 global previousscreenincr
 #global facerandom_seq
 global disp
 global flag_display
 print '###########################'
 print screenincr_quiz
 print '###########################'
 print'flag_display'
 print flag_disp_once
 #if (previousscreenincr_alphabets-screenincr_alphabets!=0):
 if (flag_disp_once==0):    
   #for i in range(0,5):
    '''
    k0=facerandom_seq[0]
    k0=k0-1
    k1=facerandom_seq[1]
    k1=k1-1
    k2=facerandom_seq[2]
    k2=k2-1
    k3=facerandom_seq[3]
    k3=k3-1
    
    k4=facerandom_seq[4]
    k4=k4-1
    '''
    flag_disp_once=1
    if(screenincr_quiz==1):
             print 'first########################'
             gpio.output(S1,False)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k0))
             disp[0].begin()
             image1= Image.open('q1.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[0].display(image1)
         
             gpio.output(S1,True)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k1))
             disp[1].begin()
             image1= Image.open('a11.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[1].display(image1)
            
             gpio.output(S1,False)
             gpio.output(S2,True)
             #gpio.output(S3,select_2(k2))
             disp[2].begin()
             image1= Image.open('a13.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[2].display(image1)

             gpio.output(S1,True)
             gpio.output(S2,True)
            #gpio.output(S3,select_2(k3))
             
             disp[3].begin()
             image1= Image.open('a12.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[3].display(image1)
    if(screenincr_quiz==2):
             print 'first########################'
             gpio.output(S1,False)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k0))
             disp[0].begin()
             image1= Image.open('q2.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[0].display(image1)
         
             gpio.output(S1,True)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k1))
             disp[1].begin()
             image1= Image.open('a21.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[1].display(image1)
         
             gpio.output(S1,False)
             gpio.output(S2,True)
             #gpio.output(S3,select_2(k2))
             disp[2].begin()
             image1= Image.open('a22.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[2].display(image1)

             gpio.output(S1,True)
             gpio.output(S2,True)
            #gpio.output(S3,select_2(k3))
             
             disp[3].begin()
             image1= Image.open('a23.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[3].display(image1)
             
    if(screenincr_quiz==3):
             print 'first########################'
             gpio.output(S1,False)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k0))
             disp[0].begin()
             image1= Image.open('q3.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[0].display(image1)
         
             gpio.output(S1,True)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k1))
             disp[1].begin()
             image1= Image.open('a31.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[1].display(image1)
         
             gpio.output(S1,False)
             gpio.output(S2,True)
             #gpio.output(S3,select_2(k2))
             disp[2].begin()
             image1= Image.open('a32.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[2].display(image1)

             gpio.output(S1,True)
             gpio.output(S2,True)
            #gpio.output(S3,select_2(k3))
             
             disp[3].begin()
             image1= Image.open('a33.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[3].display(image1)
             
    if(screenincr_quiz==4):
             print 'first########################'
             gpio.output(S1,False)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k0))
             disp[0].begin()
             image1= Image.open('q4.jpg')
             image1 = image1.rotate(90).resize((240, 320))
             disp[0].display(image1)
         
             gpio.output(S1,True)
             gpio.output(S2,False)
             #gpio.output(S3,select_2(k1))
             disp[1].begin()
             image1= Image.open('a41.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[1].display(image1)
         
             gpio.output(S1,False)
             gpio.output(S2,True)
             #gpio.output(S3,select_2(k2))
             disp[2].begin()
             image1= Image.open('a42.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[2].display(image1)
             
             gpio.output(S1,True)
             gpio.output(S2,True)
            #gpio.output(S3,select_2(k3))
             
             disp[3].begin()
             image1= Image.open('a43.jpg')
             image1= image1.rotate(90).resize((240, 320))
             disp[3].display(image1)
def play_quiz():
      
      global screenincr_quiz
      global flag_disp_once
      if(screenincr_quiz==1):
       if(face==3):
        pygame.mixer.music.load("correct.wav")
        pygame.mixer.music.play()
        screenincr_quiz=screenincr_quiz+1
        flag_disp_once=0
        time.sleep(1)
        '''
       elif(face !=3 and face!=0):
         pygame.mixer.music.load("pleasetryagain.wav")
         pygame.mixer.music.play()
         time.sleep(2)
         '''
      if(screenincr_quiz==2):
       if(face==4):
        pygame.mixer.music.load("correct.wav")
        pygame.mixer.music.play()
        screenincr_quiz=screenincr_quiz+1
        flag_disp_once=0
        time.sleep(1)
        '''
       elif(face !=4 and face!=0):
         pygame.mixer.music.load("pleasetryagain.wav")
         pygame.mixer.music.play()
         flag_disp_once=0
         time.sleep(2)
         '''
      if(screenincr_quiz==3):
       if(face==2):
        pygame.mixer.music.load("correct.wav")
        pygame.mixer.music.play()
        screenincr_quiz=screenincr_quiz+1
        flag_disp_once=0
        time.sleep(1)
        '''
       elif(face !=2 and face!=0):
         pygame.mixer.music.load("pleasetryagain.wav")
         pygame.mixer.music.play()
         time.sleep(2)
         '''
      if(screenincr_quiz==4):
       if(face==3):
        pygame.mixer.music.load("correct.wav")
        pygame.mixer.music.play()
        screenincr_quiz=screenincr_quiz+1
        time.sleep(1)
        '''
       elif(face !=3 and face!=0):
         pygame.mixer.music.load("pleasetryagain.wav")
         pygame.mixer.music.play()
         time.sleep(2)
        '''  
def disp_levels():
    
      #Random_face_array()
      gpio.output(S1,False)
      gpio.output(S2,False)
      #gpio.output(S3,select_2(k0))
      disp[0].begin()
      image1= Image.open('alphabets.jpg')
      image1 = image1.rotate(90).resize((240, 320))
      disp[0].display(image1)
             
      gpio.output(S1,True)
      gpio.output(S2,False)
      #gpio.output(S3,select_2(k1))
      disp[1].begin()
      image1= Image.open('kannada.jpg')
      image1= image1.rotate(90).resize((240, 320))
      disp[1].display(image1)
         
      gpio.output(S1,False)
      gpio.output(S2,True)
      #gpio.output(S3,select_2(k2))
      disp[2].begin()
      image1= Image.open('numbers.jpg')
      image1= image1.rotate(90).resize((240, 320))
      disp[2].display(image1)

      gpio.output(S1,True)
      gpio.output(S2,True)
      #gpio.output(S3,select_2(k3))
             
      disp[3].begin()
      image1= Image.open('quiz.jpg')
      image1= image1.rotate(90).resize((240, 320))
      disp[3].display(image1)
            
###############################################################
Random_face_array()

while(True):
  global shake
  global previous_face
  global face
  global alphabets
  global previous_face
  global levelselect
  global flag_level_disp
  global flag_levelselect
  global flag_firsttime
  global flag_disp_once
  global flag_alphabets_start
  global flag_display_alphabets
  global screenincr_alphabets
  global previousscreenincr_alphabets
  global shake
     
  if __name__ == '__main__':
    #import time
    sensor = SensorADXL345(1, 0x53)
    sensor.default_init()
    time.sleep(0.1)
    ax, ay, az = sensor.read_data()
    sensor.standby()
        
###################IMU Face Caliberation#######################
    xrad = math.atan(ax/(math.sqrt((ay*ay)+(az*az))+180))
    xang = xrad*(180/3.142)
    yrad = math.atan(ay/(math.sqrt((ax*ax)+(az*az))+180))
    yang = yrad*(180/3.142)
    zrad = math.atan((az)/math.sqrt((ay*ay)+(ax*ax)+180))
    zang = zrad*(180/3.142)
    #########################################
    print xang,yang
    ########################################
 
    #if zang>=83.00 and zang<=88.00:
        #face=1
        #face=face_array[1]
    if xang>=-57.00 and xang<=-47.00:
        #face=2
        #facesecondproto=4
        #face=face_array[1]
        
        if(flag_levelselect==0):
         levelselect=4
         flag_levelselect=1
        face=face_array[4]
        if(levelselect==4):
          face=4

    if xang>=48.00 and xang<=57.00:
        #face=3
         #facesecondproto=2
        if(flag_levelselect==0):
          levelselect=2
          flag_levelselect=1
        face=face_array[2]
        if(levelselect==4):
          face=2

    if yang>=46.00 and yang<=54.00:
        #face=4
        #facesecondproto=1
        #face=face_array[3]
        if(flag_levelselect==0):
          levelselect=1
          flag_levelselect=1
        face=face_array[1]
        if(levelselect==4):
          face=1

    if yang>=-58.00 and yang<=-53.00:
        #face=5
        #facesecondproto=3
        #face=face_array[4]
        if(flag_levelselect==0):
          levelselect=3
          flag_levelselect=1
        face=face_array[3]
        if(levelselect==4):
          face=3

    '''
    if zang>=-87.00 and zang<=-83.00:
        facdisp_alphabets()e=6
    '''
    print 'face'
    print face
    
   #flag_firsttime=flag_firsttime+1
    shake_detect()
    if(shake==True):
      
       face=0
       alphabets=0
       previous_face=0
       levelselect=0
       flag_level_disp=0
       flag_levelselect=0
       flag_firsttime=0
       flag_disp_once=0
       flag_alphabets_start=0
       flag_display_alphabets=0
       screenincr_alphabets=1
       previousscreenincr_alphabets=0
       shake=False
       #face_array=[0 for i in range(5)] # craeting the list
    
    if(flag_levelselect==0 and flag_level_disp==0 ):
      disp_levels()
      flag_level_disp=1

    if(levelselect==1):
        print('alphabets')
        disp_alphabets()
        ##############
        if(face==1 and flag_firsttime==0):
            flag_alphabets_start=1
            flag_firsttime=1
            previous_face=0
        if(flag_alphabets_start==1):    
            play_alphabets()
        
    if(levelselect==3):
        print('numbers')
        disp_numbers()
        if(face==1 and flag_firsttime==0):
            flag_alphabets_start=1
            flag_firsttime=1
            previous_face=0
           
        if(flag_alphabets_start==1): 
            play_numbers()

    if(levelselect==2):
        print('kannada')
        disp_kannada()
        if(face==1 and flag_firsttime==0):
            flag_alphabets_start=1
            flag_firsttime=1
            previous_face=0
           
        if(flag_alphabets_start==1): 
            play_kannada()
             
    if(levelselect==4):
        print('quiz')
        disp_quiz()
        play_quiz()
  previous_face=face
  previousscreenincr_alphabets=screenincr_alphabets

##################################################################
                      #End
##################################################################
