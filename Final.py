import math
import operator
import os
import sys
import time
from datetime import datetime
from signal import SIGHUP, SIGTERM, pause, signal

import adafruit_ads1x15.ads1115 as ADS
import adafruit_adxl34x
import adafruit_dht
import board
import busio
import I2C_LCD_driver
import requests
import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn
from colorzero import Color
from gpiozero import LED, Button, Buzzer, DistanceSensor
from rpi_lcd import LCD

"""
LCD

VSS -> GND
VDD -> 5V
V0  -> GND
RS  -> GPIO 4  / PIN 7
RW  -> GND
E   -> GPIO 17 / PIN 11
D4  -> GPIO 18 / PIN 12
D5  -> GPIO 22 / PIN 15
D6  -> GPIO 23 / PIN 16
D7  -> GPIO 24 / PIN 18
A   -> POTENTIOMETER
K   -> GND

------------------------------

HC-SR04
VCC  -> 5V pin
TRIG -> GPIO 26 / PIN 37
ECHO -> to 1K ohm -> GPIO19 / PIN35 -> 2K ohm -> GND
GND  -> GND

---------------------------------------------------------

DHT22
Middle Pin -> GPIO 20 / PIN 38
Left Pin   -> 5V
Right Pin  -> GND
(CHECK THE BACK FOR + & - SYMBOLS)

---------------------------------------------------------


LED Blue
Short Leg -> GND
Long Leg ->  Resistor -> GPIO 5 / PIN 29

---------------------------------------------------------

LED Green
Short Leg -> GND
Long Leg-> Resistor -> GPIO 6 / PIN 31


---------------------------------------------------------

LED Red
Short Leg -> GND
Long Leg -> Resistor -> GPIO 12 / PIN 32

---------------------------------------------------------

ADXL
GND -> GND
VCC -> 5V
SDO -> GPIO 2 / PIN 3
SCL -> GPIO3 / PIN 5

"""


# Pins Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
buzzer = Buzzer(16)  # Buzzer
button = Button(13)  # Button
blueLED = LED(5)  # LED
greenLED = LED(6)  # LED
redLED = LED(12)  # LED
lcd = LCD()

# Sensors Setup
# Distance Sensor HC-SR04
sensor = DistanceSensor(echo=19, trigger=26, max_distance=5)
# DHT22 Temperatur & Humidity
dhtDevice = adafruit_dht.DHT22(board.D20, use_pulseio=False)
i2c = busio.I2C(board.SCL, board.SDA)  # Initaliziation of i2c Protocol
accelerometer = adafruit_adxl34x.ADXL345(i2c)  # ADxl 355 Accelerometer
# Analog-To-Digtal Converter -> For the MQ135 Gas Sensor
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)


# Influx DataBase Setup
utcnow = datetime.now()
timestamp = utcnow.strftime("%s")
t1 = timestamp + '000000000'
url = 'http://localhost:8086/write'
dbname = 'home'
user = 'grfana'
passwd = '123456'
measurement = 'temperature'
measurement2 = 'humidity'
measurement3 = 'Distance'
measurement4 = 'gas'
measurement5 = 'falldetection'
params = {'db': dbname, 'u': user, 'p': passwd}


# The load resistance on the board
RLOAD = 10.0
# Calibration resistance at atmospheric CO2 level
RZERO = 76.63
# Parameters for calculating ppm of CO2 from sensor resistance
PARA = 116.6020682
PARB = 2.769034857

# Parameters to model temperature and humidity dependence
CORA = 0.00035
CORB = 0.02718
CORC = 1.39538
CORD = 0.0018
CORE = -0.003333333
CORF = -0.001923077
CORG = 1.130128205

# Atmospheric CO2 level for calibration purposes
ATMOCO2 = 415.26



def getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG):
	# Linearization of the temperature dependency curve under and above 20 degree C
	# below 20degC: fact = a * t * t - b * t - (h - 33) * d
	# above 20degC: fact = a * t + b * h + c
	# this assumes a linear dependency on humidity
	if t < 20:
		return CORA * t * t - CORB * t + CORC - (h-33.)*CORD
	else:
		return CORE * t + CORF * h + CORG

"""
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
"""

def getResistance(value_pin,RLOAD):
	return ((1023./value_pin) - 1.)*RLOAD

"""
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
"""

def getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD):
	return getResistance(value_pin,RLOAD) / getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
"""

def getPPM(PARA,RZERO,PARB,value_pin,RLOAD):
	return PARA * math.pow((getResistance(value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
"""

def getCorrectedPPM(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB):
	return PARA * math.pow((getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
"""

def getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB):
	return getResistance(value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
"""

def getCorrectedRZero(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB):
	return getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, 
a value of fromHigh to toHigh, values in-between to values in-between, etc.

# Arduino: (0 a 1023)
# Raspberry Pi: (0 a 26690)

More Info: https://www.arduino.cc/reference/en/language/functions/math/map/
"""

def map(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# HC-SR04 Sensor Method
def distance():
    distance = sensor.distance*100
    distance = round(distance, 1)
    if distance < 75:
        distanceAlarm()

    return distance
# DHT22 Sensor Method -> Returns String with Temperature & Humidity


def dht22():
    try:
        # Print the values to the serial port
        temperature_c = round(dhtDevice.temperature,1)
        humidity = round(dhtDevice.humidity,1)
        if temperature_c > 45 or humidity > 70:
            temp_and_humidity()

        return str(temperature_c) + "C H="+str(humidity) + "%"
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        # print(error.args[0])
        time.sleep(2.0)
    except Exception as error:
        dhtDevice.exit()
        raise error


# Temperature Method -> just like DHT22 Method but returns only Temperature data
def Temperature():
    try:
        # Print the values to the serial port
        temperature_c = dhtDevice.temperature
        return round(temperature_c,1)
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        # print(error.args[0])
        time.sleep(2.0)
    except Exception as error:
        dhtDevice.exit()
        raise error

# Humidity Method -> just like DHT22 Method but returns only Humidity data


def Humidity():
    try:
        # Print the values to the serial port
        humidity = dhtDevice.humidity
        return round(humidity,1)
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        # print(error.args[0])
        time.sleep(2.0)
    except Exception as error:
        dhtDevice.exit()
        raise error


# ADxL 355 Method
def Adxl():
   # print(" %f %f %f" % accelerometer.acceleration)
    accelerometer.enable_freefall_detection(threshold=10, time=2)

    return accelerometer.events["freefall"]


# Alarm For Distance
# if x = 1 Soft Alarm else Hard Alarm
def distanceAlarm():
    blueLED.blink(0.5,  0.5, 5, True)

# Alarm For DHT22


def temp_and_humidity():
    greenLED.blink(0.5,  0.5, 5, True)

def gasAlarm():
    redLED.blink(0.5,  0.5, 5, True)
# Alarm For Accelerometer

dht22()
lcd.text("Welcome to the" ,1 )
lcd.text("Instructions", 2)
time.sleep(2)
lcd.text("Red is for",1)
lcd.text("Gas Alarm",2)
time.sleep(2)
lcd.text("Green is for",1)
lcd.text("Distance Alarm",2)
time.sleep(2)
lcd.text("Blue is for Temp",1)
lcd.text("Humid Alarm",2)
time.sleep(2)
# Method To Display the Message on LCD
Gadxl = None
Badxl = 0
TEMP = "0C H=0%"
TempT= 23
TempH = 25
while True:
    if isinstance(Gadxl, int) and Gadxl < 50:
       Gadxl += 1
    elif isinstance(Gadxl, int) and Gadxl == 50:
       Gadxl = None
       Badxl = 0
    T = Temperature()
    H = Humidity()
    if (T == None):
       T = TempT
    else:
       TempT = T
    if(H == None):
        H = TempH
    else:
       TempH = H
    value_ads = value_ads = chan.value # value obtained by ADS1115
    value_pin = map((value_ads - 565), 0, 26690, 0, 1023) # 565 / 535 fix value
    rzero = getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB)
    correctedRZero = getCorrectedRZero(TempT,TempH,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB)
    resistance = getResistance(value_pin,RLOAD)
    ppm = getPPM(PARA,RZERO,PARB,value_pin,RLOAD)
    correctedPPM = getCorrectedPPM(TempT,TempH,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB)
    correctedPPM = round(correctedPPM,1)
    if(correctedPPM > 1000):
        gasAlarm()
    dht = dht22()
    dis = distance()
    adxl = Adxl()
    if adxl:
        Gadxl = 0
        Badxl = 1
    if(str(dis) == "500.0"):
        display1 =  ("Dis= T="+str(TempT))
    else:
        display1 = ("Dis="+str(dis)+" T="+str(TempT))
    display2 = ("PPM="+str(correctedPPM)+" H="+str(TempH))
    lcd.text(display1, 1)
    lcd.text(display2, 2)
    #print("\t MQ135 RZero: %s" % round(rzero))
    #print("\t Corrected RZero: %s" % round(correctedRZero))
    #print("\t Resistance: %s" % round(resistance))
    #print("\t PPM: %s" % round(ppm))
    #print("\t Corrected PPM: %s ppm" % round(correctedPPM))
    payload = "%s,location=%s value=%s %s\n" % (
        measurement, 'living_room',TempT, t1)
    payload += "%s,location=%s value=%s %s\n" % (
        measurement2, 'living_room', TempH, t1)
    payload += "%s,location=%s value=%s %s\n" % (
        measurement3, 'living_room', dis, t1)
    payload += "%s,location=%s value=%s %s\n" % (
        measurement4, 'living_room', correctedPPM, t1)
    payload += "%s,location=%s value=%s %s\n" % (
        measurement5, 'living_room', Badxl, t1)
    r = requests.post(url, params=params, data=payload)
#    print(r)
#    print(payload) 
    time.sleep(0.25)
