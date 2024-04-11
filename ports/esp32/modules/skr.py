# MicroPython module for Pitsco SEEKER Middle School Computer Science
# # Written by Paul W. Uttley
# Pitsco Education
# 03/25/2024
# Version 1.0
##################### DO NOT MODIFY THIS LIBRARY #####################

import time
import machine
import neopixel
import random
import math
from dht import DHT11					#Temp and Humidity sensor
from tcs34725 import TCS34725			#Color Sensor
from VL53L0X import VL53L0X				#TOF sensor
from machine import Pin, ADC, SoftI2C, PWM, time_pulse_us, I2C

pixels = neopixel.NeoPixel(machine.Pin(16), 4)

piezo = PWM(Pin(15), freq=440, duty=0)

# this is the internal bus for the rp2040 motor control chip --- DO NOT CHANGE
i2c_mc = I2C(0,scl=Pin(2), sda=Pin(1), freq=400000)

button_start = machine.Pin(21, machine.Pin.IN)
button_a = machine.Pin(12, machine.Pin.IN)
button_b = machine.Pin(14, machine.Pin.IN)
button_c = machine.Pin(17, machine.Pin.IN)
button_d = machine.Pin(18, machine.Pin.IN)

# constants for calculating spin and pivot turns
wheel_diameter = 81   #  mm
wheel_distance = 250  #  mm
encoder_counts_per_rev = 2200  # encoder counts per wheel revolution
circumference = math.pi * wheel_diameter  # Calculate the circumference of each wheel
scale_factor = 1 # applies an error correction factor to calculated encoder_counts for angle spins and pivots


LED_green = machine.Pin(13, machine.Pin.OUT)

LED_green.value(True) #off
drive_invert = False
servo1_invert = False
servo2_invert = False
servo3_invert = False
servo4_invert = False
i2c_pace = 10  #ms pause to pace i2c writes

color_sensor_threshold = 0.1

RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

#NOTES
OFF = 0
B0 = 31
C1 = 33
CS1 = 35
D1 = 37
DS1 = 39
E1 = 41
F1 = 44
FS1 = 46
G1 = 49
GS1 = 52
A1 = 55
AS1 = 58
B1 = 62
C2 = 65
CS2 = 69
D2 = 73
DS2 = 78
E2 = 82
F2 = 87
FS2 = 93
G2 = 98
GS2 = 104
A2 = 110
AS2 = 117
B2 = 123
C3 = 131
CS3 = 139
D3 = 147
DS3 = 156
E3 = 165
F3 = 175
FS3 = 185
G3 = 196
GS3 = 208
A3 = 220
AS3 = 233
B3 = 247
C4 = 262
CS4 = 277
D4 = 294
DS4 = 311
E4 = 330
F4 = 349
FS4 = 370
G4 = 392
GS4 = 415
A4 = 440
AS4 = 466
B4 = 494
C5 = 523
CS5 = 554
D5 = 587
DS5 = 622
E5 = 659
F5 = 698
FS5 = 740
G5 = 784
GS5 = 831
A5 = 880
AS5 = 932
B5 = 988
C6 = 1047
CS6 = 1109
D6 = 1175
DS6 = 1245
E6 = 1319
F6 = 1397
FS6 = 1480
G6 = 1568
GS6 = 1661
A6 = 1760
AS6 = 1865
B6 = 1976
C7 = 2093
CS7 = 2217
D7 = 2349
DS7 = 2489
E7 = 2637
F7 = 2794
FS7 = 2960
G7 = 3136
GS7 = 3322
A7 = 3520
AS7 = 3729
B7 = 3951
C8 = 4186
CS8 = 4435
D8 = 4699
DS8 = 4978



################################################# Sensors

def getTOFSensor(port = 1):
    if (port == 1):
        scl_pin = 4
        sda_pin = 5
    if (port == 2):
        scl_pin = 6
        sda_pin = 7
    if (port == 3):
        scl_pin = 9
        sda_pin = 8
    if (port == 4):
        scl_pin = 10
        sda_pin = 11
    if (port == 5):
        scl_pin = 44
        sda_pin = 43       
    try:
        I2C_tof = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        tof = VL53L0X(I2C_tof)
    except:
        print('I2C bus error or sensor not plugged into port')
    try:
        tof.start()
        distance = tof.read()
        tof.stop()
        distance = (min(max(distance, 30), 1000)) # max range is 30 ~ 1000 mm
        return distance
    except:
        print('error: sensor not detected on port')
        return 0


def getSonicSensor(port=0):
    if port == 1:
        input_pin = 4
    if port == 2:
        input_pin = 6
    if port == 3:
        input_pin = 9
    if port == 4:
        input_pin = 10
    if port == 5:
        input_pin = 44
    if port == 0:
        print("port selection out of range")
        return 0
    ultrasonic_pin = Pin(input_pin, Pin.OUT)
    ultrasonic_pin.value(0)
    time.sleep_us(2)
    ultrasonic_pin.value(1)
    time.sleep_us(5)
    ultrasonic_pin.value(0)
    ultrasonic_pin = Pin(input_pin, Pin.IN)
    time.sleep_us(5)
    pulse_duration = time_pulse_us(ultrasonic_pin, 1, 100000)  # Timeout set to 0.1 second (100000 microseconds)
    if (pulse_duration < 0):
        print ('sensor error, or not plugged into port')
        return 0
    # Calculate distance in centimeters
    distance_cm = (pulse_duration / 2) / 29.1
    distance_round = round(distance_cm, 1)	# one decimal place
    distance_round = (min(max(distance_round, 2), 350)) # max range is ~ 350 cm
    return distance_round


def getLineSensor(port=0):
    if port == 1:
        input_pin = Pin(4, Pin.IN)
    if port == 2:
        input_pin = Pin(6, Pin.IN)
    if port == 3:
        input_pin = Pin(9, Pin.IN)
    if port == 4:
        input_pin = Pin(10, Pin.IN)
    if port == 5:
        input_pin = Pin(44, Pin.IN)
    if port == 0:
        print("port selection out of range")
        return 0
    value = input_pin.value()
    return value	#return sensor logic state 0/1 False/True


def getColorSensor(port=0, level='neutral'):
    if (port == 1):
        scl_pin = 4
        sda_pin = 5
    if (port == 2):
        scl_pin = 6
        sda_pin = 7
    if (port == 3):
        scl_pin = 9
        sda_pin = 8
    if (port == 4):
        scl_pin = 10
        sda_pin = 11
    if (port == 5):
        scl_pin = 44
        sda_pin = 43       
    try:
        I2C_color = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        sensor = TCS34725(I2C_color)
        sensor.gain(16)
    except:
        print('I2C bus error or sensor not plugged into port')
    try:
        red, green, blue, ambient = sensor.read('rgb')
        if level == 'red':
            return red
        elif level == 'green':
            return green
        elif level == 'blue':
            return blue
        elif level == 'ambient':
            sensor.gain(4)  # lower the gain and read again - ambient saturates easily
            red, green, blue, ambient = sensor.read('rgb')
            ambient = (min(max(ambient, 0), 1000))  # trim to 1000
            return ambient
        elif level == 'rgb':
            return red,green,blue
        elif level == 'hex':
            hex_color = rgb_to_hex(red, green, blue) 
            return hex_color
        else:
            return 'none'
    except:
        print('sensor error')
        return 'none'
    

def getColorSensorIsColor(port=0, color = 'neutral', threshold = -1):
    if threshold != -1:
        global color_sensor_threshold
        threshold = (min(max(threshold, 0), 1))  # 0.01 - 1.0
        color_sensor_threshold = threshold
        
    if (port == 1):
        scl_pin = 4
        sda_pin = 5
    if (port == 2):
        scl_pin = 6
        sda_pin = 7
    if (port == 3):
        scl_pin = 9
        sda_pin = 8
    if (port == 4):
        scl_pin = 10
        sda_pin = 11
    if (port == 5):
        scl_pin = 44
        sda_pin = 43       
    try:
        I2C_color = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        sensor = TCS34725(I2C_color)
        sensor.gain(16)
    except:
        print('I2C bus error or sensor not plugged into port')
    try:
        red, green, blue, lux = sensor.read('rgb')
        computed_color = compute_color(red, green, blue)
        if computed_color == color:
            return True
        else:
            return False
    except:
        print('sensor error')
        return 'none'

def getTHSensor(port, measure = 'none', unit = 'none'):
    if port == 1:
        d = DHT11(Pin(4))
    if port == 2:
        d = DHT11(Pin(6))
    if port == 3:
        d = DHT11(Pin(9))
    if port == 4:
        d = DHT11(Pin(10))
    if port == 5:
        d = DHT11(Pin(44))
    if port == 0:
        print("port selection out of range")
        return 0
    
    if measure == 'temp':
        try:   
            d.measure()
            t = d.temperature() # celcius
        except:
            print('sensor error or not plugged into port')
            return 0      
        if unit == 'f':
            t = int((t * 1.8) + 32)  # integer    
        return t	# temperature
    
    if measure == 'humid':
        try:   
            d.measure()
            h = d.humidity()
        except:
            print('sensor error or not plugged into port')
            return 0
        return h	# humidity
    
def getRotarySensor(port = 0):
    if port == 1:
        input_pin = ADC(Pin(4))
        input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
        value = input_pin.read()
        scaled_value = map_value(value, 0, 4095, 100, 0)
        return int(scaled_value) # return 0 - 100
    if port == 2:
        input_pin = ADC(Pin(6))
        input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
        value = input_pin.read()
        scaled_value = map_value(value, 0, 4095, 100, 0)
        return int(scaled_value) # return 0 - 100
    if port == 3:
        input_pin = ADC(Pin(9))
        input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
        value = input_pin.read()
        scaled_value = map_value(value, 0, 4095, 100, 0)
        return int(scaled_value) # return 0 - 100
    if port == 4:
        input_pin = ADC(Pin(10))
        input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
        value = input_pin.read()
        scaled_value = map_value(value, 0, 4095, 100, 0)
        return int(scaled_value) # return 0 - 100
    if port == 5:
        try:
            input_pin = ADC(Pin(44))
        except:
            print('port 5 cannot be used as analog input') #invalid when wireless is on
            return -1
        input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
        value = input_pin.read()
        scaled_value = map_value(value, 0, 4095, 100, 0)
        return int(scaled_value) # return 0 - 100
        
        
'''
analog range is 0 - 4095
'''
def getSensorPortValue(port, AD = 'digital'):
    if port == 1:
        if AD == 'digital':
            input_pin = Pin(4, Pin.IN)
            value = input_pin.value()
            return value # return digital 0/1 False/True
        else:
            input_pin = ADC(Pin(4))
            input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
            value = input_pin.read()
            return value # return analog
    if port == 2:
        if AD == 'digital':
            input_pin = Pin(6, Pin.IN)
            value = input_pin.value()
            return value # return digital 0/1 False/True
        else:
            input_pin = ADC(Pin(6))
            input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
            value = input_pin.read()
            return value # return analog
    if port == 3:
        if AD == 'digital':
            input_pin = Pin(9, Pin.IN)
            value = input_pin.value()
            return value # return digital 0/1 False/True
        else:
            input_pin = ADC(Pin(9))
            input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
            value = input_pin.read()
            return value # return analog
    if port == 4:
        if AD == 'digital':
            input_pin = Pin(10, Pin.IN)
            value = input_pin.value()
            return value # return digital 0/1 False/True
        else:
            input_pin = ADC(Pin(10))
            input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
            value = input_pin.read()
            return value # return analog
    if port == 5:
        if AD == 'digital':
            input_pin = Pin(44, Pin.IN)
            value = input_pin.value()
            return value # return digital 0/1 False/True
        else:
            try:
                input_pin = ADC(Pin(44))
            except:
                print('port 5 cannot be used as analog input')
                return -1
            input_pin.atten(ADC.ATTN_11DB) # Full range: 3.3v
            value = input_pin.read()
            return value # return analog


def setColorSensorThreshold(threshold = 0.05):
    global color_sensor_threshold
    color_sensor_threshold = threshold


def compute_color(red, green, blue):
    # Calculate the total color value
    total_color = red + green + blue

    # Calculate the percentage of each color component
    red_percent = red / total_color
    green_percent = green / total_color
    blue_percent = blue / total_color
    
    # Set the threshold range
    #color_threshold = color_sensor_threshold * total_color
    color_threshold = color_sensor_threshold
    
    # Determine the dominant color
    if red_percent > green_percent + (color_threshold + 0.05) and red_percent > blue_percent + color_threshold:
        return "red"
    elif green_percent > red_percent + color_threshold and green_percent > blue_percent + color_threshold:
        return "green"
    elif blue_percent > red_percent + color_threshold and blue_percent > green_percent + color_threshold:
        return "blue"
    else:
        return "neutral"
    

def map_value(value, from_low, from_high, to_low, to_high):
    """
    Map the given value from one range to another.

    :param value: The value to be mapped.
    :param from_low: The low end of the input range.
    :param from_high: The high end of the input range.
    :param to_low: The low end of the output range.
    :param to_high: The high end of the output range.
    :return: The mapped value.
    """
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def write_reg(register, values):
    i2c_mc.writeto_mem(5, register, bytes(values))
    time.sleep_ms(i2c_pace)
    

def setMotorPower(motor=0, power=0):
    if motor == 1234:
        setMotorPowers(power,power,power,power)
        return

    register = 0
    motor = int(motor)
    power = int(min(max(power, -125), 125))   # constrain power range
    power = max(min(power, 32767), -32768)    # make sure it's within 16-bit signed integer range
    powerH = ((power >> 8) & 0xff)            # negative number takes 2 bytes
    powerL = power & 0xff
        
    if motor == 1:
        register = 47
    if motor == 2:
        register = 48
    if motor == 3:
        register = 49
    if motor == 4:
        register = 50
    
    values = [powerH, powerL]
    write_reg(register, values)
    

def setMotorPowers(power1=0, power2=0, power3=0, power4=0):
    power1 = int(min(max(power1, -125), 125))   # constrain power range
    power2 = int(min(max(power2, -125), 125))   # constrain power range
    power3 = int(min(max(power3, -125), 125))   # constrain power range
    power4 = int(min(max(power4, -125), 125))   # constrain power range
     
    power1H = ((power1 >> 8) & 0xff) # negative number takes 2 bytes
    power1L = power1 & 0xff
    power2H = ((power2 >> 8) & 0xff) # negative number takes 2 bytes
    power2L = power2 & 0xff
    power3H = ((power3 >> 8) & 0xff) # negative number takes 2 bytes
    power3L = power3 & 0xff
    power4H = ((power4 >> 8) & 0xff) # negative number takes 2 bytes
    power4L = power4 & 0xff
    
    values = [power1H, power1L, power2H, power2L, power3H, power3L, power4H, power4L]
    write_reg(51, values)
    
    
def setMotorSpeed(motor=0, speed=0):
    if motor == 1234:
        setMotorSpeeds(speed,speed,speed,speed)
        return

    speed = int(min(max(speed, -100), 100))   # constrain -100 - 100
    speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    
    register = 0
    motor = int(motor)
     
    speedH = ((speed >> 8) & 0xff) # negative number takes 2 bytes
    speedL = speed & 0xff
    
    if motor < 0:
        motor = 0
    if motor > 4:
        motor = 0
        
    if motor == 1:
        register = 52
    if motor == 2:
        register = 53
    if motor == 3:
        register = 54
    if motor == 4:
        register = 55

    values = [speedH, speedL]
    write_reg(register, values)

    
   
def setMotorSpeeds(speed1=0, speed2=0, speed3=0, speed4=0):  
    speed1 = int(min(max(speed1, -100), 100))   # constrain -100 - 100
    speed1 = int(speed1 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed2 = int(min(max(speed2, -100), 100))   # constrain -100 - 100
    speed2 = int(speed2 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed3 = int(min(max(speed3, -100), 100))   # constrain -100 - 100
    speed3 = int(speed3 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed4 = int(min(max(speed4, -100), 100))   # constrain -100 - 100
    speed4 = int(speed4 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    
    speed1H = ((speed1 >> 8) & 0xff) 
    speed1L = speed1 & 0xff
    speed2H = ((speed2 >> 8) & 0xff) 
    speed2L = speed2 & 0xff
    speed3H = ((speed3 >> 8) & 0xff) 
    speed3L = speed3 & 0xff
    speed4H = ((speed4 >> 8) & 0xff) 
    speed4L = speed4 & 0xff
 
    values = [speed1H, speed1L, speed2H, speed2L, speed3H, speed3L, speed4H, speed4L]
    write_reg(56, values)

    
# To Do: Degree accuracy is not working right --- need to fix in FW or in Function    
def setMotorDegree(motor=0, speed=0, degree=0):
    resetEncoder(1234)
    if motor == 1234:
        setMotorDegrees(speed,degree, speed,degree, speed,degree, speed,degree)
        return
    
    speed = abs(speed)
    speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
    speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    invert = 1
    register = 0
    motor = int(motor)
    degree = int(degree)
    
    speedH = ((speed >> 8) & 0xff) # negative number takes 2 bytes
    speedL = speed & 0xff
    
    degreebuff = bytearray(4)
    degreebuff[0] = (degree>>24) & 0xff
    degreebuff[1] = (degree>>16) & 0xff
    degreebuff[2] = (degree>>8)  & 0xff
    degreebuff[3] = degree & 0xff
    
    if motor < 0:
        motor = 0
    if motor > 4:
        motor = 0
        
    if motor == 1:
        register = 82
    if motor == 2:
        register = 83
    if motor == 3:
        register = 84
    if motor == 4:
        register = 85

    values = [speedH, speedL, degreebuff[0], degreebuff[1], degreebuff[2], degreebuff[3]]
    write_reg(register, values)


def setMotorDegrees(speed1=0,degree1=0, speed2=0,degree2=0, speed3=0,degree3=0, speed4=0,degree4=0):  
    speed1 = abs(speed1)
    speed1 = int(min(max(speed1, 0), 100))   # constrain 0 - 100
    speed1 = int(speed1 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed2 = abs(speed2)
    speed2 = int(min(max(speed2, 0), 100))   # constrain 0 - 100
    speed2 = int(speed2 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed3 = abs(speed3)
    speed3 = int(min(max(speed3, 0), 100))   # constrain 0 - 100
    speed3 = int(speed3 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed4 = abs(speed4)
    speed4 = int(min(max(speed4, 0), 100))   # constrain 0 - 100
    speed4 = int(speed4 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
     
    degree1 = int(degree1)
    degree2 = int(degree2)
    degree3 = int(degree3)
    degree4 = int(degree4)
    
    speed1H = ((speed1 >> 8) & 0xff)   # negative number takes 2 bytes
    speed1L = speed1 & 0xff
    speed2H = ((speed2 >> 8) & 0xff)   
    speed2L = speed2 & 0xff
    speed3H = ((speed3 >> 8) & 0xff)   
    speed3L = speed3 & 0xff
    speed4H = ((speed4 >> 8) & 0xff)   
    speed4L = speed4 & 0xff
    
    degreebuff = bytearray(16)
    degreebuff[0] = (degree1>>24) & 0xff
    degreebuff[1] = (degree1>>16) & 0xff
    degreebuff[2] = (degree1>>8)  & 0xff
    degreebuff[3] = degree1 & 0xff
    
    degreebuff[4] = (degree2>>24) & 0xff
    degreebuff[5] = (degree2>>16) & 0xff
    degreebuff[6] = (degree2>>8)  & 0xff
    degreebuff[7] = degree2 & 0xff
    
    degreebuff[8] = (degree3>>24) & 0xff
    degreebuff[9] = (degree3>>16) & 0xff
    degreebuff[10] = (degree3>>8)  & 0xff
    degreebuff[11] = degree3 & 0xff
    
    degreebuff[12] = (degree4>>24) & 0xff
    degreebuff[13] = (degree4>>16) & 0xff
    degreebuff[14] = (degree4>>8)  & 0xff
    degreebuff[15] = degree4 & 0xff

    values = [speed1H, speed1L, degreebuff[0], degreebuff[1], degreebuff[2], degreebuff[3], speed2H, speed2L, degreebuff[4], degreebuff[5], degreebuff[6], degreebuff[7], speed3H, speed3L, degreebuff[8], degreebuff[9], degreebuff[10], degreebuff[11], speed4H, speed4L, degreebuff[12], degreebuff[13], degreebuff[14], degreebuff[15]]
    write_reg(86, values)
  
  
def setDriveSpeed(direction = 'stop', speed = 0):
    speed = int(abs(speed))
 #   speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
 #   speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
 #   Don't need to convert % to dps because we're calling setMotorTarget where it's done
 #   This could change if I put this function directly in FW
    invert = 1
    if drive_invert == True:
        invert = -1
    speed1 = 0
    speed2 = 0
    if direction == 'forward':
        speed1 = -speed * invert
        speed2 = speed * invert
    if direction == 'reverse':
        speed1 = speed * invert
        speed2 = -speed * invert
    if direction == 'pivotR':
        speed1 = -speed
        speed2 = 0
        if invert == -1:
            speed1 = 0
            speed2 = -speed
    if direction == 'pivotL':
        speed1 = 0
        speed2 = speed
        if invert == -1:
            speed1 = speed
            speed2 = 0
    if direction == 'spinR':
        speed1 = -speed
        speed2 = -speed
    if direction == 'spinL':
        speed1 = speed
        speed2 = speed
    #setMotorSpeed(1, speed1)
    #setMotorSpeed(2, speed2)
    setMotorSpeeds(speed1,speed2,0,0)
    
    
def setDriveSpeeds(direction = 'stop', speed1 = 0, speed2 = 0):
    speed1 = int(abs(speed1))
    speed2 = int(abs(speed2))
 #   speed1 = int(min(max(speed1, 0), 100))   # constrain 0 - 100
 #   speed1 = int(speed1 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
 #   speed2 = int(min(max(speed2, 0), 100))   # constrain 0 - 100
 #   speed2 = int(speed2 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
 #   Don't need to convert % to dps because we're calling setMotorTarget where it's done
 #   This could change if I put this function directly in FW
    invert = 1
    if drive_invert == True:
        invert = -1
    if direction == 'forward':
        speed1 = -speed1 * invert
        speed2 = speed2 * invert
    if direction == 'reverse':
        speed1 = speed1 * invert
        speed2 = -speed2 * invert
    #setMotorSpeed(1, speed1)
    #setMotorSpeed(2, speed2)
    setMotorSpeeds(speed1,speed2,0,0)
    

def setDriveDistance(direction = 'stop', speed = 0, distance = 0, wait = 'false'):
    speed = int(abs(speed))
    distance = int(abs(distance * 10))  # change distance to mm
  #  speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
  #  speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
  #  print(speed)
  #  Don't need to convert % to dps because we're calling setMotorTarget where it's done
  #  This could change if I put this function directly in FW
    invert = 1
    if drive_invert == True:
        invert = -1
    
    resetEncoder(1)
    resetEncoder(2)
    
    revolutions = distance / circumference
     
    if direction == 'forward':
        target1 = int(2200 * revolutions) * invert
        target2 = int(2200 * revolutions) * invert
        setMotorTargets(speed,-target1,speed,target2,0,0,0,0)
    if direction == 'reverse':
        target1 = int(2200 * revolutions) * invert
        target2 = int(2200 * revolutions) * invert
        setMotorTargets(speed,target1,speed,-target2,0,0,0,0)
        
    if wait == True:
        setDriveWait()


def setMotorTarget(motor=0, speed=0, target=0):
    if motor == 1234:
        setMotorTargets(speed,target, speed,target, speed,target, speed,target)
        return
    
    speed = abs(speed)
    speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
    speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    
    register = 0
    motor = int(motor)
    target = int(target)
    
    speedH = ((speed >> 8) & 0xff) # negative number takes 2 bytes
    speedL = speed & 0xff
    
    targetbuff = bytearray(4)
    targetbuff[0] = (target>>24) & 0xff
    targetbuff[1] = (target>>16) & 0xff
    targetbuff[2] = (target>>8)  & 0xff
    targetbuff[3] = target & 0xff
    
    if motor < 0:
        motor = 0
    if motor > 4:
        motor = 0
        
    if motor == 1:
        register = 57
    if motor == 2:
        register = 58
    if motor == 3:
        register = 59
    if motor == 4:
        register = 60
        
    values = [speedH, speedL, targetbuff[0], targetbuff[1], targetbuff[2], targetbuff[3]]
    write_reg(register, values)        

          
def setMotorTargets(speed1=0,target1=0, speed2=0,target2=0, speed3=0,target3=0, speed4=0,target4=0):   
    speed1 = abs(speed1)
    speed1 = int(min(max(speed1, 0), 100))   # constrain 0 - 100
    speed1 = int(speed1 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed2 = abs(speed2)
    speed2 = int(min(max(speed2, 0), 100))   # constrain 0 - 100
    speed2 = int(speed2 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed3 = abs(speed3)
    speed3 = int(min(max(speed3, 0), 100))   # constrain 0 - 100
    speed3 = int(speed3 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    speed4 = abs(speed4)
    speed4 = int(min(max(speed4, 0), 100))   # constrain 0 - 100
    speed4 = int(speed4 * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
    
    target1 = int(target1)
    target2 = int(target2)
    target3 = int(target3)
    target4 = int(target4)
    
    speed1H = ((speed1 >> 8) & 0xff)   # negative number takes 2 bytes
    speed1L = speed1 & 0xff
    speed2H = ((speed2 >> 8) & 0xff)   
    speed2L = speed2 & 0xff
    speed3H = ((speed3 >> 8) & 0xff)   
    speed3L = speed3 & 0xff
    speed4H = ((speed4 >> 8) & 0xff)   
    speed4L = speed4 & 0xff
    
    targetbuff = bytearray(16)
    targetbuff[0] = (target1>>24) & 0xff
    targetbuff[1] = (target1>>16) & 0xff
    targetbuff[2] = (target1>>8)  & 0xff
    targetbuff[3] = target1 & 0xff
    
    targetbuff[4] = (target2>>24) & 0xff
    targetbuff[5] = (target2>>16) & 0xff
    targetbuff[6] = (target2>>8)  & 0xff
    targetbuff[7] = target2 & 0xff
    
    targetbuff[8] = (target3>>24) & 0xff
    targetbuff[9] = (target3>>16) & 0xff
    targetbuff[10] = (target3>>8)  & 0xff
    targetbuff[11] = target3 & 0xff
    
    targetbuff[12] = (target4>>24) & 0xff
    targetbuff[13] = (target4>>16) & 0xff
    targetbuff[14] = (target4>>8)  & 0xff
    targetbuff[15] = target4 & 0xff
     
    values = [speed1H, speed1L, targetbuff[0], targetbuff[1], targetbuff[2], targetbuff[3], speed2H, speed2L, targetbuff[4], targetbuff[5], targetbuff[6], targetbuff[7], speed3H, speed3L, targetbuff[8], targetbuff[9], targetbuff[10], targetbuff[11], speed4H, speed4L, targetbuff[12], targetbuff[13], targetbuff[14], targetbuff[15]]
    write_reg(61, values) 

def setWheelDistance(distance = 253): # make adjustments to distance between left and right wheels
    global wheel_distance   # millimeters - use to calculate angle
    wheel_distance = distance
    
def setWheelDiameter(diameter = 81): # change the wheel diameter used to calculate angle
    global wheel_diameter # millimeters
    wheel_diameter = diameter

def set_angle_calc_parameters(wheel_diameter_set = 81, wheel_distance_set = 253, encoder_counts_per_rev_set = 2200, scale_factor_set = 1):
    # constants for calculating spin and pivot turns - functions enables them to be changed programatically
    global wheel_diameter
    global wheel_distance
    global encoder_counts_per_rev 
    global circumference
    global scale_factor
    wheel_diameter = wheel_diameter_set #mm
    wheel_distance = wheel_distance_set #mm
    encoder_counts_per_rev = encoder_counts_per_rev_set
    circumference = math.pi * wheel_diameter  # Calculate the circumference of each wheel
    scale_factor = scale_factor_set # applies a scaling adjustment for turining error. 1.1 = 10% added on 0.9 = 10% substracted


# Function to calculate encoder counts for a spin turn angle
def calculate_encoder_counts_angle(angle):
    # Calculate the distance the rotating wheel needs to travel for the given angle
    distance_to_travel = (math.pi * wheel_distance * angle) / 360
    # Convert the distance to encoder counts
    return int(scale_factor * (distance_to_travel / circumference * encoder_counts_per_rev))
   

def setSpinDegrees(direction = 'stop', speed = 0, desired_angle = 0, wait = 'false'):
    speed = int(abs(speed))
    desired_angle = int(abs(desired_angle))
  #  speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
  #  speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
  #  Don't need to convert % to dps because we're calling setMotorTarget where it's done
  #  This could change if I put this function directly in FW
  
    resetEncoder(1)
    resetEncoder(2)
    
    encoder_counts = calculate_encoder_counts_angle(desired_angle) # get spin turn calculation
    if direction == 'left':
        target1 = encoder_counts
        target2 = encoder_counts
        setMotorTargets(speed,target1,speed,target2,0,0,0,0)
    if direction == 'right':
        target1 = encoder_counts * -1
        target2 = encoder_counts * -1
        setMotorTargets(speed,target1,speed,target2,0,0,0,0)
    if wait == True:
        setDriveWait()
        

def setPivotDegrees(direction = 'stop', speed = 0, desired_angle = 0, wait = 'false'):
    speed = int(abs(speed))
    desired_angle = int(abs(desired_angle))
  #  speed = int(min(max(speed, 0), 100))   # constrain 0 - 100
  #  speed = int(speed * 540 // 100)  # scale 0 - 100% to 0 - 720 dps using integer math
  #  Don't need to convert % to dps because we're calling setMotorTarget where it's done
  #  This could change if I put this function directly in FW    

    resetEncoder(1)
    resetEncoder(2)

    encoder_counts = calculate_encoder_counts_angle(desired_angle) # get encoder angle calculation
    encoder_counts = encoder_counts * 2 # a pivot turn circule is twice as large as a spin
    
    if direction == 'left':
        target1 = encoder_counts
        target2 = encoder_counts
        if drive_invert == False:
            setMotorTargets(0,0,speed,target2,0,0,0,0)
        else:
            setMotorTargets(speed,target1,0,0,0,0,0,0)
                
    if direction == 'right':
        target1 = encoder_counts * -1
        target2 = encoder_counts * -1
        if drive_invert == False:
            setMotorTargets(speed,target1,0,0,0,0,0,0)
            
        else:
            setMotorTargets(0,0,speed,target2,0,0,0,0)
    if wait == True:
        setDriveWait()


def setServoSpeed(servo=0, speed=0):
    if servo == 1234:
        setServoSpeeds(speed,speed,speed,speed)
        return
    
    register = 0
    servo = int(servo)
    speed = int(min(max(speed, 0), 100))   # constrain speed
    
    if servo < 0:
        servo = 0
    if servo > 4:
        servo = 0

    if servo == 1:
        register = 32
    if servo == 2:
        register = 33
    if servo == 3:
        register = 34
    if servo == 4:
        register = 35
    
    values = [speed]
    write_reg(register, values)
    
     
def setServoSpeeds(speed1=0, speed2=0, speed3=0, speed4=0):
    speed1 = int(min(max(speed1, 0), 100))   # constrain speed
    speed2 = int(min(max(speed2, 0), 100))   # constrain speed
    speed3 = int(min(max(speed3, 0), 100))   # constrain speed
    speed4 = int(min(max(speed4, 0), 100))   # constrain speed
    
    values = [speed1, speed2, speed3, speed4]
    write_reg(36, values)


def setServoPosition(servo=0, position=0):
    if servo == 1234:
        setServoPositions(position,position,position,position)
        return
    
    register = 0
    servo = int(servo)
    position = int(min(max(position, 0), 180))   # constrain position

    if servo < 0:
        servo = 0
    if servo > 4:
        servo = 0

    if servo == 1:
        register = 37
        if servo1_invert == True:
            position = 180 - position
    if servo == 2:
        register = 38
        if servo2_invert == True:
            position = 180 - position
    if servo == 3:
        register = 39
        if servo3_invert == True:
            position = 180 - position
    if servo == 4:
        register = 40
        if servo4_invert == True:
            position = 180 - position
    
    values = [position]
    write_reg(register, values)

    

def setServoPositions(position1=0, position2=0, position3=0, position4=0):
    position1 = int(min(max(position1, 0), 180))   # constrain position
    position2 = int(min(max(position2, 0), 180))   # constrain position
    position3 = int(min(max(position3, 0), 180))   # constrain position
    position4 = int(min(max(position4, 0), 180))   # constrain position
    
    if servo1_invert == True:
        position1 = 180 - position1
    if servo2_invert == True:
        position2 = 180 - position2
    if servo3_invert == True:
        position3 = 180 - position3
    if servo4_invert == True:
        position4 = 180 - position4
    
    values = [position1, position2, position3, position4]
    write_reg(41, values)


def setServoInvert(servo = 0, invert = False):
    register = 0
    servo = int(servo)
       
    if servo == 1:
        register = 92
    if servo == 2:
        register = 93
    if servo == 3:
        register = 94
    if servo == 4:
        register = 95
    
    values = [invert]
    write_reg(register, values)
    
    
def getServoPosition(servo=0):
    register = 0
    servo = int(servo)
    
    if servo < 0:
        servo = 0
    if servo > 4:
        servo = 0

    if servo == 1:
        register = 42
    if servo == 2:
        register = 43
    if servo == 3:
        register = 44
    if servo == 4:
        register = 45
        
    time.sleep_ms(i2c_pace)  # pace I2C bus
    result = i2c_mc.readfrom_mem(5, register, 1)  # read 1 byte from register
    value = result[0]
    return value         


def setDriveWait():
    time.sleep_ms(100)
    while (getMotorBusy(1)) or (getMotorBusy(2)):
        pass
    time.sleep_ms(500)
    resetEncoder(1)
    resetEncoder(2)
    
    
def getMotorBusy(motor=0):
    register = 0
    motor = int(motor)
    
    if motor < 0:
        motor = 0
    if motor > 4:
        motor = 0

    if motor == 1:
        register = 72
    if motor == 2:
        register = 73
    if motor == 3:
        register = 74
    if motor == 4:
        register = 75
    
    result = i2c_mc.readfrom_mem(5, register, 1)  # read 1 byte from register
    value = result[0]
    return value


def setMotorInvert(motor=0, invert= 'false'):
    register = 0
    motor = int(motor)
    
    if invert == 'true':
        invert = 1
    else:
        invert = 0
       
    if motor == 1:
        register = 76
    if motor == 2:
        register = 77
    if motor == 3:
        register = 78
    if motor == 4:
        register = 79
    
    values = [invert]
    write_reg(register, values)
    
 
def setDriveInvert(invert = False):
    global drive_invert
    drive_invert = invert
    resetEncoder(1)
    resetEncoder(2)


def setDriveStop():
    #setMotorSpeed(1,0)
    #setMotorSpeed(2,0)
    setMotorSpeeds(0,0,0,0)
    #setMotorPowers(125,125,125,125)
    
    
def setMotorStop(motor=0, stop=0):
    if stop == 'brake':
        stop = 125
    else:
        stop = 0
    if motor == 1234:
        setMotorPowers(stop, stop, stop, stop)
    else:
        setMotorPower(motor, stop)    


def getEncoderCount(encoder=0):
    register = 0
    encoder = int(encoder)
    
    if encoder < 0:
        encoder = 0
    if encoder > 4:
        encoder = 0
    
    if encoder == 1:
        register = 62
    if encoder == 2:
        register = 63
    if encoder == 3:
        register = 64
    if encoder == 4:
        register = 65

    time.sleep_ms(i2c_pace)  # pace I2C bus
    result = i2c_mc.readfrom_mem(5, register, 4)  # read 4 bytes from register 
    value = int.from_bytes(bytearray(result), 'big')
    if (value & (1 << (32 - 1))) != 0:    # if sign bit is set (32 bit)- 2's complement
        value = value - (1 << 32)         # compute negative value
    return value  
    
   

def getButtonStatus(button = 'none'):
    if button == 'start':
        return not button_start.value()
    if button == 'a':
        return not button_a.value()
    if button == 'b':
        return not button_b.value()
    if button == 'c':
        return not button_c.value()
    if button == 'd':
        return not button_d.value()


def resetEncoder(encoder=0):
    register = 0
    encoder = int(encoder)
                
    if encoder == 1:
        register = 67
    if encoder == 2:
        register = 68
    if encoder == 3:
        register = 69
    if encoder == 4:
        register = 70
    if encoder == 1234:
        register = 71        
   
    i2c_mc.writeto_mem(5, register, bytes(0))
    time.sleep_ms(50) # reset encoders need a delay to reset
    


###########################################  NEOPIXELS

def hex_to_rgb(value):
    value = value.lstrip('#')
    lv = len(value)
    return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))


def rgb_to_hex(red, green, blue):
    # Convert each RGB component to hexadecimal
    red_hex = "{:02x}".format(int(red))
    green_hex = "{:02x}".format(int(green))
    blue_hex = "{:02x}".format(int(blue))
    # Combine the hexadecimal components into a single string
    hex_color = "#" + red_hex + green_hex + blue_hex
    return hex_color


'''
color can be RGB or HEX
If color is HEX, it must be a string: example '#ff0000'
An RGB value can be a variable: example: color = 255,0,0
RGB can also be passed as list: example: [0,255,0]
brighness is optional -1 does not apply brightness to color level

'''
def setPixelColor(pixel, color, bright = -1):
    bright = int(bright)  # cannot be a float
    try:
        color = hex_to_rgb(color) #if function succeeds, color is HEX converted to rgb
    except:
        pass # exception means color is an rgb list variable - continue on
    
    if bright != -1:
        brightness = int(min(max(bright, 0), 100))
        brightness = bright / 100  #returns floating point division - range is 0.0 - 1.0

    if pixel == 'A':
        pixels[0] = (color)
        if bright != -1:
            pixels[0] = tuple(int(value * brightness) for value in pixels[0])
        pixels.write()
    if pixel == 'B':
        pixels[1] = (color)
        if bright != -1:
            pixels[1] = tuple(int(value * brightness) for value in pixels[1])
        pixels.write()
    if pixel == 'C':
        pixels[2] = (color)
        if bright != -1:
            pixels[2] = tuple(int(value * brightness) for value in pixels[2])
        pixels.write()
    if pixel == 'D':
        pixels[3] = (color)
        if bright != -1:
            pixels[3] = tuple(int(value * brightness) for value in pixels[3])
        pixels.write()
    if pixel == 'ABCD':
        pixels[0] = (color)
        pixels[1] = (color)
        pixels[2] = (color)
        pixels[3] = (color)
        if bright != -1:
            pixels[0] = tuple(int(value * brightness) for value in pixels[0])
            pixels[1] = tuple(int(value * brightness) for value in pixels[1])
            pixels[2] = tuple(int(value * brightness) for value in pixels[2])
            pixels[3] = tuple(int(value * brightness) for value in pixels[3])
        pixels.write()


def setPixelColorRGB(pixel=0, red=0, blue=0, green=0):
    try:
        red = int(min(max(red, 0), 255))
        blue = int(min(max(blue, 0), 255))
        green = int(min(max(green, 0), 255))
    except:
        print('error: invalid red, green, blue type')
        return
    if pixel == 'A':
        pixels[0] = (red,blue,green)
        pixels.write()
    if pixel == 'B':
        pixels[1] = (red,blue,green)
        pixels.write()
    if pixel == 'C':
        pixels[2] = (red,blue,green)
        pixels.write()
    if pixel == 'D':
        pixels[3] = (red,blue,green)
        pixels.write()
    if pixel == 'ABCD':
        pixels[0] = (red,blue,green)
        pixels[1] = (red,blue,green)
        pixels[2] = (red,blue,green)
        pixels[3] = (red,blue,green)
        pixels.write()

'''
HEX color must be a string - example: color = '#ff0000'
'''
def setPixelColorHex(pixel, color):
    try:
        rgb = hex_to_rgb(color)
    except:
        print('error: color is not a valid HEX string')
        return

    if pixel == 'A':
        pixels[0] = (rgb)
        pixels.write()
    if pixel == 'B':
        pixels[1] = (rgb)
        pixels.write()
    if pixel == 'C':
        pixels[2] = (rgb)
        pixels.write()
    if pixel == 'D':
        pixels[3] = (rgb)
        pixels.write()
    if pixel == 'ABCD':
        pixels[0] = (rgb)
        pixels[1] = (rgb)
        pixels[2] = (rgb)
        pixels[3] = (rgb)
        pixels.write()
        
'''
color must be a hex string - example: color = '#ff0000'
'''
def setPixelBlink(pixel, color, loop):
    loop = int(loop)  # cannot be a float
    try:
        rgb = hex_to_rgb(color)
    except:
        print('error: color is not a valid HEX string')
        return
    
    brightness = 0.5  # force to 50%
    if pixel == 'A':
        x = 0
        while x < loop:
            x += 1
            pixels[0] = (rgb)
            pixels[0] = tuple(int(value * brightness) for value in pixels[0])
            pixels.write()
            time.sleep(.25)
            pixels[0] = BLACK
            pixels.write()
            time.sleep(.25)
    if pixel == 'B':
        x = 0
        while x < loop:
            x += 1
            pixels[1] = (rgb)
            pixels[1] = tuple(int(value * brightness) for value in pixels[1])
            pixels.write()
            time.sleep(.25)
            pixels[1] = BLACK
            pixels.write()
            time.sleep(.25)
    if pixel == 'C':
        x = 0
        while x < loop:
            x += 1
            pixels[2] = (rgb)
            pixels[2] = tuple(int(value * brightness) for value in pixels[2])
            pixels.write()
            time.sleep(.25)
            pixels[2] = BLACK
            pixels.write()
            time.sleep(.25)
    if pixel == 'D':
        x = 0
        while x < loop:
            x += 1
            pixels[3] = (rgb)
            pixels[3] = tuple(int(value * brightness) for value in pixels[3])
            pixels.write()
            time.sleep(.25)
            pixels[3] = BLACK
            pixels.write()
            time.sleep(.25)
    if pixel == 'ABCD':
        x = 0
        while x < loop:
            x += 1
            pixels[0] = (rgb)
            pixels[1] = (rgb)
            pixels[2] = (rgb)
            pixels[3] = (rgb)
            pixels[0] = tuple(int(value * brightness) for value in pixels[0])
            pixels[1] = tuple(int(value * brightness) for value in pixels[1])
            pixels[2] = tuple(int(value * brightness) for value in pixels[2])
            pixels[3] = tuple(int(value * brightness) for value in pixels[3])
            pixels.write()
            time.sleep(.25)
            pixels[0] = BLACK
            pixels[1] = BLACK
            pixels[2] = BLACK
            pixels[3] = BLACK
            pixels.write()
            time.sleep(.25)


def setPixelOff(pixel = 0):  
    if pixel == 'A':
        pixels[0] = (0,0,0)
    if pixel == 'B':
        pixels[1] = (0,0,0)
    if pixel == 'C':
        pixels[2] = (0,0,0)
    if pixel == 'D':
        pixels[3] = (0,0,0)
    if pixel == 'ABCD':
        pixels[0] = (0,0,0)
        pixels[1] = (0,0,0)
        pixels[2] = (0,0,0)
        pixels[3] = (0,0,0)   
    pixels.write()


def setPixelAnimation(animation='none', duration=0):
    duration = int(duration)
    if animation == 'rainbow':
        setPixelRainbow(duration)
    if animation == 'comet':
        setPixelComet(duration)        
    if animation == 'chase':
        setPixelChase(duration)
    if animation == 'fade':
        setPixelfade(duration)
    if animation == 'sparkle':
        setPixelSparkle(duration)
    if animation == 'colorcycle':
        setPixelColorCycle(duration)
    if animation == 'sparkle_fade':
        setPixelSparkleFade(duration)
    if animation == 'rainbow_comet':
        setPixelRainbowComet(duration)
    if animation == 'rainbow_chase':
        setPixelRainbowChase(duration)
    if animation == 'rainbow_sparkle':
        setPixelRainbowSparkle(duration)
    
        
def setPixelChase(duration = 0):
    rgb = hex_to_rgb('#00cccc')
    duration = int(duration)
    duration = duration * 2  #scale loop passes for seconds - 125*4*2 = 1000
    for j in range(duration):
        for q in range(4):
            for i in range(0, 4 - 3, 3):
                pixels[i + q] = rgb
                pixels.write()
                time.sleep_ms(125)
                pixels[i + q] = (0, 0, 0)                
    setPixelOff('ABCD')
    
def setPixelRainbow(duration=0):
    x = 0
    while x < duration:
        x += 1
        for j in range(255):
            for i in range(4):
                rc_index = (i * 256 // 4) + j
                pixels[i] = wheel(rc_index & 255)
            pixels.write()
            time.sleep_ms(3)
        setPixelOff('ABCD')        

def setPixelComet(duration = 0):
    duration = int(duration)  # cannot be a float
    rgb = hex_to_rgb('#00cccc')
    brightness = 0.2  #20%
    x = 0
    duration = duration * 5 #number of loop passes that equals 1 second
    while x < duration:
        x += 1
        for i in range(4 + 3): #4 is the pixels number, 3 is the tail length
            for j in range(3): #3 is the tail length
                if i - j >= 0 and i - j < 4: #4 is the pixel number
                    pixels[i - j] = (128,128,128)
            pixels.write()
            time.sleep_ms(25)
            pixels.fill((0, 0, 0))
    
    setPixelOff('ABCD')
    
def setPixelRainbowComet(duration = 0):
    duration = int(duration)  # cannot be a float
    rgb = hex_to_rgb('#00cccc')
    brightness = 0.2  #20%
    x = 0
    duration = duration * 5 #number of loop passes that equals 1 second
    while x < duration:
        x += 1
        for i in range(4 + 3): #4 is the pixels number, 3 is the tail length
            for j in range(3): #3 is the tail length
                if i - j >= 0 and i - j < 4: #4 is the pixel number
                    pixels[i - j] = (random.getrandbits(8), random.getrandbits(8), random.getrandbits(8))
            pixels.write()
            time.sleep_ms(25)
            pixels.fill((0, 0, 0))
    
    setPixelOff('ABCD')    
    
def setPixelFade(duration = 0):
    steps = 100  # Number of steps for the fade
    delay = duration / (2 * steps)

    for i in range(steps):
        brightness = int(i * 255 / steps)
        pixels.fill((brightness, 0, 0))  #RGB - this fades red
        pixels.write()
        time.sleep(delay)

    #time.sleep(1)  # Pause at full brightness

    for i in range(steps, 0, -1):
        brightness = int(i * 255 / steps)
        pixels.fill((brightness, 0, 0))  #RGB - this fades red 
        pixels.write()
        time.sleep(delay)
        
    setPixelOff('ABCD') # fully off


def setPixelSparkle(iterations=0): # 20 interations is 1 second
    iterations = int(iterations)
    iterations = iterations * 20
    for _ in range(iterations):
        pixel_index = random.getrandbits(8) % 4 #4 is num pixels
        color = (128,128,128)            
        pixels[pixel_index] = color
        pixels.write()
        time.sleep_ms(50)

        # Turn off the pixel after a short delay
        pixels[pixel_index] = (0, 0, 0)
        pixels.write()
        
def setPixelRainbowSparkle(iterations=0): # 20 interations is 1 second
    iterations = int(iterations)
    iterations = iterations * 20
    for _ in range(iterations):
        pixel_index = random.getrandbits(8) % 4 #4 is num pixels
        color = (
            random.getrandbits(8),
            random.getrandbits(8),
            random.getrandbits(8)
        )
        pixels[pixel_index] = color
        pixels.write()
        time.sleep_ms(50)

        # Turn off the pixel after a short delay
        pixels[pixel_index] = (0, 0, 0)
        pixels.write()    

def setPixelColorCycle(iterations = 0):
    iterations = int(iterations)
    iterations = iterations * 5  #5 iterations is ~1 second
    for _ in range(iterations):
        color = (random.getrandbits(8), random.getrandbits(8), random.getrandbits(8))
        pixels[0] = color
        pixels[1] = color
        pixels[2] = color
        pixels[3] = color
        pixels.write()
        time.sleep_ms(200)    
    setPixelOff('ABCD') # fully off
    
    
def setPixelSparkleFade(iterations=20): # 20 interations is 1 second
    iterations = int(iterations)
    iterations = iterations * 20
    for _ in range(iterations):
        pixel_index = random.getrandbits(8) % 4 #4 is num pixels
        color = (
            random.getrandbits(8),
            random.getrandbits(8),
            random.getrandbits(8)
        )
        pixels[pixel_index] = color
        pixels.fill((brightness, 0, 0))  #RGB - this fades red
        pixels.write()
        time.sleep_ms(50)

        # Turn off the pixel after a short delay
        pixels[pixel_index] = (0, 0, 0)
        pixels.write()
        
def setPixelRainbowChase(duration=0):
    duration = int(duration)
    duration = duration * 2  #scale loop passes for seconds
    for j in range(duration):
        for q in range(4):
            for i in range(0, 4 - 3, 3):
                rc_index = (i * 256 // 4) + j
                pixels[i + q] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                pixels.write()
                time.sleep_ms(125)
                pixels[i + q] = (0, 0, 0)                
    setPixelOff('ABCD')

    # Turn off all pixels
    pixels.fill((0, 0, 0))
    pixels.write()    

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 85:
        return (int(pos * 3), int(255 - (pos * 3)), 0)
    elif pos < 170:
        pos -= 85
        return (int(255 - pos * 3), 0, int(pos * 3))
    else:
        pos -= 170
        return (0, int(pos * 3), int(255 - pos * 3))

def random_blink(iterations=50):
    for _ in range(iterations):
        pixel_index = urandom.getrandbits(8) % 4
        np[pixel_index] = (255, 255, 255)
        np.write()
        time.sleep_ms(50)
        np[pixel_index] = (0, 0, 0)
    setPixelOff('ABCD')        


############################################ SOUND EFFECTS
    
def setSoundEffect(effect):
    if effect == 'siren':
        siren_effect()
    if effect == 'whistle_down':
        whistle_down_effect()
    if effect == 'whistle_up':
        whistle_up_effect()
    if effect == 'correct':
        correct_effect()
    if effect == 'incorrect':
        incorrect_effect()
    if effect == 'r2d2':
        r2d2_effect()
    if effect == 'phaser':
        phaser_effect()
    if effect == 'happy_birthday':
        happy_bday_effect()
    if effect == 'close_encounters':
        close_encounters_effect()
    if effect == 'super_mario':
        super_mario_effect()
    if effect == 'up_and_down':
        up_down_effect()
    if effect == 'random_sounds':
        random_effect()
    if effect == 'red_alert':
        red_alert_effect()
    if effect == 'whoops':
        whoops_effect()
    if effect == 'buzz_buzz_buzz':
        buzzer_effect()    
    
def setSoundOff():
    piezo.duty(0)    

def setTone(f, length):
    if length < 0:
        piezo.duty(0) 
        return
    f = abs(int(f))  # Ensure frequency is an integer (cannot be a float)
    if f < 31:  # 31hz is the bottom
        piezo.duty(0)
        return
    piezo.freq(f)
    piezo.duty(512)  # On 50%
    if length > 0:
        time.sleep(length)
        piezo.duty(0)
       

def setNote(n, length):
    if n == 'OFF':
        piezo.duty(0)
        return 
    if length < 0:
        piezo.duty(0) 
        return
    
    if n == 'B0':
        n = 31      
    if n == 'C1':
        n = 33
    if n == 'CS1':
        n = 35      
    if n == 'D1':
        n = 37
    if n == 'DS1':
        n = 39      
    if n == 'E1':
        n = 41
    if n == 'F1':
        n = 44      
    if n == 'FS1':
        n = 46
    if n == 'G1':
        n = 49      
    if n == 'GS1':
        n = 52
    if n == 'A1':
        n = 55      
    if n == 'AS1':
        n = 58
    if n == 'B1':
        n = 62      
    if n == 'C2':
        n = 65
    if n == 'CS2':
        n = 69      
    if n == 'D2':
        n = 73
    if n == 'DS2':
        n = 78      
    if n == 'E2':
        n = 82
    if n == 'F2':
        n = 87      
    if n == 'FS2':
        n = 93
    if n == 'G2':
        n = 98      
    if n == 'GS2':
        n = 104
    if n == 'A2':
        n = 110      
    if n == 'AS2':
        n = 117
    if n == 'B2':
        n = 123      
    if n == 'C3':
        n = 131
    if n == 'CS3':
        n = 139      
    if n == 'D3':
        n = 147
    if n == 'DS3':
        n = 156
    if n == 'E3':
        n = 165      
    if n == 'F3':
        n = 175
    if n == 'FS3':
        n = 185      
    if n == 'G3':
        n = 196
    if n == 'GS3':
        n = 208      
    if n == 'A3':
        n = 220
    if n == 'AS3':
        n = 233      
    if n == 'B3':
        n = 247
    if n == 'C4':
        n = 262      
    if n == 'CS4':
        n = 277
    if n == 'D4':
        n = 294      
    if n == 'DS4':
        n = 311
    if n == 'E4':
        n = 330      
    if n == 'F4':
        n = 349
    if n == 'FS4':
        n = 370      
    if n == 'G4':
        n = 392
    if n == 'GS4':
        n = 415      
    if n == 'A4':
        n = 440
    if n == 'AS4':
        n = 466      
    if n == 'B4':
        n = 494
    if n == 'C5':
        n = 523      
    if n == 'CS5':
        n = 554
    if n == 'D5':
        n = 587      
    if n == 'DS5':
        n = 622
    if n == 'E5':
        n = 659      
    if n == 'F5':
        n = 698
    if n == 'FS5':
        n = 740      
    if n == 'G5':
        n = 784
    if n == 'GS5':
        n = 831
    if n == 'A5':
        n = 880
    if n == 'AS5':
        n = 932      
    if n == 'B5':
        n = 988
    if n == 'C6':
        n = 1047      
    if n == 'CS6':
        n = 1109
    if n == 'D6':
        n = 1175      
    if n == 'DS6':
        n = 1245
    if n == 'E6':
        n = 1319      
    if n == 'F6':
        n = 1397
    if n == 'FS6':
        n = 1480      
    if n == 'G6':
        n = 1568
    if n == 'GS6':
        n = 1661      
    if n == 'A6':
        n = 1760
    if n == 'AS6':
        n = 1865      
    if n == 'B6':
        n = 1976
    if n == 'C7':
        n = 2093      
    if n == 'CS7':
        n = 2217
    if n == 'D7':
        n = 2349      
    if n == 'DS7':
        n = 2489
    if n == 'E7':
        n = 2637      
    if n == 'F7':
        n = 2794
    if n == 'FS7':
        n = 2960      
    if n == 'G7':
        n = 3136
    if n == 'GS7':
        n = 3322      
    if n == 'A7':
        n = 3520
    if n == 'AS7':
        n = 3729      
    if n == 'B7':
        n = 3951
    if n == 'C8':
        n = 4186      
    if n == 'CS8':
        n = 4435
    if n == 'D8':
        n = 4699
    if n == 'DS8':
        n = 4978

    piezo.freq(n)
    piezo.duty(512)  # On 50%
    
    if length > 0:
        time.sleep(length)
        piezo.duty(0)
        
def siren_effect():
    piezo.duty(512) # On 50%
    for frequency in range(500, 1000, 50):  # Rising pitch
        piezo.freq(frequency)
        time.sleep_ms(20)

    for frequency in range(1000, 500, -50):  # Falling pitch
        piezo.freq(frequency)
        time.sleep_ms(20)
    piezo.duty(0)

def whistle_up_effect():
    piezo.duty(512) # On 50%
    for frequency in range(100,1000,1):
        piezo.freq(frequency)
        time.sleep_ms(frequency // 100)
    piezo.duty(0) 

def whistle_down_effect():
    piezo.duty(512) # On 50%
    steps = 20.0
    freq = 2000
    while freq > 500:
        steps += 1.5
        freq = freq - int(steps)
        piezo.freq(freq)
        time.sleep_ms(20)
    piezo.duty(0)

def whistle_up_effect():
    piezo.duty(512) # On 50%
    steps = 20.0
    freq = 500
    while freq < 2000:
        steps += 1.5
        freq = freq + int(steps)
        piezo.freq(freq)
        time.sleep_ms(20)
    piezo.duty(0)
    
def correct_effect():
    piezo.duty(512)  # On 50%
    piezo.freq(523)
    time.sleep(0.2)
    piezo.freq(698)
    time.sleep(0.2)
    piezo.duty(0)
    
def incorrect_effect():
    piezo.duty(512)  # On 50%
    piezo.freq(698)
    time.sleep(0.2)
    piezo.freq(349)
    time.sleep(0.2)
    piezo.duty(0)
    
def phaser_effect():
    piezo.duty(512) # On 50%
    for _ in range(4): 
        for i in range(100,1000,50):
            piezo.freq(i)
            #time.sleep(i / 100000)
            time.sleep_ms(20)
    piezo.duty(0)
    
def happy_bday_effect():
    piezo.duty(512) # On 50%
    piezo.freq(262)
    time.sleep(.2)
    piezo.duty(0) 
    time.sleep(.02)
    piezo.duty(512) # On 50%
    piezo.freq(262)
    time.sleep(.2)
    piezo.freq(294)
    time.sleep(.4)
    piezo.freq(262)
    time.sleep(.4)
    piezo.freq(349)
    time.sleep(.4)
    piezo.freq(330)
    time.sleep(.5)
    piezo.duty(0)
    
def close_encounters_effect():
    piezo.duty(512) # On 50%
    piezo.freq(392)
    time.sleep(.3)
    piezo.freq(440)
    time.sleep(.3)
    piezo.freq(349)
    time.sleep(.3)
    piezo.freq(175)
    time.sleep(.3)
    piezo.freq(262)
    time.sleep(.3)
    piezo.duty(0)  # off
    
def super_mario_effect():
    piezo.duty(512) # On 50%
    piezo.freq(330)
    time.sleep(.15)
    piezo.duty(0)  # off
    time.sleep(.05)
    piezo.duty(512) # On 50%
    piezo.freq(330)
    time.sleep(.15)
    piezo.duty(0)  # off
    time.sleep(.1)
    piezo.duty(512) # On 50%
    piezo.freq(330)
    time.sleep(.3)
    piezo.duty(512) # On 50%
    piezo.freq(262)
    time.sleep(.2)
    piezo.freq(330)
    time.sleep(.3)
    piezo.freq(392)
    time.sleep(.6)
    piezo.freq(196)
    time.sleep(.4)
    piezo.duty(0)  # off
    
def up_down_effect():
    piezo.duty(512) # On 50%
    for i in range(100,600,25):
        piezo.freq(i)
        time.sleep_ms(20)
    for i in range(600,100,-25):
        piezo.freq(i)
        time.sleep_ms(20)
    piezo.duty(0)  # off
    
def random_effect():
    piezo.duty(512)  # On 50%
    for _ in range(20):
        f = random.randrange(100, 1000)
        piezo.freq(f)
        time.sleep(.1)
    piezo.duty(0)
    
def red_alert_effect():
    for _ in range(3):    
        piezo.duty(512)  # On 50%
        for i in range(440,1200,20):
            piezo.freq(i)
            time.sleep_ms(20)
        piezo.duty(0)  # off
        time.sleep(.05)

def whoops_effect():
    for i in range (0, 3):
        piezo.duty(512)  # On 50%
        piezo.freq(1500)
        time.sleep_ms(100)
        piezo.duty(0)  # off
        time.sleep_ms(10)
    for i in range (0, 3):
        piezo.duty(512)  # On 50%
        piezo.freq(100)
        time.sleep_ms(100)
        piezo.duty(0)  # off
        time.sleep_ms(10)

def buzzer_effect():
    for _ in range(3):
        for i in range (0, 5):
            piezo.duty(512)  # On 50%
            piezo.freq(262)
            time.sleep_ms(15)
            piezo.duty(0)  # off
            time.sleep_ms(10)
        time.sleep_ms(60)




# send enable byte to motor chip - DC and Servo motor channels will wakeup when received
def setEnableMotors():        
    values = [0]
    write_reg(26, values)
    time.sleep_ms(100)
    setMotorStop(1234,'coast') # release the brakes
    time.sleep_ms(100)

def resetMC():                     # reboots the internal mc chip
    values = [0]
    write_reg(28, values)
    time.sleep_ms(250)


def End():
    print('Program Stopped.....rebooting')
    #exec(open('code.py').read()) # restart the code
    file_open = False
    try:
        with open('main.py') as file:
            file_open = True
            code_content = file.read()
            exec(code_content)
    except:
        print('error running main.py, or it is missing')
    finally:
        if file_open:
            file.close()
            file_open = False
        
    
def Begin():
    resetMC()
    setSoundOff()
    pixels[0] = (0,0,0)
    pixels[1] = (0,0,0)
    pixels[2] = (0,0,0)
    pixels[3] = (0,0,0)
    pixels.write()
    setPixelColor('ABCD','#ffffff',50) #flash white lights
    time.sleep_ms(200)
    setPixelOff('ABCD')
    LED_green.value(False)  #on
    setTone(440, 0.1)
    time.sleep_ms(250)
    print('Program Ready -- waiting for start')
    while button_start.value():
        time.sleep_ms(100)    
                 
    LED_green.value(True) #off
    setTone(440, 0.1)
    setTone(880, 0.1)
    setEnableMotors()
    print('Running Program')  




