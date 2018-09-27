# Simple demo of reading the MMA8451 orientation every second.
# Author: Tony DiCola
import time

import board
import busio

import adafruit_mma8451
import RPi.GPIO as GPIO

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MMA8451 module.
sensor = adafruit_mma8451.MMA8451(i2c)
# Optionally change the address if it's not the default:
#sensor = adafruit_mma8451.MMA8451(i2c, address=0x1C)

# Optionally change the range from its default of +/-4G:
#sensor.range = adafruit_mma8451.RANGE_2G  # +/- 2G
#sensor.range = adafruit_mma8451.RANGE_4G  # +/- 4G (default)
#sensor.range = adafruit_mma8451.RANGE_8G  # +/- 8G

# Optionally change the data rate from its default of 800hz:
#sensor.data_rate = adafruit_mma8451.DATARATE_800HZ  #  800Hz (default)
#sensor.data_rate = adafruit_mma8451.DATARATE_400HZ  #  400Hz
sensor.data_rate = adafruit_mma8451.DATARATE_200HZ  #  200Hz
#sensor.data_rate = adafruit_mma8451.DATARATE_100HZ  #  100Hz
#sensor.data_rate = adafruit_mma8451.DATARATE_50HZ   #   50Hz
#sensor.data_rate = adafruit_mma8451.DATARATE_12_5HZ # 12.5Hz
#sensor.data_rate = adafruit_mma8451.DATARATE_6_25HZ # 6.25Hz
#sensor.data_rate = adafruit_mma8451.DATARATE_1_56HZ # 1.56Hz



pin = 18
before = time.perf_counter()
after = before
totalSamples = 0
loops = 0
keepGoing = True

def wrapperCallback(channel):
    sensor._internalFifoCallback(channel)

def dataCallback(status, samplesAvail, data):
    global totalSamples
    global loops
    global before
    global pin
    global keepGoing
    global after
    totalSamples += samplesAvail
    loops += 1
    preMsg = ""
    if status >= 128:
        preMsg = "Overflow "
    if status & 0x40 > 0:
        preMsg += "Watermark "
    print("{0} load {1:d} samples: {2:b} {3:b}".format(preMsg, samplesAvail, (status & 128)>0, (status & 64)>0))
    x, y, z = sensor.demux(data)
    print("{0:d} {1:d}".format(x[0], x[1]))
    if (totalSamples > 1000):
        after = time.perf_counter()
        delta = after-before
        print("time take for {0:d} loops is {1:3.2f}, {2:d} samples at {3:3.2f} sps".format(loops, delta, totalSamples, totalSamples/delta))
        sensor.reset()
        GPIO.remove_event_detect(pin)
        keepGoing = False

before = time.perf_counter()
sensor.enableFifoBuffer(30, pin, dataCallback)

# enable interupt
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.add_event_detect(pin, GPIO.FALLING, callback=wrapperCallback, bouncetime=40)


# Main loop to print the acceleration and orientation every second.
while keepGoing:
    time.sleep(1.1)

delta = after-before
print("DONE: time take for {0:d} reads is {1:3.2f}, {2:d} samples at {3:3.2f} sps".format(loops, delta, totalSamples, (totalSamples-1)/delta))

GPIO.remove_event_detect(pin)
