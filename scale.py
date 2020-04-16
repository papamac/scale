#!/usr/bin/env python3
"""
 PACKAGE:  Raspberry Pi based recording scale (scale)
  MODULE:  scale.py
   TITLE:  Raspberry Pi scale (scale)
FUNCTION:  scale provides an application program that powers a Raspberry Pi
           based weight/volume scale.  It controls the HX711 24-bit A/D
           converter connected to the scale's load cell and the Adafruit
           ssd1306 OLED display that reports the results.
   USAGE:  scale is normally run as a daemon on a standalone Raspberry Pi using
           the scale daemon (scaled).  It can also be executed from the command
           line with options specified in the argsandlogs module.  When running
           as a daemon the additional daemon option (-d) must be set.  See
           usage examples below.
  AUTHOR:  papamac
 VERSION:  1.0.12
    DATE:  April 16, 2020


MIT LICENSE:

Copyright (c) 2020 David A. Krause, aka papamac

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


DESCRIPTION:

****************************** needs work *************************************

DEPENDENCIES/LIMITATIONS:

****************************** needs work *************************************

"""
__author__ = 'papamac'
__version__ = '1.0.1'
__date__ = 'April 6, 2020'


from os import fork
from pathlib import Path
from signal import signal, SIGTERM
from subprocess import run
from time import monotonic, sleep

import adafruit_ssd1306
import board
import busio
from gpiozero import Device, Button
from gpiozero.pins.pigpio import PiGPIOFactory
from PIL import Image, ImageDraw, ImageFont
import pigpio

from hx711 import CH_A_GAIN_64, CH_A_GAIN_128, HX711
from papamaclib.argsandlogs import AL
from papamaclib.colortext import getLogger

# Global constants:

LOG = getLogger('Plugin')
PID_PATH = Path('/run/scaled.pid')


# scale functions:

def display(text, font='arialbd.ttf', size=18):
    image = Image.new('1', (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(font, size)
    draw.text((0, 0), text, font=font, fill=1)
    oled.image(image)
    oled.show()
    del image


def terminate(*args):
    global running
    running = False


# scale main program:

AL.parser.add_argument('-d', '--daemon', action='store_true',
                       help='daemon: forks and writes a pid file')
AL.start(__version__)
if AL.args.daemon:
    pid = fork()
    if pid:
        pid_file = open(PID_PATH, 'w')
        pid_file.write(str(pid))
        pid_file.close()
        exit()
signal(SIGTERM, terminate)
gpio = pigpio.pi()
scale = HX711(gpio)

Device.pin_factory = PiGPIOFactory()
top_button = Button(17)
left_button = Button(27)
center_button = Button(4)
right_button = Button(23)
bottom_button = Button(22)
button1 = Button(5)
button2 = Button(6)

# Create the I2C interface and the SSD1306 OLED class.

i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Clear display.

oled.fill(0)
oled.show()

display('Zero', size=28)
scale.zero()
prior_value = scale.read()
prior_time = start_time = monotonic()
max_rate = 0
num_readings = 0
running = True
stop = False

while running:
    value = scale.read()
    time = monotonic()
    num_readings += 1
    rate = (value - prior_value)/(time-prior_time)
    if max_rate < rate < 100:
        max_rate = rate
    display('%6.1f\n%6.1f' % (value, max_rate), font='courbd.ttf', size=28)
    LOG.data('%12f %12f', value, max_rate)
    prior_value = value
    prior_time = time

    if button1.is_pressed:
        display('Zero', size=28)
        scale.zero()
        prior_value = scale.read()
        prior_time = monotonic()
        max_rate = 0
    elif button2.is_pressed:
        display('Record', size=28)
        LOG.info('%12f %12f', value, max_rate)
        sleep(1.0)
    elif bottom_button.is_pressed:
        display('Calibrate', size=28)
        scale.zero()
        display('Place 100g wt\nPress again')
        bottom_button.wait_for_press()
        scale.calibrate()
        prior_value = scale.read()
        prior_time = monotonic()
        max_rate = 0
    elif left_button.is_pressed:
        display('Low gain', size=28)
        scale.set_mode(CH_A_GAIN_64)
        sleep(1.0)
    elif right_button.is_pressed:
        display('High gain', size=28)
        scale.set_mode(CH_A_GAIN_128)
        sleep(1.0)
    elif top_button.is_held:
        display('Stop', size=28)
        running = False
        stop = True
        sleep(1.0)

measurement_rate = num_readings / (monotonic() - start_time)
display('%6.1f' % measurement_rate, font='courbd.ttf', size=28)
sleep(1.0)

oled.fill(0)
oled.show()
gpio.stop()
AL.stop()

if PID_PATH.is_file():
    PID_PATH.unlink()

if stop:
    sleep(5.0)
    run(['sudo', 'poweroff'])
