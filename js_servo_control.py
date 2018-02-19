# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array, time, spidev, RPi.GPIO as GPIO
from fcntl import ioctl
from multiprocessing import Process
from Adafruit_PWM_Servo_Driver import PWM
GPIO.setmode(GPIO.BOARD)

# Name GPIO pins
hatForward = 7
hatBackward = 11
hatRight = 15 
hatLeft = 13

# Define pins for GPIO use
GPIO.setup(hatForward, GPIO.OUT)
GPIO.setup(hatBackward, GPIO.OUT)
GPIO.setup(hatLeft, GPIO.OUT)
GPIO.setup(hatRight, GPIO.OUT)
# Default pins LOW
GPIO.output(hatForward, GPIO.LOW)
GPIO.output(hatBackward, GPIO.LOW)
GPIO.output(hatLeft, GPIO.LOW)
GPIO.output(hatRight, GPIO.LOW)




# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096
# Setting up Servo driver
def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz


###  Jostick stuff starts here ###

# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('c', ['\0'] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tostring()
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print '%d axes found: %s' % (num_axes, ', '.join(axis_map))
print '%d buttons found: %s' % (num_buttons, ', '.join(button_map))


# Reading the joystick device loop
def loop_a():
    try:
        stick_mode = 'direct_follow'
        last_y_value = '0'
        gear_tapped = 'no'
        vGear = 0
        vGear_value = 0
        while True:
            evbuf = jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                    print "(initial)",

                # Actions for BUTTON PRESS values are 1(pressed) or 0(released)
                if type & 0x01:
                    button = button_map[number]
                    if button:
                        button_states[button] = value
                        if value:
                            print "%s pressed" % (button)
                        else:
                            print "%s released" % (button)

                    # Thumb Button
                    if button is 'thumb2':
                        button_states[button] = value
                        if value == 1:
                            print value #"%s pressed" % (button)
                            pwm.setPWM(1, 0, 375)
                            last_y_value = []
                        else:
                            print value #"%s released" % (button)                    

                    # Thumb Button2
                    if button is 'thumb':
                        button_states[button] = value
                        if value == 1:
                            if stick_mode is 'direct_follow':
                                stick_mode = 'virtual_gears'
                                print "Stick Mode changed to: ", stick_mode
                                pwm.setPWM(1, 0, 375)
                                gear_tapped = 'no'
                            else:
                                stick_mode = 'direct_follow'
                                print "Stick Mode changed to: ", stick_mode
                                pwm.setPWM(1, 0, int(fvalue))
                                

                # Actions for AXIS movement
                if type & 0x02:
                    axis = axis_map[number]
                    # output all axis values
                    #if axis:
                        #fvalue = value / 32767.0
                        #fvalue = value / 145.631 + 375 # full range from 150 to 600
                        #fvalue = value / 262.136 + 375 # usable range from 250 to 500
                        #axis_states[axis] = fvalue
                        #print "%s: %.2f" % (axis, fvalue)

                    #Stick Forward and Backward
                    if axis is 'y':
                        #fvalue = value / 145.631 + 375                    

                        #  CRUISE MODE NOT WORKING LOGIC
                        if stick_mode is 'cruise':

                            if (value < 0) and (value < last_y_value):
                                print "RISING: ", "last_y_value:", last_y_value, "new value:", value, "f value:", int(fvalue)
                                last_y_value = value
                                fvalue = last_y_value / 145.631 + 375
                                pwm.setPWM(1, 0, int(fvalue))
                                #axis_states[axis] = fvalue
                            elif (value > 0) and (value > last_y_value) and (last_y_value < 0):
                                print "FALLING: ", "last_y_value:", last_y_value, "new value:", value, "f value:", int(fvalue)
                                last_y_value = last_y_value + value / 2
                                fvalue = last_y_value / 145.631 + 375
                                pwm.setPWM(1, 0, int(fvalue))
                            else:
                                what = 0
                                #print stick_mode, "last_y_value:", last_y_value, "new value:", value, "f value:", int(fvalue)

                                
                                #pwm.setPWM(1, 0, int(fvalue))
                            #elif (fvalue > 375) and ()
                        # Virtual Gears    
                        elif stick_mode is 'virtual_gears':
                            #print stick_mode, value
                            if (value < -20000) and (gear_tapped is 'no') and (-10 <= vGear <= 9):
                                vGear = vGear + 1
                                fvalue = (vGear_value - 3276.7) / 145.631 + 375
                                vGear_value = (vGear_value - 3276.7)
                                pwm.setPWM(1, 0, int(fvalue))
                                gear_tapped = 'yes'
                                print "vGear:", vGear, fvalue
                            elif (value > 20000) and (gear_tapped is 'no') and (-9 <= vGear <= 10):
                                vGear = vGear - 1
                                fvalue = (vGear_value + 3276.7) / 145.631 + 375
                                vGear_value = (vGear_value + 3276.7)
                                pwm.setPWM(1, 0, int(fvalue))
                                gear_tapped = 'yes'                        
                                print "vGear:", vGear, fvalue
                            elif -1000 <= value <= 1000:
                                gear_tapped = 'no'

                            else:

                                what = 0

                                
                                
                            
                        else:
                            print stick_mode, "last_y_value:", last_y_value, "new value:", value, "f value:", int(fvalue)
                            fvalue = value / 145.631 + 375
                            #axis_states[axis] = fvalue
                            last_y_value = value
                            pwm.setPWM(1, 0, int(fvalue))



                    #Stick Left and Right
                    if axis is 'x':
                        fvalue = (value * -1) / 262.136 + 375
                        axis_states[axis] = fvalue
                        pwm.setPWM(2, 0, int(fvalue))

                    # Throttle Position
                    if axis is 'z':
                        fvalue = (value * -1) / 262.136 + 375
                        axis_states[axis] = fvalue
                        print "%s: %.0f" % (axis, fvalue)
                        pwm.setPWM(0, 0, int(fvalue))    

                    # Hat Forward and Backward
                    if axis is 'hat0y':
                        fvalue = value / 32767.0
                        axis_states[axis] = fvalue
                        if fvalue > 0:
                            print "HAT BACKWARD"
                            GPIO.output(hatBackward, GPIO.HIGH)
                        elif fvalue < 0:
                            print "HAT FORWARD"
                            GPIO.output(hatForward, GPIO.HIGH)
                        else:
                            print "HAT CENTER "
                            GPIO.output(hatBackward, GPIO.LOW)
                            GPIO.output(hatForward, GPIO.LOW) 
                        
                    # Hat Left and Right
                    if axis is 'hat0x':
                        fvalue = value / 32767.0
                        axis_states[axis] = fvalue
                        if fvalue > 0:
                            print "HAT RIGHT "
                            GPIO.output(hatRight, GPIO.HIGH)
                        elif fvalue < 0:
                            print "HAT LEFT"
                            GPIO.output(hatLeft, GPIO.HIGH)
                        else:
                            print "HAT CENTER "
                            GPIO.output(hatLeft, GPIO.LOW)
                            GPIO.output(hatRight, GPIO.LOW) 
    except KeyboardInterrupt:
        print " You pressed CTRL+C"
    finally:  
        GPIO.cleanup() # this ensures a clean exit

####   END OF THE JOYSTICK STUFF   ####

# Open SPI bus for the MCP3008
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data

# Reading MCP3008 loop
def loop_b():
    try:
        last_pot_value = '0'
        while (True):
            pot_value = ReadChannel(0)
            fpot_value = ((pot_value * 32) + 1) / 93.62 + 200
            #print ((pot_value * 32) + 1) / 93.62 + 200
            pwm.setPWM(3, 0, int(fpot_value))
            if abs(float(last_pot_value)-pot_value) > 3: # get rid of the jitter
                print "last_pot_value:", last_pot_value, "new pot_value:", pot_value
            last_pot_value = pot_value

            time.sleep(.08)
    except KeyboardInterrupt:
        print " You pressed CTRL+C"
    finally:  
        GPIO.cleanup() # this ensures a clean exit 



# Starts running the loops in parallel
if __name__ == '__main__':
    Process(target=loop_a).start()
    Process(target=loop_b).start()
  

