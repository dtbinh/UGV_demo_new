#!/usr/bin/env python
import sys, rospy
import glob
import serial
import time

# Can be used to figure out the port connected to PC
'''def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            #print port
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result'''

def set_command(throttle, steering, truck_id):

    ser = serial.Serial(port="/dev/ttyACM0",
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1,
    interCharTimeout=0.1)

    msg = str(throttle)+','+str(steering)+','+str(truck_id)+','

    ser.flush()
    ser.write(msg)
    ser.close()


if __name__ == '__main__':
    if (len(sys.argv) == 2):
        th= 100
        val=100
        idt=sys.argv[1]
    elif(len(sys.argv)>2):
        th= sys.argv[1]
        val=sys.argv[2]
        idt=sys.argv[3]
    else:
        th= 100
        val=100
        idt=3
    set_command(th, val, idt)
