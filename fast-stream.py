#!/usr/bin/env python
"""\
Stream g-code to Smoothie USB serial connection

Based on GRBL stream.py, but completely different
"""

from __future__ import print_function
import sys
import argparse
import serial
import threading
import time
import signal
import sys
 
errorflg= False
intrflg= False

def signal_term_handler(signal, frame):
   global intrflg
   print('got SIGTERM...')
   intrflg= True
 
signal.signal(signal.SIGTERM, signal_term_handler)

# Define command line argument interface
parser = argparse.ArgumentParser(description='Stream g-code file to Smoothie over telnet.')
parser.add_argument('gcode_file', type=argparse.FileType('r'),
        help='g-code filename to be streamed')
parser.add_argument('device',
        help='Smoothie Serial Device')
parser.add_argument('-q','--quiet',action='store_true', default=False,
        help='suppress output text')
args = parser.parse_args()

f = args.gcode_file
verbose = not args.quiet

# Stream g-code to Smoothie

dev= args.device

# Open port
s = serial.Serial(dev, 115200)
s.flushInput()  # Flush startup text in serial input

print("Streaming " + args.gcode_file.name + " to " + args.device)

okcnt= 0

def read_thread():
    """thread worker function"""
    global okcnt, errorflg
    flag= 1
    while flag :
        rep= s.readline()
        n= rep.count("ok")
        if n == 0 :
            print("Incoming: " + rep)
            if "error" in rep or "!!" in rep or "ALARM" in rep or "ERROR" in rep:
                errorflg= True
                break
        else :
            okcnt += n

    print("Read thread exited")
    return

# start read thread
t = threading.Thread(target=read_thread)
t.daemon = True
t.start()

linecnt= 0
try:
    for line in f:
        if errorflg :
            break
        # strip comments
        if line.startswith(';') :
            continue
        l= line.strip()
        s.write(l + '\n')
        linecnt+=1
        if verbose: print("SND " + str(linecnt) + ": " + line.strip() + " - " + str(okcnt))
        
except KeyboardInterrupt:
    print("Interrupted...")
    intrflg= True

if intrflg :
    # We need to consume oks otherwise smoothie will deadlock on a full tx buffer
    print("Sending Abort - this may take a while...")
    s.write('\x18') # send halt
    
if errorflg :
    print("Target halted due to errors")

else :
    print("Waiting for complete...")
    while okcnt < linecnt :
        if verbose: print(str(linecnt) + " - " + str(okcnt) )
        if errorflg :
            s.read(s.inWaiting()) # rad all remaining characters
            break
        time.sleep(1)

    # Wait here until finished to close serial port and file.
    raw_input("  Press <Enter> to exit")


# Close file and serial port
f.close()
s.close()
