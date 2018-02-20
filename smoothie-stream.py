#!/usr/bin/env python
"""\
Stream g-code to Smoothie telnet connection

Based on GRBL stream.py
"""

from __future__ import print_function
import sys
import telnetlib
import re
import argparse

def write_raw_sequence(tn, seq):
    sock = tn.get_socket()
    if sock is not None:
        sock.send(seq)

# Define command line argument interface
parser = argparse.ArgumentParser(description='Stream g-code file to Smoothie over telnet.')
parser.add_argument('gcode_file', type=argparse.FileType('r'),
        help='g-code filename to be streamed')
parser.add_argument('ipaddr',
        help='Smoothie IP address')
parser.add_argument('-q','--quiet',action='store_true', default=False,
        help='suppress output text')
parser.add_argument('-l','--log',action='store_true', default=False,
        help='suppress output text and output to file (gcode file with .log appended)')
args = parser.parse_args()

f = args.gcode_file
verbose = not (args.quiet or args.log)

# Stream g-code to Smoothie
print("Streaming " + args.gcode_file.name + " to " + args.ipaddr)
outlog = None
if args.log: 
    outlog = open(args.gcode_file.name + ".log", 'w')

tn = telnetlib.Telnet(args.ipaddr)
# turn on prompt
#write_raw_sequence(tn, telnetlib.IAC + telnetlib.DO + "\x55")

# read startup prompt
tn.read_until("Smoothie command shell")

okcnt= 0
linecnt= 0
for line in f:
    tn.write(line)
    linecnt+=1
    rep= tn.read_eager()
    okcnt += rep.count("ok")
    if verbose: print("SND " + str(linecnt) + ": " + line.strip() + " - " + str(okcnt))
    if args.log: outlog.write("SND " + str(linecnt) + ": " + line.strip() + " - " + str(okcnt) + "\n" )
print("Waiting for complete...")

while okcnt < linecnt:
    rep= tn.read_some()
    okcnt += rep.count("ok")
    if verbose: print(str(linecnt) + " - " + str(okcnt) )
    if args.log: outlog.write(str(linecnt) + " - " + str(okcnt) + "\n" )


if args.log: outlog.close()
tn.write("exit\n")
tn.read_all()

print("Done")



