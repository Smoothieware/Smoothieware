#!/usr/bin/env python
"""\
Stream g-code to Smoothie telnet connection

Based on GRBL stream.py
"""

from __future__ import print_function
import sys
import telnetlib
import argparse

# Define command line argument interface
parser = argparse.ArgumentParser(description='Stream g-code file to Smoothie over telnet.')
parser.add_argument('gcode_file', type=argparse.FileType('r'),
        help='g-code filename to be streamed')
parser.add_argument('ipaddr',
        help='Smoothie IP address')
parser.add_argument('-q','--quiet',action='store_true', default=False,
        help='suppress output text')
args = parser.parse_args()

f = args.gcode_file
verbose = not args.quiet

# Stream g-code to Smoothie
print("Streaming " + args.gcode_file.name + " to " + args.ipaddr)

tn = telnetlib.Telnet(args.ipaddr)
# read startup prompt
tn.read_until("> ")

okcnt= 0
linecnt= 0
for line in f:
    tn.write(line)
    linecnt+=1
    rep= tn.read_eager()
    okcnt += rep.count("ok")
    if verbose: print("SND " + str(linecnt) + ": " + line.strip() + " - " + str(okcnt))

print("Waiting for complete...")

while okcnt < linecnt:
    rep= tn.read_some()
    okcnt += rep.count("ok")
    if verbose: print(str(linecnt) + " - " + str(okcnt) )

tn.write("exit\n")
tn.read_all()

print("Done")



