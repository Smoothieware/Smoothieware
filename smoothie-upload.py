#!/usr/bin/env python
"""\
Upload a file to Smoothie over the network
"""

from __future__ import print_function
import sys
import argparse
import socket
import os
import re
# Define command line argument interface
parser = argparse.ArgumentParser(description='Upload a file to Smoothie over network.')
parser.add_argument('file', type=argparse.FileType('r'),
        help='filename to be uploaded')
parser.add_argument('ipaddr',
        help='Smoothie IP address')
parser.add_argument('-v','--verbose',action='store_true',
        help='Show data being uploaded')
parser.add_argument('-o','--output',
        help='Set output filename')
parser.add_argument('-q','--quiet',action='store_true',
        help='suppress all output to terminal')
parser.add_argument('-s','--space',action='store_true',
        help='Leave whitespaces in output filename')

args = parser.parse_args()

f = args.file
verbose = args.verbose
output = args.output
if output == None :
    output= args.file.name
if not args.space:
    output = re.sub("\s", "_", output)

filesize= os.path.getsize(args.file.name)

if not args.quiet : print("Uploading " + args.file.name + " to " + args.ipaddr + " as " + output + " size: " + str(filesize) )

# make connection to sftp server
s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(4.0)
s.connect((args.ipaddr, 115))
tn= s.makefile()

# read startup prompt
ln= tn.readline()
if not ln.startswith("+") :
    print("Failed to connect with sftp: " + ln)
    sys.exit();

if verbose: print("RSP: " + ln.strip())

# Issue initial store command
tn.write("STOR OLD /sd/" + output + "\n")
tn.flush()

ln= tn.readline()
if not ln.startswith("+") :
    print("Failed to create file: " + ln)
    sys.exit();

if verbose: print("RSP: " + ln.strip())

# send size of file
tn.write("SIZE " + str(filesize) + "\n")
tn.flush()

ln= tn.readline()
if not ln.startswith("+") :
    print("Failed: " + ln)
    sys.exit();

if verbose: print("RSP: " + ln.strip())

cnt= 0
# now send file
for line in f:
    tn.write(line)
    if verbose :
        print("SND: " + line.strip())
    elif not args.quiet :
        cnt += len(line)
        print(str(cnt) + "/" + str(filesize) + "\r", end='')


tn.flush()

ln= tn.readline()
if not ln.startswith("+") :
    print("Failed to save file: " + ln)
    sys.exit();

if verbose: print("RSP: " + ln.strip())

# exit
tn.write("DONE\n")
tn.flush()
ln= tn.readline()
tn.close()
f.close()

if not args.quiet : print("Upload complete")

