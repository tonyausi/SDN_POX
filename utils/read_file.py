#!/usr/bin/python

'''
Cloud project:
- read bandwidth log csv file 
Tony Lin
'''

import os, sys, getopt
import numpy as np
import re
import csv
from collections import defaultdict as dd
# import all constants
from utils.constant_read_topo import *



# with open('test.csv', 'r') as textfile:
#     for row in reversed(list(csv.reader(textfile))):
#         print ', '.join(row)

# for line in reversed(open("filename").readlines()):
#     print line.rstrip()

# file = open("mycsvfile.csv")
# reversedLines = [line[::-1] for line in file]
# file.close()
# reader = csv.reader(reversedLines)
# for backwardRow in reader:
#     lastField = backwardRow[0][::-1]
#     secondField = backwardRow[1][::-1]
        
            
def reversed_lines(file):
    "Generate the lines of file in reverse order."
    part = ''
    for block in reversed_blocks(file):
        for c in reversed(block):
            if c == '\n' and part:
                yield part[::-1]
                part = ''
            part += c
    if part: yield part[::-1]

def reversed_blocks(file, blocksize=4096):
    "Generate blocks of file's contents in reverse order."
    file.seek(0, os.SEEK_END)        # Seek @ EOF, os.SEEK_END=2
    here = file.tell()               # Get Size
    while 0 < here:
        delta = min(blocksize, here)
        here -= delta
        
# with open('test.csv', 'r') as textfile:
#     for row in csv.reader(reversed_lines(textfile)):
#         print ', '.join(row)
#         file.seek(here, os.SEEK_SET) # os.SEEK_SET=0
#         yield file.read(delta)