#!/usr/bin/python3

# parses the output of strace and stroes the binary writes to a file

import re, binascii
from struct import *


p = re.compile('read\(\d+, "(.*)", \d+\) = \d+')

fi = open("e1_ts2_short.log", "r")
fo = open("e1_ts2_short.bin", "wb")

for line in fi:
    m = p.match(line)
    data = m.group(1)
    snippets = [data[2+i:4+i] for i in range(0, len(data), 4)]
    for s in snippets:
        b = binascii.unhexlify(s)
        fo.write(b)
