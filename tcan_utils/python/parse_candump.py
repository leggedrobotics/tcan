#!/usr/bin/env python

import argparse
import re
from operator import itemgetter

stuffBitsFactor = 1.2
numHeaderBits = 47
messageGap = 10 # todo: ??


parser = argparse.ArgumentParser()
parser.add_argument("file", help="File output by 'candump -tz' or similar.", type=str)
parser.add_argument("-s", dest="start", help="Start time of bps calculation", type=float)
parser.add_argument("-g", dest="messageGap", help="Number of bits between to consecutive messages. This is heavily dependent on the used driver (e.g. peak netdev driver or linux kernel socketcan driver). Todo: use microseconds?", type=int)


args = parser.parse_args()


file = open(args.file, 'r')

lengths = []
timestamps = []
ids = []
occurences = dict()
bytes_per_message = dict()
tot_bytes = 0

for line in file:
    m = re.match("( *)\((?P<_0>.+)\)( *)(.*)( *)(?P<_1>[0-9a-fA-F]{3})( *)\[(?P<_2>[0-9]+)\](.*)$", line)
    if m:
        data = map(itemgetter(1), sorted(m.groupdict().items()))
        time = float(data[0].strip())
        id = int(data[1].strip(), 16)
        length = int(data[2].strip())
        
        tot_bytes += length
        if id in bytes_per_message:
            bytes_per_message[id] += length
            occurences[id] += 1
        else:
            bytes_per_message[id] = length
            occurences[id] = 1
                             
        ids.append(id)
        lengths.append(length)
        timestamps.append(time)

#
print(str(tot_bytes) + ' bytes sent in ' + str(timestamps[-1]) + ' seconds in ' + str(len(bytes_per_message)) + ' different messages.')
print('  ' + "id".ljust(6)+": "+"bytes".ljust(10) + '(#messages)')
sorted_dict = sorted(bytes_per_message.iteritems());
for key, value in sorted_dict:
    print('  ' + str(hex(key)).ljust(6)+": "+str(value).ljust(10) + '(' + str(occurences[key]) + ')')

# (also see http://www.esacademy.com/en/library/calculators/can-best-and-worst-case-calculator.html)
bps = ((tot_bytes*8+numHeaderBits)*stuffBitsFactor + messageGap) / timestamps[-1]
print('bps (total): ' + str(bps))

#
intervalBytes = 0
startIndex = timestamps.index(min(timestamps, key=lambda x:abs(x-args.start)))
endIndex = timestamps.index(min(timestamps, key=lambda x:abs(x-(args.start+1.0))))
for length in lengths[startIndex:endIndex]:
    intervalBytes += length
bpsInterval = ((intervalBytes*8+numHeaderBits)*stuffBitsFactor + messageGap) / (timestamps[endIndex] - timestamps[startIndex])
print('bps (interval ' + str(timestamps[startIndex]) + '-' + str(timestamps[endIndex]) + '/' + str(startIndex) + '-' + str(endIndex) + '): ' + str(bpsInterval))


file.close()
