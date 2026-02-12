#! /usr/bin/env python3

import sys
import math

def next_power_of_2(x):
    return 1 if x == 0 else 2**math.ceil(math.log2(x))

outfile = sys.argv[1]
infile = sys.argv[2]
step_size = int(sys.argv[3], 0)
start_byte = int(sys.argv[4], 0)
length = next_power_of_2(int(sys.argv[5], 0))

data = open(infile, "rb").read()

split_data = data[start_byte::step_size]
real_len = len(split_data)
pow2_len = next_power_of_2(real_len)

if pow2_len > length:
    print(f"{outfile}: input data too large for ROM")
    sys.exit(-1)

print(f"{outfile}: {real_len:#x} bytes of real data.")
if real_len < length:
    delta = length - real_len
    split_data += bytes([0xff] * delta)
    print(f"{outfile}: padding to {length:#x} bytes.")

#if pow2_len < length:
#    split_data = split_data * int( length / pow2_len )
#    print(f"{outfile}: duplicating to {length:#x} bytes.")

with open(outfile, "wb") as fp:
    fp.write(split_data)
