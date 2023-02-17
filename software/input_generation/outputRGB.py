#Import libraries
from struct import pack,unpack
from math import sqrt
import sys


def uint8_t(val):
    return unpack("B",pack("B",val % 256))[0]

def uint16_t(val):
    return unpack("H",pack("H",val % 65536))[0]

def int16_t(val):
    return unpack("h",pack("h",val % 65536))[0]

def OutputRGB(filename_out):
    f_out = open(filename_out, "wb")

    colors = [[255,0,255],[0,0,255],[0,255,255],[0,255,0],[255,255,0],[255,0,0]]

    zero = 0
    zero = zero.to_bytes(2, byteorder="little")
    file_line = [zero] * 16

    for j in range(80):
        offset = j*123457 % 80;
        mul_16 = uint16_t(uint16_t(offset)*uint16_t(uint8_t(0x10))); #Q8.0 *Q0.8 = Q8.8
        ratio = uint8_t(mul_16>>2); #Q8.8 to Q2.6
        ratio_min = (ratio >> 6);
        ratio_max = ratio_min + 1;
        ratio = ratio & 0x3F;#Q2.6
        mul_16 = uint16_t(uint16_t(0x40-ratio)*uint16_t(colors[ratio_min][2])); #Q2.6 *Q8.0 = Q10.6
        mul_16 += uint16_t(uint16_t(ratio)*uint16_t(colors[ratio_max][2])); #Q2.6 *Q8.0 = Q10.6
        red = (mul_16 >> 6); #Q10.6 to Q8.0
        mul_16 = uint16_t(uint16_t(0x40-ratio)*uint16_t(colors[ratio_min][1])); #Q2.6 *Q8.0 = Q10.6
        mul_16 += uint16_t(uint16_t(ratio)*uint16_t(colors[ratio_max][1])); #Q2.6 *Q8.0 = Q10.6
        green = (mul_16 >> 6); #Q10.6 to Q8.0
        mul_16 = uint16_t(uint16_t(0x40-ratio)*uint16_t(colors[ratio_min][0])); #Q2.6 *Q8.0 = Q10.6
        mul_16 += uint16_t(uint16_t(ratio)*uint16_t(colors[ratio_max][0])); #Q2.6 *Q8.0 = Q10.6
        blue = (mul_16 >> 6); #Q10.6 to Q8.0
    
        file_line[0] = int16_t(red).to_bytes(2, byteorder="little");
        file_line[1] = int16_t(green).to_bytes(2, byteorder="little");
        file_line[2] = int16_t(blue).to_bytes(2, byteorder="little");

        for a in file_line:
            f_out.write(a)

    one = 1
    one = one.to_bytes(2, byteorder="little")
    file_line[0] = one
    file_line[1] = one 
    file_line[2] = one 

    for a in file_line:
        f_out.write(a)

if __name__ == "__main__":
    if(len(sys.argv) != 2):
        print("Format: python <Script> <Output filepath>")
    else:
        OutputRGB(sys.argv[1])