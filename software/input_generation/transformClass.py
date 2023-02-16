#Import libraries
from struct import pack,unpack
from math import sqrt
import sys

def TransformClass(filename_in,filename_out):
    f_out = open(filename_out, "wb")

    # Class
    f_in = open(filename_in,"rb")

    data = f_in.read()

    zero = 0
    zero = zero.to_bytes(1,byteorder="little")

    for i in range(81):
        f_out.write(data[i].to_bytes(1,byteorder="little"))
        f_out.write(zero)

    for i in range(15*2):
        f_out.write(zero)

    data = data[81:]

    points = []
    values = [56,60,27,86,81,30,40,45,37,96,98,75,116,50,35,26,31,46,48,34,71,37,46,56,79,75,70,23,67,58,34,89,87,33,99,117,90,80,106,49,87,31,36,43,50,40,59,46,78,57,69,50,63,45,48,39,43,36,94,31,99,45,83,53,55,59,77,86,90,39,57,37,96,43,47,38,66,87,78,89,12]

    for v in values:
        for x in range(20):
            points.append(v)

    for p in points:
        for i in range(p):
            d = data[i].to_bytes(1,byteorder="little")

            f_out.write(d)
            f_out.write(zero)
            f_out.write(d)
            f_out.write(zero)
            f_out.write(d)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)

        data = data[p:]

        for i in range(120 - p):
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)
            f_out.write(zero)

    f_in.close()

if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("Format: python <Script> <Class in filepath> <Class out filepath>")
    else:
        TransformClass(sys.argv[1],sys.argv[2])