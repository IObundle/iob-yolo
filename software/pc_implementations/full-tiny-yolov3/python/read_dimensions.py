import struct
import sys
import os

#Check file path is an argument
if len(sys.argv)==2:
    
    #Open file
    filename = sys.argv[1]
    f = open(filename, 'rb')
    
    #Read dimensions
    w = struct.unpack('i', f.read(4))[0]
    h = struct.unpack('i', f.read(4))[0]

    #Replace dimensions in header file
    w_string = "\'s/#define IMG_W.*/#define IMG_W %d/\'"%(w)
    h_string = "\'s/#define IMG_H.*/#define IMG_H %d/\'"%(h)
    os.system("sed -i %s src/embedded.h"%(w_string))
    os.system("sed -i %s src/embedded.h"%(h_string))

    #Close file    
    f.close()

else:
    print("INVALID ARGUMENT: NEEDS IMAGE INPUT")
    print(len(sys.argv))