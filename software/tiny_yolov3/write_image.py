import numpy as np
from PIL import Image
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
    c = struct.unpack('i', f.read(4))[0]
    rgbArray = np.zeros((h,w,c), 'uint8')
    
    #Read pixels
    for j in range(h):
        for k in range(w):
            for i in range(c):
                rgbArray[j, k, i] = struct.unpack('B', f.read(1))[0]
            f.read(1) #channel 4
    
    f.close()
    
    #Save image
    img = Image.fromarray(rgbArray)
    img.save(os.path.splitext(filename)[0] + '.png')
    
else:
        print("INVALID ARGUMENT: NEEDS IMAGE INPUT")
        print(len(sys.argv))
