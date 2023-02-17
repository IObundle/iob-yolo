#import libraries
import struct

#Config parameters
NUM_IMAGES = 20
NUM_LAYERS = 13

#max and min arrays
max_arr = [-1000000]*NUM_LAYERS
min_arr = [1000000]*NUM_LAYERS

#Open file
f_in = open('../max_min.hex', 'rb')

#Loop to read max and min of each image
for i in range(NUM_IMAGES):
    
    #Loop to read max and min of each image layer
    for j in range(NUM_LAYERS):
        
        #Check max
        max_val = struct.unpack('f', f_in.read(4))[0]
        if(max_arr[j] < max_val):
            max_arr[j] = max_val
        
        #Check min
        min_val = struct.unpack('f', f_in.read(4))[0]
        if(min_arr[j] > min_val):
            min_arr[j] = min_val

#Print final max and min of each layer
for j in range(NUM_LAYERS):
    print("Layer %d: max = %f, min = %f"%(j, max_arr[j], min_arr[j]))

#Close file
f_in.close()