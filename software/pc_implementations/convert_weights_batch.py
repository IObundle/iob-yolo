#Import libraries
from struct import pack, unpack
from math import sqrt

#Filenames
filename_in = 'yolov3-tiny.weights'
filename_out = 'yolov3-tiny_batch.weights'

#YOLOv3-Tiny configuration of convolutional layers
conv_conf = [
        {'n' : 16, 'batch_normalize' : 1, 'num' : 432},
        {'n' : 32, 'batch_normalize' : 1, 'num' : 4608},
        {'n' : 64, 'batch_normalize' : 1, 'num' : 18432},
        {'n' : 128, 'batch_normalize' : 1, 'num' : 73728},
        {'n' : 256, 'batch_normalize' : 1, 'num' : 294912},
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648},
        {'n' : 1024, 'batch_normalize' : 1, 'num' : 4718592},
        {'n' : 256, 'batch_normalize' : 1, 'num' : 262144},
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648},
        {'n' : 255, 'batch_normalize' : 0, 'num' : 130560},
        {'n' : 128, 'batch_normalize' : 1, 'num' : 32768},
        {'n' : 256, 'batch_normalize' : 1, 'num' : 884736},
        {'n' : 255, 'batch_normalize' : 0, 'num' : 65280}
]

#Open files
f_in = open(filename_in, 'rb')
f_out = open(filename_out, "wb")

#Read version info
f_in.read(4) #major
f_in.read(4) #minor
f_in.read(4) #revision
f_in.read(8) #seen

#Constant
epsilon = 1e-6

#Iterate through conv configs
for i in range(len(conv_conf)):
    
    #Init arrays
    bias_arr = []
    scale_arr = []
    mean_arr = []
    variance_arr = []
    new_bias_arr = []
    new_scale_arr = []
    
    #Read bias
    for j in range(conv_conf[i]['n']):
        bias_arr.append(unpack('f', f_in.read(4))[0])
        
    #Check if conv has batch normalization
    if(conv_conf[i]['batch_normalize']):
        
        #Read scale
        for j in range(conv_conf[i]['n']):
            scale_arr.append(unpack('f', f_in.read(4))[0])
            
        #Read mean
        for j in range(conv_conf[i]['n']):
            mean_arr.append(unpack('f', f_in.read(4))[0])
        
        #Read variance
        for j in range(conv_conf[i]['n']):
            variance_arr.append(unpack('f', f_in.read(4))[0])
            
        #Determine new bias and scale
        for j in range(conv_conf[i]['n']):
            den = sqrt(variance_arr[j] + epsilon)
            new_scale_arr.append(scale_arr[j]/den)
            term = mean_arr[j]*scale_arr[j]/den
            new_bias_arr.append(bias_arr[j]-term)

    #Check if conv has batch normalization
    if(conv_conf[i]['batch_normalize']):
        
        #Store new weight (multiplication between formed weight with new_scale)
        for k in range(conv_conf[i]['n']):
            for j in range(int(conv_conf[i]['num']/conv_conf[i]['n'])):
                new_weight = unpack('f', f_in.read(4))[0]*new_scale_arr[k]
                f_out.write(pack('f', new_weight))
            
        #Store new bias
        for j in range(conv_conf[i]['n']):
            f_out.write(pack('f', new_bias_arr[j]))
            
    #If not, simply store original weights and bias
    else:
        
        #Store weights
        for j in range(conv_conf[i]['num']):
            f_out.write(pack('f', unpack('f', f_in.read(4))[0]))
        
        #Store bias
        for j in range(conv_conf[i]['n']):
            f_out.write(pack('f', bias_arr[j]))
            
#Close files  
f_in.close()
f_out.close()