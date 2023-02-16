#Import libraries
from struct import pack,unpack
from math import sqrt
import sys

#YOLOv3-Tiny configuration of convolutional layers
conv_conf = [
        {'n' : 16, 'batch_normalize' : 1, 'num' : 432, 'shift' : 10, 'b_shift' : 11}, #Q6.10, Q5.11 0
        {'n' : 32, 'batch_normalize' : 1, 'num' : 4608, 'shift' : 15, 'b_shift' : 12}, #Q1.15 1
        {'n' : 64, 'batch_normalize' : 1, 'num' : 18432, 'shift' : 14, 'b_shift' : 11}, #Q2.14 2
        {'n' : 128, 'batch_normalize' : 1, 'num' : 73728, 'shift' : 15, 'b_shift' : 12}, #Q1.15 3
        {'n' : 256, 'batch_normalize' : 1, 'num' : 294912, 'shift' : 15, 'b_shift' : 12}, #Q1.15 4
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 15, 'b_shift' : 12}, #Q1.15 5
        {'n' : 1024, 'batch_normalize' : 1, 'num' : 4718592, 'shift' : 14, 'b_shift' : 12}, #Q2.14 6
        {'n' : 256, 'batch_normalize' : 1, 'num' : 262144, 'shift' : 15, 'b_shift' : 14}, #Q1.15 7
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 15, 'b_shift' : 13}, #Q1.15 8
        {'n' : 255, 'batch_normalize' : 0, 'num' : 130560, 'shift' : 14, 'b_shift' : 13}, #Q2.14 9
        {'n' : 128, 'batch_normalize' : 1, 'num' : 32768, 'shift' : 14, 'b_shift' : 13}, #Q2.14 10
        {'n' : 256, 'batch_normalize' : 1, 'num' : 884736, 'shift' : 15, 'b_shift' : 13}, #Q1.15 11
        {'n' : 255, 'batch_normalize' : 0, 'num' : 65280, 'shift' : 15, 'b_shift' : 12} #Q1.15 12
]

def TransformWeights(filename_in,filename_out):
    f_out = open(filename_out, "wb")

    # Weights
    f_in = open(filename_in, 'rb')

    f_in.read(4)
    f_in.read(4)
    f_in.read(4)
    f_in.read(8)

    #Constant
    epsilon = 1e-6

    #Iterate through conv configs
    for i in range(len(conv_conf)):
        
        #Init arrays
        init_bias_arr = []
        scale_arr = []
        mean_arr = []
        variance_arr = []
        new_bias_arr = []
        new_scale_arr = []
        
        weights_arr = []
        bias_arr = []

        #Read bias
        for j in range(conv_conf[i]['n']):
            init_bias_arr.append(unpack('f', f_in.read(4))[0])
            
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
                new_bias_arr.append(init_bias_arr[j]-term)
            
            #Store new weight (multiplication between formed weight with new_scale)
            for k in range(conv_conf[i]['n']):
                for j in range(int(conv_conf[i]['num']/conv_conf[i]['n'])):
                    floatValue = unpack('f', f_in.read(4))[0]
                    new_weight = floatValue*new_scale_arr[k]

                    new_weight = unpack('f',pack('f',new_weight))[0]

                    if(i == 12):
                        new_weight = new_weight / 2

                    val_fixed = int(new_weight * (1 << conv_conf[i]['shift'])) & 0xFFFF
                    weights_arr.append(val_fixed.to_bytes(2, byteorder="little"))
                
            #Store new bias
            for j in range(conv_conf[i]['n']):
                val = unpack('f',pack('f',new_bias_arr[j]))[0]

                val_fixed = int(val * (1 << conv_conf[i]['b_shift'])) & 0xFFFF
                bias_arr.append(val_fixed.to_bytes(2, byteorder="little"))
                
        #If not, simply store original weights and bias
        else:
            
            #Store weights
            for j in range(conv_conf[i]['num']):
                new_weight = unpack('f', f_in.read(4))[0]

                new_weight = unpack('f',pack('f',new_weight))[0]

                if(i == 12):
                    new_weight = new_weight / 2
                val_fixed = int(new_weight * (1 << conv_conf[i]['shift'])) & 0xFFFF
                weights_arr.append(val_fixed.to_bytes(2, byteorder="little"))
            
            #Store bias
            for j in range(conv_conf[i]['n']):
                val = unpack('f',pack('f',init_bias_arr[j]))[0]

                val_fixed = int(init_bias_arr[j] * (1 << conv_conf[i]['b_shift'])) & 0xFFFF
                bias_arr.append(val_fixed.to_bytes(2, byteorder="little"))

        print(i)

        zero = 0
        zero = zero.to_bytes(2, byteorder="little")
        if(i == 0):
            for j in bias_arr:
                f_out.write(j)

            for j in range(len(weights_arr)//27):
                w = weights_arr[j*27:(j+1)*27]

                for k in range(9):
                    f_out.write(w[k])
                    f_out.write(w[k+9])
                    f_out.write(w[k+18])
                    f_out.write(zero)

                for k in range(12):
                    f_out.write(zero)
        elif(i < 7 or i == 8):
            r = conv_conf[i]['n'] // 2
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for l in range(p*16,(p+1)*16):
                    w = weights_arr[l*9*r:]
                    for k in range(9): # The number of lines
                        for j in range(r): # The characters for four lines
                            val = w[j*9+k]
                            f_out.write(val)
        elif(i == 7):
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for j in weights_arr[p*16384:(p+1)*16384]:
                    f_out.write(j)
        elif(i == 9):
            # from 0 to 14 (255 // 16 = 15)
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for j in weights_arr[p*8192:(p+1)*8192]:
                    f_out.write(j)

            for j in bias_arr[15*16:]:
                f_out.write(j)

            f_out.write(zero)

            for j in weights_arr[15*8192:]:
                f_out.write(j)

            for j in range(32*16):
                f_out.write(zero)
        elif(i == 10):
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for j in weights_arr[p*4096:(p+1)*4096]:
                    f_out.write(j)
        elif(i == 11):
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for l in range(p*16,(p+1)*16):
                    w = weights_arr[l*9*48*8:]
                    for k in range(9): # The number of lines
                        for j in range(48*8): # The characters for four lines
                            val = w[j*9+k]
                            f_out.write(val)
        elif(i == 12):
            # from 0 to 14 (255 // 16 = 15)
            for p in range(conv_conf[i]['n'] // 16):
                for j in bias_arr[p*16:(p+1)*16]:
                    f_out.write(j)

                for j in weights_arr[p*4096:(p+1)*4096]:
                    f_out.write(j)

            for j in bias_arr[15*16:]:
                f_out.write(j)

            f_out.write(zero)

            for j in weights_arr[15*4096:]:
                f_out.write(j)

            for j in range(32*8):
                f_out.write(zero)

    f_in.close()
    f_out.close()

if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("Format: python <Script> <Weights in filepath> <Transformed weights out filepath>")
    else:
        TransformWeights(sys.argv[1],sys.argv[2])