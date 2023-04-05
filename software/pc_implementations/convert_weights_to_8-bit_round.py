#Import libraries
from struct import unpack

#Filenames
filename_in = 'yolov3-tiny_batch-float.weights'
filename_out = 'yolov3-tiny_batch-fp28bit.weights'

#YOLOv3-Tiny configuration of convolutional layers
conv_conf = [
        {'n' : 16, 'batch_normalize' : 1, 'num' : 432, 'shift' : 2, 'b_shift' : 3}, #Q6.2, Q5.3
        {'n' : 32, 'batch_normalize' : 1, 'num' : 4608, 'shift' : 7, 'b_shift' : 4}, #Q1.7, Q4.4
        {'n' : 64, 'batch_normalize' : 1, 'num' : 18432, 'shift' : 6, 'b_shift' : 3}, #Q2.6, Q5.3
        {'n' : 128, 'batch_normalize' : 1, 'num' : 73728, 'shift' : 7, 'b_shift' : 4}, #Q1.7, Q4.4
        {'n' : 256, 'batch_normalize' : 1, 'num' : 294912, 'shift' : 7, 'b_shift' : 4}, #Q1.7, Q4.4
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 7, 'b_shift' : 4}, #Q1.7, Q4.4
        {'n' : 1024, 'batch_normalize' : 1, 'num' : 4718592, 'shift' : 6, 'b_shift' : 4}, #Q2.6, Q4.4
        {'n' : 256, 'batch_normalize' : 1, 'num' : 262144, 'shift' : 7, 'b_shift' : 6}, #Q1.7, Q2.6
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 7, 'b_shift' : 5}, #Q1.7, Q3.5
        {'n' : 255, 'batch_normalize' : 0, 'num' : 130560, 'shift' : 6, 'b_shift' : 5}, #Q2.6, Q3.5
        {'n' : 128, 'batch_normalize' : 1, 'num' : 32768, 'shift' : 6, 'b_shift' : 5}, #Q2.6, Q3.5
        {'n' : 256, 'batch_normalize' : 1, 'num' : 884736, 'shift' : 7, 'b_shift' : 5}, #Q1.7, Q3.5
        {'n' : 255, 'batch_normalize' : 0, 'num' : 65280, 'shift' : 7, 'b_shift' : 4} #Q1.7, Q4.4
]

#Open files
f_in = open(filename_in, 'rb')
f_out = open(filename_out, "wb")

#_max=0.0
#_min=0.0

#Iterate through conv configs
for i in range(len(conv_conf)):
                
	#Convert weights to Q(16-shift).shift	
	for j in range(conv_conf[i]['num']):
		val = unpack('f', f_in.read(4))[0]
		#if val > _max:
		#	_max=val
		#if val<_min:
		#	_min=val
		val_fixed = int(val * (1 << conv_conf[i]['shift']))
		#val_fixed += int(val * (1 << conv_conf[i]['shift']+1)) & 1
		if val_fixed >= 128:
			val_fixed = 0x7F
		if val_fixed <= -128:
			val_fixed = 0x80
		
		val_fixed &= 0xFF
		f_out.write(val_fixed.to_bytes(1, byteorder="little"))
		print(f'original: {val:.5}\t8-bit: {val_fixed:b}')
    
	#print(i, ":", "max =",_max, "\tmin =", _min)
	#_max=0
	#_min=0       
	#Convert bias to Q(16-b_shift).b_shift	
	for j in range(conv_conf[i]['n']):
		val = unpack('f', f_in.read(4))[0]
		val_fixed = int(val * (1 << conv_conf[i]['b_shift']))
		
		#val_fixed += int(val * (1<< conv_conf[i]['b_shift']+1)) & 1
		if val_fixed >= 128:
			val_fixed = 0x7F
		if val_fixed <= -128:
			val_fixed = 0x80
		
		val_fixed &= 0xFF
		f_out.write(val_fixed.to_bytes(1, byteorder="little"))  
		print(f'original: {val:.5}\t8-bit: {val_fixed:b}')

#Close files  
f_in.close()
f_out.close()
