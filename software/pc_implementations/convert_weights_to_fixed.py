#Import libraries
from struct import unpack

#Filenames
filename_in = 'yolov3-tiny_batch-float.weights'
filename_out = 'yolov3-tiny_batch-fixed__script.weights'

#YOLOv3-Tiny configuration of convolutional layers
conv_conf = [
        {'n' : 16, 'batch_normalize' : 1, 'num' : 432, 'shift' : 10, 'b_shift' : 11}, #Q6.10, Q5.11
        {'n' : 32, 'batch_normalize' : 1, 'num' : 4608, 'shift' : 15, 'b_shift' : 12}, #Q1.15
        {'n' : 64, 'batch_normalize' : 1, 'num' : 18432, 'shift' : 14, 'b_shift' : 11}, #Q2.14
        {'n' : 128, 'batch_normalize' : 1, 'num' : 73728, 'shift' : 15, 'b_shift' : 12}, #Q1.15
        {'n' : 256, 'batch_normalize' : 1, 'num' : 294912, 'shift' : 15, 'b_shift' : 12}, #Q1.15
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 15, 'b_shift' : 12}, #Q1.15
        {'n' : 1024, 'batch_normalize' : 1, 'num' : 4718592, 'shift' : 14, 'b_shift' : 12}, #Q2.14
        {'n' : 256, 'batch_normalize' : 1, 'num' : 262144, 'shift' : 15, 'b_shift' : 14}, #Q1.15
        {'n' : 512, 'batch_normalize' : 1, 'num' : 1179648, 'shift' : 15, 'b_shift' : 13}, #Q1.15
        {'n' : 255, 'batch_normalize' : 0, 'num' : 130560, 'shift' : 14, 'b_shift' : 13}, #Q2.14
        {'n' : 128, 'batch_normalize' : 1, 'num' : 32768, 'shift' : 14, 'b_shift' : 13}, #Q2.14
        {'n' : 256, 'batch_normalize' : 1, 'num' : 884736, 'shift' : 15, 'b_shift' : 13}, #Q1.15
        {'n' : 255, 'batch_normalize' : 0, 'num' : 65280, 'shift' : 15, 'b_shift' : 12} #Q1.15
]

#Open files
f_in = open(filename_in, 'rb')
f_out = open(filename_out, "wb")

_max=0.0
_min=0.0

#Iterate through conv configs
for i in range(len(conv_conf)):
                
	#Convert weights to Q(16-shift).shift	
	for j in range(conv_conf[i]['num']):
		val = unpack('f', f_in.read(4))[0]
		if val > _max:
			_max=val
		if val<_min:
			_min=val
		val_fixed = int(val * (1 << conv_conf[i]['shift'])) & 0xFFFF
		f_out.write(val_fixed.to_bytes(2, byteorder="little"))
    
	print("Layer",i+1,"W:", "max =",_max, "\tmin =", _min, f"\tQ{16-conv_conf[i]['shift']:d}.{conv_conf[i]['shift']:d}")
	_max=0
	_min=0

	#Convert bias to Q(16-b_shift).b_shift	
	for j in range(conv_conf[i]['n']):
		val = unpack('f', f_in.read(4))[0]
		val_fixed = int(val * (1 << conv_conf[i]['b_shift'])) & 0xFFFF
		f_out.write(val_fixed.to_bytes(2, byteorder="little"))  
		if val > _max:
			_max=val
		if val<_min:
			_min=val

	print("Layer",i+1, "B:", "max =",_max, "\tmin =", _min, f"\tQ{16-conv_conf[i]['b_shift']:d}.{conv_conf[i]['b_shift']:d}", "\n")
	_max=0
	_min=0

#Close files  
f_in.close()
f_out.close()
