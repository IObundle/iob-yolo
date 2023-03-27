from struct import unpack
import numpy as np

# Filenames
filename_16b = '../yolov3-tiny_16-bit_conv.hex'
filename_fp = '../yolov3-tiny_float_conv.hex'

#########################################################
#                      Network Info
#
# width:     output FM width
# kernels:   number of kernels
# f_weights: number of fractional bits in the weights
# f_in:      number of fractional bits in the input FM
# f_out:     number of fractional bits in the out put FM

net_conf = [{'size': 2768896, 'f_weights': 10, 'f_in': 15, 'shift': 17},
			{'size': 1384448, 'f_weights': 15, 'f_in': 0, 'shift': 14},
			{'size': 692224, 'f_weights': 14, 'f_in': 0, 'shift': 14},
			{'size': 346112, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 86528, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 173056, 'f_weights': 14, 'f_in': 0, 'shift': 15},
			{'size': 43264, 'f_weights': 15, 'f_in': 0, 'shift': 13},
			{'size': 86528, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 43264, 'f_weights': 14, 'f_in': 0, 'shift': 11},
			{'size': 21632, 'f_weights': 14, 'f_in': 0, 'shift': 15},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 14},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 11},
			]
#########################################################

# Calculate FM fractional bits for conversion
for i in range(len(net_conf)):
	if i==10:
		net_conf[i]['f_in'] = net_conf[7]['f_out']
	elif i>0:
		net_conf[i]['f_in'] = net_conf[i-1]['f_out']
	net_conf[i]['f_out'] = net_conf[i]['f_weights']+net_conf[i]['f_in']-net_conf[i]['shift']
	print(i, net_conf[i])

# Open files
file_16b = open(filename_16b, 'rb')
file_fp = open(filename_fp, 'rb')

# Compute maximum error for each convolutional layer
for l in range(len(net_conf)): # l is for 'layer'
	print('=================================================')
	print('                    LAYER', l+1,'\n')
	print('Format: Q'+str(16-net_conf[l]['f_out'])+'.'+str(net_conf[l]['f_out']))

	fm_fp=np.zeros(net_conf[l]['size'])
	fm_16b=np.zeros(net_conf[l]['size'])
	
	for p in range(net_conf[l]['size']): # p is for 'pixel'
		val_16b = unpack('h', file_16b.read(2))[0]
		fbuf = file_fp.read(4)
		#print('read the following bytes from the file: ',fbuf.hex())
		val_fp = unpack('<f', fbuf)[0]
		#print('unpacked value:', val_fp)
		
		val_16b_conv = float(val_16b)/(2**(net_conf[l]['f_out']))
		#print('corresponding 8-bit value after shift:', val_8b_conv)
		#exit()
		
		if l==4:
			print('float:', val_fp,'\tfixed:',val_16b_conv,'\t before conversion:',val_16b)
	
		fm_fp[p] = val_fp
		fm_16b[p] = val_16b_conv
		
	err_max = max(abs(fm_fp-fm_16b))
	err_avg = np.mean(abs(fm_fp-fm_16b))
	idx = np.argmax(fm_fp)
	max_fp = fm_fp[idx]
	max_16b = fm_16b[idx]
	idx = np.argmin(fm_fp)
	min_fp = fm_fp[idx]
	min_16b = fm_16b[idx]

	print('-------------------------------------------------')
	print('Maximum and minimum values:')
	print('min_fp',min_fp,'\t','min_16b',min_16b)
	print('max_fp',max_fp,'\t','max_16b',max_16b)
	print('-------------------------------------------------')
	print('\nMaximum error:',err_max)
	print('Mean error:',err_avg,'\n')

# Close files
file_fp.close()
file_16b.close()
