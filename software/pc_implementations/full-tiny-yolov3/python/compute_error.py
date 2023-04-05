from struct import unpack
import numpy as np

# Filenames
filename_8b = '../yolov3-tiny_8-bit_conv.hex'
filename_fp = '../yolov3-tiny_float_conv.hex'

#########################################################
#                      Network Info
#
# width:     output FM width
# kernels:   number of kernels
# f_weights: number of fractional bits in the weights
# f_in:      number of fractional bits in the input FM
# f_out:     number of fractional bits in the out put FM

net_conf = [{'size': 2768896, 'f_weights': 10, 'f_in': 15, 'shift': 16},
			{'size': 1384448, 'f_weights': 15, 'f_in': 0, 'shift': 14},
			{'size': 692224, 'f_weights': 14, 'f_in': 0, 'shift': 14},
			{'size': 346112, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 15},
			{'size': 86528, 'f_weights': 15, 'f_in': 0, 'shift': 14},
			{'size': 173056, 'f_weights': 14, 'f_in': 0, 'shift': 16},
			{'size': 43264, 'f_weights': 15, 'f_in': 0, 'shift': 12},
			{'size': 86528, 'f_weights': 15, 'f_in': 0, 'shift': 16},
			{'size': 43264, 'f_weights': 14, 'f_in': 0, 'shift': 11},
			{'size': 21632, 'f_weights': 14, 'f_in': 0, 'shift': 14},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 16},
			{'size': 173056, 'f_weights': 15, 'f_in': 0, 'shift': 12},
			]
#########################################################

# Calculate FM fractional bits for conversion
print('Network Configuration:')
net_conf[0]['f_in'] -= 8
for i in range(len(net_conf)):
	net_conf[i]['shift'] -= 8
	net_conf[i]['f_weights'] -= 8
	if i==10:
		net_conf[i]['f_in'] = net_conf[7]['f_out']
	elif i>0:
		net_conf[i]['f_in'] = net_conf[i-1]['f_out']
	net_conf[i]['f_out'] = net_conf[i]['f_weights']+net_conf[i]['f_in']-net_conf[i]['shift']
	print(i, net_conf[i])

# Open files
file_8b = open(filename_8b, 'rb')
file_fp = open(filename_fp, 'rb')

# Compute maximum error for each convolutional layer
for l in range(len(net_conf)): # l is for 'layer'
#for l in range(1): # l is for 'layer'
	fm_fp=np.zeros(net_conf[l]['size'])
	fm_8b=np.zeros(net_conf[l]['size'])
	
	print('=================================================')
	print('                    LAYER', l+1,'\n')
	print('Format: Q'+str(8-net_conf[l]['f_out'])+'.'+str(net_conf[l]['f_out']))
	
	for p in range(net_conf[l]['size']): # p is for 'pixel'
		val_8b = unpack('b', file_8b.read(1))[0]
		fbuf = file_fp.read(4)
		#print('read the following bytes from the file: ',fbuf.hex())
		val_fp = unpack('<f', fbuf)[0]
		#print('unpacked value:', val_fp)
		
		val_8b_conv = float(val_8b)/(2**(net_conf[l]['f_out']))
		#print('corresponding 8-bit value after shift:', val_8b_conv)
		#exit()

		fm_fp[p] = val_fp
		fm_8b[p] = val_8b_conv
	
	idx = np.argmax(fm_fp)
	max_fp = fm_fp[idx]
	max_8b = fm_8b[idx]
	#max_8b = max(fm_8b)
	idx = np.argmin(fm_fp)
	min_fp = fm_fp[idx]
	min_8b = fm_8b[idx]
	#min_8b = min(fm_8b)

	error = abs(fm_fp-fm_8b)
	err_ordered_idx = np.argsort(error, axis=0)
	
	idx = err_ordered_idx[-1] #np.argmax(error)
	err_max = error[idx]
	err_max_fpval = fm_fp[idx]
	err_max_8bval = fm_8b[idx]
	err_avg = np.mean(abs(fm_fp-fm_8b))

	print('Greatest error values:' )
	for i in (np.arange(5)+1):
		print(f'fp: {fm_fp[err_ordered_idx[-i]]:.4f}, 8b: {fm_8b[err_ordered_idx[-i]]:.4f} (index={err_ordered_idx[-i]:d})')

	print('------------------------------------------------')
	print('Maximum and minimum values:')
	print('min_fp',min_fp,'\t','min_8b',min_8b)
	print('max_fp',max_fp,'\t','max_8b',max_8b)
	print('------------------------------------------------')
	print('\nMax error:',err_max,f'(fp: {err_max_fpval:.4f}, 8b: {err_max_8bval:.4f})')
	print('Mean error:',err_avg,'\n')

# Close files
file_fp.close()
file_8b.close()
