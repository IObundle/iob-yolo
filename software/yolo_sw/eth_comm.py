#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
from time import sleep
import sys
from definitions import *

#Check if argument identifying type of board is present
if len(sys.argv) != 3:
    print("<usage>: python eth_comm.py <interface> <RMAC> ")
    sys.exit()

#Ethernet parameters
interface = sys.argv[1]
src_addr = bytearray.fromhex(sys.argv[2])   # sender MAC address
dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
eth_type = "\x08\x00"                       # ethernet frame type
ETH_P_ALL = 0x0800  

#Open files
input_ntw_filename = "../input_fixed.network"
weights_filename = "../yolov3-tiny_batch-fixed.weights"
all_data_filename = '../all_data_fixed.network'
output_ntw_filename = '../output_fixed.network'
f_in = open(input_ntw_filename, 'rb')
f_weights = open(weights_filename, 'rb')
f_data = open(all_data_filename, 'rb')
f_out = open(output_ntw_filename, "rb")
input_ntw_file_size = getsize(input_ntw_filename)
weights_file_size = getsize(weights_filename)

#Frame parameters
eth_nbytes = 1024-18
num_frames_input_ntw = int(input_ntw_file_size/eth_nbytes)
num_frames_weights = int(weights_file_size/eth_nbytes)
print("input_ntw_file_size: %d" % input_ntw_file_size)     
print("num_frames_input_ntw: %d" % (num_frames_input_ntw+1))
print("weights_file_size: %d" % weights_file_size)     
print("num_frames_weights: %d" % (num_frames_weights+1))
    
#Open socket and bind
s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
s.bind((interface, 0))
print("\nStarting input.network transmission...")

#Counters
count_bytes = 0
count_errors = 0

# Loop to send input.network frames
for j in range(num_frames_input_ntw+1):
    
    # check if it is last packet (not enough for full payload)
    if j == num_frames_input_ntw:
        bytes_to_send = input_ntw_file_size - count_bytes
        padding = '\x00' * (eth_nbytes-bytes_to_send)
    else:
        bytes_to_send = eth_nbytes
        padding = ''

    #form frame
    payload = f_in.read(bytes_to_send)
        
    # accumulate sent bytes
    count_bytes += eth_nbytes

    #Send packet
    s.send(dst_addr + src_addr + eth_type + payload + padding)
    
    #receive data back as ack
    rcv = s.recv(4096)
    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_send+14]):
        if sent_byte != rcv_byte:
            count_errors += 1

print("input.network transmitted with %d errors..." %(count_errors))

#Reset byte counter
count_bytes = 0
count_errors = 0
print("\nStarting input weight transmission...")

# Loop to send weights frames
for j in range(num_frames_weights+1):
    
    # check if it is last packet (not enough for full payload)
    if j == num_frames_weights:
        bytes_to_send = weights_file_size - count_bytes
        padding = '\x00' * (eth_nbytes-bytes_to_send)
    else:
        bytes_to_send = eth_nbytes
        padding = ''

    #form frame
    payload = f_weights.read(bytes_to_send)
        
    # accumulate sent bytes
    count_bytes += eth_nbytes

    #Send packet
    s.send(dst_addr + src_addr + eth_type + payload + padding)
    
    #receive data back as ack
    rcv = s.recv(4096)
    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_send+14]):
        if sent_byte != rcv_byte:
            count_errors += 1

print("weights transmitted with %d errors..." %(count_errors))

#Reset error counter
count_errors = 0
print("\nStarting reception of yolo layers...")
yolo_layer_file_size_arr = [DATA_LAYER_17, DATA_LAYER_24]

#Loop to receive yolo layers output frames 
for k in range(len(yolo_layer_file_size_arr)):
    
    #Reset byte counter
    count_bytes = 0
    
    #Frame parameters
    num_frames_yolo_layer = int(yolo_layer_file_size_arr[k]/eth_nbytes)
    print("layer_file_size: %d" % yolo_layer_file_size_arr[k])     
    print("num_frames_yolo_layer: %d" % (num_frames_yolo_layer+1))
    
    #Loop to receive one yolo layer output
    for j in range(num_frames_yolo_layer+1):
    
        #Check if it is last packet (not enough for full payload)
        if j == num_frames_yolo_layer:
            bytes_to_receive = yolo_layer_file_size_arr[k] - count_bytes
        else:
            bytes_to_receive = eth_nbytes
    
        #Form frames
        payload = f_out.read(bytes_to_receive)
    
        #Accumulate sent bytes
        count_bytes += eth_nbytes
        
        #Receve frame
        rcv = s.recv(4096)
        
        #Check if data is correct
        for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
            if sent_byte != rcv_byte:
                count_errors += 1  
print("Number of errors in yolo layers output: " + str(count_errors))

#######################################################################################
##Reset byte counter
#count_bytes = 0
#print("\nChecking layer output")
#
#layer_file_size = DATA_LAYER_17
#
##Frame parameters
#num_frames_layer = int(layer_file_size/eth_nbytes)
#print("layer_file_size: %d" % layer_file_size)     
#print("num_frames_layer: %d" % (num_frames_layer+1))
#
##Check output of layer
#pos = NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16
#f_data.seek(pos)
#for j in range(num_frames_layer+1):
#    
#    #Check if it is last packet (not enough for full payload)
#    if j == num_frames_layer:
#        bytes_to_receive = layer_file_size - count_bytes
#    else:
#        bytes_to_receive = eth_nbytes
#        
#    #Form frames
#    payload = f_data.read(bytes_to_receive)
#
#    #Accumulate sent bytes
#    count_bytes += eth_nbytes
#    
#    #Receve frame
#    rcv = s.recv(4096)
#    
#    #Check if data is correct
#    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
#        if sent_byte != rcv_byte:
#            count_errors += 1  
#print("Number of errors in layer: " + str(count_errors))
#######################################################################################

#Close files
f_in.close()
f_weights.close()
f_data.close() 
f_out.close()
