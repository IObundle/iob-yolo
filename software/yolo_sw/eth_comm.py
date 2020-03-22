#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
from time import sleep
import sys
from definitions import *

#Check if argument identifying type of board is present
if len(sys.argv) != 2:
    print("<usage>: python eth_comm.py <val> ")
    print("val = 0 (ALTERA), val = 1 (XILINX)")
    sys.exit()

#Check type of board
if(int(sys.argv[1]) == 1):
    BOARD = "XILINX"
else:
    BOARD = "ALTERA"

#Ethernet parameters
if(BOARD == "ALTERA"):
    interface = "enp0s31f6"
    src_addr = "\x30\x9C\x23\x1E\x62\x4B"   # sender MAC address
else:
    interface = "eno1"
    src_addr = "\x00\x1E\x37\x3A\xE0\x2E"   # sender MAC address
dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
eth_type = "\x08\x00"                       # ethernet frame type
ETH_P_ALL = 0x0800  

#Open files
input_ntw_filename = "../input_fixed.network"
weights_filename = "../yolov3-tiny_batch-fixed.weights"
all_data_filename = '../all_data_fixed.network'
f_in = open(input_ntw_filename, 'rb')
f_weights = open(weights_filename, 'rb')
f_data = open(all_data_filename, 'rb')
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
    
    #Wait some time
    sleep(0.0035)
print("input.network transmitted...")

#######################################################################################
##Check if input.network was well transmitted
#f_in.seek(0)
#count_bytes = 0
#for j in range(num_frames_input_ntw+1):
#    if j == num_frames_input_ntw:
#        bytes_to_receive = input_ntw_file_size - count_bytes
#    else:
#        bytes_to_receive = eth_nbytes
#    payload = f_in.read(bytes_to_receive)
#    count_bytes += eth_nbytes
#    rcv = s.recv(4096)
#    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
#        if sent_byte != rcv_byte:
#            count_errors += 1
#print("DEBUG: input.network sent and received with %d errors\n" % (count_errors))
#######################################################################################

#Reset byte counter
count_bytes = 0
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
    
    #Wait some time
    sleep(0.0035)
print("weights transmitted...")            

#######################################################################################
##Check if weights were well transmitted
#f_weights.seek(0)
#count_bytes = 0
#for j in range(num_frames_weights+1):
#    if j == num_frames_weights:
#        bytes_to_receive = weights_file_size - count_bytes
#    else:
#        bytes_to_receive = eth_nbytes
#    payload = f_weights.read(bytes_to_receive)
#    count_bytes += eth_nbytes
#    rcv = s.recv(4096)
#    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
#        if sent_byte != rcv_byte:
#            count_errors += 1    
#print("DEBUG: weights sent and received with %d errors\n" % (count_errors))
#######################################################################################

#Reset byte counter
count_bytes = 0
print("\nChecking layer output")

layer_file_size = DATA_LAYER_8

#Frame parameters
num_frames_layer = int(layer_file_size/eth_nbytes)
print("layer_file_size: %d" % layer_file_size)     
print("num_frames_layer: %d" % (num_frames_layer+1))

#Check output of layer
pos = NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7
f_data.seek(pos)
for j in range(num_frames_layer+1):
    
    #Check if it is last packet (not enough for full payload)
    if j == num_frames_layer:
        bytes_to_receive = layer_file_size - count_bytes
    else:
        bytes_to_receive = eth_nbytes
        
    #Form frames
    payload = f_data.read(bytes_to_receive)

    #Accumulate sent bytes
    count_bytes += eth_nbytes
    
    #Receve frame
    rcv = s.recv(4096)
    
    #Check if data is correct
    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
        if sent_byte != rcv_byte:
            count_errors += 1  
print("Number of errors in layer: " + str(count_errors))

#Close files
f_in.close()
f_weights.close()
f_data.close() 
