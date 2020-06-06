#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
import sys
import struct

#Check if argument identifying type of board is present
if len(sys.argv) < 4:
    print("<usage>: python eth_comm.py <interface> <RMAC> <filename_path>")
    sys.exit()

#Ethernet parameters
interface = sys.argv[1]
src_addr = bytearray.fromhex(sys.argv[2])   # sender MAC address
dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
eth_type = "\x08\x00"                       # ethernet frame type
ETH_P_ALL = 0x0800  

#Frame parameters
eth_nbytes = 1024-18
    
#Open socket and bind
s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
s.bind((interface, 0))

################################# SEND IMAGE ##############################################

#Open image file
print("\nStarting input image transmission...")
input_filename = "../dog.bin"
f_in = open(input_filename, 'rb')
image_w = struct.unpack('i', f_in.read(4))[0]
image_h = struct.unpack('i', f_in.read(4))[0]
image_c = struct.unpack('i', f_in.read(4))[0]
print("Input image is %dx%dx%d .." %(image_w, image_h, image_c))

#Frame parameters
input_file_size = getsize(input_filename)-12
num_frames_input = int(input_file_size/eth_nbytes)
print("input_file_size: %d" % input_file_size)     
print("num_frames_input: %d" % (num_frames_input+1))

#Counters
count_bytes = 0
count_errors = 0

# Loop to send input.network frames
for j in range(num_frames_input+1):
    
    # check if it is last packet (not enough for full payload)
    if j == num_frames_input:
        bytes_to_send = input_file_size - count_bytes
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

#Close file
f_in.close()
print("input.network transmitted with %d errors..." %(count_errors))

################################# SEND WEIGHTS ##############################################

#Open weight file
print("\nStarting input weight transmission...")
weights_filename = "../yolov3-tiny_batch-fixed.weights"
f_weights = open(weights_filename, 'rb')

#Frame parameters
#weights_file_size = getsize(weights_filename)
layer1_w = 16 + 16*3*3*3
layer2_w = 32 + 32*3*3*16
layer_w_total = layer1_w + layer2_w
weights_file_size = layer_w_total*2
num_frames_weights = int(weights_file_size/eth_nbytes)
print("weights_file_size: %d" % weights_file_size)
print("num_frames_weights: %d" % (num_frames_weights+1))

#Reset byte counter
count_bytes = 0
count_errors = 0

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

#Close file
f_weights.close()
print("weights transmitted with %d errors..." %(count_errors))

################################# RECEIVE RESULT ##############################################

#Open output image files
print("\nStarting reception of result...")
output_filename = '../result.bin'
f_output = open(output_filename, "rb")

#Reset counters
count_errors = 0
count_bytes = 0

#Frame parameters
output_file_size = getsize(output_filename)
num_frames_output = int(output_file_size/eth_nbytes)
print("output_file_size: %d" % output_file_size)     
print("num_frames_output: %d" % (num_frames_output+1))

#Loop to receive one yolo layer output
for j in range(num_frames_output+1):

    #Check if it is last packet (not enough for full payload)
    if j == num_frames_output:
        bytes_to_receive = output_file_size - count_bytes
        padding = '\x00' * (eth_nbytes-bytes_to_send)
    else:
        bytes_to_receive = eth_nbytes
        padding = ''

    #Accumulate rcv bytes
    count_bytes += eth_nbytes
    
    #form frame
    payload = f_output.read(bytes_to_receive)
    
    #Check if data is correct
    rcv = s.recv(4096)
    for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_receive+14]):
        if sent_byte != rcv_byte:
            count_errors += 1
            
    #Send data back as ack
    if(j != num_frames_output):
    	s.send(dst_addr + src_addr + eth_type + payload + padding)
        
#Close file
f_output.close()
print("Result received with %d errors..." %(count_errors))
