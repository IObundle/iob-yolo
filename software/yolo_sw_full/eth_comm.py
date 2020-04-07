#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
import sys
import definitions

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
input_ntw_filename = "../dog.bin"
weights_filename = "../yolov3-tiny_batch-fixed.weights"
output_ntw_filename = '../output_fixed_full.network'
interm_data_filename = '../interm_data.network'
f_in = open(input_ntw_filename, 'rb')
f_in.read(12) #ignore image size info (3 int values)
f_weights = open(weights_filename, 'rb')
f_interm_data = open(interm_data_filename, 'rb')
f_out = open(output_ntw_filename, "rb")
input_ntw_file_size = definitions.IMAGE_INPUT
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

################################# SEND IMAGE ##############################################

print("\nStarting input image transmission...")

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

################################# SEND WEIGHTS ##############################################

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

################################# SEND INTERM DATA ##############################################

#Reset byte counter
count_bytes = 0
count_errors = 0
print("\nStarting interm data transmission...")

def interm_data(num_frames_interm_data, interm_data_size):
    
    count_bytes = 0
    count_errors = 0
    for j in range(num_frames_interm_data+1):
    
        # check if it is last packet (not enough for full payload)
        if j == num_frames_interm_data:
            bytes_to_send = interm_data_size - count_bytes
            padding = '\x00' * (eth_nbytes-bytes_to_send)
        else:
            bytes_to_send = eth_nbytes
            padding = ''
    
        #form frame
        payload = f_interm_data.read(bytes_to_send)
            
        # accumulate sent bytes
        count_bytes += eth_nbytes
    
        #Send packet
        s.send(dst_addr + src_addr + eth_type + payload + padding)
        
        #receive data back as ack
        rcv = s.recv(4096)
        for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_send+14]):
            if sent_byte != rcv_byte:
                count_errors += 1
    return count_errors
   
#Send layer 1 intermediate data
layer1_int_size = definitions.NTW_IN_W*2
num_frames_layer1 = int(layer1_int_size/eth_nbytes)
print("layer1_int_size: %d (x%d)" % (layer1_int_size, definitions.NTW_IN_NUM_KER*2))    
print("num_frames_layer1: %d (x%d)" % (num_frames_layer1+1, definitions.NTW_IN_NUM_KER*2))
for k in range(definitions.NTW_IN_NUM_KER):
    count_errors += interm_data(num_frames_layer1, layer1_int_size)
    count_errors += interm_data(num_frames_layer1, layer1_int_size)
print("layer 1 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 2 intermediate data
layer2_int_size = (definitions.LAYER_3_W+2)*2*2
num_frames_layer2 = int(layer2_int_size/eth_nbytes)
print("layer2_int_size: %d (x%d)" % (layer2_int_size, definitions.LAYER_2_NUM_KER*2))    
print("num_frames_layer2: %d (x%d)" % (num_frames_layer2+1, definitions.LAYER_2_NUM_KER*2))
for k in range(definitions.LAYER_2_NUM_KER):
    count_errors += interm_data(num_frames_layer2, layer2_int_size)
    count_errors += interm_data(num_frames_layer2, layer2_int_size)
print("layer 2 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 4 intermediate data
layer4_int_size = (definitions.LAYER_5_W+2)*2*2
num_frames_layer4 = int(layer4_int_size/eth_nbytes)
print("layer4_int_size: %d (x%d)" % (layer4_int_size, definitions.LAYER_4_NUM_KER*2))    
print("num_frames_layer4: %d (x%d)" % (num_frames_layer4+1, definitions.LAYER_4_NUM_KER*2))
for k in range(definitions.LAYER_4_NUM_KER):
    count_errors += interm_data(num_frames_layer4, layer4_int_size)
    count_errors += interm_data(num_frames_layer4, layer4_int_size)
print("layer 4 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 5 intermediate data
layer5_int_size = definitions.LAYER_5_W*2
num_frames_layer5 = int(layer5_int_size/eth_nbytes)
print("layer5_int_size: %d (x%d)" % (layer5_int_size, definitions.LAYER_5_NUM_KER*2))    
print("num_frames_layer5: %d (x%d)" % (num_frames_layer5+1, definitions.LAYER_5_NUM_KER*2))
for k in range(definitions.LAYER_5_NUM_KER):
    count_errors += interm_data(num_frames_layer5, layer5_int_size)
    count_errors += interm_data(num_frames_layer5, layer5_int_size)
print("layer 5 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 6 intermediate data
layer6_int_size = (definitions.LAYER_7_W+2)*2*2 
num_frames_layer6 = int(layer6_int_size/eth_nbytes)
print("layer6_int_size: %d (x%d)" % (layer6_int_size, definitions.LAYER_6_NUM_KER*2))    
print("num_frames_layer6: %d (x%d)" % (num_frames_layer6+1, definitions.LAYER_6_NUM_KER*2))
for k in range(definitions.LAYER_6_NUM_KER):
    count_errors += interm_data(num_frames_layer6, layer6_int_size)
    count_errors += interm_data(num_frames_layer6, layer6_int_size)
print("layer 6 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 8 intermediate data
layer8_int_size = (definitions.LAYER_9_W+2)*2*2 
num_frames_layer8 = int(layer8_int_size/eth_nbytes)
print("layer8_int_size: %d (x%d)" % (layer8_int_size, definitions.LAYER_8_NUM_KER*2))    
print("num_frames_layer8: %d (x%d)" % (num_frames_layer8+1, definitions.LAYER_8_NUM_KER*2))
for k in range(definitions.LAYER_8_NUM_KER):
    count_errors += interm_data(num_frames_layer8, layer8_int_size)
    count_errors += interm_data(num_frames_layer8, layer8_int_size)
print("layer 8 interm data transmitted with %d errors...\n" %(count_errors))

#Send layer 9 intermediate data
layer9_int_size = (definitions.LAYER_9_W+2)*2 
num_frames_layer9 = int(layer9_int_size/eth_nbytes)
print("layer9_int_size: %d (x%d)" % (layer9_int_size, definitions.LAYER_9_NUM_KER*2))    
print("num_frames_layer9: %d (x%d)" % (num_frames_layer9+1, definitions.LAYER_9_NUM_KER*2))
for k in range(definitions.LAYER_9_NUM_KER):
    count_errors += interm_data(num_frames_layer9, layer9_int_size)
    count_errors += interm_data(num_frames_layer9, layer9_int_size)
print("layer 9 interm data transmitted with %d errors..." %(count_errors))
print("interm data transmitted with %d errors..." %(count_errors))

################################# RECEIVE YOLO LAYERS ##############################################

#Reset error counter
count_errors = 0
print("\nStarting reception of yolo layers...")
yolo_layer_file_size_arr = [definitions.DATA_LAYER_17, definitions.DATA_LAYER_24]

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

#Close files
f_in.close()
f_weights.close()
f_out.close()
