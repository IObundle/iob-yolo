#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
import sys
import definitions
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
if(fixed_flag):
    eth_nbytes = 1024-18
else:
    eth_nbytes = 1022-18
    
#Open socket and bind
s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
s.bind((interface, 0))

################################# SEND IMAGE ##############################################

#Open image file
print("\nStarting input image transmission...")
input_ntw_filename = "../dog.bin"
f_in = open(input_ntw_filename, 'rb')
image_w = struct.unpack('i', f_in.read(4))[0]
image_h = struct.unpack('i', f_in.read(4))[0]
image_c = struct.unpack('i', f_in.read(4))[0]
print("Input image is %dx%dx%d .." %(image_w, image_h, image_c))

#Frame parameters
if(fixed_flag):
    input_ntw_file_size = definitions.IMAGE_INPUT
else:
    input_ntw_file_size = definitions.IMAGE_INPUT*4
num_frames_input_ntw = int(input_ntw_file_size/eth_nbytes)
print("input_ntw_file_size: %d" % input_ntw_file_size)     
print("num_frames_input_ntw: %d" % (num_frames_input_ntw+1))

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
    if(fixed_flag):
        payload = f_in.read(bytes_to_send)
    else:
        aux_payload = f_in.read(int(bytes_to_send/4))
        for i in range(len(aux_payload)):
            val_bytes = struct.pack('f', ord(aux_payload[i])/255.)
            if(i == 0):
                payload = val_bytes
            else:
                payload += val_bytes
                
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

#Open weight file
print("\nStarting input weight transmission...")
if(fixed_flag):
    weights_filename = "../yolov3-tiny_batch-fixed.weights"
else:
    weights_filename = "../yolov3-tiny_batch-float.weights"
f_weights = open(weights_filename, 'rb')

#Frame parameters
weights_file_size = getsize(weights_filename)
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

print("weights transmitted with %d errors..." %(count_errors))

################################# SEND LABELS ##############################################

#Open labels file
print("\nStarting labels transmission...")
labels_filename = '../classes.bin'
f_labels = open(labels_filename, 'rb')

#Reset byte counter
count_bytes = 0
count_errors = 0
label_height = 20
w_arr = []

def label_data(num_frames_label_data, label_data_size):
    
    count_bytes = 0
    count_errors = 0
    for j in range(num_frames_label_data+1):
    
        # check if it is last packet (not enough for full payload)
        if j == num_frames_label_data:
            bytes_to_send = label_data_size - count_bytes
            padding = '\x00' * (eth_nbytes-bytes_to_send)
        else:
            bytes_to_send = eth_nbytes
            padding = ''
    
        #form frame
        if(fixed_flag):
            payload = f_labels.read(bytes_to_send)
        else:
            aux_payload = f_labels.read(int(bytes_to_send/4))
            for i in range(len(aux_payload)):
                if(num_frames_label_data == 0):
                    val_bytes = struct.pack('f', float(ord(aux_payload[i])))
                    w_arr.append(aux_payload[i])
                else:
                    val_bytes = struct.pack('f', ord(aux_payload[i])/255.)
                if(i == 0):
                    payload = val_bytes
                else:
                    payload += val_bytes
            
        # accumulate sent bytes
        count_bytes += eth_nbytes
    
        #Send packet
        s.send(dst_addr + src_addr + eth_type + payload + padding)
        
        #receive data back as ack
        rcv = s.recv(4096)
        for sent_byte, rcv_byte in zip(payload, rcv[14:bytes_to_send+14]):
            if sent_byte != rcv_byte:
                count_errors += 1
            if fixed_flag:
                if label_data_size == 81:
                    w_arr.append(rcv_byte)
    return count_errors

if(fixed_flag):
    count_errors += label_data(0, 81)
else:
    count_errors += label_data(0, 81*4)
for i in range(81):
    if(fixed_flag):
        label_size = ord(w_arr[i])*label_height
    else:
        label_size = ord(w_arr[i])*label_height*4
    count_errors += label_data(int(label_size/eth_nbytes), label_size)
print("labels data transmitted with %d errors..." %(count_errors))

################################# SEND INTERM DATA ##############################################

#Flag add during compile time
if(interm_data_flag):
    
    #Open interm data file
    print("\nStarting interm data transmission...")
    if(fixed_flag):
        interm_data_filename = '../interm_data.network'
    else:
        interm_data_filename = '../interm_data-float.network'
    f_interm_data = open(interm_data_filename, 'rb')

    #Reset byte counter
    count_bytes = 0
    count_errors = 0
    
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
    if(fixed_flag):
        layer1_int_size = definitions.NTW_IN_W*2
    else:
        layer1_int_size = definitions.NTW_IN_W*4
    num_frames_layer1 = int(layer1_int_size/eth_nbytes)
    print("layer1_int_size: %d (x%d)" % (layer1_int_size, definitions.NTW_IN_NUM_KER*2))    
    print("num_frames_layer1: %d (x%d)" % (num_frames_layer1+1, definitions.NTW_IN_NUM_KER*2))
    for k in range(definitions.NTW_IN_NUM_KER):
        count_errors += interm_data(num_frames_layer1, layer1_int_size)
        count_errors += interm_data(num_frames_layer1, layer1_int_size)
    print("layer 1 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 2 intermediate data
    if(fixed_flag):
        layer2_int_size = (definitions.LAYER_3_W+2)*2*2
    else:
        layer2_int_size = (definitions.LAYER_3_W+2)*2*4
    num_frames_layer2 = int(layer2_int_size/eth_nbytes)
    print("layer2_int_size: %d (x%d)" % (layer2_int_size, definitions.LAYER_2_NUM_KER*2))    
    print("num_frames_layer2: %d (x%d)" % (num_frames_layer2+1, definitions.LAYER_2_NUM_KER*2))
    for k in range(definitions.LAYER_2_NUM_KER):
        count_errors += interm_data(num_frames_layer2, layer2_int_size)
        count_errors += interm_data(num_frames_layer2, layer2_int_size)
    print("layer 2 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 4 intermediate data
    if(fixed_flag):
        layer4_int_size = (definitions.LAYER_5_W+2)*2*2
    else:
        layer4_int_size = (definitions.LAYER_5_W+2)*2*4
    num_frames_layer4 = int(layer4_int_size/eth_nbytes)
    print("layer4_int_size: %d (x%d)" % (layer4_int_size, definitions.LAYER_4_NUM_KER*2))    
    print("num_frames_layer4: %d (x%d)" % (num_frames_layer4+1, definitions.LAYER_4_NUM_KER*2))
    for k in range(definitions.LAYER_4_NUM_KER):
        count_errors += interm_data(num_frames_layer4, layer4_int_size)
        count_errors += interm_data(num_frames_layer4, layer4_int_size)
    print("layer 4 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 5 intermediate data
    if(fixed_flag):
        layer5_int_size = definitions.LAYER_5_W*2
    else:
        layer5_int_size = definitions.LAYER_5_W*4
    num_frames_layer5 = int(layer5_int_size/eth_nbytes)
    print("layer5_int_size: %d (x%d)" % (layer5_int_size, definitions.LAYER_5_NUM_KER*2))    
    print("num_frames_layer5: %d (x%d)" % (num_frames_layer5+1, definitions.LAYER_5_NUM_KER*2))
    for k in range(definitions.LAYER_5_NUM_KER):
        count_errors += interm_data(num_frames_layer5, layer5_int_size)
        count_errors += interm_data(num_frames_layer5, layer5_int_size)
    print("layer 5 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 6 intermediate data
    if(fixed_flag):
        layer6_int_size = (definitions.LAYER_7_W+2)*2*2
    else:
        layer6_int_size = (definitions.LAYER_7_W+2)*2*4
    num_frames_layer6 = int(layer6_int_size/eth_nbytes)
    print("layer6_int_size: %d (x%d)" % (layer6_int_size, definitions.LAYER_6_NUM_KER*2))    
    print("num_frames_layer6: %d (x%d)" % (num_frames_layer6+1, definitions.LAYER_6_NUM_KER*2))
    for k in range(definitions.LAYER_6_NUM_KER):
        count_errors += interm_data(num_frames_layer6, layer6_int_size)
        count_errors += interm_data(num_frames_layer6, layer6_int_size)
    print("layer 6 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 8 intermediate data
    if(fixed_flag):
        layer8_int_size = (definitions.LAYER_9_W+2)*2*2 
    else:
        layer8_int_size = (definitions.LAYER_9_W+2)*2*4
    num_frames_layer8 = int(layer8_int_size/eth_nbytes)
    print("layer8_int_size: %d (x%d)" % (layer8_int_size, definitions.LAYER_8_NUM_KER*2))    
    print("num_frames_layer8: %d (x%d)" % (num_frames_layer8+1, definitions.LAYER_8_NUM_KER*2))
    for k in range(definitions.LAYER_8_NUM_KER):
        count_errors += interm_data(num_frames_layer8, layer8_int_size)
        count_errors += interm_data(num_frames_layer8, layer8_int_size)
    print("layer 8 interm data transmitted with %d errors...\n" %(count_errors))
    
    #Send layer 9 intermediate data
    if(fixed_flag):
        layer9_int_size = (definitions.LAYER_9_W+2)*2 
    else:
        layer9_int_size = (definitions.LAYER_9_W+2)*4 
    num_frames_layer9 = int(layer9_int_size/eth_nbytes)
    print("layer9_int_size: %d (x%d)" % (layer9_int_size, definitions.LAYER_9_NUM_KER*2))    
    print("num_frames_layer9: %d (x%d)" % (num_frames_layer9+1, definitions.LAYER_9_NUM_KER*2))
    for k in range(definitions.LAYER_9_NUM_KER):
        count_errors += interm_data(num_frames_layer9, layer9_int_size)
        count_errors += interm_data(num_frames_layer9, layer9_int_size)
    print("layer 9 interm data transmitted with %d errors..." %(count_errors))
    print("interm data transmitted with %d errors..." %(count_errors))

################################# RECEIVE IMAGE WITH DETECTIONS ##############################################

#Open output image files
print("\nStarting reception of image with detections...")
if(fixed_flag):
    output_filename = '../dog_det.bin'
else:
    output_filename = '../dog_det_float.bin'
output_image_filename = sys.argv[3]+'/detections.bin'
f_output = open(output_filename, "rb")
f_output_image =  open(output_image_filename, "wb")

#Reset counters
count_errors = 0
count_bytes = 0

#Frame parameters
print("input_ntw_file_size: %d" % input_ntw_file_size)     
print("num_frames_input_ntw: %d" % (num_frames_input_ntw+1))

#Create image
image_w = struct.unpack('i', f_output.read(4))[0]
image_h = struct.unpack('i', f_output.read(4))[0]
image_c = struct.unpack('i', f_output.read(4))[0]
print("Output image is %dx%dx%d .." %(image_w, image_h, image_c))
f_output_image.write(struct.pack('i', image_w))
f_output_image.write(struct.pack('i', image_h))
f_output_image.write(struct.pack('i', image_c))

#Loop to receive one yolo layer output
for j in range(num_frames_input_ntw+1):

    #Check if it is last packet (not enough for full payload)
    if j == num_frames_input_ntw:
        bytes_to_receive = input_ntw_file_size - count_bytes
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
        if(fixed_flag):
            f_output_image.write(struct.pack('B', ord(rcv_byte)))
        if sent_byte != rcv_byte:
            count_errors += 1
            
    #Convert float to byte
    if(not fixed_flag):
        for i in range(len(int(bytes_to_receive/4))):
            float_val = struct.unpack('f', payload[i*4:(i+1)*4])[0]
            val_bytes = struct.pack("B", int(float_val*255))
            f_output_image.write(struct.pack('B', ord(val_bytes)))
            
    #Send data back as ack
    s.send(dst_addr + src_addr + eth_type + payload + padding)
        
print("Detections received with %d errors..." %(count_errors))

#Close files
f_in.close()
f_weights.close()
if(interm_data_flag):
    f_interm_data.close()
f_labels.close()
f_output.close()
f_output_image.close()