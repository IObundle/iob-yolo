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

################################# SEND INPUT FILE ##############################################

#Open input file
print("\nStarting input file transmission...")
input_filename = "../new_versat_in_x8.bin"
f_input = open(input_filename, 'rb')

#Frame parameters
input_file_size = getsize(input_filename)
num_frames_input = int(input_file_size/eth_nbytes)
print("input_file_size: %d" % input_file_size)
print("num_frames_input: %d" % (num_frames_input+1))

#Reset byte counter
count_bytes = 0
count_errors = 0

# Loop to send input frames
for j in range(num_frames_input+1):

    # check if it is last packet (not enough for full payload)
    if j == num_frames_input:
        bytes_to_send = input_file_size - count_bytes
        padding = '\x00' * (eth_nbytes-bytes_to_send)
    else:
        bytes_to_send = eth_nbytes
        padding = ''

    #form frame
    payload = f_input.read(bytes_to_send)

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
f_input.close()
print("input file transmitted with %d errors..." %(count_errors))

################################# RECEIVE RESULT ##############################################

#Open output image files
print("\nStarting reception of result...")
output_filename = '../new_versat_out.bin'
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
        padding = '\x00' * (eth_nbytes-bytes_to_receive)
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
