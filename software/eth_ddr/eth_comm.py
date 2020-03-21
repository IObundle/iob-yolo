#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
from os.path import getsize
from time import sleep
import sys

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

#Open output.network
filename = "../output_fixed.network"
fp = open(filename, "rb")
file_size = getsize(filename)

#Frame parameters
eth_nbytes = 1024-18
num_frames = int(file_size/eth_nbytes)
print("file_size: %d" % file_size)     
print("num_frames: %d" % (num_frames+1))
    
#Open socket and bind
s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
s.bind((interface, 0))
print("Starting data transmission...")

#Counters
count_bytes = 0
count_errors = 0

# Loop to send and receive back data frames
for j in range(num_frames+1):
    
    # check if it is last packet (not enough for full payload)
    if j == num_frames:
        bytes_to_send = file_size - count_bytes
        padding = '\x00' * (eth_nbytes-bytes_to_send)
    else:
        bytes_to_send = eth_nbytes
        padding = ''

    #form frame
    payload = fp.read(bytes_to_send)

    # accumulate sent bytes
    count_bytes += eth_nbytes

    #Send packet
    s.send(dst_addr + src_addr + eth_type + payload + padding)

    #Wait some time
    sleep(0.005)
    
#Close and open file again
fp.close()
fp = open(filename, "rb")

# Reset byte counter
count_bytes = 0
print("Data transmitted")
print("Starting data reception")

# Loop to send and receive back data frames
for j in range(num_frames+1):
    
    # check if it is last packet (not enough for full payload)
    if j == num_frames:
        bytes_to_receive = file_size - count_bytes
    else:
        bytes_to_receive = eth_nbytes

    #form frame
    payload = fp.read(bytes_to_receive)
        
    # accumulate sent bytes
    count_bytes += eth_nbytes

    #Receive packet
    rcv = s.recv(4096)
    
    # Check that sent and received packages are the same
    sent = payload
    for s2, r in zip(sent, rcv[14:bytes_to_receive+14]):
        if s2 != r:
            count_errors += 1

print("Data sent and received with %d errors\n", count_errors)

#Close file
fp.close()
