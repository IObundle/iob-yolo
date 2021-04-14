#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons, AF_UNIX, SOCK_SEQPACKET
from os.path import getsize
from time import sleep
import sys

#Check if argument identifying type of board is present
if len(sys.argv) < 3:
    print("<usage>: python eth_comm.py <interface> <RMAC> ")
    sys.exit()

#Check for PC simulation
if len(sys.argv) > 4:
    PCSIM = (sys.argv[4] == "PCsim")
else:
    PCSIM = 0

#Ethernet parameters
#Common parameters
dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
eth_type = "\x08\x00"                       # ethernet frame type
ETH_P_ALL = 0x0800   

if PCSIM: #PC simulation
    print(sys.argv[3])
    SOCKET_NAME = sys.argv[3] + "/tmpLocalSocket"
    src_addr = dst_addr
    print("**** PC simulation ***")
else: # embedded
    interface = sys.argv[1]
    src_addr = bytearray.fromhex(sys.argv[2])   # sender MAC address

#Open output.network
filename = "../output_fixed.network"
fp = open(filename, "rb")
file_size = getsize(filename)

#Frame parameters
eth_nbytes = 1024-18
num_frames = int(file_size/eth_nbytes)
print("file_size: %d" % file_size)     
print("num_frames: %d" % (num_frames+1))
    
#Connect with Firmware
if PCSIM: # PC Simulation: open local socket
    #Open socket and bind
    s = socket(AF_UNIX, SOCK_SEQPACKET, 0)
    #Connect to Peer
    s.connect(SOCKET_NAME)
else: # Embedded: open raw ethernet socket
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
    sleep(0.05)
    
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

print("Data sent and received with %d errors\n" % count_errors)

#Close file
fp.close()
