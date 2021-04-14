#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons, AF_UNIX, SOCK_SEQPACKET
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
ETH_P_ALL = 0x0800                          # ethernet frame type

if PCSIM: #PC simulation
    print(sys.argv[3])
    SOCKET_NAME = sys.argv[3] + "/tmpLocalSocket"
    src_addr = dst_addr
    print("**** PC simulation ***")
else: # embedded
    interface = sys.argv[1]
    src_addr = bytearray.fromhex(sys.argv[2])   # sender MAC address

#Frame parameter
eth_len = 256-18
payload = "Hello from PC\n"

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


#Send packet back
padding = '\x00' * (eth_len-len(payload))
s.send(dst_addr + src_addr + eth_type + payload + padding)

#Receive packet
rcv = s.recv(4096)
print("Message received: " + rcv[14:])
