#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
import sys

#Check if argument identifying type of board is present
if len(sys.argv) != 3:
    print("<usage>: python eth_comm.py <interface> <RMAC> ")
    sys.exit()

#Ethernet parameters
interface = sys.argv[1]
src_addr = bytearray.fromhex(sys.argv[2])   # sender MAC address
dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
eth_type = "\x08\x00"                       # ethernet frame type
ETH_P_ALL = 0x0800                          # ethernet frame type

#Frame parameter
eth_len = 256-18
payload = "Hello from PC\n"

#Open socket and bind
s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
s.bind((interface, 0))

#Send packet back
padding = '\x00' * (eth_len-len(payload))
s.send(dst_addr + src_addr + eth_type + payload + padding)

#Receive packet
rcv = s.recv(4096)
print("Message received: " + rcv[14:])
