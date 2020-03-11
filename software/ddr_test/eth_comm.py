#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons

#Ethernet parameters
BOARD = "XILINX"
if(BOARD == "ALTERA"):
    interface = "enp0s31f6"
    src_addr = "\x30\x9C\x23\x1E\x62\x4B"   # sender MAC address
else:
    interface = "enp0s20u4"
    src_addr = "\x00\xE0\x4C\x69\x0B\xA0"   # sender MAC address
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
