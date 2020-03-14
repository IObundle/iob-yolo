#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons
import sys
import os

# #Ethernet parameters
# BOARD = "XILINX"
# if(BOARD == "ALTERA"):
#     interface = "enp0s31f6"
#     src_addr = "\x30\x9C\x23\x1E\x62\x4B"   # sender MAC address
# else:
#     interface = "enp0s20u4"
#     src_addr = "\x00\xE0\x4C\x69\x0B\xA0"   # sender MAC address
# dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
# eth_type = "\x08\x00"                       # ethernet frame type
# ETH_P_ALL = 0x0800                          # ethernet frame type

# #Frame parameter
# eth_len = 256-18
# payload = "Hello from PC\n"

# #Open socket and bind
# s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
# s.bind((interface, 0))

# #Send packet back
# padding = '\x00' * (eth_len-len(payload))
# s.send(dst_addr + src_addr + eth_type + payload + padding)

# #Receive packet
# rcv = s.recv(4096)
# print("Message received: " + rcv[14:])


def main(argv):
    # check for correct arguments
    if len(sys.argv) != 3:
        print("Usage: eth_comm.py <path/to/.network> <path/to/.weights>")
        sys.exit(1)

    #Open .network file
    try:
        f_img = open(sys.argv[1], "r")
        data_file_size = os.path.getsize(sys.argv[1])
    except:
        print("Failed to load .network file: %s\n" % sys.argv[1])
        sys.exit(1)
    #Open .weights file
    try:
        f_weights = open(sys.argv[2], "r")
        weights_file_size = os.path.getsize(sys.argv[2])
    except:
        print("Failed to load .weights file: %s\n" % sys.argv[2])
        sys.exit(1)

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
    eth_nbytes = 256-18
    # payload = "Hello from PC\n"
    num_data_frames = int(data_file_size/eth_nbytes)
    num_weight_frames = int(weights_file_size/eth_nbytes)

    #debug
    print("data_file_size: %d\n" % data_file_size)
    print("weights_file_size: %d\n" %  weights_file_size)

    print("num_data_frames: %d\n" % num_data_frames)
    print("num_weight_frames: %d\n" % num_weight_frames)



    #Open socket and bind
    # s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
    # s.bind((interface, 0))

    print("Starting input data transmission...\n")

    count_bytes = 0
    count_errors = 0
    # Loop to send and receive back data frames
    for j in range(num_data_frames+1):
        
        # check if it is last packet (not enough for full payload)
        if j == num_data_frames:
            bytes_to_send = data_file_size - count_bytes
            #set remaining bytes to zero - same as adding padding
            padding = '\x00' * (bytes_to_send)
        else:
            bytes_to_send = eth_nbytes
            #empty string
            padding = ''

        #form frame
        payload = f_img.read(bytes_to_send)
            
        # accumulate sent bytes
        count_bytes += eth_nbytes

        #Send packet
        # padding = '\x00' * (eth_nbytes-len(payload))
        s.send(dst_addr + src_addr + eth_type + payload + padding)
    
        #Receive packet
        rcv = s.recv(4096)
        print("Message received: " + rcv[14:] + "\n")
        
        # Check that sent and received packages are the same
        sent = payload + padding
        for s, r in zip(sent, recv[14:]):
            if s != r:
                count_errors += 1

        if j%100:
            print("iter = %d, total_err = %d\n", j, count_errors)

    # Reset byte counter
    count_bytes = 0

    print("Starting input weight transmission...\n")

    # Loop to send and receive back data frames
    for j in range(num_weight_frames+1):
        
        # check if it is last packet (not enough for full payload)
        if j == num_weight_frames:
            bytes_to_send = weight_file_size - count_bytes
            #set remaining bytes to zero - same as adding padding
            padding = '\x00' * (bytes_to_send)
        else:
            bytes_to_send = eth_nbytes
            #empty string
            padding = ''

        #form frame
        payload = f_weights.read(bytes_to_send)
            
        # accumulate sent bytes
        count_bytes += eth_nbytes

        #Send packet
        # padding = '\x00' * (eth_nbytes-len(payload))
        s.send(dst_addr + src_addr + eth_type + payload + padding)
    
        #Receive packet
        rcv = s.recv(4096)
        print("Message received: " + rcv[14:] + "\n")
        
        # Check that sent and received packages are the same
        sent = payload + padding
        for s, r in zip(sent, recv[14:]):
            if s != r:
                count_errors += 1

        if j%1000:
            print("iter = %d, total_err = %d\n", j, count_errors)

    print("Data sent and received with %d errors\n", count_errors)


if __name__ == "__main__":
    main(sys.argv[1:])
