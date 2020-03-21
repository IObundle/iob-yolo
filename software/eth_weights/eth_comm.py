#Import libraries
from socket import socket, AF_PACKET, SOCK_RAW, htons, timeout
import sys
import os
import struct
import time


def get_MAC_ADDR(rcv):
    mac_addr = ""
    mac_addr = rcv[6:12]
    return mac_addr

def compare_MAC_ADDR(mac_addr):
    dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
    if dst_addr == mac_addr:
        raw_mac_addr = r"{}".format(mac_addr)
        print("Message from wrong MAC: %s" % (raw_mac_addr) )
    return
    
def print_MAC_ADDR(rcv, i):
    mac_addr = rcv[6:12] 
    readable_mac = ":".join("{:02x}".format(ord(c)) for c in mac_addr)
    dst_mac_addr = rcv[0:6] 
    readable_dst_mac = ":".join("{:02x}".format(ord(c)) for c in dst_mac_addr)
    eth_type = rcv[12:14] 
    readable_eth_type = ":".join("{:02x}".format(ord(c)) for c in eth_type)

#    raw_mac_addr = r"{}".format(mac_addr)
    print("[%d]DST: %s SRC: %s ETH: %s" %(i, readable_dst_mac, readable_mac, readable_eth_type))
    return


def main(argv):
    
    # Path to files - suposed to be on sandbox/
    weights_path = "../yolov3-tiny_batch-fixed.weights"
    data_path = "../input_fixed.network"

    #Open .network file
    try:
        # f_img = open(sys.argv[1], "r")
        f_img = open(data_path, "r")

        data_file_size = os.path.getsize(data_path)
    except:
        print("Failed to load .network file: %s\n" % data_path)
        sys.exit(1)
    #Open .weights file
    try:
        # f_weights = open(sys.argv[2], "r")
        f_weights = open(weights_path, "r")
        weights_file_size = os.path.getsize(weights_path)
    except:
        print("Failed to load .weights file: %s\n" % weights_path)
        sys.exit(1)

    #Ethernet parameters
    BOARD = "ALTERA"
    if(BOARD == "ALTERA"):
        interface = "enp0s31f6"
        src_addr = "\x30\x9C\x23\x1E\x62\x4B"   # sender MAC address
    else:
        interface = "eno1"
        src_addr = "\x00\x1e\x37\x3a\xe0\x2e"   # sender MAC address
    dst_addr = "\x01\x60\x6e\x11\x02\x0f"       # receiver MAC address
    eth_type = "\x08\x00"                       # ethernet frame type
    ETH_P_ALL = 0x0800                          # ethernet frame type
        
    #Frame parameter
    eth_nbytes = 1024-18

    num_data_frames = int(data_file_size/eth_nbytes)
    num_weight_frames = int(weights_file_size/eth_nbytes)

    #debug
    print("data_file_size: %d\n" % data_file_size)
    print("weights_file_size: %d\n" %  weights_file_size)

    print("num_data_frames: %d\n" % num_data_frames)
    print("num_weight_frames: %d\n" % num_weight_frames)

    #Open socket and bind
    s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))
    s.bind((interface, 0))

    print("Starting input data transmission...\n")

    count_bytes = 0
    count_errors = 0

    # Loop to send and receive back data frames
    for j in range(num_data_frames+1):
        
        # check if it is last packet (not enough for full payload)
        if j == num_data_frames:
            bytes_to_send = data_file_size - count_bytes
            #set remaining bytes to zero - same as adding padding
            padding = '\x00' * (eth_nbytes-bytes_to_send)
        else:
            bytes_to_send = eth_nbytes
            #empty string
            padding = ''

        #form frame
        payload = f_img.read(bytes_to_send)
            
        # accumulate sent bytes
        count_bytes += eth_nbytes
        sent = payload + padding

        #Send packet
        s.send(dst_addr + src_addr + eth_type + payload + padding)
                
        #Receive packet
        rcv = s.recv(4096)
        
        # Check that sent and received packages are the same
        for sent_byte, rcv_byte in zip(sent, rcv[14:]):
            if sent_byte != rcv_byte:
                count_errors += 1

    print("DEBUG:Data sent and received with %d errors\n" % (count_errors))

    # Reset byte counter
    count_bytes = 0

    print("Starting input weight transmission...\n")

    # Loop to send and receive back data frames
    for j in range(num_weight_frames+1):
        
        # check if it is last packet (not enough for full payload)
        if j == num_weight_frames:
            bytes_to_send = weights_file_size - count_bytes
            #set remaining bytes to zero - same as adding padding
            padding = '\x00' * (eth_nbytes-bytes_to_send)
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

        # Check that sent and received packages are the same
        sent = payload + padding
        for sent_byte, rcv_byte_ in zip(sent, rcv[14:]):
            if sent_byte != rcv_byte:
                count_errors += 1

    print("Data sent and received with %d errors\n" % (count_errors))


if __name__ == "__main__":
    main(sys.argv[1:])
