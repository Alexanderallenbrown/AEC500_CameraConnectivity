import socket
import struct
import time
import random
from math import sin
# bind all IP
HOST = '192.168.1.200'
ESP = '192.168.1.202'
# Listen on Port
PORT = 44444
#Size of receive buffer
BUFFER_SIZE = 1024
# Create a TCP/IP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
starttime =time.time()
# Bind the socket to the host and port
s.bind((HOST, PORT))
while True:
    # Receive BUFFER_SIZE bytes data
    # data is a list with 2 elements
    # first is data
    #second is client address
    tnow = time.time()-starttime
    gascmd = 90+30*sin(tnow)
    steercmd = 90+30*sin(6*tnow)

    cmdmsg = struct.pack('fff',tnow,gascmd,steercmd)
    data = s.recvfrom(BUFFER_SIZE)
    if data:
        #print received data
        print('Client to Server: ' , data)
        s.sendto(cmdmsg,data[1])
        # time.sleep(0.1)
        print("Sending: ",cmdmsg)
    # data = s.recvfrom(BUFFER_SIZE)
    # if data:
    #     #print received data
    #     print('Client to Server: ' , data)
    #     # Convert to upper case and send back to Client
    #     s.sendto(data[0].upper(), data[1])
# Close connection
s.close()
