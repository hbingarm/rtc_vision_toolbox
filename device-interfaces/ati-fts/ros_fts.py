import socket
import struct
import sys
from threading import Thread
from time import sleep
import signal
import numpy
import rospy
import sys
import serial
import time
import  os
from geometry_msgs.msg import Wrench

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

PORT = 49152  # Port the Net F/T always uses
IP = "192.168.1.41"
COMMAND = 2  # Command code 2 starts streaming
NUM_SAMPLES = 1  # Will send 1 sample before stopping
AXES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]  # The names of the force and torque axes.


class Response:
    def __init__(self):
        self.rdt_sequence = 0
        self.ft_sequence = 0
        self.status = 0
        self.FTData = [0] * 6

def main():
    rospy.init_node("fts_node", anonymous=True)
    pub = rospy.Publisher('FTS', Wrench, queue_size=10)

    # Calculate number of samples, command code, and open socket here.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    request = struct.pack("!HHI", 0x1234, COMMAND, NUM_SAMPLES)
    seq = -1
    while True:
        # Sending the request.
        addr = (IP, PORT)
        sock.connect(addr)
        sock.send(request)

        # Receiving the response.
        response = sock.recv(36)
        resp = Response()
        (
            resp.rdt_sequence,
            resp.ft_sequence,
            resp.status,
            resp.FTData[0],
            resp.FTData[1],
            resp.FTData[2],
            resp.FTData[3],
            resp.FTData[4],
            resp.FTData[5],
        ) = struct.unpack("!IIIiiiiii", response)

        if resp.ft_sequence != seq:
            seq = resp.ft_sequence
            data_to_send = Wrench()
            data_to_send.force.x = resp.FTData[0] / 1e6
            data_to_send.force.y = resp.FTData[1] / 1e6
            data_to_send.force.z = resp.FTData[2] / 1e6
            data_to_send.torque.x = resp.FTData[3] / 1e6
            data_to_send.torque.y = resp.FTData[4] / 1e6
            data_to_send.torque.z = resp.FTData[5] / 1e6
            pub.publish(data_to_send)

        # print("RDT Sequence: {}".format(resp.rdt_sequence))

if __name__ == "__main__":
    main()
