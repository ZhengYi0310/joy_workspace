#!/usr/bin/env python
# This file is used to start the socket client to receive the dmp rollout traj file

import rospy
import roslib

import os, sys
import socket
from threading import Thread 

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import *

class SocketClient:
    def __init__(self):
        rospy.init_node('Socket_Client')
        self.port = 9999
        self.filename = sys.argv[1]

        # register the necessarty service server
        self.initialhandshakeclient = rospy.Service("client_handshake", Empty, self.initialhandshake)
        self.transferfileclient = rospy.Service("client_transferfile", Empty, self.transferfile)
        self.shutdownclient = rospy.Service("client_shutdown", Empty, self.shutdown)

    def setclient(self):
        print "client connects to the server !"
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # must initialize everytime, otherwise throw all kinds of weird errors
        self.client.connect(('eve', self.port)) # connect to socket server

    def initialhandshake(self, req):
        self.setclient()
        self.client.send("Hello eve! This is a handshake from Joy!") # send the message to eve
        server_handshake = rospy.ServiceProxy("server_handshake", Empty) # call the server service to accept the message and send on message back
        server_handshake()
        print self.client.recv(8192)
        #self.shutdown()
        self.client.close()
        return EmptyResponse()

    def transferfile(self, req):
        self.setclient()
        
        # create another thread to launch the socket treceiving
        server_transferfile = rospy.ServiceProxy("server_transferfile", Empty) # call the server service to accept the message and send on message back
        t1 = Thread(target=server_transferfile, )
        t1.start()

        f = open(self.filename, "rb")
        l = f.read(8192)

        while (l):
            self.client.send(l)
            l = f.read(8192)
            print "transferring file!"

        
        print "File transferred to Eve!"
        self.client.close()

        t1.join()
        return EmptyResponse()

    def shutdown(self, req):
        #self.__del__()
        #rospy.signal_shutdown("socket client is shutdown !")
        server_shutdown = rospy.ServiceProxy("server_shutdown", Empty)
        server_shutdown()
        
        return EmptyResponse()

    #def __del__(self):
        #self.client.close()

if __name__ == "__main__":
    socket_client = SocketClient()
    rospy.spin()
