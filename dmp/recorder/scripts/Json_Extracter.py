import json
import socket

# load the recorded Biotac and joint states data
with open('trial_5.json') as  data_file:
    data = json.load(data_file)

# ipv4 address family, TCP protocol
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 9999
# get the ip address of the eve computer
server_ip = socket.gethostbyname('eve')

s.connect(('eve', port))
f = open("test.json", "rb")
l = f.read(8192)
while (l):
    s.send(l)
    l = f.read(8192)
    print "transferring"
print s.recv(8192)
print "handshake ! now starting transfer file."
s.close()


