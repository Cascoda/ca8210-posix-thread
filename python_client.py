#fe80::ba27:ebff:fed1:96ba::5005

import socket

UDP_IP = "fe80::ba27:ebff:fed1:96ba"  # localhost
UDP_PORT = 5005
MESSAGE = "Hello, World!"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))