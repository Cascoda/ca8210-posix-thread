import socket
  
UDP_IP = "::" # = 0.0.0.0 u IPv4
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM, 0)
sock.bind((UDP_IP, UDP_PORT, 0, 0))

while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print "received message:", data