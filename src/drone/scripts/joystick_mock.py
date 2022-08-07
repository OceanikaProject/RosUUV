import socket


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.1.114", 1234))
while True:
    print("waiting...")
    data, addr = sock.recvfrom(1024)
    print(data)
