#!/usr/bin/env python3

import socket

def listener():
    # Créer un socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Lier le socket à l'adresse et au port du serveur NS3
    NS3_SERVER_IP = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
    NS3_SERVER_PORT = 12345  # Remplacer par le port du serveur NS3
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((NS3_SERVER_IP, NS3_SERVER_PORT))

    while True:
        # Recevoir des données du serveur
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        print("Received message:", data.decode())

if __name__ == '__main__':
    listener()