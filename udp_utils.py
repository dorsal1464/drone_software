import socket


class udpBroadcaster(object):
    def __init__(self, port):
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send(self, message):
        return self.server.sendto(message, ('<broadcast>', self.port))

    def recv(self, bufsize):
        # no timeout...
        return self.server.recvfrom(bufsize)


class udpConsumer(object):
    def __init__(self, port, timeout=0.5):
        self.port = port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client.settimeout(timeout)
        self.client.bind(("", port))

    def recv(self, bufsize):
        try:
            return self.client.recvfrom(bufsize)
        except TimeoutError:
            return None, None

    def sendto(self, addr, msg):
        self.client.sendto(msg, (addr, self.port))
