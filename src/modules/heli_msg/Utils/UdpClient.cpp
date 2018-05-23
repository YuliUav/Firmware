#if (defined __linux__) || (defined __QNXNTO__)

#include <sys/socket.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "UdpClient.h"

UdpClient::UdpClient(const char* ip, int port)
{
	s = socket(AF_INET, SOCK_DGRAM, 0);

	length = sizeof(struct sockaddr_in);
	memset(&addr, 0, length);
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip);
	addr.sin_port = htons(port);
}

/// @return 0: success, 1: unable to bind the specified ip and port to udp client
int UdpClient::Bind(const char* ip, int port)
{
	struct sockaddr_in addrBind;

	memset(&addrBind, 0, length);
	addrBind.sin_family = AF_INET;
	addrBind.sin_addr.s_addr = inet_addr(ip);
	addrBind.sin_port = htons(port);

	if (bind(s, (struct sockaddr*) &addrBind, length) == 0)
		return 0;
	return 1;
}

int UdpClient::Receive(unsigned char* dgram, int offset, int bytes)
{
	return recvfrom(s, dgram + offset, bytes, 0, (struct sockaddr*) &addr,
		(socklen_t*)&length);
}

void UdpClient::SetBroadcast()
{
	int optval = 1;
	setsockopt(s, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(int));
}

void UdpClient::Send(unsigned char* dgram, int offset, int bytes)
{
	sendto(s, dgram + offset, bytes, 0, (struct sockaddr*) &addr, length);
}

void UdpClient::Close()
{
	close(s);
}

#endif
