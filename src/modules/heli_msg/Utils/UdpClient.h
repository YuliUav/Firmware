/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file UdpClient.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Udp socket communication
///
/// Provides user datagram protocol network services
/// Please add socket library if you're using qnx
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#if (defined __linux__) || (defined __QNXNTO__)

#pragma once

#include <netinet/in.h>

class UdpClient
{
public:
	UdpClient(const char* ip, int port);
	int Bind(const char* ip, int port);
	int Receive(unsigned char* dgram, int offset, int bytes);
	void SetBroadcast();
	void Send(unsigned char* dgram, int offset, int bytes);
	void Close();
private:
	int s;
	struct sockaddr_in addr;
	int length;
};

#endif
