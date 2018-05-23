/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file SerialPort.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Serial port communication
///
/// Represents a serial port resource
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#if (defined __linux__) || (defined __QNXNTO__)

#pragma once

class SerialPort
{
public:
	SerialPort(const char* name, int baudRate, char parity, int dataBits,
		int stopBits) :
		name(name), baudRate(baudRate), parity(parity), dataBits(dataBits),
		stopBits(stopBits)
	{
	}
	int Open();
	void DiscardInBuffer();
	int Read(unsigned char* buffer, int offset, int count);
	void DiscardOutBuffer();
	void Write(unsigned char* buffer, int offset, int count);
	void Close();
private:
	int fildes;
	const char* name;
	int baudRate;
	char parity;
	int dataBits;
	int stopBits;
};

#endif
