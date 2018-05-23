/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file Protocol.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Standard packet protocol
///
/// Binary packet structure: header lrc (u8) packet id (u8) packet length (u8) crc16 (u16) packet data
/// Ascii packet structure: $packet data (packet id, packet data1, packet data2, ..., packet datan) * checksum \r\n
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

//#if (defined _WIN32) || (defined __linux__) || (defined __QNXNTO__)

#pragma once

const int BinaryPacketHeaderLength = 5;
const int MaxPacketDataLength = 255;
const int BinaryBufferLength = (BinaryPacketHeaderLength + MaxPacketDataLength)
* 2 + 1; // Add 1 byte to store \0

const int AsciiPacketHeaderLength = 6;
const int AsciiBufferLength = (AsciiPacketHeaderLength + MaxPacketDataLength)
* 2 + 1; // Add 1 byte to store \0

typedef struct
{
    unsigned char* data;
	int id;
	int length;
} BinaryPacket;

typedef struct
{
	unsigned char data[BinaryBufferLength];
	int length;
	unsigned int errors;
} BinaryBuffer;

typedef struct
{
	unsigned char* data;
	int length;
} AsciiPacket;

typedef struct
{
	unsigned char data[AsciiBufferLength];
	int length;
	unsigned int errors;
} AsciiBuffer;

class Protocol
{
public:
	BinaryBuffer BinaryPacketEncode(BinaryPacket binaryPacket);
	BinaryPacket BinaryBufferDecode(BinaryBuffer* binaryBuffer);
	AsciiBuffer AsciiPacketEncode(AsciiPacket asciiPacket);
	AsciiPacket AsciiBufferDecode(AsciiBuffer* asciiBuffer);
private:
	unsigned char CalculateHeaderLrc(unsigned char* data);
	unsigned short CalculateCrc16Ccitt(unsigned char* data, int length);
	unsigned short CalculateChecksum(unsigned char* data, int length);
};

//#endif
