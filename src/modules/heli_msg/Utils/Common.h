/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file Common.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Common functions
///
/// Functions: MemoryPrint, BinaryPrint, GetTimestamp, GetTimeString, StatusPrint
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#if (defined _WIN32) || (defined __linux__) || (defined __QNXNTO__)

#pragma once

const double Pi = 3.14159265358979;
const int Radius = 6371004;

class Common
{
public:
	void MemoryPrint(unsigned char* data, int length);
	void BinaryPrint(int number);
	double GetTimestamp();
	char* GetTimeString();
	void StatusPrint(const char* message, const char* file,
		const char* function, int line);
};

#endif
