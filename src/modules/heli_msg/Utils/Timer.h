/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file Timer.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Timer
///
/// A simple timer for qnx
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#ifdef __QNXNTO__

#pragma once

#define PulseFromTimer _PULSE_CODE_MINAVAIL

typedef union
{
	struct _pulse pulse;
	/// Your other message structures would go here too
} TimerMessage;

class Timer
{
public:
	int chid;
	Timer(double interval);
	void Start();
	void Stop();
	void Close();
private:
	int coid;
	timer_t timerid;
	double interval;
};

#endif
