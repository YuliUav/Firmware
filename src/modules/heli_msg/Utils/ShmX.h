/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file ShmX.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Shared memory demo
///
/// Description of the shared memory
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#if (defined __linux__) || (defined __QNXNTO__)

#pragma once

typedef struct
{
	pthread_rwlock_t rwlock;
	double timestamp;
	int parameter;
} ShmX;

#endif
