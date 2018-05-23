/// ---------------------------------------------------------------------------
/// Copyright (C) 2017 by J.F.Xiong. All rights reserved.
/// @file SharedMemory.h
/// @date 2017/8/18
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Shared memory
///
/// Map shared memory and initialize rwlock
/// Please add -lrt library if you're using linux
///
/// @version 2.1.0
/// @note Please feel free to contact me if you have any questions
/// ---------------------------------------------------------------------------

#if (defined __linux__) || (defined __QNXNTO__)

#pragma once

#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>

class SharedMemory
{
public:
	/// @return 0: success, 1: unable to create shared memory
	template<typename T> int Initialize(const char* name, T** ptr)
	{
		int length = sizeof(T);
		int fildes;
		/// 1 If the shared memory exists, open it
		if (access(name, F_OK) == 0)
		{
			fildes = shm_open(name, O_RDWR, S_IRWXU | S_IRWXG | S_IRWXO);
			*ptr = (T*)mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
				fildes, 0);
			close(fildes);
			return 0;
		}
		/// 2 Else, create shared memory and initialize it
		/// 2.1 Create
		fildes = shm_open(name, O_RDWR | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO);
		if (fildes == -1)
			return 1;
		/// 2.2 Truncate
		ftruncate(fildes, length);
		/// 2.3 Map
		*ptr = (T*)mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
			fildes, 0);
		close(fildes);
		/// 2.4 Set read/write lock
		pthread_rwlockattr_t attr;
		pthread_rwlockattr_init(&attr);
		pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
		pthread_rwlock_init(&(*ptr)->rwlock, &attr);

		return 0;
	}
};

#endif
