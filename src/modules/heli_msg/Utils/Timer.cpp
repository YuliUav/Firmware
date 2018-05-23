#ifdef __QNXNTO__

#include <sys/dispatch.h>
#include <sys/netmgr.h>
#include "Timer.h"

Timer::Timer(double interval)
{
	chid = ChannelCreate(0);
	coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);

	struct sigevent event;
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = coid;
	event.sigev_priority = getprio(0);
	event.sigev_code = PulseFromTimer;

	timer_create(CLOCK_REALTIME, &event, &timerid);

	this->interval = interval;
}

void Timer::Start()
{
	int sec = interval / 1000;
	int nsec = (int)(interval * 1000000) % 1000000000;

	struct itimerspec itimer;
	itimer.it_value.tv_sec = sec;
	itimer.it_value.tv_nsec = nsec;
	itimer.it_interval.tv_sec = sec;
	itimer.it_interval.tv_nsec = nsec;

	timer_settime(timerid, 0, &itimer, NULL);
}

void Timer::Stop()
{
	struct itimerspec itimer;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_nsec = 0;

	timer_settime(timerid, 0, &itimer, NULL);
}

void Timer::Close()
{
	timer_delete(timerid);
	ConnectDetach(coid);
}

#endif
