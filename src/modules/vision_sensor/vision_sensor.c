/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>


#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/control_state.h>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int vision_sensor_task;				/**< Handle of daemon task / thread */
int vision_uart_read= -1;
struct control_state_s				_ctrl_state_q4;		/**< control state */
int _control_state_sub = -1;
bool _control_state_update = false;

/**
 * daemon management function.
 */
__EXPORT int vision_sensor_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int vision_sensor_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


/**
 * uart initial.
 */
static int uart_init(char * uart_name);

int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    if(fcntl(serial_fd, F_SETFL, FNDELAY) < 0)
        PX4_INFO("none block mode set failed!");
    return serial_fd;
}

static int set_uart_baudrate(const int fd, unsigned int baud);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    return true;
}
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int vision_sensor_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
        vision_sensor_task = px4_task_spawn_cmd("vision_sensor",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
                         vision_sensor_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int vision_sensor_thread_main(int argc, char *argv[])
{
    memset(&_ctrl_state_q4, 0, sizeof(_ctrl_state_q4));






    warnx("[daemon] starting\n");


    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */

    ///parse data from helicopter
    vision_uart_read = uart_init((char*)"/dev/ttyS6");
    PX4_INFO("uart_read:%d",vision_uart_read);
    if(false == vision_uart_read)
    {
        PX4_INFO("uart_read:%d",vision_uart_read);
    }
    if(false == set_uart_baudrate(vision_uart_read,57600))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }
    PX4_INFO("sesrial port initialized!");
	thread_running = true;

	while (!thread_should_exit) {

        if (_control_state_sub < 0)
            _control_state_sub  = orb_subscribe(ORB_ID(control_state));
        orb_check(_control_state_sub , &_control_state_update);
        if(_control_state_update)
        {
            orb_copy(ORB_ID(control_state), _control_state_sub, &_ctrl_state_q4);
        }




        math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
        math::Matrix<3, 3> R = q_att.to_dcm();
		warnx("Hello daemon!\n");
		sleep(10);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
