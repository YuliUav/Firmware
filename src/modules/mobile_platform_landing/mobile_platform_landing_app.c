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


#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int mobile_platform_landing_task;	/**< Handle of daemon task / thread */



/**
 *daemon management function.
 */
//extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);
 __EXPORT int mobile_platform_landing_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int mobile_platform_landing_thread_main(int argc, char *argv[]);

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
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */

/**********************usart**********

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

    // fill the struct for the new configuration
    tcgetattr(fd, &uart_config);
    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;
    //no parity, one stop bit
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    // set baud rate
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

************************************/


/*************mavlink***********************

void sensor_handle_message(mavlink_message_t *msg);

void sensor_handle_message(mavlink_message_t *msg)
{
////    PX4_INFO("msgid:%d",msg->msgid);
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_VISION_SENSOR:
        mavlink_msg_vision_sensor_decode(msg, &vision_sensor_msg);

        vision_sensor_BODY.vision_x = vision_sensor_msg.vision_x;
        vision_sensor_BODY.vision_y = vision_sensor_msg.vision_y;
        vision_sensor_BODY.vision_z = vision_sensor_msg.vision_z;
        vision_sensor_BODY.vision_distortion_x = vision_sensor_msg.vision_distortion_x;
        vision_sensor_BODY.vision_distortion_y = vision_sensor_msg.vision_distortion_y;

      //  orb_publish(ORB_ID(targ_heli), heli_stat_pub_fd, &heli_stat);
        break;
     }
}


*************************************/




int mobile_platform_landing_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("mobile_platform_landing already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		mobile_platform_landing_task = px4_task_spawn_cmd("mobile_platform_landing",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 mobile_platform_landing_thread_main,
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

int mobile_platform_landing_thread_main(int argc, char *argv[])
{

	warnx("mobile_platform_landing starting\n");

	thread_running = true;

    /*  串口初始化
        * TELEM1 : /dev/ttyS1
        * TELEM2 : /dev/ttyS2
        * GPS    : /dev/ttyS3
        * NSH    : /dev/ttyS5
        * SERIAL4: /dev/ttyS6
        * N/A    : /dev/ttyS4
        * IO DEBUG (RX only):/dev/ttyS0


       ///
       vision_uart_read = uart_init((char*)"/dev/ttyS6");//选择对应接口的串口号
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

*/
    while (!thread_should_exit) {
		warnx("Hello daemon!\n");
		sleep(10);















    }

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
