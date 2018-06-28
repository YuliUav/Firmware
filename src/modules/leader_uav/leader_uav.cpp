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
#include <time.h>
#include <px4_config.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <lib/geo/geo.h>

#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <pthread.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/topics/vehicle_global_position.h>

#include <systemlib/mavlink_log.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int vision_sensor_task;				/**< Handle of daemon task / thread */
int uart_fd= -1;

struct vehicle_global_position_s global_position;
char buffer_send[67];
int global_position_sub;
/**
 * daemon management function.
 */
extern "C" __EXPORT int leader_uav_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int leader_uav_thread_main(int argc, char *argv[]);

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
int leader_uav_main(int argc, char *argv[])
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
        vision_sensor_task = px4_task_spawn_cmd("leader_uav",
                                                SCHED_DEFAULT,
                                                SCHED_PRIORITY_DEFAULT,
                                                2000,
                                                leader_uav_thread_main,
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

int leader_uav_thread_main(int argc, char *argv[])
{
    memset(buffer_send, 0, sizeof(buffer_send));
    char date_time[20];
    const char *protocol_header = "$40,";
    warnx("[daemon] starting\n");
    memset(&global_position, 0, sizeof(global_position));
    global_position_sub  = orb_subscribe(ORB_ID(vehicle_global_position));

    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    uart_fd = uart_init((char*)"/dev/ttyS6");

    if(false == uart_fd)
    {
        PX4_INFO("uart_read:%d",uart_fd);
    }
    if(false == set_uart_baudrate(uart_fd,57600))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }
    thread_running = true;
    while (!thread_should_exit) {
        bool global_position_update;
        orb_check(global_position_sub , &global_position_update);
        if(global_position_update)
        {
            orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
            //    PX4_INFO("vehicle_global_position: %.7f, %.7f, %.2f", global_position.lat, global_position.lon, (double)global_position.alt);
        }
        time_t nowtime = time(NULL);
        tm *now = localtime(&nowtime);
        strftime(date_time, sizeof(date_time), "%Y-%m-%d,%H:%M:%S,", now);
        PX4_INFO("%s", (char*)date_time);
        memcpy(buffer_send, protocol_header, 4);
        memcpy(buffer_send + 4, (char*)date_time, 20);
        sprintf(buffer_send + 24, "%010.6f,%09.6f,%06.2f,%.2f,%c*", global_position.lon, global_position.lat, (double)global_position.alt,0.46,'N');
        unsigned char check_sum = 0;
        for(int i = 0; i < 58; i++)
        {
            check_sum += buffer_send[i];
        }
        sprintf(buffer_send + 59, "%03d\r\n", check_sum);
        int32_t bytes_write = write(uart_fd, buffer_send, 67);
        PX4_INFO("%s, bytes_write:%d", (char*)buffer_send, bytes_write);
        sleep(1);
    }
    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
