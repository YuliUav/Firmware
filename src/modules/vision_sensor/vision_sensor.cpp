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
#include <lib/geo/geo.h>

//

#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <pthread.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vision_sensor.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/targ_heli.h>


//#include <v1.0/heli_msg/common/mavlink.h>
//#include <v1.0/heli_msg/mavlink_helpers.h>
//#include <v2.0/common/mavlink_msg_vision_data.h>
#include <v2.0/common/mavlink.h>
//#include <v2.0/mavlink_helpers.h>

#define BUFFERSIZE 128
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int vision_sensor_task;				/**< Handle of daemon task / thread */
int vision_uart_read= -1;
uint8_t receive_buf[BUFFERSIZE];
struct control_state_s _ctrl_state_q4;		/**< control state */
int _control_state_sub = -1;
bool _control_state_update = false;

orb_advert_t vision_sensor_pub_fd;
struct vision_sensor_s vision_sensor_NED;
struct vision_sensor_s vision_sensor_BODY;
int vehicle_status_sub = -1;
bool vehicle_status_update = false;
struct map_projection_reference_s own_ref;
struct targ_heli_s own_stat;
orb_advert_t own_stat_publish_fd;

/*************** mavlink message related data****************/
mavlink_message_t msg_received; //received mavlink message
mavlink_vision_data_t vision_sensor_msg;
mavlink_status_t status;


/**
 * daemon management function.
 */
extern "C" __EXPORT int vision_sensor_main(int argc, char *argv[]);


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

void sensor_handle_message(mavlink_message_t *msg);

void sensor_handle_message(mavlink_message_t *msg)
{
    PX4_INFO("msgid:%d",msg->msgid);
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_VISION_DATA:
        mavlink_msg_vision_data_decode(msg, &vision_sensor_msg);

        vision_sensor_BODY.vision_x = vision_sensor_msg.vision_x;
        vision_sensor_BODY.vision_y = vision_sensor_msg.vision_y;
        vision_sensor_BODY.vision_z = vision_sensor_msg.vision_z;
        vision_sensor_BODY.vision_vx = vision_sensor_msg.vision_vx;
        vision_sensor_BODY.vision_vy = vision_sensor_msg.vision_vy;
        vision_sensor_BODY.vision_vz = vision_sensor_msg.vision_vz;
        vision_sensor_BODY.vision_distortion_x = vision_sensor_msg.pixel_x;
        vision_sensor_BODY.vision_distortion_y = vision_sensor_msg.pixel_y;
        vision_sensor_BODY.status = vision_sensor_msg.status;
        PX4_INFO("vision_x:%.3f,vision_y:%.3f,vision_z:%.3f,vision_vx:%.3f,vision_vy:%.3f,vision_vz:%.3f,distortion_x:%d,distortion_y:%d",
         (double)vision_sensor_BODY.vision_x, (double)vision_sensor_BODY.vision_y, (double)vision_sensor_BODY.vision_z,
         (double)vision_sensor_BODY.vision_vx,(double)vision_sensor_BODY.vision_vy, (double)vision_sensor_BODY.vision_vz,
         vision_sensor_BODY.vision_distortion_x, vision_sensor_BODY.vision_distortion_y);
      //  orb_publish(ORB_ID(targ_heli), heli_stat_pub_fd, &heli_stat);
        break;
     }
}
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
    memset(receive_buf, 0, sizeof(receive_buf));
    memset(&msg_received, 0, sizeof(msg_received));
    memset(&status, 0, sizeof(status));

    memset(&_ctrl_state_q4, 0, sizeof(_ctrl_state_q4));
    memset(&vision_sensor_NED, 0, sizeof(vision_sensor_NED));
    memset(&vision_sensor_BODY, 0, sizeof(vision_sensor_BODY));
    vision_sensor_pub_fd = orb_advertise(ORB_ID(vision_sensor), &vision_sensor_NED);

    struct vehicle_global_position_s vehicle_status;
    memset(&vehicle_status,0,sizeof(vehicle_status));

    own_stat_publish_fd = orb_advertise(ORB_ID(targ_heli), &own_stat);
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
        int32_t bytes_read = read(vision_uart_read, receive_buf, BUFFERSIZE);
        PX4_INFO("bytes_read:%d",bytes_read);
        for(int32_t i = 0; i < bytes_read; i++)
        {
            if (mavlink_parse_char(0, receive_buf[i], &msg_received, &status))
            {
                printf("Received message with ID %d, sequence: %d from component %d of system %d", msg_received.msgid, msg_received.seq, msg_received.compid, msg_received.sysid);
                sensor_handle_message(&msg_received);
            }
        }
        if (_control_state_sub < 0)
            _control_state_sub  = orb_subscribe(ORB_ID(control_state));
        orb_check(_control_state_sub , &_control_state_update);
        if(_control_state_update)
        {
            orb_copy(ORB_ID(control_state), _control_state_sub, &_ctrl_state_q4);
        }
        math::Quaternion q_att(_ctrl_state_q4.q[0], _ctrl_state_q4.q[1], _ctrl_state_q4.q[2], _ctrl_state_q4.q[3]);
        math::Matrix<3, 3> R = q_att.to_dcm();
      //  math::Vector<3> xyz ={ x,y,z} ;
       vision_sensor_NED.vision_x = vision_sensor_BODY.vision_x * R(0,0)
                                  + vision_sensor_BODY.vision_y * R(0,1)
                                  + vision_sensor_BODY.vision_z * R(0,2);

       vision_sensor_NED.vision_y = vision_sensor_BODY.vision_x * R(1,0)
                                  + vision_sensor_BODY.vision_y * R(1,1)
                                  + vision_sensor_BODY.vision_z * R(1,2);

       vision_sensor_NED.vision_z = vision_sensor_BODY.vision_x * R(2,0)
                                  + vision_sensor_BODY.vision_y * R(2,1)
                                  + vision_sensor_BODY.vision_z * R(2,2);
///
       vision_sensor_NED.vision_vx = vision_sensor_BODY.vision_vx * R(0,0)
                                   + vision_sensor_BODY.vision_vy * R(0,1)
                                   + vision_sensor_BODY.vision_vz * R(0,2);

       vision_sensor_NED.vision_vy = vision_sensor_BODY.vision_vx * R(1,0)
                                   + vision_sensor_BODY.vision_vy * R(1,1)
                                   + vision_sensor_BODY.vision_vz * R(1,2);

       vision_sensor_NED.vision_vz = vision_sensor_BODY.vision_vx * R(2,0)
                                   + vision_sensor_BODY.vision_vy * R(2,1)
                                   + vision_sensor_BODY.vision_vz * R(2,2);

       orb_publish(ORB_ID(vision_sensor), vision_sensor_pub_fd , &vision_sensor_NED);

       if (vehicle_status_sub < 0)
           vehicle_status_sub  = orb_subscribe(ORB_ID(vehicle_global_position));
       orb_check(vehicle_status_sub , &vehicle_status_update);
       if(vehicle_status_update)
       {
           orb_copy(ORB_ID(vehicle_global_position), vehicle_status_sub, &vehicle_status);
       }
       map_projection_init(&own_ref,  vehicle_status.lat, vehicle_status.lon);
       map_projection_reproject(&own_ref, vision_sensor_NED.vision_x, vision_sensor_NED.vision_y,
                                &own_stat.lat, &own_stat.lon);
       own_stat.alt = vehicle_status.alt + vision_sensor_NED.vision_z;
       own_stat.vel_e = vehicle_status.vel_e + vision_sensor_NED.vision_vx;
       own_stat.vel_n = vehicle_status.vel_n + vision_sensor_NED.vision_vy;
       own_stat.vel_d = vehicle_status.vel_d + vision_sensor_NED.vision_vz;
       own_stat.yaw =  vehicle_status.yaw;


       orb_publish(ORB_ID(targ_heli), own_stat_publish_fd, &own_stat);
//        warnx("Hello daemon!\n");
        usleep(100000); //100ms
	}
	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
