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
//#define HOMETEST    //use home position as leader
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
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
#include <uORB/topics/control_state.h>
#include <uORB/topics/vision_sensor.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/targ_heli.h>

#include <uORB/topics/home_position.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/ui_strive_gcs_to_formation.h>
#include <v2.0/common/mavlink.h>
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
struct vehicle_global_position_s vehicle_status;

int vehicle_status_sub = -1;
bool vehicle_status_update = false;
struct map_projection_reference_s own_ref;
struct targ_heli_s own_stat;
orb_advert_t own_stat_publish_fd;
orb_advert_t mavlink_log_pub;
#ifdef HOMETEST
struct home_position_s home_pos_tag;
int  home_pos_tag_sub ;
#endif
/*********** send to gcs every one second **************/
bool show_log_msg_timer = false;
hrt_abstime start_time; //used as timer

struct ui_strive_gcs_to_formation_s gcs_to_formation;
int gcs_to_formation_sub;
bool gcs_to_formation_update = false;

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
//        PX4_INFO("vision_x:%.3f,vision_y:%.3f,vision_z:%.3f,vision_vx:%.3f,vision_vy:%.3f,vision_vz:%.3f,distortion_x:%d,distortion_y:%d",
//         (double)vision_sensor_BODY.vision_x, (double)vision_sensor_BODY.vision_y, (double)vision_sensor_BODY.vision_z,
//         (double)vision_sensor_BODY.vision_vx,(double)vision_sensor_BODY.vision_vy, (double)vision_sensor_BODY.vision_vz,
//         vision_sensor_BODY.vision_distortion_x, vision_sensor_BODY.vision_distortion_y);
        PX4_INFO("vision status :%d",vision_sensor_msg.status);
        orb_publish(ORB_ID(vision_sensor), vision_sensor_pub_fd , &vision_sensor_BODY);
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
    _ctrl_state_q4.q[0] = 1.0f;
    memset(&vision_sensor_NED, 0, sizeof(vision_sensor_NED));
    memset(&vision_sensor_BODY, 0, sizeof(vision_sensor_BODY));
    memset(&own_ref, 0, sizeof(own_ref));
#ifdef HOMETEST
    memset(&home_pos_tag, 0, sizeof(home_pos_tag));
    home_pos_tag_sub = -1;
    home_pos_tag_sub  = orb_subscribe(ORB_ID(home_position));
#endif
    vision_sensor_pub_fd = orb_advertise(ORB_ID(vision_sensor), &vision_sensor_BODY);
    memset(&vehicle_status,0,sizeof(vehicle_status));
    own_stat_publish_fd = orb_advertise(ORB_ID(targ_heli), &own_stat);

    mavlink_log_pub = nullptr;
    warnx("[daemon] starting\n");
    _control_state_sub  = orb_subscribe(ORB_ID(control_state));
    vehicle_status_sub  = orb_subscribe(ORB_ID(vehicle_global_position));


    gcs_to_formation_sub = orb_subscribe(ORB_ID(ui_strive_gcs_to_formation));
    memset(&gcs_to_formation, 0, sizeof(gcs_to_formation));
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
   // PX4_INFO("uart_read:%d",vision_uart_read);
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
      //  PX4_INFO("bytes_read:%d",bytes_read);
        for(int32_t i = 0; i < bytes_read; i++)
        {
            if (mavlink_parse_char(0, receive_buf[i], &msg_received, &status))
            {
                sensor_handle_message(&msg_received);
            }
        }
//        if (_control_state_sub < 0)
//            _control_state_sub  = orb_subscribe(ORB_ID(control_state));
        orb_check(gcs_to_formation_sub, &gcs_to_formation_update);   //领航无人机经纬高
        if(1/*gcs_to_formation_update*/)
        {
            orb_copy(ORB_ID(ui_strive_gcs_to_formation), gcs_to_formation_sub, &gcs_to_formation);
      //      PX4_INFO("gcs_to_formation.lat:%.7f", gcs_to_formation.lat);
        }

        orb_check(vehicle_status_sub , &vehicle_status_update);
        if(vehicle_status_update)
        {
            orb_copy(ORB_ID(vehicle_global_position), vehicle_status_sub, &vehicle_status);
        //    PX4_INFO("vehicle_global_position: %.7f, %.7f, %.2f", vehicle_status.lat, vehicle_status.lon, (double)vehicle_status.alt);
        }

        orb_check(_control_state_sub , &_control_state_update);
        if(_control_state_update)
        {
            orb_copy(ORB_ID(control_state), _control_state_sub, &_ctrl_state_q4);
       //     PX4_INFO("ctrl_state_q4.q[0] :%.7f,:%.7f,:%.7f,:%.7f",(double)_ctrl_state_q4.q[0],(double)_ctrl_state_q4.q[1],(double)_ctrl_state_q4.q[2],(double)_ctrl_state_q4.q[3]);
        }

      //  math::Vector<3> xyz ={ x,y,z} ;
//        vision_sensor_BODY.vision_x = 0.17f;
//        vision_sensor_BODY.vision_y = -0.16f;
//        vision_sensor_BODY.vision_z = 0.95f;
//        mavlink_log_info(&mavlink_log_pub, "vision_sensor_BODY.status:%d", vision_sensor_BODY.status);
        math::Quaternion q_att(_ctrl_state_q4.q[0], _ctrl_state_q4.q[1], _ctrl_state_q4.q[2], _ctrl_state_q4.q[3]);
        math::Matrix<3, 3> R = q_att.to_dcm();
        if(vision_sensor_BODY.status == 3)
       {
           vision_sensor_NED.vision_x = vision_sensor_BODY.vision_x * R(0,0)
                                      + vision_sensor_BODY.vision_y * R(0,1)
                                      + vision_sensor_BODY.vision_z * R(0,2);

           vision_sensor_NED.vision_y = vision_sensor_BODY.vision_x * R(1,0)
                                      + vision_sensor_BODY.vision_y * R(1,1)
                                      + vision_sensor_BODY.vision_z * R(1,2);

           vision_sensor_NED.vision_z = vision_sensor_BODY.vision_x * R(2,0)
                                      + vision_sensor_BODY.vision_y * R(2,1)
                                      + vision_sensor_BODY.vision_z * R(2,2);

           vision_sensor_NED.vision_vx = vision_sensor_BODY.vision_vx * R(0,0)
                                       + vision_sensor_BODY.vision_vy * R(0,1)
                                       + vision_sensor_BODY.vision_vz * R(0,2);

           vision_sensor_NED.vision_vy = vision_sensor_BODY.vision_vx * R(1,0)
                                       + vision_sensor_BODY.vision_vy * R(1,1)
                                       + vision_sensor_BODY.vision_vz * R(1,2);

           vision_sensor_NED.vision_vz = vision_sensor_BODY.vision_vx * R(2,0)
                                       + vision_sensor_BODY.vision_vy * R(2,1)
                                       + vision_sensor_BODY.vision_vz * R(2,2);
           vision_sensor_NED.status = vision_sensor_BODY.status;

           PX4_INFO("vision_sensor_NED: %.2f, %.2f, %.2f",(double)vision_sensor_NED.vision_x,(double)vision_sensor_NED.vision_y,(double)vision_sensor_NED.vision_z);
           PX4_INFO("vision_sensor_BODY: %.2f, %.2f, %.2f",(double)vision_sensor_BODY.vision_x,(double)vision_sensor_BODY.vision_y,(double)vision_sensor_BODY.vision_z);
         //  mavlink_log_info(&mavlink_log_pub, "vision_sensor_BODY: %.2f, %.2f, %.2f",(double)vision_sensor_BODY.vision_x,(double)vision_sensor_BODY.vision_y,(double)vision_sensor_BODY.vision_z);
    //       if (vehicle_status_sub < 0)
    //           vehicle_status_sub  = orb_subscribe(ORB_ID(vehicle_global_position));
           own_stat.timestamp = hrt_absolute_time();
           map_projection_init(&own_ref,  vehicle_status.lat, vehicle_status.lon);
           map_projection_reproject(&own_ref, vision_sensor_NED.vision_x, vision_sensor_NED.vision_y,&own_stat.lat, &own_stat.lon);
           own_stat.alt = vehicle_status.alt + vision_sensor_NED.vision_z;
           own_stat.vel_e = vehicle_status.vel_e + vision_sensor_NED.vision_vx;
           own_stat.vel_n = vehicle_status.vel_n + vision_sensor_NED.vision_vy;
           own_stat.vel_d = vehicle_status.vel_d + vision_sensor_NED.vision_vz;
//           own_stat.yaw =  vehicle_status.yaw;
//           mavlink_log_info(&mavlink_log_pub, "vision_pos: %.7f, %.7f, %.2f", own_stat.lat, own_stat.lon, (double)own_stat.alt);
           orb_publish(ORB_ID(targ_heli), own_stat_publish_fd, &own_stat);
       }
#ifndef HOMETEST
       else
       {
           own_stat.timestamp = hrt_absolute_time();//gcs_to_formation.leader_timestamp;
           own_stat.lat = gcs_to_formation.lat;
           own_stat.lon = gcs_to_formation.lon;
           own_stat.alt = gcs_to_formation.alt;
           PX4_INFO("lead plane pos: %.7f, %.7f, %.2f", own_stat.lat, own_stat.lon, (double)own_stat.alt);
//           mavlink_log_info(&mavlink_log_pub, "lead plane pos: %.7f, %.7f, %.2f", own_stat.lat, own_stat.lon, (double)own_stat.alt);
           orb_publish(ORB_ID(targ_heli), own_stat_publish_fd, &own_stat);
       }
#else
       bool _home_pos_update;
       orb_check(home_pos_tag_sub , &_home_pos_update);
       if(_home_pos_update)
       {
           orb_copy(ORB_ID(home_position), home_pos_tag_sub, &home_pos_tag);
           PX4_INFO("home: lat:%.7f, lon:%.7f, alt:%.2f,yaw:%.1f", (double)home_pos_tag.lat, (double)home_pos_tag.lon, (double)home_pos_tag.alt, (double)home_pos_tag.yaw);
           own_stat.timestamp = hrt_absolute_time();
           own_stat.lon = home_pos_tag.lon;
           own_stat.lat = home_pos_tag.lat;
           own_stat.alt = home_pos_tag.alt;
           own_stat.yaw = home_pos_tag.yaw;
           own_stat.vel_n = 0.0f;
           own_stat.vel_e = 0.0f;
           own_stat.vel_d = 0.0f;
           mavlink_log_info(&mavlink_log_pub,"home.yaw:%.1f",(double)own_stat.yaw);
       }
#endif
       own_stat.timestamp = hrt_absolute_time();//delete after test *****ZJM
       //show log msg every second
       if(!show_log_msg_timer)
       {
           show_log_msg_timer = true;
           start_time = hrt_absolute_time();
       }
       else if(hrt_absolute_time() - start_time > 1e6)
       {
           show_log_msg_timer = false;
           mavlink_log_info(&mavlink_log_pub, "visons:%d,%.2f,%.2f,%.2f,lon:%.1f,alt:%.1f",vision_sensor_BODY.status,(double)vision_sensor_NED.vision_x,(double)vision_sensor_NED.vision_y,(double)vision_sensor_NED.vision_z, (double)own_stat.lon,(double)own_stat.alt);
       }
//       PX4_INFO("own_stat: lat:%.7f, lon:%.7f, alt:%.2f,yaw:%.1f", (double)own_stat.lat, own_stat.lon, (double)own_stat.alt, (double)own_stat.yaw);
        usleep(100000);

	}
	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
