/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file helic_msg_app.c
 *  2018-03-29
 * @author: Zhang Jimin<869159813@qq.com>
 *
 * receiver of the information of the target helicopter
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <pthread.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>


#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
//#include <lib/mathlib/math/Vector.hpp>
//#include <lib/mathlib/math/Matrix.hpp>
#include <fcntl.h>
#include <uORB/topics/targ_heli.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <systemlib/mavlink_log.h>
#include <controllib/block/BlockParam.hpp>
#include <lib/controllib/block/Block.hpp>

#include "Utils/Protocol.h"

#define DT 100000
static bool thread_should_exit = false;		/**< heli_msg_app exit flag */
static bool thread_running = false;		/**< heli_msg_app status flag */
static int daemon_task;				/**< Handle of heli_msg_app task / thread */
int uart_read= -1;
/**
 * management function.
 */
extern "C" __EXPORT int heli_msg_app_main(int argc, char *argv[]);

/**
 * Mainloop
 */
int heli_msg_app_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * uart initial.
 */
static int uart_init(char * uart_name);


static int set_uart_baudrate(const int fd, unsigned int baud);

class Para_Update : public control::SuperBlock
{
public:
    Para_Update();
};
Para_Update :: Para_Update():SuperBlock(NULL, "HELI")
{

}

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


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

/**
 * heli_msg
 */
int heli_msg_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
        return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("heli_msg_app already running\n");
			/* this is not an error */
            return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("heli_msg_app",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
                         3500,
					     heli_msg_app_thread_main,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        unsigned constexpr max_wait_us = 1000000;
        unsigned constexpr max_wait_steps = 2000;
        unsigned i;
        for (i = 0; i < max_wait_steps; i++) {
            usleep(max_wait_us / max_wait_steps);
            if (thread_running) {
                break;
            }
        }

        return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {
        if (!thread_running) {
            warnx("commander already stopped");
            return 0;
        }

        thread_should_exit = true;

        while (thread_running) {
            if(uart_read > 0)       //close serial port
                if(close(uart_read) == 0)   uart_read = -1;
            usleep(200000);
            warnx(".");
        }
        warnx("terminated.");
        return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

    usage("\tunrecognized command");
    return 1;
}

int heli_msg_app_thread_main(int argc, char *argv[])
{
	thread_running = true;
    PX4_INFO("initializing!\n");


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
    uart_read = uart_init((char*)"/dev/ttyS6");
    PX4_INFO("uart_read:%d",uart_read);
    if(false == uart_read)
    {
        PX4_INFO("uart_read:%d",uart_read);
    }
    if(false == set_uart_baudrate(uart_read,115200))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }
    PX4_INFO("sesrial port initialized!");

    //message to be published
    struct targ_heli_s heli_stat;
    memset(&heli_stat, 0, sizeof(heli_stat));
    orb_advert_t heli_stat_pub_fd = orb_advertise(ORB_ID(targ_heli), &heli_stat);


    //parameters from GCS
    Para_Update para_update;
    int		_params_sub = -1;			/**< notification of parameter updates */
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    param_t     _param_lon_offset;
    param_t     _param_lat_offset;
    param_t		_param_alt_offset;

    float     lon_offset = 0.0f;
    float     lat_offset = 0.0f;
    float     alt_offset = 0.0f;

    _param_lon_offset	= param_find("HELI_LON_OFFSET");
    _param_lat_offset	= param_find("HELI_LAT_OFFSET");
    _param_alt_offset	= param_find("HELI_ALT_OFFSET");

    //decode buffer
    Protocol protocol;

    BinaryBuffer binaryReceiveBuffer;
    memset(binaryReceiveBuffer.data, 0, BinaryBufferLength);
    binaryReceiveBuffer.length = 0;

    BinaryPacket binaryPacket;

    hrt_abstime send_heartbeat_time = 0;
    uint32_t deta_heartbeat_heli = 0;
    uint32_t deta_heartbeat_craw = 0;

    hrt_abstime current_time_5Hz = 0;
    hrt_abstime current_time_1Hz = 0;

    //encode protocol
    BinaryBuffer binarySendDataBuffer;
    unsigned char binarySendData[2] = {0};
    BinaryPacket binarySendDataPacket ={ binarySendData, 30, 2 };
    BinaryBuffer binarySendHeartbeatBuffer;
    unsigned char binarySendHeartbeat[1] = {0};
    BinaryPacket binarySendHeartbeatPacket ={ binarySendHeartbeat, 31, 1};
    bool heartbeat = false;
    bool trackReady = true;
    bool catchSuccess = false;
    bool time_out_5Hz = true;
    bool time_out_1Hz = true;

    /* Mavlink log uORB handle */
    orb_advert_t mavlink_log_pub = nullptr;
//    long stack_size = 0;
    pthread_attr_t attr;
    int ret = pthread_attr_init(&attr);
    if(ret != 0)
    {
        PX4_INFO("pthread attr init");
    }

    while(!thread_should_exit){
        /********** get gcs parameters *************/
        bool updated;
        //get GCS parameters
        struct parameter_update_s param_upd;
        orb_check(_params_sub, &updated);

        if (updated) {
            orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
        }
        para_update.updateParams();
        param_get(_param_lon_offset, &lon_offset);
        param_get(_param_lat_offset, &lat_offset);
        param_get(_param_alt_offset, &alt_offset);
//        PX4_INFO("lon_offset:%.1f,%.8f,lat_offset:%.1f,%.8f,alt_offset:%.1f%.4f,",(double)lon_offset,(double)lon_offset/1e7,(double)lat_offset,(double)lat_offset/1e7,(double)alt_offset,(double)(alt_offset/1e3f));

        ssize_t bytes_read= read(uart_read, binaryReceiveBuffer.data + binaryReceiveBuffer.length, BinaryBufferLength);
        if(bytes_read > 0)      //got data
        {
            binaryReceiveBuffer.length += bytes_read;
//            PX4_INFO("bytes_read:%d, Buffer.length:%d",bytes_read,binaryReceiveBuffer.length);
            binaryPacket = protocol.BinaryBufferDecode(&binaryReceiveBuffer);
            //        PX4_INFO("buffer.length:%d,stack_size:%d,binaryPacket.length:%d,binaryPacket.id:%d", binaryReceiveBuffer.length, stack_size,binaryPacket.length,binaryPacket.id);
            if(binaryPacket.id == 0)    //info msg from target
            {
                memcpy(&heli_stat.lon,binaryPacket.data + 0, 8);
                memcpy(&heli_stat.lat,binaryPacket.data + 8, 8);
                memcpy(&heli_stat.alt,binaryPacket.data + 16, 4);
                memcpy(&heli_stat.yaw,binaryPacket.data + 20, 4);
                memcpy(&heli_stat.vel_n,binaryPacket.data + 24, 4);
                memcpy(&heli_stat.vel_e,binaryPacket.data + 28, 4);
                memcpy(&heli_stat.vel_d,binaryPacket.data + 32, 4);
                memcpy(&heli_stat.x,binaryPacket.data + 36, 4);
                memcpy(&heli_stat.y,binaryPacket.data + 40, 4);
                memcpy(&heli_stat.z,binaryPacket.data + 44, 4);
                memcpy(&heli_stat.vel_x,binaryPacket.data + 48, 4);
                memcpy(&heli_stat.vel_y,binaryPacket.data + 52, 4);
                memcpy(&heli_stat.vel_z,binaryPacket.data + 56, 4);
                memcpy(&heli_stat.track_enabled,binaryPacket.data + 60, 1);
                memcpy(&heli_stat.catched,binaryPacket.data + 61, 1);
                heli_stat.timestamp = hrt_absolute_time();
                PX4_INFO("lon:%.7f,lat:%.7f,alt:%.3f,yaw:%.3f,vN:%.3f,vE:%.3f,vD:%.3f,x:%.3f,y:%.3f,z:%.3f,vx:%.3f,vy:%.3f,vz:%.3f,track:%d,catch:%d",
                         heli_stat.lon,heli_stat.lat,(double)heli_stat.alt,(double)heli_stat.yaw,(double)heli_stat.vel_n,(double)heli_stat.vel_e,(double)heli_stat.vel_d,(double)heli_stat.x,(double)heli_stat.y,(double)heli_stat.z,(double)heli_stat.vel_x,
                         (double)heli_stat.vel_y,(double)heli_stat.vel_z,heli_stat.track_enabled,heli_stat.catched);
                heli_stat.lon += (double)lon_offset / 1e7;         //change to degree
                heli_stat.lat += (double)lat_offset / 1e7;         //change to degree
                heli_stat.alt += alt_offset / 1e3f;                  //change to meter
                orb_publish(ORB_ID(targ_heli), heli_stat_pub_fd, &heli_stat);

            }
            else if(binaryPacket.id == 01)   //heartbeat from heli
            {
                deta_heartbeat_heli =  (uint32_t)(hrt_absolute_time() - send_heartbeat_time) / 2000;
                PX4_INFO("deta_heartbeat_heli:%u",deta_heartbeat_heli);
            }
            else if(binaryPacket.id == 02)  //heartbeat from craw
            {
                deta_heartbeat_craw = (uint32_t)(hrt_absolute_time() - send_heartbeat_time) / 2000;
                PX4_INFO("deta_heartbeat_craw:%u",deta_heartbeat_craw);
            }
            delete binaryPacket.data;
            mavlink_log_info(&mavlink_log_pub, "dely heli:%u,craw:%u", deta_heartbeat_heli, deta_heartbeat_craw);
        }

        /******************send data msg********************/
        if(time_out_5Hz)
        {
            time_out_5Hz = false;
            current_time_5Hz = hrt_absolute_time();
        }
        else if(hrt_absolute_time() - current_time_5Hz > 200000)
        {
            //get data

            memcpy(binarySendDataPacket.data, &trackReady, 1);
            memcpy(binarySendDataPacket.data + 1 ,&catchSuccess, 1);
            binarySendDataBuffer = protocol.BinaryPacketEncode(binarySendDataPacket);
            write(uart_read, binarySendDataBuffer.data, binarySendDataBuffer.length);
            time_out_5Hz = true;
            PX4_INFO("track data sended to heli");
        }

        /**************** send heartbeat msg ****************/
        if(time_out_1Hz)
        {
            time_out_1Hz = false;
            current_time_1Hz = hrt_absolute_time();
        }
        else if(hrt_absolute_time() - current_time_1Hz > 1000000)
        {
            //get heartbeat data
            memcpy(binarySendHeartbeatPacket.data, &heartbeat, 1);
            binarySendHeartbeatBuffer = protocol.BinaryPacketEncode(binarySendHeartbeatPacket);

            write(uart_read, binarySendHeartbeatBuffer.data, binarySendHeartbeatBuffer.length);
            send_heartbeat_time = hrt_absolute_time();
            time_out_1Hz = true;
            PX4_INFO("HEARTBEAT SEND!");
        }

        /******************* climb up to expected alititude ****************/

        usleep(1000);       //if not sleep in the while(), the thread will be jamed
    }    

    warnx("exiting.\n");
    thread_running = false;
    return 0;
}
