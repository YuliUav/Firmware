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
#define PARSEHELI
//#define MCSP        //get manual_control_setpoint
#define MAVLINK     //use mavlink message protocol insted of utiles
#define VEHICLE_STATUS // get vehicle status(pos and vel)
#define BufferLength 64
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

#include <fcntl.h>
#include <uORB/topics/targ_heli.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vision_sensor.h>

#include <systemlib/mavlink_log.h>
#include <controllib/block/BlockParam.hpp>
#include <lib/controllib/block/Block.hpp>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/heli_followtarg.h>

#ifdef MCSP
#include <uORB/topics/manual_control_setpoint.h>
#endif

#ifdef MAVLINK
#include <v1.0/heli_msg/common/mavlink.h>
#include <v1.0/heli_msg/mavlink_helpers.h>
#include <v1.0/heli_msg/common/mavlink_msg_heli_msg.h>
#include <v1.0/heli_msg/common/mavlink_msg_craw_heartbeat.h>
#include <v1.0/heli_msg/common/mavlink_msg_heli_heartbeat.h>
#include <v1.0/heli_msg/common/mavlink_msg_mc2heli_heartbeat.h>
#include <v1.0/heli_msg/common/mavlink_msg_mc2heli.h>
#endif

#ifdef VEHICLE_STATUS
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <v1.0/heli_msg/common/mavlink_msg_lead_heli_msg.h>
#endif

#include <lib/geo/geo.h>
//#include "Utils/Protocol.h"

#define DT 10000
static bool thread_should_exit = false;		/**< heli_msg_app exit flag */
static bool thread_running = false;		/**< heli_msg_app status flag */
static int daemon_task;				/**< Handle of heli_msg_app task / thread */
int uart_read= -1;
hrt_abstime send_heartbeat_time = 0;
uint32_t deta_heartbeat_heli = 0;
uint32_t deta_heartbeat_craw = 0;

hrt_abstime current_time_5Hz = 0;
hrt_abstime current_time_1Hz = 0;
#ifdef MAVLINK
mavlink_heli_msg_t heli_msg;                //receive data
mavlink_heli_msg_t send_heli_msg;           //send data
mavlink_message_t sendMavMsg;                      //packeted data
mavlink_message_t sendHeartbeatMsg;                      //packeted data
//mavlink_message_t sendDataPacket;           //send data to helicoptor

mavlink_mc2heli_t mc2heli;                  //send data to helicoptor
mavlink_mc2heli_heartbeat_t mc2heli_heartbeat;//send heartbeat to helicoptor
mavlink_lead_heli_msg_t lead_heli_msg;
uint8_t sendBuffer[64];
#endif
struct targ_heli_s heli_stat;
orb_advert_t heli_stat_pub_fd;
orb_advert_t mavlink_log_pub = nullptr;

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
void handle_message(mavlink_message_t *msg);

void handle_message(mavlink_message_t *msg)
{
//    PX4_INFO("msgid:%d",msg->msgid);
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HELI_MSG:
        mavlink_msg_heli_msg_decode(msg, &heli_msg);
        heli_stat.lon = heli_msg.lon / 1e7f;
        heli_stat.lat = heli_msg.lat / 1e7f;
        heli_stat.alt = heli_msg.alt / 1e3f;
        heli_stat.yaw = heli_msg.yaw;
        heli_stat.vel_n = heli_msg.velN;
        heli_stat.vel_e = heli_msg.velE;
        heli_stat.vel_d = heli_msg.velD;
        heli_stat.x = heli_msg.x;
        heli_stat.y = heli_msg.y;
        heli_stat.z = heli_msg.z;
        heli_stat.vel_x = heli_msg.velX;
        heli_stat.vel_y = heli_msg.velY;
        heli_stat.vel_z = heli_msg.velZ;
        heli_stat.track_enabled = heli_msg.trackEnalbled;
        heli_stat.catched = heli_msg.catched;
        orb_publish(ORB_ID(targ_heli), heli_stat_pub_fd, &heli_stat);
     //   mavlink_log_info(&mavlink_log_pub,"targ_heli published!");
        PX4_INFO("lon:%.7f,lat:%.7f,alt:%.3f,yaw:%.1f,vel_n:%.1f,vel_e:%.1f,vel_d:%.1f,x:%.1f,y:%.1f,z:%.1f,vel_x:%.1f,vel_y:%.1f,vel_z:%.1f,track_enabled:%d,catched:%d,msg_seq:%d",
                 heli_stat.lon,heli_stat.lat,(double)heli_stat.alt,(double)heli_stat.yaw,(double)heli_stat.vel_n,(double)heli_stat.vel_e,(double)heli_stat.vel_d,
                 (double)heli_stat.x,(double)heli_stat.y,(double)heli_stat.z,(double)heli_stat.vel_x,(double)heli_stat.vel_y,(double)heli_stat.vel_z,heli_stat.track_enabled,heli_stat.catched,msg->seq);
        break;

    case MAVLINK_MSG_ID_HELI_HeartBeat:
        deta_heartbeat_heli =  (uint32_t)(hrt_absolute_time() - send_heartbeat_time) / 2000;
        PX4_INFO("deta_heartbeat_heli:%u, ***********************************************************************************************************************************msg_seq:%d",deta_heartbeat_heli,msg->seq);

        break;

    case MAVLINK_MSG_ID_Craw_HeartBeat:
        deta_heartbeat_craw = (uint32_t)(hrt_absolute_time() - send_heartbeat_time) / 2000;
        PX4_INFO("deta_heartbeat_craw:%u",deta_heartbeat_craw);
        break;

    default:
        break;
    }
}
static void usage(const char *reason)
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

/************* two setpoints struct****************/
typedef struct{
    double lon;
    double lat;
    float alt;
} Pos_Setpoint;

typedef struct{
    double lon_offset;
    double lat_offset;
    float alt_offset;
} Pos_Offset;


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
                         SCHED_PRIORITY_DEFAULT + 50,
                         3000,
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
    // Mavlink log uORB handle

#ifdef MAVLINK
    /* the serial port buffers internally as well, we just need to fit a small chunk */
    uint8_t receiveBuf[BufferLength];
    memset(receiveBuf, 0, sizeof(receiveBuf));
    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&msg, 0, sizeof(msg));
    memset(&status, 0, sizeof(status));
    memset(&heli_msg, 0, sizeof(heli_msg));
    memset(&sendMavMsg, 0, sizeof(sendMavMsg));
    memset(&sendHeartbeatMsg, 0, sizeof(sendHeartbeatMsg));
#endif
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
    if(false == set_uart_baudrate(uart_read,57600))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }
    PX4_INFO("sesrial port initialized!");
#ifdef PARSEHELI
     //two position setpoints
    Pos_Setpoint first_pos;
    Pos_Offset  pos_offset;
    memset(&first_pos, 0, sizeof(first_pos));
    memset(&pos_offset, 0, sizeof(pos_offset));
#endif
    //message to be published
    memset(&heli_stat, 0, sizeof(heli_stat));
    heli_stat_pub_fd = orb_advertise(ORB_ID(targ_heli), &heli_stat);

    bool heartbeat = false;
    bool trackReady = true;
    bool catchSuccess = true;
    bool time_out_5Hz = true;
    bool time_out_1Hz = true;

    /*********** receive data from navigator thread **************/
    float z = 0.0f;
    bool first_time = true;
    bool arrive_flag = false;
    hrt_abstime firsttime = 0;
    hrt_abstime lasttime = 0;
    struct heli_followtarg_s follow_result;
    memset(&follow_result,0,sizeof(follow_result));
    int follow_result_sub = -1;
    follow_result_sub = orb_subscribe(ORB_ID(heli_followtarg));
#ifdef MCSP
    /*********** receive data from manual controller **************/
    struct manual_control_setpoint_s sp_man = {};		///< the current manual control setpoint
    int sp_man_sub  = -1;
    memset(&sp_man, 0, sizeof(sp_man));
#endif

#ifdef VEHICLE_STATUS
    struct vehicle_global_position_s vehicle_status;
    memset(&vehicle_status,0,sizeof(vehicle_status));
    int vehicle_status_sub = -1;
    bool vehicle_status_update = false;
    struct map_projection_reference_s own_ref;
#endif
    struct vision_sensor_s vision_receive;
    memset(&vision_receive,0,sizeof(vision_receive));
    int vision_receive_sub = -1;
    bool vision_receive_sub_updated = false;
//    long stack_size = 0;
    pthread_attr_t attr;
    int ret = pthread_attr_init(&attr);
    if(ret != 0)
    {
        PX4_INFO("pthread attr init");
    }

//    bool updated = false;
    bool follow_result_update = false;

    while(!thread_should_exit){

#ifdef MCSP
        if(sp_man_sub < 0)
            sp_man_sub  = orb_subscribe(ORB_ID(manual_control_setpoint));
        orb_check(sp_man_sub, &updated);
        if (updated) {
            orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
        }
#endif
        orb_check(follow_result_sub, &follow_result_update);
        if(follow_result_update)
        {
            orb_copy(ORB_ID(heli_followtarg), follow_result_sub, &follow_result);
//            mavlink_log_info(&mavlink_log_pub, "finally_arrive:%d",follow_result.finally_arrive);
            PX4_INFO("finally_arrive:%d",follow_result.finally_arrive);
        }

        if (vision_receive_sub < 0)
            vision_receive_sub  = orb_subscribe(ORB_ID(vision_sensor));
        orb_check(vision_receive_sub , & vision_receive_sub_updated);
        if( vision_receive_sub_updated)
        {
            orb_copy(ORB_ID(vision_sensor), vision_receive_sub, &vision_receive);
        }

#ifdef VEHICLE_STATUS
        if (vehicle_status_sub < 0)
            vehicle_status_sub  = orb_subscribe(ORB_ID(vehicle_global_position));
        orb_check(vehicle_status_sub , &vehicle_status_update);
        if(vehicle_status_update)
        {
            orb_copy(ORB_ID(vehicle_global_position), vehicle_status_sub, &vehicle_status);
        //////////
            lead_heli_msg.lat = static_cast<int>(vehicle_status.lat * 1e7);
            lead_heli_msg.lon = static_cast<int>(vehicle_status.lon * 1e7);
            lead_heli_msg.alt = static_cast<int>((double)vehicle_status.alt * 1e3);
            lead_heli_msg.velD = vehicle_status.vel_d;
            lead_heli_msg.velE = vehicle_status.vel_e;
            lead_heli_msg.velN = vehicle_status.vel_n;
        //////////
            send_heli_msg.lat = static_cast<int>(vehicle_status.lat * 1e7);
            send_heli_msg.lon = static_cast<int>(vehicle_status.lon * 1e7);
            send_heli_msg.alt = static_cast<int>((double)vehicle_status.alt * 1e3);
            send_heli_msg.velD = vehicle_status.vel_d;
            send_heli_msg.velE = vehicle_status.vel_e;
            send_heli_msg.velN = vehicle_status.vel_n;
            ///////

            // PX4_INFO("vehicle_status_update");
        }
#endif
#ifdef MAVLINK
        ssize_t bytes_read= read(uart_read, receiveBuf, BufferLength);
        if(bytes_read > 0)      //got data
        {
            PX4_INFO("got data");
            for (ssize_t i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(0, receiveBuf[i], &msg, &status)) {
                    /* handle generic messages and commands */
                    handle_message(&msg);
                     PX4_INFO("handle_message(&msg)");
                }
            }
#endif
//            binaryPacket.data = nullptr;
//            mavlink_log_info(&mavlink_log_pub, "dely heli:%u,craw:%u", deta_heartbeat_heli, deta_heartbeat_craw);
        }
        /****************** send data msg ********************/
        if(time_out_5Hz)
        {
            time_out_5Hz = false;
            current_time_5Hz = hrt_absolute_time();
        }
        else if(hrt_absolute_time() - current_time_5Hz > 200000)
        {
            //get data
            /************************ !!!! warning: anyone who using this program should assign the "trackReady" and "catchSuccess" values  according to real situation *********************/
            mc2heli.trackReady = trackReady;
            mc2heli.catchSuccess = catchSuccess;
            mavlink_msg_mc2heli_encode(0, 0, &sendMavMsg, &mc2heli);
            int nBytes = mavlink_msg_to_send_buffer(sendBuffer, &sendMavMsg);
            write(uart_read, sendBuffer, nBytes);
            PX4_INFO("mc2heli sended, seq:%d", sendMavMsg.seq);
#ifdef VEHICLE_STATUS
            mavlink_msg_lead_heli_msg_encode(0, 0, &sendMavMsg, &lead_heli_msg);
            int mBytes = mavlink_msg_to_send_buffer(sendBuffer, &sendMavMsg);
            write(uart_read, sendBuffer, mBytes);
            PX4_INFO("lead_heli_msg sended");
            mavlink_msg_heli_msg_encode(0, 0, &sendMavMsg, &send_heli_msg);
            int xBytes = mavlink_msg_to_send_buffer(sendBuffer, &sendMavMsg);
            write(uart_read, sendBuffer, xBytes);
            PX4_INFO("send_heli_msg sended");
#endif
            time_out_5Hz = true;
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
            mc2heli_heartbeat.heartBeat = heartbeat;        //useless data
            mavlink_msg_mc2heli_heartbeat_encode(0, 0, &sendHeartbeatMsg, &mc2heli_heartbeat);
            int nBytes = mavlink_msg_to_send_buffer(sendBuffer, &sendHeartbeatMsg);
            write(uart_read, sendBuffer, nBytes);
            send_heartbeat_time = hrt_absolute_time();
            time_out_1Hz = true;
            PX4_INFO("HEARTBEAT SEND,seq:%d",sendHeartbeatMsg.seq);
           // mavlink_log_info(&mavlink_log_pub,"HEARTBEAT SEND");
        }
#ifdef MCSP
        /*************** get two setpoints by rc*****************/
        if(sp_man.aux2 > 0)
        {
            first_pos.lon = heli_stat.lon;
            first_pos.lat = heli_stat.lat;
            first_pos.alt = heli_stat.alt;
        }
        if(sp_man.aux3 > 0)
        {
            pos_offset.lon_offset = heli_stat.lon  -  first_pos.lon;
            pos_offset.lat_offset = heli_stat.lat  -  first_pos.lat;
            pos_offset.alt_offset = heli_stat.alt  -  first_pos.alt;
//            mavlink_log_info(&mavlink_log_pub, "got offset lon:%d,lat:%d,alt:%d",(int)(pos_offset.lon_offset*(double)1e7f),(int)(pos_offset.lat_offset*(double)1e7f),(int)(pos_offset.alt_offset*1e7f));
        }
#endif


        map_projection_init(&own_ref,  vehicle_status.lat, vehicle_status.lon);
        map_projection_reproject(&own_ref, vision_receive.vision_x, vision_receive.vision_y,
                                 &heli_stat.lat, &heli_stat.lon);
        /******************* climb up to expected alititude ****************/
        if(follow_result.surely_arrive == true)
        {
            arrive_flag = true;
        }

        if(arrive_flag == true && false)
        {

            //velocity track test
            heli_stat.vel_d = -0.1f;

            if(z < -1.0f * follow_result.param_min_alt)
            {
                heli_stat.alt -= heli_stat.vel_d * DT / 1000000;
//                mavlink_log_info(&mavlink_log_pub,"z:%.3f,param_min:%.3f",(double)z,(double)follow_result.param_min_alt );

            }
            else
            {
                heli_stat.vel_d =0.0f;
                //  mavlink_log_info(&mavlink_log_pub,"param_min:%.3f",(double)follow_target_flag.param_min_alt);
            }
            if(first_time)
            {
                first_time = false;
                firsttime = hrt_absolute_time();
            }
            else
            {
                lasttime = hrt_absolute_time();
                hrt_abstime deta = lasttime - firsttime;
                if(deta > 1e5)
                {
                    first_time = true;
                    z -= heli_stat.vel_d * deta / 1000000;
                }
            }
//            PX4_INFO("z:%.3f,vel_d.3f",(double)z,(double)heli_stat.vel_d );
            // map_projection_init(&target_ref, heli_stat.lat, heli_stat.lon);

            //PX4_INFO("x:%.1f,y:%.1f,lon:%.7f,lat:%.7f", (double)x_sum, (double)y_sum, (double)heli_stat.lon, (double)heli_stat.lat);


        }
        if(follow_result.finally_arrive == true)
        {
//            trackReady = true;
            mavlink_log_info(&mavlink_log_pub,"catched!!! ");
        }
        else
        {
//            trackReady = false;
          //  mavlink_log_info(&mavlink_log_pub,"open!!! ");
        }
        usleep(1000);       //if not sleep in the while(), the thread will be jamed
    }    

    warnx("exiting.\n");
    thread_running = false;
    return 0;
}
