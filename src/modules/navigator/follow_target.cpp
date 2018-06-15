/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#define AUTOTEST
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/mavlink_log.h>
#ifdef FOLLOWTARGET
#include <uORB/topics/targ_heli.h>
#endif
#include <lib/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>
#include "navigator.h"
#define SET_OFFSET
FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
    _navigator(navigator),
    _param_min_alt(this, "NAV_MIN_FT_HT", false),
    _param_tracking_dist(this, "NAV_FT_DST", false),
    _param_tracking_side(this, "NAV_FT_FS", false),
    _param_tracking_resp(this, "NAV_FT_RS", false),
    _param_yaw_auto_max(this, "MC_YAWRAUTO_MAX", false),
    _follow_target_state(SET_WAIT_FOR_TARGET_POSITION),
    _follow_target_position(FOLLOW_FROM_BEHIND),
    _follow_target_sub(-1),
    #ifdef FOLLOWTARGET
    _targ_heli_sub(-1),
    mavlink_log_pub(nullptr),
    _heli_followtarg_pub(nullptr),
    firstTime(true),
    currentTime(0),
    #endif
    _step_time_in_ms(0.0f),
    _follow_offset(OFFSET_M),
    #ifdef AUTOTEST
    arrive_time(0),
    first_arrive(true),
    vel_d_flag(false),
    second_arrive_time(0),
    second_arrive(true),
    target_altitude(0.0f),

    #endif

    #ifdef UI_STRIVE
        _formation_sub(-1),
    #endif
    _target_updates(0),
    _last_update_time(0),
    _current_target_motion(),
    _previous_target_motion(),
    _heli_follow_result(),
    _yaw_rate(0.0F),
    _responsiveness(0.0F),
    _yaw_auto_max(0.0F),
    _yaw_angle(0.0F)
{
    updateParams();
    _current_target_motion = {};
    _previous_target_motion =  {};
    _heli_follow_result =  {};
    _current_vel.zero();
    _step_vel.zero();
    _est_target_vel.zero();
    _target_distance.zero();
    _target_position_offset.zero();
    _target_position_delta.zero();
#ifdef FOLLOWTARGET
    _heli_yaw.zero();
    _heli_followtarg_pub = nullptr;
    // _heli_followtarg_pub = orb_advertise(ORB_ID(heli_followtarg), &_heli_follow_result);
#endif
#ifdef UI_STRIVE
    memset(&formation, 0, sizeof(formation));
#endif
#ifdef HOME_POSTION
    memset(&home_position_distance, 0, sizeof(home_position_distance));
    home_position_sub = -1;
#endif

#ifdef RTL_flag
    memset(&rtl_status, 0, sizeof(rtl_status));
    rtl_status_pub_fd = nullptr;
    memset(&manual_control_lastrtl, 0, sizeof(manual_control_lastrtl));
#endif

}



FollowTarget::~FollowTarget()
{
}

void FollowTarget::on_inactive()
{
    reset_target_validity();
}

void FollowTarget::on_activation()
{
    updateParams();
   // _follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();
    _follow_offset = _param_tracking_dist.get();                                 //yuli add this
    _responsiveness = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

    _yaw_auto_max = math::radians(_param_yaw_auto_max.get());

    _follow_target_position = _param_tracking_side.get();
    set_offset(3.0f,1.5f);//according the mc's ID

    if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
        _follow_target_position = FOLLOW_FROM_BEHIND;
    }

    _rot_matrix = (_follow_position_matricies[_follow_target_position]);

    if (_follow_target_sub < 0) {
        _follow_target_sub = orb_subscribe(ORB_ID(follow_target));
    }
#ifdef FOLLOWTARGET
    if (_targ_heli_sub < 0) {
        _targ_heli_sub = orb_subscribe(ORB_ID(targ_heli));
    }

#endif
#ifdef UI_STRIVE
    if (_formation_sub < 0) {
        _formation_sub = orb_subscribe(ORB_ID(ui_strive_formation));
    }
#endif

#ifdef HOME_POSTION
    if(home_position_sub < 0)
    {
        home_position_sub = orb_subscribe(ORB_ID(home_position));
    }
#endif
    if(manual_control_lastrtl_sub < 0)
    {
        manual_control_lastrtl_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    }
}

void FollowTarget::on_active()
{
//    mavlink_log_info(&mavlink_log_pub, "follow target on active1");
//    usleep(10000);
 //   mavlink_log_info(&mavlink_log_pub, "system id:%d", mavlink_system.sysid);

    struct map_projection_reference_s target_ref;
    math::Vector<3> target_reported_velocity(0, 0, 0);
    follow_target_s target_motion_with_offset = {};
    uint64_t current_time = hrt_absolute_time();
    bool _radius_entered = false;
    bool _radius_exited = false;
    bool updated = false;
    float dt_ms = 0;
    bool climb_finished = false;
    // orb_check(_follow_target_sub, &updated);

#ifdef HOME_POSTION
    orb_check(home_position_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(home_position), home_position_sub, &home_position_distance);
       // PX4_INFO("foramtion1.lat:%.7f", formation.lat);
    }
#endif

#ifdef UI_STRIVE
    orb_check(_formation_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(ui_strive_formation), _formation_sub, &formation);
        PX4_INFO("foramtion1.lat:%.7f, sysid:%d", formation.lat, formation.sysid);
    }
    if(mavlink_system.sysid == formation.sysid + 1)

    {
        last_rtl = formation.status == 12;
    }
//    if(MC_ID == 2)
//    {
//        last_rtl = formation.status == 12;
//    }
//    if(MC_ID == 3)
//    {
//        last_rtl = formation.status == 12;
//    }
//    if(MC_ID == 4)
//    {
//        last_rtl = formation.status == 12;
 //   }
#endif

     orb_check(manual_control_lastrtl_sub, &updated);
     if(updated)
     {
         orb_copy(ORB_ID(manual_control_setpoint), manual_control_lastrtl_sub, &manual_control_lastrtl);
     }

#ifdef FOLLOWTARGET
//    _follow_offset = _param_tracking_dist.get();
    orb_check(_targ_heli_sub, &updated);
#endif
#ifdef SET_OFFSET
   // set_hgt_offset = _param_min_alt.get();
#endif
#ifdef HOME_POSTION
//    if(home_position_sub < 0)
//    {
//        home_position_sub = orb_subscribe(ORB_ID(home_position));
//    }
//    orb_check(home_position_sub, &updated);
     PX4_INFO("updated :%d", updated);
//    if(updated)
//    {
//        orb_copy(ORB_ID(home_position), home_position_sub, &home_position_distance);
//       // PX4_INFO("foramtion1.lat:%.7f", formation1.lat);
//    }
#endif

    if (updated) {
        PX4_INFO("home_position_sub, &updated");
                follow_target_s target_motion;
                _target_updates++;

                // save last known motion topic
                _previous_target_motion = _current_target_motion;
        #ifdef FOLLOWTARGET
                orb_copy(ORB_ID(targ_heli), _targ_heli_sub, &targ_heli);
 //               orb_copy(ORB_ID(home_position), home_position_sub, &home_position_distance);
                target_motion.timestamp = targ_heli.timestamp;
                target_motion.lon = targ_heli.lon;
                target_motion.lat = targ_heli.lat;
                target_motion.alt = targ_heli.alt;
                target_motion.vx = targ_heli.vel_n;
                target_motion.vy = targ_heli.vel_e;
                target_motion.vz = targ_heli.vel_d;
//////*********************************
//                target_motion.timestamp = home_position_distance.timestamp;
//                target_motion.lon = home_position_distance.lon;
//                target_motion.lat = home_position_distance.lat;
//                target_motion.alt = home_position_distance.alt;

//////*************************************

                climb_finished = targ_heli.climb_finished;
                _yaw_angle = targ_heli.yaw;
   //             _yaw_angle = home_position_distance.yaw;
                _heli_follow_result.surely_arrive = false;
                _heli_follow_result.finally_arrive = false;
#ifdef SET_OFFSET
                _heli_follow_result.param_min_alt = set_hgt_offset;
#else
                _heli_follow_result.param_min_alt =  _param_min_alt.get();
#endif
                _heli_follow_result.global_alt = _navigator->get_global_position()->alt;
                _heli_follow_result.timestamp = hrt_absolute_time();
   //             mavlink_log_info(&mavlink_log_pub,"_heli_follow_result:%llu",_heli_follow_result.timestamp);
                 target_altitude =  target_motion.alt;
   //               mavlink_log_info(&mavlink_log_pub," target_altitude:%.3f", (double)target_altitude);//yuli20180411
   //             PX4_INFO("targ_heli.lon:%.7f, targ_heli.lat:%.7f, targ_heli.alt:%.2f, targ_heli.yaw:%.1f", targ_heli.lon, targ_heli.lat, (double)targ_heli.alt, (double)targ_heli.yaw);
   //           PX4_INFO("target_motion.surely_arrive:%d",_heli_follow_result.surely_arrive);
        #ifdef AUTOTEST
               // follow_targ = orb_advertise(ORB_ID(follow_target), &target_motion);

                        if ((_target_distance).length() < 0.1f )
                        {
                            if(first_arrive == true)
                            {
                                 arrive_time = hrt_absolute_time();
                                 first_arrive = false;
                            }

                            if(hrt_elapsed_time(&arrive_time) > 10)
                            {
                                // _param_min_alt.set(0.0f);
                                 _heli_follow_result.surely_arrive = true;
                             //    mavlink_log_info(&mavlink_log_pub, "target_motion.surely_arrive = true");
                            }
                        }
                        else
                        {
                            first_arrive = true;
                            _heli_follow_result.surely_arrive = false;
                        }

        //                if(target_motion.vz < -0.01f)
        //                {
        //                    vel_d_flag = true;
        //                }
#ifdef SET_OFFSET
                        if ( climb_finished == true && fabs(_current_target_motion.alt + set_hgt_offset - _navigator->get_global_position()->alt) < 0.15  && (_target_distance).length() < 0.1f)
#else
                        if ( climb_finished == true && fabs(_current_target_motion.alt + _param_min_alt.get() - _navigator->get_global_position()->alt) < 0.15  && (_target_distance).length() < 0.1f)
#endif
                        {
                            if(second_arrive == true)
                            {
                                 second_arrive_time = hrt_absolute_time();
                                 second_arrive = false;
                            }

                            if(hrt_elapsed_time(&second_arrive_time) > 10)
                            {
                                 _heli_follow_result.finally_arrive = true;
                                 mavlink_log_info(&mavlink_log_pub, "target_motion.finally_arrive = true");
                            }
                        }
                        else
                        {
                            second_arrive = true;
                            _heli_follow_result.finally_arrive = false;
                        }
                    //    mavlink_log_info(&mavlink_log_pub, "_param_min:%.3f,target_motion:%.3f",(double)_param_min_alt.get(),(double)target_motion.param_min_alt);
                        if ( _heli_followtarg_pub != nullptr)
                        {
                            orb_publish(ORB_ID(heli_followtarg),  _heli_followtarg_pub, &_heli_follow_result);

                        } else
                        {
                           _heli_followtarg_pub = orb_advertise(ORB_ID(heli_followtarg), &_heli_follow_result);
                        }

                       // orb_publish(ORB_ID(heli_followtarg),  _heli_followtarg_pub, &_heli_follow_result);


        #endif
        //#else
         //       orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);
        #endif
                if (_current_target_motion.timestamp == 0) {
                    _current_target_motion = target_motion;
                }
                _current_target_motion.timestamp = target_motion.timestamp;
                _current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
                            1 - _responsiveness);
                _current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
                            1 - _responsiveness);

                _current_target_motion.alt = target_motion.alt;    //YULI  tianjia
//××××××××××××××××××××××××××××××××××××××//
                //x和y方向的速度，如果视觉可用，则速度信息为heli_msg线程接收，如果视觉不可用，则来源与本线程经纬度计算
                if(vision_enabled == true)
                {
                    target_reported_velocity(0) = _current_target_motion.vx;
                    target_reported_velocity(1) = _current_target_motion.vy;
                }
                else
                {
                    target_reported_velocity(0) = _est_target_vel(0);
                    target_reported_velocity(1) = _est_target_vel(1);
                }
                target_reported_velocity(0) = _est_target_vel(0);
                target_reported_velocity(1) = _est_target_vel(1);
//×××××××××××××××××××××××××××××××××××××××××//

    }
    else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
      //  mavlink_log_info(&mavlink_log_pub,"TARGET_TIMEOUT_MS!!!!!!!!!!!");
        reset_target_validity();

    }

    // update distance to target

    if (target_position_valid()) {

        // get distance to target

        map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
        map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
                               &_target_distance(1));

    }

    // update target velocity
 // mavlink_log_info(&mavlink_log_pub, "follow target on active2");
    if (target_velocity_valid() && updated) {
        dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);
        PX4_INFO("valid & updated");
        // ignore a small dt
        if (dt_ms > 10.0F) {
  //          mavlink_log_info(&mavlink_log_pub, "dt_ms > 10.0F");
#ifdef SET_OFFSET


            switch (follow_state)
            {
            case 0:
            {
                mavlink_log_info(&mavlink_log_pub, "case 0");
                if((_target_position_offset + _target_distance).length() < 1.0f && fabs(_current_target_motion.alt + set_hgt_offset - _navigator->get_global_position()->alt) < 1.0
                        && (last_rtl || mavlink_system.sysid == 1||manual_control_lastrtl.aux2 > 0.5f)
                        &&  sqrt(home_position_distance.x * home_position_distance.x + home_position_distance.y * home_position_distance.y) < 1000.0)
                {
                   // set_offset(10.0f,0.0f);
                    set_offset(5.0f,3.0f);
                    follow_state = 1;//从准备区到视觉区
                }

                break;
            }

            case 1:
            {
                mavlink_log_info(&mavlink_log_pub, "case 1");
                if( (_target_position_offset + _target_distance).length() < 1.0f && fabs(_current_target_motion.alt + set_hgt_offset - _navigator->get_global_position()->alt) < 1.0  )
                {
                     set_offset(10.0f,3.0f);
                     follow_state = 2;//向mark点接近
                    if(vision_time_start == false)
                    {
                        vision_time_start = true;
                        vision_time = hrt_absolute_time();
                    }

                 }
                 if((hrt_absolute_time() - vision_time) > 60e6 && vision_time_start == true)    //1min内没有对接成功，则认为此飞机不能完成对接任务
                 {
                     rtl_status.status = 5;
                     mavlink_log_info(&mavlink_log_pub, "rtl_status.status = 5");
                 }



                if(sqrt(home_position_distance.x * home_position_distance.x + home_position_distance.y * home_position_distance.y) > 1000.0)
                {
                    follow_state = 10;
                    vision_time_start = false;
                }
                break;

            }
            case 2:
            {
                mavlink_log_info(&mavlink_log_pub, "case 2");
                if((_target_position_offset + _target_distance).length() < 1.0f && fabs(_current_target_motion.alt + set_hgt_offset - _navigator->get_global_position()->alt) < 1.0)
                {
                    rtl_status.status = 5;   //超过极限距离，返航
                }

                if((hrt_absolute_time() - vision_time) > 60e6 &&  vision_time_start == true)    //1min内没有对接成功，则认为此飞机不能完成对接任务
                {
                    rtl_status.status = 5;
                    mavlink_log_info(&mavlink_log_pub, "rtl_status.status = 5");
                }

                if(sqrt(home_position_distance.x * home_position_distance.x + home_position_distance.y * home_position_distance.y) > 1000.0)
                {
                    follow_state = 10;
                    vision_time_start = false;
                }

                if(vision_enabled)
                {
                     set_offset(10.0f,5.0f);
                     follow_state = 3;    // 视觉定位

                }
                break;

            }
            case 3:
            {
                mavlink_log_info(&mavlink_log_pub, "case 3");
                if((_target_position_offset + _target_distance).length() < 1.0f && fabs(_current_target_motion.alt + set_hgt_offset - _navigator->get_global_position()->alt) < 1.0 && midair_refueling ==false)
                {
                    midair_refueling = true;
                    refueling_time = hrt_absolute_time();
                }
                if((hrt_absolute_time() - refueling_time) > 10e6)    //对接10秒钟返航
                {
                    rtl_status.status = 5;
                }

                if(vision_enabled == false)
                {
                    follow_state = 2;
                    midair_refueling = false;
                }
                break;
            }
            case 10:
            {
                mavlink_log_info(&mavlink_log_pub, "case 10");
  //              set_offset(20.0f,-3.0f);//according the mc's ID
                set_offset(3.0f,3.0f);//according the mc's ID
                follow_state = 0;
                midair_refueling = false;
                vision_time_start = false;
                break;

            }

            default:
                break;
            }


            if (rtl_status_pub_fd != nullptr) {
                orb_publish(ORB_ID(follow_to_commander), rtl_status_pub_fd, &rtl_status);

            } else {
                rtl_status_pub_fd = orb_advertise(ORB_ID(follow_to_commander), &rtl_status);
            }

#endif


            //   math::Vector<3> prev_position_delta = _target_position_delta;

            // get last gps known reference for target

            map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

            // calculate distance the target has moved

            map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
                                   &(_target_position_delta(0)), &(_target_position_delta(1)));

            // update the average velocity of the target based on the position

            _est_target_vel = _target_position_delta / (dt_ms / 1000.0f);

            // if the target is moving add an offset and rotation


            if (_est_target_vel.length() > .5F) {
                _target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
            }
#ifdef FOLLOWTARGET
            else
            {
            //    _follow_target_position = _param_tracking_side.get();

                if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
                    _follow_target_position = FOLLOW_FROM_BEHIND;
                }

                _rot_matrix = (_follow_position_matricies[_follow_target_position]);

                _heli_yaw(0) = cos((double)targ_heli.yaw);
                _heli_yaw(1) = sin((double)targ_heli.yaw);
                _heli_yaw(2) = 0;
                _target_position_offset =  _rot_matrix * _heli_yaw * _follow_offset;
            }

#endif
            // are we within the target acceptance radius?
            // give a buffer to exit/enter the radius to give the velocity controller
            // a chance to catch up

            _radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
            _radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);
#ifdef FOLLOWTARGET

            if(firstTime)
            {
                currentTime = hrt_absolute_time();
                firstTime = false;
//              PX4_INFO("currentTime, firstTime:%d", firstTime);
                mavlink_log_info(&mavlink_log_pub, "currentTime, firstTime:%d", firstTime);
            }
            else if(hrt_elapsed_time(&currentTime) > 1e6)
            {
                firstTime = true;

                float x = 0.0,  y = 0.0, z= 0.0;
                x = (_target_position_offset + _target_distance)(0);
                y = (_target_position_offset + _target_distance)(1);
#ifdef SET_OFFSET
                z = _current_target_motion.alt - _navigator->get_global_position()->alt + set_hgt_offset;
#else
                z = _current_target_motion.alt - _navigator->get_global_position()->alt + _param_min_alt.get();
#endif

                mavlink_log_info(&mavlink_log_pub, " get_global_position()->alt:%.3f",
                                 (double) _navigator->get_global_position()->alt);

                float _deta_yaw_angle = targ_heli.yaw - _navigator->get_global_position()->yaw;
                mavlink_log_info(&mavlink_log_pub, "x:%.1f y:%.1f z:%.1f len:%.3f yaw:%.1f sta:%d",
                                 (double)x,(double)y, (double)z, (double)sqrt(x*x + y*y), (double)(_deta_yaw_angle * 180 / 3.14f), _follow_target_state);
 //               mavlink_log_info(&mavlink_log_pub, "cur(0):%.1f step(0):%.1f cur(1):%.1f, step(1):%.1f:",(double)_current_vel(0),(double)_step_vel(0), (double)_current_vel(1),(double)_step_vel(1));

 //               PX4_INFO("x:%.1f y:%.1f z:%.1f length:%.1f yaw:%.1f state:%d",
 //                        (double)x,(double)y, (double)z, (double)sqrt(x*x + y*y), (double)(_deta_yaw_angle * 180 / 3.14f), _follow_target_state);
            }

#endif
            // to keep the velocity increase/decrease smooth
            // calculate how many velocity increments/decrements
            // it will take to reach the targets velocity
            // with the given amount of steps also add a feed forward input that adjusts the
            // velocity as the position gap increases since
            // just traveling at the exact velocity of the target will not
            // get any closer or farther from the target

            _step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
//            PX4_INFO("_current_vel(0):%.1f, _current_vel(1):%.1f,_target_distance(0):%.1f, _target_distance(1):%.1f:",(double) _current_vel(0),(double) _current_vel(1), (double)_target_distance(0),(double)_target_distance(1));
//            PX4_INFO("current_vel(0):%.1f, _step_vel(0):%.1f:_current_vel(1):%.1f, _step_vel(1):%.1f:",(double) _current_vel(0),(double) _step_vel(0), (double)_current_vel(1),(double) _step_vel(1));

            _step_vel /= (/*dt_ms / 1000.0F * */(float) INTERPOLATION_PNTS);
            _step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);
#ifdef FOLLOWTARGET0
            if ((_target_distance).length() > 1.0F) {
#else
            // if we are less than 1 meter from the target don't worry about trying to yaw
            // lock the yaw until we are at a distance that makes sense

            if ((_target_distance).length() > 1.0F) {
#endif

#ifdef FOLLOWTARGET
                //keep yaw angle is the same with the target yaw       *****jim 2018.1.10
                _yaw_angle = targ_heli.yaw;
#else
                // yaw rate smoothing

                // this really needs to control the yaw rate directly in the attitude pid controller
                // but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

                _yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
                                                          _navigator->get_global_position()->lon,
                                                          _current_target_motion.lat,
                                                          _current_target_motion.lon);
#endif
                _yaw_rate = (_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0F);

                _yaw_rate = _wrap_pi(_yaw_rate);

                _yaw_rate = math::constrain(_yaw_rate, -1.0F * _yaw_auto_max, _yaw_auto_max);

            } else {
                _yaw_angle = _yaw_rate = NAN;
#ifdef FOLLOWTARGET
                _yaw_angle = targ_heli.yaw;
                // _yaw_angle = home_position_distance.yaw;
#endif
            }
        }

        //		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d con ratio = %3.6f yaw rate = %3.6f",
        //				(double) _step_vel(0),
        //				(double) _step_vel(1),
        //				(double) _current_vel(0),
        //				(double) _current_vel(1),
        //				(double) _est_target_vel(0),
        //				(double) _est_target_vel(1),
        //				(double) (_target_distance).length(),
        //				(double) (_target_position_offset + _target_distance).length(),
        //				_follow_target_state,
        //				(double)_avg_cos_ratio, (double) _yaw_rate);
    }
//mavlink_log_info(&mavlink_log_pub, "follow target on active3");
    if (target_position_valid()) {

        // get the target position using the calculated offset

        map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
        map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
                                 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
        target_motion_with_offset.alt =  _current_target_motion.alt;
    }

    // clamp yaw rate smoothing if we are with in
    // 3 degrees of facing target

    if (PX4_ISFINITE(_yaw_rate)) {
        if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
            _yaw_rate = NAN;
        }
    }

    // update state machine
#ifdef FOLLOWTARGET

//    PX4_INFO("state:%d, yaw:%.1f, dist:%.1f", _follow_target_state,(double)(_yaw_angle),(double)(_target_position_offset + _target_distance).length());
#endif
//    PX4_INFO("_param_min_alt.get():%.3f", (double)_param_min_alt.get());
    switch (_follow_target_state) {

    case TRACK_POSITION: {
   //     mavlink_log_info(&mavlink_log_pub,"TRACK_POSITION");

        if (_radius_entered == true) {
            _follow_target_state = TRACK_VELOCITY;

        } else if (target_velocity_valid()) {
#ifdef SET_OFFSET
            set_follow_target_item(&_mission_item, set_hgt_offset, target_motion_with_offset, _yaw_angle);
#else
            set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);
#endif
            // keep the current velocity updated with the target velocity for when it's needed
            _current_vel = _est_target_vel;

            update_position_sp(true, true, _yaw_rate);

        } else {
            _follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
        }

        break;
    }

    case TRACK_VELOCITY: {
    //    mavlink_log_info(&mavlink_log_pub,"TRACK_VELOCITY");

        if (_radius_exited == true) {
            _follow_target_state = TRACK_POSITION;

        } else if (target_velocity_valid()) {

            if ((current_time - _last_update_time) / 1000 >= _step_time_in_ms) {
                _current_vel+= _step_vel;
//                PX4_INFO("velocity:current_vel(0):%.1f, _step_vel(0):%.1f:_current_vel(1):%.1f, _step_vel(1):%.1f:",(double)_current_vel(0),(double)_step_vel(0), (double)_current_vel(1),(double)_step_vel(1));
                _last_update_time = current_time;
            }
#ifdef SET_OFFSET
            set_follow_target_item(&_mission_item, set_hgt_offset, target_motion_with_offset, _yaw_angle);
#else
            set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);
#endif
            update_position_sp(true, false, _yaw_rate);

        } else {
            _follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
        }//

        break;
    }

    case SET_WAIT_FOR_TARGET_POSITION: {

        // Climb to the minimum altitude
        // and wait until a position is received

        follow_target_s target = {};
    //     mavlink_log_info(&mavlink_log_pub,"SET_WAIT_FOR_TARGET_POSITION");
        // for now set the target at the minimum height above the uav


 //       PX4_INFO("SET_WAIT_global_position_lat:%.7f lon:%.1f alt:%.2f",(double)target.lat,(double)target.lon, (double)target.alt);
        //target.alt = 0.0F;
        if(updated)
        {
            target.lat = _navigator->get_global_position()->lat;
            target.lon = _navigator->get_global_position()->lon;
            //target.alt = _navigator->get_global_position()->alt;
            //target.alt = _
            target.alt = target_altitude;
    #ifdef SET_OFFSET
            set_follow_target_item(&_mission_item, set_hgt_offset, target, _yaw_angle);
    #else
            set_follow_target_item(&_mission_item, _param_min_alt.get(), target, _yaw_angle);
    #endif
 //       PX4_INFO("SET_WAIT__mission_item_lat:%.7f lon:%.1f alt:%.2f",(double)_mission_item.lat,(double)_mission_item.lon, (double)_mission_item.altitude);

//PX4_INFO("SET_yaw_angle:%.1f",(double)_yaw_angle);
        update_position_sp(false, false, _yaw_rate);

        _follow_target_state = WAIT_FOR_TARGET_POSITION;
        }
    }

    case WAIT_FOR_TARGET_POSITION: {
#ifdef FOLLOWTARGET
  //      mavlink_log_info(&mavlink_log_pub,"WAIT_FOR_TARGET_POSITION");
        _mission_item.yaw = targ_heli.yaw;
    //    _mission_item.yaw = home_position_distance.yaw;
        bool item_reached = is_mission_item_reached();
//        PX4_INFO("item_reached:%d, item_yaw:%.1f", item_reached,(double)_mission_item.yaw);
#endif
        if (item_reached/*is_mission_item_reached()*/ && target_velocity_valid()) {
            _target_position_offset(0) = _follow_offset;   ///????
            _follow_target_state = TRACK_POSITION;
        }

        break;
    }
    }
}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
    // convert mission item to current setpoint

    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    // activate line following in pos control if position is valid

    pos_sp_triplet->previous.valid = use_position;
    pos_sp_triplet->previous = pos_sp_triplet->current;
    mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
    pos_sp_triplet->current.position_valid = use_position;
    pos_sp_triplet->current.velocity_valid = use_velocity;
    pos_sp_triplet->current.vx = _current_vel(0);
    pos_sp_triplet->current.vy = _current_vel(1);
    pos_sp_triplet->next.valid = false;
    pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
    pos_sp_triplet->current.yawspeed = yaw_rate;
    _navigator->set_position_setpoint_triplet_updated();
}

void FollowTarget::reset_target_validity()
{
    _yaw_rate = NAN;
    _previous_target_motion = {};
    _current_target_motion = {};
    _target_updates = 0;
    _current_vel.zero();
    _step_vel.zero();
    _est_target_vel.zero();
    _target_distance.zero();
    _target_position_offset.zero();
    reset_mission_item_reached();
    _follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}

bool FollowTarget::target_velocity_valid()
{
    // need at least 2 continuous data points for velocity estimate
    return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
    // need at least 1 continuous data points for position estimate
    return (_target_updates >= 1);
}
#ifdef SET_OFFSET
void FollowTarget::set_offset(float x,float y)
{
    _follow_offset = x;
    set_hgt_offset = y;
}
#endif
