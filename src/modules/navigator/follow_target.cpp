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
//#define VISIONTEST
#define TESTANGLE M_PI//0

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
    ready_to_dock(false),
    _formation_sub(-1),
    _vision_sensor_sub(-1),
    #endif
    _target_updates(0),
    _last_update_time(0),
    _current_target_motion(),
    _previous_target_motion(),
    _yaw_rate(0.0F),
    _responsiveness(0.0F),
    _yaw_auto_max(0.0F),
    _yaw_angle(0.0F)
{
    updateParams();
    _current_target_motion = {};
    _previous_target_motion =  {};
    _current_vel.zero();
    _step_vel.zero();
    _est_target_vel.zero();
    _target_distance.zero();
    _target_position_offset.zero();
    _target_position_delta.zero();
    _heli_yaw.zero();
    _rotated_target_distance.zero();
    _heli_followtarg_pub = nullptr;

    memset(&formation, 0, sizeof(formation));
    memset(&vision_sensor, 0, sizeof(vision_sensor));
#ifdef HOME_POSTION
    memset(&home_position_distance, 0, sizeof(home_position_distance));
    home_position_sub = -1;
#endif

#ifdef RTL_flag
    memset(&rtl_status, 0, sizeof(rtl_status));

    rtl_status_pub_fd = nullptr;
    memset(&manual_control_lastrtl, 0, sizeof(manual_control_lastrtl));
    total_rtl_vehicles = 0;
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

    if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
        _follow_target_position = FOLLOW_FROM_BEHIND;
    }

    _rot_matrix = (_follow_position_matricies[_follow_target_position]);

    if (_follow_target_sub < 0) {
        _follow_target_sub = orb_subscribe(ORB_ID(follow_target));
    }

    if (_targ_heli_sub < 0) {
        _targ_heli_sub = orb_subscribe(ORB_ID(targ_heli));
    }
    rtl_status.status = commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET;


    if (_formation_sub < 0) {
        _formation_sub = orb_subscribe(ORB_ID(ui_strive_formation));
    }
    if (_vision_sensor_sub < 0) {
        _vision_sensor_sub = orb_subscribe(ORB_ID(vision_sensor));
    }

#ifdef HOME_POSTION
    if(home_position_sub < 0)
    {
        home_position_sub = orb_subscribe(ORB_ID(home_position));
        mavlink_log_info(&mavlink_log_pub,"sysID:%d,home position subscrive:%d-------", mavlink_system.sysid, home_position_sub);
    }
#endif
    if(manual_control_lastrtl_sub < 0)
    {
        manual_control_lastrtl_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    }
}

void FollowTarget::on_active()
{
    //    mavlink_log_info(&mavlink_log_pub, "system id:%d", mavlink_system.sysid);
    struct map_projection_reference_s target_ref;
    math::Vector<3> target_reported_velocity(0, 0, 0);
    follow_target_s target_motion_with_offset = {};
    uint64_t current_time = hrt_absolute_time();
    bool _radius_entered = false;
    bool _radius_exited = false;
    bool updated = false;
    float dt_ms = 0;

#ifdef HOME_POSTION
    orb_check(home_position_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(home_position), home_position_sub, &home_position_distance);
    }
#endif

    orb_check(_formation_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(ui_strive_formation), _formation_sub, &formation);
        //        PX4_INFO("foramtion1.lat:%.7f, sysid:%d", formation.lat, formation.sysid);
        if(formation.status == commander_state_s::MAIN_STATE_AUTO_RTL)
        {
            if(formation.sysid > total_rtl_vehicles)
                total_rtl_vehicles = formation.sysid;
        }
    }
    orb_check(_vision_sensor_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(vision_sensor), _vision_sensor_sub, &vision_sensor);
        //        PX4_INFO("foramtion1.lat:%.7f, sysid:%d", formation.lat, formation.sysid);
    }

    //caculate different offset for vehicles in formation  ***zjm
#ifdef VISIONTEST
    if(vision_sensor.status == 3 && total_rtl_vehicles + 1 == mavlink_system.sysid)   //target in vision is available current vehicle should being docking
    {
        //move to docking area(3.5m behind target)
        ready_to_dock = true;
        set_offset((mavlink_system.sysid - total_rtl_vehicles - 1) * HORIZONTAL_OFFSET + 3.5, (mavlink_system.sysid - total_rtl_vehicles - 1) * VERTICAL_OFFSET);
    }
    else
    {
        ready_to_dock = false;
        //move to wait area(8.5m behind target)
        set_offset((mavlink_system.sysid - total_rtl_vehicles) * HORIZONTAL_OFFSET + 3.5, (mavlink_system.sysid - total_rtl_vehicles - 1) * VERTICAL_OFFSET);
    }
#else
    //no vision, so we set dock area 8 meters behind target for safty
    if(total_rtl_vehicles + 1 == mavlink_system.sysid)   //current vehicle should being docking
    {
        //move to docking area(3.5m behind target)
        ready_to_dock = true;
        set_offset((mavlink_system.sysid - total_rtl_vehicles - 1) * HORIZONTAL_OFFSET + 8, (mavlink_system.sysid - total_rtl_vehicles - 1) * VERTICAL_OFFSET);
    }
    else
    {
        ready_to_dock = false;
        //move to wait area(8.5m behind target)
        set_offset((mavlink_system.sysid - total_rtl_vehicles) * HORIZONTAL_OFFSET + 8, (mavlink_system.sysid - total_rtl_vehicles - 1) * VERTICAL_OFFSET);
    }
#endif
    orb_check(manual_control_lastrtl_sub, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(manual_control_setpoint), manual_control_lastrtl_sub, &manual_control_lastrtl);
    }
    //    _follow_offset = _param_tracking_dist.get();
    orb_check(_targ_heli_sub, &updated);
    if (updated) {
        follow_target_s target_motion;
        _target_updates++;

        // save last known motion topic
        _previous_target_motion = _current_target_motion;

        orb_copy(ORB_ID(targ_heli), _targ_heli_sub, &targ_heli);
        target_motion.timestamp = targ_heli.timestamp;
        target_motion.lon = targ_heli.lon;
        target_motion.lat = targ_heli.lat;
        target_motion.alt = targ_heli.alt;
        target_motion.vx = targ_heli.vel_n;
        target_motion.vy = targ_heli.vel_e;
        target_motion.vz = targ_heli.vel_d;

        if (_current_target_motion.timestamp == 0) {
            _current_target_motion = target_motion;
        }
        _current_target_motion.timestamp = target_motion.timestamp;
        _current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
                    1 - _responsiveness);
        _current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
                    1 - _responsiveness);

        _current_target_motion.alt = target_motion.alt;    //YULI  tianjia

        target_reported_velocity(0) = _est_target_vel(0);
        target_reported_velocity(1) = _est_target_vel(1);
    }
    else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
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
    if (target_velocity_valid() && updated) {
        dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);
        // ignore a small dt
        if (dt_ms > 10.0F) {
            //   math::Vector<3> prev_position_delta = _target_position_delta;
            // get last gps known reference for target
            map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

            // calculate distance the target has moved
            map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
                                   &(_target_position_delta(0)), &(_target_position_delta(1)));

            // update the average velocity of the target based on the position
            _est_target_vel = _target_position_delta / (dt_ms / 1000.0f);

            // if the target is moving add an offset and rotation
#ifdef VISIONTEST
            if (_est_target_vel.length() > 3.0F) {  //.5F
                _target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
            }
            else
            {
                _yaw_angle = TESTANGLE;//targ_heli.yaw;
                math::Vector<3> tart_yaw_vector(cos(_yaw_angle),sin(_yaw_angle), 0);//*****ZJMTEST
                _target_position_offset = _rot_matrix * tart_yaw_vector * _follow_offset;//*****ZJMTEST
            }
#else
            //            _yaw_matrix = {cos(_yaw_angle), sin(_yaw_angle), 0,
            //                           -sin(_yaw_angle), cos(_yaw_angle), 0,
            //                           0,                0,               1};
            _yaw_angle = TESTANGLE;//targ_heli.yaw;
            math::Vector<3> tart_yaw_vector(cos(_yaw_angle),sin(_yaw_angle), 0);//*****ZJMTEST
            _target_position_offset = _rot_matrix * tart_yaw_vector * _follow_offset;//*****ZJMTEST
#endif
            // are we within the target acceptance radius?
            // give a buffer to exit/enter the radius to give the velocity controller
            // a chance to catch up

            _radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
            _radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

            //show debug msg every second
            if(firstTime)
            {
                currentTime = hrt_absolute_time();
                firstTime = false;
            }
            else if(hrt_elapsed_time(&currentTime) > 1e6)
            {
                firstTime = true;
                mavlink_log_info(&mavlink_log_pub, "distance to target, x:%.1f, y:%.1f, z:%.1f",(double)_rotated_target_distance(0), (double)_rotated_target_distance(1), (double)_rotated_target_distance(2));
            }
            // to keep the velocity increase/decrease smooth
            // calculate how many velocity increments/decrements
            // it will take to reach the targets velocity
            // with the given amount of steps also add a feed forward input that adjusts the
            // velocity as the position gap increases since
            // just traveling at the exact velocity of the target will not
            // get any closer or farther from the target

            _step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
            _step_vel /= (/*dt_ms / 1000.0F * */(float) INTERPOLATION_PNTS);
            _step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

            // if we are less than 1 meter from the target don't worry about trying to yaw
            // lock the yaw until we are at a distance that makes sense

            if ((_target_distance).length() > 1.0F) {
                // yaw rate smoothing

                // this really needs to control the yaw rate directly in the attitude pid controller
                // but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode
#ifdef VISIONTEST
                if(_est_target_vel.length() > 5)    //target is moving, so we set follow position by target angle
//                _yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat, //after test, please incomment    *****ZJM
//                                                          _navigator->get_global_position()->lon,
//                                                          _current_target_motion.lat,
//                                                          _current_target_motion.lon);
                    _yaw_angle = TESTANGLE;
                else    //target is not moving, we follow in the south of it
                {
                    _yaw_angle = TESTANGLE;
                }
#endif
                _yaw_rate = (_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0F);

                _yaw_rate = _wrap_pi(_yaw_rate);

                _yaw_rate = math::constrain(_yaw_rate, -1.0F * _yaw_auto_max, _yaw_auto_max);

            } else {
//                _yaw_angle = _yaw_rate = NAN;
                _yaw_angle = TESTANGLE;//targ_heli.yaw;
#ifndef VISIONTEST
                _yaw_angle = TESTANGLE;//targ_heli.yaw;
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
    if (target_position_valid()) {

        // get the target position using the calculated offset

        map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
        map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
                                 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
        target_motion_with_offset.alt =  _current_target_motion.alt;   //"set_hgt_offset" will be added in the following set position function, so don't add  here zjm
    }

    // clamp yaw rate smoothing if we are with in
    // 3 degrees of facing target
    if (PX4_ISFINITE(_yaw_rate)) {
        if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
            _yaw_rate = NAN;
        }
    }
#ifdef SET_OFFSET   //if vehicle is in the docking area
    _yaw_matrix.data[0][0] = cos(_yaw_angle);
    _yaw_matrix.data[0][1] = sin(_yaw_angle);
    _yaw_matrix.data[0][2] = 0;
    _yaw_matrix.data[1][0] = -sin(_yaw_angle);
    _yaw_matrix.data[1][1] = cos(_yaw_angle);
    _yaw_matrix.data[1][2] = 0;
    _yaw_matrix.data[2][0] = 0;
    _yaw_matrix.data[2][1] = 0;
    _yaw_matrix.data[2][2] = 1;

    _rotated_target_distance = _yaw_matrix * (_target_distance + _target_position_offset);
    _rotated_target_distance(2) = target_motion_with_offset.alt + set_hgt_offset - _navigator->get_global_position()->alt;
    //    PX4_INFO("sysID:%d,rtl_vehicles:%d",mavlink_system.sysid, total_rtl_vehicles);
    //is the current vehicle in the dock/refule cylinder
    docking_last_time = mavlink_system.sysid == 1 ? 60e6 : 30e6;   //first vehicle has 1 minutes, and others have 30 seconds
    if(total_rtl_vehicles + 1 == mavlink_system.sysid)  //*********************** current vehicle which is ready to dock(got vision status 3)should be docking, normally +1
    {
        if(!start_docking)
        {
            start_docking_time = hrt_absolute_time();
            start_docking = true;
        }
        if(hrt_absolute_time() - start_docking_time > docking_last_time && rtl_status.status == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET)    //running out of time, return to land
        {
            rtl_status.status = commander_state_s::MAIN_STATE_AUTO_RTL;//5
            mavlink_log_info(&mavlink_log_pub,"sysID:%d timeout for 1 or 2 minutes and return to land!!!!", mavlink_system.sysid);
            if (rtl_status_pub_fd != nullptr) {     //publish rtl status
                orb_publish(ORB_ID(follow_to_commander), rtl_status_pub_fd, &rtl_status);

            } else {
                rtl_status_pub_fd = orb_advertise(ORB_ID(follow_to_commander), &rtl_status);
            }
        }
        //        PX4_INFO("pos, lon:%.7f, lat:%.7f, alt:%.3f",(double)_navigator->get_global_position()->lon, (double)_navigator->get_global_position()->lat, (double)_navigator->get_global_position()->alt);
        //        PX4_INFO("target, lon:%.7f, lat:%.7f, alt:%.3f",(double)target_motion_with_offset.lon, (double)target_motion_with_offset.lat, (double)target_motion_with_offset.alt + (double)set_hgt_offset);
        //        PX4_INFO("distance to target, x:%.1f, y:%.1f, z:%.1f",(double)_rotated_target_distance(0), (double)_rotated_target_distance(1), (double)_rotated_target_distance(2));
        if(ready_to_dock && _rotated_target_distance(0) > -1.5f && _rotated_target_distance(0) < 1.5f && sqrt(_rotated_target_distance(1) * _rotated_target_distance(1) + _rotated_target_distance(2) * _rotated_target_distance(2)) < 2)
        {
            if(!midair_refueling)   // start to dock/refule
            {
                midair_refueling = true;
                refueling_time = hrt_absolute_time();
                mavlink_log_info(&mavlink_log_pub,"refuling time start counting");
            }
            if((hrt_absolute_time() - refueling_time) > 10e6 && rtl_status.status == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET)    //对接10秒钟返航
            {
                rtl_status.status = commander_state_s::MAIN_STATE_AUTO_RTL;//5
                mavlink_log_info(&mavlink_log_pub,"sysID:%d refule for 10s and return to land!!!!", mavlink_system.sysid);
                if (rtl_status_pub_fd != nullptr) {     //publish rtl status
                    orb_publish(ORB_ID(follow_to_commander), rtl_status_pub_fd, &rtl_status);

                } else {
                    rtl_status_pub_fd = orb_advertise(ORB_ID(follow_to_commander), &rtl_status);
                }
            }
            if(rtl_status.status == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET && hrt_elapsed_time(&currentTime) > 1e6)
                mavlink_log_info(&mavlink_log_pub, "sysID:%d in refule position time lasting:%d", mavlink_system.sysid, (uint32_t)((hrt_absolute_time() - refueling_time)/1e6));
        }
        else
        {
            midair_refueling = false;
            //            if(hrt_elapsed_time(&currentTime) > 1e6)
            //                mavlink_log_info(&mavlink_log_pub, "sysID:%d out of refule position");
            //            PX4_INFO("sysID:%d refule abord and retrying out of range", mavlink_system.sysid,mavlink_system.sysid);
        }
    }
#endif

    // update state machine
    if (target_velocity_valid()) {  //only if the target has been updated, will we start to follow target  ***zjm
        _yaw_angle = TESTANGLE;//targ_heli.yaw;     //after test you must delete this   *****ZJM
        //        mavlink_log_info(&mavlink_log_pub,"_follow_target_state:%d,yaw:%.1f",_follow_target_state,(double)_yaw_angle);
        switch (_follow_target_state) {

        case TRACK_POSITION: {
            //        mavlink_log_info(&mavlink_log_pub,"TRACK_POSITION");

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
            // for now set the target at the minimum height above the uav
            target.lat = _navigator->get_global_position()->lat;
            target.lon = _navigator->get_global_position()->lon;
            target.alt = _navigator->get_global_position()->alt;
            set_follow_target_item(&_mission_item, 0, target, _yaw_angle);
            mavlink_log_info(&mavlink_log_pub,"set targ iyaw:%.1f,tyaw:%.1f",(double)_mission_item.yaw, (double)targ_heli.yaw);
            PX4_INFO("set _mission_item lat:%.7f lon:%.1f alt:%.2f",_mission_item.lat,_mission_item.lon, (double)_mission_item.altitude);
            //        set_follow_target_item(&_mission_item, _param_min_alt.get(), target, _yaw_angle);
            update_position_sp(false, false, _yaw_rate);

            _follow_target_state = WAIT_FOR_TARGET_POSITION;
        }

        case WAIT_FOR_TARGET_POSITION: {
            if (is_mission_item_reached() && target_velocity_valid()) {
                _target_position_offset(0) = _follow_offset;   ///????
                _follow_target_state = TRACK_POSITION;
            }
            break;
        }
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
