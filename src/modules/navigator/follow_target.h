/***************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once
#define FOLLOWTARGET
#define AUTOTEST
#define SET_OFFSET
#define UI_STRIVE
#define HOME_POSTION
#define RTL_flag
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <lib/mathlib/math/Vector.hpp>
#include <lib/mathlib/math/Matrix.hpp>
#include <systemlib/mavlink_log.h>
#include "navigator_mode.h"
#include "mission_block.h"
#ifdef UI_STRIVE
#include <uORB/topics/ui_strive_formation.h>
#include <uORB/topics/ui_strive_gcs_to_formation.h>
#include <v2.0/mavlink_types.h>

#endif
#ifdef HOME_POSTION
#include <uORB/topics/home_position.h>
#endif
#include <uORB/topics/vision_sensor.h>
#ifdef RTL_flag
#include <uORB/topics/follow_to_commander.h>
#endif
#ifdef UI_STRIVE
#define MC_ID 4      //本机的ID号（1、2、3、4）
extern mavlink_system_t mavlink_system;//it contains system id, you must'nt change this struct data anytime anywhere    ***zjm
#endif
class FollowTarget : public MissionBlock
{

public:
	FollowTarget(Navigator *navigator, const char *name);

	FollowTarget(const FollowTarget &) = delete;
	FollowTarget &operator=(const FollowTarget &) = delete;

	~FollowTarget();

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:

	static constexpr int TARGET_TIMEOUT_MS = 2500;
	static constexpr int TARGET_ACCEPTANCE_RADIUS_M = 5;
	static constexpr int INTERPOLATION_PNTS = 20;
	static constexpr float FF_K = .25F;
	static constexpr float OFFSET_M = 8;

	enum FollowTargetState {
		TRACK_POSITION,
		TRACK_VELOCITY,
		SET_WAIT_FOR_TARGET_POSITION,
		WAIT_FOR_TARGET_POSITION
	};

	enum {
		FOLLOW_FROM_RIGHT,
		FOLLOW_FROM_BEHIND,
		FOLLOW_FROM_FRONT,
		FOLLOW_FROM_LEFT
	};

	float _follow_position_matricies[4][9] = {
		{
			1.0F,  -1.0F, 0.0F,
			1.0F,   1.0F, 0.0F,
			0.0F,   0.0F, 1.0F
		}, // follow right

		{
			-1.0F,  0.0F, 0.0F,
			0.0F, -1.0F, 0.0F,
			0.0F,  0.0F, 1.0F
		}, // follow behind

		{
			1.0F,   0.0F, 0.0F,
			0.0F,   1.0F, 0.0F,
			0.0F,   0.0F, 1.0F
		}, // follow front

		{
			1.0F,   1.0F, 0.0F,
			-1.0F,   1.0F, 0.0F,
			0.0F,   0.0F, 1.0F
		}
	}; // follow left side


	Navigator *_navigator;
	control::BlockParamFloat	_param_min_alt;
	control::BlockParamFloat 	_param_tracking_dist;
	control::BlockParamInt 		_param_tracking_side;
	control::BlockParamFloat 	_param_tracking_resp;
	control::BlockParamFloat 	_param_yaw_auto_max;


	FollowTargetState _follow_target_state;
	int _follow_target_position;
#ifdef FOLLOWTARGET
	int _follow_target_sub;
    int _targ_heli_sub;
    /* Mavlink log uORB handle */
    orb_advert_t mavlink_log_pub;
    orb_advert_t _heli_followtarg_pub;
    bool firstTime;
    targ_heli_s targ_heli;

    int vision_senser_sub;

#endif
    hrt_abstime currentTime;
	float _step_time_in_ms;
	float _follow_offset;
#ifdef AUTOTEST
    hrt_abstime arrive_time;
    bool first_arrive;
    bool vel_d_flag;
    hrt_abstime second_arrive_time;
    bool second_arrive;
    float target_altitude;
    orb_advert_t follow_targ;
    orb_advert_t follow_result;
#endif

#ifdef UI_STRIVE
    ui_strive_formation_s formation;  //data from 4 different vehicles    *****zjm
    ui_strive_gcs_to_formation_s gcs_to_formation;  //data of leader uav
    int _formation_sub;
    int _gcs_to_formation_sub;
    orb_advert_t _formation_status_pub;     //publish formation status      *****zjm
#endif
	uint64_t _target_updates;
	uint64_t _last_update_time;

	math::Vector<3> _current_vel;
	math::Vector<3> _step_vel;
	math::Vector<3> _est_target_vel;
	math::Vector<3> _target_distance;
	math::Vector<3> _target_position_offset;
	math::Vector<3> _target_position_delta;
	math::Vector<3> _filtered_target_position_delta;
#ifdef FOLLOWTARGET
    math::Vector<3> _heli_yaw;
#endif
#ifdef SET_OFFSET
    float set_hgt_offset;
    bool last_rtl = false;
    bool vision_enabled = false;
    int follow_state = 0;
    bool midair_refueling = false;
    hrt_abstime refueling_time = 0;
    hrt_abstime vision_time = 0;
    bool vision_time_start = false;
#endif

#ifdef HOME_POSTION
    home_position_s home_position_distance;
    int home_position_sub;
#endif

#ifdef RTL_flag
    follow_to_commander_s  rtl_status;
    orb_advert_t rtl_status_pub_fd;
#endif
	follow_target_s _current_target_motion;
    follow_target_s _previous_target_motion;
    heli_followtarg_s _heli_follow_result;

	float _yaw_rate;
	float _responsiveness;
	float _yaw_auto_max;
	float _yaw_angle;

	// Mavlink defined motion reporting capabilities

	enum {
		POS = 0,
		VEL = 1,
		ACCEL = 2,
		ATT_RATES = 3
	};

	math::Matrix<3, 3> _rot_matrix;
	void track_target_position();
	void track_target_velocity();
	bool target_velocity_valid();
	bool target_position_valid();
	void reset_target_validity();
	void update_position_sp(bool velocity_valid, bool position_valid, float yaw_rate);
	void update_target_motion();
	void update_target_velocity();
#ifdef SET_OFFSET
    void set_offset(float x,float y);
#endif
};
