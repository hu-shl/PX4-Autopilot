/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/drone_task.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#define SEARCH_GRID_LENGTH 20
#define SEARCH_GRID_WIDTH 17.5
#define SEARCH_GRID_SPACING 1

 using namespace time_literals;

 extern "C" __EXPORT int rs_navigator_main(int argc, char *argv[]);
 class RobosubNavigator : public ModuleBase<RobosubNavigator>, public ModuleParams, public px4::WorkItem
 {
 public:
	 RobosubNavigator();

	~RobosubNavigator() override;

	 /** @see ModuleBase */
	 static int task_spawn(int argc, char *argv[]);

	 /** @see ModuleBase */
	 static int custom_command(int argc, char *argv[]);

	 /** @see ModuleBase */
	 static int print_usage(const char *reason = nullptr);

	 /** @see ModuleBase::run() */
	 void Run() override;

	 /** @see ModuleBase::print_status() */
	 int print_status() override;

	 bool init();

 private:

	 /**
	  * Check for parameter changes and update them if needed.
	  * @param parameter_update_sub uorb subscription to parameter_update
	  * @param force for a parameter update
	  */
	 void parameters_update(bool force = false);

	enum class NavTaskType {
		MOVE_XYZ,
		WAIT,
		ROTATE,
	};

	struct NavTask {
		NavTaskType type;
		matrix::Vector3f target;
		float wait_time_s;
		float heading;
	};
	static constexpr int MAX_TASKS = 8;
	NavTask _task_queue[MAX_TASKS];
	int _task_head = 0;
	int _task_tail = 0;
	bool _task_active = false;
	hrt_abstime _task_start_time = 0;

	void process_task(const matrix::Vector3f &current_pos, const float &heading);
	void add_task(const NavTask &task);

	uORB::Publication<trajectory_setpoint_s> trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _drone_task_sub{ORB_ID(drone_task)};
	drone_task_s _drone_task{};
	vehicle_local_position_s local_pos{};
	int grid_line = 0;
	bool grid_forward = true;


	void movement_test(const matrix::Vector3f &_current_pos);
	float distance_to(const matrix::Vector3f &a, const matrix::Vector3f &b) { return (a - b).norm(); }
	float heading_to(const float &a, const float &b) { return fabsf((a - b)); }
	void send_position_setpoint(const matrix::Vector3f &pos);
	void send_heading_setpoint(const float &heading);
	void search_grid(const matrix::Vector3f &_current_pos, const float &current_heading);
	float calculate_next_heading(const float &current_heading, const float &target_heading) {
		float diff = (target_heading - current_heading);
		if (double(diff) > M_PI) {
			diff -= float(2 * M_PI);
		} else if (double(diff) < -M_PI) {
			diff += float(2 * M_PI);
		}
		return float(current_heading + diff) ;
	}

	 // Subscriptions
	 uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
 };
