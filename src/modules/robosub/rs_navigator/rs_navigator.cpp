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

#include "rs_navigator.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int RobosubNavigator::print_status()
{
	PX4_INFO("Running");

	return 0;
}

int RobosubNavigator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RobosubNavigator::task_spawn(int argc, char *argv[])
{
RobosubNavigator *instance = new RobosubNavigator();

if (instance) {
	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

} else {
	PX4_ERR("alloc failed");
}

delete instance;
_object.store(nullptr);
_task_id = -1;

return PX4_ERROR;
}

bool RobosubNavigator::init() {
if (!_vehicle_local_position_sub.registerCallback()) {
	PX4_ERR("callback registration failed");
	return false;
}

return true;
}

RobosubNavigator::RobosubNavigator()
	: ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

void RobosubNavigator::add_task(const NavTask &task) {
	int next_tail = (_task_tail + 1) % MAX_TASKS;
	if (next_tail != _task_head) { // Not full
		_task_queue[_task_tail] = task;
		_task_tail = next_tail;
	}
}

void RobosubNavigator::process_task(const matrix::Vector3f &current_pos, const float &current_heading) {
    if (_task_head == _task_tail) {
        // No tasks
        return;
    }

    NavTask &task = _task_queue[_task_head];

    switch (task.type) {
    case NavTaskType::MOVE_XYZ:
        send_position_setpoint(task.target);
        if (distance_to(current_pos, task.target) < 0.2f) {
            _task_head = (_task_head + 1) % MAX_TASKS;
            _task_active = false;
        }
        break;
    case NavTaskType::WAIT:
        if (!_task_active) {
            _task_start_time = hrt_absolute_time();
            _task_active = true;
        }
        if (hrt_elapsed_time(&_task_start_time) > (hrt_abstime)(task.wait_time_s * 1e6f)) {
            _task_head = (_task_head + 1) % MAX_TASKS;
            _task_active = false;
        }
        break;

    case NavTaskType::ROTATE:
        send_heading_setpoint(task.heading);
	if (heading_to(current_heading, task.heading) < 0.1f) {
	    _task_head = (_task_head + 1) % MAX_TASKS;
	    _task_active = false;
	}
	break;
    }
}

void RobosubNavigator::movement_test(const matrix::Vector3f &current_pos) {
	if (_task_head == _task_tail) {
		add_task({NavTaskType::MOVE_XYZ, current_pos + matrix::Vector3f(2.f, 0.f, 0.f), 0});
		add_task({NavTaskType::WAIT, {}, 1.0f});
		add_task({NavTaskType::MOVE_XYZ, current_pos, 0});
		add_task({NavTaskType::WAIT, {}, 1.0f});
		add_task({NavTaskType::MOVE_XYZ, current_pos + matrix::Vector3f(-2.f, 0.f, 0.f), 0});
		add_task({NavTaskType::WAIT, {}, 1.0f});
	}
}

void RobosubNavigator::send_position_setpoint(const matrix::Vector3f &pos) {
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	setpoint.position[0] = pos(0);
	setpoint.position[1] = pos(1);
	setpoint.position[2] = pos(2);

	trajectory_setpoint_pub.publish(setpoint);
}

void RobosubNavigator::send_heading_setpoint(const float &heading) {
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	setpoint.yaw = heading;

	trajectory_setpoint_pub.publish(setpoint);
}

void RobosubNavigator::send_emergency_stop(bool up) {
    // Clear the task queue
    _task_head = _task_tail;
    _task_active = false;

    // Send a stop command to the vehicle
    trajectory_setpoint_s setpoint{};
    setpoint.timestamp = hrt_absolute_time();
    if (up) {
	setpoint.position[2] = 1.f; // Move up
    } else {
	setpoint.position[2] = 0.f; // Move down
    }
    setpoint.position[0] = 0.f;
    setpoint.position[1] = 0.f;
    setpoint.yaw = 0.f; // Stop rotation

    trajectory_setpoint_pub.publish(setpoint);
}

void RobosubNavigator::search_grid(const matrix::Vector3f &current_pos, const float &current_heading) {
    // Only add new tasks if the queue is empty and we haven't finished
    if (_task_head == _task_tail && (grid_line * SEARCH_GRID_SPACING < SEARCH_GRID_WIDTH)) {
        matrix::Vector3f start = current_pos;
        float heading = current_heading;

        // Determine direction for this line
        float direction = grid_forward ? 1.0f : -1.0f;

        // Move along the grid line
        add_task({NavTaskType::MOVE_XYZ, start + matrix::Vector3f(direction * SEARCH_GRID_LENGTH, 0.f, 0.f), 0, 0});
        add_task({NavTaskType::WAIT, {}, 1.0f, 0});

        // If not the last line, rotate and move to next line
        if ((grid_line + 1) * SEARCH_GRID_SPACING < SEARCH_GRID_WIDTH) {
            // Rotate 90 deg
            float next_heading = calculate_next_heading(heading, heading + direction * float(M_PI_2));
            add_task({NavTaskType::ROTATE, {}, 0, next_heading});
            add_task({NavTaskType::WAIT, {}, 1.0f, 0});

            // Move sideways to next line
            add_task({NavTaskType::MOVE_XYZ, start + matrix::Vector3f(direction * SEARCH_GRID_LENGTH, (grid_line + 1) * SEARCH_GRID_SPACING, 0.f), 0, 0});
            add_task({NavTaskType::WAIT, {}, 1.0f, 0});

            // Rotate back to original heading (opposite direction)
            float return_heading = calculate_next_heading(next_heading, heading + float(M_PI));
            add_task({NavTaskType::ROTATE, {}, 0, return_heading});
            add_task({NavTaskType::WAIT, {}, 1.0f, 0});
        }

        // Prepare for next line
        grid_line++;
        grid_forward = !grid_forward;
    }
}

void RobosubNavigator::Run()
{
	// check for parameter updates
	parameters_update();

	// check for drone task updates
	if (_drone_task_sub.updated()) {
		_drone_task_sub.copy(&_drone_task);
	}
	if (_status_sub.update(&status_msg)) { // I definatly don't agree with the way we handle emergency stop but whatever
		if (status_msg.status == status_s::STATUS_HIGH_VALUE_DETECTED) {
			PX4_ERR("High value detected, stopping navigation");
			send_emergency_stop(true);
		}
		else if (status_msg.status == status_s::STATUS_LOW_BATTERY) {
			PX4_ERR("Low battery detected, stopping navigation");
			send_emergency_stop(false);
		}
		else if (status_msg.status == status_s::STATUS_CRITICAL_BATTERY) {
			PX4_ERR("Critical battery level, stopping navigation");
			send_emergency_stop(false);
		}
		return;
	}
	if (_drone_task.task == drone_task_s::TASK_AUTONOMOUS) {
		if (_vehicle_local_position_sub.update(&local_pos)) {
			matrix::Vector3f current_pos(local_pos.x, local_pos.y, local_pos.z);
			float current_heading = local_pos.heading;
			movement_test(current_pos);

			process_task(current_pos, current_heading);
		}
	}
}

void RobosubNavigator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int RobosubNavigator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "rs navigator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

RobosubNavigator::~RobosubNavigator()
{
}

int rs_navigator_main(int argc, char *argv[])
{
	return RobosubNavigator::main(argc, argv);
}
