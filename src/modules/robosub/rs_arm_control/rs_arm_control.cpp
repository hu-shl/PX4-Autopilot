#include "rs_arm_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <cstring>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#define MODULE_NAME "rs_arm_control"

int RobosubArmControl::print_status() {
	PX4_INFO("Running");
	return 0;
}

int RobosubArmControl::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int RobosubArmControl::task_spawn(int argc, char *argv[]) {
	RobosubArmControl *instance = new RobosubArmControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->Init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool RobosubArmControl::Init() {
	ScheduleOnInterval(100_ms);
	PX4_DEBUG("RobosubArmControl::init()");
	return true;
}

RobosubArmControl::RobosubArmControl()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	  _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {}

RobosubArmControl::~RobosubArmControl() {
	// Cleanup if needed
}

void RobosubArmControl::Run() {
	perf_begin(_loop_perf);

	// _arm_ctrl.states[0] = 0; // SEG1
	// _arm_ctrl.states[1] = 1; // SEG2
	// _arm_ctrl.states[2] = 2; // SEG3
	// _arm_ctrl.states[3] = 0; // BASE
	// _arm_ctrl.states[4] = 1; // GRIPROT
	// _arm_ctrl.states[5] = 2; // GRIP

	_arm_ctrl.timestamp = hrt_absolute_time();
	_arm_ctrl_pub.publish(_arm_ctrl);
	teleoperated_arm();

	// if(_drone_task_sub.update(&_drone_task))
	// {
	// 	// _drone_task_sub.copy(&_drone_task);
	// 	// teleoperated_arm();
	// }


		perf_end(_loop_perf);
}

void RobosubArmControl::teleoperated_arm() {
	// if(_drone_task.task == TELEARM)
	if(true)
	{
		if (_input_rc_sub.update(&_input_rc)) {
		// if(1) {
			// _input_rc_sub.copy(&_input_rc);

			normalized[0] = (_input_rc.values[1] - 1500) / 400.0f;
			normalized[1] = (_input_rc.values[2] - 1500) / 400.0f;
			normalized[2] = (_input_rc.values[3] - 1500) / 400.0f;
			normalized[3] = (_input_rc.values[0] - 1500) / 400.0f;

			// normalized[0] = 0.0f;
			// normalized[1] = 0.0f;
			// normalized[2] = 0.0f;
			// normalized[3] = 0.0f;

			for (int i = 0; i < 4; i++) {
				normalized[i] = math::constrain(normalized[i], -1.0f, 1.0f);
			}

			if ((normalized[0]) < THRESHOLD && -normalized[0] >= -THRESHOLD)
				_arm_ctrl.states[SEG1] = HOLD;
			else if (normalized[0] < -THRESHOLD)
				_arm_ctrl.states[SEG1] = EXTEND;
			else
				_arm_ctrl.states[SEG1] = CONTRACT;

			if ((normalized[1]) < THRESHOLD)
				_arm_ctrl.states[SEG2] = HOLD;
			else if (normalized[1] < 0)
				_arm_ctrl.states[SEG2] = EXTEND;
			else
				_arm_ctrl.states[SEG2] = CONTRACT;

			if (normalized[2] < -THRESHOLD) {
				if (normalized[2] >= -0.45f)
					_arm_ctrl.states[BASE] = CONTRACT;
				else
					_arm_ctrl.states[BASE] = EXTEND;
			} else if (normalized[2] > THRESHOLD) {
				_arm_ctrl.states[GRIP] = EXTEND;
			} else {
				_arm_ctrl.states[BASE] = HOLD;
				_arm_ctrl.states[GRIP] = CONTRACT;
			}

			if ((normalized[3]) < THRESHOLD)
				_arm_ctrl.states[SEG3] = HOLD;
			else if (normalized[3] < 0)
				_arm_ctrl.states[SEG3] = EXTEND;
			else
				_arm_ctrl.states[SEG3] = CONTRACT;

		// 	_arm_ctrl.states[0] = 0;
		// 	_arm_ctrl.states[1] = 1;
		// 	_arm_ctrl.states[2] = 2;
		// 	_arm_ctrl.states[3] = 0;
		// 	_arm_ctrl.states[4] = 1;
		// 	_arm_ctrl.states[5] = 2;

			// memcpy(_arm_ctrl.states, _arm_ctrl.states, 6);

			// _arm_ctrl.timestamp = hrt_absolute_time();
			// _arm_ctrl_pub.publish(_arm_ctrl);


	}
}
}

void RobosubArmControl::parameters_update(bool force) {
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

int RobosubArmControl::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Control code for Robosub's robotic arm.

### Implementation
Uses ScheduledWorkItem to periodically run logic based on RC inputs.

### Examples
CLI usage example:
$ rs_arm_control start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rs_arm_control", "robosub");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rs_arm_control_main(int argc, char *argv[]) {
	return RobosubArmControl::main(argc, argv);
}
