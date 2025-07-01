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

#include "rs_remote_control.hpp"

#include "../rs_motor_control/rs_motor_control.hpp"

// PX4 defines for InternalSensors.msg
#define SENSOR_HUMIDITY 0
#define SENSOR_TEMPERATURE 1
#define SENSOR_PRESSURE 2

extern "C" __EXPORT int rs_remote_control_main(int argc, char *argv[]);

int RobosubRemoteControl::print_status() {
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module

        return 0;
}

int RobosubRemoteControl::custom_command(int argc, char *argv[]) {
        /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

        return print_usage("unknown command");
}

int RobosubRemoteControl::task_spawn(int argc, char *argv[]) {
        RobosubRemoteControl *instance = new RobosubRemoteControl();

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

bool RobosubRemoteControl::init() {
        // Execute the Run() function everytime an input_rc is publiced
        // if (!.registerCallback()) {
        // 	PX4_ERR("callback registration failed");
        // 	return true;
        // }

        ScheduleOnInterval(100_ms);
        PX4_DEBUG("RobosubRemoteControl::init()");
        return true;
}

RobosubRemoteControl::RobosubRemoteControl()
    : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {}

RobosubRemoteControl::~RobosubRemoteControl() {
        // clean up if necessary
}

void RobosubRemoteControl::Run() {
        perf_begin(_loop_perf);

        float roll_u, pitch_u, yaw_u = 0.0f;
        float thrust_x, thrust_y, thrust_z = 0.0f;

        // Rami-2025 bypass leakage safety
        taskStat(); // Set bitReg to capture which task to handle
        receiver(); // Receive rc_input

        if (bitReg == TASK_REMOTECONTROLLED) {
                // Set desired torque and thrust
                roll_u = 0.0f;
                pitch_u = normalized[3];
                yaw_u = 0.0f;
                thrust_x = normalized[0];
                thrust_y = normalized[2];
                thrust_z = normalized[1];

                // Apply water detection safety
                apply_water_safety(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);
                constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);

                publish_thrust_setpoint();
                publish_torque_setpoint();
        } else if (bitReg == TASK_RC_PID) // Perform remote controlled PID
        {
                // call PID pos controller?
                // constrain command (set torque and thrust setpoint)
        }

        // if (!_status_sub.update(&_status_msg) && status_safe) {
        //         taskStat();

        //         receiver();
        // } else { // I don't agree with handling the emergency in here. I think it should be handled in the position
        // controller. 	status_safe = false; 	if (_status_msg.status == status_s::STATUS_HIGH_VALUE_DETECTED) { 		if
        // (status_emergency_start == 0) { 			status_emergency_start = hrt_absolute_time();
        // 		}
        // 		RobosubMotorControl robosub_motor_control;
        // 		if (hrt_elapsed_time(&status_emergency_start) > 5_s) {
        // 			robosub_motor_control.actuator_test(MOTOR_UP1, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_UP2, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_UP3, 0.0f, 0, false);
        // 		}
        // 		else {
        // 			robosub_motor_control.actuator_test(MOTOR_FORWARDS1, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_FORWARDS2, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_SIDE1, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_SIDE2, 0.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_UP1, 1.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_UP2, 1.0f, 0, false);
        // 			robosub_motor_control.actuator_test(MOTOR_UP3, 1.0f, 0, false);
        // 		}
        // 	} else if (_status_msg.status == status_s::STATUS_LOW_BATTERY) {
        // 		PX4_ERR("Low battery detected");
        // 	} else if (_status_msg.status == status_s::STATUS_CRITICAL_BATTERY) {
        // 		PX4_ERR("Critical battery level");
        // 	}
        // }

        // Schedule();
        perf_end(_loop_perf);
}

/**
 * @brief Limit pitch, roll, yaw and thust xyz if water is detected
 *
 */
void RobosubRemoteControl::apply_water_safety(float &p_roll_u, float &p_pitch_u, float &p_yaw_u, float &p_thrust_x,
                                              float &p_thrust_y, float &p_thrust_z) {
        float safety_factor = 1.0f;

        if (_water_detection_sub.update(&_water_detection)) { // update water detection sensors
                sensor_mainbrain = _water_detection.mainbrain_sensor;
                sensor_power = _water_detection.power_module_sensor;
        }

        sensor_mainbrain = true; // Force mainbrain true Needed for RAMI-2025

        if (!sensor_mainbrain && !sensor_power) // Limit to 40 percent if no water detected
        {
                safety_factor = 0.4f;
        } else if (!sensor_mainbrain && sensor_power) // AUV is on the water
        {
                safety_factor = 0.4f;
        } else if (sensor_mainbrain &&
                   sensor_power) // Water is detected above (mainbrain) and below (power) AUV. Under water
        {
                safety_factor = 1.0f; // Full power 1.0f
        } else if (!sensor_power) {   // Needed for for-2025 because we don't have the sensor in the mainbrain
                safety_factor = 0.4f;
        }

        p_roll_u *= safety_factor;
        p_pitch_u *= safety_factor;
        p_yaw_u *= safety_factor;
        p_thrust_x *= safety_factor;
        p_thrust_y *= safety_factor;
        p_thrust_z *= safety_factor;
}

void RobosubRemoteControl::taskStat() {
        update1 = 0;
        if (_input_rc_sub.update(&_input_rc)) {
                update1 = 1;
                input_rc_s rc_data{};
                _input_rc_sub.copy(&rc_data);
                normalized[4] = (rc_data.values[4] - 1500) / 400.0f;
                normalized[5] = (rc_data.values[5] - 1500) / 400.0f;
                normalized[6] = (rc_data.values[6] - 1500) / 400.0f;
                normalized[7] = (rc_data.values[7] - 1500) / 400.0f;

                normalized[4] = math::constrain(normalized[4], -1.0f, 1.0f);
                normalized[5] = math::constrain(normalized[5], -1.0f, 1.0f);
                normalized[6] = math::constrain(normalized[6], -1.0f, 1.0f);
                normalized[7] = math::constrain(normalized[7], -1.0f, 1.0f);

                uint8_t stateEnable((normalized[4] > 0.0f) ? 1 : 0);

                if (stateEnable == 1) {
                        bitReg = ((normalized[5] > 0.0f) ? 1 : 0) | ((normalized[6] > 0.0f) ? 1 : 0) << 1 |
                                 ((normalized[7] > 0.0f) ? 1 : 0) << 2;
                        switch (bitReg) {
                        case 0b000:
                                _drone_task.task = TASK_REMOTECONTROLLED;
                                break;
                        case 0b001:
                                // _drone_task.task = TASK_BUOYANCYCTRL;
                                _drone_task.task = TASK_RC_PID; // RAMI-2025 use this as rc pid?
                                break;
                        case 0b010:
                                _drone_task.task = TASK_DPGOAL;
                                break;
                        case 0b011:
                                _drone_task.task = TASK_DPTELEARM;
                                break;
                        case 0b100:
                                _drone_task.task = TASK_SEARCHBUOY;
                                break;
                        case 0b101:
                                _drone_task.task = TASK_SEARCHTUBE;
                                break;
                        case 0b110:
                                _drone_task.task = TASK_TASK2;
                                break;
                        case 0b111:
                                _drone_task.task = TASK_TASK1;
                                break;
                        default:
                                _drone_task.task = TASK_REMOTECONTROLLED;
                                break;
                        }

                        _drone_task.timestamp = hrt_absolute_time();
                        if (!armed) {
                                _vehicle_command_arm.timestamp = hrt_absolute_time();
                                _vehicle_command_arm.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
                                _vehicle_command_arm.param1 = vehicle_command_s::ARMING_ACTION_ARM;
                                _vehicle_command_arm.param2 = 21196; // Some magic number to force the arm command
                                _vehicle_command_pub.publish(_vehicle_command_arm);
                                armed = true;
                        }

                        _drone_task_pub.publish(_drone_task);
                }
        }
}

void RobosubRemoteControl::receiver() {
        RobosubMotorControl robosub_motor_control;

        if (update1) {
                if (bitReg == TASK_REMOTECONTROLLED) {
                        input_rc_s rc_data{};
                        _input_rc_sub.copy(&rc_data);

                        // Normalize the rc data to a value between -1 and 1
                        normalized[0] = (rc_data.values[1] - 1500) / 400.0f; // thrust y
                        normalized[1] = (rc_data.values[2] - 1500) / 400.0f; // thrust z
                        normalized[2] = (rc_data.values[3] - 1500) / 400.0f; // thrust x
                        normalized[3] = (rc_data.values[0] - 1500) / 400.0f; // pitch

                        // robosub_motor_control.actuator_test(MOTOR_FORWARDS1, normalized[0] *
                        // _param_thrust_t200_limiter.get(), 0, false);
                        // robosub_motor_control.actuator_test(MOTOR_FORWARDS2, normalized[0] *
                        // _param_thrust_t200_limiter.get(), 0, false);

                        // robosub_motor_control.actuator_test(MOTOR_UP1, -normalized[1], 0, false);
                        // robosub_motor_control.actuator_test(MOTOR_UP2, (normalized[1] *
                        // _param_front_up_motor_reduction.get()), 0, false);
                        // robosub_motor_control.actuator_test(MOTOR_UP3, (-normalized[1] *
                        // _param_front_up_motor_reduction.get()), 0, false);

                        // if (normalized[2] > 0.1f || normalized[2] < -0.1f) {
                        //         robosub_motor_control.actuator_test(MOTOR_SIDE1, -normalized[2], 0, false);
                        //         robosub_motor_control.actuator_test(MOTOR_SIDE2, normalized[2], 0, false);
                        // } else {
                        //         if (normalized[3] <= 0)
                        // 	robosub_motor_control.actuator_test(MOTOR_UP1, -normalized[3], 0, false);
                        //         else if (normalized[3] >= 0) {
                        //                 robosub_motor_control.actuator_test(MOTOR_UP2, (-normalized[3] *
                        //                 (_param_tilt_modifier.get() * _param_front_up_motor_reduction.get())), 0,
                        //                 false); robosub_motor_control.actuator_test(MOTOR_UP3, (-normalized[3] *
                        //                 (_param_tilt_modifier.get() * _param_front_up_motor_reduction.get())), 0,
                        //                 false);
                        //         }
                        // }
                }
                update1 = 0;
        }
}

/**
 * @brief constrains values and put setpoint
 *
 * Borrow from UUVAttitudeControl::constrain_actuator_commands
 * @param pitch_u float
 */
void RobosubRemoteControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_x,
                                                       float thrust_y, float thrust_z) {

        if (PX4_ISFINITE(roll_u)) {
                roll_u = math::constrain(roll_u, -1.0f, 1.0f);
                vehicle_torque_setpoint.xyz[0] = roll_u;

        } else {
                vehicle_torque_setpoint.xyz[0] = 0.0f;
        }

        if (PX4_ISFINITE(pitch_u)) {
                pitch_u = math::constrain(pitch_u, -1.0f, 1.0f);
                vehicle_torque_setpoint.xyz[1] = pitch_u;

        } else {
                vehicle_torque_setpoint.xyz[1] = 0.0f;
        }

        if (PX4_ISFINITE(yaw_u)) {
                yaw_u = math::constrain(yaw_u, -1.0f, 1.0f);
                vehicle_torque_setpoint.xyz[2] = yaw_u;

        } else {
                vehicle_torque_setpoint.xyz[2] = 0.0f;
        }

        if (PX4_ISFINITE(thrust_x)) {
                thrust_x = math::constrain(thrust_x, -1.0f, 1.0f);
                vehicle_thrust_setpoint.xyz[0] = thrust_x;

        } else {
                vehicle_thrust_setpoint.xyz[0] = 0.0f;
        }
}

// void RobosubRemoteControl::remote_buoyancy(){
// 	if (update1) {
//                 if (bitReg == TASK_BUOYANCYCTRL) {
//                         input_rc_s rc_data{};
//                         _input_rc_sub.copy(&rc_data);

// 			normalized[0] = (rc_data.values[1] - 1500) / 400.0f;
//                         normalized[1] = (rc_data.values[2] - 1500) / 400.0f;
//                         normalized[2] = (rc_data.values[3] - 1500) / 400.0f;
//                         // normalized[3] = (rc_data.values[0] - 1500) / 400.0f;

// 			normalized[0] = math::constrain(normalized[0],  -range, range);
// 			normalized[1] = math::constrain(normalized[1],  -range, range);
// 			normalized[2] = math::constrain(normalized[2],  -range, range);
// 			// normalized[3] = math::constrain(normalized[3],  -range, range);

// 			if(normalized[0] >= -THRESHOLD && normalized[0] <= THRESHOLD)
// 				_buoyancy_ctrl.states[0] = KEEP;
// 			else if(normalized[0] >= THRESHOLD)
// 				_buoyancy_ctrl.states[0] = FILL;
// 			else if(normalized[0] <= THRESHOLD)
// 				_buoyancy_ctrl.states[0] = EMPTY;

// 			if(normalized[1] >= -THRESHOLD && normalized[1] <= THRESHOLD)
// 				_buoyancy_ctrl.states[1] = KEEP;
// 			else if(normalized[1] >= THRESHOLD)
// 				_buoyancy_ctrl.states[1] = FILL;
// 			else if(normalized[1] <= THRESHOLD)
// 				_buoyancy_ctrl.states[1] = EMPTY;

// 			if(normalized[2] >= -THRESHOLD && normalized[2] <= THRESHOLD)
// 				_buoyancy_ctrl.states[2] = KEEP;
// 			else if(normalized[2] >= THRESHOLD)
// 				_buoyancy_ctrl.states[2] = FILL;
// 			else if(normalized[2] <= THRESHOLD)
// 				_buoyancy_ctrl.states[2] = EMPTY;

// 			// if(normalized[3] >= -THRESHOLD && normalized[3] <= THRESHOLD)
// 			// 	_buoyancy_ctrl.states[3] = KEEP;
// 			// else if(normalized[3] >= THRESHOLD)
// 			// 	_buoyancy_ctrl.states[3] = FILL;
// 			// else if(normalized[3] <= THRESHOLD)
// 			// 	_buoyancy_ctrl.states[3] = EMPTY;
// 			// _buoyancy_ctrl_states[3] = KEEP;

// 			_buoyancy_ctrl.timestamp = hrt_absolute_time();
// 			_buoyancy_ctrl_pub.publish(_buoyancy_ctrl);
// 		}
// 	}
// }

/**
 * @brief Publish the thrust setpoint
 *  @param thrust_x The thrust setpoint in the x direction
 *  @param thrust_y The thrust setpoint in the y direction
 *  @param thrust_z The thrust setpoint in the z direction
 */
// void RobosubRemoteControl::publish_thrust_setpoint(const float thrust_x,
//                                                 const float thrust_y,
//                                                 const float thrust_z)
void RobosubRemoteControl::publish_thrust_setpoint(void) {
        // vehicle_thrust_setpoint_s vehicle_thrust_setpoint = {};
        vehicle_thrust_setpoint.timestamp = hrt_absolute_time();

        // vehicle_thrust_setpoint.xyz[0] = thrust_x;
        // vehicle_thrust_setpoint.xyz[1] = thrust_y;
        // vehicle_thrust_setpoint.xyz[2] = thrust_z;

        _thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
}

/**
 * @brief Publish the attitude setpoint
 *  @param roll The roll setpoint
 *  @param pitch The pitch setpoint
 *  @param yaw The yaw setpoint
 */
// void RobosubRemoteControl::publish_torque_setpoint(const float torque_roll,
//                                                 const float torque_pitch,
//                                                 const float torque_yaw)
void RobosubRemoteControl::publish_torque_setpoint(void) {
        // vehicle_torque_setpoint_s vehicle_torque_setpoint = {};
        vehicle_torque_setpoint.timestamp = hrt_absolute_time();

        // vehicle_torque_setpoint.xyz[0] = torque_roll;
        // vehicle_torque_setpoint.xyz[1] = torque_pitch;
        // vehicle_torque_setpoint.xyz[2] = torque_yaw;

        _torque_setpoint_pub.publish(vehicle_torque_setpoint);
}

void RobosubRemoteControl::parameters_update(bool force) {
        // check for parameter updates
        if (_parameter_update_sub.updated() || force) {
                // clear update
                parameter_update_s update;
                _parameter_update_sub.copy(&update);

                // update parameters from storage
                updateParams();
        }
}

int RobosubRemoteControl::print_usage(const char *reason) {
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

        PRINT_MODULE_USAGE_NAME("module", "rs arm control");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int rs_remote_control_main(int argc, char *argv[]) { return RobosubRemoteControl::main(argc, argv); }
