/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * This modules takes in the Control from both the manual and auto control
 * setpoints and applies the PID control to the motors. PID output is published
 * as a thrust- and torque setpoint, used by the thruster control module.
 *
 * @author Daan Smienk <daansmienk10@gmail.com>
 * @author Thijs Vader <thijs.vader@student.hu.nl>
 */

#include "rs_pos_control.hpp"
#include "../rs_motor_control/rs_motor_control.hpp"
#include "px4_platform_common/defines.h"
#include "px4_platform_common/log.h"

/**
 * Robosub motor control app start / stop handling function
 *
 * @ingroup apps
 */
RobosubPosControl::RobosubPosControl()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        px4::wq_configurations::nav_and_controllers),

      /* performance counters */
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
}

RobosubPosControl::~RobosubPosControl() { perf_free(_loop_perf); }

bool RobosubPosControl::init()
{
        // initialize parameters
        parameters_update(true);

        // queue the first run call
        ScheduleNow();

        return true;
}

int RobosubPosControl::parameters_update(bool force)
{
        // check for parameter updates
        if (_parameter_update_sub.updated() || force)
        {
                // clear update
                parameter_update_s update;
                _parameter_update_sub.copy(&update);

                // update parameters from storage
                updateParams();

                return 1; // return 1 to indicate parameters were updated
        }

        return 0; // return 0 to indicate no parameters were updated
}
/**
 * @brief Publish the thrust setpoint
 *  @param thrust_x The thrust setpoint in the x direction
 *  @param thrust_y The thrust setpoint in the y direction
 *  @param thrust_z The thrust setpoint in the z direction
 */
void RobosubPosControl::publish_thrust_setpoint(const float thrust_x,
                                                const float thrust_y,
                                                const float thrust_z)
{
        vehicle_thrust_setpoint_s vehicle_thrust_setpoint = {};
        vehicle_thrust_setpoint.timestamp = hrt_absolute_time();

        vehicle_thrust_setpoint.xyz[0] = thrust_x;
        vehicle_thrust_setpoint.xyz[1] = thrust_y;
        vehicle_thrust_setpoint.xyz[2] = thrust_z;

        _thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
}

/**
 * @brief Publish the attitude setpoint
 *  @param roll The roll setpoint
 *  @param pitch The pitch setpoint
 *  @param yaw The yaw setpoint
 */
void RobosubPosControl::publish_torque_setpoint(const float torque_roll,
                                                const float torque_pitch,
                                                const float torque_yaw)
{
        vehicle_torque_setpoint_s vehicle_torque_setpoint = {};
        vehicle_torque_setpoint.timestamp = hrt_absolute_time();

        vehicle_torque_setpoint.xyz[0] = torque_roll;
        vehicle_torque_setpoint.xyz[1] = torque_pitch;
        vehicle_torque_setpoint.xyz[2] = torque_yaw;

        _torque_setpoint_pub.publish(vehicle_torque_setpoint);
}

void RobosubPosControl::Run()
{
        PX4_INFO("RobosubPosControl::Run()");

        // only run the task if not to exit
        if (should_exit())
        {
                exit_and_cleanup();
                return;
        }

        // Start performance counter
        perf_begin(_loop_perf);

        // update drone task
        _drone_task_sub.update(&_drone_task);

        // update setpoint based on task
        // switch (_drone_task.task)
        // {
        // case (_drone_task.TASK_AUTONOMOUS):

        //         _auto_control_setpoint_sub.copy(&_vehicle_setpoint);
        //         break;

        // case (_drone_task.TASK_REMOTE_CONTROLLED):

        //         _manual_control_setpoint_sub.copy(&_vehicle_setpoint);
        //         break;

        // // dont update setpoint if task is unknown
        // default:
        //         PX4_ERR("Unknown task value: %d", _drone_task.task);
        //         break;
        // }

        _trajectory_setpoint_sub.update(&_trajectory_setpoint);

        // update parameters if needed
        if (parameters_update(_force_param.get()))
        {
                X_Axis.P_gain = _p_gain_p_x.get();
                X_Axis.PID_P_gain = _pid_gain_p_x.get();
                X_Axis.PID_I_gain = _pid_gain_i_x.get();
                X_Axis.PID_D_gain = _pid_gain_d_x.get();
                X_Axis.velocity_constraint = _p_output_limit.get();
                X_Axis.thrust_constraint = _pid_output_limit.get();

                Y_Axis.P_gain = _p_gain_p_y.get();
                Y_Axis.PID_P_gain = _pid_gain_p_y.get();
                Y_Axis.PID_I_gain = _pid_gain_i_y.get();
                Y_Axis.PID_D_gain = _pid_gain_d_y.get();
                Y_Axis.velocity_constraint = _p_output_limit.get();
                Y_Axis.thrust_constraint = _pid_output_limit.get();

                Z_Axis.P_gain = _p_gain_p_z.get();
                Z_Axis.PID_P_gain = _pid_gain_p_z.get();
                Z_Axis.PID_I_gain = _pid_gain_i_z.get();
                Z_Axis.PID_D_gain = _pid_gain_d_z.get();
                Z_Axis.velocity_constraint = _p_output_limit.get();
                Z_Axis.thrust_constraint = _pid_output_limit.get();

                Roll_Axis.P_gain = _p_gain_p_roll.get();
                Roll_Axis.PID_P_gain = _pid_gain_p_roll.get();
                Roll_Axis.PID_I_gain = _pid_gain_i_roll.get();
                Roll_Axis.PID_D_gain = _pid_gain_d_roll.get();
                Roll_Axis.velocity_constraint = _p_output_limit.get();
                Roll_Axis.thrust_constraint = _pid_output_limit.get();

                Pitch_Axis.P_gain = _p_gain_p_pitch.get();
                Pitch_Axis.PID_P_gain = _pid_gain_p_pitch.get();
                Pitch_Axis.PID_I_gain = _pid_gain_i_pitch.get();
                Pitch_Axis.PID_D_gain = _pid_gain_d_pitch.get();
                Pitch_Axis.velocity_constraint = _p_output_limit.get();
                Pitch_Axis.thrust_constraint = _pid_output_limit.get();

                Yaw_Axis.P_gain = _p_gain_p_yaw.get();
                Yaw_Axis.PID_P_gain = _pid_gain_p_yaw.get();
                Yaw_Axis.PID_I_gain = _pid_gain_i_yaw.get();
                Yaw_Axis.PID_D_gain = _pid_gain_d_yaw.get();
                Yaw_Axis.velocity_constraint = _p_output_limit.get();
                Yaw_Axis.thrust_constraint = _pid_output_limit.get();

        }

        // update incomming data
        _vehicle_local_position_sub.update(&_vehicle_local_position);  // position
        _vehicle_attitude_sub.update(&_vehicle_attitude); // attitude

        X_Axis.position_current = _vehicle_local_position.x;
        X_Axis.velocity_current = _vehicle_local_position.vx;
        X_Axis.position_setpoint = _vehicle_setpoint.x;
        X_Axis.velocity_setpoint = _vehicle_setpoint.x;
        X_Axis.input_type = INPUT_TYPE_VELOCITY;

        Y_Axis.position_current = _vehicle_local_position.y;
        Y_Axis.velocity_current = _vehicle_local_position.vy;
        Y_Axis.position_setpoint = _vehicle_setpoint.y;
        Y_Axis.velocity_setpoint = _vehicle_setpoint.y;
        Y_Axis.input_type = INPUT_TYPE_VELOCITY;

        Z_Axis.position_current = _vehicle_local_position.z;
        Z_Axis.velocity_current = _vehicle_local_position.vz;
        Z_Axis.position_setpoint = _vehicle_setpoint.z;
        Z_Axis.velocity_setpoint = _vehicle_setpoint.z;
        Z_Axis.input_type = INPUT_TYPE_POSITION;

        Roll_Axis.position_current = _vehicle_attitude.roll;
        Roll_Axis.velocity_current =
        Roll_Axis.position_setpoint = _vehicle_setpoint.roll;
        Roll_Axis.velocity_setpoint = 0;
        Roll_Axis.input_type = INPUT_TYPE_POSITION;

        Pitch_Axis.position_current = _vehicle_attitude.pitch;
        Pitch_Axis.velocity_current =
        Pitch_Axis.position_setpoint = _vehicle_setpoint.pitch;
        Pitch_Axis.velocity_setpoint = 0;
        Pitch_Axis.input_type = INPUT_TYPE_POSITION;

        Yaw_Axis.position_current = _vehicle_attitude.yaw;
        Yaw_Axis.velocity_current =
        Yaw_Axis.position_setpoint = _vehicle_setpoint.yaw;
        Yaw_Axis.velocity_setpoint = 0;
        Yaw_Axis.input_type = INPUT_TYPE_POSITION;

        run_axis_pid(X_Axis);
        run_axis_pid(Y_Axis);
        run_axis_pid(Z_Axis);
        run_axis_pid(Roll_Axis);
        run_axis_pid(Pitch_Axis);
        run_axis_pid(Yaw_Axis);

        // publish the thrust setpoint
        publish_thrust_setpoint(X_Axis.pid_output, Y_Axis.pid_output, Z_Axis.pid_output);

        // publish the torque setpoint
        publish_torque_setpoint(Roll_Axis.pid_output, Pitch_Axis.pid_output, Yaw_Axis.pid_output);


        ScheduleDelayed(
            1000000 / _pos_control_freq.get()); // Schedule next run at the desired frequency

        perf_end(_loop_perf);
}

/**
 * @brief Spawn the Robosub Position Control task
 *
 * This function creates an instance of the RobosubPosControl class and
 * initializes it. If successful, it stores the instance in a static object
 * store and sets the task ID.
 *
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit status
 */
int RobosubPosControl::task_spawn(int argc, char *argv[])
{
        RobosubPosControl *instance = new RobosubPosControl();

        if (instance)
        {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init())
                {
                        return PX4_OK;
                }
        }
        else
        {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

/**
 * @brief Custom command handling for the Robosub Position Control module
 *
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit status
 */
int RobosubPosControl::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

/**
 * @brief Print usage information for the Robosub Position Control module
 *
 * @param reason Optional reason for printing usage
 * @return Exit status
 */
int RobosubPosControl::print_usage(const char *reason)
{
        if (reason)
        {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
PID controller to control Thrusters and Buoayncy System.
Has no commands for now.
)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("_robosub_pos_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start")
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

/**
 * @brief Main entry point for the Robosub Position Control module
 *
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit status
 */
int rs_pos_control_main(int argc, char *argv[])
{
        return RobosubPosControl::main(argc, argv);
}

void RobosubPosControl::run_axis_pid(AxisPID_s &axis)
{
        // perform P controller: go from position to velocity
        float position_error = axis.position_setpoint - axis.position_current;
        float p_output = axis.P_gain * position_error;
        float p_constrained_output = math::constrain(p_output, -axis.velocity_constraint, axis.velocity_constraint);

        float velocity_setpoint = 0.0f;

        // choose between position or velocity setpoint based on task
        if (axis.input_type == INPUT_TYPE_POSITION)
        {
                // if input is position, use the P output as velocity setpoint
                axis.velocity_setpoint = p_constrained_output;
        }
        else if (axis.input_type == INPUT_TYPE_VELOCITY)
        {
                // if input is velocity, use the setpoint directly
                axis.velocity_setpoint = axis.velocity_setpoint;
        }

        axis.delta_time = hrt_elapsed_time(&axis.last_run_time) / 1e6f; // convert to seconds
        axis.last_run_time = hrt_absolute_time(); // update last run time

        axis.PID_controller.setSetpoint(velocity_setpoint);
        axis.PID_controller.setGains(axis.PID_P_gain, axis.PID_I_gain, axis.PID_D_gain);
        axis.PID_controller.setOutputLimit(axis.thrust_constraint);
        axis.pid_output = axis.PID_controller.update(axis.velocity_current, axis.delta_time, true);
}
