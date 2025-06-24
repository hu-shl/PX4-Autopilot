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

        // update parameters if needed
        if (parameters_update(_force_param.get()))
        {
                X_Axis.PID1_P_gain = _pid1_gain_x_p.get();
                X_Axis.PID1_I_gain = _pid1_gain_x_i.get();
                X_Axis.PID1_D_gain = _pid1_gain_x_d.get();
                X_Axis.PID1_I_limit = _pid1_i_limit_x.get();
                X_Axis.PID1_out_limit = _pid1_output_limit_x.get();
                X_Axis.PID1_out_scale = _pid1_output_scale_x.get();
                X_Axis.PID2_P_gain = _pid2_gain_x_p.get();
                X_Axis.PID2_I_gain = _pid2_gain_x_i.get();
                X_Axis.PID2_D_gain = _pid2_gain_x_d.get();
                X_Axis.PID2_I_limit = _pid2_i_limit_x.get();
                X_Axis.PID2_out_limit = _pid2_output_limit.get();
                X_Axis.PID2_out_scale = _pid2_output_scale_x.get();

                Y_Axis.PID1_P_gain = _pid1_gain_y_p.get();
                Y_Axis.PID1_I_gain = _pid1_gain_y_i.get();
                Y_Axis.PID1_D_gain = _pid1_gain_y_d.get();
                Y_Axis.PID1_I_limit = _pid1_i_limit_y.get();
                Y_Axis.PID1_out_limit = _pid1_output_limit_y.get();
                Y_Axis.PID1_out_scale = _pid1_output_scale_y.get();
                Y_Axis.PID2_P_gain = _pid2_gain_y_p.get();
                Y_Axis.PID2_I_gain = _pid2_gain_y_i.get();
                Y_Axis.PID2_D_gain = _pid2_gain_y_d.get();
                Y_Axis.PID2_I_limit = _pid2_i_limit_y.get();
                Y_Axis.PID2_out_limit = _pid2_output_limit.get();
                Y_Axis.PID2_out_scale = _pid2_output_scale_y.get();

                Z_Axis.PID1_P_gain = _pid1_gain_z_p.get();
                Z_Axis.PID1_I_gain = _pid1_gain_z_i.get();
                Z_Axis.PID1_D_gain = _pid1_gain_z_d.get();
                Z_Axis.PID1_I_limit = _pid1_i_limit_z.get();
                Z_Axis.PID1_out_limit = _pid1_output_limit_z.get();
                Z_Axis.PID1_out_scale = _pid1_output_scale_z.get();
                Z_Axis.PID2_P_gain = _pid2_gain_z_p.get();
                Z_Axis.PID2_I_gain = _pid2_gain_z_i.get();
                Z_Axis.PID2_D_gain = _pid2_gain_z_d.get();
                Z_Axis.PID2_I_limit = _pid2_i_limit_z.get();
                Z_Axis.PID2_out_limit = _pid2_output_limit.get();
                Z_Axis.PID2_out_scale = _pid2_output_scale_z.get();

                Pitch_Axis.PID1_P_gain = _pid1_gain_pitch_p.get();
                Pitch_Axis.PID1_I_gain = _pid1_gain_pitch_i.get();
                Pitch_Axis.PID1_D_gain = _pid1_gain_pitch_d.get();
                Pitch_Axis.PID1_I_limit = _pid1_i_limit_pitch.get();
                Pitch_Axis.PID1_out_limit = _pid1_output_limit_pitch.get();
                Pitch_Axis.PID1_out_scale = _pid1_output_scale_pitch.get();
                Pitch_Axis.PID2_P_gain = _pid2_gain_pitch_p.get();
                Pitch_Axis.PID2_I_gain = _pid2_gain_pitch_i.get();
                Pitch_Axis.PID2_D_gain = _pid2_gain_pitch_d.get();
                Pitch_Axis.PID2_I_limit = _pid2_i_limit_pitch.get();
                Pitch_Axis.PID2_out_limit = _pid2_output_limit.get();
                Pitch_Axis.PID2_out_scale = _pid2_output_scale_pitch.get();

                Roll_Axis.PID1_P_gain = _pid1_gain_roll_p.get();
                Roll_Axis.PID1_I_gain = _pid1_gain_roll_i.get();
                Roll_Axis.PID1_D_gain = _pid1_gain_roll_d.get();
                Roll_Axis.PID1_I_limit = _pid1_i_limit_roll.get();
                Roll_Axis.PID1_out_limit = _pid1_output_limit_roll.get();
                Roll_Axis.PID1_out_scale = _pid1_output_scale_roll.get();
                Roll_Axis.PID2_P_gain = _pid2_gain_roll_p.get();
                Roll_Axis.PID2_I_gain = _pid2_gain_roll_i.get();
                Roll_Axis.PID2_D_gain = _pid2_gain_roll_d.get();
                Roll_Axis.PID2_I_limit = _pid2_i_limit_roll.get();
                Roll_Axis.PID2_out_limit = _pid2_output_limit.get();
                Roll_Axis.PID2_out_scale = _pid2_output_scale_roll.get();

                Yaw_Axis.PID1_P_gain = _pid1_gain_yaw_p.get();
                Yaw_Axis.PID1_I_gain = _pid1_gain_yaw_i.get();
                Yaw_Axis.PID1_D_gain = _pid1_gain_yaw_d.get();
                Yaw_Axis.PID1_I_limit = _pid1_i_limit_yaw.get();
                Yaw_Axis.PID1_out_limit = _pid1_output_limit_yaw.get();
                Yaw_Axis.PID1_out_scale = _pid1_output_scale_yaw.get();
                Yaw_Axis.PID2_P_gain = _pid2_gain_yaw_p.get();
                Yaw_Axis.PID2_I_gain = _pid2_gain_yaw_i.get();
                Yaw_Axis.PID2_D_gain = _pid2_gain_yaw_d.get();
                Yaw_Axis.PID2_I_limit = _pid2_i_limit_yaw.get();
                Yaw_Axis.PID2_out_limit = _pid2_output_limit.get();
                Yaw_Axis.PID2_out_scale = _pid2_output_scale_yaw.get();
        }

        _drone_task_sub.update(&_drone_task); // current mode of the drone
        _vehicle_local_position_sub.update(&_vehicle_local_position); // x, y, z position and velocity
        _vehicle_attitude_sub.update(&_vehicle_attitude); // roll, pitch, yaw attitude
        _vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity); // vehicle setpoint for position and attitude

        matrix::Quatf current_attitude_quat(_vehicle_attitude.q);
        matrix::Eulerf current_attitude(current_attitude_quat.dcm_z());

        switch (_drone_task.task)
        {
        case _drone_task.TASK_REMOTECONTROLLED: // RAW Remote Control
        case _drone_task.TASK_BUOYANCYCRTL:     // Buoyancy Control
                // Task: Disable all position control
                configure_axis(X_Axis, PID_MODE_DISABLED);
                configure_axis(Y_Axis, PID_MODE_DISABLED);
                configure_axis(Z_Axis, PID_MODE_DISABLED);
                configure_axis(Roll_Axis, PID_MODE_DISABLED);
                configure_axis(Pitch_Axis, PID_MODE_DISABLED);
                configure_axis(Yaw_Axis, PID_MODE_DISABLED);

                break;

        case _drone_task.TASK_SEARCHBUOY: // Search Buoy Algo
        case _drone_task.TASK_SEARCHTUBE: // Search Tube Algo
                // setup all axis for position control
                // keep attitude setpoints at 0

                configure_axis(X_Axis, PID_MODE_POSITION, true, &_vehicle_local_position.x, &_trajectory_setpoint.position[0]);
                configure_axis(Y_Axis, PID_MODE_POSITION, true, &_vehicle_local_position.y, &_trajectory_setpoint.position[1]);
                configure_axis(Z_Axis, PID_MODE_POSITION, true, &_vehicle_local_position.z, &_trajectory_setpoint.position[2]);
                configure_axis(Roll_Axis, PID_MODE_POSITION, true, &current_attitude.phi(), &zero);
                configure_axis(Pitch_Axis, PID_MODE_POSITION, true, &current_attitude.theta(), &zero);
                configure_axis(Yaw_Axis, PID_MODE_POSITION, true, &current_attitude.psi(), &zero);
                break;

        case _drone_task.TASK_DPTELEARM: // Remote Controlled arm
        case _drone_task.TASK_DPGOAL:    // Remote Controlled Positioning
        case _drone_task.TASK_TASK2:     // default initialisation
        case _drone_task.TASK_TASK1:

        default: // in default state, keep position.

                // the setpoint is not used, so we can use zero
                configure_axis(X_Axis, PID_MODE_HOLD_POSITION, false, &_vehicle_local_position.x, &zero);
                configure_axis(Y_Axis, PID_MODE_HOLD_POSITION, false, &_vehicle_local_position.y, &zero);
                configure_axis(Z_Axis, PID_MODE_HOLD_POSITION, false, &_vehicle_local_position.z, &zero);
                configure_axis(Roll_Axis, PID_MODE_HOLD_POSITION, false, &current_attitude.phi(), &zero);
                configure_axis(Pitch_Axis, PID_MODE_HOLD_POSITION, false, &current_attitude.theta(), &zero);
                configure_axis(Yaw_Axis, PID_MODE_HOLD_POSITION, false, &current_attitude.psi(), &zero);
                break;
        }

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
            1000000 / _pid_frequency.get()); // Schedule next run at the desired frequency

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
        if(axis.PID_mode <= PID_MODE_DISABLED || axis.PID_mode > PID_MODE_HOLD_VELOCITY)
        {
                // if axis is in an unknown mode, do not run PID control
                axis.pid_output = 0.0f;
                axis.PID_mode = PID_MODE_DISABLED;
                return;
        }

        // dont use build in output limit, as we first want to scale the output
        axis.PID1.setOutputLimit(axis.PID1_out_limit);
        axis.PID1.setIntegralLimit(axis.PID1_I_limit);
        axis.PID1.setGains(axis.PID1_P_gain, axis.PID1_I_gain, axis.PID1_D_gain);

        axis.PID2.setOutputLimit(axis.PID2_out_limit);
        axis.PID2.setIntegralLimit(axis.PID2_I_limit);
        axis.PID2.setGains(axis.PID2_P_gain, axis.PID2_I_gain, axis.PID2_D_gain);

        if(!(axis.PID_mode == PID_MODE_HOLD_POSITION || axis.PID_mode == PID_MODE_HOLD_VELOCITY))
        {
                // only update the setpoint if we are not in hold mode
                axis._setpoint = *axis.setpoint_ptr;
        }

        // calculate delta time in seconds
        float delta_time = hrt_elapsed_time(&axis.last_run_time) / 1e6f;
        axis.last_run_time = hrt_absolute_time(); // update last run time

        // run first PID
        if(axis.PID_mode == PID_MODE_POSITION || axis.PID_mode == PID_MODE_HOLD_POSITION)
        {
                // set the setpoint
                axis.PID1.setSetpoint(axis._setpoint);

                // update the position PID controller with the current feedback
                float PID1_output = axis.PID1.update(*axis.feedback_ptr, delta_time, true);

                // scale and limit the output
                PID1_output = scale_and_limit(PID1_output, axis.PID1_out_scale, axis.PID1_out_limit);

                axis.PID2.setSetpoint(PID1_output); // set the output of the position PID as setpoint for the velocity PID
        }
        else
        {
                // if we are in velocity mode, we can directly set the setpoint for the velocity PID
                axis.PID2.setSetpoint(axis._setpoint);
        }

        // update the velocity PID controller with the current feedback
        float PID2_output = axis.PID2.update(*axis.feedback_ptr, delta_time, true);
        axis.pid_output = scale_and_limit(PID2_output, axis.PID2_out_scale, axis.PID2_out_limit);


}

float RobosubPosControl::scale_and_limit(float value, float scale, float limit)
{
        // scale the value
        value *= scale;

        // constrain the value to the limit
        return math::constrain(value, -limit, limit);
}


/**
 * @brief Configure an axis for PID control
 */
void RobosubPosControl::configure_axis(AxisPID_s &axis, PIDMode_e mode, bool reset_on_change, float* feedback, float* setpoint)
{
        bool reset = false;

        if((axis.PID_mode        != mode         ||
           axis.feedback_ptr    != feedback     ||
           axis.setpoint_ptr    != setpoint     ) &&
           reset_on_change)
        {
                // if anything changed, reset the PID controllers if reset_on_change is true
                reset = true;
        }

        // load data into the axis
        axis.PID_mode = mode;
        axis.feedback_ptr = feedback;
        axis.setpoint_ptr = setpoint;

        if(reset)
        {
        // reset the PID controllers if config changed
        axis.PID1.resetDerivative();
        axis.PID1.resetIntegral();

        axis.PID2.resetDerivative();
        axis.PID2.resetIntegral();
        }
}


