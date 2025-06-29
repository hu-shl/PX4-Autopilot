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

// LIBRARY INCLUDES
#include <float.h>                 // floating point
#include <lib/pid/PID.hpp>         // PID Controller
#include <drivers/drv_hrt.h>       // High Resolution Timer
#include <lib/perf/perf_counter.h> // Performance Counters

#include <lib/mathlib/mathlib.h>     // Math Library
// #include <matrix/math.hpp>

// PX4 INCLUDES
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// PX4 UORB INCLUDES
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

// PX4 TOPICS
// parameters
#include <uORB/topics/parameter_update.h>

// Setpoints
#include <uORB/topics/trajectory_setpoint.h>        // setpoints from navigation module
// add other topics when other sources are needed

// Drone status
#include <uORB/topics/drone_task.h>                 // current drone task (search buoy, search tube, etc.)
#include <uORB/topics/status.h>                     // current status of drone (critical battery, low battery, etc.)

// Current data
#include <uORB/topics/vehicle_local_position.h>     // position and velocity for x, y and z
#include <uORB/topics/vehicle_attitude.h>           // position for roll, pitch and yaw
#include <uORB/topics/vehicle_angular_velocity.h>   // velocity for roll, pitch and yaw

// Publications
#include <uORB/topics/vehicle_thrust_setpoint.h>    // vehicle thrust setpoint publication (x, y and z)
#include <uORB/topics/vehicle_torque_setpoint.h>    // vehicle torque setpoint publication (roll, pitch and yaw)

#include <uORB/uORB.h>

// PX4 LOG
#include <px4_platform_common/log.h>

// ENTRY POINT
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);

// NAMESPACES
using uORB::SubscriptionData;

using namespace time_literals;

typedef enum PIDMode
{                          //                                               Switch 1    Switch 2
    PID_MODE_DISABLED = 0, // PID control is disabled                       DNC         L
    PID_MODE_POSITION = 1, // PID control is in position mode               L           H
    PID_MODE_VELOCITY = 2, // PID control is in velocity mode               H           H
    PID_MODE_HOLD_POSITION = 3, // PID control is in hold position mode     L           H
    PID_MODE_HOLD_VELOCITY = 4, // PID control is in hold velocity mode     H           H
} PIDMode_e;

typedef struct AxisPID
{
    // use pointers so that we dont have to copy the data
    // setpoint
    float* setpoint_ptr;
    float _setpoint; // hold setpoint for position or velocity

    // current data
    float* feedback_ptr;


    // Position PID
    float PID1_P_gain;          // PID gains
    float PID1_I_gain;
    float PID1_D_gain;
    float PID1_I_limit;
    float PID1_out_limit;       // output gains and limits
    float PID1_out_scale;

    // Velocity PID
    float PID2_P_gain;          // PID gains
    float PID2_I_gain;
    float PID2_D_gain;
    float PID2_I_limit;
    float PID2_out_limit;       // output gains and limits
    float PID2_out_scale;

    float pid_output;           // PID output value

    uint64_t last_run_time;     // last time the PID was run

    PIDMode_e PID_mode;         // PID Mode

    PID PID1;
    PID PID2;                  // PID controllers for position and velocity
} AxisPID_s;

// MAIN CLASS
class RobosubPosControl : public ModuleBase<RobosubPosControl>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
      public:
        RobosubPosControl();
        ~RobosubPosControl();

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);

        static int custom_command(int argc, char *argv[]);

        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        bool init();

        void Run() override;

      private:
        AxisPID_s X_Axis;
        AxisPID_s Y_Axis;
        AxisPID_s Z_Axis;
        AxisPID_s Roll_Axis;
        AxisPID_s Pitch_Axis;
        AxisPID_s Yaw_Axis;

        // uORB subscriptions
        uORB::Subscription _drone_task_sub{
            ORB_ID(drone_task)};                                // drone task subscription

        uORB::Subscription _status_sub{
            ORB_ID(status)};                                   // status subscription

        uORB::Subscription _vehicle_local_position_sub{
            ORB_ID(vehicle_local_position)};                           // vehicle local position subscription

        uORB::Subscription _vehicle_attitude_sub{
            ORB_ID(vehicle_attitude)};                          // vehicle attitude subscription

        uORB::Subscription _vehicle_angular_velocity_sub{
            ORB_ID(vehicle_angular_velocity)};                  // vehicle angular velocity subscription

        uORB::Subscription _trajectory_setpoint_sub{
            ORB_ID(trajectory_setpoint)};                       // vehicle trajectory setpoint subscription

        uORB::SubscriptionInterval _parameter_update_sub{
            ORB_ID(parameter_update), 1_s};                     // parameter update subscription

        // uORB publications
        uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{
            ORB_ID(vehicle_thrust_setpoint)}; // vehicle thrust setpoint

        uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{
            ORB_ID(vehicle_torque_setpoint)}; // vehicle torque setpoint

        perf_counter_t _loop_perf;

        drone_task_s
            _drone_task{};   // drone task
        status_s
            _status{};       // status

        trajectory_setpoint_s
            _trajectory_setpoint{}; // vehicle trajectory setpoint
        vehicle_local_position_s
            _vehicle_local_position{}; // vehicle local position
        vehicle_attitude_s
            _vehicle_attitude{}; // vehicle attitude
        vehicle_angular_velocity_s
            _vehicle_angular_velocity{}; // vehicle angular velocity


        // Define publication variables
        vehicle_thrust_setpoint_s
            _vehicle_thrust_setpoint{}; // vehicle thrust setpoint
        vehicle_torque_setpoint_s
            _vehicle_torque_setpoint{}; // vehicle torque setpoint

        DEFINE_PARAMETERS(
            (ParamFloat<px4::params::RS_PID1_X_P>)_pid1_gain_x_p, // PID gains for X axis
            (ParamFloat<px4::params::RS_PID1_X_I>)_pid1_gain_x_i,
            (ParamFloat<px4::params::RS_PID1_X_D>)_pid1_gain_x_d,
            (ParamFloat<px4::params::RS_PID1_X_I_LIM>)_pid1_i_limit_x,
            (ParamFloat<px4::params::RS_PID1_X_OP_LIM>)_pid1_output_limit_x, // PID output limit for X axis
            (ParamFloat<px4::params::RS_PID1_X_OP_SCL>)_pid1_output_scale_x, // PID output scale for X axis
            (ParamFloat<px4::params::RS_PID2_X_P>)_pid2_gain_x_p, // PID gains for X axis
            (ParamFloat<px4::params::RS_PID2_X_I>)_pid2_gain_x_i,
            (ParamFloat<px4::params::RS_PID2_X_D>)_pid2_gain_x_d,
            (ParamFloat<px4::params::RS_PID2_X_I_LIM>)_pid2_i_limit_x,
            (ParamFloat<px4::params::RS_PID2_X_OP_SCL>)_pid2_output_scale_x, // PID output scale for X axis

            (ParamFloat<px4::params::RS_PID1_Y_P>)_pid1_gain_y_p, // PID output limit for X axis
            (ParamFloat<px4::params::RS_PID1_Y_I>)_pid1_gain_y_i, // PID gains for Y axis
            (ParamFloat<px4::params::RS_PID1_Y_D>)_pid1_gain_y_d,
            (ParamFloat<px4::params::RS_PID1_Y_I_LIM>)_pid1_i_limit_y,
            (ParamFloat<px4::params::RS_PID1_Y_OP_LIM>)_pid1_output_limit_y, // PID output limit for Y axis
            (ParamFloat<px4::params::RS_PID1_Y_OP_SCL>)_pid1_output_scale_y, // PID output scale for Y axis
            (ParamFloat<px4::params::RS_PID2_Y_P>)_pid2_gain_y_p, // PID gains for Y axis
            (ParamFloat<px4::params::RS_PID2_Y_I>)_pid2_gain_y_i,
            (ParamFloat<px4::params::RS_PID2_Y_D>)_pid2_gain_y_d,
            (ParamFloat<px4::params::RS_PID2_Y_I_LIM>)_pid2_i_limit_y,
            (ParamFloat<px4::params::RS_PID2_Y_OP_SCL>)_pid2_output_scale_y, // PID output scale for Y axis

            (ParamFloat<px4::params::RS_PID1_Z_P>)_pid1_gain_z_p, // PID output limit for Y axis
            (ParamFloat<px4::params::RS_PID1_Z_I>)_pid1_gain_z_i, // PID gains for Z axis
            (ParamFloat<px4::params::RS_PID1_Z_D>)_pid1_gain_z_d,
            (ParamFloat<px4::params::RS_PID1_Z_I_LIM>)_pid1_i_limit_z,
            (ParamFloat<px4::params::RS_PID1_Z_OP_LIM>)_pid1_output_limit_z, // PID output limit for Z axis
            (ParamFloat<px4::params::RS_PID1_Z_OP_SCL>)_pid1_output_scale_z, // PID output scale for Z axis
            (ParamFloat<px4::params::RS_PID2_Z_P>)_pid2_gain_z_p, // PID gains for Z axis
            (ParamFloat<px4::params::RS_PID2_Z_I>)_pid2_gain_z_i,
            (ParamFloat<px4::params::RS_PID2_Z_D>)_pid2_gain_z_d,
            (ParamFloat<px4::params::RS_PID2_Z_I_LIM>)_pid2_i_limit_z,
            (ParamFloat<px4::params::RS_PID2_Z_OP_SCL>)_pid2_output_scale_z, // PID output scale for Z axis

            (ParamFloat<px4::params::RS_PID1_A_P>)_pid1_gain_pitch_p, // PID output limit for Z axis
            (ParamFloat<px4::params::RS_PID1_A_I>)_pid1_gain_pitch_i, // PID gains for Pitch axis
            (ParamFloat<px4::params::RS_PID1_A_D>)_pid1_gain_pitch_d,
            (ParamFloat<px4::params::RS_PID1_A_I_LIM>)_pid1_i_limit_pitch,
            (ParamFloat<px4::params::RS_PID1_A_OP_LIM>)_pid1_output_limit_pitch, // PID output limit for Pitch axis
            (ParamFloat<px4::params::RS_PID1_A_OP_SCL>)_pid1_output_scale_pitch, // PID output scale for Pitch axis
            (ParamFloat<px4::params::RS_PID2_A_P>)_pid2_gain_pitch_p, // PID gains for Pitch axis
            (ParamFloat<px4::params::RS_PID2_A_I>)_pid2_gain_pitch_i,
            (ParamFloat<px4::params::RS_PID2_A_D>)_pid2_gain_pitch_d,
            (ParamFloat<px4::params::RS_PID2_A_I_LIM>)_pid2_i_limit_pitch,
            (ParamFloat<px4::params::RS_PID2_A_OP_SCL>)_pid2_output_scale_pitch, // PID output scale for Pitch axis

            (ParamFloat<px4::params::RS_PID1_B_P>)_pid1_gain_roll_p, // PID output limit for Roll axis
            (ParamFloat<px4::params::RS_PID1_B_I>)_pid1_gain_roll_i, // PID gains for Roll axis
            (ParamFloat<px4::params::RS_PID1_B_D>)_pid1_gain_roll_d,
            (ParamFloat<px4::params::RS_PID1_B_I_LIM>)_pid1_i_limit_roll,
            (ParamFloat<px4::params::RS_PID1_B_OP_LIM>)_pid1_output_limit_roll, // PID output limit for Roll axis
            (ParamFloat<px4::params::RS_PID1_B_OP_SCL>)_pid1_output_scale_roll, // PID output scale for Roll axis
            (ParamFloat<px4::params::RS_PID2_B_P>)_pid2_gain_roll_p, // PID gains for Roll axis
            (ParamFloat<px4::params::RS_PID2_B_I>)_pid2_gain_roll_i,
            (ParamFloat<px4::params::RS_PID2_B_D>)_pid2_gain_roll_d,
            (ParamFloat<px4::params::RS_PID2_B_I_LIM>)_pid2_i_limit_roll,
            (ParamFloat<px4::params::RS_PID2_B_OP_SCL>)_pid2_output_scale_roll, // PID output scale for Roll axis

            (ParamFloat<px4::params::RS_PID1_C_P>)_pid1_gain_yaw_p, // PID output limit for Yaw axis
            (ParamFloat<px4::params::RS_PID1_C_I>)_pid1_gain_yaw_i, // PID gains for Yaw axis
            (ParamFloat<px4::params::RS_PID1_C_D>)_pid1_gain_yaw_d,
            (ParamFloat<px4::params::RS_PID1_C_I_LIM>)_pid1_i_limit_yaw,
            (ParamFloat<px4::params::RS_PID1_C_OP_LIM>)_pid1_output_limit_yaw, // PID output limit for Yaw axis
            (ParamFloat<px4::params::RS_PID1_C_OP_SCL>)_pid1_output_scale_yaw, // PID output scale for Yaw axis
            (ParamFloat<px4::params::RS_PID2_C_P>)_pid2_gain_yaw_p, // PID gains for Yaw axis
            (ParamFloat<px4::params::RS_PID2_C_I>)_pid2_gain_yaw_i,
            (ParamFloat<px4::params::RS_PID2_C_D>)_pid2_gain_yaw_d,
            (ParamFloat<px4::params::RS_PID2_C_I_LIM>)_pid2_i_limit_yaw,
            (ParamFloat<px4::params::RS_PID2_C_OP_SCL>)_pid2_output_scale_yaw, // PID output scale for Yaw axis

            (ParamFloat<px4::params::RS_PID2_OP_LIM>)_pid2_output_limit, // PID output limit for all axes
            (ParamInt<px4::params::RS_POS_CTRL_FREQ>)_pid_frequency, // force parameter update
            (ParamInt<px4::params::RS_FORCE_PARAMS>)_force_param // PID mode parameter
        );

        float zero = 0;

        // publication methods
        void publish_thrust_setpoint(const float thrust_x,
                                     const float thrust_y,
                                     const float thrust_z);

        void publish_torque_setpoint(const float torque_pitch,
                                     const float torque_roll,
                                     const float torque_yaw);

        // paramter update methods
        int parameters_update(bool force = false);


        void run_axis_pid(AxisPID_s &axis);

        float scale_and_limit(float val, float scale, float limit);



        void configure_axis(AxisPID_s &axis, PIDMode_e mode, bool reset_on_change = true, float* feedback = NULL, float* setpoint = NULL);
    };
