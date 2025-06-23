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
#include <lib/mathlib/mathlib.h>   // Math Library
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
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/vehicle_position_attitude_setpoint.h> // vehicle position and attitude setpoint subscription
#include <uORB/topics/drone_task.h>                         // drone task subscription
#include <uORB/topics/vehicle_local_position.h>             // local position subscription
#include <uORB/topics/vehicle_attitude.h>                   // vehicle attitude subscription

#include <uORB/topics/vehicle_thrust_setpoint.h> // vehicle thrust setpoint publication
#include <uORB/topics/vehicle_torque_setpoint.h> // vehicle torque setpoint publication

#include <uORB/topics/trajectory_setpoint.h> // vehicle trajectory setpoint publication
#include <uORB/topics/water_detection.h>

#include <uORB/uORB.h>

// PX4 LOG
#include <px4_platform_common/log.h>

// ENTRY POINT
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);

// NAMESPACES
using uORB::SubscriptionData;

using namespace time_literals;

typedef enum inputType {
        INPUT_TYPE_POSITION = 0, // manual input
        INPUT_TYPE_VELOCITY = 1, // auto input
} inputType_e;

typedef struct AxisPID {
        float position_setpoint;
        float velocity_setpoint;

        float position_current;
        float velocity_current;

        float P_gain;
        float PID_P_gain;
        float PID_I_gain;
        float PID_D_gain;

        float velocity_constraint;
        float thrust_constraint;

        uint64_t last_run_time; // last run time in microseconds
        float delta_time;

        float pid_output;
        inputType_e input_type;

        PID PID_controller;
} AxisPID_s;

// MAIN CLASS
class RobosubPosControl : public ModuleBase<RobosubPosControl>, public ModuleParams, public px4::ScheduledWorkItem {
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
        uORB::Subscription _manual_control_setpoint_sub{
            ORB_ID(rs_manual_setpoint)}; // manual control setpoint subscription

        uORB::Subscription _auto_control_setpoint_sub{ORB_ID(rs_auto_setpoint)}; // auto control setpoint subscription

        uORB::Subscription _drone_task_sub{ORB_ID(drone_task)}; // drone task subscription

        uORB::Subscription _vehicle_local_position_sub{
            ORB_ID(vehicle_local_position)}; // vehicle local position subscription

        uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)}; // vehicle attitude subscription

        uORB::Subscription _water_detection_sub{
            ORB_ID(water_detection)}; // Mainbrain and power exeterior water dectection sensor outside */

        uORB::Subscription _trajectory_setpoint_sub{
            ORB_ID(trajectory_setpoint)}; // vehicle trajectory setpoint subscription

        uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                         1_s}; // parameter update subscription

        // uORB publications
        uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{
            ORB_ID(vehicle_thrust_setpoint)}; // vehicle thrust setpoint

        uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{
            ORB_ID(vehicle_torque_setpoint)}; // vehicle torque setpoint

        perf_counter_t _loop_perf;

        // Define subscription variables
        vehicle_position_attitude_setpoint_s _vehicle_setpoint{}; // manual control setpoint
        drone_task_s _drone_task{};                               // drone task
        vehicle_local_position_s _vehicle_local_position{};       // vehicle local position
        vehicle_attitude_s _vehicle_attitude{};                   // vehicle attitude
        trajectory_setpoint_s _trajectory_setpoint{};             // vehicle trajectory setpoint

        water_detection_s _water_detection{}; // water detection sensor

        // Define publication variables
        vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{}; // vehicle thrust setpoint
        vehicle_torque_setpoint_s _vehicle_torque_setpoint{}; // vehicle torque setpoint

        DEFINE_PARAMETERS((ParamFloat<px4::params::RS_P_X_P>)_p_gain_p_x, // PID gains for X axis
                          (ParamFloat<px4::params::RS_PID_X_P>)_pid_gain_p_x,
                          (ParamFloat<px4::params::RS_PID_X_I>)_pid_gain_i_x,
                          (ParamFloat<px4::params::RS_PID_X_D>)_pid_gain_d_x,

                          (ParamFloat<px4::params::RS_P_Y_P>)_p_gain_p_y, // PID gains for Y axis
                          (ParamFloat<px4::params::RS_PID_Y_P>)_pid_gain_p_y,
                          (ParamFloat<px4::params::RS_PID_Y_I>)_pid_gain_i_y,
                          (ParamFloat<px4::params::RS_PID_Y_D>)_pid_gain_d_y,

                          (ParamFloat<px4::params::RS_P_Z_P>)_p_gain_p_z, // PID gains for Z axis
                          (ParamFloat<px4::params::RS_PID_Z_P>)_pid_gain_p_z,
                          (ParamFloat<px4::params::RS_PID_Z_I>)_pid_gain_i_z,
                          (ParamFloat<px4::params::RS_PID_Z_D>)_pid_gain_d_z,

                          (ParamFloat<px4::params::RS_P_ROLL_P>)_p_gain_p_roll, // PID gains for Roll axis
                          (ParamFloat<px4::params::RS_PID_ROLL_P>)_pid_gain_p_roll,
                          (ParamFloat<px4::params::RS_PID_ROLL_I>)_pid_gain_i_roll,
                          (ParamFloat<px4::params::RS_PID_ROLL_D>)_pid_gain_d_roll,

                          (ParamFloat<px4::params::RS_P_PITCH_P>)_p_gain_p_pitch, // PID gains for Pitch axis
                          (ParamFloat<px4::params::RS_PID_PITCH_P>)_pid_gain_p_pitch,
                          (ParamFloat<px4::params::RS_PID_PITCH_I>)_pid_gain_i_pitch,
                          (ParamFloat<px4::params::RS_PID_PITCH_D>)_pid_gain_d_pitch,

                          (ParamFloat<px4::params::RS_P_YAW_P>)_p_gain_p_yaw, // PID gains for Yaw axis
                          (ParamFloat<px4::params::RS_PID_YAW_P>)_pid_gain_p_yaw,
                          (ParamFloat<px4::params::RS_PID_YAW_I>)_pid_gain_i_yaw,
                          (ParamFloat<px4::params::RS_PID_YAW_D>)_pid_gain_d_yaw,

                          (ParamInt<px4::params::RS_POS_CTRL_FREQ>)_pos_control_freq, // control frequency
                          (ParamInt<px4::params::RS_FORCE_PARAMS>)_force_param,       // control frequency
                          (ParamFloat<px4::params::RS_P_OUT_LIM>)_p_output_limit,     // p output limit
                          (ParamFloat<px4::params::RS_PID_OUT_LIM>)_pid_output_limit  // pid output limit
        );

        /**
         * @brief In-/underwater actuator safety factor
         */
        void apply_water_safety(float &roll_u, float &pitch_u, float &yaw_u, float &thrust_x, float &thrust_y,
                                float &thrust_z);
        // publication methods
        void constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_x, float thrust_y,
                                         float thrust_z);

        // paramter update methods
        int parameters_update(bool force = false);

        void run_axis_pid(AxisPID_s &axis);
};
