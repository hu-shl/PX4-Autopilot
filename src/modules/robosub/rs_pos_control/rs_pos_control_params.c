/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file rs_pos_control_params.c
 *
 * Parameters defined by the position control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Thijs Vader <Thijs.vader@student.hu.nl>
*/



// #### PID PARAMETERS FOR X AXIS ####
/**
 * P gain of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_P, 1.0f);
/**
 * I gain of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_I, 1.0f);
/**
 * D gain of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_D, 1.0f);
/**
 * I limit of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_X_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_X_P, 1.0f);
/**
 * I gain of PID controller 2 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_X_I, 1.0f);
/**
 * D gain of PID controller 2 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_X_D, 1.0f);
/**
 * I limit of PID controller 2 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_X_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 X-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_X_OP_SCL, 1.0f);





// #### PID PARAMETERS FOR Y AXIS ####
/**
 * P gain of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_P, 1.0f);
/**
 * I gain of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_I, 1.0f);
/**
 * D gain of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_D, 1.0f);
/**
 * I limit of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Y_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Y_P, 1.0f);
/**
 * I gain of PID controller 2 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Y_I, 1.0f);
/**
 * D gain of PID controller 2 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Y_D, 1.0f);
/**
 * I limit of PID controller 2 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Y_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 Y-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Y_OP_SCL, 1.0f);





// #### PID PARAMETERS FOR Z AXIS ####
/**
 * P gain of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_P, 1.0f);
/**
 * I gain of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_I, 1.0f);
/**
 * D gain of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_D, 1.0f);
/**
 * I limit of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_Z_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Z_P, 1.0f);
/**
 * I gain of PID controller 2 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Z_I, 1.0f);
/**
 * D gain of PID controller 2 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Z_D, 1.0f);
/**
 * I limit of PID controller 2 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Z_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 Z-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_Z_OP_SCL, 1.0f);





// #### PID PARAMETERS FOR PITCH AXIS ####
// to keep the names under 16 characters,
// we use A instead of Pitch
/**
 * P gain of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_P, 1.0f);
/**
 * I gain of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_I, 1.0f);
/**
 * D gain of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_D, 1.0f);
/**
 * I limit of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_A_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_A_P, 1.0f);
/**
 * I gain of PID controller 2 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_A_I, 1.0f);
/**
 * D gain of PID controller 2 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_A_D, 1.0f);
/**
 * I limit of PID controller 2 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_A_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 Pitch-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_A_OP_SCL, 1.0f);





// #### PID PARAMETERS FOR ROLL AXIS ####
// to keep the names under 16 characters,
// we use B instead of Roll
/**
 * P gain of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_P, 1.0f);
/**
 * I gain of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_I, 1.0f);
/**
 * D gain of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_D, 1.0f);
/**
 * I limit of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_B_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_B_P, 1.0f);
/**
 * I gain of PID controller 2 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_B_I, 1.0f);
/**
 * D gain of PID controller 2 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_B_D, 1.0f);
/**
 * I limit of PID controller 2 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_B_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 Roll-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_B_OP_SCL, 1.0f);





// #### PID PARAMETERS FOR YAW AXIS ####
// to keep the names under 16 characters,
// we use C instead of Yaw
/**
 * P gain of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_P, 1.0f);
/**
 * I gain of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_I, 1.0f);
/**
 * D gain of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_D, 1.0f);
/**
 * I limit of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_I_LIM, 1.0f);
/**
 * Output limit of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_OP_LIM, 1.0f);
/**
 * Output scaling of PID controller 1 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID1_C_OP_SCL, 1.0f);
/**
 * P gain of PID controller 2 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_C_P, 1.0f);
/**
 * I gain of PID controller 2 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_C_I, 1.0f);
/**
 * D gain of PID controller 2 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_C_D, 1.0f);
/**
 * I limit of PID controller 2 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_C_I_LIM, 1.0f);
/**
 * Output scaling of PID controller 2 Yaw-axis
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_C_OP_SCL, 1.0f);





// #### PID SETTINGS ####
/**
 * Global Output limit for PID 2 controllers
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID2_OP_LIM, 1.0f);
/**
 * Frequency of the position control loop (Hz)
 *
 *  @group RS Position Control
 */
PARAM_DEFINE_INT32(RS_POS_CTRL_FREQ, 5);
/**
 * Force the parameters to be updated
 *
 *  @group RS Position Control
 */
PARAM_DEFINE_INT32(RS_FORCE_PARAMS, 0);
