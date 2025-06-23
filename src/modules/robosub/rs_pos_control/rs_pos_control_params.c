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



// #### PID GAIN PARAMETERS FOR X AXIS ####

/**
 * Gain of P controller X
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_X_P, 1.0f);
/**
 * Gain of PID P controller X
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_X_P, 1.0f);
/**
 * Gain of PID I controller X
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_X_I, 1.0f);
/**
 * Gain of PID D controller X
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_X_D, 1.0f);



// #### PID GAIN PARAMETERS FOR Y AXIS ####
/**
 * Gain of P controller Y
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_Y_P, 1.0f);
/**
 * Gain of PID P controller Y
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Y_P, 1.0f);
/**
 * Gain of PID I controller Y
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Y_I, 1.0f);
/**
 * Gain of PID D controller Y
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Y_D, 1.0f);



// #### PID GAIN PARAMETERS FOR Z AXIS ####
/**
 * Gain of P controller Z
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_Z_P, 1.0f);
/**
 * Gain of PID P controller Z
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Z_P, 1.0f);
/**
 * Gain of PID I controller Z
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Z_I, 1.0f);
/**
 * Gain of PID D controller Z
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_Z_D, 1.0f);



// #### PID GAIN PARAMETERS FOR ROLL ####
/**
 * Gain of P controller Roll
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_ROLL_P, 1.0f);
/**
 * Gain of PID P controller Roll
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_ROLL_P, 1.0f);
/**
 * Gain of PID I controller Roll
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_ROLL_I, 1.0f);
/**
 * Gain of PID D controller Roll
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_ROLL_D, 1.0f);


// #### PID GAIN PARAMETERS FOR PITCH ####
/**
 * Gain of P controller Pitch
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_PITCH_P, 1.0f);
/**
 * Gain of PID P controller Pitch
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_PITCH_P, 1.0f);
/**
 * Gain of PID I controller Pitch
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_PITCH_I, 1.0f);
/**
 * Gain of PID D controller Pitch
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_PITCH_D, 1.0f);



// #### PID GAIN PARAMETERS FOR YAW ####
/**
 * Gain of P controller Yaw
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_YAW_P, 1.0f);
/**
 * Gain of PID P controller Yaw
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_YAW_P, 1.0f);
/**
 * Gain of PID I controller Yaw
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_YAW_I, 1.0f);
/**
 * Gain of PID D controller Yaw
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_YAW_D, 1.0f);



// #### PID SETTINGS ####
/**
 * Integral limit for the position control loop
 *
 * @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_P_OUT_LIM, 100.0f);
/**
 * Output limit for the position control loop
 *
 *  @group RS Position Control
 */
PARAM_DEFINE_FLOAT(RS_PID_OUT_LIM, 1.0f);
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
