#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>

#include <px4_platform_common/getopt.h>
// #include <drivers/drv_canfd.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>


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

 #include "rs_canfd_status.hpp"

 #include <px4_platform_common/getopt.h>
 #include <px4_platform_common/log.h>
 #include <px4_platform_common/posix.h>

 #include <uORB/topics/parameter_update.h>
 #include <uORB/topics/sensor_combined.h>

 extern "C" __EXPORT int rs_canfd_status_main(int argc, char *argv[]);


 int RobosubCanfdStatus::print_status()
 {
	 PX4_INFO_RAW("Running");
	 // TODO: print additional runtime information about the state of the module

	 return 0;
 }

 int RobosubCanfdStatus::custom_command(int argc, char *argv[])
 {
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


 int RobosubCanfdStatus::task_spawn(int argc, char *argv[])
 {
	 _task_id = px4_task_spawn_cmd("module",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT,
				       1024,
				       (px4_main_t)&run_trampoline,
				       (char *const *)argv);

	 if (_task_id < 0) {
		 _task_id = -1;
		 return -errno;
	 }

	 return 0;
 }

 RobosubCanfdStatus *RobosubCanfdStatus::instantiate(int argc, char *argv[])
 {
	 int example_param = 0;
	 bool example_flag = false;
	 bool error_flag = false;

	 int myoptind = 1;
	 int ch;
	 const char *myoptarg = nullptr;

	 // parse CLI arguments
	 while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		 switch (ch) {
		 case 'p':
			 example_param = (int)strtol(myoptarg, nullptr, 10);
			 break;

		 case 'f':
			 example_flag = true;
			 break;

		 case '?':
			 error_flag = true;
			 break;

		 default:
			 PX4_WARN("unrecognized flag");
			 error_flag = true;
			 break;
		 }
	 }

	 if (error_flag) {
		 return nullptr;
	 }

	 RobosubCanfdStatus *instance = new RobosubCanfdStatus(example_param, example_flag);

	 if (instance == nullptr) {
		 PX4_ERR("alloc failed");
	 }

	 return instance;
 }

 RobosubCanfdStatus::RobosubCanfdStatus(int example_param, bool example_flag)
	 : ModuleParams(nullptr)
 {
 }

 void RobosubCanfdStatus::run()
 {
	 // Example: run the loop synchronized to the sensor_combined topic publication
	 // int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	// if (sock < 0) {
	// 	PX4_ERR("CAN socket open failed");
	// 	return -1;
	// }

	struct ifreq ifr {};
	strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

	// if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
	// 	PX4_ERR("SIOCGIFINDEX failed");
	// 	close(sock);
	// 	return -1;
	// }

	// Bitrate is not available through socket, but you can log it manually if known
	printf("CAN FD Interface: can0\n");
	printf("Nominal Bitrate: 500000 (example)\n");
	printf("Data Bitrate: 2000000 (example)\n");

	// Loopback status
	int loopback = 0;
	// socklen_t optlen = sizeof(loopback);
	// getsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, &optlen);
	printf("Loopback mode: %s\n", loopback ? "ENABLED" : "DISABLED");

	int recv_own_msgs = 0;
	// getsockopt(sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, &optlen);
	printf("Receive own messages: %s\n", recv_own_msgs ? "YES" : "NO");

	// close(sock);
 }

 void RobosubCanfdStatus::parameters_update(bool force)
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

 int RobosubCanfdStatus::print_usage(const char *reason)
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

	 PRINT_MODULE_USAGE_NAME("module", "rs arm control");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	 PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	 return 0;
 }

 int rs_canfd_status_main(int argc, char *argv[])
 {
	 return RobosubCanfdStatus::main(argc, argv);
 }
