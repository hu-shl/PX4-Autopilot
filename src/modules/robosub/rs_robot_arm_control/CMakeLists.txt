px4_add_module(
	MODULE modules_rs_robot_arm_control
	MAIN rs_robot_arm_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	STACK_MAIN
		3000
	SRCS
		rs_robot_arm_control.cpp
		rs_robot_arm_control.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mathlib
		px4_work_queue
)
