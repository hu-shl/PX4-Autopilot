px4_add_module(
	MODULE modules__robosub__rs_status_monitor
	MAIN rs_status_monitor
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_status_monitor.cpp
		rs_status_monitor.hpp
	MODULE_CONFIG
		rs_status_monitor.yaml
	DEPENDS
		px4_work_queue
	)
