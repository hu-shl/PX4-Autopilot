px4_add_module(
	MODULE modules__robosub__rs_canfd_status
	MAIN rs_canfd_status
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_canfd_status.cpp
)
