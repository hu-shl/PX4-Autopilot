
uint8 TASK_MISSIONPLANNING = 0
uint8 TASK_REINFORCED = 1
uint8 TASK_AUTONOMOUS = 2
uint8 TASK_TELEARM = 3
uint8 TASK_REMOTE_POSITION = 5
uint8 TASK_REMOTE_PROCESSED = 6
uint8 TASK_REMOTE_CONTROLLED = 7

uint64 timestamp			# time since system startup				(microseconds)

uint8 task 				# task that the drone is currently in. Tasks are set in rs_remote_control.cpp
