uint8 TASK_REMOTECONTROLLED 	= 0	# completely remote controlled
uint8 TASK_BUOYANCYCRTL 	= 1	# buoyancy tanks remote controlled
uint8 TASK_DPGOAL	 	= 2	# change the goal position
uint8 TASK_DPTELEARM 		= 3	# complete robot arm control
uint8 TASK_SEARCHBUOY 		= 4	# buoy search pattern
uint8 TASK_SEARCHTUBE		= 5	# tube search pattern
uint8 TASK_TASK2		= 6	# challenge B
uint8 TASK_TASK1 		= 7	# challenge A

uint64 timestamp

uint8 task 				# task that the drone is currently in. see TASK_*
