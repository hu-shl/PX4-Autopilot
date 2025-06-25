# Local position setpoint in NED frame
# Used to transport information to the PID loop
# NaN means the state was not controlled

uint64 timestamp	# time since system start (microseconds)

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED

float32 pitch		# in radians NED -PI..+PI
float32 roll		# in radians NED -PI..+PI
float32 yaw		# in radians NED -PI..+PI

# TOPICS rs_auto_setpoint rs_manual_setpoint
