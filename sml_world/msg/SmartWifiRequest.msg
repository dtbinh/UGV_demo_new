# This messenger will send any requests from the vehicle to
# the smart wifi module. It should be sent at frequent intervals
# so that the smart wifi module can update the vehicles trajectory

# Send -1 for no request
# 1 to see if overtaking is safe
# 2 to get the speed we should travel
# 3 to inform wifi module we are overtaking
# 4 we started overtaking
# 5 we finished overtaking, revert to original speed
# 6 Light that we were waiting at changed to green


Pose2D[] trajectory
int32 request
int32 desired_speed