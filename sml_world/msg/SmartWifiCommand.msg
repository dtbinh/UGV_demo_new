# Description of command_id's:
# command_id = 1: Overtake. Result = 0 means unsafe to pass, otherwise it is safe
# command_id = 2:Set speed to result. msg is extra information
# command_id = 3: Finish overtaking.


int32 command_id
int32 result
string msg