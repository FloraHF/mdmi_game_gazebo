import rospy
from pyllist import dllist
from mdmi_game.srv import DroneStatus, DroneStatusResponse

a = {'I0':1}
a_sort = dllist([[k, v] for k, v in sorted(a.items(), key=lambda x: x[1])])
print(a_sort)

