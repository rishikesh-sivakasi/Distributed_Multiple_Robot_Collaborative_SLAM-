import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arz-1029/Desktop/Rishikesh_Evaluvation/task_ws/install/multi_robot_explore'
