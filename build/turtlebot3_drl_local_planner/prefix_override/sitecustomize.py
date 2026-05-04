import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hug/tb3_sim_ws/install/turtlebot3_drl_local_planner'
