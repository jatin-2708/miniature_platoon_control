import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bayesi4n/iop/install/turtlebot3_lane_changing_leader'
