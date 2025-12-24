import sys
if sys.prefix == '/home/salamut/asv_2025/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/salamut/asv_2025/ros_yolo/install/mav_interface'
