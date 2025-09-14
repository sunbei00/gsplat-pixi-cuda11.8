import sys
if sys.prefix == '/root/code/.pixi/envs/ros':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/code/ros2_ws/install/GSImageSampler'
