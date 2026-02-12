import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jorge/Documentos/GitHub/RobotsAereosYSubmarinos/proyecto_drone/ros2_ws/install/drone_project'
