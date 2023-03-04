#!/bin/sh
rosbag record --tcpnodelay --repeat-latched -O $1 /SwerveDiagnostics /static_shapes /tf /tf_static /MotorStatus
