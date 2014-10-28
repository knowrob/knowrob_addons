cmake_minimum_required(VERSION 2.8.3)
project(comp_mocap)

install(PROGRAMS
    scripts/mocap_marker.py
    scripts/tf_restamper.py
    scripts/tf_dynamic_transform.py
    scripts/csv_tf_publisher.py
    scripts/xsens_tf_broadcaster.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

