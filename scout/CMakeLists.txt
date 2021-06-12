cmake_minimum_required(VERSION 2.8.3)
project(scout)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roslaunch
    message_generation
    std_msgs
)

# Install the python package
catkin_python_setup()

# Message generation
add_message_files(FILES
    Actuators.msg
    Calibration.msg
    CarControl.msg
    CarControlStamped.msg
)
generate_messages(DEPENDENCIES std_msgs)

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

# Install executables
catkin_install_python(PROGRAMS
                        nodes/controller_node
                        nodes/joystick_node
                        nodes/wall_follower_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY urdf/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY map/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map)
