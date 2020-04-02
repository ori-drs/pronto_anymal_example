cmake_minimum_required(VERSION 2.8.3)
project(pronto_anymal)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        pronto_ros
                                        pronto_quadruped_ros
                                        pronto_anymal_commons
                                        sensor_msgs)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME} src/pronto_anymal_node.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

