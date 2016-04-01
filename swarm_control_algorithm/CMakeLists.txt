cmake_minimum_required(VERSION 2.8.3)
project(swarm_control_algorithm)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(swarm_control_algorithm src/swarm_control_algorithm.cpp)   

#cs_add_executable(swarm_control_algorithm_main src/swarm_control_algorithm_main.cpp)
#cs_add_executable(swarm_control_algorithm_main2 src/swarm_control_algorithm_main2.cpp)
cs_add_executable(swarm_control_algorithm_test src/swarm_control_algorithm_test.cpp)
#cs_add_executable(swarm_control_algorithm_test2 src/swarm_control_algorithm_test2.cpp)

# edit the arguments to reference the named node and named library within this package
#target_link_libraries(swarm_control_algorithm_main swarm_control_algorithm)
#target_link_libraries(swarm_control_algorithm_main2 swarm_control_algorithm)
target_link_libraries(swarm_control_algorithm_test swarm_control_algorithm)
#target_link_libraries(swarm_control_algorithm_test2 swarm_control_algorithm)

cs_install()
cs_export()
    