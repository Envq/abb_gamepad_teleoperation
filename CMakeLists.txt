cmake_minimum_required(VERSION 3.10.2)
project(abb_egm_example)


##################################################################
# boost
find_package(Boost REQUIRED COMPONENTS regex system thread)

# google protobuffer
find_package(Protobuf REQUIRED)


##################################################################
# Require c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Add warnings
# add_compile_options("-Wall")
# add_compile_options("-Wextra")
# add_compile_options("-Wpedantic")
# add_compile_options("-Werror")


# FILES
string(TOLOWER "example1" BIN_NAME)
set(INCLUDES
    ${Boost_INCLUDE_DIR} 
    ${PROTOBUF_INCLUDE_DIRS}
    /workspaces/abb_libegm/include/
    /workspaces/abb_libegm/build/
)
set(LIBRERIES
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    /workspaces/abb_libegm/build/libabb_libegm.so
)



##################################################################
# BUILD EXECUTABLE
add_executable(${BIN_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)
target_include_directories(${BIN_NAME} PRIVATE ${INCLUDES})
target_link_libraries(${BIN_NAME} ${LIBRERIES})

#add_executable(${PROJECT_NAME}_joint_trajectory_node src/a1_joint_trajectory_node.cpp)
#set_target_properties(${PROJECT_NAME}_joint_trajectory_node PROPERTIES OUTPUT_NAME a1_joint_trajectory_node PREFIX "")
#add_dependencies(${PROJECT_NAME}_joint_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_joint_trajectory_node ${catkin_LIBRARIES})




##################################################################
# CLEAN COMMAND
add_custom_target(rm
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/build/*
    COMMENT "Clean build directory"
)