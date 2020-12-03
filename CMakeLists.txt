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
# string(TOLOWER "main" BIN_NAME)
# add_executable(${BIN_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp)
# target_include_directories(${BIN_NAME} PRIVATE ${INCLUDES})
# target_link_libraries(${BIN_NAME} ${LIBRERIES})

string(TOLOWER "trajectory_joint" BIN_NAME)
add_executable(${BIN_NAME} ${PROJECT_SOURCE_DIR}/src/trajectory_joint.cpp)
target_include_directories(${BIN_NAME} PRIVATE ${INCLUDES})
target_link_libraries(${BIN_NAME} ${LIBRERIES})




##################################################################
# CLEAN COMMAND
add_custom_target(rm
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/build/*
    COMMENT "Clean build directory"
)