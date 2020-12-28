CMAKE_MINIMUM_REQUIRED(VERSION 3.10.2)
PROJECT(abb_egm_example)



# DEPENDENCIES ###########################################################################
# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS regex system thread)

# Google protobuffer
FIND_PACKAGE(Protobuf REQUIRED)

# Require c++ standard
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)



# CONFIGS ################################################################################
# Add warnings
# ADD_COMPILE_OPTIONS("-Wall")
# ADD_COMPILE_OPTIONS("-Wextra")
# ADD_COMPILE_OPTIONS("-Wpedantic")
# ADD_COMPILE_OPTIONS("-Werror")

SET(LIBRERIES
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    /workspaces/abb_libegm/build/libabb_libegm.so
)
SET(INCLUDES
    ${Boost_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    /workspaces/abb_libegm/include/
    /workspaces/abb_libegm/build/
    ${PROJECT_SOURCE_DIR}/include
)
SET(SOURCES
    ${PROJECT_SOURCE_DIR}/src/simple_interface/simple_interface.cpp
)



# BUILDS #################################################################################
# Build Examples
IF (BUILD_EXAMPLES STREQUAL "true" OR BUILD_EXAMPLES STREQUAL "TRUE")
    SET(TYPES_NAME 
        "joint" 
        "pose"
    )
    SET(FILES_NAME 
        "controller" 
        "static_goal" 
        "trajectory" 
        "velocity_controller"
    )
    FOREACH(TYPE ${TYPES_NAME})
        FOREACH(NAME ${FILES_NAME})
            ADD_EXECUTABLE(${TYPE}_${NAME} ${PROJECT_SOURCE_DIR}/abb_examples/${TYPE}_${NAME}.cpp)
            TARGET_INCLUDE_DIRECTORIES(${TYPE}_${NAME} PRIVATE ${INCLUDES})
            TARGET_LINK_LIBRARIES(${TYPE}_${NAME} ${LIBRERIES})
        ENDFOREACH()
    ENDFOREACH()
ENDIF()


# Build Sources
SET(FILES_NAME 
    # "raw_controller" 
    # "simple_controller" 
    # "keyboard_controller"
    "test_controller"
    "my_test"
)
FOREACH(NAME ${FILES_NAME})
    ADD_EXECUTABLE(${NAME} ${SOURCES} ${PROJECT_SOURCE_DIR}/src/${NAME}.cpp)
    TARGET_INCLUDE_DIRECTORIES(${NAME} PRIVATE ${INCLUDES})
    TARGET_LINK_LIBRARIES(${NAME} ${LIBRERIES})
ENDFOREACH()



# COMMANDS ###############################################################################
# Clean
ADD_CUSTOM_TARGET(rm
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/build/*
    COMMENT "Clean build directory"
)