CMAKE_MINIMUM_REQUIRED(VERSION 3.10.2)
PROJECT(abb_egm_example)


##################################################################
# boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS regex system thread)

# google protobuffer
FIND_PACKAGE(Protobuf REQUIRED)


##################################################################
# Require c++ standard
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)


# Add warnings
# ADD_COMPILE_OPTIONS("-Wall")
# ADD_COMPILE_OPTIONS("-Wextra")
# ADD_COMPILE_OPTIONS("-Wpedantic")
# ADD_COMPILE_OPTIONS("-Werror")


# FILES
SET(INCLUDES
    ${Boost_INCLUDE_DIR} 
    ${PROTOBUF_INCLUDE_DIRS}
    /workspaces/abb_libegm/include/
    /workspaces/abb_libegm/build/
)
SET(LIBRERIES
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    /workspaces/abb_libegm/build/libabb_libegm.so
)



##################################################################
# BUILD EXAMPLES
SET(BUILD_EXAMPLE TRUE)
IF(BUILD_EXAMPLE)
    SET(TYPE_FILE 
        "joint" 
        "pose"
    )
    SET(NAME_FILE 
        "controller" 
        "static_goal" 
        "trajectory" 
        "velocity_controller"
    )
    FOREACH(TYPE ${TYPE_FILE})
        FOREACH(NAME ${NAME_FILE})
            ADD_EXECUTABLE(${TYPE}_${NAME} ${PROJECT_SOURCE_DIR}/examples/${TYPE}_${NAME}.cpp)
            TARGET_INCLUDE_DIRECTORIES(${TYPE}_${NAME} PRIVATE ${INCLUDES})
            TARGET_LINK_LIBRARIES(${TYPE}_${NAME} ${LIBRERIES})
        ENDFOREACH()
    ENDFOREACH()
ENDIF()


# BUILD MAIN
STRING(TOLOWER "main" BIN_NAME)
ADD_EXECUTABLE(${BIN_NAME} ${PROJECT_SOURCE_DIR}/src/${BIN_NAME}.cpp)
TARGET_INCLUDE_DIRECTORIES(${BIN_NAME} PRIVATE ${INCLUDES})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${LIBRERIES})



##################################################################
# CLEAN COMMAND
ADD_CUSTOM_TARGET(rm
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/build/*
    COMMENT "Clean build directory"
)