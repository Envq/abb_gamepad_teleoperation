CMAKE_MINIMUM_REQUIRED(VERSION 3.10.2)
PROJECT(joystick_reader)



# DEPENDENCIES ###########################################################################
# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS regex system thread)
FIND_PACKAGE(SDL2 REQUIRED)


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
    ${SDL2_LIBRARIES}
)
SET(INCLUDES
    ${Boost_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)
SET(SOURCES
)



# BUILDS #################################################################################
# Build Sources
SET(NAME "joystick_sender")
ADD_EXECUTABLE(${NAME} ${SOURCES} ${PROJECT_SOURCE_DIR}/${NAME}.cpp)
TARGET_INCLUDE_DIRECTORIES(${NAME} PRIVATE ${INCLUDES})
TARGET_LINK_LIBRARIES(${NAME} ${LIBRERIES})




# COMMANDS ###############################################################################
# Clean
ADD_CUSTOM_TARGET(rm
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/build/*
    COMMENT "Clean build directory"
)