# cmake_minimum_required (VERSION 3.0)

# project (slq_mpc VERSION 1.0)
# set(PROJECT slq_mpc)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # specify the C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(CMAKE_CPP_COMPILER clang++-11)
# set(CMAKE_C_COMPILER clang-11)


# find_package (Eigen3 REQUIRED NO_MODULE)

# SET(CMAKE_CXX_FLAGS  "-g  -Weffc++ -Wall -I/home/epiphyte/.mujoco/mjpro150/include ${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

# link_directories($ENV{WEBOTS_HOME}/lib/controller)
# link_directories($ENV{WEBOTS_HOME}/lib)

# # find_library(GLFW libglfw.so.3 HINTS /home/epiphyte/.mujoco/mjpro150/bin)
# # message(STATUS "GLFW lib found at: " ${GLFW})

# file(GLOB LIB_MUJOCO /home/epiphyte/.mujoco/mjpro150/bin/libmujoco[0-9][0-9][0-9].so)
# # message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

# # include_directories(/home/epiphyte/.mujoco/mjpro150/include)
# # link_directories(/home/epiphyte/.mujoco/mjpro150/bin)

# set (LIBRARIES 
    # ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    # ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
# )

# include_directories(
    # $ENV{WEBOTS_HOME}/include/controller/c 
    # $ENV{WEBOTS_HOME}/include/controller/cpp
    # ${CMAKE_SOURCE_DIR}/include/Dynamics
    # ${CMAKE_SOURCE_DIR}/include
    # ${LIBRARIES}
    # /home/epiphyte/EigenRand-0.3.3
# )

# file(GLOB
    # CPP_SOURCES
    # ${CMAKE_SOURCE_DIR}/src/*.cpp
    # ${CMAKE_SOURCE_DIR}/src/Dynamics/*.cpp
# )

# add_executable(SLQ_MPC ${CPP_SOURCES})

# # target_link_libraries(SLQ_MPC 
                    # # ${LIB_MUJOCO}
                      # # ${GLFW}
                      # # libGL.so
                      # # libglew.so)

# target_link_libraries(SLQ_MPC Eigen3::Eigen)


# add_custom_command(TARGET SLQ_MPC POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  # copy ${CMAKE_BINARY_DIR}/SLQ_MPC ${CMAKE_SOURCE_DIR}
# )

cmake_minimum_required (VERSION 3.0)

project (slq_mpc VERSION 1.0)
set(PROJECT slq_mpc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CPP_COMPILER clang++-11)
set(CMAKE_C_COMPILER clang-11)


file(GLOB LIB_MUJOCO /home/epiphyte/.mujoco/mjpro150/bin/libmujoco[0-9][0-9][0-9]nogl.so)
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

include_directories(/home/epiphyte/.mujoco/mjpro150/include)
link_directories(/home/epiphyte/.mujoco/mjpro150/bin)


find_package (Eigen3 REQUIRED NO_MODULE)

SET(CMAKE_CXX_FLAGS  "-g  -Weffc++ -Wall ${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

link_directories($ENV{WEBOTS_HOME}/lib/controller)
link_directories($ENV{WEBOTS_HOME}/lib)



set (LIBRARIES 
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

include_directories(
    $ENV{WEBOTS_HOME}/include/controller/c 
    $ENV{WEBOTS_HOME}/include/controller/cpp
    ${CMAKE_SOURCE_DIR}/include/Dynamics
    ${CMAKE_SOURCE_DIR}/include
    /home/epiphyte/EigenRand-0.3.3
)

file(GLOB
    CPP_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/Dynamics/*.cpp
)

add_executable(SLQ_MPC ${CPP_SOURCES})
target_link_libraries(SLQ_MPC ${LIBRARIES} ${LIB_MUJOCO})
target_link_libraries(SLQ_MPC Eigen3::Eigen)


add_custom_command(TARGET SLQ_MPC POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/SLQ_MPC ${CMAKE_SOURCE_DIR}
)

