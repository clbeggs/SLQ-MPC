cmake_minimum_required (VERSION 3.0)

project (slq_mpc VERSION 1.0)
set(PROJECT slq_mpc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CPP_COMPILER clang++-11)
set(CMAKE_C_COMPILER clang-11)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
# find_package(LAPACK REQUIRED NO_MODULE)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/intel/oneapi/mkl/latest/tools/pkgconfig")
set(PKG_CONFIG_MKL_DYNAMIC_LP64_IOMP_MKLROOT ${MKLROOT} )
pkg_check_modules(MKL REQUIRED mkl-dynamic-lp64-iomp)
# pkg_check_modules(LAPACK REQUIRED lapack)

message("Intel MKL include dirs found at: " ${MKL_LIBRARY_DIRS})
message("Intel MKL libraries dirs found at: " ${MKL_INCLUDE_DIRS})
message("Intel MKL Libraries: " ${MKL_LIBRARIES})

# message("LAPACK include dirs found at: " ${LAPACK_LIBRARY_DIRS})
# message("LAPACK libraries dirs found at: " ${LAPACK_INCLUDE_DIRS})
# message("LAPACK Libraries: " ${LAPACK_LIBRARIES})


file(GLOB LIB_MUJOCO /home/epiphyte/.mujoco/mjpro150/bin/libmujoco[0-9][0-9][0-9]nogl.so)
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

include_directories(/home/epiphyte/.mujoco/mjpro150/include)
include_directories(${MKL_INCLUDE_DIRS})
# include_directories(${LAPACK_INCLUDE_DIRS})
link_directories(/home/epiphyte/.mujoco/mjpro150/bin)
link_directories(${MKL_LIBRARY_DIRS})
# link_directories(${LAPACK_LIBRARY_DIRS})



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
    ${CMAKE_SOURCE_DIR}/include/Robots
    ${CMAKE_SOURCE_DIR}/include
    /home/epiphyte/EigenRand-0.3.3
)

file(GLOB
    CPP_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/Dynamics/*.cpp
    ${CMAKE_SOURCE_DIR}/src/Robots/*.cpp
)

add_executable(SLQ_MPC ${CPP_SOURCES})
target_link_libraries(SLQ_MPC ${LIBRARIES} ${LIB_MUJOCO})
target_link_libraries(SLQ_MPC Eigen3::Eigen)
# target_link_libraries(SLQ_MPC ${LAPACK_LIBRARIES})
target_link_libraries(SLQ_MPC ${MKL_LIBRARIES})


add_custom_command(TARGET SLQ_MPC POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/SLQ_MPC ${CMAKE_SOURCE_DIR}
)

