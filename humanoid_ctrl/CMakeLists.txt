cmake_minimum_required(VERSION 3.16)
project(humanoid_ctrl)

find_package(Threads)

set(CMAKE_CXX_STANDARD 17)
FIND_PACKAGE(pinocchio REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(osqp REQUIRED)

include_directories(
        ${pinocchio_INCLUDE_DIRS}
)
link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_executable(humanoid_ctrl main.cpp udp.c udp.h)

target_link_libraries(humanoid_ctrl ${CMAKE_THREAD_LIBS_INIT} ${pinocchio_LIBRARIES} Eigen3::Eigen osqp::osqp osqp::osqpstatic)
