cmake_minimum_required(VERSION 3.1)
project(humanoid_sim C)

set(MUJOCO_INCLUDE_DIRS "~/.mujoco/mujoco210/include")
set(MUJOCO_LIBRARY_DIRS "~/.mujoco/mujoco210/bin")

include_directories(${MUJOCO_INCLUDE_DIRS})

link_directories(~/.mujoco/mujoco210/bin)
find_library(GLFW libglfw.so.3 HINTS ~/.mujoco/mujoco210/bin)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto")
file(GLOB LIB_MUJOCO ~/.mujoco/mujoco210_linux/bin/libmujoco210.so)

add_executable(humanoid_sim main.c humanoidmujoco.c humanoidmujoco.h udp.c udp.h)

target_link_libraries(humanoid_sim
	${LIB_MUJOCO}
        ${GLFW}
        libGL.so
        libglew.so
        m
        dl)
