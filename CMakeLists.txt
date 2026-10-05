cmake_minimum_required(VERSION 4.2)
project(OptiBench)

set(CMAKE_CXX_STANDARD 20)

add_executable(OptiBench main.cpp
        include/optibench/Color.h
        include/optibench/Vec2.h
        include/optibench/Ray.h)
