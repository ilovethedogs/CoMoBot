cmake_minimum_required(VERSION 3.22)
project(comobot_keyboard)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

FetchContent_Declare (
        asio
        GIT_REPOSITORY https://github.com/chriskohlhoff/asio
        GIT_TAG asio-1-23-0
        UPDATE_DISCONNECTED 1
)

FetchContent_Populate(asio)
include_directories(${asio_SOURCE_DIR}/asio/include/)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE -pthread")

add_executable(comobot_keyboard main.cpp session.cpp session.h)