cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tunnel_server)

add_executable(tunnel_server
    tunnel_server.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(tunnel_server
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(tunnel_server PRIVATE -Wall -Wextra)
else()
    add_compile_options(tunnel_server PRIVATE -W2)
endif()
