cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tunnel)

add_executable(tunnel
    tunnel.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(tunnel
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(tunnel PRIVATE -Wall -Wextra)
else()
    add_compile_options(tunnel PRIVATE -W2)
endif()