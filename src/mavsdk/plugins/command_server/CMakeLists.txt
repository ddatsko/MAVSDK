target_sources(mavsdk
    PRIVATE
    command_server.cpp
    command_server_impl.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(FILES
    include/plugins/command_server/command_server.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/command_server
)