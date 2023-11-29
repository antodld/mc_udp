set(AUTOLOAD_UDPPlugin_PLUGIN false CACHE BOOL "When true the plugin will always be loaded")

find_package(mc_rtc REQUIRED)

set(plugin_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/UDPPlugin.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/UDPRobotControl.cpp
)

set(plugin_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/UDPPlugin.h
  ${CMAKE_CURRENT_SOURCE_DIR}/UDPRobotControl.h
)
add_plugin(UDPPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(UDPPlugin PUBLIC mc_udp_client mc_rtc::mc_control)
target_include_directories(UDPPlugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
install(FILES etc/UDPPlugin.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
