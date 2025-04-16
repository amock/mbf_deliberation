cmake_minimum_required(VERSION 3.16)
project(mbf_btcpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    geometry_msgs
    mbf_msgs)


add_library(mbf_btcpp_plugin SHARED
    src/mbf_btcpp_plugin.cpp
    )
target_include_directories(mbf_btcpp_plugin PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_compile_definitions(mbf_btcpp_plugin PRIVATE 
    BT_PLUGIN_EXPORT)
ament_target_dependencies(mbf_btcpp_plugin
    ${THIS_PACKAGE_DEPS}
)

install(TARGETS
    mbf_btcpp_plugin
    LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
    ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
    RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)




add_executable(navigate_to_pose src/navigate_to_pose.cpp)
target_link_libraries(navigate_to_pose mbf_btcpp_plugin)
ament_target_dependencies(navigate_to_pose ${THIS_PACKAGE_DEPS})

install(TARGETS
    navigate_to_pose
    DESTINATION lib/${PROJECT_NAME}
  )


ament_export_dependencies(behaviortree_ros2)
ament_package()






