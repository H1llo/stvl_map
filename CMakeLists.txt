cmake_minimum_required(VERSION 3.22)
project(stvl_map)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(finenav_core REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(stvl_map_params config/stvl_map.yaml)

add_library(stvl_map SHARED
  src/stvl_map.cpp
  src/plugin_registration.cpp
)
target_compile_features(stvl_map PUBLIC cxx_std_20)
target_include_directories(stvl_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stvl_map PUBLIC finenav_core rclcpp sensor_msgs)
target_link_libraries(stvl_map PUBLIC Eigen3::Eigen stvl_map_params)

install(TARGETS stvl_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_map test/test_grid_map.cpp)
  target_include_directories(test_grid_map PRIVATE include)
  target_link_libraries(test_grid_map stvl_map)

  ament_add_gtest(test_stvl_map test/test_stvl_map.cpp)
  target_include_directories(test_stvl_map PRIVATE include)
  target_link_libraries(test_stvl_map stvl_map)
  ament_target_dependencies(test_stvl_map finenav_core rclcpp sensor_msgs)

  ament_add_gtest(test_plugin_registration test/test_plugin_registration.cpp)
  target_compile_definitions(test_plugin_registration PRIVATE
    STVL_MAP_LIBRARY="$<TARGET_FILE:stvl_map>")
  target_link_libraries(test_plugin_registration ${CMAKE_DL_LIBS})
  ament_target_dependencies(test_plugin_registration finenav_core rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(stvl_map)
ament_export_dependencies(eigen3_cmake_module finenav_core rclcpp sensor_msgs)
ament_package()
