cmake_minimum_required(VERSION 3.8)
project(semantic_costmap_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  nav2_costmap_2d
  nav2_map_server
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
)

add_library(${PROJECT_NAME}_core SHARED
  src/fuzzy_inference_engine.cpp
  src/mask_inflator.cpp
  src/mask_layer.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d custom_layer.xml)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_fuzzy_inference_engine
    test/test_fuzzy_inference_engine.cpp
  )
  target_link_libraries(test_fuzzy_inference_engine ${PROJECT_NAME}_core)
  target_include_directories(test_fuzzy_inference_engine PUBLIC include)
  ament_target_dependencies(test_fuzzy_inference_engine nav2_costmap_2d)

  ament_add_gtest(test_mask_inflator
    test/test_mask_inflator.cpp
  )
  target_link_libraries(test_mask_inflator ${PROJECT_NAME}_core)
  target_include_directories(test_mask_inflator PUBLIC include)
  ament_target_dependencies(test_mask_inflator nav2_costmap_2d)

  ament_add_gtest(test_plugin_registration
    test/test_plugin_registration.cpp
  )
  ament_target_dependencies(test_plugin_registration
    nav2_costmap_2d
    pluginlib
  )

  ament_add_gtest(test_mask_layer
    test/test_mask_layer.cpp
  )
  target_link_libraries(test_mask_layer ${PROJECT_NAME}_core)
  target_include_directories(test_mask_layer PUBLIC include)
  target_compile_definitions(test_mask_layer PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  ament_target_dependencies(test_mask_layer
    nav2_costmap_2d
    nav2_util
    rclcpp
    tf2_ros
  )
endif()

ament_package()
