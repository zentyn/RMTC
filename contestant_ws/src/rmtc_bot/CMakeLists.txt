cmake_minimum_required(VERSION 3.8)
project(rmtc_bot CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rmtc_bot_node
  src/main.cpp
  src/http_client.cpp
  src/perception.cpp
  src/world_memory.cpp
  src/planner.cpp
  src/strategy.cpp
  src/llm_advisor.cpp
)
target_include_directories(rmtc_bot_node PRIVATE include)
ament_target_dependencies(rmtc_bot_node rclcpp std_msgs)
target_link_libraries(rmtc_bot_node pthread)

install(TARGETS rmtc_bot_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
