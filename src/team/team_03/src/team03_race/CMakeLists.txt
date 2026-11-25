cmake_minimum_required(VERSION 3.8)
project(team03_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vehicle_adapter.py
  scripts/wads_teleop.py
  scripts/race_driver.py
  scripts/start_signal_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES race_entry.json DESTINATION share/${PROJECT_NAME})

ament_package()
