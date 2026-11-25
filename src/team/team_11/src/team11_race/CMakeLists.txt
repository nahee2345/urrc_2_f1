cmake_minimum_required(VERSION 3.16)
project(team11_race)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/race_driver.py
  DESTINATION lib/${PROJECT_NAME} RENAME race_driver
)

install(PROGRAMS scripts/vehicle_adapter.py
  DESTINATION lib/${PROJECT_NAME} RENAME vehicle_adapter
)

install(PROGRAMS scripts/wads_teleop.py
  DESTINATION lib/${PROJECT_NAME} RENAME wads_teleop
)

ament_package()
