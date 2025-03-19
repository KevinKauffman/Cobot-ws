cmake_minimum_required(VERSION 3.22)
project(mycobot_280_moveit2)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install custom Python nodes (scripts)
install(PROGRAMS
  scripts/csv_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME csv_publisher
)

install(PROGRAMS
  scripts/planning_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planning_bridge
)
