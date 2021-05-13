cmake_minimum_required(VERSION 2.8.3)
project(roomba_stage)

# Find catkin
find_package(catkin REQUIRED)
find_package(stage REQUIRED)

catkin_package()


add_library(wander_controller SHARED ${PROJECT_SOURCE_DIR}/src/wander_controller.cc)

target_include_directories(wander_controller
  PRIVATE ${STAGE_INCLUDE_DIRS} include
)

target_link_libraries(wander_controller
  PRIVATE ${STAGE_LIBRARIES}
)

install(TARGETS wander_controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Install maps files
install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install params files
install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install other files
install(FILES move_base_lse_arena.launch roomba_lse_arena.world roomba_stage.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
