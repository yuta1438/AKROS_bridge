##  脚移動系  ##
# Prototype2020_initialPose
add_executable(Prototype2020_initialPose        initialPose.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_initialPose      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_initialPose ${catkin_LIBRARIES})

# Prototype2020_jointmove
add_executable(Prototype2020_jointsmove         JointsMove.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_jointsmove       ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_jointsmove  ${catkin_LIBRARIES})

# Prototype2020_bending_stretching
add_executable(Prototype2020_bending_stretching         bending_stretching.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_bending_stretching       ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_bending_stretching  ${catkin_LIBRARIES})

# Prototype2020_legmove
add_executable(Prototype2020_legmove        LegMove.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_legmove      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_legmove ${catkin_LIBRARIES})

# Prototype2020_legmove_joypad
add_executable(Prototype2020_legmove_joypad        LegMove_joypad.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_legmove_joypad      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_legmove_joypad ${catkin_LIBRARIES})


# Prototype2020_bending_stretching_offset
add_executable(Prototype2020_bending_stretching_offset          bending_stretching_offset.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_bending_stretching_offset        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_bending_stretching_offset   ${catkin_LIBRARIES})

##  車輪移動系  ##
# Prototype2020_wheelmove
add_executable(Prototype2020_wheelmove          WheelMove.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_wheelmove        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_wheelmove   ${catkin_LIBRARIES})

# Prototype2020_wheelmove_teleop
add_executable(Prototype2020_wheelmove_teleop         WheelMove_teleop.cpp)
add_dependencies(Prototype2020_wheelmove_teleop       ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_wheelmove_teleop  ${catkin_LIBRARIES})

##  脚車輪系  ##
# Prototype2020_bending_stretching_wheel
add_executable(Prototype2020_bending_stretching_wheel          bending_stretching_wheel.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_bending_stretching_wheel        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_bending_stretching_wheel   ${catkin_LIBRARIES})

# Prototype2020_hopping
add_executable(Prototype2020_hopping          hopping.cpp Prototype2020_BaseController.cpp)
add_dependencies(Prototype2020_hopping        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_hopping   ${catkin_LIBRARIES})

# その他
# FootPosition_Publisher
add_executable(Prototype2020_footposition_publisher          footPosition_Publisher.cpp)
add_dependencies(Prototype2020_footposition_publisher        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_footposition_publisher   ${catkin_LIBRARIES})