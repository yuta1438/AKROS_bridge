# Prototype2020_initialPose
add_executable(Prototype2020_initialPose initialPose.cpp)
add_dependencies(Prototype2020_initialPose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_initialPose ${catkin_LIBRARIES})

# Prototype2020_bending_stretching
add_executable(Prototype2020_bending_stretching bending_stretching.cpp)
add_dependencies(Prototype2020_bending_stretching  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_bending_stretching ${catkin_LIBRARIES})

# Prototype2020_legmove
add_executable(Prototype2020_legmove LegMove.cpp)
add_dependencies(Prototype2020_legmove  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_legmove ${catkin_LIBRARIES})

# Prototype2020_bending_stretching_offset
add_executable(Prototype2020_bending_stretching_offset bending_stretching_offset.cpp)
add_dependencies(Prototype2020_bending_stretching_offset  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_bending_stretching_offset ${catkin_LIBRARIES})

# Prototype2020_wheelmove
add_executable(Prototype2020_wheelmove WheelMove.cpp)
add_dependencies(Prototype2020_wheelmove  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_wheelmove ${catkin_LIBRARIES})

# Prototype2020_jointmove
add_executable(Prototype2020_jointsmove JointsMove.cpp)
add_dependencies(Prototype2020_jointsmove  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prototype2020_jointsmove ${catkin_LIBRARIES})

