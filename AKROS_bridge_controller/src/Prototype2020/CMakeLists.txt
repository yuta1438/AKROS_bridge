add_executable(bending_stretching bending_stretching.cpp)
add_dependencies(bending_stretching  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bending_stretching ${catkin_LIBRARIES})