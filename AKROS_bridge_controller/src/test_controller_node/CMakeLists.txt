add_executable(motor_controller             motor_controller.cpp )
add_executable(move_to_zero                 move_to_zero.cpp )
add_executable(single_motor_controller      single_motor_controller.cpp)
add_executable(wheel_velocity_controller    wheel_velocity_controller.cpp)

add_dependencies(motor_controller           ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_to_zero               ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(single_motor_controller    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(wheel_velocity_controller  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motor_controller          ${catkin_LIBRARIES})
target_link_libraries(move_to_zero              ${catkin_LIBRARIES})
target_link_libraries(single_motor_controller   ${catkin_LIBRARIES})
target_link_libraries(wheel_velocity_controller ${catkin_LIBRARIES})