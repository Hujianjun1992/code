add_executable(serial_uart serial_uart.cpp)
target_link_libraries(serial_uart ${catkin_LIBRARIES})# ${PROJECT_NAME}) # ${serial_hxb_HDRS})
add_executable(test_hxb test_hxb.cpp)
target_link_libraries(test_hxb ${catkin_LIBRARIES})

add_executable(Test Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})
add_dependencies(Test serial_hxb_generate_messages_cpp)

add_executable(smartcar_joy smartcar_joy.cpp)
target_link_libraries(smartcar_joy ${catkin_LIBRARIES})
