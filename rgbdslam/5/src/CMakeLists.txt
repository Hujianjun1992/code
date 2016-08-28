find_package(PCL REQUIRED COMPONENTS commom io visualization filters)

find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})


add_library(slambase slamBase.cpp)
target_link_libraries(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(visualOdometry visualOdometry.cpp)
target_link_libraries(visualOdometry slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})
