find_package(PCL REQUIRED COMPONENTS commom io visualization filters)

find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})


add_library(slambase slamBase.cpp)
target_link_libraries(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( G2O_ROOT /usr/local/include/g2o )
find_package( G2O )

find_package( CSparse )
include_directories( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

add_executable( slam slam.cpp)
target_link_libraries(slam slambase ${OpenCV_LIBS} ${PCL_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse
  g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} )
