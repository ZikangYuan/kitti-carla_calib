set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE( motion_calib main.cpp )
target_link_libraries(motion_calib ${PCL_LIBRARIES} ${OpenCV_LIBS})
