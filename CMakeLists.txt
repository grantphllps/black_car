cmake_minimum_required(VERSION 3.0.2)
project(black_car)
set(OpenCV_DIR /home/car/opencv/build)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs OpenCV cv_bridge image_transport message_generation)
add_message_files(
	DIRECTORY
	msg
	FILES
	CarObject.msg
  CarObjects.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
set(Torch_DIR /home/car/.local/lib/python3.6/site-packages/torch/share/cmake/Torch)
find_package(Torch REQUIRED)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ${Torch_INCLUDE_DIRS}
)

add_executable(grantphllps_safety src/grantphllps_safety.cpp)
target_link_libraries(grantphllps_safety ${catkin_LIBRARIES})

add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES})

add_executable(error_provider src/error_provider.cpp)
target_link_libraries(error_provider ${catkin_LIBRARIES})