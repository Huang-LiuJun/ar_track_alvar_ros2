cmake_minimum_required(VERSION 3.8)
project(ar_track_alvar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation filters registration sample_consensus)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML REQUIRED tinyxml)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/Bitset.cpp
  src/Camera.cpp
  src/CaptureDevice.cpp
  src/CaptureFactory.cpp
  src/CaptureFactory_unix.cpp
  src/ConnectedComponents.cpp
  src/CvTestbed.cpp
  src/DirectoryIterator.cpp
  src/DirectoryIterator_unix.cpp
  src/Draw.cpp
  src/FileFormatUtils.cpp
  src/Filter.cpp
  src/Kalman.cpp
  src/Line.cpp
  src/Marker.cpp
  src/MarkerDetector.cpp
  src/MultiMarker.cpp
  src/MultiMarkerBundle.cpp
  src/MultiMarkerInitializer.cpp
  src/Mutex.cpp
  src/Mutex_unix.cpp
  src/Optimization.cpp
  src/Plugin.cpp
  src/Plugin_unix.cpp
  src/Pose.cpp
  src/Rotation.cpp
  src/Threads.cpp
  src/Threads_unix.cpp
  src/Util.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TINYXML_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  sensor_msgs
  tf2
)

add_library(kinect_filtering SHARED src/kinect_filtering.cpp)
target_link_libraries(kinect_filtering ${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(kinect_filtering geometry_msgs tf2)

add_library(medianFilter SHARED src/medianFilter.cpp)
target_link_libraries(medianFilter ${PROJECT_NAME})
ament_target_dependencies(medianFilter)

set(ATA_NODE_DEPS
  rclcpp
  ar_track_alvar_msgs
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
set(ATA_NODE_LIBS ${PROJECT_NAME})

add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ${ATA_NODE_LIBS})
ament_target_dependencies(individualMarkersNoKinect ${ATA_NODE_DEPS})

add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ${ATA_NODE_LIBS} kinect_filtering ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(individualMarkers ${ATA_NODE_DEPS} pcl_conversions)

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ${ATA_NODE_LIBS})
ament_target_dependencies(findMarkerBundlesNoKinect ${ATA_NODE_DEPS})

add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ${ATA_NODE_LIBS} kinect_filtering medianFilter ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(findMarkerBundles ${ATA_NODE_DEPS} pcl_conversions)

add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ${ATA_NODE_LIBS})
ament_target_dependencies(trainMarkerBundle ${ATA_NODE_DEPS})

add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ${PROJECT_NAME} ${OpenCV_LIBS})
ament_target_dependencies(createMarker)

install(TARGETS
  ${PROJECT_NAME}
  kinect_filtering
  medianFilter
  createMarker
  findMarkerBundles
  findMarkerBundlesNoKinect
  individualMarkers
  individualMarkersNoKinect
  trainMarkerBundle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY bundles launch markers config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} kinect_filtering medianFilter)
ament_export_dependencies(geometry_msgs sensor_msgs tf2)
ament_package()
