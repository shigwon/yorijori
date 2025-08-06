cmake_minimum_required(VERSION 3.14)
project(webrtc_streamer_pkg)

# ROS 2 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# GStreamer 패키지들
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GSTREAMER_SDP REQUIRED gstreamer-sdp-1.0)
pkg_check_modules(GSTREAMER_WEBRTC REQUIRED gstreamer-webrtc-1.0)

# include 및 lib 경로 지정
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${GSTREAMER_VIDEO_INCLUDE_DIRS}
  ${GSTREAMER_SDP_INCLUDE_DIRS}
  ${GSTREAMER_WEBRTC_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_APP_LIBRARY_DIRS}
  ${GSTREAMER_VIDEO_LIBRARY_DIRS}
  ${GSTREAMER_SDP_LIBRARY_DIRS}
  ${GSTREAMER_WEBRTC_LIBRARY_DIRS}
)

# 실행 파일 생성
add_executable(webrtc_streamer_node src/webrtc_streamer.cpp)

# 컴파일 시 unstable API 사용 허용
target_compile_definitions(webrtc_streamer_node PRIVATE GST_USE_UNSTABLE_API)

# ROS 2 의존성 연결
ament_target_dependencies(webrtc_streamer_node
  rclcpp
  std_msgs
  sensor_msgs
)

# GStreamer 라이브러리 연결
target_link_libraries(webrtc_streamer_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
  ${GSTREAMER_SDP_LIBRARIES}
  ${GSTREAMER_WEBRTC_LIBRARIES}
)

# 설치 설정
install(TARGETS
  webrtc_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
