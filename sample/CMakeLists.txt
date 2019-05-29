include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(calibrate_single_camera calibrate_single_camera.cc)
add_executable(calibrate_stereo_camera calibrate_stereo_camera.cc)

target_link_libraries(calibrate_single_camera calib ${OpenCV_LIBS})
target_link_libraries(calibrate_stereo_camera calib ${OpenCV_LIBS})
