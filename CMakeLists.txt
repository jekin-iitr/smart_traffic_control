cmake_minimum_required( VERSION 2.8)
project( motion_tracking )
find_package( OpenCV REQUIRED )
add_executable( motion_tracking main )
target_link_libraries( motion_tracking ${OpenCV_LIBS} )

