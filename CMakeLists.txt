cmake_minimum_required(VERSION 2.8)

project( vision_exps )

find_package( OpenCV REQUIRED )

add_executable( image_capture image_capture.cpp )
target_link_libraries( image_capture ${OpenCV_LIBS} )

add_executable( face_detection face_detection.cpp )
target_link_libraries( face_detection ${OpenCV_LIBS} )

add_executable( control control.cpp gopigo.c)
target_link_libraries( control ${OpenCV_LIBS} )

add_executable( face_following face_following.cpp gopigo.c )
target_link_libraries( face_following ${OpenCV_LIBS} )


