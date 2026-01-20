cmake_minimum_required(VERSION 4.1)
project(chess_arm)

set(CMAKE_CXX_STANDARD 23)

if (TESTING)
    add_compile_definitions(TESTING)
    add_compile_definitions(PIPELINE=0)
else (TESTING)
    add_compile_definitions("PIPELINE=\"libcamerasrc ! videoconvert ! \" \
                                      \"video/x-raw,format=GRAY8,width=640,height=480 ! \" \
                                      \"appsink sync=false drop=true max-buffers=1\"")
endif (TESTING)

add_executable(chess_arm
        src/main.cpp
        src/quaternion.cpp
        src/path.cpp
        src/arm.cpp
        src/camera.cpp
        src/easywsclient.cpp
)

target_sources(chess_arm
        PUBLIC
        FILE_SET modules TYPE CXX_MODULES FILES
        lib/quaternion.cppm
        lib/path.cppm
        lib/arm.cppm
        lib/camera.cppm
)

find_package(OpenCV REQUIRED)
target_link_libraries(chess_arm ${OpenCV_LIBRARIES})
target_include_directories(chess_arm PRIVATE ${OpenCV_INCLUDE_DIRS})
