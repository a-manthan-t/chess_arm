cmake_minimum_required(VERSION 4.1)
project(chess_arm)

set(CMAKE_CXX_STANDARD 23)

if (TESTING)
    add_compile_definitions(TESTING)
endif (TESTING)

add_executable(chess_arm
        src/main.cpp
        src/quaternion.cpp
        src/path.cpp
        src/arm.cpp
        src/camera.cpp
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
target_link_libraries(chess_arm ${OpenCV_LIBS})
target_include_directories(chess_arm PRIVATE ${OpenCV_INCLUDE_DIRS})
