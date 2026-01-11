cmake_minimum_required(VERSION 4.1)

project(chess_arm)
set(CMAKE_CXX_STANDARD 23)

if (TESTING)
    add_compile_definitions(TESTING)
endif (TESTING)

add_executable( chess_arm
        src/main.cpp
        src/quaternion.cpp
        src/arm.cpp
        src/path.cpp
)

target_sources( chess_arm
        PUBLIC
        FILE_SET modules TYPE CXX_MODULES FILES
        lib/quaternion.cppm
        lib/arm.cppm
        lib/path.cppm
)
