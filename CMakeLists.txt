cmake_minimum_required(VERSION 3.9)
project(kobuki_control)

#set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )

set(SOURCE_FILES
        src/CKobuki.cpp
        src/CKobuki.h
        src/main.cpp)

add_executable(kobuki_control ${SOURCE_FILES})
#target_link_libraries(kobuki_control )