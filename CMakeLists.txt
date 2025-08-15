cmake_minimum_required(VERSION 3.10)


project(MyProject)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

include_directories(include)
file(GLOB SRC_FILES "src/*.cpp" "src/*.c")

add_executable(main ${SRC_FILES})
target_link_libraries(main wiringPi)
