cmake_minimum_required(VERSION 3.10)


project(MyProject)

# Enable C++20
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(OpenSSL REQUIRED)

include_directories(
    include
    /usr/local/include
)
file(GLOB SRC_FILES "src/*.cpp" "src/*.c")

add_executable(main ${SRC_FILES})
target_link_libraries(
    main 
    wiringPi
    /usr/local/lib/libpaho-mqtt3c.so
)
