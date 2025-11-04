cmake_minimum_required(VERSION 3.10)


project(MyProject)

# Enable C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(OpenSSL REQUIRED)

include_directories(
    include
    /usr/local/include
)
file(GLOB SRC_FILES "src/*.cpp" "src/*.c")

add_executable(solar-tracker ${SRC_FILES})
target_link_libraries(
    solar-tracker 
    paho-mqtt3cs
    OpenSSL::SSL
    OpenSSL::Crypto
)
