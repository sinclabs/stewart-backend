cmake_minimum_required(VERSION 3.5L)
project(furhat-servocore)
set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DDEBUG -g -fdiagnostics-show-option -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(dxl_lib "${CMAKE_CURRENT_LIST_DIR}/deps/dynamixel/lib/libdxl_x64_cpp.so")
set(dxl_incl ${CMAKE_CURRENT_LIST_DIR}/deps/dynamixel/include)

# Set CMAKE to output the entire linker list
set(CMAKE_VERBOSE_MAKEFILE on)
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

add_executable(stewart src/main.cpp src/main.cpp)

include_directories(${dxl_incl})
target_link_libraries(stewart pthread ${dxl_lib})