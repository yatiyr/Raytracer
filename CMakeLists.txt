cmake_minimum_required(VERSION 3.10)
project(RaytracingHomework VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCE_FILES
     ${CMAKE_SOURCE_DIR}/src/*.c
     ${CMAKE_SOURCE_DIR}/src/*.cpp)

file(GLOB_RECURSE HEADER_FILES
     ${CMAKE_SOURCE_DIR}/src/*.h
     ${CMAKE_SOURCE_DIR}/src/*.hpp)

link_directories(${CMAKE_SOURCE_DIR}/lib)

configure_file(src/config/RootDir.h.in src/RootDir.h)

include_directories(${CMAKE_BINARY_DIR}/src)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})


include_directories("${CMAKE_SOURCE_DIR}/src"
                    "${CMAKE_SOURCE_DIR}/include"
)
