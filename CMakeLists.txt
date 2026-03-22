cmake_minimum_required(VERSION 3.28)
project(laptime_simulator LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
endif()

set(SOURCE_FILES )

add_executable(${PROJECT_NAME} 
    src/main.cpp
)

target_include_directories(laptime_simulator PRIVATE src)
