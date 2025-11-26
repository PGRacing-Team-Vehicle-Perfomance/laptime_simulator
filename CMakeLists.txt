cmake_minimum_required(VERSION 3.28)
project(gfr_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCE_FILES )

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/vehicle/vehicle.cpp
    src/simulation/skidpad.cpp
    src/vehicle/tire/tire.cpp
)

target_include_directories(gfr_sim PRIVATE src)