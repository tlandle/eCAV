cmake_minimum_required(VERSION 3.14)
project(channel_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pybind11 REQUIRED)

pybind11_add_module(channel_engine
    channel_engine.cpp
    bindings.cpp
)

target_include_directories(channel_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(channel_engine PRIVATE
    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
)

install(TARGETS channel_engine
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
)
