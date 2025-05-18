cmake_minimum_required(VERSION 3.13)
include(CMakePrintHelpers)

project(booksim
    DESCRIPTION "Cycle accurate NoC simulator"
    HOMEPAGE_URL "https://github.com/SLAM-Lab/booksim2-sanafe"
    LANGUAGES C CXX
)

# Set up build flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(
    -Wall -pedantic -Werror
)

add_subdirectory(src)
export(TARGETS booksim NAMESPACE booksim:: FILE booksimTargets.cmake)