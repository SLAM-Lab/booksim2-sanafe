cmake_minimum_required(VERSION 3.13)
#set(CMAKE_VERBOSE_MAKEFILE ON)
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

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

set(PARSER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
# Generate parser files with BISON
BISON_TARGET(ConfigParser
             ${CMAKE_CURRENT_SOURCE_DIR}/config.y
             ${PARSER_OUTPUT_DIR}/config.tab.c
             DEFINES_FILE ${PARSER_OUTPUT_DIR}/config.tab.h)

# Generate lexer files with FLEX
FLEX_TARGET(ConfigScanner
            ${CMAKE_CURRENT_SOURCE_DIR}/config.l
            ${PARSER_OUTPUT_DIR}/config.lex.c)
ADD_FLEX_BISON_DEPENDENCY(ConfigScanner ConfigParser)

# Define source directories and include paths
set(BOOKSIM_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/arbiters
    ${CMAKE_CURRENT_SOURCE_DIR}/allocators
    ${CMAKE_CURRENT_SOURCE_DIR}/routers
    ${CMAKE_CURRENT_SOURCE_DIR}/networks
    ${CMAKE_CURRENT_SOURCE_DIR}/power
)

# Find all source files
set(SRC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/arbiters
    ${CMAKE_CURRENT_SOURCE_DIR}/allocators
    ${CMAKE_CURRENT_SOURCE_DIR}/routers
    ${CMAKE_CURRENT_SOURCE_DIR}/networks
    ${CMAKE_CURRENT_SOURCE_DIR}/power
)

# Collect CPP files explicitly from the source directories
set(COMMON_SOURCES "")
foreach(DIR ${SRC_DIRS})
    file(GLOB DIR_SRCS "${DIR}/*.cpp")
    list(APPEND COMMON_SOURCES ${DIR_SRCS})
endforeach()

list(FILTER COMMON_SOURCES EXCLUDE REGEX ".*booksim_lib\\.cpp$") # Exclude library interface file
list(FILTER COMMON_SOURCES EXCLUDE REGEX ".*main\\.cpp$") # Exclude main.cpp

cmake_print_variables(YACC_OUTPUT)
cmake_print_variables(LEX_OUTPUT)

# Create object library with common code
add_library(booksim_common OBJECT
    ${COMMON_SOURCES}
    ${BISON_ConfigParser_OUTPUTS}
    ${FLEX_ConfigScanner_OUTPUTS}
)
target_include_directories(booksim_common PUBLIC ${BOOKSIM_INCLUDE_DIRS})
set_target_properties(booksim_common PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# The library with the booksim_lib API
add_library(booksim SHARED
    $<TARGET_OBJECTS:booksim_common>
    ${CMAKE_CURRENT_SOURCE_DIR}/booksim_lib.cpp
)
target_include_directories(booksim PUBLIC ${BOOKSIM_INCLUDE_DIRS})

# Static library version
add_library(booksim_static STATIC
    $<TARGET_OBJECTS:booksim_common>
    ${CMAKE_CURRENT_SOURCE_DIR}/booksim_lib.cpp
)
set_target_properties(booksim_static PROPERTIES OUTPUT_NAME booksim)
target_include_directories(booksim_static PUBLIC ${BOOKSIM_INCLUDE_DIRS})

# The standalone executable with main.cpp
 add_executable(booksim_exe
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    $<TARGET_OBJECTS:booksim_common>
)
set_target_properties(booksim_exe PROPERTIES OUTPUT_NAME booksim)
target_include_directories(booksim_exe PRIVATE ${BOOKSIM_INCLUDE_DIRS})

# Installation rules
install(TARGETS booksim booksim_static booksim_exe
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)

# Also install booksim_lib.h
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/booksim_lib.hpp
    DESTINATION include/
)
