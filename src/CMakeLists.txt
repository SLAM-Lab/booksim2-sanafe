cmake_minimum_required(VERSION 3.13)
project(booksim2 LANGUAGES CXX)

# Set up build flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find lex and yacc
find_program(LEX_EXECUTABLE NAMES flex)
find_program(YACC_EXECUTABLE NAMES bison)

# Define source directories and include paths
set(BOOKSIM_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/arbiters
    ${CMAKE_CURRENT_SOURCE_DIR}/allocators
    ${CMAKE_CURRENT_SOURCE_DIR}/routers
    ${CMAKE_CURRENT_SOURCE_DIR}/networks
    ${CMAKE_CURRENT_SOURCE_DIR}/power
)

# Get all source files except main.cpp
file(GLOB_RECURSE BOOKSIM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
list(FILTER BOOKSIM_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# Generate lexer and parser files
set(LEX_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.c)
set(YACC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/y.tab.c)
set(YACC_HEADER ${CMAKE_CURRENT_BINARY_DIR}/y.tab.h)

execute_process(
    COMMAND ${LEX_EXECUTABLE} -o ${LEX_OUTPUT} ${CMAKE_CURRENT_SOURCE_DIR}/config.l
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

execute_process(
    COMMAND ${YACC_EXECUTABLE} -d -o ${YACC_OUTPUT} ${CMAKE_CURRENT_SOURCE_DIR}/config.y
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the booksim library
add_library(booksim STATIC ${BOOKSIM_SOURCES} ${LEX_OUTPUT} ${YACC_OUTPUT})
set_target_properties(booksim PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(booksim PUBLIC ${BOOKSIM_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

# Create a shared library version
add_library(booksim_shared SHARED ${BOOKSIM_SOURCES} ${LEX_OUTPUT} ${YACC_OUTPUT})
set_target_properties(booksim_shared PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(booksim_shared PROPERTIES
    OUTPUT_NAME booksim
    POSITION_INDEPENDENT_CODE ON
)
target_include_directories(booksim_shared PUBLIC ${BOOKSIM_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

# Installation rules
install(TARGETS booksim booksim_shared
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
    DESTINATION include/booksim2
    FILES_MATCHING PATTERN "*.hpp"
)
