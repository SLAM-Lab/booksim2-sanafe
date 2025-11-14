cmake_minimum_required(VERSION 3.13)

set(LEX_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.c)
set(YACC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/y.tab.c)
set(YACC_HEADER ${CMAKE_CURRENT_BINARY_DIR}/y.tab.h)

set(PARSER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

if(BOOKSIM_GENERATE_PARSER)
    message("Generating parser using bison and flex")
    find_package(BISON REQUIRED)
    find_package(FLEX REQUIRED)

    # Generate parser files with BISON
    BISON_TARGET(ConfigParser
                ${CMAKE_CURRENT_SOURCE_DIR}/config.y
                ${YACC_OUTPUT}
                DEFINES_FILE ${YACC_HEADER})
    # Generate lexer files with FLEX
    FLEX_TARGET(ConfigScanner
                ${CMAKE_CURRENT_SOURCE_DIR}/config.l
                ${LEX_OUTPUT})
    ADD_FLEX_BISON_DEPENDENCY(ConfigScanner ConfigParser)
else()
    message(STATUS "Using pre-generated parser files from repository")
    # Point to the pre-generated files in your repository
    # Adjust these paths to match where you store the pre-generated files
    set(BISON_ConfigParser_OUTPUTS
        ${CMAKE_CURRENT_SOURCE_DIR}/y.tab.c
        ${CMAKE_CURRENT_SOURCE_DIR}/y.tab.h
    )
    set(FLEX_ConfigScanner_OUTPUTS
        ${CMAKE_CURRENT_SOURCE_DIR}/lex.yy.c
    )

    # Also set these for consistency (in case they're used elsewhere)
    set(YACC_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/y.tab.c)
    set(YACC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/y.tab.h)
    set(LEX_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/lex.yy.c)
endif()

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

# Dynamic library version
#  Disable for now, since it makes my life more difficult
# The library with the booksim_lib API
# add_library(booksim SHARED
#     $<TARGET_OBJECTS:booksim_common>
#     ${CMAKE_CURRENT_SOURCE_DIR}/booksim_lib.cpp
# )
# target_include_directories(booksim PUBLIC ${BOOKSIM_INCLUDE_DIRS})

# Static library version
add_library(booksim STATIC
    $<TARGET_OBJECTS:booksim_common>
    ${CMAKE_CURRENT_SOURCE_DIR}/booksim_lib.cpp
)
set_target_properties(booksim PROPERTIES OUTPUT_NAME booksim)
target_include_directories(booksim PUBLIC ${BOOKSIM_INCLUDE_DIRS})

# The standalone executable with main.cpp
 add_executable(booksim_exe
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    $<TARGET_OBJECTS:booksim_common>
)
set_target_properties(booksim_exe PROPERTIES OUTPUT_NAME booksim)
target_include_directories(booksim_exe PRIVATE ${BOOKSIM_INCLUDE_DIRS})

if(BOOKSIM_INSTALL)
    # Installation rules
    install(TARGETS booksim booksim_exe
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
endif()
