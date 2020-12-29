cmake_minimum_required(VERSION 3.0)
project(BoostSerial)

###################################################################
# Option Setting
option(ENABLE_TEST "Download gtest and build test program.(Default:OFF)" OFF)
option(ENABLE_SAMPLE "build sample program.(Default:OFF)" OFF)

###################################################################
# General Setting

# enalble C++17
set (CMAKE_CXX_STANDARD 17)

###################################################################
# Dependency

# Add boost::asio
find_package(Boost 1.71.0 COMPONENTS REQUIRED system)

###################################################################
# Library Build Setting

# globbing file
file(GLOB BOOST_SERIAL_SRC src/*.cpp)

# boost_serial.a
add_library(boost_serial STATIC ${BOOST_SERIAL_SRC})

# define include directories
target_include_directories(boost_serial PUBLIC include/)

# add dependency
target_link_libraries(boost_serial PUBLIC ${Boost_LIBRARIES})

# enable optimizations
target_compile_options(boost_serial PUBLIC -g -Wall -O0)

if(ENABLE_SAMPLE)
# globbing file
file(GLOB BYTE_TERMINAL_SRC sample/ByteTerminal.cpp)

# boost_serial.a
add_executable(ByteTerminal ${BYTE_TERMINAL_SRC})

# define include directories
target_include_directories(ByteTerminal PRIVATE include/)

# add dependency
target_link_libraries(ByteTerminal PUBLIC boost_serial ${Boost_LIBRARIES} pthread)

# enable optimizations
target_compile_options(ByteTerminal PUBLIC -g -Wall -O0)

# globbing file
file(GLOB STRING_TERMINAL_SRC sample/StringTerminal.cpp)

# boost_serial.a
add_executable(StringTerminal ${STRING_TERMINAL_SRC})

# define include directories
target_include_directories(StringTerminal PRIVATE include/)

# add dependency
target_link_libraries(StringTerminal PUBLIC boost_serial ${Boost_LIBRARIES} pthread)

# enable optimizations
target_compile_options(StringTerminal PUBLIC -Wall -O3)
endif()

if(ENABLE_TEST)
###################################################################
# enable ExternalProject
include(ExternalProject)
set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)

# Download and install GoogleTest
ExternalProject_Add(
    googletest
    URL https://github.com/google/googletest/archive/release-1.8.1.zip
    PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/external/gtest
    # Disable install step
    INSTALL_COMMAND ""
)

# Get GTest source and binary directories from CMake project
ExternalProject_Get_Property(googletest source_dir binary_dir)

# Create a libgtest target to be used as a dependency by test programs
add_library(gtest_main IMPORTED STATIC GLOBAL)
add_dependencies(gtest_main googletest)

# Set libgtest properties
set_target_properties(gtest_main PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest_main.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# Create a libgtest target to be used as a dependency by test programs
add_library(gtest IMPORTED STATIC GLOBAL)
add_dependencies(gtest googletest)

# Set libgtest properties
set_target_properties(gtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# Create a libgmock target to be used as a dependency by test programs
add_library(gmock_main IMPORTED STATIC GLOBAL)
add_dependencies(gmock_main googletest)

# Set libgmock properties
set_target_properties(gmock_main PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# Create a libgmock target to be used as a dependency by test programs
add_library(gmock IMPORTED STATIC GLOBAL)
add_dependencies(gmock googletest)

# Set libgmock properties
set_target_properties(gmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
include_directories("${source_dir}/googletest/include"
                    "${source_dir}/googlemock/include")


# globbing file
file(GLOB TEST_SRC test/*.cpp)

add_executable(rungtest ${TEST_SRC})
target_link_libraries(rungtest PUBLIC boost_serial boost_system pthread gtest gtest_main)
target_compile_options(rungtest PUBLIC -Wall -O3 -Wno-sign-compare)

enable_testing()
add_test(rungtest rungtest)
endif()
