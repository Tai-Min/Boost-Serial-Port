cmake_minimum_required(VERSION 2.6)


set (CMAKE_CXX_STANDARD 11)
add_executable(rungtest gtest.cpp)
target_link_libraries(rungtest boost_system pthread gtest)
