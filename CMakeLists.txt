cmake_minimum_required(VERSION 3.16)

project(cpu6502 VERSION 0.0.4)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
# The Microsoft compiler doesn't include the headers "format", "ranges", "chrono" 
# under c++ 20 specification
	add_compile_options(/std:c++latest)
endif()

find_package(fmt)

list(APPEND CPU6502_SRC ${CMAKE_CURRENT_SOURCE_DIR}/6502.cc)
list(APPEND CPU6502_HEADERS_ONLY ${CMAKE_CURRENT_SOURCE_DIR}/6502_Header.hh)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(lib6502 STATIC ${CPU6502_SRC})
target_link_libraries(lib6502 fmt::fmt)

add_subdirectory(test)
