cmake_minimum_required (VERSION 3.16)

project (cpu6502 VERSION 0.0.3)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
# The Microsoft compiler doesn't include the headers "format", "ranges", "chrono" 
# under c++ 20 specification
	add_compile_options (/std:c++latest)
endif ()

find_package (fmt)

set (CPU6502_SRC 
	${CMAKE_CURRENT_SOURCE_DIR}/cpu6502.cc
)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory (test)
