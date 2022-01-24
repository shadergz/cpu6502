cmake_minimum_required (VERSION 3.8)

project (cpu6502 VERSION 0.0.2)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (fmt)

if (MSVC)
# The Microsoft compiler doesn't include the headers "format", "ranges", "chrono" 
# under c++ 20 specification
	add_compile_options (/std:c++latest)
endif ()

add_executable (cpu6502_test "test_cpu6502.cc" "cpu6502.cc")

target_link_libraries (cpu6502_test fmt::fmt)
#set_property (TARGET cpu6502_test PROPERTY CXX_STANDARD 20)
