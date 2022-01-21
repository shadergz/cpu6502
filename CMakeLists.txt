cmake_minimum_required (VERSION 3.8)

project (cpu6502 VERSION 0.0.1)

if (UNIX)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

else ()
if (MSVC)
# The Microsoft compiler doesn't include the headers "format", "ranges", "chrono" 
# under c++ 20 specification
	add_compile_options (/std:c++latest)
endif ()
endif ()

add_executable (cpu6502 "test_cpu6502.cc" "cpu6502.cc")
