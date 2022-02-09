cmake_minimum_required (VERSION 3.14)

project (cpu6502 VERSION 0.0.3)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (fmt)

if (MSVC)
# The Microsoft compiler doesn't include the headers "format", "ranges", "chrono" 
# under c++ 20 specification
	add_compile_options (/std:c++latest)
endif ()

include (FetchContent)

FetchContent_Declare (
	googletest
	URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set (gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable (googletest)

enable_testing ()

add_executable (test_cpu6502 "test_cpu6502.cc" "cpu6502.cc")

target_link_libraries (test_cpu6502 fmt::fmt gtest_main)


include (GoogleTest)
gtest_discover_tests (test_cpu6502)
