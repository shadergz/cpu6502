include (FetchContent)

FetchContent_Declare (
	googletest
	URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set (gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable (googletest)

enable_testing ()

include (GoogleTest)

find_package (fmt)

add_executable (test_cpu6502 "test_cpu6502.cc" ${CPU6502_SRC})

target_link_libraries (test_cpu6502 fmt::fmt gtest_main)

gtest_discover_tests (test_cpu6502)
