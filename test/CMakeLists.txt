include(FetchContent)

FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)

# For Windows: Prevent overriding the parent project compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)

find_package(fmt)

add_executable(test6502)
target_sources(test6502 PRIVATE test6502.cpp)

target_link_libraries(test6502 fmt::fmt gtest_main core6502)

gtest_discover_tests(test6502)
