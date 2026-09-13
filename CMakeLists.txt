cmake_minimum_required(VERSION 3.20)

project(SALAHC VERSION 1.0.0 LANGUAGES CXX)

include(CTest)
include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(GTest CONFIG QUIET)

if (NOT GTest_FOUND)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_Declare(
		googletest
		URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
		DOWNLOAD_EXTRACT_TIMESTAMP TRUE
	)
	FetchContent_MakeAvailable(googletest)
endif()

add_executable(SALAHC
	src/main.cpp
	src/graph/graph.cpp
	src/geometry/lbvh.cpp
	src/optimization/forces.cpp
	src/optimization/lahc.cpp
	src/optimization/sa.cpp
	src/optimization/pipeline.cpp)

target_compile_features(SALAHC PRIVATE cxx_std_20)

if (MSVC)
	target_compile_options(SALAHC PRIVATE /arch:AVX2)
else()
	target_compile_options(SALAHC PRIVATE -mavx2)
endif()

add_executable(SALAHC_tests
	tests/exact_math_graph_tests.cpp
	tests/parser_json_tests.cpp
	tests/lahc_tests.cpp
	tests/time_budget_tests.cpp
	tests/forces_mutators_tests.cpp
	tests/lbvh_tests.cpp
	src/graph/graph.cpp
	src/geometry/lbvh.cpp
	src/optimization/forces.cpp
	src/optimization/lahc.cpp
	src/optimization/sa.cpp
	src/optimization/pipeline.cpp
)

target_include_directories(SALAHC_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(SALAHC_tests PRIVATE cxx_std_20)

if (MSVC)
	target_compile_options(SALAHC_tests PRIVATE /arch:AVX2)
else()
	target_compile_options(SALAHC_tests PRIVATE -mavx2)
endif()

target_link_libraries(SALAHC_tests PRIVATE GTest::gtest_main)

add_test(NAME SALAHC_tests COMMAND SALAHC_tests)
