cmake_minimum_required(VERSION 3.19)

project(capture_to_image LANGUAGES CXX)

add_executable(${PROJECT_NAME} capture_to_image.cpp lodepng.cpp lodepng.h)

set(CLANG_WARNINGS
        -Wfatal-errors
        -Wall
        -Wextra
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        )

set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -Wmisleading-indentation
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wuseless-cast
        )

set(MSVC_WARNINGS /W3)

if (CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_WARNINGS ${CLANG_WARNINGS})
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS})
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(PROJECT_WARNINGS ${MSVC_WARNINGS})
else ()
    message(WARNING "No warnings set for compiler '${CMAKE_CXX_COMPILER_ID}'")
endif ()

target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_WARNINGS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

