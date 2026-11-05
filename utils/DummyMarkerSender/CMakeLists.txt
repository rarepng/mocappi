 # Define the minimum CMAKE version required:
cmake_minimum_required(VERSION 3.11)

if (DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
endif ()
# set vcpkg target triplet to silence some warnings
if (DEFINED ENV{VCPKG_DEFAULT_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "")
endif ()

# Define the project:
project(DummyMarkerSender C CXX)

# Set C++ stardard:
set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)
find_package(Boost 1.71.0 REQUIRED)
find_package(Threads REQUIRED)

add_executable(DummyMarkerSender main.cpp)

target_compile_definitions(DummyMarkerSender PUBLIC -DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(DummyMarkerSender PRIVATE Boost::boost Threads::Threads OpenMP::OpenMP_CXX)

# MSVC links this automatically
if(WIN32)
    target_compile_definitions(DummyMarkerSender PRIVATE _WIN32_WINNT=0x0A00)
    target_link_libraries(DummyMarkerSender PRIVATE ws2_32 mswsock)
endif()


target_include_directories(DummyMarkerSender PRIVATE ${Boost_INCLUDE_DIRS})
