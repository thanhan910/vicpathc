# Minimum CMake version
cmake_minimum_required(VERSION 3.30)

# Project name and language
project(VicpathcServer LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vcpkg integration (optional, but recommended)
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

# Add include directories
include_directories(/vcpkg/installed/x64-linux/include)

# Link library directories
link_directories(/vcpkg/installed/x64-linux/lib)

# Find required libraries via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBPQXX REQUIRED libpqxx)
pkg_check_modules(LIBPQ REQUIRED libpq)
pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx-static)

# Collect all source files recursively
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/*.cpp")

# # Filter out files in the CMakeFiles directory
# list(FILTER SOURCES EXCLUDE REGEX ".*CMakeFiles/.*")

# Filter out files in the build directory
list(FILTER SOURCES EXCLUDE REGEX "${CMAKE_BINARY_DIR}/.*")

# Define the executable
add_executable(local-server ${SOURCES})

# Link libraries
target_link_libraries(local-server PRIVATE
    ${LIBPQXX_LIBRARIES}
    ${LIBPQ_LIBRARIES}
    ${LIBMONGOCXX_LIBRARIES}
    pthread  # Explicitly link pthread for static linking
)

# Include directories for libraries and project subfolders
target_include_directories(local-server PRIVATE
    ${LIBPQXX_INCLUDE_DIRS}
    ${LIBPQ_INCLUDE_DIRS}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}  # Include all project directories
)

# Add link directories explicitly if necessary
target_link_directories(local-server PRIVATE
    ${LIBPQXX_LIBRARY_DIRS}
    ${LIBPQ_LIBRARY_DIRS}
    ${LIBMONGOCXX_LIBRARY_DIRS}
)
