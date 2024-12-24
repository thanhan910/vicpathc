# Minimum CMake version
cmake_minimum_required(VERSION 3.30)

# Project name and language
project(VicpathcServer LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directories
include_directories(/vcpkg/installed/x64-linux/include)

# Link library directories
link_directories(/vcpkg/installed/x64-linux/lib)

# Find required libraries via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBPQXX REQUIRED libpqxx)
pkg_check_modules(LIBPQ REQUIRED libpq)
pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx-static)

# Collect all source files
file(GLOB SOURCES 
    "*.cpp"
)

# Define the executable
add_executable(local-server ${SOURCES})

# Link libraries
target_link_libraries(local-server
    ${LIBPQXX_LIBRARIES}
    ${LIBPQ_LIBRARIES}
    ${LIBMONGOCXX_LIBRARIES}
)

# Include directories for libraries
target_include_directories(local-server PRIVATE
    ${LIBPQXX_INCLUDE_DIRS}
    ${LIBPQ_INCLUDE_DIRS}
    ${LIBMONGOCXX_INCLUDE_DIRS}
)

# Add link directories
target_link_directories(local-server PRIVATE
    ${LIBPQXX_LIBRARY_DIRS}
    ${LIBPQ_LIBRARY_DIRS}
    ${LIBMONGOCXX_LIBRARY_DIRS}
)
