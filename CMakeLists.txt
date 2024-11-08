# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(MachiningSimulation)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set RPATH settings
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Locate the carve library and include directories
find_package(Boost REQUIRED)
find_package(carve REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(VTK REQUIRED)
find_package(Vulkan REQUIRED)

# Specify include directories
# include_directories(${Boost_INCLUDE_DIRS})
include_directories(${carve_INCLUDE_DIRS})
# message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
# message(STATUS "Include directories: ${carve_INCLUDE_DIRS}")

# Add the executable for the main source file
add_executable(main src/main.cpp ${LZ4_LIBRARY} ${TOOL_FILES})

# Link the carve library
target_link_libraries(main PRIVATE carve::carve ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# Optional: Add debugging and optimization flags
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
