cmake_minimum_required(VERSION 2.8)
project(F2F_ICP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# FreeImage
find_package(FreeImage REQUIRED)
include_directories(${FREEIMAGE_INCLUDE_DIRS})

# Flann
find_package(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Set files to be compiled
set(HEADER_FILES 
    src/Eigen.h 
    src/SimpleMesh.h 
    src/PointCloud.h 
    src/VirtualSensor.h 
    src/NearestNeighbor.h 
    src/ProcrustesAligner.h 
    src/ICPOptimizer.h 
    src/FreeImageHelper.h
)
set(SOURCE_FILES 
    src/FreeImageHelper.cpp
)

add_executable(F2F_ICP src/main.cpp ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(F2F_ICP ${FREEIMAGE_LIBRARIES} ${FLANN_LIBRARIES} ${CERES_LIBRARIES})
