cmake_minimum_required(VERSION 2.8.11)

add_compile_options(-std=c++11 -O3 -fopenmp)
project(pcl_visualizer)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)

find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(libnabo REQUIRED)

find_package ( OpenMP)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(project_SOURCES src/main.cpp src/pclviewer.cpp src/interface_handler.cpp src/control_handler.cpp src/sliders_handler.cpp src/algorithms.cpp src/helpers.cpp)

# Compilation of the GUI
add_executable(${PROJECT_NAME} ${project_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen Qt5::Widgets libnabo::nabo)

# add_executable(cloud_analyzer src/cloud_analyzer.cpp src/helpers.cpp)
# target_link_libraries(cloud_analyzer ${PCL_LIBRARIES} Eigen3::Eigen)

# Compilation of the optimizer 
add_executable(camera_calibration_optimization src/camera_calibration_optimization.cpp src/helpers.cpp)
target_link_libraries(camera_calibration_optimization ${PCL_LIBRARIES} Eigen3::Eigen)

