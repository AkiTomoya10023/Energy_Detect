cmake_minimum_required(VERSION 3.5)

project(Energy_Detect)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(X11 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${X11_INCLUDE_DIRS})

add_executable(Energy_Detect 
            # EnergyDetect.cpp
            ${PROJECT_SOURCE_DIR}/Main/main.cpp
            ${PROJECT_SOURCE_DIR}/Main/ImageUpdating.cpp
            ${PROJECT_SOURCE_DIR}/Main/EnergyDetecting.cpp
            ${PROJECT_SOURCE_DIR}/Main/MeseagesUpdating.cpp
            ${PROJECT_SOURCE_DIR}/Src/RunEnergy_Detector.cpp
            ${PROJECT_SOURCE_DIR}/Include/RunEnergy_Detector.hpp
            ${PROJECT_SOURCE_DIR}/Src/RunEnergy_Predictor.cpp
            ${PROJECT_SOURCE_DIR}/Include/RunEnergy_Predictor.hpp
            ${PROJECT_SOURCE_DIR}/Src/RunEnergy_AngleSolve.cpp
            ${PROJECT_SOURCE_DIR}/Include/RunEnergy_AngleSolve.hpp
            ${PROJECT_SOURCE_DIR}/Filter/particle_filter.cpp
            ${PROJECT_SOURCE_DIR}/Filter/particle_filter.h
	        ${PROJECT_SOURCE_DIR}/GxCamera/GxCamera.cpp
            ${PROJECT_SOURCE_DIR}/GxCamera/GxCamera.hpp
	        ${PROJECT_SOURCE_DIR}/GxCamera/Include/DxImageProc.h
	        ${PROJECT_SOURCE_DIR}/GxCamera/Include/GxIAPI.h
  	        ${PROJECT_SOURCE_DIR}/Serial/Serial.cpp
	        ${PROJECT_SOURCE_DIR}/Serial/Serial.hpp
	        ${PROJECT_SOURCE_DIR}/General/general.hpp
            )

target_link_libraries(Energy_Detect ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES}
            yaml-cpp  ${CERES_LIBRARIES} ${CSPARSE_LIBRARY} ${X11_LIBRARIES} glog
	    -lgxiapi -lpthread)

