cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
find_package(nlohmann_json 3.2.0 REQUIRED)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
include_directories(

        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/
        ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/
        ${PROJECT_SOURCE_DIR}/Thirdparty/eigen-3.1.3/
        ${PROJECT_SOURCE_DIR}/utils
        ${PROJECT_SOURCE_DIR}/slam/
        ${PROJECT_SOURCE_DIR}/tello
        ${PROJECT_SOURCE_DIR}/algo
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}
        slam/src/System.cc
        slam/src/Tracking.cc
        slam/src/LocalMapping.cc
        slam/src/ORBextractor.cc
        slam/src/ORBmatcher.cc
        slam/src/FrameDrawer.cc
        slam/src/Sim3Solver.cc
        slam/src/Converter.cc
        slam/src/MapPoint.cc
        slam/src/KeyFrame.cc
        slam/src/Map.cc
        slam/src/MapDrawer.cc
        slam/src/Optimizer.cc
        slam/src/PnPsolver.cc
        slam/src/Frame.cc
        slam/src/KeyFrameDatabase.cc
        slam/src/Initializer.cc
        slam/src/Viewer.cc
        slam/src/LoopClosing.cc
        algo/src/DBSCAN.cpp
        tello/src/AutonomousDrone.cpp
        algo/src/Graph.cpp
        algo/src/Polygon.cpp
        utils/src/Point.cpp
        algo/src/Pizza.cpp
        algo/src/RRT.cpp
        utils/src/Line.cpp
        utils/src/Node.cpp
        utils/src/Frame.cpp
        utils/src/Room.cpp
        utils/src/Auxiliary.cpp
        tello/src/Charger.cpp
        algo/src/Navigation.cpp)


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        -lpthread ctello spdlog ${PYTHON_LIBRARIES} -lbluetooth
        )

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exe)

# add_executable(mono_tello
#         Examples/Monocular/mono_tello.cc)
# target_link_libraries(mono_tello PUBLIC ${PROJECT_NAME})
# target_compile_options(mono_tello PRIVATE -Wno-deprecated-declarations) # -Wall -Werror -Wall -Wextra)

#add_executable(tello
#        exe/tello.cc tello/include/gpio.h tello/src/gpio.cpp)
#target_link_libraries(tello PUBLIC ${PROJECT_NAME})
add_executable(testPolygon exe/testPolygon.cpp)
target_link_libraries(testPolygon ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)

add_executable(testNavigationPathing exe/testNavigationPathing.cpp)
target_link_libraries(testNavigationPathing ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)
add_executable(runAutonomousDrone exe/runAutonomousDrone.cpp)
target_link_libraries(runAutonomousDrone ${PROJECT_NAME} nlohmann_json::nlohmann_json)

add_executable(cameraCalibretion exe/cameraCalibretion.cpp)
target_link_libraries(cameraCalibretion ${PROJECT_NAME} nlohmann_json::nlohmann_json)
add_executable(runCharger exe/runCharger.cpp)
target_link_libraries(runCharger ${PROJECT_NAME})
add_executable(offline_orb_slam
        exe/offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

add_executable(mapCreating
        exe/mapCreating.cpp)
target_link_libraries(mapCreating ${PROJECT_NAME})
add_executable(testDroneCommunication exe/testDroneCommunication.cpp)
target_link_libraries(testDroneCommunication ctello spdlog Threads::Threads)
#target_compile_options(offline_orb_slam PRIVATE -Wno-deprecated-declarations)
#add_executable(computeScaleFactor
#       Examples/Monocular/computeScaleFactor.cc)
#target_link_libraries(computeScaleFactor PUBLIC ${PROJECT_NAME})
#target_compile_options(computeScaleFactor PRIVATE -Wno-deprecated-declarations)
# -Wall -Werror -Wall -Wextra)

# add_executable(mono_tello_debug_save
#         Examples/Monocular/mono_tello_debug_save.cc)
# target_link_libraries(mono_tello_debug PUBLIC ${PROJECT_NAME})
# target_compile_options(mono_tello_debug PRIVATE -Wno-deprecated-declarations) # -Wall -Werror -Wall -Wextra)
