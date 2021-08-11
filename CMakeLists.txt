cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

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

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/tello
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Sim3Solver.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Initializer.cc
        src/Viewer.cc
        src/LoopClosing.cc
        tello/DBSCAN.cpp
        tello/AutonomousDrone.cpp
        tello/Polygon.cpp
        tello/Point.cpp
        tello/Pizza.cpp
        tello/Line.cpp
        tello/Frame.cpp
        tello/Room.cpp
        tello/Auxiliary.cpp
        tello/Charger.cpp
        )


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        -lpthread ctello spdlog ${PYTHON_LIBRARIES} -lbluetooth
        )

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tello
#         Examples/Monocular/mono_tello.cc)
# target_link_libraries(mono_tello PUBLIC ${PROJECT_NAME})
# target_compile_options(mono_tello PRIVATE -Wno-deprecated-declarations) # -Wall -Werror -Wall -Wextra)

add_executable(tello
        Examples/Monocular/tello.cc)
target_link_libraries(tello PUBLIC ${PROJECT_NAME})
add_executable(testPolygon tello/testPolygon.cpp)
target_link_libraries(testPolygon ${PROJECT_NAME})
target_compile_options(tello PRIVATE -Wno-deprecated-declarations)
add_executable(runAutonomousDrone tello/runAutonomousDrone.cpp)
target_link_libraries(runAutonomousDrone ${PROJECT_NAME})

add_executable(runCharger tello/runCharger.cpp)
target_link_libraries(runCharger ${PROJECT_NAME})
add_executable(offline_orb_slam
        Examples/Monocular/offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

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
