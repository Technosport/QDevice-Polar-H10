cmake_minimum_required(VERSION 3.20)
project(qdevice_polar_h10 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(QDEVICE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../qdevice-ximu3/third_party/qdevice_sdk" CACHE PATH "Path to Qualisys QDevice SDK")

add_library(polar_core STATIC
    src/PolarPacketParser.cpp
    src/PolarSampleBuffer.cpp
    src/PolarBleClientWinrt.cpp
)
target_include_directories(polar_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if (WIN32)
    target_link_libraries(polar_core PUBLIC windowsapp)
endif ()

enable_testing()
add_executable(polar_h10_tests tests/test_polar_parser.cpp)
target_link_libraries(polar_h10_tests PRIVATE polar_core)
add_test(NAME polar_h10_tests COMMAND polar_h10_tests)

if (WIN32 AND EXISTS "${QDEVICE_SDK_DIR}/api/qualisys/qdevice-api/qdevice_api.h")
    add_library(qdevice_polar_h10 SHARED src/QDevicePolarPlugin.cpp)
    target_include_directories(qdevice_polar_h10 PRIVATE "${QDEVICE_SDK_DIR}/api")
    target_link_libraries(qdevice_polar_h10 PRIVATE polar_core)
endif ()
