cmake_minimum_required(VERSION 3.20)
project(qdevice_polar_h10 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(QDEVICE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../qdevice-ximu3/third_party/qdevice_sdk" CACHE PATH "Path to Qualisys QDevice SDK")

add_library(polar_core STATIC
    src/PolarDiagnostics.cpp
    src/PolarPacketParser.cpp
    src/PolarSampleBuffer.cpp
    src/PolarBleClientWinrt.cpp
)
target_include_directories(polar_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if (MSVC)
    target_compile_options(polar_core PRIVATE /W4)
else ()
    target_compile_options(polar_core PRIVATE -Wall -Wextra -Wpedantic)
endif ()

find_package(Threads REQUIRED)
target_link_libraries(polar_core PUBLIC Threads::Threads)
if (WIN32)
    target_link_libraries(polar_core PUBLIC windowsapp)
endif ()

enable_testing()
add_executable(polar_h10_tests tests/test_polar_parser.cpp)
target_link_libraries(polar_h10_tests PRIVATE polar_core)
if (MSVC)
    target_compile_options(polar_h10_tests PRIVATE /W4)
else ()
    target_compile_options(polar_h10_tests PRIVATE -Wall -Wextra -Wpedantic)
endif ()
add_test(NAME polar_h10_tests COMMAND polar_h10_tests)

add_executable(polar_h10_diag tools/polar_h10_diag.cpp)
target_link_libraries(polar_h10_diag PRIVATE polar_core)
if (MSVC)
    target_compile_options(polar_h10_diag PRIVATE /W4)
else ()
    target_compile_options(polar_h10_diag PRIVATE -Wall -Wextra -Wpedantic)
endif ()

if (EXISTS "${QDEVICE_SDK_DIR}/api/qualisys/qdevice-api/qdevice_api.h")
    add_library(qdevice_polar_h10 SHARED src/QDevicePolarPlugin.cpp)
    target_include_directories(qdevice_polar_h10 PRIVATE "${QDEVICE_SDK_DIR}/api")
    target_link_libraries(qdevice_polar_h10 PRIVATE polar_core)
    if (MSVC)
        target_compile_options(qdevice_polar_h10 PRIVATE /W4)
    else ()
        target_compile_options(qdevice_polar_h10 PRIVATE -Wall -Wextra -Wpedantic)
    endif ()
endif ()
