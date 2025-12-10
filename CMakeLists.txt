cmake_minimum_required(VERSION 3.16)
project(piper_boot_init VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# 根据架构选择库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ARCH_DIR "x86_64")
else()
    message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}, trying aarch64")
    set(ARCH_DIR "aarch64")
endif()

# SDK 路径配置（现在在根目录下）
set(PIPER_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(PIPER_SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH_DIR}")

# 查找静态库或动态库
if(EXISTS "${PIPER_SDK_LIB_DIR}/libpiper_sdk.a")
    set(PIPER_SDK_LIB "${PIPER_SDK_LIB_DIR}/libpiper_sdk.a")
    message(STATUS "Found SDK (static): ${PIPER_SDK_LIB}")
elseif(EXISTS "${PIPER_SDK_LIB_DIR}/libpiper_sdk.so")
    set(PIPER_SDK_LIB "${PIPER_SDK_LIB_DIR}/libpiper_sdk.so")
    message(STATUS "Found SDK (shared): ${PIPER_SDK_LIB}")
else()
    message(FATAL_ERROR "Cannot find Piper SDK library in ${PIPER_SDK_LIB_DIR}")
endif()

# 主程序
add_executable(piper_boot_init piper_boot_init.cpp)
target_include_directories(piper_boot_init PRIVATE ${PIPER_SDK_INCLUDE_DIR})
target_link_libraries(piper_boot_init PRIVATE ${PIPER_SDK_LIB} pthread)

set_target_properties(piper_boot_init PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS piper_boot_init RUNTIME DESTINATION bin)

if(PIPER_SDK_LIB MATCHES "\\.so$")
    install(FILES ${PIPER_SDK_LIB} DESTINATION lib)
endif()
