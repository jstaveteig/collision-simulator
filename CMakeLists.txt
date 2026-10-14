cmake_minimum_required(VERSION 3.24)
project(sweep VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

include(FetchContent)
find_package(Threads REQUIRED)
FetchContent_Declare(glm
    URL https://github.com/g-truc/glm/archive/0af55ccecd98d4e5a8d1fad7de25ba429d60e863.tar.gz
    SOURCE_SUBDIR header_only DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_Declare(tinyobjloader
    URL https://github.com/tinyobjloader/tinyobjloader/archive/2945a967c5303b2c8c14174117c45f3302591150.tar.gz
    SOURCE_SUBDIR header_only DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(glm tinyobjloader)

add_library(sweep STATIC src/geometry.cpp src/collision.cpp)
add_library(sweep::sweep ALIAS sweep)
target_compile_features(sweep PUBLIC cxx_std_20)
target_include_directories(sweep PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/sweep>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sweep SYSTEM PUBLIC $<BUILD_INTERFACE:${glm_SOURCE_DIR}>)
target_include_directories(sweep SYSTEM PRIVATE ${tinyobjloader_SOURCE_DIR})
target_link_libraries(sweep PUBLIC Threads::Threads)

option(SWEEP_BUILD_DEMO "Build the visual application" ON)
if(SWEEP_BUILD_DEMO)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    set(CUSTOMIZE_BUILD ON CACHE BOOL "" FORCE)
    set(SUPPORT_MODULE_RAUDIO OFF CACHE BOOL "" FORCE)
    set(SUPPORT_CLIPBOARD_IMAGE OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
        URL https://github.com/raysan5/raylib/archive/c1ab645ca298a2801097931d1079b10ff7eb9df8.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(raylib)
    add_executable(sweep_demo src/main.cpp)
    target_link_libraries(sweep_demo PRIVATE sweep raylib)
    if(MSVC)
        target_link_options(sweep_demo PRIVATE /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup)
    endif()
    add_custom_command(TARGET sweep_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:sweep_demo>/assets
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/factory.obj $<TARGET_FILE_DIR:sweep_demo>/assets/factory.obj)
endif()

foreach(targetName sweep sweep_demo)
    if(TARGET ${targetName})
        if(MSVC)
            target_compile_options(${targetName} PRIVATE /W4 /permissive- /EHsc)
        else()
            target_compile_options(${targetName} PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    endif()
endforeach()

include(CMakePackageConfigHelpers)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/sweepConfig.cmake
    "include(CMakeFindDependencyMacro)\nfind_dependency(Threads)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/sweepTargets.cmake\")\n")
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/sweepConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(TARGETS sweep EXPORT sweepTargets ARCHIVE DESTINATION lib)
install(EXPORT sweepTargets NAMESPACE sweep:: DESTINATION lib/cmake/sweep)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sweepConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/sweepConfigVersion.cmake
    DESTINATION lib/cmake/sweep)
install(FILES src/collision.hpp DESTINATION include/sweep)
install(DIRECTORY ${glm_SOURCE_DIR}/glm DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.inl" PATTERN "*.h")
configure_file(cmake/CollectLicenses.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/CollectLicenses.cmake" @ONLY)
install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/CollectLicenses.cmake")
