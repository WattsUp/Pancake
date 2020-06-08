cmake_minimum_required(VERSION 3.13)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Default to CMAKE_BUILD_TYPE = Release unless overridden on command line
if( DEFINED CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Set to either \"Release\" or \"Debug\"" )
else()
    set( CMAKE_BUILD_TYPE Release CACHE STRING "Set to either \"Release\" or \"Debug\"" )
endif()

if(WIN32 AND NOT CMAKE_TOOLCHAIN_FILE)
  message(FATAL_ERROR "CMAKE_TOOLCHAIN_FILE must be set, see vcpkg integrate install")
endif()

project("pancake")

# Global setting: Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile commands for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

if( WIN32 )
  # define UNICODE and_UNICODE definition on Windows
  # Both definitions are required
  add_definitions(-DUNICODE -D_UNICODE)
endif()

# Allow target_sources to use relative paths
cmake_policy(SET CMP0076 NEW)

# Allow install CODE|SCRIPT to use generator expressions
cmake_policy(SET CMP0087 NEW)

# Output binaries to ./bin
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/output/debug")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/output/release")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/output/debug")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/output/release")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/output/debug")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/output/release")

# Targets not included in the public distributable
set (TARGETS_NODIST
  "test"          # Unit tester common to all projects
  "installer-win" # Windows self-extracting installer
)

set (TARGETS_NODISTCONFIG
  "default" # test
  "default" # installer-win
)

# Targets included in the public distributable
set (TARGETS_DIST
  "stack"
)

set (TARGETS_DISTCONFIG
  "default" # stack
)

list(APPEND TARGETS ${TARGETS_NODIST})
list(APPEND TARGETS ${TARGETS_DIST})
list(APPEND TARGETS_CONFIG ${TARGETS_NODISTCONFIG})
list(APPEND TARGETS_CONFIG ${TARGETS_DISTCONFIG})

list(LENGTH TARGETS_CONFIG TARGETS_LENGTH)
math(EXPR TARGETS_LENGTH "${TARGETS_LENGTH} - 1")

# Remove existing /W0-4 flag before adding the desired one
string(REGEX REPLACE " /W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
string(REGEX REPLACE " /W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# Add the suffix "d" to debug builds
set(CMAKE_DEBUG_POSTFIX "d")

# Update semantic version from git tag
# Script executed every time to check for changes
set(VERSION_FILE "${CMAKE_SOURCE_DIR}/common/version.h")
set(VERSION_DEPENDENCY "${VERSION_FILE}.notgenerated")
find_package(Python3 COMPONENTS Interpreter)
add_custom_command(
  OUTPUT "${VERSION_DEPENDENCY}"
  BYPRODUCTS "${VERSION_FILE}"
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_SOURCE_DIR}/tools/CreateVersionFromGitTag.py"
    --output "${VERSION_FILE}"
    --quiet
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating version file"
)

foreach(INDEX RANGE ${TARGETS_LENGTH})
  list(GET TARGETS ${INDEX} TARGET)
  list(GET TARGETS_CONFIG ${INDEX} CONFIG)

  if (${CONFIG} MATCHES "^WinMain$" AND WIN32)
    # Looks for int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char* args, int nShowCmd)
    add_executable(${TARGET} WIN32)
  elseif(NOT ${TARGET} MATCHES "^.*-win$" OR WIN32)
    # Looks for int main(int argc, char* argv[])
    add_executable(${TARGET})
    if (WIN32)
      target_compile_definitions(${TARGET} PRIVATE WIN_CONSOLE)
    endif()
  else()
    message("Windows only, excluding from all: ${TARGET}")
    add_executable(${TARGET} EXCLUDE_FROM_ALL)
  endif()

  # Add the root as an include directory: i.e. #include "common/logging.hpp" from any level
  target_include_directories(${TARGET} PRIVATE ${CMAKE_SOURCE_DIR})

  # Set debug flag when building debug release, add a suffix to output files
  target_compile_definitions(${TARGET} PUBLIC "$<$<CONFIG:DEBUG>:DEBUG>")
  set_target_properties(${TARGET} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

  # Make the version file a dependency so it always gets updated
  target_sources(${TARGET} PRIVATE "${VERSION_DEPENDENCY}")

  # Set compiler options for warnings (treat as errors)
  target_compile_options(${TARGET} PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wall>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wextra>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wconversion>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>)

  add_custom_command(TARGET ${TARGET} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/res/ $<TARGET_FILE_DIR:${TARGET}>)

endforeach()

foreach(TARGET ${TARGETS_NODIST})
  set_target_properties(${TARGET} PROPERTIES EXCLUDE_FROM_ALL ON)
endforeach()


# Generate doxygen when building release type builds
find_package(Doxygen)
add_custom_target(
  "doc"
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_SOURCE_DIR}/tools/UpdateDoxygen.py"
    --doxygen "${DOXYGEN_EXECUTABLE}"
    --doxygen-output "${CMAKE_SOURCE_DIR}/docs/project.doxyfile"
    --project-name "${PROJECT_NAME}"
    --project-brief "${PROJECT_DESCRIPTION}"
    --quiet
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating API documentation with Doxygen"
  DEPENDS "${VERSION_DEPENDENCY}"
)

# Add support for Mingw RC compiler
if(WIN32)
  enable_language(RC)
  include(CMakeDetermineRCCompiler)

  if(MINGW)
    set(CMAKE_RC_COMPILER_INIT windres)
    set(CMAKE_RC_COMPILE_OBJECT "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -I${CMAKE_CURRENT_BINARY_DIR} -i <SOURCE> -o <OBJECT>")
  endif()
endif()

# CMake targets cannot have spaces, to have spaces in the output use the OUTPUT_NAME property
set_target_properties("installer-win" PROPERTIES OUTPUT_NAME "${PROJECT_NAME}-installer")

# Add libraries first to setup library include directives
add_subdirectory("libraries")

# Include public interface (for a library)
include_directories("include")

# Add each subdirectory
add_subdirectory("common")
add_subdirectory("installer-win")

# Add each target subdirectory
add_subdirectory("project-stack")

set(EXES "")

foreach(TARGET ${TARGETS_DIST})
  if (WIN32)
    install(TARGETS ${TARGET}
            RUNTIME DESTINATION .
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION .)
    set(EXE $<TARGET_FILE:${TARGET}>)
    list(APPEND EXES ${EXE})
  else()
    install(TARGETS ${TARGET}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib/static)
  endif()
endforeach()

install(DIRECTORY "res" DESTINATION .)
# file(GLOB RESOURCES "res/*")
# file(COPY ${RESOURCES} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
# file(COPY ${RESOURCES} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A template repository for C++ projects")
set(CPACK_PACKAGE_VENDOR "Bradley Davis")
set(CPACK_PACKAGE_CONTACT "Bradley Davis me@bradleydavis.tech")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "1")

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "any")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")

if(WIN32)
  set(CPACK_GENERATOR "ZIP")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
  set(CPACK_PACKAGE_DIRECTORY "${CMAKE_SOURCE_DIR}")
  set(CPACK_PACKAGE_FILE_NAME "archive")
endif()

include(CPack)

if (WIN32)
  install(CODE
    "file(GET_RUNTIME_DEPENDENCIES
            RESOLVED_DEPENDENCIES_VAR resolved_deps
            UNRESOLVED_DEPENDENCIES_VAR unresolved_deps
            EXECUTABLES ${EXES}
            PRE_EXCLUDE_REGEXES \"api-ms.*\"
            POST_INCLUDE_REGEXES \"${CMAKE_SOURCE_DIR}.*\"
            POST_EXCLUDE_REGEXES \".*\"
            )
    file(COPY \${resolved_deps} DESTINATION \"${CPACK_PACKAGE_DIRECTORY}/_CPack_Packages/win32/ZIP/archive\")
    file(INSTALL \${resolved_deps} DESTINATION \"${CMAKE_INSTALL_PREFIX}\")"
  )
endif()