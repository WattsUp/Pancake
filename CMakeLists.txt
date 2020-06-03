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

# Output binaries to ./bin
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin")

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
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Werror>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wall>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wextra>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wconversion>
    $<$<CXX_COMPILER_ID:MSVC>:/WX>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>)
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

# Create an archive of the output files, ready to distribute
set (ARCHIVE_FILES
  "${CMAKE_SOURCE_DIR}/bin/include"
)
foreach(TARGET ${TARGETS_DIST})
  list(APPEND ARCHIVE_FILES $<TARGET_FILE:${TARGET}>)
endforeach()

set (ARCHIVE_OUTPUT "${CMAKE_SOURCE_DIR}/bin/archive")

if(WIN32)
  add_custom_target(
    "archive"
    COMMAND ${CMAKE_COMMAND} -E tar "cfv" "${ARCHIVE_OUTPUT}.zip" --format=zip
      ${ARCHIVE_FILES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    COMMENT "Generating archive of output files"
    DEPENDS ${ARCHIVE_FILES}
  )
else()
  add_custom_target(
    "archive"
    COMMAND ${CMAKE_COMMAND} -E tar "cfvz" "${ARCHIVE_OUTPUT}.tar.gz"
      ${ARCHIVE_FILES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    COMMENT "Generating archive of output files"
    DEPENDS ${ARCHIVE_FILES}
  )
endif()

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
add_subdirectory("bin")

# Add each target subdirectory
add_subdirectory("project-stack")

target_link_libraries("test" gtest)

foreach(TARGET ${TARGETS})
  get_target_property(EXCLUDED ${TARGET} EXCLUDE_FROM_ALL)
  if (${EXCLUDED} STREQUAL "EXCLUDED-NOTFOUND")
    if (WIN32)
      install(TARGETS ${TARGET}
              RUNTIME DESTINATION .
              LIBRARY DESTINATION lib
              ARCHIVE DESTINATION .)
    else()
      install(TARGETS ${TARGET}
              RUNTIME DESTINATION bin
              LIBRARY DESTINATION lib
              ARCHIVE DESTINATION lib/static)
    endif()
  endif()
endforeach()

if(NOT WIN32)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A template repository for C++ projects")
  set(CPACK_PACKAGE_VENDOR "Bradley Davis")
  set(CPACK_PACKAGE_CONTACT "Bradley Davis me@bradleydavis.tech")
  set(CPACK_PACKAGE_VERSION_MAJOR "0")
  set(CPACK_PACKAGE_VERSION_MINOR "1")
  set(CPACK_PACKAGE_VERSION_PATCH "1")

  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "any")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "")

  include(CPack)
endif()