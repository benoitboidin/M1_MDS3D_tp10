# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.8)
   message(FATAL_ERROR "CMake >= 2.8.0 required")
endif()
if(CMAKE_VERSION VERSION_LESS "2.8.3")
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.8.3...3.23)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_cmake_targets_defined "")
set(_cmake_targets_not_defined "")
set(_cmake_expected_targets "")
foreach(_cmake_expected_target IN ITEMS ::glbinding)
  list(APPEND _cmake_expected_targets "${_cmake_expected_target}")
  if(TARGET "${_cmake_expected_target}")
    list(APPEND _cmake_targets_defined "${_cmake_expected_target}")
  else()
    list(APPEND _cmake_targets_not_defined "${_cmake_expected_target}")
  endif()
endforeach()
unset(_cmake_expected_target)
if(_cmake_targets_defined STREQUAL _cmake_expected_targets)
  unset(_cmake_targets_defined)
  unset(_cmake_targets_not_defined)
  unset(_cmake_expected_targets)
  unset(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT _cmake_targets_defined STREQUAL "")
  string(REPLACE ";" ", " _cmake_targets_defined_text "${_cmake_targets_defined}")
  string(REPLACE ";" ", " _cmake_targets_not_defined_text "${_cmake_targets_not_defined}")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_cmake_targets_defined_text}\nTargets not yet defined: ${_cmake_targets_not_defined_text}\n")
endif()
unset(_cmake_targets_defined)
unset(_cmake_targets_not_defined)
unset(_cmake_expected_targets)


# Create imported target ::glbinding
add_library(::glbinding STATIC IMPORTED)

set_target_properties(::glbinding PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "\$<\$<NOT:\$<BOOL:OFF>>:GLBINDING_STATIC_DEFINE>;SYSTEM_DARWIN"
  INTERFACE_COMPILE_OPTIONS "-Wall;-Wextra;-Wunused;-Wreorder;-Wignored-qualifiers;-Wmissing-braces;-Wreturn-type;-Wswitch;-Wswitch-default;-Wuninitialized;-Wmissing-field-initializers;\$<\$<CXX_COMPILER_ID:GNU>:;-Wmaybe-uninitialized;\$<\$<VERSION_GREATER:\$<CXX_COMPILER_VERSION>,4.8>:;-Wpedantic;-Wreturn-local-addr;>;>;\$<\$<CXX_COMPILER_ID:Clang>:;-Wpedantic;-Wreturn-stack-address;>;\$<\$<PLATFORM_ID:Darwin>:;-pthread;>"
  INTERFACE_INCLUDE_DIRECTORIES "/Users/benoitboidin/Desktop/s8_info/3d/tp10/mds3d_td10/ext/glbinding/include;/Users/benoitboidin/Desktop/s8_info/3d/tp10/mds3d_td10/build/ext/glbinding/include"
  INTERFACE_LINK_LIBRARIES "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX13.1.sdk/System/Library/Frameworks/OpenGL.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX13.1.sdk/System/Library/Frameworks/OpenGL.framework"
)

# Import target "::glbinding" for configuration ""
set_property(TARGET ::glbinding APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(::glbinding PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "/Users/benoitboidin/Desktop/s8_info/3d/tp10/mds3d_td10/build/ext/glbinding/libglbinding.a"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
