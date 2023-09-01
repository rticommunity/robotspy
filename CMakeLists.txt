cmake_minimum_required(VERSION 3.8)
project(robotspy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(DEFINED CONNEXTDDS_DIR)
  set(nddshome "${CONNEXTDDS_DIR}")
elseif(DEFINED ENV{CONNEXTDDS_DIR})
  set(nddshome "$ENV{CONNEXTDDS_DIR}")
elseif(DEFINED NDDSHOME)
  set(nddshome "${NDDSHOME}")
elseif(DEFINED ENV{NDDSHOME})
  set(nddshome "$ENV{NDDSHOME}")
endif()
set(CONNEXTDDS_DIR "${nddshome}" CACHE INTERNAL "RTI Connext DDS installation")
list(APPEND CMAKE_MODULE_PATH "${CONNEXTDDS_DIR}/resource/cmake")
find_package(RTIConnextDDS REQUIRED)

function(generate_c_typesupport idl_file)
  cmake_parse_arguments(_idl
    "" # boolean arguments
    "PACKAGE" # single value arguments
    "" # multi-value arguments
    ${ARGN} # current function arguments
  )

  set(idl_out_dir "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen")
  get_filename_component(idl_filename "${idl_file}" NAME)
  string(REGEX REPLACE "\.idl$" "" idl_base "${idl_filename}")
  if(NOT "${_idl_PACKAGE}" STREQUAL "")
    set(idl_ns "${_idl_PACKAGE}/")
  endif()
  set(idl_generated
    "${idl_out_dir}/${idl_ns}${idl_base}.c"
    "${idl_out_dir}/${idl_ns}${idl_base}.h"
    "${idl_out_dir}/${idl_ns}${idl_base}Support.c"
    "${idl_out_dir}/${idl_ns}${idl_base}Support.h"
    "${idl_out_dir}/${idl_ns}${idl_base}Plugin.c"
    "${idl_out_dir}/${idl_ns}${idl_base}Plugin.h"
  )
  find_program(rtiddsgen_bin
    NAMES rtiddsgen
    PATHS "${CONNEXTDDS_DIR}/bin"
  )
  add_custom_command(
    OUTPUT ${idl_generated}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${idl_out_dir}/${idl_ns}
    COMMAND ${rtiddsgen_bin} -language C
                             -replace
                             -unboundedSupport
                             -d ${idl_out_dir}/${idl_ns}
                             -I "${CMAKE_CURRENT_SOURCE_DIR}/idl"
                             ${idl_file}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS ${idl_file}
  )

  string(REGEX REPLACE "/" "_" out_var "${idl_ns}${idl_base}_FILES")
  set(${out_var} ${idl_generated} PARENT_SCOPE)
endfunction()

generate_c_typesupport(idl/dds_request_reply.idl
  PACKAGE ${PROJECT_NAME})

set(LIB_NAME ${PROJECT_NAME}helpers)

add_library(${LIB_NAME}
  src/base_input_emitter.cpp
  src/base_output_emitter.cpp
  src/base_type_monitor.cpp
  src/cli.cpp
  src/dds_input_emitter.cpp
  src/log.cpp
  src/typecache.cpp
  src/typesupport.cpp
  ${${PROJECT_NAME}_dds_request_reply_FILES}
  include/robotspy/base_input_emitter.hpp
  include/robotspy/base_output_emitter.hpp
  include/robotspy/base_type_monitor.hpp
  include/robotspy/cli.hpp
  include/robotspy/dds_input_emitter.hpp
  include/robotspy/input_emitter.hpp
  include/robotspy/log_default.hpp
  include/robotspy/log.hpp
  include/robotspy/output_emitter.hpp
  include/robotspy/typecache.hpp
  include/robotspy/typecodes.hpp
  include/robotspy/typesupport.hpp
  include/robotspy/visibility_control.h
)
target_compile_features(${LIB_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${LIB_NAME}
  RTIConnextDDS::c_api
)
target_compile_definitions(${LIB_NAME} PRIVATE "ROBOTSPY_BUILDING_LIBRARY")

ament_target_dependencies(${LIB_NAME}
  rcutils
  rcpputils
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  ament_index_cpp
)

add_executable(types_scraper_cpp
  src/types_scraper_main.cpp
)
target_link_libraries(types_scraper_cpp
  ${LIB_NAME}
  RTIConnextDDS::cpp2_api
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/${PROJECT_NAME}/dds_request_reply.h
  DESTINATION include/${PROJECT_NAME}/
)
install(
  TARGETS
    ${LIB_NAME}
    types_scraper_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY scripts/robotspy
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/types_scraper
    scripts/types_converter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIB_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
