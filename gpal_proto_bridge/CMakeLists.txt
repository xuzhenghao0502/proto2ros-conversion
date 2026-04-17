cmake_minimum_required(VERSION 3.12)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(gpal_proto_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(proto2ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)

get_executable_path(PROTOC_EXECUTABLE protobuf::protoc CONFIGURE)

get_filename_component(_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
get_filename_component(PROTO_INTERFACE_DIR "${_REPO_ROOT}/proto-interface" ABSOLUTE)
if(NOT EXISTS "${PROTO_INTERFACE_DIR}")
  message(FATAL_ERROR "proto-interface not found at ${PROTO_INTERFACE_DIR}")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/gpal_proto_seeds.cmake")

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gpal_proto_seeds.cmake"
)

foreach(_s ${GPAL_PROTO_SEEDS})
  set(_p "${PROTO_INTERFACE_DIR}/${_s}")
  if(NOT EXISTS "${_p}")
    message(FATAL_ERROR "GPAL proto seed missing: ${_p}")
  endif()
endforeach()

# Single-file input so we can use --dependency_out (protoc rejects it with multiple inputs).
set(_AGG "${CMAKE_CURRENT_BINARY_DIR}/gpal_seed_aggregator.proto")
set(_agg_content "syntax = \"proto2\";\n")
foreach(_s ${GPAL_PROTO_SEEDS})
  string(APPEND _agg_content "import \"${_s}\";\n")
endforeach()
string(APPEND _agg_content "message GpalProtoClosureRoot {}\n")
file(WRITE "${_AGG}" "${_agg_content}")

set(GPAL_CLOSURE_DESC "${CMAKE_CURRENT_BINARY_DIR}/gpal_closure.desc")
set(GPAL_CLOSURE_DEP "${CMAKE_CURRENT_BINARY_DIR}/gpal_closure.d")
execute_process(
  COMMAND ${PROTOC_EXECUTABLE}
    -I${PROTO_INTERFACE_DIR}
    -I${CMAKE_CURRENT_BINARY_DIR}
    --include_imports
    --include_source_info
    -o${GPAL_CLOSURE_DESC}
    --dependency_out=${GPAL_CLOSURE_DEP}
    gpal_seed_aggregator.proto
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND_ERROR_IS_FATAL ANY
)

# Makefile-style deps: paths separated by " \n" (backslash-newline). No Python/protobuf needed.
file(READ "${GPAL_CLOSURE_DEP}" _dep_raw)
string(REPLACE "\\\n" " " _dep_raw "${_dep_raw}")
string(STRIP "${_dep_raw}" _dep_raw)
string(REGEX REPLACE "[ \t\n]+" ";" _proto_items "${_dep_raw}")
set(GPAL_PROTO_FILES "")
foreach(_p ${_proto_items})
  if(_p MATCHES "gpal_seed_aggregator\\.proto$")
    # Synthetic; no need for a generated _pb2 module.
  elseif(_p MATCHES "\\.proto$")
    list(APPEND GPAL_PROTO_FILES "${_p}")
  endif()
endforeach()
list(REMOVE_DUPLICATES GPAL_PROTO_FILES)

list(LENGTH GPAL_PROTO_FILES _n_proto)
if(_n_proto EQUAL 0)
  message(FATAL_ERROR "proto closure is empty (protoc seeds produced no files)")
endif()

set(PROTO_PYTHON_OUT "${CMAKE_CURRENT_BINARY_DIR}/proto_python")
file(MAKE_DIRECTORY "${PROTO_PYTHON_OUT}")
file(TOUCH "${PROTO_PYTHON_OUT}/__init__.py")

set(_proto_py_sources "")
foreach(_proto ${GPAL_PROTO_FILES})
  file(RELATIVE_PATH _relpath "${PROTO_INTERFACE_DIR}" "${_proto}")
  string(REPLACE ".proto" "_pb2.py" _gen "${_relpath}")
  list(APPEND _proto_py_sources "${PROTO_PYTHON_OUT}/${_gen}")
  get_filename_component(_rel_dir "${_relpath}" DIRECTORY)
  if(NOT _rel_dir STREQUAL "")
    file(MAKE_DIRECTORY "${PROTO_PYTHON_OUT}/${_rel_dir}")
    file(TOUCH "${PROTO_PYTHON_OUT}/${_rel_dir}/__init__.py")
  endif()
endforeach()
# Descriptor set includes gpal_seed_aggregator.proto; proto2ros imports its _pb2 module when generating conversions.
list(APPEND _proto_py_sources "${PROTO_PYTHON_OUT}/gpal_seed_aggregator_pb2.py")

add_custom_command(
  OUTPUT ${_proto_py_sources}
  COMMAND ${PROTOC_EXECUTABLE}
    -I${PROTO_INTERFACE_DIR}
    -I${CMAKE_CURRENT_BINARY_DIR}
    --python_out=${PROTO_PYTHON_OUT}
    ${GPAL_PROTO_FILES}
    ${_AGG}
  DEPENDS ${GPAL_PROTO_FILES} ${_AGG}
  COMMENT "Generating Python protobuf (_pb2.py) for GPAL proto closure"
  VERBATIM
)

add_custom_target(${PROJECT_NAME}_proto_gen ALL DEPENDS ${_proto_py_sources})

set(PROTO_OUT_DIR "${PROTO_PYTHON_OUT}")

proto2ros_generate(
  ${PROJECT_NAME}_messages_gen
  PROTO_DESCRIPTORS "${GPAL_CLOSURE_DESC}"
  CONFIG_OVERLAYS "${CMAKE_CURRENT_SOURCE_DIR}/config/proto2ros_overlay.yaml"
  INTERFACES_OUT_VAR ros_messages
  PYTHON_OUT_VAR py_sources
  CPP_OUT_VAR cpp_sources
  INCLUDE_OUT_VAR cpp_include_dir
  APPEND_PYTHONPATH "${PROTO_OUT_DIR}"
  DEPENDS ${PROJECT_NAME}_proto_gen
  NO_LINT
)

rosidl_generate_interfaces(
  ${PROJECT_NAME} ${ros_messages}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs proto2ros std_msgs
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_messages_gen)

rosidl_generated_python_package_add(
  ${PROJECT_NAME}_additional_modules
  MODULES ${_proto_py_sources} ${py_sources}
  PACKAGES ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
)

install(FILES ${GPAL_CLOSURE_DESC} DESTINATION share/${PROJECT_NAME} RENAME gpal_proto_bridge_messages_gen.desc)

# protoc --python_out tree (map_server/, hal/, …) must be on sys.path for generated conversions.py imports.
install(DIRECTORY "${PROTO_PYTHON_OUT}/" DESTINATION share/${PROJECT_NAME}/pb2_py)

install(PROGRAMS
  scripts/db2bag
  scripts/bag_image_midframe
  scripts/db_peek_camera_frame
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  proto2ros
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_package()
