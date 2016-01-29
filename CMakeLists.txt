##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()

##cmake code start
project(gaitcontrol)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(./src SRC_LIST)

if(gait_FOUND)
  set(GAIT_INCLUDE_DIR ${gait_INCLUDE_DIRS})
else()
  message(STATUS "compiling gait from sources")
  add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait)
  set(GAIT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/gait/src ${GAIT_INCLUDE_DIR})

endif()

  INCLUDE_DIRECTORIES(${GAIT_INCLUDE_DIR})


add_executable(${PROJECT_NAME} ${SRC_LIST})

#link with libs
target_link_libraries( ${PROJECT_NAME} gait )
#echo_all_cmake_variable_values()


