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


#libraries
#gait
#find_package(gait REQUIRED)
if(gait_FOUND)
  set(GAIT_INCLUDE_DIR ${gait_INCLUDE_DIRS})
else()
  message(STATUS "Gait not installed. Trying to compile gait from sources")
  add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait)
  set(GAIT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/gait/src ${GAIT_INCLUDE_DIR})

endif(gait_FOUND)

  INCLUDE_DIRECTORIES(${GAIT_INCLUDE_DIR})

#teo MiddleWareInterface
message(STATUS "compiling teo MiddleWareInterface form sources")
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/teoMWI)
set(teoMWI_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/teoMWI ${teoMWI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${teoMWI_INCLUDE_DIR})

#Sensor Integration
message(STATUS "compiling SensorIntegration form sources")
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/sensor-integration)
set(SensorIntegration_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/sensor-integration ${SensorIntegration_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SensorIntegration_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${SRC_LIST})

#link with libs
target_link_libraries( ${PROJECT_NAME} gait teoMWI SensorIntegration)
#echo_all_cmake_variable_values()


