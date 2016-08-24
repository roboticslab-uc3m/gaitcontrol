
##cmake code start
project(gaitcontrol)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(./src SRC_LIST)


# Pick up our cmake modules - they are all in the cmake subdirectory
set(TEMPLATE_NAME_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${TEMPLATE_NAME_MODULE_PATH})

#libraries
set(SRC_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
#gait
FIND_PACKAGE(gait)
if(GAIT_FOUND)
    message(STATUS "Gait library found.")
else()
    #Use the source code if not installed
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait)
    set(GAIT_LIBRARIES ${GAIT_LIBRARY} )
    set(GAIT_INCLUDE_DIRS ${GAIT_INCLUDE_DIR} )
endif()
INCLUDE_DIRECTORIES(${GAIT_INCLUDE_DIRS})

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
target_link_libraries( ${PROJECT_NAME} ${GAIT_LIBRARIES} teoMWI SensorIntegration)


##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()



#echo_all_cmake_variable_values()


