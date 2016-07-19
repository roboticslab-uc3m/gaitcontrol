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
#Change directory here if Gait instalation is not found
set(GAIT_LOCATION /usr/local)
message(STATUS "Looking for local Gait in: [${GAIT_LOCATION}].")
# TODO:force to refresh find_library every run
find_library(gait_LIBRARY NAMES gait libgait PATHS "${GAIT_LOCATION}/lib/gait/")
if(${gait_LIBRARY} STREQUAL "gait_LIBRARY-NOTFOUND")
    #Try to use the source code if not installed
    message(STATUS "Gait not installed. Trying to compile gait from sources")
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/gait)
    set(GAIT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/gait/src ${GAIT_INCLUDE_DIR})
    set(gait_LIBRARY "gait")
else()
    #Try to use the local lib if installed
    message(STATUS "Gait library found. Using local files")
    FIND_PATH(GAIT_INCLUDE_DIR Gait.h /usr/local/include/gait/)
    message(STATUS "Local Gait files detected: [${gait_LIBRARY}].")
    message(STATUS "Local include directories: [${GAIT_INCLUDE_DIR}].")
endif()
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
target_link_libraries( ${PROJECT_NAME} ${gait_LIBRARY} teoMWI SensorIntegration)
#echo_all_cmake_variable_values()


