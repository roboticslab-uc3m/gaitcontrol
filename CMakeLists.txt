<<<<<<< HEAD
cmake_minimum_required (VERSION 2.6.0)
project(spgait)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(spgait SHARED spgait.cpp)
set_target_properties(spgait PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(spgait ${OpenRAVE_LIBRARIES})
install(TARGETS spgait DESTINATION .)
=======
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


>>>>>>> e9144d76f174944607410778740e7054b6184334
