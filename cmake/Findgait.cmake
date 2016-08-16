# - Try to find gait
# Once done this will define
#  GAIT_FOUND - System has libgait
#  GAIT_INCLUDE_DIRS - The libgait include directories
#  GAIT_LIBRARIES - The libraries needed to use libgait

#Default agit location is /usr/local
set(GAIT_LOCATION /usr/local)
message(STATUS "Looking for local Gait in: [${GAIT_LOCATION}].")

find_path(GAIT_INCLUDE_DIR Gait.h
    PATHS "${GAIT_LOCATION}/include/gait/"
    HINTS ${GAIT_LOCATION}
    PATH_SUFFIXES gait )

find_library(GAIT_LIBRARY NAMES gait libgait
    PATHS "${GAIT_LOCATION}/lib/gait/")

if(${GAIT_LIBRARY} STREQUAL "GAIT_LIBRARY-NOTFOUND")
    message(SEND_ERROR "Gait not installed. Find at: https://github.com/roboticslab-uc3m/gait ")
else()
    message(STATUS "Gait library found.")
    set (GAIT_FOUND TRUE)
    message(STATUS "Local Gait files detected: [${GAIT_LIBRARY}].")
    message(STATUS "Local include directories: [${GAIT_INCLUDE_DIR}].")
endif()

set(GAIT_LIBRARIES ${GAIT_LIBRARY} )
set(GAIT_INCLUDE_DIRS ${GAIT_INCLUDE_DIR} )


#mark_as_advanced(GAIT_INCLUDE_DIR GAIT_LIBRARY )
