cmake_minimum_required(VERSION 2.8)

project(core)

include(../common.cmake)
include(../luaswig/luaswig.cmake)
include(../pythonswig/pythonswig.cmake)

set(IGNORE_REGEXS 
  "${SRC_DIR}/vision2/VisionModule.*"
  "${SRC_DIR}/core_wrap.cpp"
  ".*backup.*"
  ".*bak.*"
  "${BHWALK_DIR}/*"
)

set(LOG_DEFINES
  "debugLog"
  "visionLog"
  "ukfLog"
  "oppLog"
)

STRING(REGEX MATCH ".*geode.*" COMPILE_FOR_GEODE ${CMAKE_CURRENT_BINARY_DIR})

# collect all the files into a temp var
file(GLOB_RECURSE TEMP_SRCS 
  ${SRC_DIR}/*.cpp
  ${SRC_DIR}/*.h
  ${NAO_HOME}/lib/alglib/*.cpp
  ${NAO_HOME}/lib/alglib/*.h
)

SET(SRCS ${SWIG_SRCS})

# remove files from the excluded dirs and fill out the SRCS var
FOREACH(item ${TEMP_SRCS})
  set(excluded False)
  FOREACH(ignoreRegex ${IGNORE_REGEXS})
    STRING(REGEX MATCH ${ignoreRegex} excludedTemp ${item})
    IF (excludedTemp)
      set(excluded True)
      #message("IGNORE_REGEX: " ${ignoreRegex} " ignoring " ${item})
    ENDIF(excludedTemp)
  ENDFOREACH(ignoreRegex ${IGNORE_REGEXS})
  IF(NOT excluded)
    list(APPEND SRCS ${item})
  ENDIF(NOT excluded)
ENDFOREACH(item ${TEMP_SRCS})

if(COMPILE_FOR_GEODE)
  add_definitions( -DCOMPILE_FOR_GEODE )
endif(COMPILE_FOR_GEODE)

add_subdirectory(${BHWALK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/bhwalk)

qi_create_lib(core STATIC ${SRCS})
qi_stage_lib(core)
target_link_libraries(core ${LIBBLOBS})
add_dependencies(core ${LIBBLOBS})
