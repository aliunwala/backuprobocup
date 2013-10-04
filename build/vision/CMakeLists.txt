cmake_minimum_required(VERSION 2.8)

project(vision)

include(../common.cmake)

set(SRCS ${INTERFACE_DIR}/shared_vision/main.cpp)

qi_create_bin(vision ${SRCS})
qi_use_lib(vision core opencv2_core opencv2_highgui)
target_link_libraries(vision ${LINK_LIBS} ${LIBLUA} ${LIBYAML-CPP} ${LIBPYTHONSWIG} ${LIBBLOBS})
