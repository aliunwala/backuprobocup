cmake_minimum_required(VERSION 2.8)

# You need this to find the QiBuild CMake framework
find_package(qibuild QUIET)
include(qibuild/general)

# some base variables
set(NAO_HOME $ENV{NAO_HOME})
set(SRC_DIR ${NAO_HOME}/core)
set(BHWALK_DIR ${NAO_HOME}/core/motion/bhwalk)
set(LUA_DIR ${NAO_HOME}/lib/lua/src)
set(INTERFACE_DIR ${NAO_HOME}/interfaces)
set(LIBBHWALK ${CMAKE_CURRENT_BINARY_DIR}/../core/bhwalk/libbhwalk.a)
set(LIBLUA ${CMAKE_CURRENT_BINARY_DIR}/../lua/build/liblua.a)
set(LINK_LIBS boost_thread-mt boost_system-mt dl rt)
set(EIGEN_DIR /usr/include/eigen3)
set(YAML_DIR ${NAO_HOME}/lib/yaml-cpp/include)
set(NAOQI_ROOT ${NAO_HOME}/naoqi/crosstoolchain/atom/sysroot)
set(NAOQI_LIB ${NAOQI_ROOT}/usr/lib)
set(PYTHON_INCLUDE ${NAOQI_ROOT}/usr/include/python2.7)
set(LIBYAML-CPP ${CMAKE_CURRENT_BINARY_DIR}/../yaml-cpp/libyaml-cpp.a)
set(LIBPYTHONSWIG ${NAO_HOME}/build/pythonswig/lib_pythonswig_module.so -lutil)
set(LIBBLOBS ${NAO_HOME}/build/libblobs.a)

set(SWIG_LUA_CPP_DIR "${NAO_HOME}/build2/build/luaswig/cpp")
set(SWIG_LUA_I_DIR "${NAO_HOME}/build2/build/luaswig/i")

set(SWIG_PYTHON_CPP_DIR "${NAO_HOME}/build2/build/pythonswig/cpp")
set(SWIG_PYTHON_I_DIR "${NAO_HOME}/build2/build/pythonswig/i")

if($ENV{USER} STREQUAL "sbarrett") # for lab machines
  set(OPENCV2_CORE_INCLUDE_DIRS /usr/include/opencv-2.3.1/opencv2)
  set(OPENCV2_HIGHGUI_INCLUDE_DIRS /usr/include/opencv-2.3.1/opencv2/highgui)
  set(OPENCV_DIR /usr/include/opencv-2.3.1)
endif($ENV{USER} STREQUAL "sbarrett") # for lab machines

message("C Compiler: " ${CMAKE_C_COMPILER})
message("C++ Compiler: " ${CMAKE_CXX_COMPILER})

# set the include directory, the base dir, lua, bhuman's walk, and lib eigen
INCLUDE_DIRECTORIES(${SRC_DIR} ${LUA_DIR} ${BHWALK_DIR} ${EIGEN_DIR} ${YAML_DIR} ${OPENCV_DIR} ${PYTHON_INCLUDE})
