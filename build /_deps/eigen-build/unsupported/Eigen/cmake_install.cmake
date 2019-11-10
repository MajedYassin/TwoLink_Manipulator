# Install script for directory: /home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/AdolcForward"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/AlignedVector3"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/ArpackSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/AutoDiff"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/BVH"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/EulerAngles"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/FFT"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/IterativeSolvers"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/KroneckerProduct"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/LevenbergMarquardt"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/MatrixFunctions"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/MoreVectorization"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/MPRealSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/NonLinearOptimization"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/NumericalDiff"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/OpenGLSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/Polynomials"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/Skyline"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/SparseExtra"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/SpecialFunctions"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

