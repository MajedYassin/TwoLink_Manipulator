# Install script for directory: /home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Cholesky"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/CholmodSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Core"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Dense"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Eigen"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Eigenvalues"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Geometry"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Householder"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/IterativeLinearSolvers"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Jacobi"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/LU"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/MetisSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/OrderingMethods"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/PaStiXSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/PardisoSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/QR"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/QtAlignedMalloc"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SPQRSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SVD"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/Sparse"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SparseCholesky"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SparseCore"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SparseLU"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SparseQR"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/StdDeque"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/StdList"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/StdVector"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/SuperLUSupport"
    "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/majed/CLionProjects/TwoLink_manip/build /_deps/eigen-src/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

