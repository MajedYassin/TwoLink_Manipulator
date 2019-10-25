cmake_minimum_required (VERSION 3.0)

include(FetchContent)

FetchContent_declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.0
)

FetchContent_Declare(
        catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v2.9.1
)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
        GIT_TAG 3.3.7
)

FetchContent_MakeAvailable(googletest)
FetchContent_MakeAvailable(catch2)
FetchContent_MakeAvailable(eigen)

#Work in Progress: Need to regularly update libraries
add_subdirectory(Bot)
add_subdirectory(Solvers)
add_subdirectory(OperationInterface)

##Solver cpp directories
set(SOLVER_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/InvKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/FKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TorqueCtrl.cpp")
set(TRAJ_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TrpzTrajectory.cpp")

##Struct Header Files
set(INT_DIR "${TwoLink_manip_SOURCE_DIR}/Source/OperationInterface/OpInterface.h")
set(BOT_LIST "${TwoLink_manip_SOURCE_DIR}/Source/Bot/Sbot.h")
set(STATE_HEADER "${TwoLink_manip_SOURCE_DIR}/Source/Bot/State.h")

##Common cpp and header file
set(COMMON_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Common/Common.cpp")
set(COMMON_LIST "${TwoLink_manip_SOURCE_DIR}/Source/Common/Common.h")


add_library(source_library ${SOLVER_DIR} ${SOLVER_LIST} ${INT_DIR} ${BOT_LIST} ${STATE_HEADER} ${COMMON_DIR} ${TRAJ_DIR})



target_link_libraries(source_library PUBLIC ${COMMON_LIST} gtest gtest_main Catch2::Catch2 eigen)
