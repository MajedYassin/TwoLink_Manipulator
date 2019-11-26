cmake_minimum_required (VERSION 3.15)

include(FetchContent)


add_subdirectory(Bot)
add_subdirectory(Solvers)
add_subdirectory(OperationInterface)

##Solver cpp directories
set(SOLVER_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/InvKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/FKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TorqueCtrl.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TrpzTrajectory.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/VelocityKinematics.cpp")
set(BOT_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Bot/Sbot.cpp")

##Solver header files
set(SOLVER_LIST "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/InvKinematics.h"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/FKinematics.h"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TrpzTrajectory.h"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TorqueCtrl.h"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/FKinematics.h")

##Bot and OperationInterface cpp files
set(INTERFACE_DIR "${TwoLink_manip_SOURCE_DIR}/Source/OperationInterface/OpInterface.cpp")

##Bot and OperationInterface header files
set(INTERFACE_LIST "${TwoLink_manip_SOURCE_DIR}/Source/OperationInterface/OpInterface.h")
set(BOT_LIST "${TwoLink_manip_SOURCE_DIR}/Source/Bot/Sbot.h")
set(STATE_HEADER "${TwoLink_manip_SOURCE_DIR}/Source/Bot/State.h")

##Common cpp and header file
set(COMMON_DIR "${TwoLink_manip_SOURCE_DIR}/Source/Common/Common.cpp")
set(COMMON_LIST "${TwoLink_manip_SOURCE_DIR}/Source/Common/Common.h" OperationInterface/OpInterface.cpp)


add_library(source_library ${SOLVER_DIR} ${SOLVER_LIST} ${INTERFACE_DIR} ${BOT_LIST} ${STATE_HEADER} ${COMMON_DIR}
        ${COMMON_LIST} ${BOT_DIR} ${INTERFACE_LIST})



target_link_libraries(source_library PUBLIC  gtest gtest_main eigen)
