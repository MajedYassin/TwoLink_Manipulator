cmake_minimum_required (VERSION 3.0)

project(TwoLink_manip)

include_directories(Source)

add_executable(run_tests TrpzTrajectory_test TorqueCtrl_test)

target_link_libraries(run_tests PUBLIC source_library)