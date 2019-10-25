cmake_minimum_required (VERSION 3.0)

project(TwoLink_manip)

include(FetchContent)

FetchContent_declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.0
)

FetchContent_MakeAvailable(googletest)

include_directories(Source)

set(TEST_SOURCES "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/InvKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/FKinematics.cpp"
        "${TwoLink_manip_SOURCE_DIR}/Source/Solvers/TorqueCtrl.cpp")

add_executable(run_tests FKinematics_test.cpp)

##add_test(run_tests FKinematics_test)

target_link_libraries(run_tests PUBLIC gtest gtest_main)