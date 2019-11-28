cmake_minimum_required (VERSION 3.0)

project(TwoLink_manip)

include_directories(Source)

<<<<<<< Updated upstream
add_executable(run_tests Pendulum_test)
=======
add_executable(run_tests Test_lib)
>>>>>>> Stashed changes

target_link_libraries(run_tests PUBLIC source_library)