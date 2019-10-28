cmake_minimum_required (VERSION 3.0)

project(TwoLink_manip)

FetchContent_declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.0
)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
        GIT_TAG 3.3.7
)

FetchContent_MakeAvailable(googletest)
FetchContent_MakeAvailable(eigen)

include_directories(Source)

add_executable(run_tests FKinematics_test)


target_link_libraries(run_tests PUBLIC fkine_lib)