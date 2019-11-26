cmake_minimum_required(VERSION 3.14)
project(TwoLink_manip)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)

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

add_subdirectory(Source)
add_subdirectory(App)
add_subdirectory(Test)





