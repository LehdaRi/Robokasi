cmake_minimum_required(VERSION 2.6)
project(robokasi)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)
find_package(GLEW REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2 REQUIRED system window graphics network audio)
find_package(OpenGL REQUIRED)

link_libraries(
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${SFML_LIBRARIES}
)

include_directories(
    ${GLEW_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(robokasi ${SRC_FILES})
target_compile_features(robokasi PRIVATE cxx_range_for)
