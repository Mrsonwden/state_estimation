cmake_minimum_required(VERSION 3.10)
project(LG_KF_APP)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
 
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include
                                            "/usr/include/eigen3"
                                            ${OPENGL_INCLUDE_DIRS}
                                            ${GLUT_INCLUDE_DIRS}
                                            )

file(GLOB SOURCE "${PROJECT_SOURCE_DIR}/source/*.cpp"
                                        "${PROJECT_SOURCE_DIR}/app/*.cpp")

add_executable(LG_KF_APP  ${SOURCE})

target_link_libraries(LG_KF_APP ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})