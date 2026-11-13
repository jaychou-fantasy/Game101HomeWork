cmake_minimum_required(VERSION 3.10)

# Allow this directory to be configured directly as well as from Assignment8.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RopeSim)
  find_package(OpenGL REQUIRED)
  find_package(Threads REQUIRED)
  find_package(Freetype REQUIRED)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../CGL" "${CMAKE_CURRENT_BINARY_DIR}/CGL")
  set(CGL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../CGL/include")
endif()

# Application source
set(APPLICATION_SOURCE
    rope.cpp
    application.cpp
    main.cpp
)

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${GLEW_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${CGL_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${GLEW_LIBRARY_DIRS}
  ${GLFW_LIBRARY_DIRS}
  ${CGL_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_executable(ropesim ${APPLICATION_SOURCE})

target_link_libraries( ropesim
    CGL ${CGL_LIBRARIES}
    glew ${GLEW_LIBRARIES}
    glfw ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET ropesim APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Keep the executable in the configured build directory.
set_target_properties(ropesim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Install to project root
install(TARGETS ropesim DESTINATION ${RopeSim_SOURCE_DIR})
