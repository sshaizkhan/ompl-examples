cmake_minimum_required(VERSION 3.10)
project(ompl_examples CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

# Variables to store OMPL paths
set(OMPL_INCLUDE_DIRS_FOUND "")
set(OMPL_LIBRARIES_FOUND "")

find_package(ompl QUIET)
if(ompl_FOUND)
  set(OMPL_INCLUDE_DIRS_FOUND ${OMPL_INCLUDE_DIRS})
  set(OMPL_LIBRARIES_FOUND ${OMPL_LIBRARIES})
else()
  # Try pkg-config as fallback
  pkg_check_modules(OMPL QUIET ompl)
  if(OMPL_FOUND)
    set(OMPL_INCLUDE_DIRS_FOUND ${OMPL_INCLUDE_DIRS})
    set(OMPL_LIBRARIES_FOUND ${OMPL_LIBRARIES})
  else()
    # Try local OMPL in workspace first
    get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
    get_filename_component(WORKSPACE_DEPS "${CURRENT_DIR}/../../../sorting_app_ws/src/deps" ABSOLUTE)
    set(LOCAL_OMPL_DIR "${WORKSPACE_DEPS}/3rd_party_lib/ompl")
    message(STATUS "Checking for local OMPL at: ${LOCAL_OMPL_DIR}")

    # Check if OMPL source headers exist (for header-only usage)
    if(EXISTS "${LOCAL_OMPL_DIR}/src/ompl/base/spaces/SE3StateSpace.h")
      message(STATUS "Using local OMPL headers from: ${LOCAL_OMPL_DIR}/src")
      set(OMPL_INCLUDE_DIRS_FOUND "${LOCAL_OMPL_DIR}/src")
      # Try to find library if built - check build/src/ompl first, then build/lib
      find_library(OMPL_LIBRARY
        NAMES ompl
        PATHS
          "${LOCAL_OMPL_DIR}/build/src/ompl"
          "${CMAKE_INSTALL_PREFIX}/lib"
          "${WORKSPACE_DEPS}/../install/lib"
          "${LOCAL_OMPL_DIR}/build/lib"
        NO_DEFAULT_PATH)
      if(OMPL_LIBRARY)
        message(STATUS "Found OMPL library: ${OMPL_LIBRARY}")
        set(OMPL_LIBRARIES_FOUND ${OMPL_LIBRARY})
        # OMPL may need std::filesystem (C++17)
        find_library(STDCPPFS_LIB stdc++fs PATHS /usr/lib/gcc/x86_64-linux-gnu/8 /usr/lib /usr/lib/x86_64-linux-gnu)
        if(STDCPPFS_LIB)
          list(APPEND OMPL_LIBRARIES_FOUND ${STDCPPFS_LIB})
        endif()
      else()
        message(STATUS "OMPL library not found, using headers only (may require linking OMPL later)")
      endif()
    else()
      # Check if OMPL is built and installed in workspace
      find_path(OMPL_INCLUDE_DIR ompl/base/spaces/SE3StateSpace.h
        PATHS
          "${CMAKE_INSTALL_PREFIX}/include"
          "${WORKSPACE_DEPS}/../install/include"
          /usr/include
          /usr/local/include
          /opt/ros2/foxy_ws/install/include
          /opt/ros/noetic/include
          /opt/ros/melodic/include
        PATH_SUFFIXES ompl)
      find_library(OMPL_LIBRARY
        NAMES ompl
        PATHS
          "${LOCAL_OMPL_DIR}/build/src/ompl"
          "${CMAKE_INSTALL_PREFIX}/lib"
          "${WORKSPACE_DEPS}/../install/lib"
          "${LOCAL_OMPL_DIR}/build/lib"
          /usr/lib
          /usr/local/lib
          /usr/lib/x86_64-linux-gnu
          /opt/ros2/foxy_ws/install/lib
          /opt/ros/noetic/lib
          /opt/ros/melodic/lib)
      if(OMPL_INCLUDE_DIR AND OMPL_LIBRARY)
        message(STATUS "Found OMPL manually: include=${OMPL_INCLUDE_DIR}, lib=${OMPL_LIBRARY}")
        set(OMPL_INCLUDE_DIRS_FOUND ${OMPL_INCLUDE_DIR})
        set(OMPL_LIBRARIES_FOUND ${OMPL_LIBRARY})
        # OMPL may need std::filesystem (C++17)
        find_library(STDCPPFS_LIB stdc++fs PATHS /usr/lib /usr/lib/x86_64-linux-gnu)
        if(STDCPPFS_LIB)
          list(APPEND OMPL_LIBRARIES_FOUND ${STDCPPFS_LIB})
        else()
          list(APPEND OMPL_LIBRARIES_FOUND c++fs)
        endif()
      else()
        message(STATUS "")
        message(STATUS "==========================================")
        message(STATUS "OMPL (Open Motion Planning Library) not found")
        message(STATUS "==========================================")
        message(STATUS "OMPL is required for dr_arm_planning but is not installed.")
        message(STATUS "")
        message(STATUS "To install OMPL:")
        message(STATUS "  1. Download from: https://github.com/ompl/ompl")
        message(STATUS "  2. Build and install:")
        message(STATUS "     mkdir build && cd build")
        message(STATUS "     cmake ..")
        message(STATUS "     make -j$(nproc)")
        message(STATUS "     sudo make install")
        message(STATUS "")
        message(STATUS "Or set CMake variables:")
        message(STATUS "  -DOMPL_INCLUDE_DIR=/path/to/ompl/include")
        message(STATUS "  -DOMPL_LIBRARY=/path/to/ompl/lib/libompl.so")
        message(STATUS "==========================================")
        message(STATUS "")
        message(FATAL_ERROR "OMPL is required for dr_arm_planning. Please install OMPL or set OMPL_INCLUDE_DIR and OMPL_LIBRARY")
      endif()
    endif()
  endif()
endif()
set(PKG_CONFIG_REQ_PUB "${PKG_CONFIG_REQ_PUB} ompl")

message(STATUS "OMPL version: ${ompl_VERSION}")
message(STATUS "OMPL include dirs: ${OMPL_INCLUDE_DIRS_FOUND}")
message(STATUS "OMPL libraries: ${OMPL_LIBRARIES_FOUND}")

# Example 1: 2D Point Robot
add_executable(point_robot src/point_robot.cpp)
target_link_libraries(point_robot PRIVATE ${OMPL_LIBRARIES_FOUND} Eigen3::Eigen Boost::system Boost::filesystem Boost::serialization)
target_include_directories(point_robot PRIVATE ${OMPL_INCLUDE_DIRS_FOUND})

# Example 2: SE3 Rigid Body Planning
add_executable(se3_planning src/se3_planning.cpp)
target_link_libraries(se3_planning PRIVATE ${OMPL_LIBRARIES_FOUND} Eigen3::Eigen Boost::system Boost::filesystem Boost::serialization)
target_include_directories(se3_planning PRIVATE ${OMPL_INCLUDE_DIRS_FOUND})

# Example 3: Custom State Space with Compound Space
add_executable(compound_space src/compound_space.cpp)
target_link_libraries(compound_space PRIVATE ${OMPL_LIBRARIES_FOUND} Eigen3::Eigen Boost::system Boost::filesystem Boost::serialization)
target_include_directories(compound_space PRIVATE ${OMPL_INCLUDE_DIRS_FOUND})

add_executable(robot_6dof_planning src/robot_6dof_planning.cpp)
target_link_libraries(robot_6dof_planning ${OMPL_LIBRARIES_FOUND} Eigen3::Eigen Boost::system Boost::filesystem Boost::serialization)
target_include_directories(robot_6dof_planning PRIVATE ${OMPL_INCLUDE_DIRS_FOUND})
