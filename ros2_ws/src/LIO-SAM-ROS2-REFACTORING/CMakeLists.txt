cmake_minimum_required(VERSION 3.5)
project(lio_sam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message(STATUS "Pixi Env: $ENV{CONDA_PREFIX}")
list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/lib/cmake")
link_directories(BEFORE "$ENV{CONDA_PREFIX}/lib")
include_directories("$ENV{CONDA_PREFIX}/include")

set(CMAKE_BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")

# --- TBB 찾기 (우선 CONFIG, 실패시 수동) ---
# conda-forge/tbb-devel 이 있으면 CONFIG 경로가 생깁니다.
find_package(TBB CONFIG QUIET)

if(NOT TBB_FOUND)
  # CONFIG가 없을 때: .pixi의 lib에서 직접 찾아 링크
  message(WARNING "TBB CONFIG not found; falling back to manual find_library in $ENV{CONDA_PREFIX}/lib")
  find_library(TBB_LIBRARY        tbb        HINTS "$ENV{CONDA_PREFIX}/lib")
  find_library(TBBMALLOC_LIBRARY  tbbmalloc  HINTS "$ENV{CONDA_PREFIX}/lib")
  if(NOT TBB_LIBRARY OR NOT TBBMALLOC_LIBRARY)
    message(FATAL_ERROR "tbb / tbbmalloc not found in $ENV{CONDA_PREFIX}/lib. 
    conda/pixi 환경에 tbb, tbb-devel 패키지가 있는지 확인하세요.")
  endif()
endif()

# Set policies
if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if (POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if (POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(geometry_msgs REQUIRED)

if (OpenMP_CXX_FOUND)
  set(USE_OPENMP OpenMP::OpenMP_CXX)
endif ()

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include/lio_sam
)

set(COMMON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/QoS.cpp")
set(PCL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/pclUtils.cpp")
set(GTSAM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/gtsamUtils.cpp")

set(COMMON_DEP1 rclcpp rclpy  tf2_eigen tf2_geometry_msgs pcl_conversions)
set(COMMON_DEP2 ${COMMON_DEP1} nav_msgs sensor_msgs std_msgs)


# featureExtraction
add_executable(${PROJECT_NAME}_featureExtraction
        src/nodes/nodeFE.cpp
        src/components/FeatureExtraction.cpp
        ${COMMON_SRC}
        ${PCL_SRC})
ament_target_dependencies(${PROJECT_NAME}_featureExtraction
        ${COMMON_DEP1}
        PCL)
target_link_libraries(${PROJECT_NAME}_featureExtraction
        "${cpp_typesupport_target}")


# imageProjection
add_executable(${PROJECT_NAME}_imageProjection
        src/nodes/nodeIP.cpp
        src/components/ImageProjection.cpp
        ${COMMON_SRC}
        ${PCL_SRC})
ament_target_dependencies(${PROJECT_NAME}_imageProjection
        ${COMMON_DEP2}
        PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection
        "${cpp_typesupport_target}")


# imuPreintegration
add_executable(${PROJECT_NAME}_imuPreintegration
        src/nodes/nodeIPI.cpp
        src/components/ImuPreintegration.cpp
        src/components/TransformFusion.cpp
        ${COMMON_SRC})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration
        ${COMMON_DEP2}
        GTSAM)
target_link_libraries(${PROJECT_NAME}_imuPreintegration
        gtsam
        "${cpp_typesupport_target}"
          $<IF:$<TARGET_EXISTS:TBB::tbb>,TBB::tbb,${TBB_LIBRARY}>
	  $<IF:$<TARGET_EXISTS:TBB::tbbmalloc>,TBB::tbbmalloc,${TBBMALLOC_LIBRARY}>
        )


# mapOptimization
add_executable(${PROJECT_NAME}_mapOptimization
        src/nodes/nodeMO.cpp
        src/components/MapOptmization.cpp
        src/components/MOInitializer.cpp
        src/components/MOVisualize.cpp
        src/components/MOLoopClosure.cpp
        src/components/MOPublish.cpp
        ${COMMON_SRC}
        ${PCL_SRC}
        ${GTSAM_SRC})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization
        ${COMMON_DEP2}
        PCL
        visualization_msgs
        tf2
        OpenCV
        GTSAM
        geometry_msgs)
target_link_libraries(${PROJECT_NAME}_mapOptimization
        gtsam
        "${cpp_typesupport_target}"
        ${USE_OPENMP}
          $<IF:$<TARGET_EXISTS:TBB::tbb>,TBB::tbb,${TBB_LIBRARY}>
	  $<IF:$<TARGET_EXISTS:TBB::tbbmalloc>,TBB::tbbmalloc,${TBBMALLOC_LIBRARY}>
        )


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

