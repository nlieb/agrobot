cmake_minimum_required(VERSION 2.8.3)
project(detector)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/darknet/")
message(STATUS "Darknet path dir = ${DARKNET_PATH}")

# Find CUDA
find_package(CUDA)

if (CUDA_FOUND)
  #Get CUDA compute capability

  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${ARCH}")
  add_definitions(-DGPU)

  #Add library path to linker
  link_directories(
    "${CUDA_TOOLKIT_ROOT_DIR}/lib64"
  )
endif()

if (NOT CUDA_FOUND OR NOT ${CUDA_SUCCESS})
  message( FATAL_ERROR "Could not detect CUDA. CMake will exit." )
endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  src
# ${catkin_INCLUDE_DIRS}
)

set(DARKNET_SOURCE
  ${DARKNET_PATH}/src/activation_layer.c
  ${DARKNET_PATH}/src/activations.c
  ${DARKNET_PATH}/src/art.c
  ${DARKNET_PATH}/src/avgpool_layer.c
  ${DARKNET_PATH}/src/batchnorm_layer.c
  ${DARKNET_PATH}/src/blas.c
  ${DARKNET_PATH}/src/box.c
  ${DARKNET_PATH}/src/captcha.c
  ${DARKNET_PATH}/src/cifar.c
  ${DARKNET_PATH}/src/classifier.c
  ${DARKNET_PATH}/src/coco.c
  ${DARKNET_PATH}/src/col2im.c
  ${DARKNET_PATH}/src/compare.c
  ${DARKNET_PATH}/src/connected_layer.c
  ${DARKNET_PATH}/src/convolutional_layer.c
  ${DARKNET_PATH}/src/cost_layer.c
  ${DARKNET_PATH}/src/cpu_gemm.c
  ${DARKNET_PATH}/src/crnn_layer.c
  ${DARKNET_PATH}/src/crop_layer.c
  ${DARKNET_PATH}/src/cuda.c
  ${DARKNET_PATH}/src/darknet.c
  ${DARKNET_PATH}/src/data.c
  ${DARKNET_PATH}/src/deconvolutional_layer.c
  ${DARKNET_PATH}/src/demo.c
  ${DARKNET_PATH}/src/detection_layer.c
  ${DARKNET_PATH}/src/detector.c
  ${DARKNET_PATH}/src/dice.c
  ${DARKNET_PATH}/src/dropout_layer.c
  ${DARKNET_PATH}/src/gemm.c
  ${DARKNET_PATH}/src/getopt.c
  ${DARKNET_PATH}/src/go.c
  ${DARKNET_PATH}/src/gru_layer.c
  ${DARKNET_PATH}/src/im2col.c
  ${DARKNET_PATH}/src/image.c
  ${DARKNET_PATH}/src/layer.c
  ${DARKNET_PATH}/src/list.c
  ${DARKNET_PATH}/src/local_layer.c
  ${DARKNET_PATH}/src/matrix.c
  ${DARKNET_PATH}/src/maxpool_layer.c
  ${DARKNET_PATH}/src/network.c
  ${DARKNET_PATH}/src/nightmare.c
  ${DARKNET_PATH}/src/normalization_layer.c
  ${DARKNET_PATH}/src/option_list.c
  ${DARKNET_PATH}/src/parser.c
  ${DARKNET_PATH}/src/region_layer.c
  ${DARKNET_PATH}/src/reorg_layer.c
  ${DARKNET_PATH}/src/reorg_old_layer.c
  ${DARKNET_PATH}/src/rnn.c
  ${DARKNET_PATH}/src/rnn_layer.c
  ${DARKNET_PATH}/src/rnn_vid.c
  ${DARKNET_PATH}/src/route_layer.c
  ${DARKNET_PATH}/src/shortcut_layer.c
  ${DARKNET_PATH}/src/softmax_layer.c
  ${DARKNET_PATH}/src/super.c
  ${DARKNET_PATH}/src/swag.c
  ${DARKNET_PATH}/src/tag.c
  ${DARKNET_PATH}/src/tree.c
  ${DARKNET_PATH}/src/upsample_layer.c
  ${DARKNET_PATH}/src/utils.c
  ${DARKNET_PATH}/src/voxel.c
  ${DARKNET_PATH}/src/writing.c
  ${DARKNET_PATH}/src/yolo.c
  ${DARKNET_PATH}/src/yolo_layer.c
  )


###################
## Yolo Detector ##
###################

set(DARKNET_KERNELS
  ${DARKNET_PATH}/src/activation_kernels.cu
  ${DARKNET_PATH}/src/avgpool_layer_kernels.cu
  ${DARKNET_PATH}/src/blas_kernels.cu
  ${DARKNET_PATH}/src/col2im_kernels.cu
  ${DARKNET_PATH}/src/convolutional_kernels.cu
  ${DARKNET_PATH}/src/crop_layer_kernels.cu
  ${DARKNET_PATH}/src/deconvolutional_kernels.cu
  ${DARKNET_PATH}/src/dropout_layer_kernels.cu
  ${DARKNET_PATH}/src/im2col_kernels.cu
  ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
  ${DARKNET_PATH}/src/network_kernels.cu
  )

include_directories(
  ${DARKNET_PATH}/src
)

# create yolo detector library
cuda_add_library(yolo_detector
  src/yolo_v2_class.cpp
  src/yolo_detector.hpp

  ${DARKNET_SOURCE}
  ${DARKNET_KERNELS}
  )

add_dependencies(yolo_detector
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(yolo_detector
  m
  pthread
  stdc++
  cuda
  cudart
  cublas
  curand
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/detector_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
