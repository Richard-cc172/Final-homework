cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        control_toolbox
        dynamic_reconfigure
        roslint
        geometry_msgs
        std_msgs
        tf
        nav_msgs
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)


generate_dynamic_reconfigure_options(
        cfg/PIDConfig.cfg
)

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
        INCLUDE_DIRS
        include
        ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
        ${EIGEN3_INCLUDE_DIR}
        LIBRARIES hero_chassis_controller
        ${PROJECT_NAME}_core
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        control_toolbox
        dynamic_reconfigure
        DEPENDS
        Boost
        tf
        nav_msgs
        pluginlib
        controller_interface
        ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
        ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
        # Eigen3

        ## Boost is not part of the DEPENDS since it is only used in source files,
        ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/devel/include
        # Set manually because Eigen sets a non standard INCLUDE DIR
        ${EIGEN3_INCLUDE_DIR}
        # Set because Boost is an internal dependency, not transitive.
        ${Boost_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/hero_chassis_controller_node.cpp
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

add_executable(keyboard_control_node src/keyboard_control_node.cpp)

target_link_libraries(keyboard_control_node ${catkin_LIBRARIES})

## Declare cpp executables


add_executable(velocity_transform_node
        src/velocity_transform_node.cpp
)
target_link_libraries(velocity_transform_node
        ${catkin_LIBRARIES}
        yaml-cpp
)


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_gencfg}
)


## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${pluginlib_LIBRARIES}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        DIRECTORY doc
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES simple_chassis_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS ${PROJECT_NAME} velocity_transform_node
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    #    catkin_add_gtest(${PROJECT_NAME}-test
    #            test/test_ros_package_template.cpp
    #            test/AlgorithmTest.cpp)
    #    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
