cmake_minimum_required(VERSION 3.5)
project(robot_description VERSION 3.0.0)

find_package(ament_cmake REQUIRED)
file(GLOB ALL_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)
foreach(dir ${ALL_DIRS})
    string(REGEX REPLACE "/$" "" dir_name ${dir})
    if(NOT dir_name MATCHES "^(test|build|install|log|__pycache__)$" AND 
       IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir_name})
        
        message(STATUS "Installing robot: ${dir_name}")
        install(
            DIRECTORY ${dir_name}/
            DESTINATION share/${PROJECT_NAME}/${dir_name}
        )
    endif()
endforeach()

ament_package()