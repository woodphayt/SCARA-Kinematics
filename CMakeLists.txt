cmake_minimum_required(VERSION 2.8.3)

project(ah3_demo)

include_directories(${PROJECT_SOURCE_DIR}/include/
                    ${PROJECT_SOURCE_DIR}/src/)

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/ah3_demo.cpp
            ${PROJECT_SOURCE_DIR}/src/Scara.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/Scara.h)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})