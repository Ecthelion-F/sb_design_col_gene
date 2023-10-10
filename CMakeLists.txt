cmake_minimum_required(VERSION 3.21)
project(Gurobi_Demo)

set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}\\lib")
set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}\\include")

set(CMAKE_CXX_STANDARD 20)

#MATH(EXPR stack_size "5000*1024*1024")
#set(CMAKE_EXE_LINKER_FLAGS "-Wl,--stack,${stack_size}")

add_executable(Gurobi_Demo main.cpp ArcSets.cpp ArcSets.h LineSets.cpp LineSets.h UsefulConstants.h ODSets.cpp ODSets.h MainModel.cpp MainModel.h LppModel.cpp LppModel.h)

message(STATUS "GUROBI_LIB_DIR: ${GUROBI_LIB_DIR}")
message(STATUS "GUROBI_INCLUDE_DIR: ${GUROBI_INCLUDE_DIR}")

link_directories(${GUROBI_LIB_DIR})
include_directories(${GUROBI_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} "${GUROBI_LIB_DIR}\\gurobi100.lib" "${GUROBI_LIB_DIR}\\gurobi_c++mdd2017.lib")