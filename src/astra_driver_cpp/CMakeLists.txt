cmake_minimum_required(VERSION 3.10)
project(astra_driver_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENNI2_ROOT
    /home/anup/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux
)

set(OPENNI2_INCLUDE_DIR
    ${OPENNI2_ROOT}/sdk/Include
)

set(OPENNI2_LIBRARY
    ${OPENNI2_ROOT}/sdk/libs/libOpenNI2.so
)

include_directories(${OPENNI2_INCLUDE_DIR})

add_executable(astra_driver
    src/astra_driver.cpp
)

# 🔑 FORCE exact library + embed runtime path
target_link_libraries(astra_driver
    ${OPENNI2_LIBRARY}
    pthread
)

set_target_properties(astra_driver PROPERTIES
    BUILD_RPATH ${OPENNI2_ROOT}/sdk/libs
    INSTALL_RPATH ${OPENNI2_ROOT}/sdk/libs
)

