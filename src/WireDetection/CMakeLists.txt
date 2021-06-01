cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project (get_image_photoneo)

find_package(PCL 1.10) ########
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
FIND_FILE(ZMQLIB libzmq.so /usr/lib/x86_64-linux-gnu/)
find_package(OpenCV REQUIRED)
IF(NOT ZMQLIB)
    MESSAGE(SEND_ERROR "Ah.. Cannot find library libzmq.so.")
ENDIF(NOT ZMQLIB)

include_directories(${PCL_INCLUDE_DIRS})######
link_directories(${PCL_LIBRARY_DIRS})#####
add_definitions(${PCL_DEFINITIONS})#####


if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++17)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
        # it is asumed, you are in "build" order
        ../src/main.cpp
        ../src/pcl_main_operations.hpp
        ../src/additional_functions.hpp
)

add_executable (get_image_photoneo
    ${Files}
)
target_link_libraries (get_image_photoneo ${PCL_LIBRARIES} ${ZMQLIB} ${OpenCV_LIBS}) ##########


if (UNIX AND NOT APPLE)
    target_link_libraries(get_image_photoneo
        ${PHOXI_LIBRARY}
        rt
    )
else (UNIX AND NOT APPLE)
    target_link_libraries(get_image_photoneo
        ${PHOXI_LIBRARY}
    )
endif(UNIX AND NOT APPLE)


target_include_directories (get_image_photoneo PUBLIC ${PHOXI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}     ${OpenCV_INCLUDE_DIRS} )


