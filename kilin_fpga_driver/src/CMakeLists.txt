find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

# to generate compile_commands.json for intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 


add_executable(fpga_driver  fpga_server.cpp
                            console.cpp    
                            color.cpp
                            fpga_handler.cpp
                            fsm.cpp
                            hip_module.cpp
                            hip_motor.cpp
                            limb_module.cpp
                            NiFpga_FPGA_RS485_v1_2.c
                            NiFpga.c                       
)

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(yaml-cpp REQUIRED)
target_link_libraries(fpga_driver yaml-cpp::yaml-cpp ${CURSES_LIBRARY} -lncurses ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB})

find_package(Eigen3 REQUIRED)
target_link_libraries(fpga_driver Eigen3::Eigen ${CURSES_LIBRARY} -lncurses ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB})

