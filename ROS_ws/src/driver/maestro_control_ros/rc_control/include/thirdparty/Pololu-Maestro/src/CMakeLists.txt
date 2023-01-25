cmake_minimum_required(VERSION 3.11.4)

project(Maestro)

add_library(maestro STATIC
            maestro/Device.h
            maestro/Device.cpp
            maestro/Instruction.cpp
            maestro/Instruction.h
            maestro/Program.cpp
            maestro/Program.h
            maestro/Opcode.h
            )
target_link_libraries(maestro PRIVATE usb-1.0)
set_target_properties(maestro PROPERTIES CXX_STANDARD 11)
set_target_properties(maestro PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(maestro PROPERTIES PUBLIC_HEADER "maestro/Device.h;maestro/Program.h")
set_target_properties(maestro PROPERTIES FOLDER "Maestro")
target_include_directories(maestro PUBLIC .)

install(TARGETS maestro
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)

