cmake_minimum_required(VERSION 3.14.0)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.6.0
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(pymaestro maestro.cpp example.py)
target_link_libraries(pymaestro PRIVATE maestro)
set_target_properties(pymaestro PROPERTIES FOLDER "python")
set_target_properties(pymaestro PROPERTIES OUTPUT_NAME "maestro")

install(TARGETS pymaestro LIBRARY DESTINATION python)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/example.py DESTINATION python)
