cmake_minimum_required(VERSION 3.5)

project(jellyblend CXX)
set(CMAKE_CXX_STANDARD 20)

set(src_core python_helper.cpp fixedbody.cpp softbody.cpp collision.cpp collision_constrain.cpp softbody_constrain.cpp physics_world.cpp)
list(TRANSFORM src_core PREPEND "src/engine/")
set(src_lib ${src_core} src/engine/api.cpp src/engine/mesh_utils.cpp)

find_package(Python 3.10 EXACT REQUIRED COMPONENTS Interpreter Development)

add_subdirectory(extern/pybind11)
add_subdirectory(extern/tetgen)
add_subdirectory(extern/eigen)

# python lib
pybind11_add_module(jellyblend_engine ${src_lib})

target_include_directories(jellyblend_engine PRIVATE extern/tetgen)
target_include_directories(jellyblend_engine PRIVATE extern/cereal/include)
target_include_directories(jellyblend_engine PRIVATE extern/spdlog/include)
target_include_directories(jellyblend_engine PRIVATE extern/thread-pool)

target_link_libraries(jellyblend_engine PRIVATE tetgen)
target_link_libraries(jellyblend_engine PRIVATE Eigen3::Eigen)

target_compile_options(jellyblend_engine PRIVATE -Wall)

# for mingw, link the dll
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_options(jellyblend_engine PRIVATE -static-libgcc -static-libstdc++ -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive)
endif(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

# pack c++ library with python source file
# create addon directory
set(addon_directory_name jelly_blend_addon)
add_custom_command(TARGET jellyblend_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/${addon_directory_name}
)

# copy c++ library
add_custom_command(TARGET jellyblend_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:jellyblend_engine> ${PROJECT_BINARY_DIR}/${addon_directory_name}/.
)

# copy python files
file(GLOB python_source
    "src/addon/*.py"
)
add_custom_command(TARGET jellyblend_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${python_source} ${PROJECT_BINARY_DIR}/${addon_directory_name}/.
)
