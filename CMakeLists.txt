cmake_minimum_required(VERSION 3.21)
project(viterbi)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(nlohmann_json)

add_executable(viterbi generate_trajectory.cpp utils.cpp thread_pool.cpp)
target_link_libraries(viterbi PRIVATE nlohmann_json::nlohmann_json)

set_target_properties(viterbi PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        COMPILE_OPTIONS "-O3;-Wpedantic;-Wall;-Wextra"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
