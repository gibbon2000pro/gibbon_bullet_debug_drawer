cmake_minimum_required(VERSION 3.20)
project(gibbon_bullet_debug_drawer)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

find_package(spdlog REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
    "raylib/include"
    ${BULLET_INCLUDE_DIR}
)

link_directories(
    "raylib/"
)

######################################
# raylib test
######################################
add_executable(models_box_collisions
    models_box_collisions.cpp
)
target_link_libraries(models_box_collisions
    libraylib.a
)


######################################
# example
######################################
add_executable(one_ball_world
    one_ball_world.cpp
)
target_link_libraries(one_ball_world PUBLIC
    libraylib.a
    spdlog::spdlog
    ${BULLET_LIBRARIES}
)


add_executable(test_compound
    test_compound_shape.cpp
)
target_link_libraries(test_compound PUBLIC
    libraylib.a
    spdlog::spdlog
    ${BULLET_LIBRARIES}
)
target_link_options(test_compound
    BEFORE PUBLIC -fsanitize=undefined PUBLIC -fsanitize=address
)