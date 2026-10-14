cmake_minimum_required(VERSION 3.25)
project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

add_library(physics STATIC
        engine/rigid/Shape.cpp
        engine/rigid/Body.cpp
        engine/rigid/Collision.cpp
        engine/rigid/Joint.cpp
        engine/soft/ParticleSystem.cpp
        engine/soft/Builders.cpp
        engine/field/QuadTree.cpp
        engine/field/Fields.cpp
        engine/core/World.cpp
)
target_include_directories(physics PUBLIC engine)
target_compile_options(physics PRIVATE -Wall -Wextra)

find_package(SDL3 REQUIRED CONFIG)

add_executable(PhysicsEngine
        app/main.cpp
        app/App.cpp
        app/Ui.cpp
        app/Renderer.cpp
        app/Scenes.cpp
)
target_link_libraries(PhysicsEngine PRIVATE physics SDL3::SDL3)
target_compile_options(PhysicsEngine PRIVATE -Wall -Wextra)
