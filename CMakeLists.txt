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
        engine/fluid/FluidSystem.cpp
        engine/field/QuadTree.cpp
        engine/field/Fields.cpp
        engine/core/World.cpp
        engine/core/Parallel.cpp
)
target_include_directories(physics PUBLIC engine)
find_package(Threads REQUIRED)
target_link_libraries(physics PUBLIC Threads::Threads)
target_compile_options(physics PRIVATE -Wall -Wextra)

find_package(SDL3 CONFIG QUIET)
if (NOT SDL3_FOUND)
    message(STATUS "SDL3 not found; fetching and building it from source")
    include(FetchContent)
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(SDL3
            GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
            GIT_TAG release-3.2.24
            GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(SDL3)
endif ()

add_executable(PhysicsEngine
        app/main.cpp
        app/App.cpp
        app/Ui.cpp
        app/Renderer.cpp
        app/Scenes.cpp
        app/scenes/Machines.cpp
        app/scenes/Oscillations.cpp
        app/scenes/Collisions.cpp
        app/scenes/Projectiles.cpp
        app/scenes/Rotation.cpp
        app/scenes/Thermo.cpp
        app/scenes/Optics.cpp
)
target_include_directories(PhysicsEngine PRIVATE app)
target_link_libraries(PhysicsEngine PRIVATE physics SDL3::SDL3)
target_compile_options(PhysicsEngine PRIVATE -Wall -Wextra)
