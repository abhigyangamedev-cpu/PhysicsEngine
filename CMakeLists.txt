cmake_minimum_required(VERSION 3.25)
project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

option(PE_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if (PE_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address,undefined)
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

add_library(physics3d STATIC
        engine3d/rigid/Body3.cpp
        engine3d/rigid/Collision3.cpp
        engine3d/rigid/Joint3.cpp
        engine3d/core/World3.cpp
)
target_include_directories(physics3d PUBLIC engine3d)
target_compile_options(physics3d PRIVATE -Wall -Wextra)

# Turns a physics question into a checked simulation with Claude; shared by the app and its tests.
set(AI_SOURCES
        ai/Json.cpp
        ai/QuestionSpec.cpp
        ai/Prompt.cpp
        ai/Generator.cpp
        ai/Http.cpp
)
set(AI_LIBS)
if (WIN32)
    set(AI_LIBS winhttp)
endif ()

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
        app/WorldDraw.cpp
        app/Expr.cpp
        app/editor/EditorScene.cpp
        app/editor/Editor.cpp
        app/editor/EditorPanels.cpp
        app/editor/Simulation.cpp
        app/editor/Examples.cpp
        app/Scenes.cpp
        app/scenes/Machines.cpp
        app/scenes/Oscillations.cpp
        app/scenes/Collisions.cpp
        app/scenes/Projectiles.cpp
        app/scenes/Rotation.cpp
        app/scenes/Thermo.cpp
        app/scenes/Optics.cpp
        app3d/Viewer3.cpp
        app3d/Overlay3.cpp
        app3d/Gl.cpp
        app3d/Renderer3.cpp
        app3d/Scenes3.cpp
        app3d/EditorScene3.cpp
        app3d/Problems3.cpp
        app3d/Editor3.cpp
        app/Workspace.cpp
        ${AI_SOURCES}
)
target_include_directories(PhysicsEngine PRIVATE app app3d ai)
target_link_libraries(PhysicsEngine PRIVATE physics physics3d SDL3::SDL3 ${AI_LIBS})
target_compile_options(PhysicsEngine PRIVATE -Wall -Wextra)

add_executable(PhysicsEngine3D
        app3d/main3d.cpp
        app3d/Viewer3.cpp
        app3d/Overlay3.cpp
        app3d/Gl.cpp
        app3d/Renderer3.cpp
        app3d/Scenes3.cpp
        app3d/EditorScene3.cpp
        app3d/Problems3.cpp
        app3d/Editor3.cpp
        app/Ui.cpp
        app/Renderer.cpp
        app/Expr.cpp
)
# the overlay reuses the 2D app's Ui widgets, which need the 2D math header
target_include_directories(PhysicsEngine3D PRIVATE app3d app engine)
target_link_libraries(PhysicsEngine3D PRIVATE physics3d SDL3::SDL3)
target_compile_options(PhysicsEngine3D PRIVATE -Wall -Wextra)

# Headless tests for the editor's scene model, simulation and examples (no SDL needed).
add_executable(editor_tests
        tests/main.cpp
        tests/ExprTests.cpp
        tests/SceneTests.cpp
        tests/MechanicsTests.cpp
        tests/OscillationTests.cpp
        tests/CollisionFieldTests.cpp
        tests/ExampleTests.cpp
        tests/StressTests.cpp
        tests/EdgeCaseTests.cpp
        app/Expr.cpp
        app/editor/EditorScene.cpp
        app/editor/Simulation.cpp
        app/editor/Examples.cpp
        app/Scenes.cpp
        app/scenes/Machines.cpp
        app/scenes/Oscillations.cpp
        app/scenes/Collisions.cpp
        app/scenes/Projectiles.cpp
        app/scenes/Rotation.cpp
        app/scenes/Thermo.cpp
        app/scenes/Optics.cpp
)
target_include_directories(editor_tests PRIVATE app tests)
target_link_libraries(editor_tests PRIVATE physics)
target_compile_options(editor_tests PRIVATE -Wall -Wextra)

enable_testing()
add_test(NAME editor_tests COMMAND editor_tests)

add_executable(physics3d_tests
        tests/main.cpp
        tests3d/PhysicsTests3.cpp
        tests3d/JointTests3.cpp
        tests3d/ProblemTests3.cpp
        tests3d/EdgeCaseTests3.cpp
        app3d/Scenes3.cpp
        app3d/EditorScene3.cpp
        app3d/Problems3.cpp
)
target_include_directories(physics3d_tests PRIVATE app3d tests)
target_link_libraries(physics3d_tests PRIVATE physics3d)
target_compile_options(physics3d_tests PRIVATE -Wall -Wextra)
add_test(NAME physics3d_tests COMMAND physics3d_tests)

# Headless tests for the question workspace: JSON, scene reading, measuring, and the repair loop
# against a scripted API (no network needed).
add_executable(ai_tests
        tests/main.cpp
        tests_ai/JsonTests.cpp
        tests_ai/SpecTests.cpp
        tests_ai/GeneratorTests.cpp
        tests_ai/EdgeCaseTests.cpp
        ${AI_SOURCES}
        app/editor/EditorScene.cpp
        app/editor/Simulation.cpp
        app3d/EditorScene3.cpp
)
target_include_directories(ai_tests PRIVATE app app3d ai tests)
target_link_libraries(ai_tests PRIVATE physics physics3d ${AI_LIBS})
target_compile_options(ai_tests PRIVATE -Wall -Wextra)
add_test(NAME ai_tests COMMAND ai_tests)

# Drives the real app windows (needs a display): every screen, problem and 3D mode opens, runs and exits.
add_executable(ui_tests
        tests/main.cpp
        tests/UiTests.cpp
        app/App.cpp
        app/Ui.cpp
        app/Renderer.cpp
        app/WorldDraw.cpp
        app/Expr.cpp
        app/editor/EditorScene.cpp
        app/editor/Editor.cpp
        app/editor/EditorPanels.cpp
        app/editor/Simulation.cpp
        app/editor/Examples.cpp
        app/Scenes.cpp
        app/scenes/Machines.cpp
        app/scenes/Oscillations.cpp
        app/scenes/Collisions.cpp
        app/scenes/Projectiles.cpp
        app/scenes/Rotation.cpp
        app/scenes/Thermo.cpp
        app/scenes/Optics.cpp
        app3d/Viewer3.cpp
        app3d/Overlay3.cpp
        app3d/Gl.cpp
        app3d/Renderer3.cpp
        app3d/Scenes3.cpp
        app3d/EditorScene3.cpp
        app3d/Problems3.cpp
        app3d/Editor3.cpp
        app/Workspace.cpp
        ${AI_SOURCES}
)
target_include_directories(ui_tests PRIVATE app app3d ai tests tests_ai)
target_link_libraries(ui_tests PRIVATE physics physics3d SDL3::SDL3 ${AI_LIBS})
target_compile_options(ui_tests PRIVATE -Wall -Wextra)
add_test(NAME ui_tests COMMAND ui_tests)
