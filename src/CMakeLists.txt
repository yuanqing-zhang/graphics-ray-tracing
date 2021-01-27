project(main)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/utils UTILS_SRC)
add_library(utils SHARED ${UTILS_SRC})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/io)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/io IO_SRC)
add_library(io SHARED ${IO_SRC})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/raytracer)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/raytracer RAYTRACER_SRC)
add_library(raytracer SHARED ${RAYTRACER_SRC})

add_executable(ray-tracing main.cpp)
target_link_libraries(ray-tracing utils io raytracer)
