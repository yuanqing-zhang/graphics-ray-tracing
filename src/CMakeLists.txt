project(main)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/utils UTILS_SRC)
add_library(utils SHARED ${UTILS_SRC})
target_link_libraries(utils ${OpenCV_LIBS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/io)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/io IO_SRC)
add_library(io SHARED ${IO_SRC})
target_link_libraries(io ${OpenCV_LIBS} utils)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/raytracer)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/raytracer RAYTRACER_SRC)
add_library(raytracer SHARED ${RAYTRACER_SRC})
target_link_libraries(raytracer ${OpenCV_LIBS} utils)

add_executable(ray-tracing main.cpp)
target_link_libraries(ray-tracing OpenMP::OpenMP_CXX utils io raytracer)
