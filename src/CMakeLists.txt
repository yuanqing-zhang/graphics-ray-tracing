project(main)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/io)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/io IO_SRC)
add_library(io SHARED ${IO_SRC}) # 将源文件打包成动态库

add_executable(ray-tracing main.cpp)
target_link_libraries(ray-tracing io)
