cmake_minimum_required(VERSION 3.10)

include(GenerateExportHeader)

add_library(tracking SHARED tracking.cpp)
target_link_libraries(tracking PUBLIC rbgt)

GENERATE_EXPORT_HEADER(tracking)
target_include_directories(tracking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tracking PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

add_executable(tracking_demo test.cpp)
target_link_libraries(tracking_demo tracking)