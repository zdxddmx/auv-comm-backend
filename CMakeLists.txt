cmake_minimum_required(VERSION 3.14)
project(auv_comm_backend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(Threads REQUIRED)

set(CORE_SOURCES
    src/frame.cpp
    src/channel.cpp
    src/router.cpp)

add_library(auvcore STATIC ${CORE_SOURCES})
target_include_directories(auvcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(auvcore PUBLIC Threads::Threads)

add_executable(auv_sim src/main.cpp)
target_link_libraries(auv_sim PRIVATE auvcore)

add_executable(auv_test test/test_auv.cpp)
target_link_libraries(auv_test PRIVATE auvcore)

enable_testing()
add_test(NAME unit COMMAND auv_test)

install(TARGETS auv_sim RUNTIME DESTINATION bin)
