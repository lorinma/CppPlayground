cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall")
project(collision)

#for rapidjson
find_package(RapidJSON REQUIRED)
include_directories(${RAPIDJSON_INCLUDE_DIRS})
include_directories(.)
find_package(Threads REQUIRED)
find_package(Boost 1.54.0 COMPONENTS regex system thread coroutine context filesystem date_time REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#for bullet3, it is relative to rapidjson path
INCLUDE_DIRECTORIES(${RAPIDJSON_INCLUDE_DIRS}/bullet)
link_directories(${RAPIDJSON_INCLUDE_DIRS}/../lib)
LINK_LIBRARIES(BulletDynamics BulletCollision LinearMath)
message(${RAPIDJSON_INCLUDE_DIRS})

add_executable(collision main.cpp)
target_link_libraries(collision ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})