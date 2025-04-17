cmake_minimum_required(VERSION 3.10)
project(hokuyoZoneServer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)

# nlohmann_json can be installed via package manager or added as submodule
find_package(nlohmann_json REQUIRED)

if(WIN32)
    add_definitions(-D_WIN32_WINNT=0x0601)
    set(PLATFORM_LIBS ws2_32)
else()
    find_package(X11 REQUIRED)
    set(PLATFORM_LIBS ${X11_LIBRARIES})
endif()

add_executable(hokuyoZoneServer hokuyoZoneServer.cpp)

target_include_directories(hokuyoZoneServer PRIVATE ${Boost_INCLUDE_DIRS} ${X11_INCLUDE_DIR})
target_link_libraries(hokuyoZoneServer PRIVATE ${Boost_LIBRARIES} Threads::Threads nlohmann_json::nlohmann_json ${PLATFORM_LIBS})
