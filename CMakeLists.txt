cmake_minimum_required(VERSION 3.23)
project(YouBot)

set(CMAKE_CXX_STANDARD 14)

include_directories(. Eigen MRLib remoteApi remoteApi/include)

link_libraries(ws2_32 winmm)

add_library(MRLib SHARED ${CMAKE_CURRENT_SOURCE_DIR}/MRLib/modern_robotics.cpp ${CMAKE_CURRENT_SOURCE_DIR}/MRLib/modern_robotics.h)

add_executable(YouBot
        main.cpp
        remoteApi/extApi.c
        remoteApi/extApiPlatform.c
        YouBotFuncs.cpp YouBotFuncs.h)

target_link_libraries(YouBot MRLib)

target_compile_definitions(YouBot PRIVATE
#        WIN32
#        NDEBUG
#        _CONSOLE
        NON_MATLAB_PARSING
        MAX_EXT_API_CONNECTIONS=255
        DO_NOT_USE_SHARED_MEMORY
        )

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O3")