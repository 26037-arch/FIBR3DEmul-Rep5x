cmake_minimum_required(VERSION 3.16)
project(simExtFIBR3D LANGUAGES C CXX)

# Upstream references programming/libPlugin files and callback inputs that are
# not in its checkout. FIBR3DEmul already vendors the 4.0 regular API binding,
# so this compatibility build retains the original simStart/simMessage ABI.
option(FIBR3D_USE_BUNDLED_COPPELIA_API "Build against the vendored CoppeliaSim 4.0 API" ON)
if(NOT FIBR3D_USE_BUNDLED_COPPELIA_API)
    message(FATAL_ERROR "Only the audited bundled legacy API build is implemented in phase 1")
endif()

find_package(Boost 1.64 REQUIRED COMPONENTS system thread serialization date_time)
find_package(Threads REQUIRED)

file(GLOB FIBR3D_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB COPPELIA_SOURCES CONFIGURE_DEPENDS coppelia/common/*.cpp coppelia/common/*.c coppelia/common/stack/*.cpp)

add_library(simExtFIBR3D SHARED
    simExtFIBR3D.cpp
    ${FIBR3D_SOURCES}
    ${COPPELIA_SOURCES}
)
target_compile_features(simExtFIBR3D PRIVATE cxx_std_14)
target_include_directories(simExtFIBR3D PRIVATE
    include
    spdlog
    coppelia/include
    coppelia/include/stack
)
target_link_libraries(simExtFIBR3D PRIVATE
    Boost::system
    Boost::thread
    Boost::serialization
    Boost::date_time
    Threads::Threads
)

if(WIN32)
    target_compile_definitions(simExtFIBR3D PRIVATE _WIN32_WINNT=0x0601)
    target_link_libraries(simExtFIBR3D PRIVATE ws2_32 shlwapi)
endif()

set_target_properties(simExtFIBR3D PROPERTIES PREFIX "" OUTPUT_NAME "simExtFIBR3D")

if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
    install(TARGETS simExtFIBR3D RUNTIME DESTINATION "$ENV{COPPELIASIM_ROOT_DIR}")
else()
    install(TARGETS simExtFIBR3D RUNTIME DESTINATION bin)
endif()
