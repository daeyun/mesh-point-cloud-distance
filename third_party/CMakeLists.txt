#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/third_party/cmake/Modules/")
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(BUILD_SHARED_LIBS OFF)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)


add_library(eigen3 INTERFACE)
target_include_directories(eigen3 BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/eigen
        )

add_library(cpp_gsl INTERFACE)
target_include_directories(cpp_gsl BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/cpp_gsl/include
        )

add_library(spdlog INTERFACE)
target_include_directories(spdlog BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/spdlog/include
        )
target_link_libraries(spdlog INTERFACE Threads::Threads)

add_library(cxxopts INTERFACE)
target_include_directories(cxxopts BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/cxxopts/include
        )

add_library(catch2 INTERFACE)
target_include_directories(catch2 BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/Catch2/single_include
        )

add_library(tinyply INTERFACE)
target_include_directories(tinyply BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/repos/tinyply/source
        )

add_library(assimp INTERFACE)
target_link_libraries(assimp INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/install/assimp/lib/libassimp.a
        ${CMAKE_SOURCE_DIR}/third_party/install/assimp/lib/libIrrXML.a
        z
        pthread
        )
target_include_directories(assimp BEFORE INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/install/assimp/include
        )
