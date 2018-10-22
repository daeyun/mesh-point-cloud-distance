set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_library(common common.cc)
target_link_libraries(common PUBLIC cpp_gsl eigen3 spdlog)

add_library(mesh mesh.h)
target_link_libraries(mesh PUBLIC common)

add_library(benchmark benchmark.h benchmark.cc)
target_link_libraries(benchmark PUBLIC common)

add_library(meshdist meshdist.h meshdist.cc)
target_link_libraries(meshdist PUBLIC common benchmark mesh CGAL)
target_compile_options(meshdist BEFORE PUBLIC "-lCGAL")

add_library(string_utils string_utils.h string_utils.cc)
target_link_libraries(string_utils PUBLIC common)

add_library(file_io file_io.cc file_io.h)
target_link_libraries(file_io spdlog assimp common cpp_gsl Boost::filesystem tinyply string_utils)
